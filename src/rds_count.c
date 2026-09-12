/*
 * rds_count.c
 *
 *  Created on: 20.02.2022
 *      Author: pvvx
 */

#include "tl_common.h"
#if (DEV_SERVICES & SERVICE_RDS)
#include "stack/ble/ble.h"
#include "app.h"
#include "ble.h"
#include "drivers.h"
#include "sensor.h"
#include "trigger.h"
#include "ble.h"
#include "lcd.h"
#include "custom_beacon.h"
#if USE_MIHOME_BEACON
#include "mi_beacon.h"
#endif
#if USE_BTHOME_BEACON
#include "bthome_beacon.h"
#endif
#include "rds_count.h"
#include "battery.h"
#if (USE_SENSOR_HX71X)
#include "hx71x.h"
#endif

#if (DEV_SERVICES & SERVICE_LE_LR)
extern u32 blt_advExpectTime;
extern u8 app_adv_set_param[];
#endif

RAM	rds_count_t rds;		// Reed switch pulse counter
#ifdef GPIO_IR
RAM u8  rds1_beam_state;		// dead-band state for the IR optical read (get_rds1_input)
RAM u32 rds1_last_edge_tick;	// clock_time() of the last counted pulse edge

// Adaptive IR sample wake: fast while flowing, slow when idle, so counting doesn't rely
// on the ~6 s advertise wake. All periods MUST stay above the 40 ms adv deep-sleep-
// retention threshold (blc_pm_setDeepsleepRetentionThreshold) or the SoC drops from
// retention (uA) to suspend (mA) between wakes - why 25 ms drained and ~60 ms doesn't.
// Tiers: flowing -> sample_active_ms, idle -> sample_idle_ms, silence -> sample_deepidle_ms.
#ifndef IR_MIN_PERIOD_MS
#define IR_MIN_PERIOD_MS		50		// floor: stay above the 40 ms retention threshold
#endif
#ifndef IR_FLOW_HOLD_MS
#define IR_FLOW_HOLD_MS			2000	// stay at the active rate this long after an edge
#endif
#ifndef IR_DEEPIDLE_ENTER_MS
#define IR_DEEPIDLE_ENTER_MS	120000	// drop to the deep-idle rate after this much silence
#endif

// Wake callback: the actual IR sample runs in main_loop -> rds_task after the wake,
// so this only needs to exist for the low-power timer wake to fire.
_attribute_ram_code_
static void irwm_ir_wakeup_cb(int par) { (void)par; }

_attribute_ram_code_
static void irwm_schedule_next_sample(void) {
	u32 now = clock_time();
	// Wrap-safe elapsed-since-edge (signed diff is valid for intervals < ~134 s).
	s32 since_edge = (s32)(now - rds1_last_edge_tick);
	u16 period_ms;
	if (since_edge >= 0
			&& (u32)since_edge < (u32)IR_FLOW_HOLD_MS * CLOCK_16M_SYS_TIMER_CLK_1MS) {
		period_ms = trg.sample_active_ms;
	} else if (since_edge >= 0
			&& (u32)since_edge < (u32)IR_DEEPIDLE_ENTER_MS * CLOCK_16M_SYS_TIMER_CLK_1MS) {
		period_ms = trg.sample_idle_ms;
	} else {
		period_ms = trg.sample_deepidle_ms;
		// Re-anchor so the tick diff stays bounded across the ~268 s clock_time wrap.
		rds1_last_edge_tick = now - (u32)IR_DEEPIDLE_ENTER_MS * CLOCK_16M_SYS_TIMER_CLK_1MS;
	}
	if (period_ms < IR_MIN_PERIOD_MS)
		period_ms = IR_MIN_PERIOD_MS;	// keep every wake above the retention threshold
	bls_pm_registerAppWakeupLowPowerCb(irwm_ir_wakeup_cb);
	bls_pm_setAppWakeupLowPower(now + (u32)period_ms * CLOCK_16M_SYS_TIMER_CLK_1MS, 1);
}

// IR receiver read: strobe the emitter, sample the receiver level via ADC, and classify
// it with an ADAPTIVE hysteresis threshold. The envelope (hi/lo) tracks the clear/blocked
// levels with fast attack + slow decay, so the thresholds ride battery/LED drift; a steady
// partial-block "leak" sits in the dead-band and can't toggle. If the swing collapses
// (parked / no real flow) we stop deciding, which stops phantom counts.
#ifndef IR_ADC_MIN_RANGE_MV
#define IR_ADC_MIN_RANGE_MV	150		// need >= this clear-vs-blocked swing (mV) to decide
#endif
#ifndef IR_ADC_DECAY_SHL
#define IR_ADC_DECAY_SHL	6		// envelope decay per sample: (gap >> 6) toward the level
#endif
RAM u16 rds1_adc_raw;				// last raw receiver level (mV) - beacon debug
RAM u16 rds1_adc_hi;				// tracked high envelope (mV)
RAM u16 rds1_adc_lo;				// tracked low envelope (mV)
RAM u8  rds1_adc_seeded;

u8 get_rds1_input(void) {
	gpio_set_output_en(GPIO_IR, 1);
	gpio_set_input_en(GPIO_IR, 0);
	gpio_write(GPIO_IR, 1);				// strobe IR emitter on
	sleep_us(IR_SETTLE_US);
	u16 mv = get_adc_mv(SHL_ADC_RDS1);	// receiver level, emitter lit
	gpio_write(GPIO_IR, 0);				// emitter off
	gpio_set_output_en(GPIO_IR, 0);
	gpio_set_input_en(GPIO_IR, 1);
	rds1_adc_raw = mv;

	if (!rds1_adc_seeded) {				// first sample: seed envelope, decide nothing yet
		rds1_adc_hi = rds1_adc_lo = mv;
		rds1_adc_seeded = 1;
	}
	// fast attack to new extremes, slow decay toward the current level
	if (mv > rds1_adc_hi) rds1_adc_hi = mv;
	else rds1_adc_hi -= (rds1_adc_hi - mv) >> IR_ADC_DECAY_SHL;
	if (mv < rds1_adc_lo) rds1_adc_lo = mv;
	else rds1_adc_lo += (mv - rds1_adc_lo) >> IR_ADC_DECAY_SHL;

	// decide only on a real swing; hysteresis at 1/3 and 2/3 of the tracked range
	u16 range = rds1_adc_hi - rds1_adc_lo;
	if (range >= IR_ADC_MIN_RANGE_MV) {
		u16 hi_thr = rds1_adc_lo + (u16)(((u32)range * 2) / 3);
		u16 lo_thr = rds1_adc_lo + (u16)((u32)range / 3);
		if (mv >= hi_thr) rds1_beam_state = 1;
		else if (mv <= lo_thr) rds1_beam_state = 0;
		// between thresholds: hold (dead-band)
	}
	// range too small (parked / no flow): hold state -> no toggle -> no phantom counts
	u8 r = rds1_beam_state;
	if (trg.rds.rs1_invert)
		r ^= 1;
	return r;
}
#endif

_attribute_ram_code_
void rds_input_off(void) {
	if (trg.rds.type1 == RDS_NONE) {
		trg.flg.rds1_input = get_rds1_input();
		rds1_input_off();
	}
#ifdef GPIO_RDS2
	if (trg.rds.type2 == RDS_NONE) {
		trg.flg.rds2_input = get_rds2_input();
		rds2_input_off();
	}
#endif
}


void rds_init(void) {
	if (trg.rds.type1) {
#if (DEV_SERVICES & SERVICE_KEY)  // defined GPIO_KEY2
		if(trg.rds.type1 == RDS_CONNECT) {
			trg.rds.type1 = RDS_SWITCH;
		}
#else
		if(trg.rds.type1 == RDS_CONNECT)
			trg.rds.rs1_invert = 1;
#endif
		rds1_input_on();
	} else
		cpu_set_gpio_wakeup(GPIO_RDS1, Level_Low, 0);  // pad wakeup deepsleep disable
#ifdef GPIO_RDS2
	if (trg.rds.type2) {
#if (DEV_SERVICES & SERVICE_KEY)  // defined GPIO_KEY2
		if(trg.rds.type2 == RDS_CONNECT) {
			trg.rds.type2 = RDS_SWITCH;
		}
#else
		if(trg.rds.type2 == RDS_CONNECT)
			trg.rds.rs2_invert = 1;
#endif
		rds2_input_on();
	} else
		cpu_set_gpio_wakeup(GPIO_RDS2, Level_Low, 0);  // pad wakeup deepsleep disable
#endif
#if USE_WK_RDS_COUNTER32 // save 32 bits?
	if ((flash_read_cfg(&rds.count_short[1], EEP_ID_RPC, sizeof(rds.count_short[1])) != sizeof(rds.count_short[1])) {
		rds.count = 0;
	}
#endif
	rds.report_tick = wrk.utc_time_sec;
#ifdef GPIO_IR
	// Start in the idle tier (not "flowing"): backdate the edge past IR_FLOW_HOLD_MS so
	// boot arms the idle-rate sample wake, ready to catch flow onset without paying the
	// active rate up front.
	rds1_last_edge_tick = clock_time() - (u32)(IR_FLOW_HOLD_MS + 100) * CLOCK_16M_SYS_TIMER_CLK_1MS;
	irwm_schedule_next_sample();
#endif
}

//_attribute_ram_code_
__attribute__((optimize("-Os")))
static void set_rds_adv_data(void) {
	adv_buf.send_count++;
#if (USE_CUSTOM_BEACON + USE_BTHOME_BEACON + USE_MIHOME_BEACON + USE_ATC_BEACON) > 1
	int advertising_type = cfg.flg.advertising_type;
#endif
#if (DEV_SERVICES & SERVICE_BINDKEY)
	if (cfg.flg2.adv_crypto) {
#if (USE_CUSTOM_BEACON + USE_BTHOME_BEACON + USE_MIHOME_BEACON + USE_ATC_BEACON) > 1
#if USE_CUSTOM_BEACON
		if (advertising_type == ADV_TYPE_PVVX)
			pvvx_encrypt_event_beacon(rds.event);
		else
#endif
#if USE_BTHOME_BEACON
		if (advertising_type == ADV_TYPE_BTHOME)
			bthome_encrypt_event_beacon(rds.event);
		else
#endif
#if USE_MIHOME_BEACON
		if (advertising_type == ADV_TYPE_MI)
			mi_encrypt_event_beacon(rds.event);
		else
#endif
#if USE_ATC_BEACON
		if (advertising_type == ADV_TYPE_ATC)
			default_event_beacon();
		else
#endif
		{}
#else
		bthome_encrypt_event_beacon(rds.event);
#endif // (USE_CUSTOM_BEACON + USE_BTHOME_BEACON + USE_MIHOME_BEACON + USE_ATC_BEACON) > 1
	} else
#endif //	#if (DEV_SERVICES & SERVICE_BINDKEY)
	{
#if (USE_CUSTOM_BEACON + USE_BTHOME_BEACON + USE_MIHOME_BEACON + USE_ATC_BEACON) > 1
#if USE_CUSTOM_BEACON
		if (advertising_type == ADV_TYPE_PVVX)
			pvvx_event_beacon(rds.event);
		else
#endif
#if USE_MIHOME_BEACON
		if (advertising_type == ADV_TYPE_MI)
			mi_event_beacon(rds.event);
		else
#endif
#if USE_BTHOME_BEACON
		if (advertising_type == ADV_TYPE_BTHOME)
			bthome_event_beacon(rds.event);
		else
#endif
#if USE_ATC_BEACON
		if (advertising_type == ADV_TYPE_ATC)
			default_event_beacon();
		else
#endif
		{}
#else
		bthome_event_beacon(rds.event);
#endif // (USE_CUSTOM_BEACON + USE_BTHOME_BEACON + USE_MIHOME_BEACON + USE_ATC_BEACON) > 1
	}
	adv_buf.update_count = 0; // refresh adv_buf.data in next set_adv_data()
	load_adv_data();
}

//_attribute_ram_code_
static void start_ext_adv(void) {
#if (DEV_SERVICES & SERVICE_LE_LR)
	if (adv_buf.ext_adv_init != EXT_ADV_Off) { // support extension advertise
		set_rds_adv_data();
		blta.adv_duraton_en = cfg.event_adv_cnt;
		adv_buf.data_size = 0; // flag adv_buf.send_count++ over adv.event
		ll_ext_adv_t *p = (ll_ext_adv_t *)&app_adv_set_param;
		// patch: set time next ext.adv = 0
		blt_advExpectTime = clock_time() + 250*CLOCK_16M_SYS_TIMER_CLK_1US; // set time next ext.adv
		p->adv_event_tick = blt_advExpectTime;
		p->advInt_use = EXT_ADV_INTERVAL; // new adv. interval
	} else
#endif
	{
		bls_ll_setAdvEnable(BLC_ADV_DISABLE);  // adv disable
		bls_ll_setAdvParam(EXT_ADV_INTERVAL, EXT_ADV_INTERVAL,
				ADV_TYPE_NONCONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, 0, NULL,
				BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
		set_rds_adv_data();
		adv_buf.data_size = 0; // flag adv_buf.send_count++ over adv.event
		bls_ll_setAdvDuration(EXT_ADV_INTERVAL*625*cfg.event_adv_cnt+33, 1);
		blta.adv_interval = 0; // system tick
		bls_ll_setAdvEnable(BLC_ADV_ENABLE);  // adv enable
	}
}

/* if (trg.rds.type1) //  switch or counter [or connect] */
_attribute_ram_code_
__attribute__((optimize("-Os")))
void rds_task(void) {
#ifdef GPIO_RDS2
	if(trg.rds.type2 != RDS_NONE) { // rds: switch or counter
		if (get_rds2_input()) {
			if (!trg.flg.rds2_input) {
				// RDS2 on event
				trg.flg.rds2_input = 1;
				if (trg.rds.type2 == RDS_SWITCH) { // switch mode, send event if on
					rds.event = trg.rds.type2;
				}
			}
		} else {
			if (trg.flg.rds2_input) {
				// RDS2 off event, empty tank
				trg.flg.rds2_input = 0;
				if(!trg.flg.rds2_input) {
/*
					if (trg.rds.type2 == RDS_COUNTER) { // counter mode
						if ((rds.count2 & 0xffff) == 0) { // report 'overflow 16 bit count'
							rds.event = trg.rds.type2;
						}
					} else
*/
						if (trg.rds.type2 == RDS_SWITCH) { // switch mode
						rds.event = trg.rds.type2;
					}
				}
			}
		}
	}
	if(trg.rds.type1 != RDS_NONE) { // rds: switch or counter
#endif // GPIO_RDS2
		if (get_rds1_input()) { // key on
			if (!trg.flg.rds1_input) {
				// RDS1 on event, keypress event
				trg.flg.rds1_input = 1;
#if (USE_SENSOR_HX71X) && (DEVICE_TYPE == DEVICE_TNK01)
				hx71x_calibration();
#endif
				if (trg.rds.type1 == RDS_SWITCH) { // switch mode
					rds.event = trg.rds.type1;
				}
#if !(DEV_SERVICES & SERVICE_KEY)
				else if (trg.rds.type1 == RDS_CONNECT) { // connect mode
					// connect keypress event
					u32 new = clock_time();
					ext_key.key_pressed_tik1 = new;
					ext_key.key_pressed_tik2 = new;
					set_adv_con_time(0); // set connection adv.
					SET_LCD_UPDATE();
				}
#endif
			}
#if !(DEV_SERVICES & SERVICE_KEY)
			else if (trg.rds.type1 == RDS_CONNECT) { // connect mode
				// connection key held
				u32 new = clock_time();
				if(new - ext_key.key_pressed_tik1 > 1750*CLOCK_16M_SYS_TIMER_CLK_1MS) {
					ext_key.key_pressed_tik1 = new;
					cfg.flg.temp_F_or_C ^= 1;
					if(ext_key.rest_adv_int_tad) {
						set_adv_con_time(1); // restore default adv.
						ext_key.rest_adv_int_tad = 0;
					}
					SET_LCD_UPDATE();
				}
				if(new - ext_key.key_pressed_tik2 > 20*CLOCK_16M_SYS_TIMER_CLK_1S) {
					set_default_cfg();
				}
			}
#endif
		} else { // key off
			if (trg.flg.rds1_input) {
				// RDS1 off event, key released event
				trg.flg.rds1_input = 0;
				rds.count1++;
#ifdef GPIO_IR
				rds1_last_edge_tick = clock_time();
#endif
#if USE_WK_RDS_COUNTER32 // save 32 bits?
				if (rds.count_short[0] == 0) {
					flash_write_cfg(&rds.count_short[1], EEP_ID_RPC, sizeof(rds.count_short[1]));
				}
#endif
				if (trg.rds.type1 == RDS_COUNTER) { // counter mode
					if ((rds.count1 & 0xffff) == 0) { // report 'overflow 16 bit count'
						rds.event = trg.rds.type1;
					}
				} else if (trg.rds.type1 == RDS_SWITCH) { // switch mode
					rds.event = trg.rds.type1;
				}
			}
#if !(DEV_SERVICES & SERVICE_KEY)
			if (trg.rds.type1 == RDS_CONNECT) { // connect mode
					// connection key released
					u32 new = clock_time();
					ext_key.key_pressed_tik1 = new;
					ext_key.key_pressed_tik2 = new;
/* in app.c: main_loop()
					if(ext_key.rest_adv_int_tad < -80) {
						set_adv_con_time(1); // restore default adv.
						SET_LCD_UPDATE();
					}
*/
			}
#endif
		}
#ifdef GPIO_RDS2
	}
#endif
	if (trg.rds_time_report
			&& wrk.utc_time_sec - rds.report_tick > trg.rds_time_report) {
		rds.event = trg.rds.type1;
		rds.report_tick = wrk.utc_time_sec;
	}
	if (rds.event != RDS_NONE) {
		if(wrk.ble_connected) {
			if (rds.event == RDS_SWITCH) { // switch mode
				if(blc_ll_getTxFifoNumber() < 9)
					ble_send_trg_flg();
			}
		} else {
			start_ext_adv();
		}
		rds.event = RDS_NONE;
	}
#ifdef GPIO_IR
	irwm_schedule_next_sample(); // re-arm the adaptive IR sample wake
#endif
}

#endif // #if (DEV_SERVICES & SERVICE_RDS)
