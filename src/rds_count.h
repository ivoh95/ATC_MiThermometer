/*
 * rds_count.h
 *
 *  Created on: 20.02.2022
 *      Author: pvvx
 */

#ifndef RDS_COUNT_H_
#define RDS_COUNT_H_

#include "app_config.h"

#if (DEV_SERVICES & SERVICE_RDS)

//#include "mi_beacon.h"

#ifndef RDS1_PULLUP
#define RDS1_PULLUP PM_PIN_PULLUP_1M
#endif

#ifndef RDS2_PULLUP
#define RDS2_PULLUP PM_PIN_PULLUP_1M
#endif

enum {
	RDS_NONE = 0,
	RDS_SWITCH,
	RDS_COUNTER,
	RDS_CONNECT // version 4.2+
} RDS_TYPES;

typedef struct _rds_count_t {
	u32 report_tick; // timer reed switch count report interval (wrk.utc_time_sec)
	union {				// rs1 counter pulses
		u8 count1_byte[4];
		u16 count1_short[2];
		u32 count1;
	};
/*
#ifdef GPIO_RDS2
	union {				// rs2 counter pulses
		u8 count2_byte[4];
		u16 count2_short[2];
		u32 count2;
	};
#endif
*/
	u8 event;  // Reed Switch event
} rds_count_t;
extern rds_count_t rds;		// Reed switch pulse counter

#ifdef GPIO_RDS1
#ifdef GPIO_IR
// IR optical pulse read (DEVICE_IRWM). The IR emitter is strobed on only for the
// sample. As the meter's fork crosses the beam the receiver dwells near the input
// threshold, so a single threshold would chatter and over-count; two thresholds
// plus a held state give a dead-band (values between them keep the previous level),
// so one tooth = exactly one edge.
#ifndef IR_SAMPLES
#define IR_SAMPLES		24		// reads per strobe
#endif
#ifndef IR_HI_THRES
#define IR_HI_THRES		20		// >= this many high reads -> beam clear
#endif
#ifndef IR_LO_THRES
#define IR_LO_THRES		4		// <= this many high reads -> beam blocked
#endif
#ifndef IR_SETTLE_US
#define IR_SETTLE_US	20		// emitter-on to sample delay (original used 0)
#endif
extern u8 rds1_beam_state;		// persisted dead-band state (pre-invert)

static inline u8 get_rds1_input(void) {
	gpio_set_output_en(GPIO_IR, 1);
	gpio_set_input_en(GPIO_IR, 0);
	gpio_write(GPIO_IR, 1);				// strobe IR emitter on
	gpio_set_input_en(GPIO_RDS1, 1);
	sleep_us(IR_SETTLE_US);
	u8 high = 0;
	for (u8 i = 0; i < IR_SAMPLES; i++) {
		if (BM_IS_SET(reg_gpio_in(GPIO_RDS1), GPIO_RDS1 & 0xff))
			high++;
	}
	gpio_write(GPIO_IR, 0);				// emitter off
	gpio_set_output_en(GPIO_IR, 0);
	gpio_set_input_en(GPIO_IR, 1);
	if (high >= IR_HI_THRES)
		rds1_beam_state = 1;			// beam clear
	else if (high <= IR_LO_THRES)
		rds1_beam_state = 0;			// beam blocked (else: hold previous - dead-band)
	u8 r = rds1_beam_state;
	if(trg.rds.rs1_invert)
		r ^= 1;
	return r;
}
#else
static inline u8 get_rds1_input(void) {
	u8 r = BM_IS_SET(reg_gpio_in(GPIO_RDS1), GPIO_RDS1 & 0xff)? 1 : 0;
	if(trg.rds.rs1_invert)
		r ^= 1;
	return r;
}
#endif // GPIO_IR

static inline void rds1_input_on(void) {
	gpio_setup_up_down_resistor(GPIO_RDS1, RDS1_PULLUP);
}

static inline void rds1_input_off(void) {
	gpio_setup_up_down_resistor(GPIO_RDS1, PM_PIN_UP_DOWN_FLOAT);
}

#endif

#ifdef GPIO_RDS2
static inline u8 get_rds2_input(void) {
	u8 r = BM_IS_SET(reg_gpio_in(GPIO_RDS2), GPIO_RDS2 & 0xff)? 1 : 0;
	if(trg.rds.rs2_invert)
		r ^= 1;
	return r;
}

static inline void rds2_input_off(void) {
	gpio_setup_up_down_resistor(GPIO_RDS2, PM_PIN_UP_DOWN_FLOAT);
}

static inline void rds2_input_on(void) {
	gpio_setup_up_down_resistor(GPIO_RDS2, RDS2_PULLUP);
}
#endif


static inline void rds_input_on(void) {
#ifdef GPIO_RDS1
	gpio_setup_up_down_resistor(GPIO_RDS1, RDS1_PULLUP);
#endif
#ifdef GPIO_RDS2
	gpio_setup_up_down_resistor(GPIO_RDS2, RDS2_PULLUP);
#endif
}


void rds_init(void);
void rds_suspend(void);
void rds_task(void);

#endif // (DEV_SERVICES & SERVICE_RDS)

#endif /* RDS_COUNT_H_ */
