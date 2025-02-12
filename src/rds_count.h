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

#define EXT_ADV_INTERVAL ADV_INTERVAL_50MS
#define EXT_ADV_COUNT 6

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
	uint32_t report_tick; // timer reed switch count report interval (utc_time_sec)
	union {				// rs1 counter pulses
		uint8_t count1_byte[4];
		uint16_t count1_short[2];
		uint32_t count1;
	};
/*
#ifdef GPIO_RDS2
	union {				// rs2 counter pulses
		uint8_t count2_byte[4];
		uint16_t count2_short[2];
		uint32_t count2;
	};
#endif
*/
	uint8_t event;  // Reed Switch event
} rds_count_t;
extern rds_count_t rds;		// Reed switch pulse counter

#ifdef GPIO_RDS1
static inline uint8_t get_rds1_input(void) {
	#ifdef GPIO_IR
		gpio_set_output_en(GPIO_IR, 1);
		gpio_set_input_en(GPIO_IR, 0);
		gpio_write(GPIO_IR, 1);
	#endif
	gpio_set_input_en(GPIO_RDS1, 1);
	uint8_t numberOfSamples = 24; 
	uint8_t highSamples = 0; 
	uint8_t lowSamples = 0;

	for (int i = 0; i<numberOfSamples; i++){
		uint8_t thisSample = BM_IS_SET(reg_gpio_in(GPIO_RDS1), GPIO_RDS1 & 0xff)? 1 : 0;
		if (thisSample){ //high sample
			highSamples++;
		} else{ //low sample
			lowSamples++;
		}
	}
	uint8_t r = 0; 
	if(highSamples > (numberOfSamples-6)){ // 80 percent of the samples or greater were high. 
		r = 1; 
	}
	#ifdef GPIO_IR
		gpio_set_output_en(GPIO_IR, 0);
		gpio_set_input_en(GPIO_IR, 1);
		gpio_write(GPIO_IR, 0);
	#endif
	if(trg.rds.rs1_invert)
		r ^= 1;
	return r;
}

static inline void rds1_input_on(void) {
	gpio_setup_up_down_resistor(GPIO_RDS1, RDS1_PULLUP);
}

static inline void rds1_input_off(void) {
	gpio_setup_up_down_resistor(GPIO_RDS1, PM_PIN_UP_DOWN_FLOAT);
}

#endif

#ifdef GPIO_RDS2
static inline uint8_t get_rds2_input(void) {
	uint8_t r = BM_IS_SET(reg_gpio_in(GPIO_RDS2), GPIO_RDS2 & 0xff)? 1 : 0;
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
