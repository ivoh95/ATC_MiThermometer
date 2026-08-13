/*
 * tigger.h
 *
 *  Created on: 02.01.2021
 *      Author: pvvx
 */

#ifndef _TIGGER_H_
#define _TIGGER_H_

#include "app_config.h"

#if (DEV_SERVICES & SERVICE_TH_TRG) || (DEV_SERVICES & SERVICE_RDS)

typedef struct __attribute__((packed)) _trigger_flg_t {
	u8 	rds1_input	:	1; // Reed Switch, input
	u8 	trg_output	:	1; // GPIO_TRG pin output value (pull Up/Down)
	u8 	trigger_on	:	1; // Output GPIO_TRG pin is controlled according to the set parameters threshold temperature or humidity
	u8 	temp_out_on :	1; // Temperature trigger event
	u8 	humi_out_on :	1; // Humidity trigger event
	u8 	key_pressed	:	1; // key2 pressed (Connect/Reset)
	u8 	rds2_input 	:	1; // Reed Switch 2, input
}trigger_flg_t;

#if (DEV_SERVICES & SERVICE_RDS)

typedef struct __attribute__((packed)) _rds_type_t {
	u8 type1		: 2;		// RDS_TYPES, Reed switch 1 types: 0 - none, 1 - switch, 2 - counter [3 - connect]
	u8 type2		: 2;		// RDS_TYPES, Reed switch 2 types: 0 - none, 1 - switch, 2 - counter [3 - connect]
	// version 3.9+
	u8 rs1_invert	: 1;		// GPIO events (Reed switch 1): 0 - rising, 1 - falling
	u8 rs2_invert	: 1;		// GPIO events (Reed switch 2): 0 - rising, 1 - falling
}rds_type_t;

#endif

typedef struct __attribute__((packed)) _trigger_t {
#if (DEVICE_TYPE == DEVICE_IRWM)
	// IR water-meter config. Byte-compatible with the TelinkMiFlasher CMD_ID_TRG
	// (0x44) fields, so the stock flasher can set them (it scales the value x100
	// on the threshold/hysteresis inputs, so enter raw/100, e.g. 25 -> "0.25"):
	u16 sample_active_ms;   // <- "Temperature threshold": sample period while flowing
	u16 sample_idle_ms;     // <- "Humidity threshold": sample period when idle
	u16 sample_deepidle_ms; // <- "Temperature hysteresis": period after long silence
	u16 ml_per_pulse;       // <- "Humidity hysteresis": volume per pulse, 0.001 L units
#elif (DEV_SERVICES & SERVICE_TH_TRG)
	s16 temp_threshold; // x0.01°, Set temp threshold
	s16 humi_threshold; // x0.01%, Set humi threshold
	s16 temp_hysteresis; // Set temp hysteresis, -327.67..327.67 °
	s16 humi_hysteresis; // Set humi hysteresis, -327.67..327.67 %
#endif
#if (DEV_SERVICES & SERVICE_RDS)
	// version 3.6+
	u16 rds_time_report; // Reed switch count report interval (sec)
	rds_type_t rds;	  		  // type Reed switch
#endif
	union {
		trigger_flg_t flg;
		u8	flg_byte;
	};
}trigger_t;

#define FEEP_SAVE_SIZE_TRG (sizeof(trg)-1)

extern trigger_t trg;
extern const trigger_t def_trg;

#if (DEV_SERVICES & SERVICE_TH_TRG)
void set_trigger_out(void);
void test_trg_on(void);
#endif

#endif // (DEV_SERVICES & SERVICE_TH_TRG) || (DEV_SERVICES & SERVICE_RDS)

#endif /* _TIGGER_H_ */
