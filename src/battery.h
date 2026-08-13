#ifndef _BATTERY_H_
#define _BATTERY_H_

#include "app_config.h"

// Battery chemistry presets for the voltage->percent mapping (get_battery_level)
// and the low/end-of-life thresholds. Select one per build with BATTERY_TYPE,
// e.g. in a device block of app_config.h: #define BATTERY_TYPE BATT_TYPE_ALKALINE
#define BATT_TYPE_COIN		0	// CR2032 (Xiaomi stock)
#define BATT_TYPE_ALKALINE	1	// 2x AA/AAA alkaline
#define BATT_TYPE_NIMH		2	// 2x AA/AAA NiMH

#ifndef BATTERY_TYPE
#define BATTERY_TYPE		BATT_TYPE_NIMH	// fork default (keeps prior values)
#endif

#if BATTERY_TYPE == BATT_TYPE_COIN
#define MAX_VBAT_MV		3000 // 100% (fresh ~3.0V under load)
#define MIN_VBAT_MV		2200 // 0%
#define LOW_VBAT_MV		2800 // level set LOW_CONNECT_LATENCY
#elif BATTERY_TYPE == BATT_TYPE_ALKALINE
#define MAX_VBAT_MV		3000 // 100% (2x fresh ~1.5V/cell under load)
#define MIN_VBAT_MV		2050 // ~0%, just above the flash/deep-sleep floor
#define LOW_VBAT_MV		2200
#elif BATTERY_TYPE == BATT_TYPE_NIMH
#define MAX_VBAT_MV		2600 // 100% (2x ~1.3V/cell under load)
#define MIN_VBAT_MV		2200 // 0% (~1.1V/cell)
#define LOW_VBAT_MV		2400
#else
#error "Unknown BATTERY_TYPE (use BATT_TYPE_COIN / BATT_TYPE_ALKALINE / BATT_TYPE_NIMH)"
#endif

// Flash-write floor (chip limit): below this, stop writing Flash and deep-sleep.
// Chemistry-independent, so it lives outside the presets.
#define END_VBAT_MV		2000

u16 get_adc_mv(u32 p_ain);

#define get_battery_mv() get_adc_mv(SHL_ADC_VBAT)	// Channel B0P/B5P

u8 get_battery_level(u16 battery_mv);

#endif // _BATTERY_H_
