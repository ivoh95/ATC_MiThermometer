#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

#define VERSION 0x48	 // BCD format (0x34 -> '3.4')
#define EEP_SUP_VER 0x09 // EEP data minimum supported version


// DevID:
#ifndef DEVICE_CGG1_ver
#define DEVICE_CGG1_ver		0 // =2022 - CGG1-M version 2022, or = 0 - CGG1-M version 2020,2021
#endif

#define DEVICE_MHO_C401		1	// E-Ink display MHO-C401 2020
#if DEVICE_CGG1_ver == 0
#define DEVICE_CGG1 		2  // E-Ink display Old CGG1-M "Qingping Temp & RH Monitor"
#else
#define DEVICE_CGG1 		7  // E-Ink display New CGG1-M "Qingping Temp & RH Monitor"
#endif
#define DEVICE_CGDK2 		6  // LCD display "Qingping Temp & RH Monitor Lite"
#define DEVICE_MHO_C401N	8	// E-Ink display MHO-C401 2022
#define DEVICE_MJWSD05MMC	9  // LCD display MJWSD05MMC
#define DEVICE_LYWSD03MMC	10	// LCD display LYWSD03MMC
// HW_VER_LYWSD03MMC_B14 = 0
// HW_VER_LYWSD03MMC_B19 = 3
// HW_VER_LYWSD03MMC_B16 = 4
// HW_VER_LYWSD03MMC_B17 = 5
// HW_VER_LYWSD03MMC_B15 = 10
#define DEVICE_MHO_C122		11	// LCD display MHO_C122
#define DEVICE_TB03F  		16	// DIY, TB-03F-Kit module + INA226 or MY18B20
#define DEVICE_TS0201   	17	// ZigBee TS0201, analog: IH-K009
#define DEVICE_TNK01		18	// DIY, PB-03F module, Water tank controller
//#define DEVICE_THB2		19	// PHY62x2 BLE https://github.com/pvvx/THB2
//#define DEVICE_BTH01		20	// PHY62x2 BLE https://github.com/pvvx/THB2
//#define DEVICE_TH05		21	// PHY62x2 BLE LCD https://github.com/pvvx/THB2
#define DEVICE_TH03Z   		22	// ZigBee TH03Z
//#define BOARD_THB1		23 // https://github.com/pvvx/THB2
//#define BOARD_TH05D		24 // TH05_V1.3 https://github.com/pvvx/THB2
//#define BOARD_TH05F		25 // TH05Y_V1.2 https://github.com/pvvx/THB2
//#define BOARD_THB3		26 // https://github.com/pvvx/THB2
#define DEVICE_ZTH01   		27	// ZigBee ZTH01
#define DEVICE_ZTH02   		28	// ZigBee ZTH02

#ifndef DEVICE_TYPE
#define DEVICE_TYPE			DEVICE_TNK01
#endif

// supported services by the device (bits)
#define SERVICE_OTA			0x00000001	// OTA all enable!
#define SERVICE_OTA_EXT		0x00000002	// Compatible BigOTA/ZigbeeOTA
#define SERVICE_PINCODE 	0x00000004	// support pin-code
#define SERVICE_BINDKEY 	0x00000008	// support encryption beacon (bindkey)
#define SERVICE_HISTORY 	0x00000010	// flash logger enable
#define SERVICE_SCREEN		0x00000020	// screen
#define SERVICE_LE_LR		0x00000040	// support extension advertise + LE Long Range
#define SERVICE_THS			0x00000080	// T & H sensor
#define SERVICE_RDS			0x00000100	// wake up when the reed switch + pulse counter
#define SERVICE_KEY			0x00000200	// key "connect"
#define SERVICE_OUTS		0x00000400	// GPIO output
#define SERVICE_INS			0x00000800	// GPIO input
#define SERVICE_TIME_ADJUST 0x00001000	// time correction enabled
#define SERVICE_HARD_CLOCK	0x00002000	// RTC enabled
#define SERVICE_TH_TRG		0x00004000	// use TH trigger out
#define SERVICE_LED			0x00008000	// use led
#define SERVICE_MI_KEYS		0x00010000	// use mi keys
#define SERVICE_PRESSURE	0x00020000	// pressure sensor
#define SERVICE_18B20		0x00040000	// use sensor(s) MY18B20
#define SERVICE_IUS			0x00080000	// use I and U sensor (INA226)

/* minimal DEV_SERVICES:
#define DEV_SERVICES ( SERVICE_OTA \ // OTA enable
		| SERVICE_OTA_EXT \ // Compatible BigOTA/ZigbeeOTA
		| SERVICE_THS )  // T & H sensor
*/

#if DEVICE_TYPE == DEVICE_MHO_C401
// TLSR8251F512ET24
// GPIO_PA5 - used EPD_SHD
// GPIO_PA6 - used EPD_RST
// GPIO_PA7 - SWS, free, (debug TX)
// GPIO_PB6 - used KEY, pcb mark "P5"
// GPIO_PB7 - used EPD_SDA
// GPIO_PC2 - SDA, used I2C
// GPIO_PC3 - SCL, used I2C
// GPIO_PC4 - used EPD_SHD
// GPIO_PD2 - used EPD_CSB
// GPIO_PD7 - used EPD_SCL

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_SCREEN \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_MI_KEYS \
)

#define USE_EPD			(600/50 - 1) // min update time x50 ms
#define MI_HW_VER_FADDR 0x55000 // Mi HW version

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 			GPIO_PC2
#define I2C_SDA 			GPIO_PC3
#define I2C_GROUP 			I2C_GPIO_GROUP_C2C3
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC3	PM_PIN_PULLUP_10K

#define EPD_SHD				GPIO_PC4 // should be high
#define PULL_WAKEUP_SRC_PC4 PM_PIN_PULLUP_10K

#define EPD_BUSY			GPIO_PA5
#define PULL_WAKEUP_SRC_PA5 PM_PIN_PULLUP_1M
#define PA5_INPUT_ENABLE	1
#define PA5_FUNC			AS_GPIO

#define EPD_RST				GPIO_PA6
#define PULL_WAKEUP_SRC_PA6 PM_PIN_PULLUP_1M
#define PA6_INPUT_ENABLE	1
#define PA6_DATA_OUT		1
#define PA6_OUTPUT_ENABLE	1
#define PA6_FUNC			AS_GPIO

#define EPD_CSB				GPIO_PD2
#define PULL_WAKEUP_SRC_PD2 PM_PIN_PULLUP_1M
#define PD2_INPUT_ENABLE	1
#define PD2_DATA_OUT		1
#define PD2_OUTPUT_ENABLE	1
#define PD2_FUNC			AS_GPIO

#define EPD_SDA				GPIO_PB7
#define PULL_WAKEUP_SRC_PB7 PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M
#define PB7_INPUT_ENABLE	1
#define PB7_DATA_OUT		1
#define PB7_OUTPUT_ENABLE	1
#define PB7_FUNC			AS_GPIO

#define EPD_SCL				GPIO_PD7
#define PULL_WAKEUP_SRC_PD7 PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M
#define PD7_INPUT_ENABLE	1
#define PD7_DATA_OUT		0
#define PD7_OUTPUT_ENABLE	1
#define PD7_FUNC			AS_GPIO

#define GPIO_TRG			GPIO_PA0	// none
#define PA0_INPUT_ENABLE	1
#define PA0_DATA_OUT		0
#define PA0_OUTPUT_ENABLE	0
#define PA0_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA0	PM_PIN_PULLDOWN_100K

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#if (DEV_SERVICES & SERVICE_KEY)
#define GPIO_KEY2 			GPIO_PB6	// input, pcb mark "P5"
#else
#define GPIO_RDS1 			GPIO_PB6	// Reed Switch, input, pcb mark "P5"
#endif
#define PB6_INPUT_ENABLE	1
#define PB6_DATA_OUT		0
#define PB6_OUTPUT_ENABLE	0
#define PB6_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB6 RDS1_PULLUP

#elif DEVICE_TYPE == DEVICE_MHO_C401N

// TLSR8251F512ET24
// GPIO_PA5 - used EPD_BUSY
// GPIO_PA6 - used EPD_CSB
// GPIO_PA7 - SWS, free, (debug TX)
// GPIO_PB6 - used KEY, pcb mark "P5"
// GPIO_PB7 - used EPD_RST2
// GPIO_PC2 - SDA, used I2C
// GPIO_PC3 - SCL, used I2C
// GPIO_PC4 - used EPD_SCL
// GPIO_PD2 - used EPD_SDA
// GPIO_PD7 - used EPD_RST

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_SCREEN \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_MI_KEYS \
)

#define USE_EPD			(600/50 - 1) // min update time ms

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 			GPIO_PC2
#define I2C_SDA 			GPIO_PC3
#define I2C_GROUP 			I2C_GPIO_GROUP_C2C3
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC3	PM_PIN_PULLUP_10K

#define EPD_RST2			GPIO_PB7 // should be high
#define PULL_WAKEUP_SRC_PB7 PM_PIN_PULLUP_1M

#define EPD_RST 			GPIO_PD7 // should be high
#define PULL_WAKEUP_SRC_PD7 PM_PIN_PULLUP_1M
#define PD7_INPUT_ENABLE	1
#define PD7_DATA_OUT		1
#define PD7_OUTPUT_ENABLE	1
#define PD7_FUNC			AS_GPIO

#define EPD_BUSY			GPIO_PA5
#define PULL_WAKEUP_SRC_PA5 PM_PIN_PULLUP_1M
#define PA5_INPUT_ENABLE	1
#define PA5_FUNC			AS_GPIO

#define EPD_CSB				GPIO_PA6
#define PULL_WAKEUP_SRC_PA6 PM_PIN_PULLUP_1M
#define PA6_INPUT_ENABLE	1
#define PA6_DATA_OUT		1
#define PA6_OUTPUT_ENABLE	1
#define PA6_FUNC			AS_GPIO

#define EPD_SDA				GPIO_PD2
#define PULL_WAKEUP_SRC_PD2 PM_PIN_PULLDOWN_100K
#define PD2_INPUT_ENABLE	1
#define PD2_DATA_OUT		1
#define PD2_OUTPUT_ENABLE	1
#define PD2_FUNC			AS_GPIO

#define EPD_SCL				GPIO_PC4
#define PULL_WAKEUP_SRC_PC4 PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M
#define PC4_INPUT_ENABLE	1
#define PC4_DATA_OUT		0
#define PC4_OUTPUT_ENABLE	1
#define PC4_FUNC			AS_GPIO

#define GPIO_TRG			GPIO_PA0	// none
#define PA0_INPUT_ENABLE	1
#define PA0_DATA_OUT		0
#define PA0_OUTPUT_ENABLE	0
#define PA0_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA0	PM_PIN_PULLDOWN_100K

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#if (DEV_SERVICES & SERVICE_KEY)
#define GPIO_KEY2 			GPIO_PB6	// key "Connect", input, pcb mark "P5"
#else
#define GPIO_RDS1 			GPIO_PB6	// Reed Switch, input, pcb mark "P5"
#endif
#define PB6_INPUT_ENABLE	1
#define PB6_DATA_OUT		0
#define PB6_OUTPUT_ENABLE	0
#define PB6_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB6 RDS1_PULLUP

#elif DEVICE_TYPE == DEVICE_CGG1

// TLSR8253F512ET32
// GPIO_PA0 - used EPD_RST
// GPIO_PA1 - used EPD_SHD
// GPIO_PA7 - SWS, free, (debug TX)
// GPIO_PB1 - free
// GPIO_PB4 - free
// GPIO_PB5 - free (ADC1)
// GPIO_PB6 - free (ADC2)
// GPIO_PB7 - used EPD_SDA
// GPIO_PC0 - SDA, used I2C
// GPIO_PC1 - SCL, used I2C
// GPIO_PC2 - TX, free, (Trigger, Output)
// GPIO_PC3 - RX, free, (RDS)
// GPIO_PC4 - used KEY
// GPIO_PD2 - used EPD_CSB
// GPIO_PD3 - free
// GPIO_PD4 - used EPD_BUSY
// GPIO_PD7 - used EPD_SCL

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_SCREEN \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_MI_KEYS \
)

#define USE_EPD			(550/50 - 1) // min update time ms

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8253F512ET32
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 			GPIO_PC0
#define I2C_SDA 			GPIO_PC1
#define I2C_GROUP 			I2C_GPIO_GROUP_C0C1
#define PULL_WAKEUP_SRC_PC0	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC1	PM_PIN_PULLUP_10K

#define EPD_SHD				GPIO_PA1 // should be high
#define PULL_WAKEUP_SRC_PA1 PM_PIN_PULLUP_10K

#define EPD_BUSY			GPIO_PD4
#define PULL_WAKEUP_SRC_PD4 PM_PIN_PULLUP_1M
#define PD4_INPUT_ENABLE	1
#define PD4_FUNC			AS_GPIO

#define EPD_RST				GPIO_PA0
#define PULL_WAKEUP_SRC_PA0 PM_PIN_PULLUP_1M
#define PA0_INPUT_ENABLE	1
#define PA0_DATA_OUT		1
#define PA0_OUTPUT_ENABLE	1
#define PA0_FUNC			AS_GPIO

#define EPD_CSB				GPIO_PD2
#define PULL_WAKEUP_SRC_PD2 PM_PIN_PULLUP_1M
#define PD2_INPUT_ENABLE	1
#define PD2_DATA_OUT		1
#define PD2_OUTPUT_ENABLE	1
#define PD2_FUNC			AS_GPIO

#define EPD_SDA				GPIO_PB7
#define PULL_WAKEUP_SRC_PB7 PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M
#define PB7_INPUT_ENABLE	1
#define PB7_DATA_OUT		1
#define PB7_OUTPUT_ENABLE	1
#define PB7_FUNC			AS_GPIO

#define EPD_SCL				GPIO_PD7
#define PULL_WAKEUP_SRC_PD7 PM_PIN_PULLDOWN_100K // PM_PIN_PULLUP_1M
#define PD7_INPUT_ENABLE	1
#define PD7_DATA_OUT		0
#define PD7_OUTPUT_ENABLE	1
#define PD7_FUNC			AS_GPIO

#if (DEV_SERVICES & SERVICE_KEY)

// PC4 - key
#define GPIO_KEY2			GPIO_PC4
#define PC4_INPUT_ENABLE	1
#define PC4_DATA_OUT		0
#define PC4_OUTPUT_ENABLE	0
#define PC4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC4 PM_PIN_PULLUP_1M

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PC3	// Reed Switch, Input
#define PC3_INPUT_ENABLE	1
#define PC3_DATA_OUT		0
#define PC3_OUTPUT_ENABLE	0
#define PC3_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC3 RDS1_PULLUP

#else

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PC4	// Reed Switch, Input
#define PC4_INPUT_ENABLE	1
#define PC4_DATA_OUT		0
#define PC4_OUTPUT_ENABLE	0
#define PC4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC4 RDS1_PULLUP

#endif

#define GPIO_TRG			GPIO_PC2
#define PC2_INPUT_ENABLE	1
#define PC2_DATA_OUT		0
#define PC2_OUTPUT_ENABLE	0
#define PC2_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLDOWN_100K

#if DIY_ADC_TO_TH // Special version: Temperature 0..36 = ADC pin PB5 input 0..3.6В
#define CHL_ADC1 			6	// B5P
#define GPIO_ADC1 			GPIO_PB5	// ADC1 input
#define PB5_OUTPUT_ENABLE	0
#define PB5_FUNC			AS_ADC
#endif

#if DIY_ADC_TO_TH // Special version: Humidity 0..36 = ADC pin PB6 input 0..3.6В
#define CHL_ADC2 			7	// B6P
#define GPIO_ADC2 			GPIO_PB6	// ADC2 input
#define PB6_OUTPUT_ENABLE	0
#define PB6_DATA_STRENGTH	0
#define PB6_FUNC			AS_GPIO
#endif

#elif DEVICE_TYPE == DEVICE_LYWSD03MMC
/* Original Flash markup:
  0x00000 Old Firmware bin
  0x20000 OTA New bin storage Area
  0x55000 SerialStr: "B1.5F2.0-CFMK-LB-JHBD---"
  0x74000 Pair & Security info (CFG_ADR_BIND)?
  0x76000 MAC address (CFG_ADR_MAC)
  0x77000 Customize freq_offset adjust cap value (CUST_CAP_INFO_ADDR)
  0x78000 Mijia key-code
  0x80000 End Flash (FLASH_SIZE)
 */
#define MI_HW_VER_FADDR 0x55000 // Mi HW version (DEVICE_LYWSD03MMC)
// TLSR8251F512ET24
// GPIO_PA5 - DM, free, pcb mark "reset" (KEY)
// GPIO_PA6 - DP, free, pcb mark "P8" (RDS)
// GPIO_PA7 - SWS, free, pcb mark "P14", (debug TX)
// GPIO_PB6 - used LCD, set "1"
// GPIO_PB7 - free, pcb mark "B1" (ADC1)
// GPIO_PC2 - SDA, used I2C, pcb mark "P12"
// GPIO_PC3 - SCL, used I2C, pcb mark "P15"
// GPIO_PC4 - free, pcb mark "P9" (ADC2)
// GPIO_PD2 - CS/PWM, free (TRG)
// GPIO_PD7 - free [B1.4], UART TX LCD [B1.6], pcb mark "P7"
#define DEV_SERVICES ( SERVICE_OTA \
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_SCREEN \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_KEY \
		| SERVICE_RDS \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_MI_KEYS \
)

#define USE_EPD				0 // min update time ms

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 			GPIO_PC2
#define I2C_SDA 			GPIO_PC3
#define I2C_GROUP 			I2C_GPIO_GROUP_C2C3
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC3	PM_PIN_PULLUP_10K

#define GPIO_TRG			GPIO_PD2	// Trigger, output
#define PD2_INPUT_ENABLE	1
#define PD2_DATA_OUT		0
#define PD2_OUTPUT_ENABLE	0
#define PD2_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PD2	PM_PIN_PULLDOWN_100K

#define GPIO_KEY2			GPIO_PA5	// key "Connect", input, pcb mark "P8"
#define PA5_INPUT_ENABLE	1
#define PA5_DATA_OUT		0
#define PA5_OUTPUT_ENABLE	0
#define PA5_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA5 PM_PIN_PULLUP_1M

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1			GPIO_PA6	// Reed Switch, input, pcb mark "P8"
#define PA6_INPUT_ENABLE	1
#define PA6_DATA_OUT		0
#define PA6_OUTPUT_ENABLE	0
#define PA6_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA6 RDS1_PULLUP

#define PULL_WAKEUP_SRC_PD7	PM_PIN_PULLUP_1M // UART TX (B1.6)
//#define PD7_INPUT_ENABLE	1
//#define PD7_FUNC			AS_UART

#define PULL_WAKEUP_SRC_PB6 PM_PIN_PULLUP_10K // LCD on low temp needs this, its an unknown pin going to the LCD controller chip

#if DIY_ADC_TO_TH // Special version: Temperature 0..36 = ADC pin PB7 input 0..3.6В, pcb mark "B1"
#define CHL_ADC1 			8	// B7P
#define GPIO_ADC1 			GPIO_PB7	// ADC input, pcb mark "B1"
#define PB7_OUTPUT_ENABLE	0
#define PB7_FUNC			AS_ADC
#endif

#if DIY_ADC_TO_TH // Special version: Humidity 0..36 = ADC pin PC4 input 0..3.6В, pcb mark "P9"
#define CHL_ADC2 			9	// C4P
#define GPIO_ADC2 			GPIO_PC4	// ADC input, pcb mark "P9"
#define PC4_OUTPUT_ENABLE	0
#define PC4_DATA_STRENGTH	0
#define PC4_FUNC			AS_GPIO
#endif

#elif DEVICE_TYPE == DEVICE_CGDK2

// TLSR8253F512ET32
// GPIO_PA0 - free, UART_RX, pcb mark "TP3", (Reed Switch, input)
// GPIO_PA1 - free
// GPIO_PA7 - SWS, free, (debug TX), pcb mark "TP17"
// GPIO_PB1 - free, UART_TX, pcb mark "TP1", (TRG)
// GPIO_PB4 - free
// GPIO_PB5 - free
// GPIO_PB6 - free, (ADC2)
// GPIO_PB7 - free, (ADC1)
// GPIO_PC0 - SDA, used I2C
// GPIO_PC1 - SCL, used I2C
// GPIO_PC2 - free
// GPIO_PC3 - free
// GPIO_PC4 - used KEY
// GPIO_PD2 - free
// GPIO_PD3 - free
// GPIO_PD4 - free
// GPIO_PD7 - free

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_SCREEN \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_MI_KEYS \
)

#define USE_EPD				0 // min update time ms

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8253F512ET32
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 			GPIO_PC0
#define I2C_SDA 			GPIO_PC1
#define I2C_GROUP 			I2C_GPIO_GROUP_C0C1
#define PULL_WAKEUP_SRC_PC0	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC1	PM_PIN_PULLUP_10K

#define GPIO_TRG			GPIO_PB1
#define PB1_INPUT_ENABLE	1
#define PB1_DATA_OUT		0
#define PB1_OUTPUT_ENABLE	0
#define PB1_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB1	PM_PIN_PULLDOWN_100K

#if (DEV_SERVICES & SERVICE_KEY)
// PC4 - key
#define GPIO_KEY2			GPIO_PC4	// key "Connect"
#define PC4_INPUT_ENABLE	1
#define PC4_DATA_OUT		0
#define PC4_OUTPUT_ENABLE	0
#define PC4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC4 PM_PIN_PULLUP_1M

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PA0	// Reed Switch, Input
#define PA0_INPUT_ENABLE	1
#define PA0_DATA_OUT		0
#define PA0_OUTPUT_ENABLE	0
#define PA0_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA0 RDS1_PULLUP
#else
#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PC4	// Reed Switch, Input
#define PC4_INPUT_ENABLE	1
#define PC4_DATA_OUT		0
#define PC4_OUTPUT_ENABLE	0
#define PC4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC4 RDS1_PULLUP
#endif

#if DIY_ADC_TO_TH // Special version: Temperature 0..36 = ADC pin PB7 input 0..3.6В
#define CHL_ADC1 			8	// B7P
#define GPIO_ADC1 			GPIO_PB7	// ADC1 input
#define PB7_OUTPUT_ENABLE	0
#define PB7_FUNC			AS_ADC
#endif

#if DIY_ADC_TO_TH // Special version: Humidity 0..36 = ADC pin PB6 input 0..3.6В
#define CHL_ADC2 			7	// B6P
#define GPIO_ADC2 			GPIO_PB6	// ADC2 input
#define PB6_OUTPUT_ENABLE	0
#define PB6_DATA_STRENGTH	0
#define PB6_FUNC			AS_GPIO
#endif

#elif DEVICE_TYPE == DEVICE_MJWSD05MMC
/* Original Flash markup:
  0x00000 Old Firmware bin
  0x40000 OTA New bin storage Area
  0x66000 Logger, saving measurements ?
  0x74000 Pair & Security info (CFG_ADR_BIND)
  0x76000 MAC address (CFG_ADR_MAC)
  0x77000 Customize freq_offset adjust cap value (CUST_CAP_INFO_ADDR)
  0x78000 Mijia key-code
  0x7D000 Ver+SerialStr: "V2.3F2.0-CFMK-LB-TMDZ---"
  0x7E000 ? 5A 07 00 02 CE 0C 5A 07 00 02 CB 0C 5A 07 00 02 CD 0C
  0x80000 End Flash (FLASH_SIZE)
 */

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_SCREEN \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_KEY \
		| SERVICE_HARD_CLOCK \
		| SERVICE_TH_TRG \
		| SERVICE_MI_KEYS \
)

#define MI_HW_VER_FADDR 0x7D000 // Mi HW version
// ### TLSR8250F512ET32
// GPIO_PA0 - UART_RX
// GPIO_PA1
// GPIO_PA7 - SWS

// GPIO_PB1
// GPIO_PB4
// GPIO_PB5 - R5 -> +BAT
// GPIO_PB6 - key1 (KEY2)
// GPIO_PB7 - R10 -> /INT AT85163T

// GPIO_PC0 - SDA, used I2C
// GPIO_PC1 - SCL, used I2C
// GPIO_PC2
// GPIO_PC3
// GPIO_PC4 - key2 (RDS)

// GPIO_PD2
// GPIO_PD3
// GPIO_PD4
// GPIO_PD7 - UART_TX

// scan i2c: 0x7c, 0x88, 0xa2

#define USE_EPD				0 // min update time ms

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		0
#define USE_SENSOR_SHT30		0

#define SHL_ADC_VBAT		6  // "B5P" in adc.h
#define GPIO_VBAT			GPIO_PB5 // R5 -> +Vbat
//#define PULL_WAKEUP_SRC_PB5 PM_PIN_PULLUP_1M
//#define PB5_INPUT_ENABLE	1
//#define PB5_DATA_OUT		1
//#define PB5_OUTPUT_ENABLE	1

#define I2C_MAX_SPEED 		400000 // 400 kHz
#define I2C_SCL 			GPIO_PC1
#define I2C_SDA 			GPIO_PC0
#define I2C_GROUP 			I2C_GPIO_GROUP_C0C1
#define PULL_WAKEUP_SRC_PC0	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC1	PM_PIN_PULLUP_10K

#define PULL_WAKEUP_SRC_PD7	PM_PIN_PULLUP_1M // UART TX

#define SHL_ADC_VBAT	6  // "B5P" in adc.h
#define GPIO_VBAT	GPIO_PB5 // R5 -> +Vbat
//#define PULL_WAKEUP_SRC_PB5 PM_PIN_PULLUP_1M
//#define PB5_INPUT_ENABLE	1
//#define PB5_DATA_OUT		1
//#define PB5_OUTPUT_ENABLE	1

#define GPIO_TRG			GPIO_PA0	// none
#define PA0_INPUT_ENABLE	1
#define PA0_DATA_OUT		0
#define PA0_OUTPUT_ENABLE	0
#define PA0_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA0	PM_PIN_PULLDOWN_100K

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1			GPIO_PC4
#define PC4_INPUT_ENABLE	1
#define PC4_DATA_OUT		0
#define PC4_OUTPUT_ENABLE	0
#define PC4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC4	RDS1_PULLUP

#define GPIO_KEY1			GPIO_PC4
#define GPIO_KEY2			GPIO_PB6
#define PB6_INPUT_ENABLE	1
#define PB6_DATA_OUT		0
#define PB6_OUTPUT_ENABLE	0
#define PB6_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB6	PM_PIN_PULLUP_10K

#elif DEVICE_TYPE == DEVICE_MHO_C122
// TLSR8251F512ET24
// GPIO_PA5 - DM, free, (TRG)
// GPIO_PA6 - DP, free, (RDS)
// GPIO_PA7 - SWS, free, (debug TX)
// GPIO_PB6 - used LCD, set "1"
// GPIO_PB7 - free, (ADC1)
// GPIO_PC2 - SDA, used I2C
// GPIO_PC3 - SCL, used I2C
// GPIO_PC4 - free
// GPIO_PD2 - CS/PWM, free
// GPIO_PD7 - free

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_SCREEN \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_MI_KEYS \
)

#define USE_EPD				0 // min update time ms

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 			GPIO_PC2
#define I2C_SDA 			GPIO_PC3
#define I2C_GROUP 			I2C_GPIO_GROUP_C2C3
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC3	PM_PIN_PULLUP_10K

#define GPIO_TRG			GPIO_PA5	// Trigger, output, pcb mark "reset"
#define PA5_INPUT_ENABLE	1
#define PA5_DATA_OUT		0
#define PA5_OUTPUT_ENABLE	0
#define PA5_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA5	PM_PIN_PULLDOWN_100K

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PA6	// Reed Switch, input, pcb mark "P8"
#define PA6_INPUT_ENABLE	1
#define PA6_DATA_OUT		0
#define PA6_OUTPUT_ENABLE	0
#define PA6_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA6 RDS1_PULLUP

#define PULL_WAKEUP_SRC_PD7	PM_PIN_PULLUP_1M // UART TX (B1.6)
//#define PD7_INPUT_ENABLE	1
//#define PD7_FUNC			AS_UART

#define PULL_WAKEUP_SRC_PB6 PM_PIN_PULLUP_10K // LCD on low temp needs this, its an unknown pin going to the LCD controller chip

#if DIY_ADC_TO_TH // Special version: Temperature 0..36 = ADC pin PB7 input 0..3.6В, pcb mark "B1"
#define CHL_ADC1 			8	// B7P
#define GPIO_ADC1 			GPIO_PB7	// ADC input, pcb mark "B1"
#define PB7_OUTPUT_ENABLE	0
#define PB7_FUNC			AS_ADC
#endif

#if DIY_ADC_TO_TH // Special version: Humidity 0..36 = ADC pin PC4 input 0..3.6В, pcb mark "P9"
#define CHL_ADC2 			9	// C4P
#define GPIO_ADC2 			GPIO_PC4	// ADC input, pcb mark "P9"
#define PC4_OUTPUT_ENABLE	0
#define PC4_DATA_STRENGTH	0
#define PC4_FUNC			AS_GPIO
#endif


#elif DEVICE_TYPE == DEVICE_TS0201

// GPIO_PB1 - TX
// GPIO_PB4 - LED
// GPIO_PB7 - RX
// GPIO_PC0 - KEY
// GPIO_PC2 - SDA
// GPIO_PC3 - SCL
// GPIO_PD7 - Alert

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_KEY \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_LED \
)

#define ZIGBEE_TUYA_OTA 	1
#define USE_EPD				0 // min update time ms

#define USE_SENSOR_CHT8305		1
#define USE_SENSOR_AHT20_30		1
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 	GPIO_PC3
#define I2C_SDA 	GPIO_PC2
#define I2C_GROUP 	I2C_GPIO_GROUP_C2C3
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC3	PM_PIN_PULLUP_10K

#define GPIO_KEY2			GPIO_PC0
#define PC0_INPUT_ENABLE	1
#define PC0_DATA_OUT		0
#define PC0_OUTPUT_ENABLE	0
#define PC0_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC0	PM_PIN_PULLUP_10K

#define GPIO_LED			GPIO_PB4
#define PB4_INPUT_ENABLE	1
#define PB4_DATA_OUT		1
#define PB4_OUTPUT_ENABLE	0
#define PB4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB4	PM_PIN_PULLDOWN_100K

#define GPIO_TRG			GPIO_PB1	// mark "TX"
#define PB1_INPUT_ENABLE	1
#define PB1_DATA_OUT		0
#define PB1_OUTPUT_ENABLE	0
#define PB1_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB1	PM_PIN_PULLDOWN_100K

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PB7	// mark "RX",  Reed Switch
#define PB7_INPUT_ENABLE	1
#define PB7_DATA_OUT		0
#define PB7_OUTPUT_ENABLE	0
#define PB7_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB7 RDS1_PULLUP

#elif DEVICE_TYPE == DEVICE_TH03Z

// GPIO_PB1 - TX
// GPIO_PB4 - LED
// GPIO_PB7 - RX
// GPIO_PC0 - KEY
// GPIO_PC2 - SDA
// GPIO_PC3 - SCL
// GPIO_PD7 - Alert

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_KEY \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_LED \
)

#define ZIGBEE_TUYA_OTA 	1
#define USE_EPD				0 // min update time ms

#define USE_SENSOR_CHT8305		1
#define USE_SENSOR_AHT20_30		1
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 	GPIO_PC3
#define I2C_SDA 	GPIO_PC2
#define I2C_GROUP 	I2C_GPIO_GROUP_C2C3
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC3	PM_PIN_PULLUP_10K

#define GPIO_KEY2			GPIO_PB5
#define PB5_INPUT_ENABLE	1
#define PB5_DATA_OUT		0
#define PB5_OUTPUT_ENABLE	0
#define PB5_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB5	PM_PIN_PULLUP_10K

#define GPIO_LED			GPIO_PB4
#define PB4_INPUT_ENABLE	1
#define PB4_DATA_OUT		1
#define PB4_OUTPUT_ENABLE	0
#define PB4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB4	PM_PIN_PULLDOWN_100K

#define GPIO_TRG			GPIO_PB1	// mark "TX"
#define PB1_INPUT_ENABLE	1
#define PB1_DATA_OUT		0
#define PB1_OUTPUT_ENABLE	0
#define PB1_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB1	PM_PIN_PULLDOWN_100K

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PB7	// mark "RX",  Reed Switch
#define PB7_INPUT_ENABLE	1
#define PB7_DATA_OUT		0
#define PB7_OUTPUT_ENABLE	0
#define PB7_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB7 RDS1_PULLUP

#elif DEVICE_TYPE == DEVICE_ZTH01

// GPIO_PB1 - TX
// GPIO_PB4 - KEY
// GPIO_PB5 - LED
// GPIO_PB7 - RX
// GPIO_PC2 - SDA
// GPIO_PC3 - SCL

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_KEY \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_LED \
)

#define ZIGBEE_TUYA_OTA 	1
#define USE_EPD				0 // min update time ms

#define USE_SENSOR_CHT8305		1
#define USE_SENSOR_AHT20_30		1
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 			GPIO_PC3
#define I2C_SDA 			GPIO_PC2
#define I2C_GROUP 			I2C_GPIO_GROUP_C2C3
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC3	PM_PIN_PULLUP_10K

#define GPIO_KEY2			GPIO_PB4
#define PB4_INPUT_ENABLE	1
#define PB4_DATA_OUT		0
#define PB4_OUTPUT_ENABLE	0
#define PB4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB4	PM_PIN_PULLUP_1M

#define GPIO_LED			GPIO_PB5
#define PB5_INPUT_ENABLE	1
#define PB5_DATA_OUT		1
#define PB5_OUTPUT_ENABLE	0
#define PB5_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB5	PM_PIN_PULLDOWN_100K

#define GPIO_TRG			GPIO_PB1	// mark "TX"
#define PB1_INPUT_ENABLE	1
#define PB1_DATA_OUT		0
#define PB1_OUTPUT_ENABLE	0
#define PB1_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB1	PM_PIN_PULLDOWN_100K

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PB7	// mark "RX",  Reed Switch
#define PB7_INPUT_ENABLE	1
#define PB7_DATA_OUT		0
#define PB7_OUTPUT_ENABLE	0
#define PB7_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB7 RDS1_PULLUP

#elif DEVICE_TYPE == DEVICE_ZTH02

// GPIO_PB1 - TX
// GPIO_PB4 - KEY
// GPIO_PB5 - LED
// GPIO_PB7 - RX
// GPIO_PC2 - SDA
// GPIO_PC3 - SCL

#define DEV_SERVICES ( SERVICE_OTA \
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_KEY \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_LED \
)

#define ZIGBEE_TUYA_OTA 	1
#define USE_EPD				0 // min update time ms

#define USE_SENSOR_CHT8305		1
#define USE_SENSOR_AHT20_30		1
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		1
#define USE_SENSOR_SHT30		1

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_MAX_SPEED 		700000 // 700 kHz
#define I2C_SCL 			GPIO_PC3
#define I2C_SDA 			GPIO_PC2
#define I2C_GROUP 			I2C_GPIO_GROUP_C2C3
#define PULL_WAKEUP_SRC_PC2	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC3	PM_PIN_PULLUP_10K

#define GPIO_KEY2			GPIO_PB4
#define PB4_INPUT_ENABLE	1
#define PB4_DATA_OUT		0
#define PB4_OUTPUT_ENABLE	0
#define PB4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB4	PM_PIN_PULLUP_1M

#define GPIO_LED			GPIO_PB5
#define PB5_INPUT_ENABLE	1
#define PB5_DATA_OUT		1
#define PB5_OUTPUT_ENABLE	0
#define PB5_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB5	PM_PIN_PULLDOWN_100K

#define GPIO_TRG			GPIO_PB1	// mark "TX"
#define PB1_INPUT_ENABLE	1
#define PB1_DATA_OUT		0
#define PB1_OUTPUT_ENABLE	0
#define PB1_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB1	PM_PIN_PULLDOWN_100K

#define RDS1_PULLUP			PM_PIN_PULLUP_1M
#define GPIO_RDS1 			GPIO_PB7	// mark "RX",  Reed Switch
#define PB7_INPUT_ENABLE	1
#define PB7_DATA_OUT		0
#define PB7_OUTPUT_ENABLE	0
#define PB7_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB7 RDS1_PULLUP


#elif DEVICE_TYPE == DEVICE_TNK01

// TLSR8253F512ET32 (TB-03F module)
// GPIO_PA0 - free
// GPIO_PA1 - free, used TRG
// GPIO_PA7 - SWS, used KEY
// GPIO_PB1 - free
// GPIO_PB4 - LED E
// GPIO_PB5 - LED W
// GPIO_PB6 - free (ADC2?)
// GPIO_PB7 - free
// GPIO_PC0 - SDA, used I2C AHT25
// GPIO_PC1 - SCL, used I2C AHT25
// GPIO_PC2 - LED B, used OUT HX710B
// GPIO_PC3 - LED R, used SCK HX710B
// GPIO_PC4 - LED G
// GPIO_PD2 - free
// GPIO_PD3 - free, used RDS1 Upper level sensor
// GPIO_PD4 - free, used RDS2 Lower level sensor
// GPIO_PD7 - free

#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_RDS \
		| SERVICE_KEY \
		| SERVICE_TIME_ADJUST \
		| SERVICE_TH_TRG \
		| SERVICE_PRESSURE \
)

#define USE_EPD					500 // min update time ms

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		0
#define USE_SENSOR_SHTC3		0
#define USE_SENSOR_SHT30		0

#define USE_CUSTOM_BEACON	0
#define USE_BTHOME_BEACON	1 	// = 1 BTHome v2 https://bthome.io/
#define USE_MIHOME_BEACON	0 	// = 1 Compatible with MiHome beacon
#define USE_ATC_BEACON		0

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_SCL 			GPIO_PC0
#define I2C_SDA 			GPIO_PC1
#define I2C_GROUP 			I2C_GPIO_GROUP_C0C1
#define PULL_WAKEUP_SRC_PC0	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC1	PM_PIN_PULLUP_10K

#define GPIO_KEY2			GPIO_PA7
#define PA7_INPUT_ENABLE	1
#define PA7_DATA_OUT		0
#define PA7_OUTPUT_ENABLE	0
#define PA7_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA7	PM_PIN_PULLUP_10K

#define GPIO_IR				GPIO_PB5
#define PB5_INPUT_ENABLE	1
#define PB5_DATA_OUT		1
#define PB5_OUTPUT_ENABLE	0
#define PB5_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB5	PM_PIN_PULLDOWN_100K

#define GPIO_TRG			GPIO_PA1
#define PA1_INPUT_ENABLE	1
#define PA1_DATA_OUT		0
#define PA1_OUTPUT_ENABLE	0
#define PA1_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA1	PM_PIN_PULLDOWN_100K

#define USE_RDS_WAKEAP		0

#define RDS1_PULLUP			PM_PIN_PULLUP_10K
#define GPIO_RDS1			GPIO_PD3	// Reed Switch "Full", input
#define PD3_INPUT_ENABLE	1
#define PD3_DATA_OUT		0
#define PD3_OUTPUT_ENABLE	0
#define PD3_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PD3	RDS1_PULLUP

// #define RDS2_PULLUP			PM_PIN_PULLUP_10K
// #define GPIO_RDS2			GPIO_PD4	// Reed Switch "Low", input
// #define PD4_INPUT_ENABLE	1
// #define PD4_DATA_OUT		0
// #define PD4_OUTPUT_ENABLE	0
// #define PD4_FUNC			AS_GPIO
// #define PULL_WAKEUP_SRC_PD4 RDS2_PULLUP

#define USE_SENSOR_HX71X	1

#define GPIO_HX71X_SCK		GPIO_PC3
#define PC3_INPUT_ENABLE	1
#define PC3_DATA_OUT		0
#define PC3_OUTPUT_ENABLE	0
#define PC3_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC3 PM_PIN_PULLUP_10K

#define GPIO_HX71X_DOUT  	GPIO_PC2
#define PC2_INPUT_ENABLE	1
#define PC2_DATA_OUT		0
#define PC2_OUTPUT_ENABLE	0
#define PC2_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PC2 PM_PIN_PULLUP_10K

#elif DEVICE_TYPE == DEVICE_TB03F

// TLSR8253F512ET32 (TB-03F module)
// GPIO_PA0 - free
// GPIO_PA1 - free, used TRG
// GPIO_PA7 - SWS, used KEY
// GPIO_PB1 - free
// GPIO_PB4 - LED E
// GPIO_PB5 - LED W
// GPIO_PB6 - free, 1-wire "1" MY18B20
// GPIO_PB7 - free, 1-wire "2" MY18B20
// GPIO_PC0 - SDA, used I2C
// GPIO_PC1 - SCL, used I2C
// GPIO_PC2 - LED B
// GPIO_PC3 - LED R
// GPIO_PC4 - LED G
// GPIO_PD2 - free
// GPIO_PD3 - free, used RDS1 Upper level sensor
// GPIO_PD4 - free, used RDS2 Lower level sensor
// GPIO_PD7 - free

#define USE_SENSOR_MY18B20		2 // 0 - Off, 1 - 1 sensor, 2 - 2 sensors

#if USE_SENSOR_MY18B20
#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_LE_LR \
		| SERVICE_THS \
		| SERVICE_TH_TRG \
		| SERVICE_RDS \
		| SERVICE_KEY \
		| SERVICE_TIME_ADJUST \
		| SERVICE_LED \
		| SERVICE_18B20 \
)
#else
#define USE_SENSOR_INA226		1
#define DEV_SERVICES ( SERVICE_OTA\
		| SERVICE_OTA_EXT \
		| SERVICE_PINCODE \
		| SERVICE_BINDKEY \
		| SERVICE_HISTORY \
		| SERVICE_LE_LR \
		| SERVICE_TH_TRG \
		| SERVICE_RDS \
		| SERVICE_KEY \
		| SERVICE_TIME_ADJUST \
		| SERVICE_LED \
		| SERVICE_IUS \
)
//| SERVICE_TH_TRG
// | SERVICE_THS
#endif


#define USE_EPD					0 // min update time ms

#define USE_SENSOR_CHT8305		0
#define USE_SENSOR_AHT20_30		0
#define USE_SENSOR_SHT4X		1
#define USE_SENSOR_SHTC3		0
#define USE_SENSOR_SHT30		0

#define USE_CUSTOM_BEACON	0
#define USE_BTHOME_BEACON	1 	// = 1 BTHome v2 https://bthome.io/
#define USE_MIHOME_BEACON	0 	// = 1 Compatible with MiHome beacon
#define USE_ATC_BEACON		0

#define SHL_ADC_VBAT		1  // "B0P" in adc.h
#define GPIO_VBAT			GPIO_PB0 // missing pin on case TLSR8251F512ET24
#define PB0_INPUT_ENABLE	1
#define PB0_DATA_OUT		1
#define PB0_OUTPUT_ENABLE	1
#define PB0_FUNC			AS_GPIO

#define I2C_SCL 			GPIO_PC0
#define I2C_SDA 			GPIO_PC1
#define I2C_GROUP 			I2C_GPIO_GROUP_C0C1
#define PULL_WAKEUP_SRC_PC0	PM_PIN_PULLUP_10K
#define PULL_WAKEUP_SRC_PC1	PM_PIN_PULLUP_10K

#define GPIO_KEY2			GPIO_PA7
#define PA7_INPUT_ENABLE	1
#define PA7_DATA_OUT		0
#define PA7_OUTPUT_ENABLE	0
#define PA7_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA7	PM_PIN_PULLUP_10K

#define GPIO_LED			GPIO_PB4 // (LED E)
#define PB4_INPUT_ENABLE	1
#define PB4_DATA_OUT		1
#define PB4_OUTPUT_ENABLE	0
#define PB4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB4	PM_PIN_PULLDOWN_100K

#define GPIO_TRG			GPIO_PA1
#define PA1_INPUT_ENABLE	1
#define PA1_DATA_OUT		0
#define PA1_OUTPUT_ENABLE	0
#define PA1_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PA1	PM_PIN_PULLDOWN_100K

#define USE_RDS_WAKEAP		0

#define RDS1_PULLUP			PM_PIN_PULLUP_10K
#define GPIO_RDS1			GPIO_PD3	// Reed Switch "Full", input
#define PD3_INPUT_ENABLE	1
#define PD3_DATA_OUT		0
#define PD3_OUTPUT_ENABLE	0
#define PD3_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PD3	RDS1_PULLUP

#define RDS2_PULLUP			PM_PIN_PULLUP_10K
#define GPIO_RDS2			GPIO_PD4	// Reed Switch "Low", input
#define PD4_INPUT_ENABLE	1
#define PD4_DATA_OUT		0
#define PD4_OUTPUT_ENABLE	0
#define PD4_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PD4 RDS2_PULLUP

#if USE_SENSOR_MY18B20

#define GPIO_ONEWIRE1		GPIO_PB6
#define PB6_INPUT_ENABLE	1
#define PB6_DATA_OUT		0
#define PB6_OUTPUT_ENABLE	0
#define PB6_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB6	PM_PIN_PULLDOWN_100K

#if USE_SENSOR_MY18B20 == 2
#define GPIO_ONEWIRE2		GPIO_PB7
#define PB7_INPUT_ENABLE	1
#define PB7_DATA_OUT		0
#define PB7_OUTPUT_ENABLE	0
#define PB7_FUNC			AS_GPIO
#define PULL_WAKEUP_SRC_PB7	PM_PIN_PULLDOWN_100K
#endif

#endif // USE_SENSOR_MY18B20

//#define SENSOR_SLEEP_MEASURE 	0
#define USE_FLASH_SERIAL_UID	1

#else // DEVICE_TYPE
#error ("DEVICE_TYPE = ?")
#endif // DEVICE_TYPE == ?

#ifndef USE_DEVICE_INFO_CHR_UUID
#define USE_DEVICE_INFO_CHR_UUID 	1 // = 1 enable Device Information Characteristics
#endif

#ifndef USE_FLASH_SERIAL_UID
#define USE_FLASH_SERIAL_UID		0 // = 1 Set my_SerialStr "$SOC_ID_Rev-$FLASH_JEDEC-$FLASH_UID"
#endif

#ifndef SENSOR_SLEEP_MEASURE
#define SENSOR_SLEEP_MEASURE	1
#endif

#ifndef USE_SENSOR_HX71X
#define USE_SENSOR_HX71X	0
#endif

#ifndef USE_AVERAGE_BATTERY
#define USE_AVERAGE_BATTERY	1
#endif

#define USE_DISPLAY_CLOCK 	1 // = 1 display clock, = 0 smile blinking

#ifndef USE_ATC_BEACON
#define USE_ATC_BEACON		1
#endif
#ifndef USE_CUSTOM_BEACON
#define USE_CUSTOM_BEACON	1
#endif
#ifndef USE_MIHOME_BEACON
#define USE_MIHOME_BEACON	1 // = 1 Compatible with MiHome beacon
#endif
#ifndef USE_BTHOME_BEACON
#define USE_BTHOME_BEACON	1 // = 1 BTHome v2 https://bthome.io/
#endif
#if (USE_CUSTOM_BEACON + USE_BTHOME_BEACON + USE_MIHOME_BEACON + USE_ATC_BEACON) == 0
#define USE_BTHOME_BEACON	1 // = 1 BTHome v2 https://bthome.io/
#elif (USE_CUSTOM_BEACON || USE_MIHOME_BEACON || USE_ATC_BEACON) && defined(GPIO_RDS2)
#warning "Cusrom or ATC or MiHome beacon not support RDS2!"
#endif

/* Special DIY version - Voltage Logger:
 * Temperature 0..36.00 = ADC pin PB7 input 0..3.6V, LYWSD03MMC pcb mark "B1"
 * Humidity 0..36.00 = ADC pin PC4 input 0..3.6V, LYWSD03MMC pcb mark "P9"
 * Set DIY_ADC_TO_TH 1 */
#define DIY_ADC_TO_TH 	0

#define UART_PRINT_DEBUG_ENABLE		0 // =1 use u_printf() (PA7/SWS), source: SDK/components/application/print/u_printf.c

#if UART_PRINT_DEBUG_ENABLE
#define PRINT_BAUD_RATE 1500000 // real ~1000000
#define DEBUG_INFO_TX_PIN	GPIO_PA7 // SWS
#define PA7_DATA_OUT		1
#define PA7_OUTPUT_ENABLE	1
#define PULL_WAKEUP_SRC_PA7 PM_PIN_PULLUP_1M
#define PA7_FUNC		AS_GPIO
#endif // UART_PRINT_DEBUG_ENABLE

#if (DEV_SERVICES & SERVICE_TH_TRG) && (!defined(GPIO_TRG))
#error "Set GPIO_TRG!"
#endif
#if (DEV_SERVICES & SERVICE_RDS) && (!defined(GPIO_RDS1))
#error "Set GPIO_RDS1!"
#endif

#define MODULE_WATCHDOG_ENABLE		0 //
#define WATCHDOG_INIT_TIMEOUT		15000  //ms (min 5000 ms if pincode)

/* DEVICE_LYWSD03MMC Average consumption (Show battery on, Comfort on, advertising 2.0 sec, measure 10 sec):
 * 16 MHz - 17.43 uA
 * 24 MHz - 17.28 uA
 * 32 MHz - 17.36 uA
 * (TX +3 dB)
 * Average consumption Original Xiaomi LYWSD03MMC (advertising 1700 ms, measure 6800 ms):
 * 18.64 uA (TX 0 dB)
 */
#define CLOCK_SYS_CLOCK_HZ  	24000000 // 16000000, 24000000, 32000000, 48000000
enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

#define pm_wait_ms(t) cpu_stall_wakeup_by_timer0(t*CLOCK_SYS_CLOCK_1MS);
#define pm_wait_us(t) cpu_stall_wakeup_by_timer0(t*CLOCK_SYS_CLOCK_1US);

#define RAM _attribute_data_retention_ // short version, this is needed to keep the values in ram after sleep

/* Flash map:
  0x00000 Old Firmware bin
  0x20000 OTA New bin storage Area
  0x40000 User Data Area (Logger, saving measurements) (FLASH_ADDR_START_MEMO)
  0x74000 Pair & Security info (CFG_ADR_BIND)
  0x76000 MAC address (CFG_ADR_MAC)
  0x77000 Customize freq_offset adjust cap value (CUST_CAP_INFO_ADDR)
  0x78000 User Data Area (EEP, saving configuration) (FMEMORY_SCFG_BASE_ADDR), conflict in master mode (CFG_ADR_PEER)
  0x80000 End Flash (FLASH_SIZE)
 */
/* flash sector address with binding information */
#define		CFG_ADR_BIND	0x74000 //no master, slave device (blt_config.h)

//#define CHG_CONN_PARAM	// test
#define DEV_NAME "pvvx_ble"

#include "vendor/common/default_config.h"

#if defined(__cplusplus)
}
#endif
