#ifndef DEFINES_H_
#define DEFINES_H_

#define HW_NAME					"40"

// Firmware version
#define FW_VERSION_MAJOR			5
#define FW_VERSION_MINOR			02

// Set to 0 for building a release and iterate during beta test builds
#define FW_TEST_VERSION_NUMBER		0

#define STM32_UUID					((uint32_t*)0x1FFF7A10)
#define STM32_UUID_8				((uint8_t*)0x1FFF7A10)


#define VOLT_SCALING (65535/ADC_REFERENCE_VOLTAGE*VBUS_PARTITIONING_FACTOR)
#define ANG_TO_DEG(x) (x/(65536.0/360.0))
#define DEG_TO_ANG(x) (x*(65536.0/360.0))

#define CURRENT_FACTOR_A 317.73
#define CURRENT_FACTOR_mA 0.31773


#endif
