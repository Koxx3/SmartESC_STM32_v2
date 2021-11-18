#ifndef DEFINES_H_
#define DEFINES_H_

#define HW_NAME					"40"

#include "parameters_conversion.h"
#include "mc_config.h"
#include "mc_tuning.h"
#include "mc_interface.h"

// Firmware version
#define FW_VERSION_MAJOR			5
#define FW_VERSION_MINOR			02

// Set to 0 for building a release and iterate during beta test builds
#define FW_TEST_VERSION_NUMBER		0

#define STM32_UUID					((uint32_t*)0x1FFF7A10)
#define STM32_UUID_8				((uint8_t*)0x1FFF7A10)

#define MS_TO_TICKS( xTimeInMs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) SYS_TICK_FREQUENCY ) / ( TickType_t ) 1000 ) )

#define ANG_TO_DEG(x) (x/(65536.0/360.0))
#define DEG_TO_ANG(x) (x*(65536.0/360.0))

#define ADC_GAIN (float)(3.3 / 4095.0)

extern MCT_Handle_t* pMCT[NBR_OF_MOTORS];
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];




#endif
