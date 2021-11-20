/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#include "main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"
#include "product.h"
#include "defines.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#include "pqd_motor_power_measurement.h"
/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1;

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX,
  .hLowerOutputLimit       = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  FluxWeakeningCtrl component parameters Motor 1
  */
FW_Handle_t FW_M1 =
{
  .hMaxModule             = MAX_MODULE,
  .hDefaultFW_V_Ref       = (int16_t)FW_VOLTAGE_REF,
  .hDemagCurrent          = ID_DEMAG,
  .wNominalSqCurr         = ((int32_t)NOMINAL_CURRENT*(int32_t)NOMINAL_CURRENT),
  .hVqdLowPassFilterBW    = M1_VQD_SW_FILTER_BW_FACTOR,
  .hVqdLowPassFilterBWLOG = M1_VQD_SW_FILTER_BW_FACTOR_LOG
};

/**
  * @brief  PI Flux Weakening control parameters Motor 1
  */
PID_Handle_t PIDFluxWeakeningHandle_M1 =
{
  .hDefKpGain          = (int16_t)FW_KP_GAIN,
  .hDefKiGain          = (int16_t)FW_KI_GAIN,
  .wUpperIntegralLimit = 0,
  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT) * (int32_t)FW_KIDIV,
  .hUpperOutputLimit       = 0,
  .hLowerOutputLimit       = -INT16_MAX,
  .hKpDivisor          = (uint16_t)FW_KPDIV,
  .hKiDivisor          = (uint16_t)FW_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)FW_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)FW_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,
  .ModeDefault =					DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,
};
PWMC_R3_2_Handle_t PWM_Handle_M1 =
{
  {
    .pFctGetPhaseCurrents              = &R3_2_GetPhaseCurrents,
    .pFctSwitchOffPwm                  = &R3_2_SwitchOffPWM,
    .pFctSwitchOnPwm                   = &R3_2_SwitchOnPWM,
    .pFctCurrReadingCalib              = &R3_2_CurrentReadingPolarization,
    .pFctTurnOnLowSides                = &R3_2_TurnOnLowSides,
    .pFctSetADCSampPointSectX          = &R3_2_SetADCSampPointSectX,
    .pFctIsOverCurrentOccurred         = &R3_2_IsOverCurrentOccurred,
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = MC_NULL,
    .pFctRLDetectionModeDisable        = MC_NULL,
    .pFctRLDetectionModeSetDuty        = MC_NULL,
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
    .Sector = 0,
    .CntPhA = 0,
    .CntPhB = 0,
    .CntPhC = 0,
    .SWerror = 0,
    .TurnOnLowSidesAction = false,
    .OffCalibrWaitTimeCounter = 0,
    .Motor = M1,
    .RLDetectionMode = false,
    .Ia = 0,
    .Ib = 0,
    .Ic = 0,
    .DTTest = 0,
    .DTCompCnt = DTCOMPCNT,
    .PWMperiod          = PWM_PERIOD_CYCLES,
    .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),
    .Ton                 = TON,
    .Toff                = TOFF
  },
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,
  .PhaseAOffset = 0,
  .PhaseBOffset = 0,
  .PhaseCOffset = 0,

  .pParams_str = &R3_2_ParamsM1
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */

HALL_Handle_t HALL_M1 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP         =	65535,
    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
    .DPPConvFactor                     =  DPP_CONV_FACTOR,
  },
  .SensorPlacement     = HALL_SENSORS_PLACEMENT,
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,
 .TIMClockFreq       = HALL_TIM_CLK,
 .TIMx                = TIM3,

 .ICx_Filter          = M1_HALL_IC_FILTER,

 .PWMFreqScaling      = PWM_FREQ_SCALING,
 .HallMtpa            = HALL_MTPA,

 .H1Port             =  M1_HALL_H1_GPIO_Port,
 .H1Pin              =  M1_HALL_H1_Pin<<8,
 .H2Port             =  M1_HALL_H2_GPIO_Port,
 .H2Pin              =  M1_HALL_H2_Pin<<8,
 .H3Port             =  M1_HALL_H3_GPIO_Port,
 .H3Pin              =  M1_HALL_H3_Pin<<8,
 .lut				 = {0,1,2,3,4,5,6,7}
};

/**
  * temperature sensor parameters Motor 1
  */
NTC_Handle_t TempSensorParamsM1 =
{
  .bSensorType = TEMP_SENSOR_TYPE,
  .hExpectedTemp_d = 555,
  .hExpectedTemp_C = M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE,
  .TempRegConv =
  {
    .regADC = ADC1,
    .channel = MC_ADC_CHANNEL_0,
    .samplingTime = M1_TEMP_SAMPLING_TIME,
  },
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1
  */
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,
    .ConversionFactor    = (uint16_t)(BATTERY_VOLTAGE_GAIN),
  },

  .VbusRegConv =
  {
    .regADC = ADC1,
    .channel = VBUS_ADC_CHANNEL,
    .samplingTime = M1_VBUS_SAMPLING_TIME,
  },
  .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,
  .OverVoltageThreshold  = OV_VOLTAGE_THRESHOLD_V * BATTERY_VOLTAGE_GAIN,
  .UnderVoltageThreshold =  UD_VOLTAGE_THRESHOLD_V * BATTERY_VOLTAGE_GAIN,
  .aBuffer = RealBusVoltageSensorFilterBufferM1,
};
/*
UI_Handle_t UI_Params =
{
  .bDriveNum = 0,
};*/

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule          = MAX_MODULE,
  .MaxVd          	  = (uint16_t)(MAX_MODULE * FW_VOLTAGE_REF / 1000),
  .Circle_limit_table = MMITABLE,
  .Start_index        = START_INDEX,
};


/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
