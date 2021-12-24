
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM       16000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM       0 /*!< rpm, mechanical,
                                                           absolute value */
#define MEAS_ERRORS_BEFORE_FAULTS       20 /*!< Number of speed
                                                             measurement errors before
                                                             main sensor goes in fault */
/****** Hall sensors ************/
#define HALL_MEAS_ERRORS_BEFORE_FAULTS  3 /*!< Number of failed
                                                           derived class specific speed
                                                           measurements before main sensor
                                                           goes in fault */

#define HALL_AVERAGING_FIFO_DEPTH        12 /*!< depth of the FIFO used to
                                                           average mechanical speed in
                                                           0.1Hz resolution */
#define HALL_MTPA  true

/* USER CODE BEGIN angle reconstruction M1 */
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */

#define PWM_FREQUENCY   16000
#define PWM_FREQ_SCALING 1

#define LOW_SIDE_SIGNALS_ENABLING        LS_PWM_TIMER
#define SW_DEADTIME_NS                   800 /*!< Dead-time to be inserted
                                                           by FW, only if low side
                                                           signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                           number of PWM cycles */
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT         500
#define PID_TORQUE_KI_DEFAULT         300
#define PID_TORQUE_KD_DEFAULT         100
#define PID_FLUX_KP_DEFAULT           500
#define PID_FLUX_KI_DEFAULT           300
#define PID_FLUX_KD_DEFAULT           100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      1024
#define TF_KIDIV                      16384
#define TF_KDDIV                      8192
#define TF_KPDIV_LOG                  LOG2(1024)
#define TF_KIDIV_LOG                  LOG2(16384)
#define TF_KDDIV_LOG                  LOG2(8192)
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ       1000 /*!<Execution rate of speed
                                                      regulation loop (Hz) */

#define PID_SPEED_KP_DEFAULT          400/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          50/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV                      16
#define SP_KIDIV                      256
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2(16)
#define SP_KIDIV_LOG                  LOG2(256)
#define SP_KDDIV_LOG                  LOG2(16)

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX                          4766

/* Default settings */
#define DEFAULT_CONTROL_MODE           STC_TORQUE_MODE /*!< STC_TORQUE_MODE or
                                                        STC_SPEED_MODE */
#define DEFAULT_TARGET_SPEED_RPM      0
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/_RPM)
#define DEFAULT_TORQUE_COMPONENT       0
#define DEFAULT_FLUX_COMPONENT         0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_PROT_ENABLING        ENABLE
#define UV_VOLTAGE_PROT_ENABLING        ENABLE
#define OV_VOLTAGE_THRESHOLD_V          42 /*!< Over-voltage
                                                         threshold */
#define UD_VOLTAGE_THRESHOLD_V          10 /*!< Under-voltage
                                                          threshold */
#if 0
#define ON_OVER_VOLTAGE                 TURN_OFF_PWM /*!< TURN_OFF_PWM,
                                                         TURN_ON_R_BRAKE or
                                                         TURN_ON_LOW_SIDES */
#endif /* 0 */
#define R_BRAKE_SWITCH_OFF_THRES_V      34

#define OV_TEMPERATURE_THRESHOLD_C      70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     5 /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /*!< In case ON_OVER_VOLTAGE
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by
                                                          power stage) */
/******************************   START-UP PARAMETERS   **********************/

#define TRANSITION_DURATION            25  /* Switch over duration, ms */
/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(1)
/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(1)
/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES (1 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/
#define FW_VOLTAGE_REF                985 /*!<Vs reference, tenth
                                                        of a percent */
#define FW_KP_GAIN                    6000 /*!< Default Kp gain */
#define FW_KI_GAIN                    10000 /*!< Default Ki gain */
#define FW_KPDIV                      32768
                                                /*!< Kp gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through
                                                algebrical right shifts to speed up PIs execution.
                                                Only in this case this parameter specifies the
                                                number of right shifts to be executed */
#define FW_KIDIV                      32768
                                                /*!< Ki gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through
                                                algebrical right shifts to speed up PIs execution.
                                                Only in this case this parameter specifies the
                                                number of right shifts to be executed */
#define FW_KPDIV_LOG                  LOG2(32768)
#define FW_KIDIV_LOG                  LOG2(32768)


/*  Maximum Torque Per Ampere strategy parameters */

#define MTPA_ENABLING
#define SEGDIV                         595
#define ANGC                           {0,0,0,0,-55,0,-54,-54}
#define OFST                           {0,0,0,0,4,-1,4,4}

/*** On the fly start-up ***/

/****** State Observer + CORDIC ***/
#define CORD_VARIANCE_THRESHOLD           4  /*!<Maxiumum accepted
                                                            variance on speed
                                                            estimates (percentage) */
#define CORD_F1                          16384
#define CORD_F2                          16384
#define CORD_F1_LOG                      LOG2(16384)
#define CORD_F2_LOG                      LOG2(16384)

/* State observer constants */
#define CORD_GAIN1                       -22073
#define CORD_GAIN2                       5845

#define CORD_MEAS_ERRORS_BEFORE_FAULTS   3  /*!< Number of consecutive errors
                                                           on variance test before a speed
                                                           feedback error is reported */
#define CORD_FIFO_DEPTH_DPP              64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in dpp format */
#define CORD_FIFO_DEPTH_DPP_LOG          LOG2(64)

#define CORD_FIFO_DEPTH_UNIT            64  /*!< Depth of the FIFO used
                                                           to average mechanical speed
                                                           in dpp format */
#define CORD_MAX_ACCEL_DPPP              60  /*!< Maximum instantaneous
                                                              electrical acceleration (dpp
                                                              per control period) */
#define CORD_BEMF_CONSISTENCY_TOL        64  /* Parameter for B-emf
                                                           amplitude-speed consistency */
#define CORD_BEMF_CONSISTENCY_GAIN       64  /* Parameter for B-emf
                                                          amplitude-speed consistency */

/******************************   START-UP PARAMETERS   **********************/

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM          1000

#define NB_CONSECUTIVE_TESTS           2 /* corresponding to
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT         17 /*!< It expresses how much
                                                            estimated speed can exceed
                                                            forced stator electrical
                                                            without being considered wrong.
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT         15  /*!< It expresses how much
                                                             estimated speed can be below
                                                             forced stator electrical
                                                             without being considered wrong.
                                                             In 1/16 of forced speed */

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*__DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
