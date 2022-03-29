/**
  ******************************************************************************
  * @file    sto_pll_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          State Observer + PLL Speed & Position Feedback component of the Motor
  *          Control SDK.
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
  * @ingroup SpeednPosFdbk_STO
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HFI_SPEEDNPOSFDBK_H
#define __HFI_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "sto_speed_pos_fdbk.h"
#include "pid_regulator.h"
#include "mc_math.h"


/* Exported types ------------------------------------------------------------*/

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  
  alphabeta_t Vinject;
  uint8_t inject_seq;
  int16_t inductance;
  int32_t inductance_now;
  int32_t sample_prev;
  int32_t d_out;
  int32_t d_cnt;
  int8_t sign_last;
  uint32_t hfi_cnt;
  int16_t hfi_hyst;
  int16_t hfi_voltage;
  int16_t hfi_angle;
  int16_t speed_avg;
  int16_t hElAngle_last;
  uint16_t samples_avg;
  Trig_Components trig;
} HFI_Handle_t;


/* Exported functions ------------------------------------------------------- */

/* It initializes the HFI object */
void HFI_Init( HFI_Handle_t * pHandle );

alphabeta_t HFI_Inject( HFI_Handle_t * pHandle, alphabeta_t Valfabeta, alphabeta_t Ialphabeta, qd_t Iqd);
void HFI_update(HFI_Handle_t * pHandle);
bool HFI_is_ready(HFI_Handle_t * pHandle);
int16_t HFI_get_angle(HFI_Handle_t * pHandle);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__HFI_SPEEDNPOSFDBK_H*/

