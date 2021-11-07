/**
  ******************************************************************************
  * @file    motorcontrol.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem initialization functions.
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
#include "mc_tuning.h"
#include "mc_interface.h"
#include "mc_tasks.h"
#include "motorcontrol.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCAPI
  * @{
  */


MCI_Handle_t* pMCI[NBR_OF_MOTORS];
MCT_Handle_t* pMCT[NBR_OF_MOTORS];

/**
 * @brief Initializes and configures the Motor Control Subsystem
 *
 *  This function initializes and configures all the structures and components needed
 * for the Motor Control subsystem required by the Application. It expects that
 * all the peripherals needed for Motor Control purposes are already configured but
 * that their interrupts are not enabled yet.
 *
 * CubeMX calls this function after all peripherals initializations and
 * before the NVIC is configured
 */
__weak void MX_MotorControl_Init(void)
{
  /* Initialize the Motor Control Subsystem */
  MCboot(pMCI,pMCT);
  mc_lock_pins();

}

void vPortSetupTimerInterrupt( void )
{
  /* Reconfigure the SysTick interrupt to fire every 500 us. */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/SYS_TICK_FREQUENCY);
}
/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
