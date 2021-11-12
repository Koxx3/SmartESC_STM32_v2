/**
  ******************************************************************************
  * @file    stm32f10x_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32F1 Family.
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
  * @ingroup STM32F10x_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32f1xx_ll_exti.h"
/* USER CODE BEGIN Includes */
#include "product.h"
#include "task_cli.h"

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F10x_IRQ_Handlers STM32F10x IRQ Handlers
  * @{
  */
/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC1_2_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void DMAx_R1_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);
void USART_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void PFC_TIM_IRQHandler(void);
void EXTI15_10_IRQHandler (void);

/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */

  ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);

  TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/

  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */

}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_M1_IRQn 0 */
    LL_TIM_ClearFlag_UPDATE(TIM1);
    R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);

   /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

   /* USER CODE END TIMx_UP_M1_IRQn 1 */
}

/**
  * @brief  This function handles first motor BRK interrupt.
  * @param  None
  * @retval None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */
  if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx))
  {
    LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);

    R3_2_BRK_IRQHandler(&PWM_Handle_M1);

  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */

  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M1);
    /* USER CODE BEGIN M1 HALL_Update */

    /* USER CODE END M1 HALL_Update   */
  }
  else
  {
    /* Nothing to do */
  }
  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx))
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    HALL_TIMx_CC_IRQHandler(&HALL_M1);
    /* USER CODE BEGIN M1 HALL_CC1 */

    /* USER CODE END M1 HALL_CC1 */
  }
  else
  {
  /* Nothing to do */
  }
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */
}

/*Start here***********************************************************/
/*GUI, this section is present only if serial communication is enabled*/
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  * @retval None
  */
void USART_IRQHandler(void)
{
  /* USER CODE BEGIN USART_IRQn 0 */

  /* USER CODE END USART_IRQn 0 */
  uint16_t hUSART_SR = VESC_USART->SR;

  if (hUSART_SR & USART_SR_ORE) /* Overrun error occurs before SR access */
  {
    /* Send Overrun message */
    LL_USART_ClearFlag_ORE(VESC_USART); /* Clear overrun flag */
    /* USER CODE BEGIN USART_ORE */

    /* USER CODE END USART_ORE   */
  }
  uint8_t c;

  if (hUSART_SR & USART_SR_RXNE) /* Valid data received */
  {


	c = LL_USART_ReceiveData8(VESC_USART);
	if(UART_RX!=NULL) xStreamBufferSendFromISR(UART_RX, &c, sizeof(c), 0);

  /* USER CODE BEGIN USART_RXNE */

  /* USER CODE END USART_RXNE   */
  }

  /* USER CODE BEGIN USART_IRQn 1 */

  /* USER CODE END USART_IRQn 1 */
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */
  TSK_HardwareFaultTask();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
    {
      if (LL_USART_IsActiveFlag_ORE(VESC_USART)) /* Overrun error occurs */
      {
        /* Send Overrun message */
        //UFCP_OVR_IRQ_Handler(&pUSART);
        LL_USART_ClearFlag_ORE(VESC_USART); /* Clear overrun flag */
        //UI_SerialCommunicationTimeOutStop();
      }

      if (LL_USART_IsActiveFlag_TXE(VESC_USART))
      {
        //UFCP_TX_IRQ_Handler(&pUSART);
      }

      if (LL_USART_IsActiveFlag_RXNE(VESC_USART)) /* Valid data have been received */
      {
        //uint16_t retVal;
        //retVal = *(uint16_t*)(UFCP_RX_IRQ_Handler(&pUSART,LL_USART_ReceiveData8(pUSART.USARTx)));
        //if (retVal == 1)
        //{
        //  UI_SerialCommunicationTimeOutStart();
        //}
        //if (retVal == 2)
        //{
        //  UI_SerialCommunicationTimeOutStop();
        //
      }
      else
      {
      }
    }
  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */

}

/**
  * @brief  This function handles Button IRQ on PIN PC15.
  */
void EXTI15_10_IRQHandler (void)
{
	/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_15) )
  {
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_15);
    //UI_HandleStartStopButton_cb ();
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
