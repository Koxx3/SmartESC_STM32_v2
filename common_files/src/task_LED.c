/*
 * m365
 *
 * Copyright (c) 2021 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "task_LED.h"
#include "task_init.h"
#include "task_cli.h"
#include "main.h"
#include "conf_general.h"
#include "FreeRTOS.h"
#include "task.h"
#include "product.h"
#include "VescCommand.h"


TaskHandle_t LEDHandle;
const osThreadAttr_t LED_attributes = {
  .name = "LED",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};

void prv_LED_blink(uint32_t ticks){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, pdFALSE);
	vTaskDelay(ticks);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, pdTRUE);
	vTaskDelay(ticks);
}


//#define  MC_NO_ERROR  (uint16_t)(0x0000u)      /**< @brief No error.*/
//#define  MC_NO_FAULTS  (uint16_t)(0x0000u)     /**< @brief No error.*/
//#define  MC_FOC_DURATION  (uint16_t)(0x0001u)  /**< @brief Error: FOC rate to high.*/
//#define  MC_OVER_VOLT  (uint16_t)(0x0002u)     /**< @brief Error: Software over voltage.*/
//#define  MC_UNDER_VOLT  (uint16_t)(0x0004u)    /**< @brief Error: Software under voltage.*/
//#define  MC_OVER_TEMP  (uint16_t)(0x0008u)     /**< @brief Error: Software over temperature.*/
//#define  MC_START_UP  (uint16_t)(0x0010u)      /**< @brief Error: Startup failed.*/
//#define  MC_SPEED_FDBK  (uint16_t)(0x0020u)    /**< @brief Error: Speed feedback.*/
//#define  MC_BREAK_IN  (uint16_t)(0x0040u)      /**< @brief Error: Emergency input (Over current).*/
//#define  MC_SW_ERROR  (uint16_t)(0x0080u)      /**< @brief Software Error.*/


void task_LED(void * argument)
{
	volatile uint32_t last_fault_time=0;
	volatile uint16_t last_fault = 0;
	/* Infinite loop */
	for(;;)
	{
		if(pMCI[M1]->pSTM->hFaultOccurred){
			if(last_fault != pMCI[M1]->pSTM->hFaultOccurred){
				last_fault_time = (xTaskGetTickCount() / 2) + mc_conf.m_fault_stop_time_ms;
			}
			last_fault = pMCI[M1]->pSTM->hFaultOccurred;
			if((xTaskGetTickCount() / 2) > last_fault_time){
#if ERROR_PRINTING
				commands_printf("FAULTS: %x", last_fault);

#endif
				MCI_FaultAcknowledged(pMCI[M1]);
				vTaskDelay(MS_TO_TICKS(100));
				MCI_StopMotor(pMCI[M1]);
				vTaskDelay(MS_TO_TICKS(100));
				MCI_ExecTorqueRamp(pMCI[M1], 0, 0);
				MCI_StartMotor(pMCI[M1]);
				last_fault = 0;
			}
			prv_LED_blink(MS_TO_TICKS(100));
		}else{
			prv_LED_blink(MS_TO_TICKS(500));
		}

	}
}

void task_LED_init(){
	xTaskCreate(task_LED, "tskLED", 128, NULL, PRIO_NORMAL, &LEDHandle);
}
