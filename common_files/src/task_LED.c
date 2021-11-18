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


osThreadId_t LEDHandle;
const osThreadAttr_t LED_attributes = {
  .name = "LED",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};

void prv_LED_blink(uint32_t ticks){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, pdFALSE);
	osDelay(ticks);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, pdTRUE);
	osDelay(ticks);
}


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
	LEDHandle = osThreadNew(task_LED, NULL, &LED_attributes);
}
