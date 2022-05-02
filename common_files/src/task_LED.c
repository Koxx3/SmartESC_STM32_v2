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
#include "VescToSTM.h"


TaskHandle_t LEDHandle;

en_brake brake_mode = BRAKE_LIGHT_OFF;

extern stm_state VescToSTM_mode;

void prv_LED_blink(uint32_t speed){
	static uint16_t cnt=0;
	static uint8_t brake_cnt=0;
	if(cnt>speed){
		cnt=0;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}else{
		cnt++;
	}


	if(VescToSTM_mode == STM_STATE_BRAKE && (FW_M1.AvAmpere_qd.q < (-1*CURRENT_FACTOR_A) || FW_M1.AvAmpere_qd.q > (1*CURRENT_FACTOR_A))){
		if(brake_cnt>10){
			brake_cnt=0;
			HAL_GPIO_TogglePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin);
		}else{
			brake_cnt++;
		}
	}else{
		if(brake_mode == BRAKE_LIGHT_ON){
			HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_RESET);
			brake_cnt=10;
		}else{
			HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_SET);
			brake_cnt=0;
		}
	}


}



void task_LED_set_brake_light(en_brake mode){
	brake_mode = mode;
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
	uint32_t last_fault_time=0;
	uint16_t last_fault = 0;
	/* Infinite loop */
	for(;;)
	{
		if(pMCI[M1]->pSTM->hFaultOccurred){
			if(last_fault != pMCI[M1]->pSTM->hFaultOccurred){
				last_fault_time = (xTaskGetTickCount() / 2) + mc_conf.m_fault_stop_time_ms;
			}
			last_fault = pMCI[M1]->pSTM->hFaultOccurred;
			if((xTaskGetTickCount() / 2) > last_fault_time){
#if ERROR_PRINTING && VESC_TOOL_ENABLE
				commands_printf(((port_str*)argument)->phandle, "FAULTS: %x", last_fault);
#endif

#if AUTO_RESET_FAULT
				MCI_FaultAcknowledged(pMCI[M1]);
				VescToSTM_set_current(0, 0);
				STM[M1].bState = RUN;
				VescToSTM_pwm_start();

				last_fault = 0;
#endif
			}
			prv_LED_blink(10);
		}else{
			prv_LED_blink(50);
		}


		vTaskDelay(MS_TO_TICKS(10));
	}
}

void task_LED_init(port_str * port){
	xTaskCreate(task_LED, "tskLED", 128, (void*)port, PRIO_NORMAL, &LEDHandle);
}
