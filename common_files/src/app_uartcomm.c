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

#include "main.h"
#include "app.h"
#include "utils.h"
#include "VescToSTM.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "TTerm.h"
#include "mc_config.h"
#include <string.h>
#include <math.h>
#include "product.h"



#define CH_CFG_ST_FREQUENCY             10000
#define MIN_MS_WITHOUT_POWER			50
#define CIRC_BUF_SZ       				32  /* must be power of two */
#define DMA_WRITE_PTR(channel) 			((CIRC_BUF_SZ - channel.hdmarx->Instance->CNDTR) & (CIRC_BUF_SZ - 1) )  //huart_cobs->hdmarx->Instance->NDTR.
#define RPM_FILTER_SAMPLES				8

uint8_t usart2_rx_dma_buffer[CIRC_BUF_SZ] __attribute__ ((aligned(32)));
uint8_t usart3_rx_dma_buffer[CIRC_BUF_SZ] __attribute__ ((aligned(32)));

uint8_t app_connection_timout = 8;

osThreadId_t task_app_handle;
const osThreadAttr_t task_app_attributes = {
  .name = "APP-USART",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 7
};

static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float decoded_level = 0.0;
static volatile float ms_without_power = 0.0;

void my_uart3_send_data(uint8_t *tdata, uint16_t tnum){
	//send data
	while( HAL_UART_Transmit_DMA(&APP_USART_DMA, tdata, tnum) != HAL_OK ) vTaskDelay(1);

	//Waiting to send status OK
	while(HAL_DMA_GetState(&APP_USART_TX_DMA) != HAL_DMA_STATE_READY){
		APP_USART_DMA.gState = HAL_UART_STATE_READY;
		vTaskDelay(1);
	}
}

void task_app(void * argument)
{
	HAL_UART_Receive_DMA(&APP_USART_DMA, usart2_rx_dma_buffer, sizeof(usart2_rx_dma_buffer));
	CLEAR_BIT(APP_USART_DMA.Instance->CR3, USART_CR3_EIE);

	//uint32_t rd2_ptr=0;
	is_running = true;

  /* Infinite loop */
	for(;;)
	{
//		while(rd2_ptr != DMA_WRITE_PTR(APP_USART_DMA)) {
//			//ninebot_parse_usart_frame(usart2_rx_dma_buffer[rd2_ptr], 1);
//			rd2_ptr++;
//			rd2_ptr &= (CIRC_BUF_SZ - 1);
//		}

		vTaskDelay(500);

		// For safe start when fault codes occur
//		if (VescToSTM_get_fault() != FAULT_CODE_NONE) {
//			continue;
//		}

//		uint8_t* input = (uint8_t* )ninebot_get_raw_input();
//		float pwr = (float)input[0];
//		float brake = (float)input[1];
//
//		if(pwr < 30 || brake < 30){
//			brake = 41;
//			//continue;
//		}
//
//		pwr = (float)(pwr - 40) / 100;
//		pwr = utils_map(pwr, 0.0, 1.54, 0.0, 1.0);
//
//		// Truncate the read voltage
//		utils_truncate_number(&pwr, 0.0, 1.0);
//		decoded_level = pwr;
//
//		// Map and truncate the read voltage
//		brake = (float)(brake - 40) / 100;
//		brake = utils_map(brake, 0.0, 1.54, 0.0, 1.0);
//		utils_truncate_number(&brake, 0.0, 1.0);
//
//		// All pins and buttons are still decoded for debugging, even
//		// when output is disabled.
//		if (false) {
//			continue;
//		}
//
//		pwr -= brake;
//
//		// Apply deadband
//		utils_deadband(&pwr, 0.05, 1.0);
//
//		// Apply throttle curve
//		pwr = utils_throttle_curve(pwr, -0.5, 0, 2);
//
//		// Apply ramping
//		static uint16_t last_time = 0;
//		static float pwr_ramp = 0.0;
//		float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? 0.3 : 0.1;
//
//		if (ramp_time > 0.01) {
//			const float ramp_step = (float)(HAL_GetTick() - last_time) / (ramp_time * 1000.0);
//			utils_step_towards(&pwr_ramp, pwr, ramp_step);
//			last_time = HAL_GetTick();
//			pwr = pwr_ramp;
//		}
//
//		bool current_mode_brake = false;
//
//		if (fabsf(pwr) < 0.001) {
//			ms_without_power += (1000.0 * (float)1) / (float)CH_CFG_ST_FREQUENCY;
//		}
//
//		// Reset timeout
//		VescToSTM_timeout_reset();

//		if (current_mode_brake) {
//			VescToSTM_set_brake_current_rel(pwr);
//		} else {
//			float current_out = pwr;
//			bool is_reverse = false;
//			if (current_out < 0.0) {
//				is_reverse = true;
//				current_out = -current_out;
//				pwr = -pwr;
//			}
//
//			if (is_reverse) {
//				VescToSTM_set_current_rel(-current_out);
//			} else {
//				VescToSTM_set_current_rel(current_out);
//			}
//		}
	}
}

void task_app_init(){
	is_running = true;
	task_app_handle = osThreadNew(task_app, NULL, &task_app_attributes);
}
