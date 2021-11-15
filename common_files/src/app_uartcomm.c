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
#define MIN_MS_WITHOUT_POWER			500
#define CIRC_BUF_SZ       				32  /* must be power of two */
#define DMA_WRITE_PTR 					((CIRC_BUF_SZ - APP_USART_DMA.hdmarx->Instance->CNDTR) & (CIRC_BUF_SZ - 1) )  //huart_cobs->hdmarx->Instance->NDTR.
#define RPM_FILTER_SAMPLES				8

uint8_t usart_rx_dma_buffer[CIRC_BUF_SZ];

uint8_t app_connection_timout = 8;

osThreadId_t task_app_handle;
const osThreadAttr_t task_app_attributes = {
  .name = "APP-USART",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};

static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float decoded_level = 0.0;
static volatile float ms_without_power = 0.0;

void app_uartcomm_dev_write(uint8_t *buffer, uint8_t tx_count)
{
	HAL_UART_Transmit_DMA(&APP_USART_DMA, buffer, tx_count);
	while(APP_USART_TX_DMA.State != HAL_DMA_STATE_READY){
		APP_USART_DMA.gState = HAL_UART_STATE_READY;
		vTaskDelay(1);
	}
}

void txest(float te){
}

void task_app(void * argument)
{
	HAL_UART_Receive_DMA(&APP_USART_DMA, usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));
	CLEAR_BIT(APP_USART_DMA.Instance->CR3, USART_CR3_EIE);

	uint32_t rd_ptr=0;
	is_running = true;

  /* Infinite loop */
	for(;;)
	{
		while(rd_ptr != DMA_WRITE_PTR) {
			//ninebot_parse_usart_frame(usart_rx_dma_buffer[rd_ptr], 1);
			rd_ptr++;
			rd_ptr &= (CIRC_BUF_SZ - 1);
		}

		vTaskDelay(1);

		if (stop_now) {
			is_running = false;
			return;
		}

		// For safe start when fault codes occur
		//if (mc_interface_get_fault() != FAULT_CODE_NONE) {
		//	ms_without_power = 0;
		//}

		float pwr = 0;
		pwr = utils_map(pwr, 0.0, 2.55, 0.0, 1.0);

		// Truncate the read voltage
		utils_truncate_number(&pwr, 0.0, 1.0);
		decoded_level = pwr;

		float brake = 0.0;

		// Map and truncate the read voltage
		brake = utils_map(brake, 0.0, 2.55, 0.0, 1.0);
		utils_truncate_number(&brake, 0.0, 1.0);

		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		pwr -= brake;

		// Apply deadband
		utils_deadband(&pwr, 0.05, 1.0);

		// Apply throttle curve
		pwr = utils_throttle_curve(pwr, -0.5, 0, 2);

		// Apply ramping
		//static uint16_t last_time = 0;
		static float pwr_ramp = 0.0;
		float ramp_time = 0.0;//fabsf(pwr) > fabsf(pwr_ramp) ? 0.3 : 0.1;

		if (ramp_time > 0.01) {
			//const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			const float ramp_step = 1000;
			utils_step_towards(&pwr_ramp, pwr, ramp_step);
			//last_time = chVTGetSystemTimeX();
			pwr = pwr_ramp;
		}

		float current_rel = pwr;
		bool current_mode = false;
		bool current_mode_brake = false;

		current_mode = true;

		if (fabsf(pwr) < 0.001) {
			ms_without_power += (1000.0 * (float)1) / (float)CH_CFG_ST_FREQUENCY;
		}

		// If safe start is enabled and the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER && false) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			//VescToSTM_set_brake(0);

			continue;
		}

		// Reset timeout
		VescToSTM_timeout_reset();

		if (current_mode) {
			if (current_mode_brake) {
				//VescToSTM_set_brake_current_rel(current_rel);
			} else {
				float current_out = current_rel;
				bool is_reverse = false;
				if (current_out < 0.0) {
					is_reverse = true;
					current_out = -current_out;
					current_rel = -current_rel;
				}

				if (is_reverse) {
					//VescToSTM_set_current_rel(-current_out);
				} else {
					//VescToSTM_set_current_rel(current_out);
				}
			}
		}
	}
}

void task_app_init(){
	stop_now = false;
	task_app_handle = osThreadNew(task_app, NULL, &task_app_attributes);
}
