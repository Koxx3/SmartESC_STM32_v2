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
#include "task_cli.h"
#include "task_init.h"
#include "task_pwr.h"
#include "TTerm.h"
#include "printf.h"
#include "mc_config.h"
#include <string.h>
#include "packet.h"
#include "VescCommand.h"
#include "VescToSTM.h"
#include "product.h"
#include "app.h"
#include "ninebot.h"
#include "utils.h"


void putbuffer(unsigned char *buf, unsigned int len, port_str * port){
	if(port->half_duplex){
		port->uart->Instance->CR1 &= ~USART_CR1_RE;
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(port->uart, buf, len);
	while(port->uart->hdmatx->State != HAL_DMA_STATE_READY){
		port->uart->gState = HAL_UART_STATE_READY;
		vTaskDelay(1);
	}
	if(port->half_duplex) port->uart->Instance->CR1 |= USART_CR1_RE;
}

void comm_uart_send_packet(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle) {
	packet_send_packet(data, len, phandle);
}

void process_packet(unsigned char *data, unsigned int len, PACKET_STATE_t * phandle){
	commands_process_packet(data, len, &comm_uart_send_packet, phandle);
}

static uint32_t uart_get_write_pos(port_str * port){
	return ( ((uint32_t)port->rx_buffer_size - port->uart->hdmarx->Instance->CNDTR) & ((uint32_t)port->rx_buffer_size -1));
}

void task_cli(void * argument)
{
	uint32_t rd_ptr=0;
	port_str * port = (port_str*) argument;
	uint8_t * usart_rx_dma_buffer = pvPortMalloc(port->rx_buffer_size);
	HAL_UART_MspInit(port->uart);
	if(port->half_duplex){
		HAL_HalfDuplex_Init(port->uart);
	}
	HAL_UART_Receive_DMA(port->uart, usart_rx_dma_buffer, port->rx_buffer_size);
	CLEAR_BIT(port->uart->Instance->CR3, USART_CR3_EIE);

	port->phandle = packet_init(putbuffer, process_packet, port);

	VescToSTM_set_brake_rel_int(0);

	app_check_timer();

	uint16_t slow_update_cnt=0;

  /* Infinite loop */
	for(;;)
	{
		/* `#START TASK_LOOP_CODE` */
		while(rd_ptr != uart_get_write_pos(port)) {
			packet_process_byte(usart_rx_dma_buffer[rd_ptr], port->phandle);
			rd_ptr++;
			rd_ptr &= ((uint32_t)port->rx_buffer_size - 1);
		}

		send_sample(port->phandle);
		send_position(port->phandle);
		VescToSTM_handle_timeout();

		if(appconf.app_to_use == APP_ADC_UART){
			app_check_timer();

			if(slow_update_cnt==0){
				if(m365_to_display.mode & 0x40){
					m365_to_display.speed = VescToSTM_get_speed()*2.2369;
				}else{
					m365_to_display.speed = VescToSTM_get_speed()*3.6;

				}
				m365_to_display.speed *= DIR_MUL;
				m365_to_display.battery = utils_map(VescToSTM_get_battery_level(0), 0, 1, 0, 96);
				m365_to_display.beep=0;
				m365_to_display.faultcode=pMCI[M1]->pSTM->hFaultOccurred;

			}else{
				slow_update_cnt++;
				if(slow_update_cnt==100) slow_update_cnt=0;
			}
		}

		if(ulTaskNotifyTake(pdTRUE, 1)){
			if(appconf.app_to_use == APP_ADC_UART){
				app_adc_stop_output();
			}
			HAL_UART_MspDeInit(port->uart);
			port->task_handle = NULL;
			packet_free(port->phandle);
			vPortFree(usart_rx_dma_buffer);
			vTaskDelete(NULL);
			vTaskDelay(portMAX_DELAY);
		}

	}
}

void task_cli_init(port_str * port){
#if VESC_TOOL_ENABLE
	if(port->task_handle == NULL){
		xTaskCreate(task_cli, "tskCLI", 320, (void*)port, PRIO_NORMAL, &port->task_handle);
	}
#endif
}

void task_cli_kill(port_str * port){
	if(port->task_handle){
		xTaskNotify(port->task_handle, 0, eIncrement);
		vTaskDelay(200);
	}
}
