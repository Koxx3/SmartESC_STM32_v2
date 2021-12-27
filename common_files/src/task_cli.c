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

PACKET_STATE_t * phandle;

/**
 * \brief           Calculate length of statically allocated array
 */

#define CIRC_BUF_SZ       1024  /* must be power of two */
#define DMA_WRITE_PTR ( (CIRC_BUF_SZ - VESC_USART_DMA.hdmarx->Instance->CNDTR) & (CIRC_BUF_SZ - 1) )  //huart_cobs->hdmarx->Instance->NDTR.
uint8_t usart_rx_dma_buffer[CIRC_BUF_SZ];


TaskHandle_t task_cli_handle;


void putbuffer(unsigned char *buf, unsigned int len){
	HAL_UART_Transmit_DMA(&VESC_USART_DMA, buf, len);
	while(VESC_USART_TX_DMA.State != HAL_DMA_STATE_READY){
		VESC_USART_DMA.gState = HAL_UART_STATE_READY;
		vTaskDelay(1);
	}
}

void comm_uart_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, phandle);
}

void process_packet(unsigned char *data, unsigned int len){
	commands_process_packet(data, len, &comm_uart_send_packet);
}


void task_cli(void * argument)
{
	uint32_t rd_ptr=0;

	HAL_UART_Receive_DMA(&VESC_USART_DMA, usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));
	CLEAR_BIT(VESC_USART_DMA.Instance->CR3, USART_CR3_EIE);

	phandle = packet_init(putbuffer, process_packet);

	VescToSTM_set_brake_rel_int(0);

  /* Infinite loop */
	for(;;)
	{
		/* `#START TASK_LOOP_CODE` */
		while(rd_ptr != DMA_WRITE_PTR) {
			packet_process_byte(usart_rx_dma_buffer[rd_ptr], phandle);
			rd_ptr++;
			rd_ptr &= (CIRC_BUF_SZ - 1);
		}

		send_sample();
		send_position();
		VescToSTM_handle_timeout();
		vTaskDelay(1);
	}
}

void task_cli_init(){
#if VESC_TOOL_ENABLE
	xTaskCreate(task_cli, "tskCLI", 256, NULL, PRIO_NORMAL, &task_cli_handle);
#endif
}
