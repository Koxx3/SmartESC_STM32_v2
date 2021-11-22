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
#include "product.h"



#define CIRC_BUF_SZ       				32  /* must be power of two */
#define DMA_WRITE_PTR(channel) 			((CIRC_BUF_SZ - channel.hdmarx->Instance->CNDTR) & (CIRC_BUF_SZ - 1) )  //huart_cobs->hdmarx->Instance->NDTR.

uint8_t usart_rx_dma_buffer[CIRC_BUF_SZ];
uint8_t usart2_rx_dma_buffer[CIRC_BUF_SZ];

uint8_t app_connection_timout = 8;

osThreadId_t task_app_handle;
const osThreadAttr_t task_app_attributes = {
  .name = "APP-USART",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 7
};

void my_uart_send_data(uint8_t *tdata, uint16_t tnum){
	//send data
	while( HAL_UART_Transmit_DMA(&APP_USART_DMA, tdata, tnum) != HAL_OK ) osDelay(1);

	//Waiting to send status OK
	while(HAL_DMA_GetState(&APP_USART_TX_DMA) != HAL_DMA_STATE_READY){
		APP_USART_DMA.gState = HAL_UART_STATE_READY;
		osDelay(1);
	}
}

void my_uart2_send_data(uint8_t *tdata, uint16_t tnum){
	//send data
	while( HAL_UART_Transmit_DMA(&APP2_USART_DMA, tdata, tnum) != HAL_OK ) osDelay(1);

	//Waiting to send status OK
	while(HAL_DMA_GetState(&APP2_USART_TX_DMA) != HAL_DMA_STATE_READY){
		APP2_USART_DMA.gState = HAL_UART_STATE_READY;
		osDelay(1);
	}
}

void task_app(void * argument)
{
	HAL_UART_Receive_DMA(&APP_USART_DMA, usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));
	CLEAR_BIT(APP_USART_DMA.Instance->CR3, USART_CR3_EIE);

	HAL_UART_Receive_DMA(&APP2_USART_DMA, usart2_rx_dma_buffer, sizeof(usart2_rx_dma_buffer));
	CLEAR_BIT(APP2_USART_DMA.Instance->CR3, USART_CR3_EIE);

	uint32_t rd_ptr=0;
	uint32_t rd2_ptr=0;

  /* Infinite loop */
	for(;;)
	{
		while(rd_ptr != DMA_WRITE_PTR(APP_USART_DMA)) {
			rd_ptr++;
			rd_ptr &= (CIRC_BUF_SZ - 1);
		}

		while(rd2_ptr != DMA_WRITE_PTR(APP2_USART_DMA)) {
			rd2_ptr++;
			rd2_ptr &= (CIRC_BUF_SZ - 1);
		}

		osDelay(10);
	}
}

void task_app_init(){
	task_app_handle = osThreadNew(task_app, NULL, &task_app_attributes);
}
