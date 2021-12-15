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
#include "FreeRTOS.h"
#include "task.h"
#include "product.h"
#include "ninebot.h"
#include "VescCommand.h"


NinebotPack frame;

//static uint8_t	ui8_tx_buffer[] = {0x55, 0xAA, 0x08, 0x21, 0x64, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t m365_mode[3] = { M365_MODE_SLOW, M365_MODE_DRIVE, M365_MODE_SPORT};

m365Answer m365_to_display = {.start1=NinebotHeader0, .start2=NinebotHeader1, .len=8, .addr=0x21, .cmd=0x64, .arg=0, .mode=1};


#define CIRC_BUF_SZ       				32  /* must be power of two */
#define DMA_WRITE_PTR(channel) 			((CIRC_BUF_SZ - channel.hdmarx->Instance->CNDTR) & (CIRC_BUF_SZ - 1) )  //huart_cobs->hdmarx->Instance->NDTR.

uint8_t usart_rx_dma_buffer[CIRC_BUF_SZ];
#ifdef G30P
uint8_t usart2_rx_dma_buffer[CIRC_BUF_SZ];
#endif

uint8_t app_connection_timout = 8;

TaskHandle_t task_app_handle;

void my_uart_send_data(uint8_t *tdata, uint16_t tnum){
	//send data
	//HAL_HalfDuplex_EnableTransmitter(&APP_USART_DMA);
	while( HAL_UART_Transmit_DMA(&APP_USART_DMA, tdata, tnum) != HAL_OK ) vTaskDelay(1);

	//Waiting to send status OK
	while(HAL_DMA_GetState(&APP_USART_TX_DMA) != HAL_DMA_STATE_READY){
		APP_USART_DMA.gState = HAL_UART_STATE_READY;
		vTaskDelay(1);
	}
	//HAL_HalfDuplex_EnableReceiver(&APP_USART_DMA);
}

#ifdef G30P
void my_uart2_send_data(uint8_t *tdata, uint16_t tnum){
	//send data
	while( HAL_UART_Transmit_DMA(&APP2_USART_DMA, tdata, tnum) != HAL_OK ) osDelay(1);

	//Waiting to send status OK
	while(HAL_DMA_GetState(&APP2_USART_TX_DMA) != HAL_DMA_STATE_READY){
		APP2_USART_DMA.gState = HAL_UART_STATE_READY;
		osDelay(1);
	}
}
#endif

void task_app(void * argument)
{
	HAL_UART_Receive_DMA(&APP_USART_DMA, usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));
	CLEAR_BIT(APP_USART_DMA.Instance->CR3, USART_CR3_EIE);

#ifdef G30P
	HAL_UART_Receive_DMA(&APP2_USART_DMA, usart2_rx_dma_buffer, sizeof(usart2_rx_dma_buffer));
	CLEAR_BIT(APP2_USART_DMA.Instance->CR3, USART_CR3_EIE);
#endif

	uint32_t rd_ptr=0;
#ifdef G30P
	uint32_t rd2_ptr=0;
#endif
	uint16_t slow_update_cnt=0;
  /* Infinite loop */
	for(;;)
	{
		while(rd_ptr != DMA_WRITE_PTR(APP_USART_DMA)) {
			if(ninebot_parse(usart_rx_dma_buffer[rd_ptr] ,&frame)	==0){
					//commands_printf("LEN: %d CMD: %x ARG: %x", frame.len, frame.cmd, frame.arg);
				switch(frame.cmd){
					case 0x64:
						addCRC((uint8_t*)&m365_to_display, m365_to_display.len+6);
						my_uart_send_data((uint8_t*)&m365_to_display, sizeof(m365_to_display));
					break;
					case 0x65:


					break;
				}
			}
			rd_ptr++;
			rd_ptr &= (CIRC_BUF_SZ - 1);
		}

		if(slow_update_cnt==0){
			m365_to_display.speed = VescToSTM_get_speed()*3.6;
			m365_to_display.battery = utils_map(VescToSTM_get_battery_level(0), 0, 1, 0, 96);
			m365_to_display.beep=0;

		}else{
			slow_update_cnt++;
			if(slow_update_cnt==50) slow_update_cnt=0;
		}


#ifdef G30P
		while(rd2_ptr != DMA_WRITE_PTR(APP2_USART_DMA)) {
			rd2_ptr++;
			rd2_ptr &= (CIRC_BUF_SZ - 1);
		}
#endif

		vTaskDelay(10);
	}
}

void task_app_init(){
	xTaskCreate(task_app, "APP-USART", 128, NULL, PRIO_BELOW_NORMAL, &task_app_handle);
}
