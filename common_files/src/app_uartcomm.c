/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "main.h"
#include "cmsis_os.h"
#include "packet.h"
#include "VescCommand.h"

#include <string.h>

// Settings
#define BAUDRATE					115200
#define PACKET_HANDLER				1
#define PACKET_HANDLER_P			2

osThreadId_t AppUsartHandle;
const osThreadAttr_t APPUSART_attributes = { .name = "APPUSART", .priority = osPriorityBelowNormal, .stack_size = 128 * 1 };

extern UART_HandleTypeDef APP_USART;
extern DMA_HandleTypeDef APP_USART_DMA_TX;

// Threads
void task_app_usart(void * argument);

// Variables
static volatile bool thread_is_running = false;
static volatile bool uart_is_running = false;

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, app_uartcomm_send_packet);
}

static void send_packet(unsigned char *data, unsigned int len) {
	if (uart_is_running) {
		//Waiting to send status OK
		while(HAL_DMA_GetState(&APP_USART_DMA_TX) == HAL_DMA_STATE_BUSY) osDelay(1);

		//send data
		while( HAL_UART_Transmit_DMA(&APP_USART, data, len) != HAL_OK ) osDelay(1);
	}
}

void app_uartcomm_start(void) {
	packet_init(send_packet, process_packet, PACKET_HANDLER);

	if (!thread_is_running) {
		AppUsartHandle = osThreadNew(task_app_usart, NULL, &APPUSART_attributes);
		thread_is_running = true;
	}
}

void app_uartcomm_start_permanent(void) {

}

void app_uartcomm_stop(void) {
	if (uart_is_running) {
		HAL_UART_DMAStop(&APP_USART);
		uart_is_running = false;
	}
}

void app_uartcomm_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PACKET_HANDLER);

}

void app_uartcomm_configure(uint32_t baudrate, bool permanent_enabled) {

	if (thread_is_running && uart_is_running) {
		//sdStart(&HW_UART_DEV, &uart_cfg);
	}
}

void task_app_usart(void * argument){
	(void)argument;

	for(;;) {
		bool rx = true;
		while (rx) {
			rx = false;
			if (uart_is_running) {
				//msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_IMMEDIATE);
				//if (res != MSG_TIMEOUT) {
					//packet_process_byte(res, PACKET_HANDLER);
					//rx = true;
				//}
			}
		}
		vTaskDelay(500);
	}
}
