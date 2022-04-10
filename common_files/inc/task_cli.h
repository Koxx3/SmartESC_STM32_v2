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

#ifndef TASK_CLI_H_
#define TASK_CLI_H_

//#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "main.h"
#include "task.h"
#include "stdbool.h"

//extern osThreadId_t task_cli_handle;

void cli_start_console();

typedef struct __PACKET_STATE_t__ PACKET_STATE_t;

typedef struct{
	UART_HandleTypeDef * uart;
	uint16_t rx_buffer_size;  //power of 2
	PACKET_STATE_t * phandle;
	bool half_duplex;
	TaskHandle_t task_handle;
} port_str;


void task_cli_init(port_str * port);
void task_cli_kill(port_str * port);

#endif /* TASK_LED_H_ */

