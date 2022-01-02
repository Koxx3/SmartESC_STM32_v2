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

#include "task_init.h"
#include "task_LED.h"
#include "task_pwr.h"
#include "product.h"

unsigned long getRunTimeCounterValue(void){
	return HAL_GetTick();
}

port_str main_uart = {	.uart = &VESC_USART_DMA,
					    .rx_buffer_size = 512,
						.phandle = NULL,
						.half_duplex = false,
						.task_handle = NULL
};
port_str aux_uart = {	.uart = &APP_USART_DMA,
					    .rx_buffer_size = 128,
						.phandle = NULL,
						.half_duplex = true,
						.task_handle = NULL
};

void task_init(){
	task_cli_init(&main_uart);
	task_LED_init(&main_uart);  //Bring up the blinky
	task_PWR_init(&main_uart);  //Manage power button
}
