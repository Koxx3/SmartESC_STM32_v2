/*
 * m365
 *
 * Copyright (c) 2021 Francois Deslandes
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

#include "task_pwr.h"
#include "task_init.h"
#include "main.h"
#include "task_cli.h"

osThreadId_t PwrHandle;
const osThreadAttr_t PWR_attributes = { .name = "PWR", .priority =
		(osPriority_t) osPriorityBelowNormal, .stack_size = 128 * 4 };

void poweroff(void) {
	HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_RESET);
}

void poweroffPressCheck(void) {
	if (HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin)) {
		uint16_t cnt_press = 0;
		while (HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin)) {
			HAL_Delay(10);
			cnt_press++;

#if TTERM_ENABLED
			if (cnt_press >= 4 * 100) {
				task_cli_init();
				cnt_press = 0;
				break;
			}
#endif
		}

		if (cnt_press >= 2 * 100) {
			poweroff();
		}
	}
}

void task_PWR(void *argument) {

	/* Infinite loop */
	for (;;) {
		poweroffPressCheck();
		osDelay(100);
	}
}

void task_PWR_init() {
	PwrHandle = osThreadNew(task_PWR, NULL, &PWR_attributes);
}

