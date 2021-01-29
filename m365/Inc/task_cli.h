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

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"

extern osThreadId_t task_cli_handle;

enum uart_mode{
	UART_MODE_ST,
	UART_MODE_CLI
};

struct __attribute__((__packed__)) _msg_format{
	uint8_t type;
	uint8_t destination;
	uint8_t n_esc;
	uint8_t bms_prot;
	uint8_t esc_jumps;
	uint8_t version_maj;
	uint8_t version_min;
	uint8_t power_on;
	uint8_t throttle;
	uint8_t brake;
	uint8_t torque_max;
	uint8_t brake_max;
	uint8_t lock;
	uint8_t regulator;
	uint8_t motor_direction;
	uint8_t hall_direction;
	uint8_t light;
	uint8_t temp_warn;
	uint8_t temp_max;
	uint8_t speed_lim;
	uint8_t start_speed;
	uint8_t crc;
};

struct __attribute__((__packed__)) _reply_format{
	uint8_t frame_start;
	uint8_t type;
	uint8_t version_maj;
	uint8_t version_min;
	uint8_t throttle;
	uint8_t brake;
	uint16_t voltage;
	int16_t current;
	int8_t temperature;
	int16_t rpm;
	uint8_t lock;
	uint8_t light;
	uint8_t regulator;
	int16_t phase_a_curr;
	int16_t phase_a_volt;
	uint8_t bms_version_maj;
	uint8_t bms_version_min;
	uint16_t bms_voltage;
	int16_t bms_current;
	uint8_t bms_cells[24];
	int8_t bms_temp_1;
	int8_t bms_temp_2;
	uint16_t bms_cycles_full;
	uint16_t bms_cycles_part;
	uint16_t bms_errors;
	uint8_t crc;
};

typedef struct _msg_format msg_format;
typedef struct _reply_format reply_format;

extern reply_format msg_rep;

extern enum uart_mode task_cli_mode;

extern StreamBufferHandle_t UART_RX;

void task_cli_init();


#endif /* TASK_LED_H_ */

