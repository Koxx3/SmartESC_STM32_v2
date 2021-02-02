/*
 * UD3
 *
 * Copyright (c) 2018 Jens Kerrinnes
 * Copyright (c) 2015 Steve Ward
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

#ifndef CLI_COMMON_H
#define CLI_COMMON_H

#include "cli_basic.h"

/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "TTerm.h"


void init_config();
void eeprom_load(TERMINAL_HANDLE * handle);
void eeprom_save(TERMINAL_HANDLE * handle);

uint8_t CMD_signals(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_con(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_config_get(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_eeprom(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_load_defaults(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_get(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_set(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);
uint8_t CMD_ack(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

uint8_t CMD_tune(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

uint8_t CMD_start(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

uint8_t CMD_stop(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

uint8_t CMD_reset(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args);

int16_t pCMD_calculate_curr_8(int16_t torque_percent);

uint8_t TERM_eeprom_read();

extern parameter_entry confparam[];

struct config_struct{
    uint16_t conf1;
    float conf2;
    int32_t conf3;
};
typedef struct config_struct cli_config;


typedef struct parameter_struct cli_parameter;

extern cli_config configuration;


#define CONF_SIZE sizeof(confparam) / sizeof(parameter_entry)

#endif
