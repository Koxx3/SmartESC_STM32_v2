/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se
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

#ifndef APP_H_
#define APP_H_

#include "conf_general.h"

// Functions
app_configuration* app_get_configuration(void);
void app_set_configuration(app_configuration *conf);
void app_disable_output(int time_ms);
bool app_is_output_disabled(void);
unsigned app_calc_crc(app_configuration* conf);

// Standard apps
void task_app_init(port_str * port);
void task_app_kill();
void my_uart_send_data(unsigned char *buf, unsigned int len, port_str * port);
float app_adc_get_decoded_level(void);
float app_adc_get_decoded_level2(void);
void app_adc_stop_output(void);
void app_adc_set_mode(uint8_t mode_bit);
void app_adc_clear_mode(uint8_t mode_bit);
void app_adc_speed_mode(uint8_t speed);
#endif /* APP_H_ */
