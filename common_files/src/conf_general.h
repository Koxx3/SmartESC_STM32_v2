/*
	Copyright 2017 - 2021 Benjamin Vedder	benjamin@vedder.se

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

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

// Firmware version
#define FW_VERSION_MAJOR			5
#define FW_VERSION_MINOR			02
// Set to 0 for building a release and iterate during beta test builds
#define FW_TEST_VERSION_NUMBER		0

#include "VescDatatypes.h"
#include "packet.h"


#define APP_PAGE				126
#define CONF_PAGE				127

#define ADDR_FLASH_PAGE_126    ((uint32_t)0x08000000+(APP_PAGE*0x400)) /* Base @ of Page 126, 1 Kbytes */
#define ADDR_FLASH_PAGE_127    ((uint32_t)0x08000000+(CONF_PAGE*0x400)) /* Base @ of Page 127, 1 Kbytes */

extern mc_configuration mc_conf;
extern app_configuration appconf;

// Functions
void conf_general_init(void);
void conf_general_read_app_configuration(app_configuration *conf);
void conf_general_read_mc_configuration(mc_configuration *conf, bool is_motor_2);
bool conf_general_store_mc_configuration(mc_configuration *conf, bool is_motor_2);
void conf_general_setup_mc(mc_configuration *mcconf);
void conf_general_update_current(mc_configuration *mcconf);
mc_configuration* mc_interface_get_configuration(void);
bool conf_general_store_app_configuration(app_configuration *conf);
void conf_general_mcconf_hw_limits(mc_configuration *mcconf);
int conf_general_detect_apply_all_foc_can(bool detect_can, float max_power_loss, float min_current_in, float max_current_in, float openloop_rpm, float sl_erpm, PACKET_STATE_t * phandle);
#endif /* CONF_GENERAL_H_ */
