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

#include <stdlib.h>
#include "conf_general.h"
#include "mc_interface.h"
#include "utils.h"
#include "confgenerator.h"
#include "stm32f1xx_hal.h"
#include "crc.h"
#include "defines.h"
#include "drive_parameters.h"
#include "mc_config.h"
#include "VescToSTM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "product.h"

mc_configuration mc_conf;
app_configuration appconf;


void conf_general_init(void) {
	conf_general_read_app_configuration(&appconf);
	app_set_configuration(&appconf);

	conf_general_read_mc_configuration(&mc_conf, 0);
	conf_general_setup_mc(&mc_conf);
	
	//enable cycle counter
	DEMCR |= DEMCR_TRCENA;
	DWT_CTRL |= CYCCNTENA;

}

/**
 * Get mc_configuration CRC (motor 1 or 2)
 *
 * @param conf
 * Pointer to mc_configuration or NULL for current config
 *
 * @param is_motor_2
 * true if motor2, false if motor1
 *
 * @return
 * CRC16 (with crc field in struct temporarily set to zero).
 */
unsigned conf_calc_crc(mc_configuration* conf_in) {
	volatile mc_configuration* conf = conf_in;

	unsigned crc_old = conf->crc;
	conf->crc = 0;
	unsigned crc_new = crc16((uint8_t*)conf, sizeof(mc_configuration));
	conf->crc = crc_old;
	return crc_new;
}

uint8_t Flash_ReadByte_MC(uint32_t x){
	uint8_t data[4];
	*(uint32_t*)data = (*(__IO uint32_t*)(ADDR_FLASH_PAGE_63+((x/4)*4)));
	return data[x%4];
}

uint8_t Flash_ReadByte_APP(uint32_t x){
	uint8_t data[4];
	*(uint32_t*)data = (*(__IO uint32_t*)(ADDR_FLASH_PAGE_62+((x/4)*4)));
	return data[x%4];
}

/**
 * Read app_configuration from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a app_configuration struct to write the read configuration to.
 */
void conf_general_read_app_configuration(app_configuration *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;

	for (unsigned int i = 0;i < (sizeof(mc_configuration));i++) {
		conf_addr[i] = Flash_ReadByte_APP(i);
	}

	// check CRC
#ifdef TEST_BAD_APP_CRC
	conf->crc++;
#endif
	if(conf->crc != app_calc_crc(conf)) {
		is_ok = false;
//		mc_interface_fault_stop(FAULT_CODE_FLASH_CORRUPTION_APP_CFG, false, false);
		//fault_data f;
		//f.fault = FAULT_CODE_FLASH_CORRUPTION_APP_CFG;
		//terminal_add_fault_data(&f);
	}

	// Set the default configuration
	if (!is_ok) {
		confgenerator_set_defaults_appconf(conf);
	}
}

/**
 * Read mc_configuration from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a mc_configuration struct to write the read configuration to.
 */
void conf_general_read_mc_configuration(mc_configuration *conf, bool is_motor_2) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	//unsigned int base = is_motor_2 ? EEPROM_BASE_MCCONF_2 : EEPROM_BASE_MCCONF;

	for (unsigned int i = 0;i < (sizeof(mc_configuration));i++) {
		conf_addr[i] = Flash_ReadByte_MC(i);
	}

	if(conf->crc != conf_calc_crc(conf)) {
		is_ok = false;
//		mc_interface_fault_stop(FAULT_CODE_FLASH_CORRUPTION_MC_CFG, is_motor_2, false);
		//fault_data f;
		//f.fault = FAULT_CODE_FLASH_CORRUPTION_MC_CFG;
		//terminal_add_fault_data(&f);
	}

	if (!is_ok) {
		confgenerator_set_defaults_mcconf(conf);
	}

}

/**
 * Write mc_configuration to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_mc_configuration(mc_configuration *conf, bool is_motor_2) {
	VescToSTM_stop_motor();
	vTaskDelay(300);

	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;

	uint32_t flash_incr=0;
	uint8_t byte=0;
	uint32_t word;
	uint8_t * word_ptr = (uint8_t*)&word;

	conf->crc = conf_calc_crc(conf);

	HAL_FLASH_Unlock();


	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef s_eraseinit;
	s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
	s_eraseinit.PageAddress = ADDR_FLASH_PAGE_63;
	s_eraseinit.NbPages     = 1;
	HAL_FLASHEx_Erase(&s_eraseinit, &page_error);

	for (unsigned int i = 0;i < sizeof(mc_configuration);i++) {

		word_ptr[byte] = conf_addr[i];
		byte++;
		if(byte==4){
			byte=0;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_PAGE_63+(flash_incr*4), *((uint32_t*)word_ptr));
			word=0;
			flash_incr++;
		}
	}
	if(byte!=0){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_PAGE_63+(flash_incr*4), *((uint32_t*)word_ptr));
	}
	HAL_FLASH_Lock();
	vTaskDelay(500);
	VescToSTM_start_motor();
	vTaskDelay(100);
	VescToSTM_set_torque(0);

	return is_ok;
}

mc_configuration* mc_interface_get_configuration(void){
	return &mc_conf;
}



void conf_general_setup_mc(mc_configuration *mcconf) {
	float current_max = mcconf->l_current_max * CURRENT_FACTOR_A;
	float current_min = mcconf->l_current_min * CURRENT_FACTOR_A;
	uint16_t max_app_speed;
	if(mcconf->l_max_erpm >= abs(mcconf->l_min_erpm)){
		max_app_speed = VescToSTM_erpm_to_speed(mcconf->l_max_erpm * 1.5, mcconf->si_motor_poles);
	}else{
		max_app_speed = VescToSTM_erpm_to_speed(abs(mcconf->l_min_erpm * 1.5), mcconf->si_motor_poles);
	}


	PIDSpeedHandle_M1.hKpGain             = mcconf->s_pid_kp * (float)SP_KPDIV;
	PIDSpeedHandle_M1.hKiGain         	  = mcconf->s_pid_ki * (float)SP_KIDIV / (float)SPEED_LOOP_FREQUENCY_HZ;
	PIDSpeedHandle_M1.wUpperIntegralLimit = current_max * SP_KIDIV;
	PIDSpeedHandle_M1.wLowerIntegralLimit = current_min * SP_KIDIV;
	PIDSpeedHandle_M1.hUpperOutputLimit   = current_max;
	PIDSpeedHandle_M1.hLowerOutputLimit   = current_min;
	PIDSpeedHandle_M1.hDefKpGain 		  = PIDSpeedHandle_M1.hKpGain;
	PIDSpeedHandle_M1.hDefKiGain 		  = PIDSpeedHandle_M1.hKiGain;

	PIDIqHandle_M1.hKpGain          	  = mcconf->foc_current_kp * (float)TF_KPDIV;
	PIDIqHandle_M1.hKiGain                = mcconf->foc_current_ki * (float)TF_KIDIV / (float)PWM_FREQUENCY;
	PIDIqHandle_M1.hDefKpGain 			  = PIDIqHandle_M1.hKpGain;
	PIDIqHandle_M1.hDefKiGain 			  = PIDIqHandle_M1.hKiGain;

	PIDIdHandle_M1.hKpGain             	  = PIDIqHandle_M1.hKpGain; //Torque and flux has the same P gain
	PIDIdHandle_M1.hKiGain                = PIDIqHandle_M1.hDefKiGain; //Torque and flux has the same I gain
	PIDIdHandle_M1.hDefKpGain 			  = PIDIdHandle_M1.hKpGain;
	PIDIdHandle_M1.hDefKiGain 			  = PIDIdHandle_M1.hKiGain;

	SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit = VescToSTM_erpm_to_speed(mcconf->l_max_erpm * 1.15, mcconf->si_motor_poles);
	SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit = VescToSTM_erpm_to_speed(mcconf->l_min_erpm * 1.15, mcconf->si_motor_poles);
	SpeednTorqCtrlM1.MaxPositiveTorque			= current_max;
	SpeednTorqCtrlM1.MinNegativeTorque 			= current_min;

	HALL_M1._Super.bElToMecRatio                = mcconf->si_motor_poles;
	HALL_M1._Super.hMaxReliableMecSpeedUnit     = max_app_speed;
	HALL_M1._Super.bMaximumSpeedErrorsNumber    = MEAS_ERRORS_BEFORE_FAULTS;
	HALL_M1.PhaseShift          				= DEG_TO_ANG(mcconf->foc_encoder_offset);
	for(int i=0;i<8;i++){
		HALL_M1.lut[i] = mcconf->foc_hall_table[i];
	}
	HALL_M1.SwitchSpeed = VescToSTM_erpm_to_speed(mcconf->foc_hall_interp_erpm, mcconf->si_motor_poles);

	TempSensorParamsM1.hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d);
	TempSensorParamsM1.hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d);
	TempSensorParamsM1.hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT);
	TempSensorParamsM1.wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE);
	TempSensorParamsM1.hT0                     = T0_C;

	//RealBusVoltageSensorParamsM1.UnderVoltageThreshold = mcconf->l_min_vin * BATTERY_VOLTAGE_GAIN;
	//RealBusVoltageSensorParamsM1.OverVoltageThreshold = mcconf->l_max_vin * BATTERY_VOLTAGE_GAIN;


	// BLDC switching and drive
	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->sensor_mode = SENSOR_MODE_SENSORED;
	mcconf->pwm_mode = PWM_MODE_SYNCHRONOUS;
	mcconf->foc_sensor_mode = FOC_SENSOR_MODE_HALL;

	HALL_Init(&HALL_M1);
	VescToSTM_init_odometer(mcconf);
	mc_conf = *mcconf;
}
