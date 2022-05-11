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
#include "VescCommand.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "product.h"
#include <math.h>
#include "tune.h"
#include "mcconf_default.h"
#include <string.h>
#include "main.h"

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

	MCI_StartMotor(pMCI[M1]);
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
	*(uint32_t*)data = (*(__IO uint32_t*)(ADDR_FLASH_PAGE_127+((x/4)*4)));
	return data[x%4];
}

uint8_t Flash_ReadByte_APP(uint32_t x){
	uint8_t data[4];
	*(uint32_t*)data = (*(__IO uint32_t*)(ADDR_FLASH_PAGE_126+((x/4)*4)));
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

	for (unsigned int i = 0;i < (sizeof(app_configuration));i++) {
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
		conf->modes_curr_scale[0] = MODE_SLOW_CURR;
		conf->modes_curr_scale[1] = MODE_DRIVE_CURR;
		conf->modes_curr_scale[2] = MODE_SPORT_CURR;
		conf->modes_kmh_limits[0] = MODE_SLOW_SPEED;
		conf->modes_kmh_limits[1] = MODE_DRIVE_SPEED;
		conf->modes_kmh_limits[2] = MODE_SPORT_SPEED;
	}

}

bool conf_general_write_flash(uint8_t page, uint8_t * data, uint16_t size){
	uint32_t word;
	uint8_t byte=0;
	uint8_t * word_ptr = (uint8_t*)&word;
	uint32_t flash_incr=0;

	HAL_FLASH_Unlock();

	uint32_t page_error = 0;
	FLASH_EraseInitTypeDef s_eraseinit;
	s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
	s_eraseinit.PageAddress = 0x08000000 + ((uint32_t)page*PAGE_SIZE);
	s_eraseinit.NbPages     = 1;
	HAL_FLASHEx_Erase(&s_eraseinit, &page_error);

	for (unsigned int i = 0;i < size;i++) {

		word_ptr[byte] = data[i];
		byte++;
		if(byte==4){
			byte=0;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, s_eraseinit.PageAddress+(flash_incr*4), *((uint32_t*)word_ptr));
			word=0;
			flash_incr++;
		}
	}
	if(byte!=0){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, s_eraseinit.PageAddress+(flash_incr*4), *((uint32_t*)word_ptr));
	}
	HAL_FLASH_Lock();
	return true;
}

/**
 * Write app_configuration to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_app_configuration(app_configuration *conf) {
	bool is_ok = true;
	VescToSTM_stop_motor();
	vTaskDelay(300);

	conf->crc = app_calc_crc(conf);


	is_ok = conf_general_write_flash(APP_PAGE, (uint8_t*)conf, sizeof(app_configuration));

	vTaskDelay(500);
	MCI_ExecTorqueRamp(pMCI[M1], 0, 0);
	VescToSTM_start_motor();

	return is_ok;
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

	conf->crc = conf_calc_crc(conf);

	HAL_FLASH_Unlock();

	is_ok = conf_general_write_flash(CONF_PAGE, (uint8_t*)conf, sizeof(mc_configuration));

	vTaskDelay(500);
	MCI_ExecTorqueRamp(pMCI[M1], 0, 0);
	VescToSTM_start_motor();

	return is_ok;
}

mc_configuration* mc_interface_get_configuration(void){
	return &mc_conf;
}

void conf_general_mcconf_hw_limits(mc_configuration *mcconf) {
	utils_truncate_number(&mcconf->l_current_max_scale, 0.0, 1.0);
	utils_truncate_number(&mcconf->l_current_min_scale, 0.0, 1.0);

	if(mcconf->override_limits == true) return;
	// This limit should always be active, as starving the threads never
	// makes sense.

#ifndef DISABLE_HW_LIMITS
#ifdef HW_LIM_CURRENT
	utils_truncate_number(&mcconf->l_current_max, HW_LIM_CURRENT);
	utils_truncate_number(&mcconf->l_current_min, HW_LIM_CURRENT);
#endif
#ifdef HW_LIM_CURRENT_IN
	utils_truncate_number(&mcconf->l_in_current_max, HW_LIM_CURRENT_IN);
	utils_truncate_number(&mcconf->l_in_current_min, HW_LIM_CURRENT);
#endif
#ifdef HW_LIM_CURRENT_ABS
	utils_truncate_number(&mcconf->l_abs_current_max, HW_LIM_CURRENT_ABS);
#endif
#ifdef HW_LIM_VIN
	utils_truncate_number(&mcconf->l_max_vin, HW_LIM_VIN);
	utils_truncate_number(&mcconf->l_min_vin, HW_LIM_VIN);
#endif
#ifdef HW_LIM_ERPM
	utils_truncate_number(&mcconf->l_max_erpm, HW_LIM_ERPM);
	utils_truncate_number(&mcconf->l_min_erpm, HW_LIM_ERPM);
#endif
#ifdef HW_LIM_DUTY_MIN
	utils_truncate_number(&mcconf->l_min_duty, HW_LIM_DUTY_MIN);
#endif
#ifdef HW_LIM_DUTY_MAX
	utils_truncate_number(&mcconf->l_max_duty, HW_LIM_DUTY_MAX);
#endif
#ifdef HW_LIM_TEMP_FET
	utils_truncate_number(&mcconf->l_temp_fet_start, HW_LIM_TEMP_FET);
	utils_truncate_number(&mcconf->l_temp_fet_end, HW_LIM_TEMP_FET);
#endif
#ifdef HW_LIM_F_SW
	utils_truncate_number(&mcconf->foc_f_sw, HW_LIM_F_SW);
#endif

#endif
}




void conf_general_setup_f_sw(uint32_t f_sw){

	uint32_t regulation_exec_rate = (f_sw > 20000) ? 2 : 1;

	uint16_t pwm_p_cycles = (uint16_t)((MOT_TMR_MHZ*(uint32_t)1000000u/((uint32_t)(f_sw)))&0xFFFE);

	PWM_Handle_M1._Super.hT_Sqrt3 	 = (pwm_p_cycles*SQRT3FACTOR)/16384u;
	PWM_Handle_M1._Super.PWMperiod   = pwm_p_cycles;
	PWM_Handle_M1.Half_PWMPeriod = pwm_p_cycles/2;
	HALL_M1._Super.hMeasurementFrequency = (uint16_t) ((uint32_t)(f_sw)/(regulation_exec_rate*PWM_FREQ_SCALING));
	RampExtMngrHFParamsM1.FrequencyHz = (uint32_t) ((uint32_t)(f_sw)/(regulation_exec_rate));
	R3_2_ParamsM1.RepetitionCounter = (uint16_t) ((regulation_exec_rate *2u)-1u);
	htim1.Init.Period = ((pwm_p_cycles) / 2);
	htim1.Init.RepetitionCounter = (uint16_t) ((regulation_exec_rate *2u)-1u);
	HAL_TIM_Base_Init(&htim1);
	HAL_TIM_PWM_Init(&htim1);
	TIM_OC_InitTypeDef sConfigOC = {0};

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
	Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
	Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = (((pwm_p_cycles) / 2) - (HTMIN));
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
	Error_Handler();
	};
}

void conf_general_setup_mc(mc_configuration *mcconf) {

	conf_general_setup_f_sw(mcconf->foc_f_sw);

	float current_max = mcconf->l_current_max * CURRENT_FACTOR_A * mcconf->l_current_max_scale;
	float current_min = mcconf->l_current_min * CURRENT_FACTOR_A * mcconf->l_current_min_scale;
	mcconf->lo_current_max_scale = 1.0;
	uint16_t max_app_speed;

	if(mcconf->foc_motor_flux_linkage==0){
		VescToSTM_set_minimum_current(0);  //PWM always ON
	}else{
		VescToSTM_set_minimum_current(mcconf->cc_min_current);
	}

	PWM_Handle_M1._Super.OverCurrent = mcconf->l_abs_current_max * CURRENT_FACTOR_A;

	if(mcconf->l_slow_abs_current){
		PWM_Handle_M1._Super.OverCurrentCycles = utils_map(mcconf->foc_current_filter_const,1,0,0,ABS_OVR_CURRENT_TRIP_MS) / (1000.0 / mcconf->foc_f_sw);
	}else{
		PWM_Handle_M1._Super.OverCurrentCycles = 0;  //instant trip
	}

	FOCVars[M1].max_i_batt = mcconf->l_in_current_max * CURRENT_FACTOR_A;
	FOCVars[M1].min_i_batt = mcconf->l_in_current_min * CURRENT_FACTOR_A;

	mcconf->lo_max_erpm = mcconf->l_max_erpm * mcconf->l_erpm_start;
	mcconf->lo_min_erpm = mcconf->l_min_erpm * mcconf->l_erpm_start;

	mcconf->lo_current_min = mcconf->l_current_min;
	mcconf->lo_current_max = mcconf->l_current_max;

	if(mcconf->l_max_erpm >= abs(mcconf->l_min_erpm)){
		max_app_speed = VescToSTM_erpm_to_speed(mcconf->l_max_erpm * 1.15);
	}else{
		max_app_speed = VescToSTM_erpm_to_speed(abs(mcconf->l_min_erpm * 1.15));
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
	PIDIqHandle_M1.hKiGain                = mcconf->foc_current_ki * (float)TF_KIDIV / (float)mcconf->foc_f_sw;
	PIDIqHandle_M1.hDefKpGain 			  = PIDIqHandle_M1.hKpGain;
	PIDIqHandle_M1.hDefKiGain 			  = PIDIqHandle_M1.hKiGain;
	PIDIqHandle_M1.hUpperOutputLimit	  = INT16_MAX * mcconf->l_max_duty;
	PIDIqHandle_M1.hLowerOutputLimit	  = -PIDIqHandle_M1.hUpperOutputLimit;
	PIDIqHandle_M1.wUpperIntegralLimit    = (int32_t)PIDIqHandle_M1.hUpperOutputLimit * TF_KIDIV,
	PIDIqHandle_M1.wLowerIntegralLimit    = (int32_t)-PIDIqHandle_M1.hUpperOutputLimit * TF_KIDIV,
	FOCVars[M1].min_duty				  = INT16_MAX *mcconf->l_min_duty;


	PIDIdHandle_M1.hKpGain             	  = PIDIqHandle_M1.hKpGain; //Torque and flux has the same P gain
	PIDIdHandle_M1.hKiGain                = PIDIqHandle_M1.hDefKiGain; //Torque and flux has the same I gain
	PIDIdHandle_M1.hDefKpGain 			  = PIDIdHandle_M1.hKpGain;
	PIDIdHandle_M1.hDefKiGain 			  = PIDIdHandle_M1.hKiGain;
	PIDIdHandle_M1.hUpperOutputLimit	  = INT16_MAX * mcconf->l_max_duty;
	PIDIdHandle_M1.hLowerOutputLimit	  = -PIDIdHandle_M1.hUpperOutputLimit;
	PIDIdHandle_M1.wUpperIntegralLimit    = (int32_t)PIDIdHandle_M1.hUpperOutputLimit * TF_KIDIV,
	PIDIdHandle_M1.wLowerIntegralLimit    = (int32_t)-PIDIdHandle_M1.hUpperOutputLimit * TF_KIDIV,

	FW_M1.wNominalSqCurr 				  = current_max * current_max;
	FW_M1.hFW_V_Ref						  = 1000.0 * mcconf->foc_fw_duty_start;
	FW_M1.hDemagCurrent					  = -(mcconf->foc_fw_current_max * CURRENT_FACTOR_A);
	FW_M1.fw_q_current_factor			  = mcconf->foc_fw_q_current_factor * INT16_MAX;
	PIDFluxWeakeningHandle_M1.wLowerIntegralLimit = (int32_t)FW_M1.hDemagCurrent * (int32_t)FW_KIDIV;

	SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit = VescToSTM_erpm_to_speed(mcconf->l_max_erpm * 1.15);
	SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit = VescToSTM_erpm_to_speed(mcconf->l_min_erpm * 1.15);
	SpeednTorqCtrlM1.MaxPositiveTorque			= current_max;
	SpeednTorqCtrlM1.MinNegativeTorque 			= current_min;

	HALL_M1._Super.hMaxReliableMecSpeedUnit     = max_app_speed;
	HALL_M1._Super.hMinReliableMecSpeedUnit     = VescToSTM_erpm_to_speed(mcconf->foc_hall_interp_erpm);
	HALL_M1._Super.bMaximumSpeedErrorsNumber    = MEAS_ERRORS_BEFORE_FAULTS;

	for(int i=0;i<8;i++){
		HALL_M1.lut[i] = mcconf->foc_hall_table[i];
	}
	HALL_M1.SwitchSpeed = VescToSTM_erpm_to_speed(mcconf->foc_hall_interp_erpm);

	VescToSTM_set_temp_cut(mcconf->l_temp_fet_start, mcconf->l_temp_fet_end);

	TempSensorParamsM1.hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT);
	TempSensorParamsM1.wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE);
	TempSensorParamsM1.hT0                     = T0_C;

	float temp = mcconf->l_min_vin * BATTERY_VOLTAGE_GAIN;
	utils_truncate_number(&temp, 0, UINT16_MAX);
	RealBusVoltageSensorParamsM1.UnderVoltageThreshold = mcconf->l_min_vin * BATTERY_VOLTAGE_GAIN;
	temp = mcconf->l_max_vin * BATTERY_VOLTAGE_GAIN;
	utils_truncate_number(&temp, 0, UINT16_MAX);
	RealBusVoltageSensorParamsM1.OverVoltageThreshold = mcconf->l_max_vin * BATTERY_VOLTAGE_GAIN;
	VescToSTM_set_battery_cut(mcconf->l_battery_cut_start, mcconf->l_battery_cut_end);

	// BLDC switching and drive
	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->sensor_mode = SENSOR_MODE_SENSORED;
	mcconf->pwm_mode = PWM_MODE_SYNCHRONOUS;
	mcconf->foc_sensor_mode = FOC_SENSOR_MODE_HALL;

	HALL_Init(&HALL_M1);

	VescToSTM_init_odometer(mcconf);
	mc_conf = *mcconf;

}

void conf_general_calc_apply_foc_cc_kp_ki_gain(mc_configuration *mcconf, float tc) {
	float r = mcconf->foc_motor_r;
	float l = mcconf->foc_motor_l;
	float lambda = mcconf->foc_motor_flux_linkage;

	float bw = 1.0 / (tc * 1e-6);
	float kp = l * bw;
	float ki = r * bw;
	float gain = 1.0e-3 / (lambda * lambda);

	mcconf->foc_current_kp = kp;
	mcconf->foc_current_ki = ki;
	mcconf->foc_observer_gain = gain * 1e6;
}

/**
 * Detect and apply all parameters, current limits and sensors. This is done for
 * both motors on dual controllers.
 *
 * @param max_power_loss
 * The maximum power loss to derive current limits, as well as detection currents, from.
 *
 * @param store_mcconf_on_success
 * Store motor configuration in emulated EEPROM if the detection succeeds.
 *
 * @param send_mcconf_on_success
 * Send motor configuration if the detection succeeds.
 *
 * @return
 * >=0: Success, see conf_general_autodetect_apply_sensors_foc codes
 * -10: Flux linkage detection failed
 *  -x: see conf_general_autodetect_apply_sensors_foc faults
 */
int conf_general_detect_apply_all_foc(float max_power_loss,	bool store_mcconf_on_success, bool send_mcconf_on_success, PACKET_STATE_t * phandle) {
	int result = -1;

	mc_configuration *mcconf = &mc_conf;
	mc_configuration *mcconf_old = pvPortMalloc(sizeof(mc_configuration));

	*mcconf_old = *mcconf;

	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
	mcconf->foc_f_sw = 16000.0;
	mcconf->foc_current_kp = 0.0005;
	mcconf->foc_current_ki = 1.0;
	mcconf->l_current_max = MCCONF_L_CURRENT_MAX;
	mcconf->l_current_min = MCCONF_L_CURRENT_MIN;
	mcconf->l_current_max_scale = MCCONF_L_CURRENT_MAX_SCALE;
	mcconf->l_current_min_scale = MCCONF_L_CURRENT_MIN_SCALE;
	mcconf->l_watt_max = MCCONF_L_WATT_MAX;
	mcconf->l_watt_min = MCCONF_L_WATT_MIN;
	mcconf->l_max_erpm = MCCONF_L_RPM_MAX;
	mcconf->l_min_erpm = MCCONF_L_RPM_MIN;
	//conf_general_setup_mc(mcconf);


	// Wait one second for things to get ready after
	// the fault disappears.
	vTaskDelay(MS_TO_TICKS(1000));

	// Disable timeout
	VescToSTM_enable_timeout(false);


	float r = 0.0;
	float l = 0.0;
	float i_max = 0.0;
	bool res_r_l_imax_m1 = tune_foc_measure_r_l_imax(mcconf->cc_min_current,
			mcconf->l_current_max, max_power_loss, &r, &l, &i_max);

	if (!res_r_l_imax_m1) {
		conf_general_setup_mc(mcconf_old);
		vPortFree(mcconf_old);
		return -11;
	}

	float lambda = 0.0;
	float lambda_undriven = 0.0;
	float lambda_undriven_samples = 0.0;
	int res = tune_foc_measure_flux_linkage_openloop(i_max / 2.5, 0.3, 1800, r, l,
			&lambda, &lambda_undriven, &lambda_undriven_samples);

	if (lambda_undriven_samples > 60) {
		lambda = lambda_undriven;
	}


	if (res) {
		mcconf_old->l_current_max = i_max;
		mcconf_old->l_current_min = -i_max;
		mcconf_old->motor_type = MOTOR_TYPE_FOC;
		mcconf_old->foc_motor_r = r;
		mcconf_old->foc_motor_l = l;
		mcconf_old->foc_motor_flux_linkage = lambda;
		VescToSTM_stop_motor();
		conf_general_calc_apply_foc_cc_kp_ki_gain(mcconf_old, 1000);
		conf_general_setup_mc(mcconf_old);


		// TODO: optionally apply temperature compensation here.

		vTaskDelay(MS_TO_TICKS(1000));
		//wait_motor_stop(10000);
		VescToSTM_start_motor();

		// This will also store the settings to emulated eeprom and send them to vesc tool

		uint8_t hall_tab[8];
		bool res = tune_mcpwm_foc_hall_detect((i_max / 3.0), hall_tab);
		if(res==true){
			memcpy(mcconf_old->foc_hall_table,hall_tab, 8);
			conf_general_setup_mc(mcconf_old);
			commands_send_mcconf(COMM_GET_MCCONF, mcconf_old, phandle);
			conf_general_store_mc_configuration(mcconf_old, false);
			result = 1;
		}

	} else {
		result = -10;
	}

	VescToSTM_enable_timeout(true);

	*mcconf = *mcconf_old;
	if (result < 0) {
		conf_general_setup_mc(mcconf);
	}

	vPortFree(mcconf_old);

	return result;
}

/**
 * Same as conf_general_detect_apply_all_foc, but also start detection in VESCs found on the CAN-bus.
 *
 * @param detect_can
 * Run detection on VESCs found on the CAN-bus as well. Setting this to false makes
 * this function behave like conf_general_detect_apply_all_foc, with the convenience
 * of also applying the settings.
 *
 * @param max_power_loss
 * The maximum power loss to derive current limits, as well as detection currents, from.
 *
 * @param min_current_in
 * Minimum input current (negative value). 0 means leave it unchanged.
 *
 * @param max_current_in
 * MAximum input current. 0 means leave it unchanged.
 *
 * @param openloop_rpm
 * FOC openloop ERPM in sensorless mode. 0 means leave it unchanged.
 *
 * @param sl_erpm
 * FOC ERPM above which sensorless should be used in sensored modes. 0 means leave it unchanged.
 *
 * @return
 * Same as conf_general_detect_apply_all_foc, and
 * -50: CAN detection timed out
 * -51: CAN detection failed
 */
int conf_general_detect_apply_all_foc_can(bool detect_can, float max_power_loss,
		float min_current_in, float max_current_in, float openloop_rpm, float sl_erpm, PACKET_STATE_t * phandle) {


	int res = conf_general_detect_apply_all_foc(max_power_loss, false, false, phandle);

	return res;
}
