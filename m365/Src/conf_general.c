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

mc_configuration mc_conf;


void conf_general_init(void) {
	conf_general_read_mc_configuration(&mc_conf, 0);
	conf_general_setup_mc(&mc_conf);

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
unsigned conf_calc_crc(mc_configuration* conf_in, bool is_motor_2) {
	volatile mc_configuration* conf = conf_in;

	unsigned crc_old = conf->crc;
	conf->crc = 0;
	unsigned crc_new = crc16((uint8_t*)conf, sizeof(mc_configuration));
	conf->crc = crc_old;
	return crc_new;
}

uint8_t Flash_ReadByte(uint32_t x){
	uint8_t data[4];
	*(uint32_t*)data = (*(__IO uint32_t*)(ADDR_FLASH_PAGE_63+((x/4)*4)));
	return data[x%4];
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
		conf_addr[i] = Flash_ReadByte(i);
	}

	// check CRC
#ifdef TEST_BAD_MC_CRC
	conf->crc++;
#endif
	if(conf->crc != conf_calc_crc(conf, is_motor_2)) {
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

	conf->crc = conf_calc_crc(conf, is_motor_2);

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
	VescToSTM_set_torque(0);

	return is_ok;
}

void conf_general_recalc(){
	PIDSpeedHandle_M1.wUpperIntegralLimit = (uint32_t)SpeednTorqCtrlM1.MaxPositiveTorque * SP_KDDIV;
	PIDSpeedHandle_M1.wLowerIntegralLimit = (uint32_t)SpeednTorqCtrlM1.MinNegativeTorque * SP_KDDIV;
	PIDSpeedHandle_M1.hUpperOutputLimit = SpeednTorqCtrlM1.MaxPositiveTorque;
	PIDSpeedHandle_M1.hLowerOutputLimit = SpeednTorqCtrlM1.MinNegativeTorque;
	if(SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit>SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit){
		HALL_M1._Super.hMaxReliableMecSpeedUnit = (uint16_t)(1.15*SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit);
	}else{
		HALL_M1._Super.hMaxReliableMecSpeedUnit = -(uint16_t)(1.15*SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit);
	}
	PIDIqHandle_M1.hDefKdGain = PIDIqHandle_M1.hKpGain;
	PIDIqHandle_M1.hDefKiGain = PIDIqHandle_M1.hKiGain;
	PIDIdHandle_M1.hDefKdGain = PIDIdHandle_M1.hKpGain;
	PIDIdHandle_M1.hDefKiGain = PIDIdHandle_M1.hKiGain;
}

mc_configuration* mc_interface_get_configuration(void){
	return &mc_conf;
}

void conf_general_setup_mc(mc_configuration *mcconf) {

	// Limits
		SpeednTorqCtrlM1.MaxPositiveTorque = mcconf->l_current_max * CURRENT_FACTOR;
		 SpeednTorqCtrlM1.MinNegativeTorque = mcconf->l_current_min * CURRENT_FACTOR;
//		float l_in_current_max;
//		float l_in_current_min;
		mcconf->l_abs_current_max = 60;
		SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit = (mcconf->l_min_erpm / (float)HALL_M1._Super.bElToMecRatio);
		SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit = (mcconf->l_max_erpm / (float)HALL_M1._Super.bElToMecRatio);
		HALL_M1._Super.hMinReliableMecSpeedUnit = (mcconf->l_min_erpm * 1.15 / (float)HALL_M1._Super.bElToMecRatio);
		HALL_M1._Super.hMaxReliableMecSpeedUnit = (mcconf->l_max_erpm * 1.15 / (float)HALL_M1._Super.bElToMecRatio);
		HALL_Init(&HALL_M1);

//		float l_erpm_start;
//		float l_max_erpm_fbrake;
//		float l_max_erpm_fbrake_cc;
		RealBusVoltageSensorParamsM1.UnderVoltageThreshold = mcconf->l_min_vin * VOLT_SCALING;
		RealBusVoltageSensorParamsM1.OverVoltageThreshold = mcconf->l_max_vin * VOLT_SCALING;
		//mcconf->l_battery_cut_end = RealBusVoltageSensorParamsM1.UnderVoltageThreshold / VOLT_SCALING;
		//mcconf->l_battery_cut_start = RealBusVoltageSensorParamsM1.OverVoltageThreshold / VOLT_SCALING;
//		bool l_slow_abs_current;
//		float l_temp_fet_start;
//		float l_temp_fet_end;
//		float l_temp_motor_start;
//		float l_temp_motor_end;
//		float l_temp_accel_dec;
//		float l_min_duty;
//		float l_max_duty;
//		float l_watt_max;
//		float l_watt_min;
//		float l_current_max_scale;
//		float l_current_min_scale;
//		float l_duty_start;
//		// Overridden limits (Computed during runtime)
//		float lo_current_max;
//		float lo_current_min;
//		float lo_in_current_max;
//		float lo_in_current_min;
//		float lo_current_motor_max_now;
//		float lo_current_motor_min_now;



	// Hall sensor
	//for(int i=0;i<8;i++){
	//	mcconf->hall_table[i] = HALL_M1.lut[i];
	//}

	// BLDC switching and drive
	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->sensor_mode = SENSOR_MODE_SENSORED;
	mcconf->pwm_mode = PWM_MODE_SYNCHRONOUS;

	// FOC
	PIDIqHandle_M1.hKpGain = mcconf->foc_current_kp * 100;
    PIDIqHandle_M1.hKiGain = mcconf->foc_current_ki * 100;
//	float foc_f_sw;
//	float foc_dt_us;
//	float foc_encoder_offset;
//	bool foc_encoder_inverted;
//	float foc_encoder_ratio;
//	float foc_encoder_sin_offset;
//	float foc_encoder_sin_gain;
//	float foc_encoder_cos_offset;
//	float foc_encoder_cos_gain;
//	float foc_encoder_sincos_filter_constant;
//	float foc_motor_l;
//	float foc_motor_ld_lq_diff;
//	float foc_motor_r;
//	float foc_motor_flux_linkage;
//	float foc_observer_gain;
//	float foc_observer_gain_slow;
//	float foc_pll_kp;
//	float foc_pll_ki;
//	float foc_duty_dowmramp_kp;
//	float foc_duty_dowmramp_ki;
//	float foc_openloop_rpm;
//	float foc_openloop_rpm_low;
//	float foc_d_gain_scale_start;
//	float foc_d_gain_scale_max_mod;
//	float foc_sl_openloop_hyst;
//	float foc_sl_openloop_time;
//	float foc_sl_openloop_time_lock;
//	float foc_sl_openloop_time_ramp;
	mcconf->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
	for(int i=0;i<8;i++){
		 HALL_M1.lut[i] = mcconf->foc_hall_table[i];
	}
	HALL_M1.SwitchSpeed = (float)mcconf->foc_hall_interp_erpm / (float)HALL_M1._Super.bElToMecRatio;
//	float foc_sl_erpm;
//	bool foc_sample_v0_v7;
//	bool foc_sample_high_current;
//	float foc_sat_comp;
//	bool foc_temp_comp;
//	float foc_temp_comp_base_temp;
//	float foc_current_filter_const;
//	mc_foc_cc_decoupling_mode foc_cc_decoupling;
//	mc_foc_observer_type foc_observer_type;
//	float foc_hfi_voltage_start;
//	float foc_hfi_voltage_run;
//	float foc_hfi_voltage_max;
//	float foc_sl_erpm_hfi;
//	uint16_t foc_hfi_start_samples;
//	float foc_hfi_obs_ovr_sec;
//	uint8_t foc_hfi_samples;

	// GPDrive
//	int gpd_buffer_notify_left;
//	int gpd_buffer_interpol;
//	float gpd_current_filter_const;
//	float gpd_current_kp;
//	float gpd_current_ki;

	// Speed PID
//	float s_pid_kp;
//	float s_pid_ki;
//	float s_pid_kd;
//	float s_pid_kd_filter;
//	float s_pid_min_erpm;
//	bool s_pid_allow_braking;
//	float s_pid_ramp_erpms_s;

	// Pos PID
//	float p_pid_kp;
//	float p_pid_ki;
//	float p_pid_kd;
//	float p_pid_kd_filter;
//	float p_pid_ang_div;

	// Current controller
//	float cc_startup_boost_duty;
//	float cc_min_current;
//	float cc_gain;
//	float cc_ramp_step_max;

	// Misc
//	int32_t m_fault_stop_time_ms;
//	float m_duty_ramp_step;
//	float m_current_backoff_gain;
//	uint32_t m_encoder_counts;
//	sensor_port_mode m_sensor_port_mode;
//	bool m_invert_direction;
//	drv8301_oc_mode m_drv8301_oc_mode;
//	int m_drv8301_oc_adj;
//	float m_bldc_f_sw_min;
//	float m_bldc_f_sw_max;
//	float m_dc_f_sw;
//	float m_ntc_motor_beta;
//	out_aux_mode m_out_aux_mode;
//	temp_sensor_type m_motor_temp_sens_type;
//	float m_ptc_motor_coeff;
//	int m_hall_extra_samples;

	// Setup info
	mcconf->si_motor_poles = HALL_M1._Super.bElToMecRatio;
//	float si_gear_ratio;
//	float si_wheel_diameter;
	mcconf->si_battery_type = BATTERY_TYPE_LIION_3_0__4_2;
	//	int si_battery_cells;
//	float si_battery_ah;

	// BMS Configuration
//	bms_config bms;

	conf_general_recalc();
	mc_conf = *mcconf;

}

void conf_general_readback_mc(mc_configuration *mcconf) {

	// Limits
		mcconf->l_current_max = SpeednTorqCtrlM1.MaxPositiveTorque / CURRENT_FACTOR;
		mcconf->l_current_min  = SpeednTorqCtrlM1.MinNegativeTorque  / CURRENT_FACTOR;
//		float l_in_current_max;
//		float l_in_current_min;
		mcconf->l_abs_current_max = 60;
		mcconf->l_min_erpm = SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit * HALL_M1._Super.bElToMecRatio;
		mcconf->l_max_erpm = SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit * HALL_M1._Super.bElToMecRatio;
//		float l_erpm_start;
//		float l_max_erpm_fbrake;
//		float l_max_erpm_fbrake_cc;
		mcconf->l_min_vin = RealBusVoltageSensorParamsM1.UnderVoltageThreshold / VOLT_SCALING;
		mcconf->l_max_vin = RealBusVoltageSensorParamsM1.OverVoltageThreshold / VOLT_SCALING;
		mcconf->l_battery_cut_end = RealBusVoltageSensorParamsM1.UnderVoltageThreshold / VOLT_SCALING;
		mcconf->l_battery_cut_start = RealBusVoltageSensorParamsM1.OverVoltageThreshold / VOLT_SCALING;
//		bool l_slow_abs_current;
//		float l_temp_fet_start;
//		float l_temp_fet_end;
//		float l_temp_motor_start;
//		float l_temp_motor_end;
//		float l_temp_accel_dec;
//		float l_min_duty;
//		float l_max_duty;
//		float l_watt_max;
//		float l_watt_min;
//		float l_current_max_scale;
//		float l_current_min_scale;
//		float l_duty_start;
//		// Overridden limits (Computed during runtime)
//		float lo_current_max;
//		float lo_current_min;
//		float lo_in_current_max;
//		float lo_in_current_min;
//		float lo_current_motor_max_now;
//		float lo_current_motor_min_now;




	// Hall sensor
	for(int i=0;i<8;i++){
		mcconf->hall_table[i] = HALL_M1.lut[i];
	}

	// BLDC switching and drive
	mcconf->motor_type = MOTOR_TYPE_FOC;
	mcconf->sensor_mode = SENSOR_MODE_SENSORED;
	mcconf->pwm_mode = PWM_MODE_SYNCHRONOUS;

	// FOC
	mcconf->foc_current_kp = PIDIqHandle_M1.hKpGain / 100.0;
    mcconf->foc_current_ki = PIDIqHandle_M1.hKiGain / 100.0;
    mcconf->foc_f_sw = PWM_FREQUENCY;
//	float foc_dt_us;
    mcconf->foc_encoder_offset = ANG_TO_DEG(HALL_M1.PhaseShift);
//	bool foc_encoder_inverted;
//	float foc_encoder_ratio;
//	float foc_encoder_sin_offset;
//	float foc_encoder_sin_gain;
//	float foc_encoder_cos_offset;
//	float foc_encoder_cos_gain;
//	float foc_encoder_sincos_filter_constant;
//	float foc_motor_l;
//	float foc_motor_ld_lq_diff;
//	float foc_motor_r;
//	float foc_motor_flux_linkage;
//	float foc_observer_gain;
//	float foc_observer_gain_slow;
//	float foc_pll_kp;
//	float foc_pll_ki;
//	float foc_duty_dowmramp_kp;
//	float foc_duty_dowmramp_ki;
//	float foc_openloop_rpm;
//	float foc_openloop_rpm_low;
//	float foc_d_gain_scale_start;
//	float foc_d_gain_scale_max_mod;
//	float foc_sl_openloop_hyst;
//	float foc_sl_openloop_time;
//	float foc_sl_openloop_time_lock;
//	float foc_sl_openloop_time_ramp;
	mcconf->foc_sensor_mode = FOC_SENSOR_MODE_HALL;

	for(int i=0;i<8;i++){
		mcconf->foc_hall_table[i] = HALL_M1.lut[i];
	}
	mcconf->foc_hall_interp_erpm = (float)HALL_M1.SwitchSpeed * (float)HALL_M1._Super.bElToMecRatio;
//	float foc_sl_erpm;
//	bool foc_sample_v0_v7;
//	bool foc_sample_high_current;
//	float foc_sat_comp;
//	bool foc_temp_comp;
//	float foc_temp_comp_base_temp;
//	float foc_current_filter_const;
//	mc_foc_cc_decoupling_mode foc_cc_decoupling;
//	mc_foc_observer_type foc_observer_type;
//	float foc_hfi_voltage_start;
//	float foc_hfi_voltage_run;
//	float foc_hfi_voltage_max;
//	float foc_sl_erpm_hfi;
//	uint16_t foc_hfi_start_samples;
//	float foc_hfi_obs_ovr_sec;
//	uint8_t foc_hfi_samples;

	// GPDrive
//	int gpd_buffer_notify_left;
//	int gpd_buffer_interpol;
//	float gpd_current_filter_const;
//	float gpd_current_kp;
//	float gpd_current_ki;

	// Speed PID
//	float s_pid_kp;
//	float s_pid_ki;
//	float s_pid_kd;
//	float s_pid_kd_filter;
//	float s_pid_min_erpm;
//	bool s_pid_allow_braking;
//	float s_pid_ramp_erpms_s;

	// Pos PID
//	float p_pid_kp;
//	float p_pid_ki;
//	float p_pid_kd;
//	float p_pid_kd_filter;
//	float p_pid_ang_div;

	// Current controller
//	float cc_startup_boost_duty;
//	float cc_min_current;
//	float cc_gain;
//	float cc_ramp_step_max;

	// Misc
//	int32_t m_fault_stop_time_ms;
//	float m_duty_ramp_step;
//	float m_current_backoff_gain;
//	uint32_t m_encoder_counts;
//	sensor_port_mode m_sensor_port_mode;
//	bool m_invert_direction;
//	drv8301_oc_mode m_drv8301_oc_mode;
//	int m_drv8301_oc_adj;
//	float m_bldc_f_sw_min;
//	float m_bldc_f_sw_max;
//	float m_dc_f_sw;
//	float m_ntc_motor_beta;
//	out_aux_mode m_out_aux_mode;
//	temp_sensor_type m_motor_temp_sens_type;
//	float m_ptc_motor_coeff;
//	int m_hall_extra_samples;

	// Setup info
	mcconf->si_motor_poles = HALL_M1._Super.bElToMecRatio;
//	float si_gear_ratio;
//	float si_wheel_diameter;
	mcconf->si_battery_type = BATTERY_TYPE_LIION_3_0__4_2;
	//	int si_battery_cells;
//	float si_battery_ah;

	// BMS Configuration
//	bms_config bms;

}
