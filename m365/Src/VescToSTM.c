#include <stdlib.h>
#include "VescToSTM.h"
#include "mc_interface.h"
#include "stm32f1xx_hal.h"
#include "crc.h"
#include "defines.h"
#include "drive_parameters.h"
#include "mc_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "conf_general.h"
#include "utils.h"

static float tacho_scale;

int32_t current_to_torque(int32_t curr_ma){
	float ret = curr_ma * CURRENT_FACTOR_mA;
	return ret;

}

void VescToSTM_init_odometer(mc_configuration* conf){
	tacho_scale = (conf->si_wheel_diameter * M_PI) / (3.0 * conf->si_motor_poles * conf->si_gear_ratio);
}


uint32_t last_reset=0;


float VescToSTM_get_pid_pos_now(){
	return 360.0 / 65536.0 * (float)pMCI[M1]->pSTC->SPD->hElAngle;
}


void VescToSTM_timeout_reset(){
	last_reset = xTaskGetTickCount();
};

void VescToSTM_handle_timeout(){
	if((xTaskGetTickCount() - last_reset) > 1000){
		if(pMCI[M1]->pSTC->SPD->open_loop == false){
			VescToSTM_set_brake(0);
		}
	}
};

void VescToSTM_set_torque(int32_t current){
	pMCI[M1]->pSTC->SPD->open_loop = false;
	int32_t q = current_to_torque(current);
	if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
		q = SpeednTorqCtrlM1.MaxPositiveTorque;
	}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
		q = SpeednTorqCtrlM1.MinNegativeTorque;
	}
	if(q > 0){
		pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)-q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = q;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = 0;
		MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_max_erpm / mc_conf.si_motor_poles , 0);

	}else{
		pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)-q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = 0;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = q;
		MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_min_erpm / mc_conf.si_motor_poles , 0);
	}

}

void VescToSTM_set_brake(int32_t current){
	pMCI[M1]->pSTC->SPD->open_loop = false;
	int32_t q = current_to_torque(current);
	if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
		q = SpeednTorqCtrlM1.MaxPositiveTorque;
	}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
		q = SpeednTorqCtrlM1.MinNegativeTorque;
	}
	if(q > 0){
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = q;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = -q;
		MCI_ExecSpeedRamp(pMCI[M1], 0 , 0);
	}else{
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = -q;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = q;
		MCI_ExecSpeedRamp(pMCI[M1], 0 , 0);
	}
}

void VescToSTM_set_speed(int32_t rpm){
	pMCI[M1]->pSTC->SPD->open_loop = false;
	pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = SpeednTorqCtrlM1.MaxPositiveTorque;
	pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = SpeednTorqCtrlM1.MinNegativeTorque;
	MCI_ExecSpeedRamp(pMCI[M1], rpm , 0);
}

float VescToSTM_get_temperature(){
	return NTC_GetAvTemp_C(pMCT[M1]->pTemperatureSensor);
}

float VescToSTM_get_phase_current(){
	return MCI_GetPhaseCurrentAmplitude(pMCI[M1])/CURRENT_FACTOR;
}

float VescToSTM_get_input_current(){
	return (float)pMPM[M1]->_super.hAvrgElMotorPowerW/(float)VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor);
}

float VescToSTM_get_id(){
	return pMCI[M1]->pFOCVars->Iqd.d / CURRENT_FACTOR;
}
float VescToSTM_get_iq(){
	float iq = (float)pMCI[M1]->pFOCVars->Iq_sum / (float)pMCI[M1]->pFOCVars->Iq_samples / CURRENT_FACTOR;
	pMCI[M1]->pFOCVars->Iq_sum = 0;
	pMCI[M1]->pFOCVars->Iq_samples = 0;
	return iq;
}

float VescToSTM_get_Vd(){
	return pMCI[M1]->pFOCVars->Vqd.d / VOLT_SCALING;
}
float VescToSTM_get_Vq(){
	return pMCI[M1]->pFOCVars->Vqd.q / VOLT_SCALING;
}

float VescToSTM_get_bus_voltage(){
	return VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor);
}

int32_t VescToSTM_get_erpm(){
	return MCI_GetAvrgMecSpeedUnit( pMCI[M1] ) * HALL_M1._Super.bElToMecRatio;
}

int32_t VescToSTM_get_rpm(){
	return MCI_GetAvrgMecSpeedUnit( pMCI[M1] );
}

void VescToSTM_stop_motor(){
	MCI_StopMotor( pMCI[M1] );
}
void VescToSTM_start_motor(){
	MCI_StartMotor( pMCI[M1] );
}

/**
 * Get the distance traveled based on wheel diameter, gearing and motor pole settings.
 *
 * @return
 * Distance traveled since boot, in meters
 */
float VescToSTM_get_distance(void) {
	return VescToSTM_get_tachometer_value(false) * tacho_scale;
}

/**
 * Get the absolute distance traveled based on wheel diameter, gearing and motor pole settings.
 *
 * @return
 * Absolute distance traveled since boot, in meters
 */
float VescToSTM_get_distance_abs(void) {
	return VescToSTM_get_tachometer_abs_value(false) * tacho_scale;
}

static uint32_t m_odometer_meters;
/**
 * Set odometer value in meters.
 *
 * @param new_odometer_meters
 * new odometer value in meters
 */
void VescToSTM_set_odometer(uint32_t new_odometer_meters) {
	m_odometer_meters = new_odometer_meters - VescToSTM_get_distance_abs();
}

/**
 * Return current odometer value in meters.
 *
 * @return
 * Odometer value in meters, including current trip
 */
uint32_t VescToSTM_get_odometer(void) {
	return m_odometer_meters + VescToSTM_get_distance_abs();
}

/**
 * Read the number of steps the motor has rotated. This number is signed and
 * will return a negative number when the motor is rotating backwards.
 *
 * @param reset
 * If true, the tachometer counter will be reset after this call.
 *
 * @return
 * The tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int32_t VescToSTM_get_tachometer_value(bool reset) {
	int val = HALL_M1.tachometer;

	if (reset) {
		HALL_M1.tachometer = 0;
	}

	return val;
}

/**
 * Read the absolute number of steps the motor has rotated.
 *
 * @param reset
 * If true, the tachometer counter will be reset after this call.
 *
 * @return
 * The tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int32_t VescToSTM_get_tachometer_abs_value(bool reset) {
	int val = HALL_M1.tachometer_abs;

	if (reset) {
		HALL_M1.tachometer_abs = 0;
	}

	return val;
}

/**
 * Apply a fixed static current vector in open loop to emulate an electric
 * handbrake.
 *
 * @param current
 * The brake current to use.
 */
void VescToSTM_set_handbrake(float current) {
	int32_t q = abs(current_to_torque(current*1000));

	pMCI[M1]->pSTC->SPD->open_loop = true;
	pMCI[M1]->pSTC->SPD->open_angle = 0;

	if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
		q = SpeednTorqCtrlM1.MaxPositiveTorque;
	}
	pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = q;
	pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = q;
	MCI_ExecSpeedRamp(pMCI[M1], 0 , 0);
}

/**
 * Get the battery level, based on battery settings in configuration. Notice that
 * this function is based on remaining watt hours, and not amp hours.
 *
 * @param wh_left
 * Pointer to where to store the remaining watt hours, can be null.
 *
 * @return
 * Battery level, range 0 to 1
 */
float VescToSTM_get_battery_level(float *wh_left) {
	const volatile mc_configuration *conf = &mc_conf;
	const float v_in = VescToSTM_get_bus_voltage();
	float battery_avg_voltage = 0.0;
	float battery_avg_voltage_left = 0.0;
	float ah_left = 0;
	float ah_tot = conf->si_battery_ah;

	switch (conf->si_battery_type) {
	case BATTERY_TYPE_LIION_3_0__4_2:
		battery_avg_voltage = ((3.2 + 4.2) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((3.2 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		float batt_left = utils_map(v_in / (float)(conf->si_battery_cells),
									3.2, 4.2, 0.0, 1.0);
		batt_left = utils_batt_liion_norm_v_to_capacity(batt_left);
		ah_tot *= 0.85; // 0.85 because the battery is not fully depleted at 3.2V / cell
		ah_left = batt_left * ah_tot;
		break;

	case BATTERY_TYPE_LIIRON_2_6__3_6:
		battery_avg_voltage = ((2.8 + 3.6) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((2.8 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		ah_left = utils_map(v_in / (float)(conf->si_battery_cells),
				2.6, 3.6, 0.0, conf->si_battery_ah);
		break;

	case BATTERY_TYPE_LEAD_ACID:
		// TODO: This does not really work for lead-acid batteries
		battery_avg_voltage = ((2.1 + 2.36) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((2.1 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		ah_left = utils_map(v_in / (float)(conf->si_battery_cells),
				2.1, 2.36, 0.0, conf->si_battery_ah);
		break;

	default:
		break;
	}

	const float wh_batt_tot = ah_tot * battery_avg_voltage;
	const float wh_batt_left = ah_left * battery_avg_voltage_left;

	if (wh_left) {
		*wh_left = wh_batt_left;
	}

	return wh_batt_left / wh_batt_tot;
}

float VescToSTM_get_duty_cycle_now(void) {
	qd_t Vqd = 	MCI_GetVqd(pMCI[M1]);
	int32_t amplitude = (MCI_GetPhaseVoltageAmplitude(pMCI[M1]) * SIGN(Vqd.q) * 100) / 65536;
	return amplitude;
}

/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param current
 * The relative current value, range [-1.0 1.0]
 */
void VescToSTM_set_current_rel(float val) {
	pMCI[M1]->pSTC->SPD->open_loop = false;
	uint32_t q;
	float torque;
	if(val>0){
		torque = (float)SpeednTorqCtrlM1.MaxPositiveTorque * val;
		q = torque;
	}else{
		torque = (float)SpeednTorqCtrlM1.MinNegativeTorque * val;
		q = torque;
		q = q *-1;
	}

	if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
		q = SpeednTorqCtrlM1.MaxPositiveTorque;
	}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
		q = SpeednTorqCtrlM1.MinNegativeTorque;
	}
	if(q > 0){
		pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)-q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = q;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = 0;
		MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_max_erpm / mc_conf.si_motor_poles , 0);

	}else{
		pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)-q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = 0;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = q;
		MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_min_erpm / mc_conf.si_motor_poles , 0);
	}
}