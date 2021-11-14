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
#include <string.h>
#include "product.h"

static float tacho_scale;

int32_t current_to_torque(int32_t curr_ma){
	float ret = curr_ma * CURRENT_FACTOR_mA;
	return ret;

}

void VescToSTM_init_odometer(mc_configuration* conf){
	tacho_scale = (conf->si_wheel_diameter * M_PI) / (3.0 * conf->si_motor_poles * conf->si_gear_ratio);
}

static int16_t erpm_to_int16(int32_t erpm){
	int32_t out = ((int32_t)HALL_M1._Super.DPPConvFactor * erpm) / ((int32_t) SPEED_UNIT * (int32_t)HALL_M1._Super.hMeasurementFrequency);
	return out;
}

void VescToSTM_set_open_loop(bool enabled, int16_t init_angle, int16_t erpm){
	if(enabled){
		pMCI[M1]->pSTC->SPD->open_angle = init_angle;
		pMCI[M1]->pSTC->SPD->open_loop = true;
		pMCI[M1]->pSTC->SPD->open_speed = erpm_to_int16(erpm);
	}else{
		pMCI[M1]->pSTC->SPD->open_loop = false;
		pMCI[M1]->pSTC->SPD->open_speed = 0;
	}
}

void VescToSTM_set_open_loop_rpm(int16_t erpm){
	pMCI[M1]->pSTC->SPD->open_speed = erpm_to_int16(erpm);
}


const uint8_t test[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
uint8_t VescToSTM_get_uid(uint8_t * ptr, uint8_t size){
	if(size>12) size=12;
	memcpy(ptr, test, size);
	return size;
}


float VescToSTM_get_pid_pos_now(){
	return 360.0 / 65536.0 * (float)pMCI[M1]->pSTC->SPD->hElAngle;
}

uint32_t last_reset=0;
bool timeout_enable = true;
void VescToSTM_timeout_reset(){
	last_reset = xTaskGetTickCount();
};
void VescToSTM_handle_timeout(){
	if(!timeout_enable) {
		VescToSTM_timeout_reset();
	}
	if((xTaskGetTickCount() - last_reset) > 2000){
		VescToSTM_set_brake(0);
	}
};
void VescToSTM_enable_timeout(bool enbale){
	timeout_enable = enbale;
}

void VescToSTM_set_torque(int32_t current){
	pMCI[M1]->pSTC->SPD->open_loop = false;
	int32_t q = current_to_torque(current);
	if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
		q = SpeednTorqCtrlM1.MaxPositiveTorque;
	}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
		q = SpeednTorqCtrlM1.MinNegativeTorque;
	}
	volatile float Vin = VescToSTM_get_bus_voltage();
	if(Vin < mc_conf.l_battery_cut_start){
		float diff = mc_conf.l_battery_cut_start - mc_conf.l_battery_cut_end;
		float VinDiff = Vin - mc_conf.l_battery_cut_end;
		float qRed = (float)q / diff * VinDiff;
		q = qRed;
	}
	if(Vin < mc_conf.l_battery_cut_end){
		q=0;
	}

	if(q > 0){
		pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)-q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = q;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = mc_conf.s_pid_allow_braking ? -q : 0;
		MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_max_erpm / mc_conf.si_motor_poles , 0);

	}else{
		pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)-q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = mc_conf.s_pid_allow_braking ? -q : 0;
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
	int32_t erpm = rpm * mc_conf.si_motor_poles;
	if(erpm>0){
		if(erpm < mc_conf.s_pid_min_erpm) erpm = mc_conf.s_pid_min_erpm;
		if(erpm > mc_conf.l_max_erpm) erpm = mc_conf.l_max_erpm;
	}else{
		if(erpm > -mc_conf.s_pid_min_erpm) erpm = -mc_conf.s_pid_min_erpm;
		if(erpm < mc_conf.l_min_erpm) erpm = mc_conf.l_min_erpm;
	}
	pMCI[M1]->pSTC->SPD->open_loop = false;
	if(rpm>0){
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = SpeednTorqCtrlM1.MaxPositiveTorque;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = mc_conf.s_pid_allow_braking ? SpeednTorqCtrlM1.MinNegativeTorque : 0;
	}else{
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = mc_conf.s_pid_allow_braking ? SpeednTorqCtrlM1.MaxPositiveTorque : 0;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = SpeednTorqCtrlM1.MinNegativeTorque;
	}
	int32_t ramp_time = 0;
	if(mc_conf.s_pid_ramp_erpms_s) ramp_time = (float)(erpm * 1000) / mc_conf.s_pid_ramp_erpms_s;
	MCI_ExecSpeedRamp(pMCI[M1], erpm / mc_conf.si_motor_poles, ramp_time);
}




float VescToSTM_get_temperature(){
	return NTC_GetAvTemp_C(pMCT[M1]->pTemperatureSensor);
}


float VescToSTM_get_phase_current(){
	if(!pMCI[M1]->pFOCVars->Iq_samples || !pMCI[M1]->pFOCVars->Id_samples) return 0;
	int32_t iq = pMCI[M1]->pFOCVars->Iq_sum / pMCI[M1]->pFOCVars->Iq_samples;
	int32_t id = pMCI[M1]->pFOCVars->Id_sum / pMCI[M1]->pFOCVars->Id_samples;

	pMCI[M1]->pFOCVars->Iq_sum = 0;
	pMCI[M1]->pFOCVars->Iq_samples = 0;
	pMCI[M1]->pFOCVars->Id_sum = 0;
	pMCI[M1]->pFOCVars->Id_samples = 0;

	int32_t wAux1 = iq * iq;
	int32_t wAux2 = id * id;
	wAux1 += wAux2;
	wAux1 = MCM_Sqrt(wAux1);
	wAux1 = wAux1 * SIGN(iq);

	return (float)wAux1 / CURRENT_FACTOR;
}

float VescToSTM_get_input_current(){
	return (float)pMPM[M1]->_super.hAvrgElMotorPowerW/(float)VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor);
}

/**
 * Read and reset the average direct axis motor current. (FOC only)
 *
 * @return
 * The average D axis current.
 */
float VescToSTM_get_id(){
	if(!pMCI[M1]->pFOCVars->Id_samples) return 0;
	int32_t id = pMCI[M1]->pFOCVars->Id_sum / pMCI[M1]->pFOCVars->Id_samples;
	pMCI[M1]->pFOCVars->Id_sum = 0;
	pMCI[M1]->pFOCVars->Id_samples = 0;
	return id / CURRENT_FACTOR;
}

/**
 * Read and reset the average quadrature axis motor current. (FOC only)
 *
 * @return
 * The average Q axis current.
 */
float VescToSTM_get_iq(){
	if(!pMCI[M1]->pFOCVars->Iq_samples) return 0;
	int32_t iq = pMCI[M1]->pFOCVars->Iq_sum / pMCI[M1]->pFOCVars->Iq_samples;
	pMCI[M1]->pFOCVars->Iq_sum = 0;
	pMCI[M1]->pFOCVars->Iq_samples = 0;
	return iq / CURRENT_FACTOR;
}

/**
 * Read and reset the average direct axis motor voltage. (FOC only)
 *
 * @return
 * The average D axis voltage.
 */
float VescToSTM_get_Vd(){
	if(!pMCI[M1]->pFOCVars->Vd_samples) return 0;
	float Vin = VescToSTM_get_bus_voltage();
	int32_t Vd = pMCI[M1]->pFOCVars->Vd_sum / pMCI[M1]->pFOCVars->Vd_samples;
	pMCI[M1]->pFOCVars->Vd_sum = 0;
	pMCI[M1]->pFOCVars->Vd_samples = 0;
	float fVd = Vin / 65536.0 * (float)Vd;
	return fVd;
}

/**
 * Read and reset the average quadrature axis motor voltage. (FOC only)
 *
 * @return
 * The average Q axis voltage.
 */
float VescToSTM_get_Vq(){
	if(!pMCI[M1]->pFOCVars->Vq_samples) return 0;
	float Vin = VescToSTM_get_bus_voltage();
	int32_t Vq = pMCI[M1]->pFOCVars->Vq_sum / pMCI[M1]->pFOCVars->Vq_samples;
	pMCI[M1]->pFOCVars->Vq_sum = 0;
	pMCI[M1]->pFOCVars->Vq_samples = 0;
	float fVq = Vin / 65536.0 * (float)Vq;
	return fVq;
}

float VescToSTM_get_bus_voltage(){
	return (float)(VBS_GetAvBusVoltage_d(pMCT[M1]->pBusVoltageSensor)) / BATTERY_VOLTAGE_GAIN;
}

int32_t VescToSTM_get_erpm(){
	return MCI_GetAvrgMecSpeedUnit( pMCI[M1] ) * HALL_M1._Super.bElToMecRatio;
}

int32_t VescToSTM_get_erpm_fast(){
	return ( int16_t )( (  HALL_M1._Super.hElSpeedDpp * ( int32_t )HALL_M1._Super.hMeasurementFrequency * (int32_t) SPEED_UNIT ) / (( int32_t ) HALL_M1._Super.DPPConvFactor));
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
	float amplitude = (MCI_GetPhaseVoltageAmplitude(pMCI[M1]) * SIGN(Vqd.q)) / 32768.0;
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
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = mc_conf.s_pid_allow_braking ? -q : 0;
		MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_max_erpm / mc_conf.si_motor_poles , 0);

	}else{
		pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)-q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)q * SP_KDDIV;
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = mc_conf.s_pid_allow_braking ? -q : 0;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = q;
		MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_min_erpm / mc_conf.si_motor_poles , 0);
	}
}


mc_fault_code VescToSTM_get_fault(void) {
	mc_fault_code fault = FAULT_CODE_NONE;
	return fault;
}
