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
#include "current_sense.h"
#include <math.h>

static float tacho_scale;
stm_state VescToSTM_mode;

#define DIR_MUL   (mc_conf.m_invert_direction ? -1 : 1)

int32_t current_to_torque(int32_t curr_ma){
	float ret = curr_ma * CURRENT_FACTOR_mA;
	return ret;
}

int16_t VescToSTM_Iq_lim_hook(int16_t iq){

	int32_t temp_fet_start = mc_conf.l_temp_fet_start;
	int32_t temp = VescToSTM_get_temperature();
	if(temp > temp_fet_start){
		if(temp > temp_fet_start){
			iq = utils_map_int(temp, temp_fet_start, mc_conf.l_temp_fet_end, iq, 0);
		}
	}

	return iq;
}


int32_t VescToSTM_rpm_to_speed(int32_t rpm){
	int32_t speed = ((rpm*SPEED_UNIT*mc_conf.si_motor_poles)/_RPM);
	return speed;
}

int32_t VescToSTM_speed_to_rpm(int32_t speed){
	int32_t rpm = (speed*60/10)/mc_conf.si_motor_poles;
	return rpm;
}

int32_t VescToSTM_speed_to_erpm(int32_t speed){
	int32_t erpm = speed*60/10;
	return erpm;
}

int32_t VescToSTM_erpm_to_speed(int32_t erpm){
	int32_t speed = ((erpm*SPEED_UNIT)/_RPM);
	return speed;
}

static int16_t erpm_to_int16(int32_t erpm){
	int32_t speed = VescToSTM_rpm_to_speed(erpm);
	int32_t out = ((int32_t)HALL_M1._Super.DPPConvFactor * speed) / ((int32_t) SPEED_UNIT * (int32_t)HALL_M1._Super.hMeasurementFrequency);
	return out;
}


static int last_y;
void VescToStm_nunchuk_update_erpm(){
	if(VescToSTM_mode == STM_STATE_NUNCHUCK){
		if(last_y > 126){
			VescToSTM_set_current_rel_int(utils_map_int(last_y, 127, 255, 0, 32768));
		}else{
			VescToSTM_set_brake_rel_int(utils_map(last_y, 0, 126, 32768, 0));
		}
	}
}

void VescToStm_nunchuk_update_output(chuck_data * chuck_d){
	VescToSTM_mode = STM_STATE_NUNCHUCK;
	VescToSTM_timeout_reset();
	if(chuck_d->js_y != last_y){
		last_y = chuck_d->js_y;
		if(last_y > 126){
			VescToSTM_set_current_rel_int(utils_map_int(last_y, 127, 255, 0, 32768));
		}else{
			if(chuck_d->bt_c){
				VescToSTM_set_current_rel_int(utils_map_int(last_y, 0, 126, -32768, 0));
			}else{
				VescToSTM_set_brake_rel_int(utils_map_int(last_y, 0, 126, 32768, 0));
			}
		}
	}
}

float VescToStm_nunchuk_get_decoded_chuk(void){
	return ((float)last_y - 128.0) / 128.0;
}


void VescToSTM_set_open_loop(bool enabled, int16_t init_angle, int16_t erpm){
	if(enabled){
		VescToSTM_mode = STM_STATE_OPENLOOP;
		SpeednTorqCtrlM1.SPD->open_angle = init_angle;
		SpeednTorqCtrlM1.SPD->open_loop = true;
		SpeednTorqCtrlM1.SPD->open_speed = erpm_to_int16(erpm);
	}else{
		SpeednTorqCtrlM1.SPD->open_loop = false;
		SpeednTorqCtrlM1.SPD->open_speed = 0;
	}
}

void VescToSTM_ramp_current(float current){ //in Amps
	current*=1000;
	qd_t currComp;
	currComp.q = 0;
	currComp.d = 0;
	for (int i = 0;i < 1000;i++) {
		currComp.q = current_to_torque((float)i * current / 1000.0);
		MCI_SetCurrentReferences(pMCI[M1],currComp);
		vTaskDelay(MS_TO_TICKS(1));
		VescToSTM_timeout_reset();
	}
}

void VescToSTM_set_open_loop_rpm(int16_t erpm){
	SpeednTorqCtrlM1.SPD->open_speed = erpm_to_int16(erpm);
}


const uint8_t test[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
uint8_t VescToSTM_get_uid(uint8_t * ptr, uint8_t size){
	if(size>12) size=12;
	memcpy(ptr, test, size);
	return size;
}


float VescToSTM_get_pid_pos_now(){
	return 360.0 / 65536.0 * (float)SpeednTorqCtrlM1.SPD->hElAngle;
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
	if(appconf.timeout_msec){
		if((xTaskGetTickCount() - last_reset) > (appconf.timeout_msec*2)){
			VescToSTM_set_brake(appconf.timeout_brake_current*1000);
			last_reset = xTaskGetTickCount();
		}
	}
};
void VescToSTM_enable_timeout(bool enbale){
	timeout_enable = enbale;
}

void VescToSTM_update_torque(int32_t q, int32_t min_erpm, int32_t max_erpm){
	q *= DIR_MUL;
	if(q >= 0){
		FW_M1.wNominalSqCurr = q*q;
		FW_M1.hDemagCurrent	= -((float)q*mc_conf.foc_d_gain_scale_max_mod);
		SpeednTorqCtrlM1.PISpeed->wUpperIntegralLimit = q * SP_KDDIV;
		SpeednTorqCtrlM1.PISpeed->wLowerIntegralLimit = mc_conf.s_pid_allow_braking ? -q : 0;
		SpeednTorqCtrlM1.PISpeed->hUpperOutputLimit = q;
		SpeednTorqCtrlM1.PISpeed->hLowerOutputLimit = mc_conf.s_pid_allow_braking ? -q : 0;
		//HALL_M1.q = q;
		MCI_ExecSpeedRamp(pMCI[M1], VescToSTM_erpm_to_speed(max_erpm), 0);

	}else{
		FW_M1.wNominalSqCurr = q*q;
		FW_M1.hDemagCurrent	= ((float)q*mc_conf.foc_d_gain_scale_max_mod);
		SpeednTorqCtrlM1.PISpeed->wUpperIntegralLimit = mc_conf.s_pid_allow_braking ? -q : 0;
		SpeednTorqCtrlM1.PISpeed->wLowerIntegralLimit = q * SP_KDDIV;
		SpeednTorqCtrlM1.PISpeed->hUpperOutputLimit = mc_conf.s_pid_allow_braking ? -q : 0;
		SpeednTorqCtrlM1.PISpeed->hLowerOutputLimit = q;
		//HALL_M1.q = q;
		MCI_ExecSpeedRamp(pMCI[M1], VescToSTM_erpm_to_speed(min_erpm) , 0);
	}
}

void VescToSTM_set_torque(int32_t current){
	VescToSTM_mode = STM_STATE_TORQUE;
	VescToSTM_set_open_loop(false, 0, 0);

	int q = current_to_torque(current);
	utils_truncate_number_int(&q, SpeednTorqCtrlM1.MinNegativeTorque, SpeednTorqCtrlM1.MaxPositiveTorque);

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
	VescToSTM_update_torque(q, mc_conf.lo_min_erpm, mc_conf.lo_max_erpm);
}

void VescToSTM_set_brake_current_rel(float val) {
	if (fabsf(val) > 0.001) {
		val = 0;
	}

	VescToSTM_set_brake(val * fabsf(mc_conf.lo_current_min));
}

void VescToSTM_set_brake_rel_int(int32_t val){
	VescToSTM_mode = STM_STATE_BRAKE;
	VescToSTM_set_open_loop(false, 0, 0);

	if(val<0) return;

	int32_t p_q = (int32_t)SpeednTorqCtrlM1.MaxPositiveTorque * val / 32768;
	int32_t n_q = (int32_t)SpeednTorqCtrlM1.MinNegativeTorque * val / 32768;

	if(p_q > SpeednTorqCtrlM1.MaxPositiveTorque){
		p_q = SpeednTorqCtrlM1.MaxPositiveTorque;
	}
	if(n_q < SpeednTorqCtrlM1.MinNegativeTorque){
		n_q = SpeednTorqCtrlM1.MinNegativeTorque;
	}
	FW_M1.hDemagCurrent	= 0;
	SpeednTorqCtrlM1.PISpeed->hUpperOutputLimit = p_q;
	SpeednTorqCtrlM1.PISpeed->hLowerOutputLimit = n_q;
	SpeednTorqCtrlM1.PISpeed->wUpperIntegralLimit = p_q * SP_KIDIV;
	SpeednTorqCtrlM1.PISpeed->wLowerIntegralLimit = n_q * SP_KIDIV;
	MCI_ExecSpeedRamp(pMCI[M1], 0 , 0);
}

void VescToSTM_set_brake(int32_t current){
	VescToSTM_mode = STM_STATE_BRAKE;
	VescToSTM_set_open_loop(false, 0, 0);

	int q = current_to_torque(current);
	utils_truncate_number_int(&q, SpeednTorqCtrlM1.MinNegativeTorque, SpeednTorqCtrlM1.MaxPositiveTorque);

	if(q > 0){
		FW_M1.wNominalSqCurr = q*q;
		FW_M1.hDemagCurrent	= -((float)SpeednTorqCtrlM1.MaxPositiveTorque*mc_conf.foc_d_gain_scale_max_mod);
		SpeednTorqCtrlM1.PISpeed->hUpperOutputLimit = q;
		SpeednTorqCtrlM1.PISpeed->hLowerOutputLimit = -q;
		SpeednTorqCtrlM1.PISpeed->wUpperIntegralLimit = q * SP_KIDIV;
		SpeednTorqCtrlM1.PISpeed->wLowerIntegralLimit = -q * SP_KIDIV;
		MCI_ExecSpeedRamp(pMCI[M1], 0 , 0);
	}else{
		FW_M1.wNominalSqCurr = q*q;
		FW_M1.hDemagCurrent = ((float)SpeednTorqCtrlM1.MinNegativeTorque*mc_conf.foc_d_gain_scale_max_mod);
		SpeednTorqCtrlM1.PISpeed->hUpperOutputLimit = -q;
		SpeednTorqCtrlM1.PISpeed->hLowerOutputLimit = q;
		SpeednTorqCtrlM1.PISpeed->wUpperIntegralLimit = -q * SP_KIDIV;
		SpeednTorqCtrlM1.PISpeed->wLowerIntegralLimit = q * SP_KIDIV;
		MCI_ExecSpeedRamp(pMCI[M1], 0 , 0);
	}
}


void VescToSTM_set_speed(int32_t rpm){
	rpm *= DIR_MUL;
	VescToSTM_mode = STM_STATE_SPEED;
	int32_t erpm = rpm * mc_conf.si_motor_poles;
	if(erpm>0){
		if(erpm < mc_conf.s_pid_min_erpm) erpm = mc_conf.s_pid_min_erpm;
		if(erpm > mc_conf.lo_max_erpm) erpm = mc_conf.lo_max_erpm;
	}else{
		if(erpm > -mc_conf.s_pid_min_erpm) erpm = -mc_conf.s_pid_min_erpm;
		if(erpm < mc_conf.lo_min_erpm) erpm = mc_conf.lo_min_erpm;
	}
	VescToSTM_set_open_loop(false, 0, 0);
	if(rpm>=0){
		FW_M1.wNominalSqCurr = SpeednTorqCtrlM1.MaxPositiveTorque*SpeednTorqCtrlM1.MaxPositiveTorque;
		FW_M1.hDemagCurrent	= -((float)SpeednTorqCtrlM1.MaxPositiveTorque*mc_conf.foc_d_gain_scale_max_mod);
		SpeednTorqCtrlM1.PISpeed->wUpperIntegralLimit = (int32_t)SpeednTorqCtrlM1.MaxPositiveTorque * SP_KIDIV;
		SpeednTorqCtrlM1.PISpeed->wLowerIntegralLimit = mc_conf.s_pid_allow_braking ? (int32_t)SpeednTorqCtrlM1.MinNegativeTorque * SP_KIDIV: 0;
		SpeednTorqCtrlM1.PISpeed->hUpperOutputLimit = SpeednTorqCtrlM1.MaxPositiveTorque;
		SpeednTorqCtrlM1.PISpeed->hLowerOutputLimit = mc_conf.s_pid_allow_braking ? SpeednTorqCtrlM1.MinNegativeTorque : 0;
	}else{
		FW_M1.wNominalSqCurr = SpeednTorqCtrlM1.MinNegativeTorque*SpeednTorqCtrlM1.MinNegativeTorque;
		FW_M1.hDemagCurrent = ((float)SpeednTorqCtrlM1.MinNegativeTorque*mc_conf.foc_d_gain_scale_max_mod);
		SpeednTorqCtrlM1.PISpeed->wUpperIntegralLimit = mc_conf.s_pid_allow_braking ? (int32_t)SpeednTorqCtrlM1.MaxPositiveTorque * SP_KIDIV: 0;
		SpeednTorqCtrlM1.PISpeed->wLowerIntegralLimit = (int32_t)SpeednTorqCtrlM1.MinNegativeTorque * SP_KIDIV;
		SpeednTorqCtrlM1.PISpeed->hUpperOutputLimit = mc_conf.s_pid_allow_braking ? SpeednTorqCtrlM1.MaxPositiveTorque : 0;
		SpeednTorqCtrlM1.PISpeed->hLowerOutputLimit = SpeednTorqCtrlM1.MinNegativeTorque;
	}
	int32_t ramp_time = 0;
	if(mc_conf.s_pid_ramp_erpms_s) ramp_time = (float)(erpm * 1000) / mc_conf.s_pid_ramp_erpms_s;
	MCI_ExecSpeedRamp(pMCI[M1], VescToSTM_erpm_to_speed(erpm), ramp_time);
}




float VescToSTM_get_temperature(){
	return NTC_GetAvTemp_C(pMCT[M1]->pTemperatureSensor);
}
float VescToSTM_get_temperature2(){
	return 0.0;
}


float VescToSTM_get_phase_current(){
	int32_t wAux1 = FOCVars[M1].Iq_avg * FOCVars[M1].Iq_avg;
	int32_t wAux2 = FOCVars[M1].Id_avg * FOCVars[M1].Id_avg;
	wAux1 += wAux2;
	wAux1 = MCM_Sqrt(wAux1);
	wAux1 = wAux1 * SIGN(FOCVars[M1].Iq_avg);

	return (float)wAux1 / CURRENT_FACTOR_A;
}

float VescToSTM_get_input_current(){
#ifdef M365
	return (float)pMPM[M1]->_super.hAvrgElMotorPowerW/(float)VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor);
#endif

#ifdef G30P
	return (float)pMPM[M1]->_super.hAvrgElMotorPowerW/(float)VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor);
	//return (float)CURR_GetCurrent(pMCT[M1]->pMainCurrentSensor);// NOT WORKING :(
#endif
}

/**
 * Read and reset the average direct axis motor current. (FOC only)
 *
 * @return
 * The average D axis current.
 */
float VescToSTM_get_id(){
	return (float)FOCVars[M1].Id_avg / CURRENT_FACTOR_A;
}

/**
 * Read and reset the average quadrature axis motor current. (FOC only)
 *
 * @return
 * The average Q axis current.
 */
float VescToSTM_get_iq(){
	return (float)FOCVars[M1].Iq_avg / CURRENT_FACTOR_A;
}

/**
 * Read and reset the average direct axis motor voltage. (FOC only)
 *
 * @return
 * The average D axis voltage.
 */
float VescToSTM_get_Vd(){
	float Vin = VescToSTM_get_bus_voltage();
	float fVd = Vin / 65536.0 * (float)FOCVars[M1].Vd_avg;
	return fVd;
}

/**
 * Read and reset the average quadrature axis motor voltage. (FOC only)
 *
 * @return
 * The average Q axis voltage.
 */
float VescToSTM_get_Vq(){
	float Vin = VescToSTM_get_bus_voltage();
	float fVq = Vin / 65536.0 * (float)FOCVars[M1].Vq_avg;
	return fVq;
}

float VescToSTM_get_bus_voltage(){
	return (float)(VBS_GetAvBusVoltage_d(pMCT[M1]->pBusVoltageSensor)) / BATTERY_VOLTAGE_GAIN;
}

int32_t VescToSTM_get_erpm(){
	int32_t erpm = VescToSTM_speed_to_erpm(MCI_GetAvrgMecSpeedUnit( pMCI[M1] ));
	return erpm;
}

int32_t VescToSTM_get_erpm_fast(){
	int32_t speed = ( (  HALL_M1._Super.hElSpeedDpp * ( int32_t )HALL_M1._Super.hMeasurementFrequency * (int32_t) SPEED_UNIT ) / (( int32_t ) HALL_M1._Super.DPPConvFactor));
	return VescToSTM_speed_to_erpm(speed);
}

int32_t VescToSTM_get_rpm(){
	int32_t rpm = VescToSTM_speed_to_rpm(MCI_GetAvrgMecSpeedUnit( pMCI[M1] ));
	return rpm;
}

void VescToSTM_stop_motor(){
	MCI_StopMotor( pMCI[M1] );
}
void VescToSTM_start_motor(){
	MCI_StartMotor( pMCI[M1] );
}


void VescToSTM_init_odometer(mc_configuration* conf){
	tacho_scale = (conf->si_wheel_diameter * M_PI) / (3.0 * conf->si_motor_poles * conf->si_gear_ratio);
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
 * Get the speed based on wheel diameter, gearing and motor pole settings.
 *
 * @return
 * Speed, in m/s
 */
float VescToSTM_get_speed(void) {
	float temp = (((float)MCI_GetAvrgMecSpeedUnit(pMCI[M1]) / 10.0) * mc_conf.si_wheel_diameter * M_PI) / mc_conf.si_gear_ratio / mc_conf.si_motor_poles;
	return temp;
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

	int q = abs(current_to_torque(current*1000));
	if(VescToSTM_mode != STM_STATE_HANDBRAKE){
		VescToSTM_set_open_loop(true, SpeednTorqCtrlM1.SPD->hElAngle, 0);
	}
	VescToSTM_mode = STM_STATE_HANDBRAKE;

	utils_truncate_number_int(&q, 0, SpeednTorqCtrlM1.MaxPositiveTorque);

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
#if BATTERY_SUPPORT_LIION
	case BATTERY_TYPE_LIION_3_0__4_2:
		battery_avg_voltage = ((3.2 + 4.2) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((3.2 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		float batt_left = utils_map(v_in / (float)(conf->si_battery_cells),
									3.2, 4.2, 0.0, 1.0);
		batt_left = utils_batt_liion_norm_v_to_capacity(batt_left);
		ah_tot *= 0.85; // 0.85 because the battery is not fully depleted at 3.2V / cell
		ah_left = batt_left * ah_tot;
		break;
#endif
#if BATTERY_SUPPORT_LIFEPO
	case BATTERY_TYPE_LIIRON_2_6__3_6:
		battery_avg_voltage = ((2.8 + 3.6) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((2.8 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		ah_left = utils_map(v_in / (float)(conf->si_battery_cells),
				2.6, 3.6, 0.0, conf->si_battery_ah);
		break;
#endif
#if BATTERY_SUPPORT_LEAD
	case BATTERY_TYPE_LEAD_ACID:
		// TODO: This does not really work for lead-acid batteries
		battery_avg_voltage = ((2.1 + 2.36) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((2.1 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		ah_left = utils_map(v_in / (float)(conf->si_battery_cells),
				2.1, 2.36, 0.0, conf->si_battery_ah);
		break;
#endif

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
	if(mc_conf.m_invert_direction){
	}
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
	VescToSTM_mode = STM_STATE_TORQUE;
	VescToSTM_set_open_loop(false, 0, 0);
	int q;
	float torque;
	if(val>0){
		torque = (float)SpeednTorqCtrlM1.MaxPositiveTorque * val;
		q = torque;
	}else{
		torque = (float)SpeednTorqCtrlM1.MinNegativeTorque * val;
		q = torque;
		q = q *-1;
	}

	utils_truncate_number_int(&q, SpeednTorqCtrlM1.MinNegativeTorque, SpeednTorqCtrlM1.MaxPositiveTorque);

	VescToSTM_update_torque(q, mc_conf.lo_min_erpm, mc_conf.lo_max_erpm);
}

/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param current
 * The relative current value, range [-32768 +32768]
 */
void VescToSTM_set_current_rel_int(int32_t val) {
	VescToSTM_mode = STM_STATE_TORQUE;
	VescToSTM_set_open_loop(false, 0, 0);
	int q;
	if(val>0){
		q = (int32_t)SpeednTorqCtrlM1.MaxPositiveTorque * val / 32768;
	}else{
		q = (int32_t)SpeednTorqCtrlM1.MinNegativeTorque * val / 32768;
		q = q *-1;
	}

	utils_truncate_number_int(&q, SpeednTorqCtrlM1.MinNegativeTorque, SpeednTorqCtrlM1.MaxPositiveTorque);

	VescToSTM_update_torque(q, mc_conf.lo_min_erpm, mc_conf.lo_max_erpm);
}

mc_fault_code VescToSTM_get_fault(void) {
	mc_fault_code fault = FAULT_CODE_NONE;
	return fault;
}
