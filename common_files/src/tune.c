#include "tune.h"
#include "math.h"
#include "system.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "speed_pos_fdbk.h"
#include "drive_parameters.h"
#include "mc_stm_types.h"
#include "parameters_conversion.h"
#include "FreeRTOS.h"
#include <string.h>
#include <stdlib.h>
#include "utils.h"
#include "VescToSTM.h"
#include "conf_general.h"
#include "product.h"
#include "hall_speed_pos_fdbk.h"


//const uint8_t hall_arr[8] = {0,5,1,3,2,6,4,7};

static int16_t current_to_torque(int32_t curr_ma){
	float ret = curr_ma * CURRENT_FACTOR_mA;
	return ret;
}


/**
 * Run the motor in open loop and figure out at which angles the hall sensors are.
 *
 * @param current
 * Current to use.
 *
 * @param hall_table
 * Table to store the result to.
 *
 * @return
 * true: Success
 * false: Something went wrong
 */
bool tune_mcpwm_foc_hall_detect(float current, uint8_t *hall_table) {
	VescToSTM_enable_timeout(false);
	MCI_ExecTorqueRamp(pMCI[M1], 0, 0);
	MCI_StartMotor( pMCI[M1] );

	qd_t currComp;
	currComp.q = 0;
	currComp.d = 0;

	// Lock the motor
	VescToSTM_set_open_loop(true, 0, 0);
	int old_phase_shift = HALL_M1.PhaseShift;
	HALL_M1.PhaseShift = 0;


	for (int i = 0;i < 1000;i++) {
		currComp.q = current_to_torque((float)i * current / 1000.0);
		MCI_SetCurrentReferences(pMCI[M1],currComp);
		vTaskDelay(MS_TO_TICKS(1));
	}

	float sin_hall[8];
	float cos_hall[8];
	int hall_iterations[8];
	memset(sin_hall, 0, sizeof(sin_hall));
	memset(cos_hall, 0, sizeof(cos_hall));
	memset(hall_iterations, 0, sizeof(hall_iterations));

	// Forwards
	for (int i = 0;i < 3;i++) {
		for (int j = 0;j < 360;j++) {
			float m_phase_now_override = (float)j * M_PI / 180.0;
			pMCI[M1]->pSTC->SPD->open_angle = 65536.0 / (2 *M_PI) * m_phase_now_override;
			vTaskDelay(MS_TO_TICKS(2));
			int hall = HALL_M1.HallState;
			sin_hall[hall] += sinf(m_phase_now_override);
			cos_hall[hall] += cosf(m_phase_now_override);
			hall_iterations[hall]++;
		}
	}

	// Reverse
	for (int i = 0;i < 3;i++) {
		for (int j = 360;j >= 0;j--) {
			float m_phase_now_override = (float)j * M_PI / 180.0;
			pMCI[M1]->pSTC->SPD->open_angle =65536.0 / (2 *M_PI) * m_phase_now_override;
			vTaskDelay(MS_TO_TICKS(2));
			int hall = HALL_M1.HallState;
			sin_hall[hall] += sinf(m_phase_now_override);
			cos_hall[hall] += cosf(m_phase_now_override);
			hall_iterations[hall]++;
		}
	}


	currComp.q = 0;
	MCI_SetCurrentReferences(pMCI[M1],currComp);
	VescToSTM_set_open_loop(false, 0, 0);

	int fails = 0;
	for(int i = 0;i < 8;i++) {
		if (hall_iterations[i] > 30) {
			float ang = atan2f(sin_hall[i], cos_hall[i]) * 180.0 / M_PI;
			utils_norm_angle(&ang);
			hall_table[i] = (uint8_t)(ang * 255.0 / 360.0);
		} else {
			hall_table[i] = 255;
			fails++;
		}
	}
	HALL_M1.PhaseShift = old_phase_shift;
	VescToSTM_enable_timeout(true);
	return fails == 2;
}

/**
 * Lock the motor with a current and sample the voltage and current to
 * calculate the motor resistance.
 *
 * @param current
 * The locking current.
 *
 * @param samples
 * The number of samples to take.
 *
 * @param stop_after
 * Stop motor after finishing the measurement. Otherwise, the current will
 * still be applied after returning. Setting this to false is useful if you want
 * to run this function again right away, without stopping the motor in between.
 *
 * @return
 * The calculated motor resistance.
 */
float tune_foc_measure_resistance(float current, int samples) {

	current *= 1000; //A to mA

	// Disable timeout
	VescToSTM_enable_timeout(false);

	MCI_ExecTorqueRamp(pMCI[M1], 0, 0);
	MCI_StartMotor( pMCI[M1] );

	qd_t currComp;
	currComp.q = 0;
	currComp.d = 0;

	// Lock the motor
	VescToSTM_set_open_loop(true, 0, 0);
	int old_phase_shift = HALL_M1.PhaseShift;
	HALL_M1.PhaseShift = 0;


	for (int i = 0;i < 1000;i++) {
		currComp.q = current_to_torque((float)i * current / 1000.0);
		MCI_SetCurrentReferences(pMCI[M1],currComp);
		vTaskDelay(MS_TO_TICKS(1));
	}

	// Wait for the current to rise and the motor to lock.
	vTaskDelay(MS_TO_TICKS(200));

	// Sample

	int32_t Vq = pMCI[M1]->pFOCVars->Vq_avg;
	int32_t Vd = pMCI[M1]->pFOCVars->Vd_avg;
	int32_t Iq = pMCI[M1]->pFOCVars->Iq_avg;
	int32_t Id = pMCI[M1]->pFOCVars->Id_avg;

	float Vin = VescToSTM_get_bus_voltage();
	float fVq = Vin / 32768.0 * (float)Vq;
	float fVd = Vin / 32768.0 * (float)Vd;
	float fIq = (float)Iq / CURRENT_FACTOR_A;
	float fId = (float)Id / CURRENT_FACTOR_A;

	float current_avg = sqrtf(SQ(fIq) + SQ(fId));
	float voltage_avg = sqrtf(SQ(fVq) + SQ(fVd));

	// UnLock the motor
	VescToSTM_set_open_loop(false, 0, 0);
	HALL_M1.PhaseShift = old_phase_shift;

	// Enable timeout
	VescToSTM_enable_timeout(true);

	return (voltage_avg / current_avg) / (SQRT_3) / 2.0;
}

/**
 * Lock the motor with a current and sample the voltage and current to
 * calculate the motor resistance.
 *
 * @param current
 * The locking current.
 *
 * @param samples
 * The number of samples to take.
 *
 * @param stop_after
 * Stop motor after finishing the measurement. Otherwise, the current will
 * still be applied after returning. Setting this to false is useful if you want
 * to run this function again right away, without stopping the motor in between.
 *
 * @return
 * The calculated motor resistance.
 */
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];
float tune_foc_measure_inductance(float voltage, float * used_current, uint32_t samples) {

	float voltage_calc = 32768.0 / VescToSTM_get_bus_voltage() * voltage;

	int32_t volt_int = voltage_calc;

	// Disable timeout
	VescToSTM_enable_timeout(false);

	MCI_ExecTorqueRamp(pMCI[M1], 0, 0);
	MCI_StartMotor( pMCI[M1] );


	// Lock the motor
	pMCI[M1]->pSTC->SPD->open_loop = true;
	pMCI[M1]->pSTC->SPD->open_angle =0;
	int old_phase_shift = HALL_M1.PhaseShift;
	HALL_M1.PhaseShift = 0;


	int16_t IqUpperLim = pPIDIq[M1]->hUpperOutputLimit;
	int16_t IqLowerLim = pPIDIq[M1]->hLowerOutputLimit;
	int16_t IdUpperLim = pPIDId[M1]->hUpperOutputLimit;
	int16_t IdLowerLim = pPIDId[M1]->hLowerOutputLimit;

	pPIDIq[M1]->hUpperOutputLimit = 0;
	pPIDIq[M1]->hLowerOutputLimit = 0;

	vTaskDelay(MS_TO_TICKS(50));

	int32_t Iq = 0;
	int32_t Id = 0;
	volatile float current_avg=0.0;

	for(uint32_t i=0;i<samples;i++){
		pPIDIq[M1]->hUpperOutputLimit = volt_int;
		pPIDIq[M1]->hLowerOutputLimit = volt_int;
		vTaskDelay(1);
		Iq += abs(pMCI[M1]->pFOCVars->Iqd.q);
		Id += abs(pMCI[M1]->pFOCVars->Iqd.d);
		pPIDIq[M1]->hUpperOutputLimit = 0;
		pPIDIq[M1]->hLowerOutputLimit = 0;
		vTaskDelay(MS_TO_TICKS(10));
	}
	Iq /= samples;
	Id /= samples;
	float fIq = (float)Iq / CURRENT_FACTOR_A;
	float fId = (float)Id / CURRENT_FACTOR_A;
	current_avg = sqrtf(SQ(fIq) + SQ(fId));
	if(used_current!=NULL){
		*used_current = current_avg;
	}

	float inductance = voltage * (0.5e3) / current_avg;
	vTaskDelay(MS_TO_TICKS(100));

	// UnLock the motor
	pPIDIq[M1]->hUpperOutputLimit = IqUpperLim;
	pPIDIq[M1]->hLowerOutputLimit = IqLowerLim;
	pPIDId[M1]->hUpperOutputLimit = IdUpperLim;
	pPIDId[M1]->hLowerOutputLimit = IdLowerLim;
	pPIDIq[M1]->wIntegralTerm = 0;
	pPIDIq[M1]->wPrevProcessVarError = 0;
	pPIDId[M1]->wIntegralTerm = 0;
	pPIDId[M1]->wPrevProcessVarError = 0;
	pMCI[M1]->pSTC->SPD->open_loop = false;
	pMCI[M1]->pSTC->SPD->open_angle =0;
	HALL_M1.PhaseShift = old_phase_shift;

	// Enable timeout
	VescToSTM_enable_timeout(true);

	return inductance / SQRT_3 / 2.0;
}

/**
 * Measure the motor inductance with short voltage pulses. The difference from the
 * other function is that this one will aim for a specific measurement current. It
 * will also use an appropriate switching frequency.
 *
 * @param curr_goal
 * The measurement current to aim for.
 *
 * @param samples
 * The number of samples to average over.
 *
 * @param *curr
 * The current that was used for this measurement.
 *
 * @return
 * The average d and q axis inductance in uH.
 */
float tune_foc_measure_inductance_current(float curr_goal, int samples) {
	float duty_last = 0.0;
	for (float i = 0.2;i < 10.0;i *= 1.5) {
		float i_tmp;
		tune_foc_measure_inductance(i, &i_tmp, 10);

		duty_last = i;
		if (i_tmp >= curr_goal) {
			break;
		}
	}

	float ind = tune_foc_measure_inductance(duty_last, NULL, samples);
	return ind;
}

/**
 * Automatically measure the resistance and inductance of the motor with small steps.
 *
 * @param res
 * The measured resistance in ohm.
 *
 * @param ind
 * The measured inductance in microhenry.
 *
 * @return
 * True if the measurement succeeded, false otherwise.
 */
bool tune_foc_measure_res_ind(float *res, float *ind) {
	float current = mc_conf.l_current_max / 2.0;

	*res = tune_foc_measure_resistance(current, 3000);
	*ind = tune_foc_measure_inductance_current(current, 100);

	return true;
}


/**
 * Try to measure the motor flux linkage using open loop FOC control.
 *
 * @param current
 * The Q-axis current to spin up the motor.
 *
 * @param duty
 * Duty cycle % to measure at
 *
 * @param erpm_per_sec
 * Acceleration rate
 *
 * @param res
 * The motor phase resistance.
 *
 * @param linkage
 * The calculated flux linkage.
 *
 * @param linkage_undriven
 * Flux linkage measured while the motor was undriven.
 *
 * @param undriven_samples
 * Number of flux linkage samples while the motor was undriven.
 *
 * @return
 * True for success, false otherwise.
 */
bool tune_foc_measure_flux_linkage_openloop(float current, float duty,
		float erpm_per_sec, float res, float ind, float *linkage,
		float *linkage_undriven, float *undriven_samples) {
	bool result = false;
	current *= 1000; //A to mA

	// Disable timeout
	VescToSTM_enable_timeout(false);

	MCI_ExecTorqueRamp(pMCI[M1], 0, 0);
	MCI_StartMotor( pMCI[M1] );

	qd_t currComp;
	currComp.q = 0;
	currComp.d = 0;


	int cnt = 0;
	float rpm_now = 0;

	VescToSTM_set_open_loop(true, 0, 0);
	// Start by locking the motor
	for (int i = 0;i < 1000;i++) {
		currComp.q = current_to_torque((float)i * current / 1000.0);
		MCI_SetCurrentReferences(pMCI[M1],currComp);
		vTaskDelay(MS_TO_TICKS(1));
	}

	float duty_still = 0;
	float samples = 0;
	for (int i = 0;i < 1000;i++) {
		duty_still += fabsf(VescToSTM_get_duty_cycle_now());
		samples += 1.0;
		vTaskDelay(MS_TO_TICKS(1));
	}

	duty_still /= samples;
	float duty_max = 0.0;
	const int max_time = 15000;

	while (fabsf(VescToSTM_get_duty_cycle_now()) < duty) {
		rpm_now += erpm_per_sec / 1000.0;
		VescToSTM_set_open_loop_rpm(rpm_now);

		vTaskDelay(MS_TO_TICKS(1));
		cnt++;

		float duty_now = fabsf(VescToSTM_get_duty_cycle_now());

		if (duty_now > duty_max) {
			duty_max = duty_now;
		}

		if (cnt >= max_time) {
			*linkage = -1.0;
			break;
		}

		if (cnt > 4000 && duty_now < (duty_max * 0.7)) {
			cnt = max_time;
			*linkage = -2.0;
			break;
		}

		if (cnt > 4000 && duty < duty_still * 1.1) {
			cnt = max_time;
			*linkage = -3.0;
			break;
		}

		if (rpm_now >= 12000) {
			break;
		}
	}

	vTaskDelay(2000);

	if (cnt < max_time) {
		float vq_avg = 0.0;
		float vd_avg = 0.0;
		float iq_avg = 0.0;
		float id_avg = 0.0;
		float samples2 = 0.0;

		for (int i = 0;i < 10000;i++) {
			vq_avg += VescToSTM_get_Vq();
			vd_avg += VescToSTM_get_Vd();
			iq_avg += VescToSTM_get_iq();
			id_avg += VescToSTM_get_id();
			samples2 += 1.0;
			vTaskDelay(1);
		}

		vq_avg /= samples2;
		vd_avg /= samples2;
		iq_avg /= samples2;
		id_avg /= samples2;

		float rad_s = rpm_now * ((2.0 * M_PI) / 60.0);
		float v_mag = sqrtf(SQ(vq_avg) + SQ(vd_avg));
		float i_mag = sqrtf(SQ(iq_avg) + SQ(id_avg));
		*linkage = (v_mag - (2.0 / 3.0) * res * i_mag) / rad_s - (2.0 / 3.0) * i_mag * ind;

//		mc_conf.foc_motor_r = res;
//		mc_conf.foc_motor_l = ind;
//		mc_conf.foc_motor_flux_linkage = *linkage;
//		mc_conf.foc_observer_gain = 0.5e3 / SQ(*linkage);
		//mc_interface_set_configuration(mcconf);
		vTaskDelay(MS_TO_TICKS(500));
		currComp.q = 0;

		MCI_ExecTorqueRamp(pMCI[M1], 0, 0);

		vTaskDelay(1);
		VescToSTM_set_open_loop(false, 0, 0);
		vTaskDelay(MS_TO_TICKS(5));

		float linkage_sum = 0.0;
		float linkage_samples = 0.0;
		for (int i = 0;i < 20000;i++) {
			float rad_s_now = VescToSTM_get_erpm_fast() * ((2.0 * M_PI) / 60.0); //Faster
			if (fabsf(VescToSTM_get_duty_cycle_now()) < 0.01) {
				break;
			}

			linkage_sum += VescToSTM_get_Vq() / SQRT_3 / rad_s_now;
			linkage_samples += 1.0;
			vTaskDelay(MS_TO_TICKS(1));
		}

		*undriven_samples = linkage_samples;

		if (linkage_samples > 0) {
			*linkage_undriven = linkage_sum / linkage_samples;
		} else {
			*linkage_undriven = 0.0;
		}

		result = true;
	}


	VescToSTM_enable_timeout(true);
	return result;
}
