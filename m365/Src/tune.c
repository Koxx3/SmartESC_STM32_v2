#include "tune.h"
#include "math.h"
#include "system.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "speed_pos_fdbk.h"
#include "drive_parameters.h"
#include "mc_stm_types.h"
#include "parameters_conversion.h"
#include "defines.h"
#include "FreeRTOS.h"
#include <string.h>
#include "utils.h"
#include "VescToSTM.h"


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
	pMCI[M1]->pSTC->SPD->open_loop = true;
	HALL_M1.PhaseShift=0;
	pMCI[M1]->pSTC->SPD->open_angle =0;


	for (int i = 0;i < 1000;i++) {
		currComp.q = current_to_torque((float)i * current / 1000.0);
		MCI_SetCurrentReferences(pMCI[M1],currComp);
		vTaskDelay(pdMS_TO_TICKS(1));
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
			vTaskDelay(pdMS_TO_TICKS(5));
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
			vTaskDelay(pdMS_TO_TICKS(5));
			int hall = HALL_M1.HallState;
			sin_hall[hall] += sinf(m_phase_now_override);
			cos_hall[hall] += cosf(m_phase_now_override);
			hall_iterations[hall]++;
		}
	}


	currComp.q = 0;
	MCI_SetCurrentReferences(pMCI[M1],currComp);
	pMCI[M1]->pSTC->SPD->open_loop = false;

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
	VescToSTM_enable_timeout(true);
	return fails == 2;
}
