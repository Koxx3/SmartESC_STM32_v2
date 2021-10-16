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


const uint8_t hall_arr[8] = {0,5,1,3,2,6,4,7};

struct _sensor{
	uint16_t angle_forw;
	uint16_t angle_back;
};

#define _60DEG 60*(65536.0/360.0)
#define _90DEG 90*(65536.0/360.0)

static int16_t current_to_torque(int32_t curr_ma){
	float ret = curr_ma * CURRENT_FACTOR_mA;
	return ret;

}

uint8_t tune_hall_detect(uint32_t current, uint8_t *hall_table){


	MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);

	MCI_StartMotor( pMCI[M1] );

	struct _sensor data[7];

	qd_t currComp;
	currComp = MCI_GetIqdref(pMCI[M1]);
	currComp.q = current_to_torque(current);


	vTaskDelay(pdMS_TO_TICKS(5000));

	pMCI[M1]->pSTC->SPD->open_loop = true;
	HALL_M1.PhaseShift=0;

	MCI_SetCurrentReferences(pMCI[M1],currComp);
	pMCI[M1]->pSTC->SPD->open_angle =0;

	vTaskDelay(pdMS_TO_TICKS(2000));

	uint32_t old_hall = HALL_M1.HallState;
	uint16_t angle =0;
	uint8_t state_cnt = 1;

	uint8_t hall_lut[8] = {0,1,2,3,4,5,6,7};

	for(uint32_t i=0;i<8;i++){
		HALL_M1.lut[i] = hall_lut[i];
	}


	//Getting sensor configuration...
	for(uint32_t i=0;i<0x10000;i+=16){
		pMCI[M1]->pSTC->SPD->open_angle = angle+=16;
		vTaskDelay(1);
	}
	angle=0;
	pMCI[M1]->pSTC->SPD->open_angle = angle;
	vTaskDelay(pdMS_TO_TICKS(!000));
	for(uint32_t i=0;i<0x10000;i+=8){

		pMCI[M1]->pSTC->SPD->open_angle = angle+=8;
		vTaskDelay(1);
		if(old_hall != HALL_M1.HallState){
			if(state_cnt>6){
				pMCI[M1]->pSTC->SPD->open_loop = false;
				MCI_StopMotor( pMCI[M1] );
				return pdFAIL;
			}
			hall_lut[HALL_M1.HallState] = hall_arr[state_cnt];
			state_cnt++;
			old_hall=HALL_M1.HallState;
		}
	}

	for(uint32_t i=0;i<8;i++){
		HALL_M1.lut[i] = hall_lut[i];
	}

	angle=0;

	pMCI[M1]->pSTC->SPD->open_angle = angle;
	vTaskDelay(pdMS_TO_TICKS(3000));
	old_hall = HALL_M1.HallState;

	//Getting sensor offset...;
	for(uint8_t u=0;u<1;u++){
		for(uint32_t i=0;i<0xFFFF;i+=4){
			pMCI[M1]->pSTC->SPD->open_angle = angle+=4;
			vTaskDelay(1);
			if(old_hall != HALL_M1.HallState){
				data[HALL_M1.HallState].angle_forw = angle;
				old_hall=HALL_M1.HallState;
			}
		}
	}
	//Backwards;
	old_hall = HALL_M1.HallState;
	vTaskDelay(pdMS_TO_TICKS(500));
	for(uint8_t u=0;u<1;u++){
		for(uint32_t i=0;i<0xFFFF;i+=4){
			pMCI[M1]->pSTC->SPD->open_angle = angle-=4;
			vTaskDelay(1);
			if(old_hall != HALL_M1.HallState){
				switch(HALL_M1.HallState){
				case 6:
					data[4].angle_back = angle;
					break;
				case 2:
					data[6].angle_back = angle;
					break;
				case 3:
					data[2].angle_back = angle;
					break;
				case 1:
					data[3].angle_back = angle;
					break;
				case 5:
					data[1].angle_back = angle;
					break;
				case 4:
					data[5].angle_back = angle;
					break;
				}
				old_hall=HALL_M1.HallState;
			}
		}
	}
	float el_angle = _60DEG; //60deg
	float phase_shift=0;
	//Calculating phase shift
	for(uint8_t u=1;u<7;u++){
		uint16_t middle = data[hall_arr[u]].angle_back + ((data[hall_arr[u]].angle_forw - data[hall_arr[u]].angle_back)/2);
		phase_shift += (el_angle-middle);
		el_angle += _60DEG; //60deg

	}
	phase_shift/=6;
	phase_shift+=_90DEG;
	HALL_M1.PhaseShift = phase_shift;

//	ttprintf("Set phase shift to: %.2f\r\n", (float)HALL_M1.PhaseShift/(65536.0/360.0));

	pMCI[M1]->pSTC->SPD->open_loop = false;
	MCI_StopMotor( pMCI[M1] );
	vTaskDelay(pdMS_TO_TICKS(1000));

	currComp = MCI_GetIqdref(pMCI[M1]);
	currComp.q = 0;
	currComp.d = 0;
	MCI_SetCurrentReferences(pMCI[M1],currComp);
	MCI_StartMotor( pMCI[M1] );
	for(int i=0; i<8;i++){
		hall_table[i] = HALL_M1.lut[i];
	}
	return pdPASS;
//	return TERM_CMD_EXIT_SUCCESS;

}
