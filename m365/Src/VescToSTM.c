
#include "VescToSTM.h"
#include "mc_interface.h"
#include "stm32f1xx_hal.h"
#include "crc.h"
#include "defines.h"
#include "drive_parameters.h"
#include "mc_config.h"
#include "FreeRTOS.h"
#include "task.h"

int32_t current_to_torque(int32_t curr_ma){
	float ret = curr_ma * CURRENT_FACTOR_mA;
	return ret;

}

qd_t currComp;
uint32_t is_braking=0;
uint32_t last_reset=0;

void VescToSTM_handle_brake(){

	if(is_braking){
		if(VescToSTM_get_rpm() < 10){
			is_braking=0;
			pMCI[M1]->pSTC->SPD->open_angle = pMCI[M1]->pSTC->SPD->hElAngle;
			pMCI[M1]->pSTC->SPD->open_loop = true;
		}

	}

}

void VescToSTM_timeout_reset(){
	last_reset = xTaskGetTickCount();
};

void VescToSTM_handle_timeout(){
	if((xTaskGetTickCount() - last_reset) > 1000){
		if(pMCI[M1]->pSTC->SPD->open_loop == false){
			VescToSTM_set_torque(0);
		}
	}

};

void VescToSTM_set_torque(int32_t current){
	pMCI[M1]->pSTC->SPD->open_loop = false;
	is_braking = 0;
	if(MCI_GetControlMode(pMCI[M1]) != STC_TORQUE_MODE){
		MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);
	}
	int16_t q = current_to_torque(current);

	if(q != currComp.q){
		if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
			q = SpeednTorqCtrlM1.MaxPositiveTorque;
		}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
			q = SpeednTorqCtrlM1.MinNegativeTorque;
		}
		currComp.q = q;
		MCI_SetCurrentReferences(pMCI[M1],currComp);
	}
}

void VescToSTM_set_brake(int32_t current){

	if(MCI_GetControlMode(pMCI[M1]) != STC_TORQUE_MODE){
		MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);
	}

	if(MCI_GetAvrgMecSpeedUnit( pMCI[M1] ) == 0){
		pMCI[M1]->pSTC->SPD->open_angle = pMCI[M1]->pSTC->SPD->hElAngle;
		pMCI[M1]->pSTC->SPD->open_loop = true;
	}


	int16_t q = current_to_torque(current);


	if(currComp.q < 0){
		q = q *-1;
		is_braking = q;
	}
	is_braking = q;
	if(q != currComp.q){
		if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
			q = SpeednTorqCtrlM1.MaxPositiveTorque;
		}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
			q = SpeednTorqCtrlM1.MinNegativeTorque;
		}
		currComp.q = q;
		MCI_SetCurrentReferences(pMCI[M1],currComp);
	}
}

void VescToSTM_set_speed(int32_t rpm){
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
	return pMCI[M1]->pFOCVars->Iqd.q / CURRENT_FACTOR;
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
