
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

static uint32_t odometer_divider;

int32_t current_to_torque(int32_t curr_ma){
	float ret = curr_ma * CURRENT_FACTOR_mA;
	return ret;

}

void VescToSTM_init_odometer(mc_configuration* mcconf){
	float div = (float)mcconf->si_motor_poles * ((mcconf->si_wheel_diameter * M_PI) / 10000.0);
	div = 1.0 / div;
	odometer_divider = div;
}

qd_t currComp;
uint32_t is_braking=0;
uint8_t last_direction = 0;
uint32_t last_reset=0;


float VescToSTM_get_pid_pos_now(){
	return 360.0 / 65536.0 * (float)pMCI[M1]->pSTC->SPD->hElAngle;
}


void VescToSTM_handle_brake(){

	/*if(is_braking){
		if((VescToSTM_get_rpm() < -10 && !last_direction) || (VescToSTM_get_rpm() > 10 && last_direction)){
			is_braking=0;
			pMCI[M1]->pSTC->SPD->open_angle = pMCI[M1]->pSTC->SPD->hElAngle;
			pMCI[M1]->pSTC->SPD->open_loop = true;
		}

	}*/

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
	int16_t q = current_to_torque(current);
	if(q != currComp.q){
		if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
			q = SpeednTorqCtrlM1.MaxPositiveTorque;
		}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
			q = SpeednTorqCtrlM1.MinNegativeTorque;
		}
		currComp.q = q;
		if(currComp.q > 0){
			pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)q * SP_KDDIV;
			pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)-q * SP_KDDIV;
			pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = currComp.q;
			pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = 0;
			MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_max_erpm / mc_conf.si_motor_poles , 0);

		}else{
			pMCI[M1]->pSTC->PISpeed->wUpperIntegralLimit = (uint32_t)-q * SP_KDDIV;
			pMCI[M1]->pSTC->PISpeed->wLowerIntegralLimit = (uint32_t)q * SP_KDDIV;
			pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = 0;
			pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = currComp.q;
			MCI_ExecSpeedRamp(pMCI[M1], mc_conf.l_min_erpm / mc_conf.si_motor_poles , 0);
		}
	}
}

void VescToSTM_set_brake(int32_t current){
	int16_t q = current_to_torque(current);
	if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
		q = SpeednTorqCtrlM1.MaxPositiveTorque;
	}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
		q = SpeednTorqCtrlM1.MinNegativeTorque;
	}
	currComp.q = q;
	if(currComp.q > 0){
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = currComp.q;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = -currComp.q;
		MCI_ExecSpeedRamp(pMCI[M1], 0 , 0);
	}else{
		pMCI[M1]->pSTC->PISpeed->hUpperOutputLimit = -currComp.q;
		pMCI[M1]->pSTC->PISpeed->hLowerOutputLimit = currComp.q;
		MCI_ExecSpeedRamp(pMCI[M1], 0 , 0);
	}
}

void VescToSTM_set_speed(int32_t rpm){
	last_direction = rpm > 0 ? 0 : 1;
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

uint32_t VescToSTM_get_odometer(){
	return HALL_M1.odometer / odometer_divider;
}

void VescToSTM_set_odometer(uint32_t meters){
	HALL_M1.odometer = meters * odometer_divider;
}

