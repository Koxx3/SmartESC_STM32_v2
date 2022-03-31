/* Includes ------------------------------------------------------------------*/
#include "hfi_speed_pos_fdbk.h"
#include <stdlib.h>


/* Private defines -----------------------------------------------------------*/
#define SIGN(x)				((x < 0) ? -1 : 1)

/* Private function prototypes -----------------------------------------------*/


void HFI_Init( HFI_Handle_t * pHandle )
{
	pHandle->hfi_cnt = 0;
	pHandle->sign_last = 1;
	pHandle->d_cnt = 0;
	HFI_update_busvoltage(pHandle, 65536);
	return;
}

alphabeta_t HFI_Inject( HFI_Handle_t * pHandle, alphabeta_t Valphabeta, alphabeta_t Ialphabeta, qd_t Iqd){

	switch(pHandle->inject_seq){
	case 0:
		if(abs(Iqd.q) > pHandle->hfi_hyst){
			pHandle->hfi_cnt++;
			pHandle->hfi_cnt = 3200;
			pHandle->sign_last = SIGN(Iqd.q);
		}else{
			pHandle->hfi_cnt=0;
		}

		int32_t sample_now = (((int32_t)pHandle->trig.hCos * (int32_t)Ialphabeta.alpha)) +
					 	 	 (((int32_t)pHandle->trig.hSin * (int32_t)Ialphabeta.beta));
		int32_t di = (sample_now - pHandle->sample_prev)>>8;

		pHandle->d_out += pHandle->sign_last * (di-pHandle->inductance);
		pHandle->d_cnt++;

		Valphabeta.alpha -= pHandle->Vinject.alpha;
		Valphabeta.beta -= pHandle->Vinject.beta;
		pHandle->inject_seq++;
		break;
	case 1:
		Valphabeta.alpha -= pHandle->Vinject.alpha;
		Valphabeta.beta -= pHandle->Vinject.beta;
		pHandle->inject_seq++;
		break;
	case 2:
		  pHandle->sample_prev = 	(((int32_t)pHandle->trig.hCos * (int32_t)Ialphabeta.alpha)) +
									(((int32_t)pHandle->trig.hSin * (int32_t)Ialphabeta.beta));
		  Valphabeta.alpha += pHandle->Vinject.alpha;
		  Valphabeta.beta += pHandle->Vinject.beta;
		  pHandle->inject_seq++;
		break;
	case 3:
		Valphabeta.alpha += pHandle->Vinject.alpha;
		Valphabeta.beta += pHandle->Vinject.beta;
		pHandle->inject_seq = 0;
		break;
	}

	return Valphabeta;
}

void HFI_update(HFI_Handle_t * pHandle){
	if(pHandle->d_cnt > pHandle->samples_avg){

		pHandle->inductance_diff = pHandle->d_out / pHandle->d_cnt;
		pHandle->d_out=0;
		pHandle->d_cnt=0;

		pHandle->hfi_angle += pHandle->speed_avg/256;
		pHandle->hfi_angle -= pHandle->inductance_diff/2;

		int16_t diff = pHandle->hElAngle_last - pHandle->hfi_angle;
		if (diff > 32000) {
				diff -= 65536;
			} else if (diff < -32000) {
				diff += 65536;
			}


		pHandle->speed_avg = (diff + (8 - 1) * pHandle->speed_avg) >> 3;
		pHandle->_Super.hAvrMecSpeedUnit = pHandle->speed_avg+8;

		pHandle->hElAngle_last = pHandle->hfi_angle;

		pHandle->trig = MCM_Trig_Functions( pHandle->hfi_angle + (pHandle->sign_last*8192));  //8192 = 45 deg
		pHandle->Vinject.alpha = ((int32_t)pHandle->hfi_duty * (int32_t)pHandle->trig.hCos) >> 15;
		pHandle->Vinject.beta = ((int32_t)pHandle->hfi_duty * (int32_t)pHandle->trig.hSin) >> 15;
		pHandle->_Super.hElAngle = 65536-pHandle->hfi_angle;
	}
}

bool HFI_is_ready(HFI_Handle_t * pHandle){
	 if(pHandle->hfi_cnt<2000){
		  return false;
	  }else{
		  return true;
	  }
}

int16_t HFI_get_angle(HFI_Handle_t * pHandle){
	return pHandle->_Super.hElAngle;
}

void HFI_update_busvoltage(HFI_Handle_t * pHandle, int32_t volt_d){
	pHandle->hfi_duty = 65536 * pHandle->hfi_voltage / volt_d;
	//pHandle->hfi_duty = 1000;
}

