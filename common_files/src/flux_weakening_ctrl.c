#include "flux_weakening_ctrl.h"
#include "main.h"

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation.
  * @param  pHandle Flux weakening init strutcture.
  * @param  pPIDSpeed Speed PID structure.
  * @param  pPIDFluxWeakeningHandle FW PID structure.
  * @retval none.
  */
__weak void FW_Init( FW_Handle_t * pHandle, PID_Handle_t * pPIDSpeed, PID_Handle_t * pPIDFluxWeakeningHandle ){
	pHandle->pFluxWeakeningPID = pPIDFluxWeakeningHandle;
	pHandle->pSpeedPID = pPIDSpeed;
	pHandle->hFW_V_Ref = pHandle->hDefaultFW_V_Ref;
}

/**
  * @brief  It should be called before each motor restart and clears the Flux
  *         weakening internal variables with the exception of the target
  *         voltage (hFW_V_Ref).
  * @param  pHandle Flux weakening init strutcture.
  * @retval none
  */
__weak void FW_Clear( FW_Handle_t * pHandle )
{
	qd_t NULL_qd = {(int16_t)0, (int16_t)0};

	pHandle->AvVoltAmpl = 0;
	pHandle->AvVolt_qd = NULL_qd;

	PID_SetIntegralTerm( pHandle->pFluxWeakeningPID, 0 );
}

/**
  * @brief  It low-pass filters both the Vqd voltage components. Filter
  *         bandwidth depends on hVqdLowPassFilterBW parameter
  * @param  pHandle Flux weakening init strutcture.
  * @param  Vqd Voltage componets to be averaged.
  * @retval none
  */
__weak void FW_DataProcess( FW_Handle_t * pHandle, qd_t Vqd )
{
	  int16_t v3 = Vqd.d + (pHandle->hVqdLowPassFilterBW - 1) * pHandle->AvVolt_qd.d;
	  int16_t v4 = Vqd.q + (pHandle->hVqdLowPassFilterBW - 1) * pHandle->AvVolt_qd.q;
	  pHandle->AvVolt_qd.q = v4 >> pHandle->hVqdLowPassFilterBWLOG;
	  pHandle->AvVolt_qd.d = v3 >> pHandle->hVqdLowPassFilterBWLOG;
}

/**
  * @brief  It computes Iqdref according the flux weakening algorithm.  Inputs
  *         are the starting Iqref components.
  *         As soon as the speed increases beyond the nominal one, fluxweakening
  *         algorithm take place and handles Idref value. Finally, accordingly
  *         with new Idref, a new Iqref saturation value is also computed and
  *         put into speed PI.
  * @param  pHandle Flux weakening init strutcture.
  * @param  Iqdref The starting current components that have to be
  *         manipulated by the flux weakening algorithm.
  * @retval qd_t Computed Iqdref.
  */
__weak qd_t FW_CalcCurrRef( FW_Handle_t * pHandle, qd_t Iqdref ){
	  int16_t AvVoltAmpl = MCM_Sqrt(pHandle->AvVolt_qd.d * pHandle->AvVolt_qd.d + pHandle->AvVolt_qd.q * pHandle->AvVolt_qd.q);

	  int16_t v8;
	  if (AvVoltAmpl > 0x7FFF )
		   v8 = ((pHandle->hMaxModule * pHandle->hFW_V_Ref) / 0x3E8) - 0x7FFF;
	  else
	 	   v8 = ((pHandle->hMaxModule * pHandle->hFW_V_Ref) / 0x3E8) - AvVoltAmpl;

	  pHandle->AvVoltAmpl = AvVoltAmpl;
	  int16_t v9 = PI_Controller(pHandle->pFluxWeakeningPID, v8);

	  if ( v9 < 0 ){
		  pHandle->AvVolt_qd.d = pHandle->hIdRefOffset;
	  }

	  if ( v9 >= 0 ){
		  pHandle->hIdRefOffset = pHandle->AvVolt_qd.d;
	  } else {
		  pHandle->AvVolt_qd.d += v9;
	  }

	  if (pHandle->hDemagCurrent < pHandle->AvVolt_qd.d){
		  pHandle->hDemagCurrent = pHandle->AvVolt_qd.d;
	  }

	  uint16_t v11 = pHandle->hDemagCurrent;
	  int v12 = MCM_Sqrt(pHandle->wNominalSqCurr - pHandle->hDemagCurrent * pHandle->hDemagCurrent);
	  int v13 = v12 * PID_GetKIDivisor(pHandle->pSpeedPID);

	  PID_SetLowerIntegralTermLimit(pHandle->pSpeedPID, -v13);
	  PID_SetUpperIntegralTermLimit(pHandle->pSpeedPID, v13);

	  qd_t output = {(int16_t)pHandle->AvVolt_qd.q, (int16_t)(v11 << 16)};
	  if ( pHandle->AvVolt_qd.q > v12 || (v12 = -v12, pHandle->AvVolt_qd.q < v12) ){
		  output.q = v12;
	  }

	  //return (unsigned __int16)v5 | (v11 << 16);
	  return output;
}




/**
  * @brief  Use this method to set a new value for the voltage reference used by
  *         flux weakening algorithm.
  * @param  pHandle Flux weakening init strutcture.
  * @param  uint16_t New target voltage value, expressend in tenth of percentage
  *         points of available voltage.
  * @retval none
  */
__weak void FW_SetVref( FW_Handle_t * pHandle, uint16_t hNewVref ){
	pHandle->hFW_V_Ref = hNewVref;
}

/**
  * @brief  It returns the present value of target voltage used by flux
  *         weakening algorihtm.
  * @param  pHandle Flux weakening init strutcture.
  * @retval int16_t Present target voltage value expressed in tenth of
  *         percentage points of available voltage.
  */
__weak uint16_t FW_GetVref( FW_Handle_t * pHandle ){
	return pHandle->hFW_V_Ref;
}

/**
  * @brief  It returns the present value of voltage actually used by flux
  *         weakening algorihtm.
  * @param  pHandle Flux weakening init strutcture.
  * @retval int16_t Present averaged phase stator voltage value, expressed
  *         in s16V (0-to-peak), where
  *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
  */
__weak int16_t FW_GetAvVAmplitude( FW_Handle_t * pHandle ){
	return pHandle->AvVoltAmpl;
}

/**
  * @brief  It returns the measure of present voltage actually used by flux
  *         weakening algorihtm as percentage of available voltage.
  * @param  pHandle Flux weakening init strutcture.
  * @retval uint16_t Present averaged phase stator voltage value, expressed in
  *         tenth of percentage points of available voltage.
  */
__weak uint16_t FW_GetAvVPercentage( FW_Handle_t * pHandle ){
	return (uint16_t)(1000 * pHandle->AvVoltAmpl / pHandle->hMaxModule);
}

