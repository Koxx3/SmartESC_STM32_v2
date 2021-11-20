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
	  pHandle->AvVolt_qd.q = (Vqd.q + (pHandle->hVqdLowPassFilterBW - 1) * pHandle->AvVolt_qd.q) >> pHandle->hVqdLowPassFilterBWLOG;
	  pHandle->AvVolt_qd.d = (Vqd.d + (pHandle->hVqdLowPassFilterBW - 1) * pHandle->AvVolt_qd.d) >> pHandle->hVqdLowPassFilterBWLOG;
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
	  FW_Handle_t *v2;
	  uint16_t v3;
	  int16_t v6;
	  uint16_t v7;
	  int16_t v8;
	  int16_t v9;
	  signed int hDemagCurrent;
	  int16_t v12;
	  int16_t v13;

	  v2 = pHandle;
	  v3 = pHandle->hMaxModule * pHandle->hFW_V_Ref;

	  signed int D = (int16_t)Iqdref.d;
	  qd_t Q = Iqdref;


	  v6 = MCM_Sqrt(pHandle->AvVolt_qd.d * pHandle->AvVolt_qd.d + pHandle->AvVolt_qd.q * pHandle->AvVolt_qd.q);
	  v7 = v3 / 0x3E8;

	  if (v6 > 0x7FFF)
	    v8 = v7 - 0x7FFF;
	  else
	    v8 = v7 - v6;

	  v2->AvVoltAmpl = v6;
	  v9 = PI_Controller(v2->pFluxWeakeningPID, v8);
	  if ( v9 < 0 )
	    D = v2->hIdRefOffset;

	  hDemagCurrent = v2->hDemagCurrent;
	  if ( v9 >= 0 )
	    v2->hIdRefOffset = D;
	  else
	    D += v9;
	  if ( hDemagCurrent < D )
	    hDemagCurrent = D;

	  Q.d = hDemagCurrent;

	  v12 = MCM_Sqrt(v2->wNominalSqCurr - hDemagCurrent * hDemagCurrent);
	  v13 = v12 * PID_GetKIDivisor(v2->pSpeedPID);
	  PID_SetLowerIntegralTermLimit(v2->pSpeedPID, -v13);
	  PID_SetUpperIntegralTermLimit(v2->pSpeedPID, v13);

	  if ( Q.q > v12 || (v12 = -v12, Q.q < v12) )
		  Q.q = v12;

	  return Q;
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

