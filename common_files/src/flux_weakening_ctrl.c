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
	pHandle->AvAmpere_qd = NULL_qd;

	PID_SetIntegralTerm( pHandle->pFluxWeakeningPID, 0 );
}

/**
  * @brief  It low-pass filters both the Vqd voltage components. Filter
  *         bandwidth depends on hVqdLowPassFilterBW parameter
  * @param  pHandle Flux weakening init strutcture.
  * @param  Vqd Voltage componets to be averaged.
  * @retval none
  */
__weak void FW_DataProcess( FW_Handle_t * pHandle, qd_t Vqd, qd_t Iqd)
{
	  pHandle->AvVolt_qd.q = (Vqd.q + (pHandle->hVqdLowPassFilterBW - 1) * pHandle->AvVolt_qd.q) >> pHandle->hVqdLowPassFilterBWLOG;
	  pHandle->AvVolt_qd.d = (Vqd.d + (pHandle->hVqdLowPassFilterBW - 1) * pHandle->AvVolt_qd.d) >> pHandle->hVqdLowPassFilterBWLOG;
	  pHandle->AvAmpere_qd.q = (Iqd.q + (pHandle->hVqdLowPassFilterBW - 1) * pHandle->AvAmpere_qd.q) >> pHandle->hVqdLowPassFilterBWLOG;
	  pHandle->AvAmpere_qd.d = (Iqd.d + (pHandle->hVqdLowPassFilterBW - 1) * pHandle->AvAmpere_qd.d) >> pHandle->hVqdLowPassFilterBWLOG;
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
	  int32_t varError;

	  int32_t AvVoltAmpl = MCM_Sqrt(pHandle->AvVolt_qd.d * pHandle->AvVolt_qd.d + pHandle->AvVolt_qd.q * pHandle->AvVolt_qd.q);

	  if (AvVoltAmpl > 32767 ){
		  varError = ((pHandle->hMaxModule * pHandle->hFW_V_Ref) / 1000) - 32767;
	  } else {
		  varError = ((pHandle->hMaxModule * pHandle->hFW_V_Ref) / 1000) - AvVoltAmpl;
	  }

	  pHandle->AvVoltAmpl = AvVoltAmpl;
	  int16_t pid_output = PI_Controller(pHandle->pFluxWeakeningPID, varError);

	  if (pid_output < 0){
		  Iqdref.d = pHandle->hIdRefOffset;
	  }

	  if (pid_output >= 0){
		  pHandle->hIdRefOffset = Iqdref.d;
	  } else {
		  Iqdref.d += pid_output;
	  }


	  if ( Iqdref.d < pHandle->hDemagCurrent){
		  Iqdref.d = pHandle->hDemagCurrent;
	  }

	  int sqRoot = MCM_Sqrt(pHandle->wNominalSqCurr - pHandle->hDemagCurrent * pHandle->hDemagCurrent);
	  int lowerLimit = sqRoot * PID_GetKIDivisor(pHandle->pSpeedPID);
	  if(Iqdref.d != 0){
		  PID_SetLowerIntegralTermLimit(pHandle->pSpeedPID, -lowerLimit);
		  PID_SetUpperIntegralTermLimit(pHandle->pSpeedPID, lowerLimit);
		  if ( Iqdref.q > sqRoot || (sqRoot = -sqRoot, Iqdref.q < sqRoot) ){
		      Iqdref.q = sqRoot;
		  }
	  }


	  return Iqdref;
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
