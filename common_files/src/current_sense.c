#include "current_sense.h"
#include "product.h"
#include "utils.h"

uint16_t offset_curr = 0;

__weak void CURR_Calc_Sample(CURR_Handle_t * pHandle );
__weak uint16_t CURR_SetFaultState( CURR_Handle_t * pHandle );

__weak void CURR_Init( CURR_Handle_t * pHandle ){
  uint16_t hAux;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    /* Need to be register with RegularConvManager */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->CurrRegConv);

    for ( pHandle->PolarizationCounter = 0; pHandle->PolarizationCounter < 100u; ) {
    	  hAux = RCM_ExecRegularConv(pHandle->convHandle);
    	  if(hAux > 0 && hAux != 0xFFFFu){
    		  pHandle->CurrentOffset = (hAux + pHandle->CurrentOffset) / 2;
    		  pHandle->PolarizationCounter++;
    	  }
      }
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hFaultState = MC_NO_ERROR;
  }

}

__weak uint16_t CURR_CalcMainCurrent( CURR_Handle_t * pHandle ){
  uint16_t hAux;
  int32_t wtemp;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
	  hAux = RCM_ExecRegularConv(pHandle->convHandle);
	  if ( hAux != 0xFFFFu )
	  {
		wtemp =  ( uint32_t )( 250u ) - 1u;
		wtemp *= ( uint32_t ) ( pHandle->hAvCurrent );
		wtemp += hAux;
		wtemp /= ( uint32_t )( 250u );

		pHandle->hAvCurrent = ( uint16_t ) wtemp;
	  }

    pHandle->hFaultState = CURR_SetFaultState( pHandle );
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hFaultState = MC_NO_ERROR;
  }

  return ( pHandle->hFaultState );
}

__weak uint16_t CURR_SetFaultState( CURR_Handle_t * pHandle ){
  uint16_t hFault;
    hFault = MC_NO_ERROR;
  return hFault;
}

__weak float CURR_GetCurrent( CURR_Handle_t * pHandle ){
  float wTemp;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
	//temp = pHandle->hAvCurrent - (pHandle->CurrentOffset+ 20);
	//temp /= CURRENT_FACTOR_A;
	//temp += 0.000000000001;

	  wTemp = ( int32_t )( pHandle->hAvCurrent );
	  wTemp -= ( int32_t )(uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE);
	  wTemp *= (uint16_t)(ADC_REFERENCE_VOLTAGE/0.018);
	  wTemp = wTemp / 65536 + 4.2;

  } else {
	  wTemp = 0;
  }

  return wTemp;
}

__weak uint16_t CURR_CheckCurrent( CURR_Handle_t * pHandle )
{
  return ( pHandle->hFaultState );
}
