#include "current_sense.h"
#include "product.h"

uint16_t offset_curr = 0;

__weak uint16_t CURR_SetFaultState( CURR_Handle_t * pHandle );


__weak void CURR_Init( CURR_Handle_t * pHandle )
{

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    /* Need to be register with RegularConvManager */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->CurrRegConv);
    CURR_Calc_Sample(pHandle);
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hFaultState = MC_NO_ERROR;
  }

}


__weak void CURR_Calc_Sample(CURR_Handle_t * pHandle ){
	int i;
	for (i = 1; i < 1000; ++i) {
		int16_t hAux = RCM_ExecRegularConv(pHandle->convHandle);
		offset_curr = (hAux + offset_curr) / 2;
	}
}


__weak uint16_t CURR_CalcMainCurrent( CURR_Handle_t * pHandle )
{
  uint16_t wtemp;
  int16_t hAux;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);
    if ( hAux != 0xFFFFu )
    {
      //wtemp =  ( uint32_t )( pHandle->hLowPassFilterBW ) - 1u;
      //wtemp *= ( uint32_t ) ( pHandle->hAvTemp_d );
      //wtemp += hAux;
      //wtemp /= ( uint32_t )( pHandle->hLowPassFilterBW );
    	pHandle->test = (hAux - offset_curr) / CURRENT_FACTOR_A;

      pHandle->hAvCurrent = ( int16_t ) hAux;
    }

    pHandle->hFaultState = CURR_SetFaultState( pHandle );
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hFaultState = MC_NO_ERROR;
  }

  return ( pHandle->hFaultState );
}

__weak uint16_t CURR_SetFaultState( CURR_Handle_t * pHandle )
{
  uint16_t hFault;
    hFault = MC_NO_ERROR;
  return hFault;
}

__weak uint16_t CURR_CheckCurrent( CURR_Handle_t * pHandle )
{
  return ( pHandle->hFaultState );
}
