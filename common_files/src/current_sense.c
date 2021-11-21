#include "current_sense.h"
#include "product.h"
#include "utils.h"

uint16_t offset_curr = 0;

__weak void CURR_Calc_Sample(CURR_Handle_t * pHandle );
__weak uint16_t CURR_SetFaultState( CURR_Handle_t * pHandle );


__weak void CURR_Init( CURR_Handle_t * pHandle )
{

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    /* Need to be register with RegularConvManager */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->CurrRegConv);
    //CURR_Calc_Sample(pHandle);
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
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;


  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);
    if ( hAux != 0xFFFFu )
    {
    	pHandle->aBuffer[pHandle->index] = hAux;
		wtemp = 0;
		for ( i = 0; i < 10u; i++ )
		{
		  wtemp += pHandle->aBuffer[i];
		}
		wtemp /= 10u;
		pHandle->hAvCurrent = ( uint16_t )wtemp;

		if ( pHandle->index < 10u - 1 )
		{
		  pHandle->index++;
		}
		else
		{
		  pHandle->index = 0;
		}
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

__weak float CURR_GetCurrent( CURR_Handle_t * pHandle )
{
  float temp;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {

	temp = pHandle->hAvCurrent;
	temp /= 275275;

	return temp;
  } else {
	  temp = 0;
  }

  return temp;
}

__weak uint16_t CURR_CheckCurrent( CURR_Handle_t * pHandle )
{
  return ( pHandle->hFaultState );
}
