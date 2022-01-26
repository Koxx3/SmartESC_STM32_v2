/**

  */

/* Includes ------------------------------------------------------------------*/
#include "r_divider_phase_voltage_sensor.h"
#include "regular_conversion_manager.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @defgroup RDividerBusVoltageSensor Resistor Divider Bus Voltage Sensor
  * @brief Resistor Divider Bus Voltage Sensor implementation
  *
  * @todo Document the Resistor Divider Bus Voltage Sensor "module".
  *
  * @{
  */

/**
  * @brief  It initializes bus voltage conversion (ADC, ADC channel, conversion time. 
    It must be called only after PWMC_Init.
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
__weak void RVPS_Init( RPhase_Handle_t * pHandle )
{
  /* Need to be register with RegularConvManager */
	for(int i=0;i<3;i++){
		pHandle->convHandle[i] = RCM_RegisterRegConv(&pHandle->Phase[i]);
	}
  /* Check */

}


/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
__weak void RVPS_get_phases( RPhase_Handle_t * pHandle )
{
  uint16_t hAux;
  for(int i=0;i<3;i++){
  hAux = RCM_ExecRegularConv(pHandle->convHandle[i]);
	if ( hAux != 0xFFFF ){
		pHandle->voltages[i] = hAux;
	}
  }
}

