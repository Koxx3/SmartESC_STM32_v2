#ifndef __CURRENT_SENSE_H
#define __CURRENT_SENSE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "regular_conversion_manager.h"

typedef struct
{
  SensorType_t  bSensorType;   /**< Type of instanced temperature. This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

  RegConv_t      CurrRegConv;

  int16_t hAvCurrent;

  float test;

  uint16_t hFaultState;        /**< Contains latest Fault code.**/

  uint8_t convHandle;            /*!< handle to the regular conversion */

} CURR_Handle_t;

__weak void CURR_Init( CURR_Handle_t * pHandle );
__weak void CURR_Clear( CURR_Handle_t * pHandle );
__weak uint16_t CURR_CalcMainCurrent( CURR_Handle_t * pHandle );

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CURRENT_SENSE_H */
