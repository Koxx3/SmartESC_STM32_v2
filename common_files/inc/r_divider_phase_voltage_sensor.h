/**

  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RDIVIDER_PHASEVOLTAGESENSOR_H
#define __RDIVIDER_PHASEVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "regular_conversion_manager.h"

/**
  * @brief  Rdivider class parameters definition
  */
typedef struct
{
  RegConv_t      Phase[3];
  uint8_t        convHandle[3];            /*!< handle to the regular conversion */
  int16_t 		 voltages[3];
} RPhase_Handle_t;

/* Exported functions ------------------------------------------------------- */
void RVPS_Init( RPhase_Handle_t * pHandle );
void RVPS_get_phases( RPhase_Handle_t * pHandle );



/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __RDividerBusVoltageSensor_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

