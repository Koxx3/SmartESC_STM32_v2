#include "main.h"
#include "defines.h"

#ifndef APP_PRODUCT_H_
#define APP_PRODUCT_H_

 /*  Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#ifdef G30P

#define VBUS_ADC_CHANNEL                                                     MC_ADC_CHANNEL_1
#define VOLTAGE_DIVIDER_GAIN     											 (float)3363.5

#define VESC_USART                                                 			 USART1

#define VESC_USART_DMA													     huart1
#define VESC_USART_TX_DMA													 hdma_usart1_tx
#define VESC_USART_RX_DMA													 hdma_usart1_rx
#define VESC_TOOL_ENABLE													 1

#define APP_USART_DMA														 huart2
#define APP_USART_TX_DMA												     hdma_usart2_tx
#define APP_USART_RX_DMA													 hdma_usart2_rx

#define APP2_USART_DMA														 huart3
#define APP2_USART_TX_DMA												     hdma_usart3_tx
#define APP2_USART_RX_DMA													 hdma_usart3_rx

#define RSHUNT                        										 0.00200
#define AMPLIFICATION_GAIN            										 9.4336
#define NOMINAL_CURRENT         											 2000
#define ID_DEMAG															 -2000

#define SCOPE_UVW															 0

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)14
#define HALL_PHASE_SHIFT        											 90
#define HALL_SENSORS_PLACEMENT  											 DEGREES_120
#define HALL_FAULT_RESET_CNT												 200

#define TEMP_SENSOR_TYPE													 VIRTUAL_SENSOR
#define CURR_SENSOR_TYPE													 REAL_SENSOR
#define ERROR_PRINTING														 1

#define ADC_SAMPLE_MAX_LEN 													 500

#endif

#ifdef M365
#define VBUS_ADC_CHANNEL                                                     MC_ADC_CHANNEL_2
#define VOLTAGE_DIVIDER_GAIN     											 (float)2650.0

#define VESC_USART                                                 			 USART3
#define VESC_USART_DMA													     huart3
#define VESC_USART_TX_DMA													 hdma_usart3_tx
#define VESC_USART_RX_DMA													 hdma_usart3_rx
#define VESC_TOOL_ENABLE													 1

#define APP_USART_DMA														 huart1
#define APP_USART_TX_DMA												     hdma_usart1_tx
#define APP_USART_RX_DMA													 hdma_usart1_rx

//Current Measurement
#define RSHUNT                        										 0.00200
#define AMPLIFICATION_GAIN            										 8.00
#define NOMINAL_CURRENT         											 2000
#define ID_DEMAG														     -2000

#define SCOPE_UVW															 1

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)15
#define HALL_PHASE_SHIFT        											 90
#define HALL_FAULT_RESET_CNT												 200

#define TEMP_SENSOR_TYPE													 REAL_SENSOR
#define CURR_SENSOR_TYPE													 VIRTUAL_SENSOR
#if VESC_TOOL_ENABLE
#define ERROR_PRINTING														 1
#endif

#define ADC_SAMPLE_MAX_LEN 													 1000


#endif
/****************************************************************************/

#define BATTERY_VOLTAGE_GAIN     											 ((VOLTAGE_DIVIDER_GAIN * ADC_GAIN) * 512.0)
#define CURRENT_FACTOR_A 													 ((32767.0*RSHUNT*AMPLIFICATION_GAIN)/(3.3/2))
#define CURRENT_FACTOR_mA 													 (CURRENT_FACTOR_A/1000.0)

#define DEMCR_TRCENA    0x01000000
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define CPU_CLOCK		64000000


#define PRIO_BELOW_NORMAL 4
#define PRIO_NORMAL  5
#define PRIO_HIGHER  6

#endif /* APP_PRODUCT_H_ */
