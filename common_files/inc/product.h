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

#define APP_USART_DMA														 huart3
#define APP_USART_TX_DMA												     hdma_usart3_tx
#define APP_USART_RX_DMA													 hdma_usart3_rx

#define RSHUNT                        										 0.00200
#define AMPLIFICATION_GAIN            										 9.4336
#define NOMINAL_CURRENT         											 2000
#define ID_DEMAG															 -2000


#define MOTOR_VOLTAGE_CONSTANT  3.5 /*!< Volts RMS ph-ph /kRPM */
#define RS                     0.300 /* Stator resistance , ohm*/
#define LS                     0.000200 /* Stator inductance, H*/


#define PRODUCT_L_MAX_VOLTAGE												 45
#define PRODUCT_L_MIN_VOLTAGE												 8

#define SCOPE_UVW															 0

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)14
#define HALL_PHASE_SHIFT        											 90
#define HALL_FAULT_RESET_CNT												 200

#define TEMP_SENSOR_TYPE													 VIRTUAL_SENSOR
#define CURR_SENSOR_TYPE													 VIRTUAL_SENSOR

#define ADC_SAMPLE_MAX_LEN 													 500

#define PRODUCT_APP_TO_USE													 APP_ADC

// Setting limits
#define HW_LIM_CURRENT			-70.0, 70.0
#define HW_LIM_CURRENT_IN		-70.0, 70.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 75.0
#define HW_LIM_F_SW			    4000.0, 20000.0


#endif

#ifdef M365
#define VBUS_ADC_CHANNEL                                                     MC_ADC_CHANNEL_2
#define PHASE_A_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_6
#define PHASE_B_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_7
#define PHASE_C_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_9
#define VOLTAGE_DIVIDER_GAIN     											 (float)2808.6359

#define VESC_USART_DMA													     huart3
#define VESC_USART_TX_DMA													 hdma_usart3_tx
#define VESC_USART_RX_DMA													 hdma_usart3_rx

#define APP_USART_DMA														 huart1
#define APP_USART_TX_DMA												     hdma_usart1_tx
#define APP_USART_RX_DMA													 hdma_usart1_rx

//Current Measurement
#define RSHUNT                        										 0.00200
#define AMPLIFICATION_GAIN            										 8.00
#define NOMINAL_CURRENT         											 2000
#define ID_DEMAG														     -2000


#define MOTOR_VOLTAGE_CONSTANT  3.5 /*!< Volts RMS ph-ph /kRPM */
#define RS                     0.300 /* Stator resistance , ohm*/
#define LS                     0.000200 /* Stator inductance, H*/

#define PRODUCT_L_MAX_VOLTAGE												 56
#define PRODUCT_L_MIN_VOLTAGE												 8

#define SCOPE_UVW															 1

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)15
#define HALL_PHASE_SHIFT        											 90
#define HALL_FAULT_RESET_CNT												 200

#define TEMP_SENSOR_TYPE													 REAL_SENSOR
#define CURR_SENSOR_TYPE													 VIRTUAL_SENSOR

#define ADC_SAMPLE_MAX_LEN 													 1000

#define PRODUCT_APP_TO_USE													 APP_ADC_UART

// Setting limits
#define HW_LIM_CURRENT			-70.0, 70.0
#define HW_LIM_CURRENT_IN		-70.0, 70.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 75.0
#define HW_LIM_F_SW			    4000.0, 20000.0



#endif
/****************************************************************************/

#define KMH_NO_LIMIT														 1337
#define PRODUCT_FIRMWARE_VERSION                                      		 0x0001
#define VESC_TOOL_ENABLE													 1
#define AUTO_RESET_FAULT													 1
#define ERROR_PRINTING														 1
#define MUSIC_ENABLE														 0
#define BATTERY_SUPPORT_LIION												 1
#define BATTERY_SUPPORT_LIFEPO												 1
#define BATTERY_SUPPORT_LEAD												 1
#define ABS_OVR_CURRENT_TRIP_MS												 2.0
#define MIN_DUTY_FOR_PWM_FREEWHEEL											 80
#define CURRENT_DISPLAY_OFFSET											     0   //in cnts

#define MODE_SLOW_CURR														 0.5
#define MODE_DRIVE_CURR														 0.8
#define MODE_SPORT_CURR														 1.0
#define MODE_SLOW_SPEED														 10
#define MODE_DRIVE_SPEED													 25
#define MODE_SPORT_SPEED													 KMH_NO_LIMIT

#define HW_LIM_VIN														     PRODUCT_L_MIN_VOLTAGE, PRODUCT_L_MAX_VOLTAGE

#define BATTERY_VOLTAGE_GAIN     											 ((VOLTAGE_DIVIDER_GAIN * ADC_GAIN) * 512.0)
#define CURRENT_FACTOR_A 													 ((32767.0*RSHUNT*AMPLIFICATION_GAIN)/(3.3/2))
#define CURRENT_FACTOR_mA 													 (CURRENT_FACTOR_A/1000.0)
#define MIN_DUTY_PWM												         (32768 * MIN_DUTY_FOR_PWM_FREEWHEEL / 100)

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
