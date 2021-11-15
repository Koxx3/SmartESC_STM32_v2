#include "main.h"

#ifndef APP_PRODUCT_H_
#define APP_PRODUCT_H_

#ifdef G30P

#define VBUS_ADC_CHANNEL                                                     MC_ADC_CHANNEL_1
#define VOLTAGE_DIVIDER_GAIN     											 (float)3363.5

#define VESC_USART                                                 			 USART1
#define VESC_USART_DMA													     huart1
#define VESC_USART_TX_DMA													 hdma_usart1_tx
#define VESC_USART_RX_DMA													 hdma_usart1_rx

#define APP_USART_DMA														 huart2
#define APP_USART_TX_DMA												     hdma_usart2_tx
#define APP_USART_RX_DMA													 hdma_usart2_rx

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)14
#define USART_IRQHandler 												 	 USART1_IRQHandler
#endif

#ifdef M365
#define VBUS_ADC_CHANNEL                                                     MC_ADC_CHANNEL_2
#define VOLTAGE_DIVIDER_GAIN     											 (float)2650.0

#define VESC_USART                                                 			 USART3
#define VESC_USART_DMA													     huart3
#define VESC_USART_TX_DMA													 hdma_usart3_tx
#define VESC_USART_RX_DMA													 hdma_usart3_rx

#define APP_USART_DMA														 huart1
#define APP_USART_TX_DMA												     hdma_usart1_tx
#define APP_USART_RX_DMA													 hdma_usart1_rx

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)15
#endif
/****************************************************************************/

#define BATTERY_VOLTAGE_GAIN     											 ((VOLTAGE_DIVIDER_GAIN * ADC_GAIN) * 512.0)

#define DEMCR_TRCENA    0x01000000
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define CPU_CLOCK		64000000

#endif /* APP_PRODUCT_H_ */
