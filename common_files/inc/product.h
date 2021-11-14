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
#define APP_USART_TX_DMA													 hdma_usart2_tx

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
#define APP_USART_TX_DMA													 hdma_usart1_tx

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)15
#define USART_IRQHandler 													 USART3_IRQHandler
#endif
/****************************************************************************/

#define BATTERY_VOLTAGE_GAIN     											 ((VOLTAGE_DIVIDER_GAIN * ADC_GAIN) * 512.0)

#endif /* APP_PRODUCT_H_ */
