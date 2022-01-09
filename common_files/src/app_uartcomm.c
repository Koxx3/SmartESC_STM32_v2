/*
 * m365
 *
 * Copyright (c) 2021 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "main.h"
#include "app.h"
#include "utils.h"
#include "VescToSTM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "product.h"
#include "ninebot.h"
#include "VescCommand.h"
#include "task_init.h"
#include "timers.h"
#include <math.h>

NinebotPack frame;


const uint8_t m365_mode[3] = { M365_MODE_SLOW, M365_MODE_DRIVE, M365_MODE_SPORT};

m365Answer m365_to_display = {.start1=NinebotHeader0, .start2=NinebotHeader1, .len=8, .addr=0x21, .cmd=0x64, .arg=0, .mode=1};


uint8_t app_connection_timout = 8;

TaskHandle_t task_app_handle;

void my_uart_send_data(unsigned char *buf, unsigned int len, port_str * port){
	if(port->half_duplex){
		port->uart->Instance->CR1 &= ~USART_CR1_RE;
		vTaskDelay(1);
	}
	//HAL_UART_Transmit(port->uart, buf, len, 500);
	HAL_UART_Transmit_DMA(port->uart, buf, len);
	while(port->uart->hdmatx->State != HAL_DMA_STATE_READY){
		port->uart->gState = HAL_UART_STATE_READY;
		vTaskDelay(1);
	}
	if(port->half_duplex) port->uart->Instance->CR1 |= USART_CR1_RE;
}

static uint32_t uart_get_write_pos(port_str * port){
	return ( ((uint32_t)port->rx_buffer_size - port->uart->hdmarx->Instance->CNDTR) & ((uint32_t)port->rx_buffer_size -1));
}


static uint8_t adc1;
static uint8_t adc2;
static float decoded_level = 0.0;
static float decoded_level2 = 0.0;

#define config		appconf.app_adc_conf

#define TimeMsElapsedSinceX(start)                                        \
  ((uint32_t)((xTaskGetTickCount()- (start))/2))

volatile uint32_t cyg;

void vTimerCallback( TimerHandle_t xTimer ){
	if(SpeednTorqCtrlM1.SPD->open_loop==true) return;
	uint32_t temp1 = frame.payload[1];
	uint32_t temp2 = frame.payload[2];
	temp1 <<= 4;
	temp2 <<= 4;
	static uint16_t aver1=0;
	static uint16_t aver2=0;

	if(config.use_filter){
		aver1 += temp1;
		aver1 /=2;
		aver2 += temp2;
		aver2 /=2;
	}else{
		aver1 = temp1;
		aver2 = temp2;
	}

	float pwr = utils_map(aver1, 0, 255<<4, 0, 3.3); //Throttle;
	float brake = utils_map(aver2, 0, 255<<4, 0, 3.3); //Brake;
	VescToSTM_set_ADC1(pwr);
	VescToSTM_set_ADC2(brake);


	// Map the read voltage
	switch (config.ctrl_type) {
	case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
	case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
	case ADC_CTRL_TYPE_DUTY_REV_CENTER:
	case ADC_CTRL_TYPE_PID_REV_CENTER:
		// Mapping with respect to center voltage
		if (pwr < config.voltage_center) {
			pwr = utils_map(pwr, config.voltage_start,
					config.voltage_center, 0.0, 0.5);
		} else {
			pwr = utils_map(pwr, config.voltage_center,
					config.voltage_end, 0.5, 1.0);
		}
		break;

	default:
		// Linear mapping between the start and end voltage
		pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
		break;
	}

	// Optionally invert the read voltage
	if (config.voltage_inverted) {
		pwr = 1.0 - pwr;
	}
	utils_truncate_number(&pwr, 0.0, 1.0);

	brake = utils_map(brake, config.voltage2_start, config.voltage2_end, 0.0, 1.0);
	// Optionally invert the read voltage
	if (config.voltage2_inverted) {
		brake = 1.0 - brake;
	}
	utils_truncate_number(&brake, 0.0, 1.0);
	decoded_level = pwr;
	decoded_level2 = brake;

	switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Scale the voltage and set 0 at the center
			pwr *= 2.0;
			pwr -= 1.0;
			break;

		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
			pwr -= brake;
			break;

		default:
			break;
	}

	// Apply deadband
	utils_deadband(&pwr, config.hyst, 1.0);

	// Apply throttle curve
	pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

	// Apply ramping
	static uint32_t last_time = 0;
	static float pwr_ramp = 0.0;
	float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

	if (ramp_time > 0.01) {
		const float ramp_step = (float)TimeMsElapsedSinceX(last_time) / (ramp_time * 1000.0);
		utils_step_towards(&pwr_ramp, pwr, ramp_step);
		last_time = xTaskGetTickCount();
		pwr = pwr_ramp;
	}


	// Use the filtered and mapped voltage for control according to the configuration.
	switch (config.ctrl_type) {
	case ADC_CTRL_TYPE_CURRENT:
	case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		VescToSTM_set_current_rel(pwr);
		break;

	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		if(pwr>=0){
			VescToSTM_set_current_rel(pwr);
		}else{
			VescToSTM_set_brake_current_rel(pwr);
		}
		break;
	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		if(brake>0){
			VescToSTM_set_brake_current_rel(brake);
		}else if(pwr>=0){
			VescToSTM_set_current_rel(pwr);
		}

		break;

	case ADC_CTRL_TYPE_PID:
	case ADC_CTRL_TYPE_PID_REV_CENTER:
	case ADC_CTRL_TYPE_PID_REV_BUTTON:{
		float speed = 0.0;
		if (pwr >= 0.0) {
			speed = pwr * mc_conf.lo_max_erpm;
		} else {
			speed = pwr * fabsf(mc_conf.lo_min_erpm);
		}
		VescToSTM_set_speed(speed);
	}
		break;
	default:
		break;
	}
}

float app_adc_get_decoded_level(void) {
	return decoded_level;
}
float app_adc_get_decoded_level2(void) {
	return decoded_level2;
}

TimerHandle_t xTimer;

void app_adc_stop_output(void) {
	if(xTimer!=NULL){
		xTimerStop(xTimer, 2000);
	}
}


void task_app(void * argument)
{
	uint32_t rd_ptr=0;
	port_str * port = (port_str*) argument;
	uint8_t * usart_rx_dma_buffer = pvPortMalloc(port->rx_buffer_size);
	HAL_UART_MspInit(port->uart);
	if(port->half_duplex){
		HAL_HalfDuplex_Init(port->uart);
	}
	HAL_UART_Receive_DMA(port->uart, usart_rx_dma_buffer, port->rx_buffer_size);
	CLEAR_BIT(port->uart->Instance->CR3, USART_CR3_EIE);

	if(xTimer==NULL){
		xTimer = xTimerCreate("ADC_UP",MS_TO_TICKS(20) , pdTRUE, ( void * ) 0,vTimerCallback );
	}
	xTimerStart(xTimer, 100);

	uint16_t slow_update_cnt=0;
  /* Infinite loop */
	for(;;)
	{
		while(rd_ptr != uart_get_write_pos(port)) {
			if(ninebot_parse(usart_rx_dma_buffer[rd_ptr] ,&frame)	==0){

				switch(frame.cmd){
					case 0x64:
						addCRC((uint8_t*)&m365_to_display, m365_to_display.len+6);
						my_uart_send_data((uint8_t*)&m365_to_display, sizeof(m365_to_display), port);
					break;
					case 0x65:
						adc1 = frame.payload[1];
						adc2 = frame.payload[2];
						VescToSTM_timeout_reset();
						if(xTimerIsTimerActive(xTimer)==pdFALSE){
							xTimerStart(xTimer, 100);
						}
						//commands_printf(main_uart.phandle, "LEN: %d CMD: %x ARG: %x PAY: %02x %02x %02x %02x", frame.len, frame.cmd, frame.arg, frame.payload[0], frame.payload[1], frame.payload[2], frame.payload[3]);
						//commands_printf(main_uart.phandle, "Cycles: %d", cyg);
					break;
				}
			}
			rd_ptr++;
			rd_ptr &= ((uint32_t)port->rx_buffer_size - 1);
		}

		if(slow_update_cnt==0){
			m365_to_display.speed = VescToSTM_get_speed()*3.6;
			m365_to_display.battery = utils_map(VescToSTM_get_battery_level(0), 0, 1, 0, 96);
			m365_to_display.beep=0;

		}else{
			slow_update_cnt++;
			if(slow_update_cnt==50) slow_update_cnt=0;
		}


		vTaskDelay(10);
		if(ulTaskNotifyTake(pdTRUE, 10)){
			xTimerStop(xTimer, 2000);
			HAL_UART_MspDeInit(port->uart);
			port->task_handle = NULL;
			vPortFree(usart_rx_dma_buffer);
			vTaskDelete(NULL);
			vTaskDelay(portMAX_DELAY);
		}

	}
}

void task_app_init(port_str * port){
	if(port->task_handle == NULL){
		xTaskCreate(task_app, "APP-USART", 128, (void*)port, PRIO_BELOW_NORMAL, &port->task_handle);
	}
}

void task_app_kill(port_str * port){
	if(port->task_handle){
		xTaskNotify(port->task_handle, 0, eIncrement);
		vTaskDelay(200);
	}
}
