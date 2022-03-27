#include "app.h"
#include "crc.h"
#include "product.h"
#include "task_pwr.h"
#include "task_init.h"
#include "utils.h"
#include "timers.h"

extern app_configuration appconf;
static volatile bool output_disabled_now = false;
TimerHandle_t app_disable;

app_configuration* app_get_configuration(void) {
	return &appconf;
}

void vEnableOutput( TimerHandle_t xTimer ){
	output_disabled_now = false;
}


/**
 * Reconfigure and restart all apps. Some apps don't have any configuration options.
 *
 * @param conf
 * The new configuration to use.
 */
void app_set_configuration(app_configuration *conf) {
	appconf = *conf;
	static app_use old_app;

	if(app_disable==NULL){
		app_disable = xTimerCreate("DIS_TMR",MS_TO_TICKS(20) , pdFALSE, ( void * ) 0,vEnableOutput );
	}

	VESC_USART_DMA.Init.BaudRate = appconf.app_uart_baudrate;

	HAL_UART_Init(&VESC_USART_DMA);

	switch(conf->shutdown_mode){
	case SHUTDOWN_MODE_OFF_AFTER_10S:
		PWR_set_shutdown_time(10);
		break;
	case SHUTDOWN_MODE_OFF_AFTER_1M:
		PWR_set_shutdown_time(60);
		break;
	case SHUTDOWN_MODE_OFF_AFTER_5M:
		PWR_set_shutdown_time(300);
		break;
	case SHUTDOWN_MODE_OFF_AFTER_10M:
		PWR_set_shutdown_time(600);
		break;
	case SHUTDOWN_MODE_OFF_AFTER_30M:
		PWR_set_shutdown_time(1800);
		break;
	case SHUTDOWN_MODE_OFF_AFTER_1H:
		PWR_set_shutdown_time(3600);
		break;
	case SHUTDOWN_MODE_OFF_AFTER_5H:
		PWR_set_shutdown_time(18000);
		break;
	default:
		PWR_set_shutdown_time(0);  // 0 = disable timer
		break;
	}

	utils_truncate_number_int((int*)&appconf.app_adc_conf.update_rate_hz, 1, 200);

	switch (appconf.app_to_use) {
		case APP_UART:
			if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
				if(old_app == APP_ADC){
					task_app_kill(&aux_uart);
				}
			}
			task_cli_init(&aux_uart);
			break;
		case APP_ADC:
			if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
				if(old_app == APP_UART || old_app == APP_ADC_UART){
					task_cli_kill(&aux_uart);
				}
			}
			task_app_init(&aux_uart);
		case APP_ADC_UART:
			if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
				if(old_app == APP_ADC){
					task_app_kill(&aux_uart);
				}else if(old_app == APP_UART){
					task_cli_kill(&aux_uart);
				}
			}
			task_cli_init(&aux_uart);
			break;
		default:
			break;
	}
	old_app = appconf.app_to_use;
}



/**
 * Disable output on apps
 *
 * @param time_ms
 * The amount of time to disable output in ms
 * 0: Enable output now
 * -1: Disable forever
 * >0: Amount of milliseconds to disable output
 */
void app_disable_output(int time_ms) {
	if (time_ms == 0) {
		output_disabled_now = false;
	} else if (time_ms == -1) {
		output_disabled_now = true;
	} else {
		xTimerChangePeriod(app_disable, MS_TO_TICKS(time_ms),100);
		xTimerStart(app_disable, 100);
		output_disabled_now = true;
	}
}

bool app_is_output_disabled(void) {
	return output_disabled_now;
}

/**
 * Get app_configuration CRC
 *
 * @param conf
 * Pointer to app_configuration or NULL for current appconf
 *
 * @return
 * CRC16 (with crc field in struct temporarily set to zero).
 */
unsigned app_calc_crc(app_configuration* conf) {
	//if(null == conf)
	//	conf = &appconf;

	unsigned crc_old = conf->crc;
	conf->crc = 0;
	unsigned crc_new = crc16((uint8_t*)conf, sizeof(app_configuration));
	conf->crc = crc_old;
	return crc_new;
}
