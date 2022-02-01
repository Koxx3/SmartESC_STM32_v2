/*
 * m365
 *
 * Copyright (c) 2021 Francois Deslandes
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

#include "task_pwr.h"
#include "task_led.h"
#include "task_init.h"
#include "main.h"
#include "task.h"
#include "task_cli.h"
#include <string.h>
#include "VescCommand.h"
#include "music.h"
#include "ninebot.h"
#include "conf_general.h"
#include "VescToSTM.h"
#include "app.h"

//Not a real Task... it's called from safety task. No delays allowed

#define EXECUTION_SPEED		40			//every 40 ticks (20ms)

extern m365Answer m365_to_display;
//extern const uint8_t m365_mode[3];
uint32_t shutdown_limit = 0;

uint8_t buttonState() {
    static const uint32_t DEBOUNCE_MILLIS = 20 ;
    bool buttonstate = HAL_GPIO_ReadPin( PWR_BTN_GPIO_Port, PWR_BTN_Pin ) == GPIO_PIN_SET ;
    uint32_t buttonstate_ts = HAL_GetTick() ;

    uint32_t now = HAL_GetTick() ;
    if( now - buttonstate_ts > DEBOUNCE_MILLIS )
    {
        if( buttonstate != (HAL_GPIO_ReadPin( PWR_BTN_GPIO_Port, PWR_BTN_Pin ) == GPIO_PIN_SET))
        {
            buttonstate = !buttonstate ;
            buttonstate_ts = now ;
        }
    }
    return buttonstate ;
}

eButtonEvent getButtonEvent()
{
    static const uint32_t DOUBLE_GAP_MILLIS_MAX 	= 250;
    static const uint32_t SINGLE_PRESS_MILLIS_MAX 	= 300;
    static const uint32_t LONG_PRESS_MILLIS_MAX 	= 5000;

    static uint32_t button_down_ts = 0 ;
    static uint32_t button_up_ts = 0 ;
    static bool double_pending = false ;
    static bool button_down = false ; ;

    eButtonEvent button_event = NO_PRESS ;
    uint32_t now = HAL_GetTick() ;

    if( button_down != buttonState() ) {
        button_down = !button_down ;
        if( button_down ) {
            button_down_ts = now ;
        } else {
            button_up_ts = now ;
            if( double_pending ) {
                button_event = DOUBLE_PRESS ;
                double_pending = false ;
            }
            else {
                double_pending = true ;
            }
        }
    }

    uint32_t diff =  button_up_ts - button_down_ts;
    if (!button_down && double_pending && now - button_up_ts > DOUBLE_GAP_MILLIS_MAX) {
    	double_pending = false ;
    	button_event = SINGLE_PRESS ;
	} else if (!button_down && double_pending && diff >= SINGLE_PRESS_MILLIS_MAX && diff <= LONG_PRESS_MILLIS_MAX) {
		double_pending = false ;
		button_event = LONG_PRESS ;
	} else if (button_down && now - button_down_ts > LONG_PRESS_MILLIS_MAX) {
		double_pending = false ;
		button_event = VERY_LONG_PRESS ;
	}

    return button_event ;
}

void PWR_set_shutdown_time(uint32_t seconds){

	shutdown_limit = seconds * 2000;

}

//This is not a FreeRTOS Task... its called from safety task to safe some heap space every 500us
void task_PWR(void *argument) {
	static uint8_t main_loop_counter = 0;
	static uint32_t shutdown_timer = 0;


	if(SpeednTorqCtrlM1.SPD->hAvrMecSpeedUnit){
		shutdown_timer = 0;
	}else if (shutdown_limit > 0){
		shutdown_timer++;
		if(shutdown_timer>shutdown_limit){
			shutdown_timer=0;
			power_control(DEV_PWR_OFF);
		}
	}

	if(main_loop_counter > 40){
		main_loop_counter=0;
		switch( getButtonEvent() ){
			  case NO_PRESS : break ;
			  case SINGLE_PRESS : {
				  m365_to_display.light = !m365_to_display.light;
				  if(m365_to_display.light){
					  task_LED_set_brake_light(BRAKE_LIGHT_ON);
				  }else{
					  task_LED_set_brake_light(BRAKE_LIGHT_OFF);
				  }

			  } break ;
			  case LONG_PRESS :   {
				  power_control(DEV_PWR_OFF);

			  } break ;
			  case VERY_LONG_PRESS :   {

			  } break ;
			  case DOUBLE_PRESS : {
				  uint32_t kmh=0;
				  switch(m365_to_display.mode){
				  	case M365_MODE_DRIVE:
				  		app_adc_speed_mode(M365_MODE_SPORT);
						kmh = 1337;
						break;
				  	case M365_MODE_SPORT:
				  		app_adc_speed_mode(M365_MODE_SLOW);
						kmh = 5;
						break;
				  	case M365_MODE_SLOW:
				  		app_adc_speed_mode(M365_MODE_DRIVE);
						kmh = 25;
						break;
				  }
				  if(kmh==1337){
					  mc_conf.lo_max_erpm = mc_conf.l_max_erpm;
				  }else{
					  mc_conf.lo_max_erpm = (((float)kmh * 1000.0 / 60.0)/(mc_conf.si_wheel_diameter*M_PI)) * mc_conf.si_motor_poles * mc_conf.si_gear_ratio;
				  }
				  MCI_ExecSpeedRamp(pMCI[M1], VescToSTM_erpm_to_speed(mc_conf.lo_max_erpm), 0);
			  } break ;
		 }
	}
	main_loop_counter++;
}

void task_PWR_init(port_str * port) {
	/* Check button pressed state at startup */
	buttonState();

    /* Power ON board temporarily, ultimate decision to keep hardware ON or OFF is made later */
	power_control(DEV_PWR_ON);

}

void power_control(uint8_t pwr)
{
	if(pwr == DEV_PWR_ON) {
		/* Turn the PowerON line high to keep the board powered on even when
		 * the power button is released
		 */
		HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_SET);
	} else if(pwr == DEV_PWR_OFF) {

		vTaskDelay(1);

		while(HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin));
		HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_RESET);
		while(1);
	} else if(pwr == DEV_PWR_RESTART) {

		/* Restart the system */
		NVIC_SystemReset();
	}
}
