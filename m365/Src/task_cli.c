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

#include "task_cli.h"
#include "task_init.h"
#include "main.h"
#include "TTerm.h"
#include "printf.h"
#include "cli_basic.h"
#include "cli_common.h"
#include "mc_config.h"


StreamBufferHandle_t UART_RX;

#define MSG_FLAG 0xA5
#define MSG_BYTES 22

enum _states{
	MSG_IDLE,
	MSG_DATA
};

struct _msg_format{
	uint8_t type;
	uint8_t destination;
	uint8_t n_esc;
	uint8_t bms_prot;
	uint8_t esc_jumps;
	uint8_t version_maj;
	uint8_t version_min;
	uint8_t power_on;
	uint8_t throttle;
	uint8_t brake;
	uint8_t torque_max;
	uint8_t brake_max;
	uint8_t lock;
	uint8_t regulator;
	uint8_t motor_direction;
	uint8_t hall_direction;
	uint8_t light;
	uint8_t temp_warn;
	uint8_t temp_max;
	uint8_t speed_lim;
	uint8_t start_speed;
	uint8_t crc;
};

typedef struct _msg_format msg_format;

enum uart_mode task_cli_mode = UART_MODE_ST;

#define STREAMBUFFER_RX_SIZE 32
#define STREAMBUFFER_TX_SIZE 32

osThreadId_t task_cli_handle;
const osThreadAttr_t task_cli_attributes = {
  .name = "CLI",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};

TERMINAL_HANDLE * cli_handle;


void _putchar(char character){
	while(!LL_USART_IsActiveFlag_TXE(pUSART.USARTx)){
	}
	LL_USART_TransmitData8(pUSART.USARTx, character);
}

qd_t currComp;

//A5 00 00 00 00 00 00 00 00 20 00 00 00 00 00 00 00 00 00 00 00 00 FF

void interpret(msg_format* msg){

	if(msg->brake){
		currComp.q = pCMD_calculate_curr_8(-((int16_t)msg->brake));
		MCI_SetCurrentReferences(pMCI[M1],currComp);
	}else{
		currComp.q = pCMD_calculate_curr_8((int16_t)msg->throttle);
		MCI_SetCurrentReferences(pMCI[M1],currComp);
	}
}

void task_cli(void * argument)
{

	uint8_t c=0, len=0;

	msg_format msg;
	uint8_t * msg_ptr = (uint8_t*)&msg;
	enum _states state;
	state = MSG_IDLE;
	uint8_t byte_cnt=0;

	MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);
	MCI_StartMotor( pMCI[M1] );

	currComp = MCI_GetIqdref(pMCI[M1]);

  /* Infinite loop */
	for(;;)
	{
		/* `#START TASK_LOOP_CODE` */
		len = xStreamBufferReceive(UART_RX, &c,sizeof(c), portMAX_DELAY);
		if(len){

			switch(state){
			case MSG_IDLE:
				if(c==MSG_FLAG){
					state = MSG_DATA;
					byte_cnt = MSG_BYTES;
					msg_ptr = (uint8_t*)&msg;
					break;
				}
				TERM_processBuffer(&c,len,(TERMINAL_HANDLE*)argument);
				break;
			case MSG_DATA:
				*msg_ptr = c;
				msg_ptr++;
				byte_cnt--;
				if(byte_cnt==0){
					interpret(&msg);
					state=MSG_IDLE;
				}
				break;
			}

		}

	}
}

void task_cli_init(){
	//HAL_NVIC_DisableIRQ(USART3_IRQn);

	HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);

	UART_RX = xStreamBufferCreate(STREAMBUFFER_RX_SIZE,1);

	task_cli_mode = UART_MODE_CLI;

	TERM_addCommand(CMD_get, "get", "Usage get [param]",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_set, "set","Usage set [param] [value]",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_eeprom, "eeprom","Save/Load config [load/save]",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_tune, "tune","Run autotune",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_start, "start","Start motor",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_stop, "stop","Stop motor",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_reset, "reset","Reset MCU",0,&TERM_cmdListHead);

	cli_handle = TERM_createNewHandle(printf,pdTRUE,&TERM_cmdListHead,"root");
	task_cli_handle = osThreadNew(task_cli, cli_handle, &task_cli_attributes);
}

