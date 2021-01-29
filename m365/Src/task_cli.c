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
#include "task_pwr.h"
#include "main.h"
#include "TTerm.h"
#include "printf.h"
#include "cli_basic.h"
#include "cli_common.h"
#include "mc_config.h"
#include <string.h>


StreamBufferHandle_t UART_RX;

#define MSG_FLAG 0xA5
#define MSG_BYTES 22

enum _states{
	MSG_IDLE,
	MSG_DATA
};





reply_format msg_rep;
msg_format msg;
msg_format old_msg;

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

void putbuffer(uint8_t* buf, uint16_t len){
	while(len){
		while(!LL_USART_IsActiveFlag_TXE(pUSART.USARTx)){}
		LL_USART_TransmitData8(pUSART.USARTx, *buf);
		len--;
		buf++;
	}
}


qd_t currComp;

//A5 00 00 00 00 00 00 00 00 20 00 00 00 00 00 00 00 00 00 00 00 00 FF

void interpret(msg_format* msg, msg_format* old_msg){

	if(msg->brake != old_msg->brake){
		currComp.q = pCMD_calculate_curr_8(-((int16_t)msg->brake));
		MCI_SetCurrentReferences(pMCI[M1],currComp);
	}else if(msg->throttle != old_msg->throttle){
		currComp.q = pCMD_calculate_curr_8((int16_t)msg->throttle);
		MCI_SetCurrentReferences(pMCI[M1],currComp);
	}

	if(msg->throttle==0 && msg->brake==0){
		MCI_StopMotor( pMCI[M1] );
	}else if(msg->throttle && !old_msg->throttle){
		MCI_StartMotor( pMCI[M1] );
	}else if(msg->brake && !old_msg->brake){
		MCI_StartMotor( pMCI[M1] );
	}

	if(msg->power_on == 1) poweroff();


	memcpy(old_msg,msg,sizeof(msg_format));

	msg_rep.crc=0;
	msg_rep.throttle = msg->throttle;
	msg_rep.brake = msg->brake;
	uint8_t * msg_rep_ptr = (uint8_t*)&msg_rep;
	for(uint32_t i=0;i<(sizeof(msg_format)-1);i++){ //without crc field
		msg_rep.crc ^= *msg_rep_ptr;
		msg_rep_ptr++;
	}

	putbuffer((uint8_t*)&msg_rep, sizeof(reply_format));
}

void task_cli(void * argument)
{

	uint8_t c=0, len=0;

	msg_rep.frame_start = 0x5A;


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
		len = xStreamBufferReceive(UART_RX, &c,sizeof(c), 10);
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
					interpret(&msg, &old_msg);
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

