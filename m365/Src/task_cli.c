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
#include "packet.h"
#include "VescCommand.h"

StreamBufferHandle_t UART_RX;

#define UART_HANDLE 0


enum uart_mode task_cli_mode = UART_MODE_ST;

#define STREAMBUFFER_RX_SIZE 32
#define STREAMBUFFER_TX_SIZE 32

osThreadId_t task_cli_handle;
const osThreadAttr_t task_cli_attributes = {
  .name = "CLI",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};

TERMINAL_HANDLE * cli_handle;


void _putchar(char character){
	while(!LL_USART_IsActiveFlag_TXE(pUSART.USARTx)){
	}
	LL_USART_TransmitData8(pUSART.USARTx, character);
}

void putbuffer(unsigned char *buf, unsigned int len){
	while(len){
		while(!LL_USART_IsActiveFlag_TXE(pUSART.USARTx)){}
		LL_USART_TransmitData8(pUSART.USARTx, *buf);
		len--;
		buf++;
	}
}

void cli_start_console(){

	if(cli_handle)return;
	task_cli_mode = UART_MODE_CLI;

	TERM_addCommand(CMD_get, "get", "Usage get [param]",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_set, "set","Usage set [param] [value]",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_eeprom, "eeprom","Save/Load config [load/save]",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_tune, "tune","Run autotune",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_start, "start","Start motor",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_stop, "stop","Stop motor",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_reset, "reset","Reset MCU",0,&TERM_cmdListHead);
	TERM_addCommand(CMD_ack, "ack","Clear faults",0,&TERM_cmdListHead);

	cli_handle = TERM_createNewHandle(printf,pdTRUE,&TERM_cmdListHead,"root");

};





typedef struct
{
  uint8_t type;
  uint8_t Power_ON;
  uint8_t Throttle;
  uint8_t Brake;
} SerialCommand;


void comm_uart_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, UART_HANDLE);
}

void process_packet(unsigned char *data, unsigned int len){

	commands_process_packet(data, len, &comm_uart_send_packet);


	/*int16_t q = pCMD_calculate_curr_8((int16_t)cmd->Throttle);

	if(q != currComp.q){
		currComp.q = q;
		MCI_SetCurrentReferences(pMCI[M1],currComp);
	}*/


}



void task_cli(void * argument)
{

	uint8_t c=0, len=0;




	MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);
	MCI_StartMotor( pMCI[M1] );
	currComp = MCI_GetIqdref(pMCI[M1]);


	uint32_t last_frame = xTaskGetTickCount();

	packet_init(putbuffer, process_packet, UART_HANDLE);

  /* Infinite loop */
	for(;;)
	{
		/* `#START TASK_LOOP_CODE` */
		len = xStreamBufferReceive(UART_RX, &c,sizeof(c), 20);

		if(len){
			packet_process_byte(c, UART_HANDLE);

		}
/*
		if((xTaskGetTickCount() - last_frame) > 200 && !cli_handle){
			last_frame = xTaskGetTickCount();
			currComp.q = pCMD_calculate_curr_8((int16_t)0);
			MCI_SetCurrentReferences(pMCI[M1],currComp);
		}*/

	}
}

void task_cli_init(){
	HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
	cli_handle = NULL;
	//cli_start_console();
	UART_RX = xStreamBufferCreate(STREAMBUFFER_RX_SIZE,1);
	task_cli_handle = osThreadNew(task_cli, cli_handle, &task_cli_attributes);
}

