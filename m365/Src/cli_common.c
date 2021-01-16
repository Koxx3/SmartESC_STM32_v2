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

#include "cli_common.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "printf.h"
#include "task_cli.h"
#include "math.h"
#include "system.h"
#include "mc_config.h"
#include "mc_interface.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))


#define UNUSED_VARIABLE(N) \
	do {                   \
		(void)(N);         \
	} while (0)
        
	
uint8_t callback_ConfigFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_DefaultFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);


cli_config configuration;

extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];

/*****************************************************************************
* Initializes parameters with default values
******************************************************************************/
void init_config(){
    configuration.conf1 = 123;
    configuration.conf2 = 456;

}

// clang-format off

/*****************************************************************************
* Parameter struct
******************************************************************************/



parameter_entry confparam[] = {
    //Parameter Type ,"Text   " , Value ptr                     ,Min     ,Max    ,Div    				,Callback Function           ,Help text
    ADD_PARAM("pole_pairs"      , HALL_M1._Super.bElToMecRatio  , 2      ,100    ,0      				,callback_DefaultFunction    ,"N Poles")
    ADD_PARAM("hall_placement"  , HALL_M1.SensorPlacement       , 0      ,1      ,0      				,callback_DefaultFunction    ,"[0] 120 deg [1] 60 deg")
	ADD_PARAM("hall_shift"      , HALL_M1.PhaseShift       		, 0      ,65536  ,(65536.0/360.0)       ,callback_DefaultFunction    ,"Electrical hall phase shift")
};

   
void eeprom_load(TERMINAL_HANDLE * handle){
    EEPROM_read_conf(confparam, PARAM_SIZE(confparam) ,0,handle);
}




// clang-format on




/*****************************************************************************
* Callback if a configuration relevant parameter is changed
******************************************************************************/
uint8_t callback_ConfigFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){

    return 1;
}

/*****************************************************************************
* Default function if a parameter is changes (not used)
******************************************************************************/
uint8_t callback_DefaultFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    
    return 1;
}



/*****************************************************************************
* Get a value from a parameter or print all parameters
******************************************************************************/
uint8_t CMD_get(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    
    if(argCount==0){
        print_param_help(confparam, PARAM_SIZE(confparam), handle);
        return TERM_CMD_EXIT_SUCCESS;
     }
    
    if(strcmp(args[0], "-?") == 0){
        ttprintf("Usage: get [parameter]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
    
	for (uint8_t current_parameter = 0; current_parameter < sizeof(confparam) / sizeof(parameter_entry); current_parameter++) {
		if (strcmp(args[0], confparam[current_parameter].name) == 0) {
			//Parameter found:
			print_param(confparam,current_parameter,handle);
			return TERM_CMD_EXIT_SUCCESS;
		}
	}
	ttprintf("E: unknown param\r\n");
	return 0;
}


uint8_t CMD_start(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	if(argCount==0){
		ttprintf("start [f|b] forwards/backwards\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}
	MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);
	MCI_StartMotor( pMCI[M1] );

	qd_t currComp;
	currComp = MCI_GetIqdref(pMCI[M1]);
	if(args[0][0] == 'f') currComp.q = 800;
	if(args[0][0] == 'b') currComp.q = -800;
	MCI_SetCurrentReferences(pMCI[M1],currComp);
	return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_stop(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	MCI_StopMotor( pMCI[M1] );
	return TERM_CMD_EXIT_SUCCESS;
}

/*****************************************************************************
* Set a new value to a parameter
******************************************************************************/
uint8_t CMD_set(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {

	if(argCount<2){
        ttprintf("Usage: set [parameter] [value]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
  
	for (uint8_t current_parameter = 0; current_parameter < sizeof(confparam) / sizeof(parameter_entry); current_parameter++) {
		if (strcmp(args[0], confparam[current_parameter].name) == 0) {
			//parameter name found:

			if (updateDefaultFunction(confparam, args[1],current_parameter, handle)){
                if(confparam[current_parameter].callback_function){
                    if (confparam[current_parameter].callback_function(confparam, current_parameter, handle)){
                        ttprintf("OK\r\n");
                        return TERM_CMD_EXIT_SUCCESS;
                    }else{
                        ttprintf("ERROR: Callback\r\n");
                        return TERM_CMD_EXIT_SUCCESS;
                    }
                }else{
                    ttprintf("OK\r\n");
                    return TERM_CMD_EXIT_SUCCESS;
                }
			} else {
				ttprintf("NOK\r\n");
				return TERM_CMD_EXIT_SUCCESS;
			}
		}
	}
	ttprintf("E: unknown param\r\n");
	return TERM_CMD_EXIT_SUCCESS;
}


/*****************************************************************************
* Saves confparams to eeprom
******************************************************************************/
uint8_t CMD_eeprom(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    if(argCount==0 || strcmp(args[0], "-?") == 0){
        ttprintf("Usage: eprom [load|save]\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
	if(strcmp(args[0], "save") == 0){
		taskENTER_CRITICAL();

		HAL_FLASH_Unlock();
		uint32_t page_error = 0;
		FLASH_EraseInitTypeDef s_eraseinit;
		s_eraseinit.TypeErase   = FLASH_TYPEERASE_PAGES;
		s_eraseinit.PageAddress = ADDR_FLASH_PAGE_63;
		s_eraseinit.NbPages     = 1;
		HAL_FLASHEx_Erase(&s_eraseinit, &page_error);

        EEPROM_check_hash(confparam,PARAM_SIZE(confparam),handle);
	    EEPROM_write_conf(confparam, PARAM_SIZE(confparam),0, handle);

	    HAL_FLASH_Lock();
	    taskEXIT_CRITICAL();

		return TERM_CMD_EXIT_SUCCESS;
	}
	if(strcmp(args[0], "load") == 0){

		EEPROM_read_conf(confparam, PARAM_SIZE(confparam) ,0,handle);
        
		return TERM_CMD_EXIT_SUCCESS;
	}
    return TERM_CMD_EXIT_SUCCESS;
}


/*****************************************************************************
* Loads the default parametes out of flash
******************************************************************************/
uint8_t CMD_load_defaults(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    ttprintf("Default parameters loaded\r\n");
    init_config();
    return TERM_CMD_EXIT_SUCCESS;
}



struct _sensor{
	uint16_t angle_forw;
	uint16_t angle_back;
};

const uint8_t hall_arr[] = {0,5,1,3,2,6,4,7};

uint8_t CMD_tune(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){

	MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);

	MCI_StartMotor( pMCI[M1] );

	struct _sensor data[7];

	qd_t currComp;
	currComp = MCI_GetIqdref(pMCI[M1]);
	currComp.q = 800;


	vTaskDelay(pdMS_TO_TICKS(5000));

	pMCI[M1]->pSTC->SPD->open_loop = true;
	HALL_M1.PhaseShift=0;

	MCI_SetCurrentReferences(pMCI[M1],currComp);
	pMCI[M1]->pSTC->SPD->open_angle =0;

	vTaskDelay(pdMS_TO_TICKS(2000));

	uint32_t old_hall = HALL_M1.HallState;
	uint16_t angle =0;
	uint8_t state_cnt = 1;

	uint8_t hall_lut[8] = {0,1,2,3,4,5,6,7};

	for(uint32_t i=0;i<8;i++){
		HALL_M1.lut[i] = hall_lut[i];
	}


	ttprintf("Getting sensor configuration...\r\n");
	for(uint32_t i=0;i<0x10000;i+=8){
		pMCI[M1]->pSTC->SPD->open_angle = angle+=8;
		vTaskDelay(1);
	}
	angle=0;
	pMCI[M1]->pSTC->SPD->open_angle = angle;
	vTaskDelay(pdMS_TO_TICKS(!000));
	for(uint32_t i=0;i<0x10000;i+=4){

		pMCI[M1]->pSTC->SPD->open_angle = angle+=4;
		vTaskDelay(1);
		if(old_hall != HALL_M1.HallState){
			if(state_cnt>6) return 0;
			ttprintf("Got: %u Expected: %u\r\n",HALL_M1.HallState, hall_arr[state_cnt]);
			hall_lut[HALL_M1.HallState] = hall_arr[state_cnt];
			state_cnt++;
			old_hall=HALL_M1.HallState;
		}
	}

	for(uint32_t i=0;i<8;i++){
		HALL_M1.lut[i] = hall_lut[i];
	}

	angle=0;

	pMCI[M1]->pSTC->SPD->open_angle = angle;
	vTaskDelay(pdMS_TO_TICKS(3000));
	old_hall = HALL_M1.HallState;

	ttprintf("Getting sensor offset...\r\n");
	for(uint8_t u=0;u<1;u++){
		for(uint32_t i=0;i<0xFFFF;i+=4){
			pMCI[M1]->pSTC->SPD->open_angle = angle+=4;
			vTaskDelay(1);
			if(old_hall != HALL_M1.HallState){
				data[HALL_M1.HallState].angle_forw = angle;
				ttprintf("State: %u Angle: %.2f\r\n",HALL_M1.HallState, (float)angle/(65536.0/360.0));
				old_hall=HALL_M1.HallState;
			}
		}
	}
	ttprintf("Backwards:\r\n");
	old_hall = HALL_M1.HallState;
	vTaskDelay(pdMS_TO_TICKS(500));
	for(uint8_t u=0;u<1;u++){
		for(uint32_t i=0;i<0xFFFF;i+=4){
			pMCI[M1]->pSTC->SPD->open_angle = angle-=4;
			vTaskDelay(1);
			if(old_hall != HALL_M1.HallState){
				switch(HALL_M1.HallState){
				case 6:
					data[4].angle_back = angle;
					break;
				case 2:
					data[6].angle_back = angle;
					break;
				case 3:
					data[2].angle_back = angle;
					break;
				case 1:
					data[3].angle_back = angle;
					break;
				case 5:
					data[1].angle_back = angle;
					break;
				case 4:
					data[5].angle_back = angle;
					break;
				}

				ttprintf("State: %u Angle: %.2f\r\n",HALL_M1.HallState, (float)angle/(65536.0/360.0));
				old_hall=HALL_M1.HallState;
			}
		}
	}

	for(uint8_t u=1;u<7;u++){
		uint16_t middle = data[hall_arr[u]].angle_back + ((data[hall_arr[u]].angle_forw - data[hall_arr[u]].angle_back)/2);
		ttprintf("State: %u Angle: %.2f\r\n",u, (float)middle/(65536.0/360.0));
		if(u==1){
			HALL_M1.PhaseShift = middle+16384;  //90 deg
			ttprintf("State: %u Angle: %.2f set shift to: %.2f\r\n",u, (float)middle/(65536.0/360.0), (float)HALL_M1.PhaseShift/(65536.0/360.0));

		}else{

		}

	}

	pMCI[M1]->pSTC->SPD->open_loop = false;
	MCI_StopMotor( pMCI[M1] );
	vTaskDelay(pdMS_TO_TICKS(2000));
	MCI_StartMotor( pMCI[M1] );
	vTaskDelay(pdMS_TO_TICKS(4000));
	MCI_StopMotor( pMCI[M1] );
	return TERM_CMD_EXIT_SUCCESS;

}
