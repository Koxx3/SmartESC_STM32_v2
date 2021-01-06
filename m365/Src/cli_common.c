/*
 * UD3
 *
 * Copyright (c) 2018 Jens Kerrinnes
 * Copyright (c) 2015 Steve Ward
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


#define UNUSED_VARIABLE(N) \
	do {                   \
		(void)(N);         \
	} while (0)
        
	
uint8_t callback_ConfigFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_DefaultFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);


cli_config configuration;



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
    //       Parameter Type ,"Text   "         , Value ptr                     ,Min     ,Max    ,Div    ,Callback Function           ,Help text
    ADD_PARAM("conf1"           , configuration.conf1           , 0      ,10000  ,0      ,callback_DefaultFunction    ,"Test parameter 1")
    ADD_PARAM("conf2"           , configuration.conf2           , 0      ,10000  ,0      ,callback_DefaultFunction    ,"Test parameter 2")
    ADD_PARAM("conf3"           , configuration.conf3           , 0      ,10000  ,10     ,callback_ConfigFunction     ,"Test config 1")
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

