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
#include "speed_pos_fdbk.h"
#include "drive_parameters.h"
#include "mc_stm_types.h"
#include "parameters_conversion.h"
#include "defines.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))


#define UNUSED_VARIABLE(N) \
	do {                   \
		(void)(N);         \
	} while (0)
        
	
#define CURRENT_FACTOR 317.73

uint8_t callback_ConfigFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
uint8_t callback_DefaultFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);


cli_config configuration;

extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern RDivider_Handle_t RealBusVoltageSensorParamsM1;

//extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;

/*****************************************************************************
* Initializes parameters with default values
******************************************************************************/
void init_config(){

	HALL_M1.SwitchSpeed = 100;
}

// clang-format off

/*****************************************************************************
* Parameter struct
******************************************************************************/


parameter_entry confparam[] = {
    //Parameter Type ,"Text   " , Value ptr                     				  	 ,Min     ,Max    			 ,Div    				    ,Callback Function           ,Help text
    ADD_PARAM("pole_pairs"      , HALL_M1._Super.bElToMecRatio  				 	 , 2      ,100    			 ,0      				    ,callback_ConfigFunction    ,"N Poles")
    ADD_PARAM("hall_placement"  , HALL_M1.SensorPlacement       				 	 , 0      ,1      			 ,0      				    ,callback_ConfigFunction    ,"[0] 120 deg [1] 60 deg")
	ADD_PARAM("hall_shift"      , HALL_M1.PhaseShift       						 	 , 0      ,65536  			 ,(65536.0/360.0)          ,callback_DefaultFunction   ,"Electrical hall phase shift [deg]")
	ADD_PARAM("switch_speed"    , HALL_M1.SwitchSpeed      						 	 , 0      ,1000   			 ,0         				,callback_DefaultFunction   ,"Switching from 6-Step to FOC [RPM]")
	ADD_PARAM("max_pos_curr"    , SpeednTorqCtrlM1.MaxPositiveTorque    		 	 , 0      ,32767  			 ,CURRENT_FACTOR           ,callback_ConfigFunction    ,"Max phase current positive [A]")
	ADD_PARAM("max_neg_curr"    , SpeednTorqCtrlM1.MinNegativeTorque    			 , -32768 ,0      			 ,CURRENT_FACTOR           ,callback_ConfigFunction    ,"Max phase current negative [A]")
	ADD_PARAM("max_pos_speed"   , SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit   	 , 0 	  ,1000   			 ,(1.0/(_RPM / SPEED_UNIT)),callback_ConfigFunction   ,"Max positive speed [RPM]")
	ADD_PARAM("max_neg_speed"   , SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit    	 , -1000  ,0      			 ,(1.0/(_RPM / SPEED_UNIT)),callback_ConfigFunction    ,"Max negative speed [RPM]")
	ADD_PARAM("pid_torque_p"    , PIDIqHandle_M1.hKpGain                         	 , 0      ,4096   			 ,0                        ,callback_DefaultFunction   ,"Torque-PID [P]")
	ADD_PARAM("pid_torque_i"    , PIDIqHandle_M1.hKiGain                         	 , 0      ,4096   			 ,0                        ,callback_DefaultFunction   ,"Torque-PID [I]")
	ADD_PARAM("pid_flux_p"      , PIDIdHandle_M1.hKpGain                         	 , 0      ,4096   		     ,0                        ,callback_DefaultFunction   ,"Flux-PID [P]")
	ADD_PARAM("pid_flux_i"      , PIDIdHandle_M1.hKiGain                             , 0      ,4096   			 ,0                        ,callback_DefaultFunction   ,"Flux-PID [I]")
	ADD_PARAM("bus_ov"          , RealBusVoltageSensorParamsM1.OverVoltageThreshold  , 0      ,500*VOLT_SCALING  ,VOLT_SCALING             ,callback_DefaultFunction   ,"Overvoltage [V]")
	ADD_PARAM("bus_uv"          , RealBusVoltageSensorParamsM1.UnderVoltageThreshold , 0      ,500*VOLT_SCALING  ,VOLT_SCALING             ,callback_DefaultFunction   ,"Undervoltage [V]")
	ADD_PARAM("hall_lut"        , HALL_M1.lut                   				 	 , 0      ,0      			 ,0      				    ,callback_DefaultFunction   ,"Hall LUT only internal use")

};

void recalc_config(){
	PIDSpeedHandle_M1.wUpperIntegralLimit = (uint32_t)SpeednTorqCtrlM1.MaxPositiveTorque * SP_KDDIV;
	PIDSpeedHandle_M1.wLowerIntegralLimit = (uint32_t)SpeednTorqCtrlM1.MinNegativeTorque * SP_KDDIV;
	PIDSpeedHandle_M1.hUpperOutputLimit = SpeednTorqCtrlM1.MaxPositiveTorque;
	PIDSpeedHandle_M1.hLowerOutputLimit = SpeednTorqCtrlM1.MinNegativeTorque;
	if(SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit>SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit){
		HALL_M1._Super.hMaxReliableMecSpeedUnit = (uint16_t)(1.15*SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit);
	}else{
		HALL_M1._Super.hMaxReliableMecSpeedUnit = -(uint16_t)(1.15*SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit);
	}
	PIDIqHandle_M1.hDefKdGain = PIDIqHandle_M1.hKpGain;
	PIDIqHandle_M1.hDefKiGain = PIDIqHandle_M1.hKiGain;
	PIDIdHandle_M1.hDefKdGain = PIDIdHandle_M1.hKpGain;
	PIDIdHandle_M1.hDefKiGain = PIDIdHandle_M1.hKiGain;
}
void eeprom_load(TERMINAL_HANDLE * handle){

    EEPROM_read_conf(confparam, PARAM_SIZE(confparam) ,0,handle);
    recalc_config();

}

void eeprom_save(TERMINAL_HANDLE * handle){
    EEPROM_write_conf(confparam, PARAM_SIZE(confparam) ,0,handle);
}




// clang-format on




/*****************************************************************************
* Callback if a configuration relevant parameter is changed
******************************************************************************/
uint8_t callback_ConfigFunction(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
	recalc_config();
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

int16_t calculate_curr(int16_t torque_percent){
	if(torque_percent>0){
		return ((int32_t)SpeednTorqCtrlM1.MaxPositiveTorque*torque_percent)/100;
	}else{
		return -((int32_t)SpeednTorqCtrlM1.MinNegativeTorque*torque_percent)/100;
	}
}

int16_t pCMD_calculate_curr_8(int16_t torque_percent){
	if(torque_percent>0){
		return ((int32_t)SpeednTorqCtrlM1.MaxPositiveTorque*torque_percent)/255;
	}else{
		return -((int32_t)SpeednTorqCtrlM1.MinNegativeTorque*torque_percent)/255;
	}
}

uint8_t CMD_start(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	if(argCount==0){
		ttprintf("start [torque]\r\n");
		return TERM_CMD_EXIT_SUCCESS;
	}
	MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);
	MCI_StartMotor( pMCI[M1] );



	qd_t currComp;
	currComp = MCI_GetIqdref(pMCI[M1]);
	currComp.q = calculate_curr(atoi(args[0]));
	MCI_SetCurrentReferences(pMCI[M1],currComp);
	return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_stop(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	MCI_StopMotor( pMCI[M1] );
	return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_ack(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
	ttprintf("All faults cleared...\r\n");
	STM_FaultAcknowledged( pMCI[M1]->pSTM);
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

uint8_t TERM_eeprom_read(){
	init_config();
	EEPROM_read_conf(confparam, PARAM_SIZE(confparam) ,0,NULL);
	return 0;
}

/*****************************************************************************
* Loads the default parametes out of flash
******************************************************************************/
uint8_t CMD_load_defaults(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    ttprintf("Default parameters loaded\r\n");
    init_config();
    return TERM_CMD_EXIT_SUCCESS;
}




uint8_t CMD_reset(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args) {
    NVIC_SystemReset();
	return TERM_CMD_EXIT_SUCCESS;
}
