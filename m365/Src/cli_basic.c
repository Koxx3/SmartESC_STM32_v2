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
#include "task_cli.h"
#include "cli_basic.h"
#include "printf.h"
#include <stdlib.h>
#include <string.h>
#include "TTerm.h"
#include "mc_config.h"


uint8_t EEPROM_Read_Row(uint8_t row, uint8_t * buffer);
uint8_t EEPROM_1_Write_Row(uint8_t row, uint8_t * buffer);

uint16_t byte_cnt;

#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0800FC00) /* Base @ of Page 63, 1 Kbytes */

uint8_t EEPROM_1_ReadByte(uint8_t x){
	uint8_t data[4];
	*(uint32_t*)data = (*(__IO uint32_t*)(ADDR_FLASH_PAGE_63+((x/4)*4)));
	return data[x%4];
}

uint8_t EEPROM_1_Write(uint8_t* y, uint16_t x){
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_PAGE_63+(x*4), *(uint32_t*)y);
	return 0;
}

#define EEPROM_READ_BYTE(x) EEPROM_1_ReadByte(x)
#define EEPROM_WRITE_ROW(x,y) EEPROM_1_Write(y,x)

uint8_t n_number(uint32_t n){
  if(n < 100000) {
    if(n < 1000) {
      if(n < 100) {
        if(n < 10)
          return 1;
        return 2;
      }
      return 3;
    }
    if(n < 10000)
      return 4;
    else
      return 5;
  }
  if(n < 100000000) {
    if(n < 10000000) {
      if(n < 1000000)
        return 6;
      return 7;
    }
    return 8;
  }
  if(n < 1000000000)
    return 9;
  else
    return 10;   
}

uint8_t updateDefaultFunction(parameter_entry * params, char * newValue, uint8_t index, TERMINAL_HANDLE * handle) {
    int32_t value;
    float fvalue;
    char* ch_ptr;
    switch (params[index].type){
    case TYPE_UNSIGNED:
        if(params[index].div){
            fvalue=strtof(newValue,&ch_ptr);
            value = fvalue*params[index].div;
        }else{
            value = strtoul(newValue,&ch_ptr,10);
        }
        if (value >= params[index].min && value <= params[index].max){
            if(params[index].size==1){
                *(uint8_t*)params[index].value = value;
                return 1;
            }
            if(params[index].size==2){
                *(uint16_t*)params[index].value = value;
                return 1;
            }
            if(params[index].size==4){
                *(uint32_t*)params[index].value = value;
                return 1;
            }
        }else{
            goto error;
        }
        break;
    case TYPE_SIGNED:
        if(params[index].div){
            fvalue=strtof(newValue,&ch_ptr);
            value = fvalue*params[index].div;
        }else{
            value = strtol(newValue,&ch_ptr,10);
        }
        if (value >= params[index].min && value <= params[index].max){
            if(params[index].size==1){
                *(int8_t*)params[index].value = value;
                return 1;
            }
            if(params[index].size==2){
                *(int16_t*)params[index].value = value;
                return 1;
            }
            if(params[index].size==4){
                *(int32_t*)params[index].value = value;
                return 1;
            }
        }else{
            goto error;
        }
        break;
    case TYPE_FLOAT:
            if(params[index].size==4){
                fvalue=strtof(newValue,&ch_ptr);
                if (fvalue >= (float)params[index].min && fvalue <= (float)params[index].max){
                    *(float*)params[index].value = fvalue;
                    return 1;
                }else{
                    goto error;
                }
            }
        break;
    case TYPE_CHAR:
        *(char*)params[index].value = *newValue;
        break;
    case TYPE_STRING:
        for(int i=0;i<params[index].size;i++){
            *(i+(char*)params[index].value) = newValue[i];
            if(!newValue[i]) break;
        }
        return 1;
        break;

    }
    return 0;
    error:
        if(params[index].div){
            ttprintf("E:Range %i.%u-%i.%u\r\n",
                     params[index].min/params[index].div,
                     params[index].min%params[index].div,                
                     params[index].max/params[index].div,
                     params[index].max%params[index].div);
        }else{
            ttprintf("E:Range %i-%i\r\n", params[index].min, params[index].max);
        }

    return 0;
}



void print_param_helperfunc(parameter_entry * params, uint8_t param_size, TERMINAL_HANDLE * handle, uint8_t param_type){
    #define COL_A 9
    #define COL_B 33
    #define COL_C 64
    uint8_t current_parameter;
    uint32_t u_temp_buffer=0;
    int32_t i_temp_buffer=0;
    TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
    ttprintf("Parameter");
    TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
    ttprintf("| Value");
    TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_C);
    ttprintf("| Text\r\n");
    for (current_parameter = 0; current_parameter < param_size; current_parameter++) {
		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_A);
		ttprintf("\033[36m%s", params[current_parameter].name);

		switch (params[current_parameter].type){
		case TYPE_UNSIGNED:
			switch (params[current_parameter].size){
			case 1:
				u_temp_buffer = *(uint8_t*)params[current_parameter].value;
				break;
			case 2:
				u_temp_buffer = *(uint16_t*)params[current_parameter].value;
				break;
			case 4:
				u_temp_buffer = *(uint32_t*)params[current_parameter].value;
				break;
			}

			TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
			if(params[current_parameter].div){
				ttprintf("\033[37m| \033[32m%u.%0*u",
					(u_temp_buffer/params[current_parameter].div),
					n_number(params[current_parameter].div)-1,
					(u_temp_buffer%params[current_parameter].div));
			}else{
				ttprintf("\033[37m| \033[32m%u", u_temp_buffer);
			}
			break;
		case TYPE_SIGNED:
			switch (params[current_parameter].size){
			case 1:
				i_temp_buffer = *(int8_t*)params[current_parameter].value;
				break;
			case 2:
				i_temp_buffer = *(int16_t*)params[current_parameter].value;
				break;
			case 4:
				i_temp_buffer = *(int32_t*)params[current_parameter].value;
				break;
			}

			TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
			if(params[current_parameter].div){
				uint32_t mod;
				if(i_temp_buffer<0){
					mod=(i_temp_buffer*-1)%params[current_parameter].div;
				}else{
					mod=i_temp_buffer%params[current_parameter].div;
				}

				ttprintf("\033[37m| \033[32m%i.%0*u",
				(i_temp_buffer/params[current_parameter].div),
				n_number(params[current_parameter].div)-1,
				mod);
			}else{
				ttprintf("\033[37m| \033[32m%i", i_temp_buffer);
			}

			break;
		case TYPE_FLOAT:

			TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
			ttprintf("\033[37m| \033[32m%f", *(float*)params[current_parameter].value);

			break;
		case TYPE_CHAR:

			TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
			ttprintf("\033[37m| \033[32m%c", *(char*)params[current_parameter].value);

			break;
		case TYPE_STRING:

			TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_B);
			ttprintf("\033[37m| \033[32m%s", (char*)params[current_parameter].value);

			break;

		}
		TERM_sendVT100Code(handle, _VT100_CURSOR_SET_COLUMN, COL_C);
		ttprintf("\033[37m| %s\r\n", params[current_parameter].help);
    }
}

void print_param_help(parameter_entry * params, uint8_t param_size, TERMINAL_HANDLE * handle){
    ttprintf("Parameters:\r\n");
    print_param_helperfunc(params, param_size, handle,PARAM_DEFAULT);
    ttprintf("\r\nConfiguration:\r\n");
    print_param_helperfunc(params, param_size, handle,PARAM_CONFIG);
}

void print_param(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle){
    uint32_t u_temp_buffer=0;
    int32_t i_temp_buffer=0;
    float f_temp_buffer=0.0;
    switch (params[index].type){
        case TYPE_UNSIGNED:
            switch (params[index].size){
                case 1:
                    u_temp_buffer = *(uint8_t*)params[index].value;
                    break;
                case 2:
                    u_temp_buffer = *(uint16_t*)params[index].value;
                    break;
                case 4:
                    u_temp_buffer = *(uint32_t*)params[index].value;
                    break;
            }
            if(params[index].div){
                ttprintf("\t%s=%u.%0*u\r\n",
                                params[index].name,(u_temp_buffer/params[index].div),
                                n_number(params[index].div)-1,
                                (u_temp_buffer%params[index].div));
            }else{
                ttprintf("\t%s=%u\r\n", params[index].name,u_temp_buffer);
            }
            break;
        case TYPE_SIGNED:
            switch (params[index].size){
            case 1:
                i_temp_buffer = *(int8_t*)params[index].value;
                break;
            case 2:
                i_temp_buffer = *(int16_t*)params[index].value;
                break;
            case 4:
                i_temp_buffer = *(int32_t*)params[index].value;
                break;
            }
            if(params[index].div){
                uint32_t mod;
                if(i_temp_buffer<0){
                    mod=(i_temp_buffer*-1)%params[index].div;
                }else{
                    mod=i_temp_buffer%params[index].div;
                }
                ttprintf("\t%s=%i.%0*u\r\n",
                                params[index].name,(i_temp_buffer/params[index].div),
                                n_number(params[index].div)-1,
                                mod);
            }else{
                ttprintf("\t%s=%i\r\n", params[index].name,i_temp_buffer);
            }
            break;
        case TYPE_FLOAT:
            f_temp_buffer = *(float*)params[index].value;
            ttprintf("\t%s=%f\r\n", params[index].name,f_temp_buffer);
            break;
        case TYPE_CHAR:
            ttprintf("\t%s=%c\r\n", params[index].name,*(char*)params[index].value);
            break;
        case TYPE_STRING:
            ttprintf("\t%s=%s\r\n", params[index].name,(char*)params[index].value);
            break;
        }
}


uint8_t EEPROM_Read_Row(uint8_t row, uint8_t * buffer){
    uint16_t addr;
    addr = row * CY_EEPROM_SIZEOF_ROW;
    for(uint8_t i = 0; i<CY_EEPROM_SIZEOF_ROW;i++){
        *buffer=EEPROM_READ_BYTE(addr+i);
        buffer++;
    }
    return 1;
}

uint8_t EEPROM_buffer_write(uint8_t byte, uint16_t address, uint8_t flush){
    byte_cnt++;
    static uint8_t eeprom_buffer[CY_EEPROM_SIZEOF_ROW];
    static uint16_t last_row = 0xFFFF;
    static uint8_t changed=0;
    uint8_t ret_change=0;
    uint16_t rowNumber;
    uint16_t byteNumber;
    rowNumber = address/(uint16_t)CY_EEPROM_SIZEOF_ROW;
    byteNumber = address - (rowNumber * ((uint16_t)CY_EEPROM_SIZEOF_ROW));
    if(last_row==0xFFFF){
        last_row=rowNumber;
        EEPROM_Read_Row(rowNumber,eeprom_buffer);
    }else if(last_row!=rowNumber){
        if(changed){
            EEPROM_WRITE_ROW(last_row,eeprom_buffer);
        }
        EEPROM_Read_Row(rowNumber,eeprom_buffer);
        changed=0;
    }
    if(eeprom_buffer[byteNumber]!=byte){
       eeprom_buffer[byteNumber]=byte;
       changed=1;
       ret_change=1;
    }

    last_row = rowNumber;

    if(flush && changed){
        last_row = 0xFFFF;
        EEPROM_WRITE_ROW(rowNumber,eeprom_buffer);
        changed = 0;
    }else if(flush){
        last_row = 0xFFFF;
    }
    return ret_change;
}

uint32_t djb_hash(const char* cp)
{
    uint32_t hash = 5381;
    while (*cp)
        hash = 33 * hash ^ (unsigned char) *cp++;
    return hash;
}

void EEPROM_check_hash(parameter_entry * params, uint8_t param_size, TERMINAL_HANDLE * handle){
    uint32_t temp_hash1;
    uint32_t temp_hash2;
    uint32_t collision=0;
    for (uint8_t  current_parameter = 0; current_parameter < param_size; current_parameter++) {
        if(params[current_parameter].parameter_type == PARAM_CONFIG){
            temp_hash1=djb_hash(params[current_parameter].name);
            for (uint8_t  n = 0; n < current_parameter; n++) {
                if(params[n].parameter_type == PARAM_CONFIG){
                    temp_hash2=djb_hash(params[n].name);
                    if(temp_hash1==temp_hash2){
                        ttprintf("Found collision %s <-> %s\r\n", params[current_parameter].name, params[n].name);
                        collision=1;
                    }
                }
            }
            for (uint8_t  n = current_parameter+1; n < param_size; n++) {
                if(params[n].parameter_type == PARAM_CONFIG){
                    temp_hash2=djb_hash(params[n].name);
                    if(temp_hash1==temp_hash2){
                        ttprintf("Found collision %s <-> %s\r\n", params[current_parameter].name, params[n].name);
                        collision=1;
                    }
                }
            }
        }
    }
    if(collision==0){
        ttprintf("Check for hash collision done.\r\n");   
    }
}

void EEPROM_write_conf(parameter_entry * params, uint8_t param_size, uint16_t eeprom_offset ,TERMINAL_HANDLE * handle){
    byte_cnt=0;
	uint16_t count = eeprom_offset;
	uint8_t change_flag = 0;
	uint16_t change_count = 0;
	uint32_t temp_hash=0;
	uint16_t param_count=0;
        EEPROM_buffer_write(0x00, count,0);
		count++;
		EEPROM_buffer_write(0xC0, count,0);
		count++;
		EEPROM_buffer_write(0xFF, count,0);
		count++;
		EEPROM_buffer_write(0xEE, count,0);
		count++;
		EEPROM_buffer_write(0x00, count,0);
		count++;
		for (uint8_t  current_parameter = 0; current_parameter < param_size; current_parameter++) {
            if(params[current_parameter].parameter_type == PARAM_CONFIG){
                param_count++;
                change_flag = 0;
                temp_hash=djb_hash(params[current_parameter].name);
                EEPROM_buffer_write(temp_hash, count ,0);
                count++;
                EEPROM_buffer_write(temp_hash>>8,count,0);
                count++;
                EEPROM_buffer_write(temp_hash>>16,count,0);
                count++;
                EEPROM_buffer_write(temp_hash>>24,count,0);
                count++;
                EEPROM_buffer_write(params[current_parameter].size,count,0);
                count++;
                for(uint8_t i=0;i<params[current_parameter].size;i++){
                    change_flag |= EEPROM_buffer_write(*(i+(uint8_t *)params[current_parameter].value), count,0);
                    count++;
                }
                if (change_flag) {
                    change_count++;
                }
            }
		}
		EEPROM_buffer_write(0xDE, count,0);
		count++;
		EEPROM_buffer_write(0xAD, count,0);
		count++;
		EEPROM_buffer_write(0xBE, count,0);
		count++;
		EEPROM_buffer_write(0xEF, count,0);
        count++;
		EEPROM_buffer_write(0x00, count,1);
		ttprintf("%i / %i new config params written. %i bytes from 2048 used.\r\n", change_count, param_count, byte_cnt);
}

void EEPROM_read_conf(parameter_entry * params, uint8_t param_size, uint16_t eeprom_offset ,TERMINAL_HANDLE * handle){
    uint16_t addr=eeprom_offset;
    uint32_t temp_hash=0;
    uint8_t data[DATASET_BYTES];
    uint16_t param_count=0;
    uint16_t change_count=0;
    uint8_t change_flag=0;
        for(int i=0;i<DATASET_BYTES;i++){
            data[i] = EEPROM_READ_BYTE(addr);
            addr++;
        }
        if(!(data[0]== 0x00 && data[1] == 0xC0 && data[2] == 0xFF && data[3] == 0xEE)) {
            ttprintf("WARNING: No or old EEPROM dataset found\r\n");
            return;
        }

    while(addr<CY_EEPROM_SIZE){
        change_flag=0;
        for(int i=0;i<DATASET_BYTES;i++){
            data[i] = EEPROM_READ_BYTE(addr);
            addr++;
        }
        if(data[0]== 0xDE && data[1] == 0xAD && data[2] == 0xBE && data[3] == 0xEF) break;
        uint8_t current_parameter=0;
        for ( current_parameter = 0; current_parameter < param_size; current_parameter++) {
            if(params[current_parameter].parameter_type==PARAM_CONFIG){
                temp_hash=djb_hash(params[current_parameter].name);
                if((uint8_t)temp_hash == data[0] && (uint8_t)(temp_hash>>8) == data[1]&& (uint8_t)(temp_hash>>16) == data[2]&& (uint8_t)(temp_hash>>24) == data[3]){
                    for(uint8_t i=0;i<data[4];i++){
                        *(i+(uint8_t * )params[current_parameter].value) = EEPROM_READ_BYTE(addr);
                        addr++;
                    }
                    change_count++;
                    change_flag=1;
                    break;
                }
            }
        }
        if(!change_flag) addr+=data[4];
        
        if(current_parameter == param_size){
            ttprintf("WARNING: Unknown param ID %i found in EEPROM\r\n", data[0]);
        }
    }
    uint8_t found_param=0;
    for (uint8_t current_parameter = 0; current_parameter < param_size; current_parameter++) {
        if(params[current_parameter].parameter_type==PARAM_CONFIG){
            param_count++;
            found_param=0;
            addr = DATASET_BYTES + eeprom_offset; //Skip header
            temp_hash=djb_hash(params[current_parameter].name);
            while(addr<CY_EEPROM_SIZE){
                for(int i=0;i<DATASET_BYTES;i++){
                    data[i] = EEPROM_READ_BYTE(addr);
                    addr++;
                }
                    addr += data[4];
                if(data[0] == 0xDE && data[1] == 0xAD && data[2] == 0xBE && data[3] == 0xEF) break;
                if((uint8_t)temp_hash == data[0] && (uint8_t)(temp_hash>>8) == data[1]&& (uint8_t)(temp_hash>>16) == data[2]&& (uint8_t)(temp_hash>>24) == data[3]){
                        found_param = 1;
                }
            }
            if(!found_param){
                ttprintf("WARNING: Param [%s] not found in EEPROM\r\n",params[current_parameter].name);
            }
        }
    }
    ttprintf("%i / %i config params loaded\r\n", change_count, param_count);
}




uint8_t getch(TERMINAL_HANDLE * handle, TickType_t xTicksToWait){
    uint8_t c=0;
    while(xTicksToWait){
    	if(LL_USART_IsActiveFlag_RXNE(pUSART.USARTx)){
    		return LL_USART_ReceiveData8(pUSART.USARTx);
    	}
    	vTaskDelay(1);
    	xTicksToWait--;
    }
    return c;
}

uint8_t getche(TERMINAL_HANDLE * handle, TickType_t xTicksToWait){
    uint8_t c=0;

    while(xTicksToWait){
		if(LL_USART_IsActiveFlag_RXNE(pUSART.USARTx)){
			c = LL_USART_ReceiveData8(pUSART.USARTx);
			break;
		}
		vTaskDelay(1);
		xTicksToWait--;
    }

    if(c){
    	while(!LL_USART_IsActiveFlag_TXE(pUSART.USARTx)){
    		vTaskDelay(1);
    	}
    	LL_USART_TransmitData8(pUSART.USARTx, c);
    }
    return c;
}

uint8_t kbhit(TERMINAL_HANDLE * handle){
	if(LL_USART_IsActiveFlag_RXNE(pUSART.USARTx)){
		return pdTRUE;
	}
    return pdFALSE;
}

#define CTRL_C  0x03

uint8_t Term_check_break(TERMINAL_HANDLE * handle, uint32_t ms_to_wait){
    uint8_t c = getch(handle,ms_to_wait /portTICK_RATE_MS);
    if(c == CTRL_C){  //0x03 = CTRL+C
        return pdFALSE;
    }else{
        return pdTRUE;
    } 
}
