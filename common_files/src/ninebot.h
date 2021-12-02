/****************************************************************************/
//  Function: Header file ninebot communication
//  Author:   Camilo Ruiz
//  Date:    october 10 2017
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
// I am not responsible of any damage caused by the misuse of this library
// use at your own risk
//
// If you modify this or use this, please don't delete my name and give me the credits
// Greetings from Colombia :) 
// Hardware port by ub4raf
/****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
//#include "mbed.h"
#define NinebotHeader0 0x5A//0x55
#define NinebotHeader1 0xAA//0xAA
#define Ninebotread 0x01
#define Ninebotwrite 0x03
#define NinebotMaxPayload 0x38
//message len max is 256, header, command, rw and cheksum total len is 8, therefore payload max len is 248
//max input bluetooth buffer in this chip allows a payload max 0x38
typedef struct {
	uint8_t start1;
	uint8_t start2;
	uint8_t len;
	uint8_t addr;
	uint8_t cmd;
    uint8_t arg;
    //uint8_t payload[NinebotMaxPayload];
    uint8_t payload[4];
    uint8_t CheckSum[2];
} NinebotPack;


void addCRC(uint8_t * message, uint8_t size);


uint8_t ninebot_parse(uint8_t data, NinebotPack *message);
