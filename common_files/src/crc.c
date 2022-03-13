/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "crc.h"
#include "stm32f1xx.h"

unsigned short  crc16(unsigned char *Buffer, unsigned int Len)
{
   int16_t x;
   int16_t crc = 0x0000;

   while(Len--)
   {
      x = ((uint8_t)(crc>>8)) ^ *Buffer++;
      x ^= x>>4;

      crc = (crc << 8) ^ (x << 12) ^ (x <<5) ^ x;
   }
   return crc;
}

/**
  * @brief  Computes the 32-bit CRC of a given buffer of data word(32-bit) using
  * Hardware Acceleration.
  * @param  pBuffer: pointer to the buffer containing the data to be computed
  * @param  BufferLength: length of the buffer to be computed
  * @retval 32-bit CRC
  */
uint32_t crc32(uint32_t *pBuffer, uint32_t BufferLength) {
	uint32_t index = 0;

	for(index = 0; index < BufferLength; index++) {
		CRC->DR = pBuffer[index];
	}

	return (CRC->DR);
}

/**
  * @brief  Resets the CRC Data register (DR).
  * @param  None
  * @retval None
  */
void crc32_reset(void) {
	/* Reset CRC generator */
	CRC->CR |= CRC_CR_RESET;
}
