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

#include "task_LED.h"
#include "task_init.h"
#include "main.h"


osThreadId_t LEDHandle;
const osThreadAttr_t LED_attributes = {
  .name = "LED",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};

void prv_LED_blink(uint32_t ticks){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, pdFALSE);
	osDelay(ticks);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, pdTRUE);
	osDelay(ticks);
}


void task_LED(void * argument)
{

  /* Infinite loop */
  for(;;)
  {
	  if(pMCI[M1]->pSTM->hFaultOccurred){
		  prv_LED_blink(200);
	  }else{
		  prv_LED_blink(1000);
	  }
  }
}

void task_LED_init(){
	LEDHandle = osThreadNew(task_LED, NULL, &LED_attributes);
}
