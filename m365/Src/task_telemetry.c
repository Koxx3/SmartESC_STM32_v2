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

#include "task_telemetry.h"
#include "task_init.h"
#include "task_cli.h"
#include "main.h"


osThreadId_t telemetryHandle;
const osThreadAttr_t telemetry_attributes = {
  .name = "tele",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};

extern MCT_Handle_t* pMCT[NBR_OF_MOTORS];
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];

void task_telemetry(void * argument)
{


  /* Infinite loop */
  for(;;)
  {

	  /*msg_rep.voltage = VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor)*10;
	  msg_rep.temperature = NTC_GetAvTemp_C(pMCT[M1]->pTemperatureSensor);
	  msg_rep.rpm = (int32_t)((MCI_GetAvrgMecSpeedUnit(pMCI[M1]) * _RPM)/SPEED_UNIT);*/
	  osDelay(100);
  }
}

void task_telemetry_init(){
	telemetryHandle = osThreadNew(task_telemetry, NULL, &telemetry_attributes);
}
