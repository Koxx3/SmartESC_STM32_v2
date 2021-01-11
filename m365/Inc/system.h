#if !defined(system_H)
#define system_H

#include "FreeRTOS.h"
#include "task.h"
    

uint32_t SYS_getCPULoadFine(TaskStatus_t * taskStats, uint32_t taskCount, uint32_t sysTime);
const char * SYS_getTaskStateString(eTaskState state);    
    
    
#endif