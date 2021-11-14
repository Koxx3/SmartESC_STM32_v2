#include "terminal.h"
#include <string.h>
#include "VescDatatypes.h"
#include "VescCommand.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "product.h"
#include "defines.h"


void terminal_top(){
    TaskStatus_t * taskStats;
    uint32_t taskCount = uxTaskGetNumberOfTasks();
    uint32_t sysTime;

    taskStats = pvPortMalloc( taskCount * sizeof( TaskStatus_t ) );
    if(taskStats){
        taskCount = uxTaskGetSystemState(taskStats, taskCount, &sysTime);

        commands_printf("Task info:");

       uint32_t foc_load = pMCI[M1]->pFOCVars->cycles_max * PWM_FREQUENCY * 100 / CPU_CLOCK;
        commands_printf("FOC Load: %d%% Cycles: %d Max Cycles: %d", foc_load, pMCI[M1]->pFOCVars->cycles_last, pMCI[M1]->pFOCVars->cycles_max);
        pMCI[M1]->pFOCVars->cycles_max = 0;
        commands_printf("Tasks: %d",  taskCount);

        uint32_t heapRemaining = xPortGetFreeHeapSize();
        commands_printf("Mem: %db Free: %db Used: %db (%d%%)", configTOTAL_HEAP_SIZE, heapRemaining, configTOTAL_HEAP_SIZE - heapRemaining, ((configTOTAL_HEAP_SIZE - heapRemaining) * 100) / configTOTAL_HEAP_SIZE);

        uint32_t currTask = 0;
        for(;currTask < taskCount; currTask++){
			char name[configMAX_TASK_NAME_LEN+1];
			strncpy(name, taskStats[currTask].pcTaskName, configMAX_TASK_NAME_LEN);
			commands_printf("%d Name: %s State: %s Runtime: %d Stack free: %d", taskStats[currTask].xTaskNumber, name, SYS_getTaskStateString(taskStats[currTask].eCurrentState), taskStats[currTask].ulRunTimeCounter, taskStats[currTask].usStackHighWaterMark);
        }
        commands_printf("EOL");
        vPortFree(taskStats);
    }
}



void terminal_process_string(char *str) {
	enum { kMaxArgs = 16 };
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	if (argc == 0) {
		commands_printf("No command received\n");
		return;
	}

	if (strcmp(argv[0], "ping") == 0) {
		commands_printf("pong\n");
	}else if (strcmp(argv[0], "top") == 0){
		terminal_top();
	}
}
