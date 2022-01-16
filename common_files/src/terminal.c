#include "terminal.h"
#include <string.h>
#include "VescDatatypes.h"
#include "VescCommand.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "product.h"
#include "defines.h"
#include "music.h"
#include "stdlib.h"
#include "VescToSTM.h"
#include "ninebot.h"

void terminal_top(PACKET_STATE_t * phandle){
    TaskStatus_t * taskStats;
    uint32_t taskCount = uxTaskGetNumberOfTasks();
    uint32_t sysTime;

    taskStats = pvPortMalloc( taskCount * sizeof( TaskStatus_t ) );
    if(taskStats){
        taskCount = uxTaskGetSystemState(taskStats, taskCount, &sysTime);

        commands_printf(phandle, "Task info:");

       uint32_t foc_load = pMCI[M1]->pFOCVars->cycles_max * PWM_FREQUENCY * 100 / CPU_CLOCK;
        commands_printf(phandle, "FOC Load: %d%% Cycles: %d Max Cycles: %d", foc_load, pMCI[M1]->pFOCVars->cycles_last, pMCI[M1]->pFOCVars->cycles_max);
        pMCI[M1]->pFOCVars->cycles_max = 0;
        commands_printf(phandle, "Tasks: %d",  taskCount);

        uint32_t heapRemaining = xPortGetFreeHeapSize();
        commands_printf(phandle, "Mem: %db Free: %db Used: %db (%d%%)", configTOTAL_HEAP_SIZE, heapRemaining, configTOTAL_HEAP_SIZE - heapRemaining, ((configTOTAL_HEAP_SIZE - heapRemaining) * 100) / configTOTAL_HEAP_SIZE);

        uint32_t currTask = 0;
        for(;currTask < taskCount; currTask++){
			char name[configMAX_TASK_NAME_LEN+1];
			strncpy(name, taskStats[currTask].pcTaskName, configMAX_TASK_NAME_LEN);
			commands_printf(phandle, "%d Name: %s State: %s Runtime: %d Stack free: %d", taskStats[currTask].xTaskNumber, name, SYS_getTaskStateString(taskStats[currTask].eCurrentState), taskStats[currTask].ulRunTimeCounter, taskStats[currTask].usStackHighWaterMark);
        }
        commands_printf(phandle, "EOL");
        vPortFree(taskStats);
    }
}

#if MUSIC_ENABLE
extern MUSIC_PARAM bldc_music;
#endif
extern m365Answer m365_to_display;
uint32_t xx_load;
void terminal_process_string(char *str, PACKET_STATE_t * phandle) {
	enum { kMaxArgs = 16 };
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	if (argc == 0) {
		commands_printf(phandle, "No command received\n");
		return;
	}

	if (strcmp(argv[0], "ping") == 0) {
		commands_printf(phandle, "pong\n");
	}else if (strcmp(argv[0], "top") == 0){
		terminal_top(phandle);
	}
#if MUSIC_ENABLE
	else if (strcmp(argv[0], "music_on") == 0){
		set_music_command(Music1, &bldc_music);
	}else if (strcmp(argv[0], "music_off") == 0){
		set_music_command(Music_OFF, &bldc_music);
	}
#endif
	else if (strcmp(argv[0], "foc_openloop") == 0) {
		if (argc == 3) {
			uint32_t erpm_target = 0;
			uint32_t current = atoi(argv[1]);
			erpm_target = atoi(argv[2]);

			if (current >= 0 && erpm_target >= 0) {
				VescToSTM_set_open_loop(true, SpeednTorqCtrlM1.SPD->hElAngle, 0);
				VescToSTM_ramp_current(current, 0);
				uint32_t erpm=0;
				while(erpm < erpm_target){
					VescToSTM_set_open_loop_rpm(erpm);
					erpm += 1;
					vTaskDelay(MS_TO_TICKS(1));
					VescToSTM_timeout_reset();
				}

			} else {
				commands_printf(phandle, "Invalid argument(s).\n");
			}
		} else {
			commands_printf(phandle, "This command requires two arguments.\n");
		}
	}else if (strcmp(argv[0], "mode") == 0){

		m365_to_display.mode = atoi(argv[1]);
		commands_printf(phandle, "Mode %u", m365_to_display.mode);
	}else if (strcmp(argv[0], "bench") == 0){
		commands_printf(phandle, "Cycles: %u", xx_load);
	}

}
