#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

void behaviorTask(void* parameters) {
	uint32 lastWakeTime = osTaskGetTickCount();

	rprintfSetSleepTime(1);
	for (;;) {
		rprintf("%u\n", (uint32)reflectiveSensorsGetValue(0));
//		cprintf("%u\n", (uint32)reflectiveSensorsGetValue(0));
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD*10);
	}
}

int main(void) {
	systemInit();
	systemPrintStartup();

	behaviorSystemInit(behaviorTask, 4096);

	osTaskStartScheduler();

	return 0;
}

