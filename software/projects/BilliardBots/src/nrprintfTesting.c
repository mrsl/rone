#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

void behaviorTask(void* parameters) {
	uint32 lastWakeTime;
	uint32 buttonRed,			// Buttons
		   buttonGreen,
		   buttonBlue;

	nrprintfInit();

	for (;;) {
		lastWakeTime = osTaskGetTickCount();

		buttonRed = buttonsGet(BUTTON_RED);
		buttonGreen = buttonsGet(BUTTON_GREEN);
		buttonBlue = buttonsGet(BUTTON_BLUE);

		if (buttonRed) {
			nrprintf("Hello!");
			nrprintf("my name is");

			nrprintf(" zak!");
			nrprintf("\n");

			nrprintfFlush();
		}

		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
	}
}


int main() {
	// Initialize system
	systemInit();

	behaviorSystemInit(behaviorTask, 4096);

	osTaskStartScheduler();
	return (0);
}

