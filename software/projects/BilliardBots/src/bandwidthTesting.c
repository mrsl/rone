#include <stdio.h>
#include <stdlib.h>
#include "roneos.h"
#include "ronelib.h"

// Behavior
#define BEHAVIOR_TASK_PRIORITY	(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD	50

// States
#define STATE_IDLE 		0
#define STATE_LISTEN 	1
#define STATE_SEND		2

// Constants
#define MAX_BANDWIDTH	250000	// Theoretical maximum bytes per second
#define MAX_ID			255		// Maximum robot ID we will see
#define CHECK			0xDADA	// Constant 32-bit Number to mark payloads

uint32 messageDelay = 1000;		// Time between broadcast messages

/**
 * Structure of packets sent to monitor bandwidth
 */
struct __attribute__((__packed__)) {
	uint32 check;		// Set preamble to check if packet is relevant
	uint32 nonce;		// Nonce used to measure dropped packets
	uint8 id;			// Source Robot ID
	char padding[21];	// Padding out to 30 bytes
} typedef payload;

/**
 * Structure of packets sent to change broadcast rate
 */
struct __attribute__((__packed__)) {
	uint32 check;		// Set preamble to check if packet is relevant
	uint32 delay;		// Delay
	char padding[22];	// Padding out to 30 bytes
} typedef broadcastSet;


/**
 * Information state
 */
struct {
	uint32 lastNonce;	// Last nonce recieved from this message type and ID
	uint32 totalRecv;	// Total packets recieved of this type and ID
	uint32 totalDrop;	// Total dropped packets figured from missing nonce vals
} typedef networkState;

static networkState packetInfo[MAX_ID];	// Network information

// Variables for testing
RadioCmd testMessage;	// Radio command used in broadcast
RadioMessage myMessage;	// Radio Message
uint32 myNonce = 1;		// Nonce used in broadcast

uint32 startTime;		// Starting time of packet sniffing
uint32 otherPackets;	// Packets other than ours that we recieve

// Variables for setup
SerialCmd serialCmdBC;		// Serial command to set broadcast rate
RadioCmd setBroadcastRate;	// Radio command to set broadcast rate
RadioMessage broadcastRate;	// Radio Message


/**
 * @brief Update information in the packetInfo array given a new packet
 */
void updatePacketInfo(payload *packetPtr) {
	uint8 id = packetPtr->id;
	uint32 nonce = packetPtr->nonce;

	packetInfo[id].totalRecv++;
	if (packetInfo[id].lastNonce != 0) {
		packetInfo[id].totalDrop += (nonce - packetInfo[id].lastNonce) - 1;
	}
	packetInfo[id].lastNonce = nonce;
}

/**
 * @brief Process the information contained in a received packet
 */
void processRadioMessage(RadioMessage *messagePtr) {
	payload *newPacket = (payload *)radioCommandGetDataPtr(messagePtr);

	if (newPacket->check == CHECK) {
		updatePacketInfo(newPacket);
	} else {
		otherPackets++;
	}
}

/**
 * @brief Fills out a radio message for broadcast
 */
void makeRadioMessage(RadioMessage *messagePtr) {
	payload *newPacket = (payload *)radioCommandGetDataPtr(messagePtr);

	newPacket->check = CHECK;
	newPacket->nonce = myNonce++;
	newPacket->id = roneID;
}

/**
 * @brief Calculate the current network bandwidth given an amount of packets
 */
uint32 calculateBandwidth(uint32 packetsRecv, uint32 currentTime) {
	return (packetsRecv * RADIO_MESSAGE_LENGTH_RAW) / ((currentTime - startTime) / 1000);
}

/**
 * @brief Output information about current state of the network
 */
void outputCurrentNetworkData() {
	int i;
	uint32 bandwidth;
	uint32 totalRecv = 0;
	uint32 totalDrop = 0;
	uint32 total;
	uint8 numRobots = 0;
	uint32 currentTime = osTaskGetTickCount();

	for (i = 0; i < MAX_ID; i++) {
		if (packetInfo[i].lastNonce != 0) {
			totalRecv += packetInfo[i].totalRecv;
			totalDrop += packetInfo[i].totalDrop;
			numRobots++;
		}
	}

//	if (!numRobots) {
//		return;
//	}

	total = totalRecv + totalDrop;
	bandwidth = calculateBandwidth(totalRecv, currentTime);

	cprintf("Network Bandwidth       : %6u B/s\n", bandwidth);
	cprintf("Percent of Max Bandwidth: %3u\n", bandwidth * 100 / MAX_BANDWIDTH);
	cprintf("Total Packets Received  : %10u - %3u\n", totalRecv,
													  totalRecv * 100 / total);
	cprintf("Total Packets Dropped   : %10u - %3u\n", totalDrop,
													  totalDrop * 100 / total);
	cprintf("\n");

	cprintf("Other Packets Received : %10u\n", otherPackets);
	bandwidth = calculateBandwidth(totalRecv + otherPackets, currentTime);
	cprintf("Total Network Bandwidth: %6u B/s\n", bandwidth);
	cprintf("\n");

	cprintf("Individual Robot Breakdown:\n");

	for (i = 0; i < MAX_ID; i++) {
		if (packetInfo[i].lastNonce != 0) {
			total = packetInfo[i].totalRecv + packetInfo[i].totalDrop;
			cprintf("Robot ID: %u\n", i);
			cprintf("    Packets Received: %10u - %3u\n", packetInfo[i].totalRecv,
														  packetInfo[i].totalRecv * 100 / total);
			cprintf("    Packets Dropped : %10u - %3u\n", packetInfo[i].totalDrop,
														  packetInfo[i].totalDrop * 100 / total);
		}
	}

	cprintf("\n");
}

/**
 * @brief If "bc <number>" received over serial, set broadcast rate of send mode to following unsigned integer
 */
void serialCmdBCFunc(char* command) {
	broadcastSet *newPacket = (broadcastSet *)radioCommandGetDataPtr(&broadcastRate);

	command = command + 2;
	newPacket->check = CHECK;
	if (sscanf(command, "%u", (unsigned int *)&newPacket->delay) != 1) {
		cprintf("Invalid number\n");
		return;
	}

	cprintf("Broadcast message delay set to %u ms! Sending to other robots...\n", newPacket->delay);

	radioCommandXmit(&setBroadcastRate, ROBOT_ID_ALL, &broadcastRate);
}

/**
 *
 */
void initRadioMonitoring() {
	int i;

	// We have received no other packets
	otherPackets = 0;

	// Initialize packet data information array
	for (i = 0; i < MAX_ID; i++) {
		packetInfo[i].lastNonce = 0;
		packetInfo[i].totalRecv = 0;
		packetInfo[i].totalDrop = 0;
	}

	// Start-time
	startTime = osTaskGetTickCount();

	// Use our processRadioMessage function to process packets
	radioMonitorInitDebug(&processRadioMessage);
}

void behaviorTask(void* parameters) {
	int state = STATE_IDLE;		// State
	uint32 lastWakeTime;		// Last time when task was awakened
	Beh behOutput;				// Behavior
	uint32 buttonRed,			// Buttons
		   buttonGreen,
		   buttonBlue;
	RadioMessage myMessage;		// Testing message
	broadcastSet *setDelay;

    // Print startup message and thread memory usage
	systemPrintStartup();
	systemPrintMemUsage();

	serialCommandAdd(&serialCmdBC, "bc", serialCmdBCFunc);

	// Disable the neighbor system
	neighborsDisable();

	// Initialize all radio commands needed
	radioCommandAddQueue(&testMessage, "TEST", 1);
	radioCommandAddQueue(&setBroadcastRate, "broadcastRate", 1);

	for (;;) {
		lastWakeTime = osTaskGetTickCount();

		behOutput = behInactive;
		motorSetBeh(&behOutput);

		// Set state based on button presses
		buttonRed = buttonsGet(BUTTON_RED);
		buttonGreen = buttonsGet(BUTTON_GREEN);
		buttonBlue = buttonsGet(BUTTON_BLUE);

		switch (state) {
		case (STATE_LISTEN): {
			if (buttonGreen) {
				outputCurrentNetworkData();
				state = STATE_IDLE;
			}
			if (buttonRed) {
				outputCurrentNetworkData();
			}
			break;
		}
		case (STATE_SEND): {
			if (buttonGreen) {
				state = STATE_IDLE;
			}
			break;
		}
		case (STATE_IDLE):
		default: {
			if (buttonRed) {
				// Initialize radio monitoring
				initRadioMonitoring();
				state = STATE_LISTEN;
			} else if (buttonBlue) {
				state = STATE_SEND;
			}
			break;
		}
		}

		// Set radio debugging mode
		if (state != STATE_LISTEN) {
			radioMonitorInitDebug(NULL);
		}

		// Set LEDs for each state
		switch (state) {
		case (STATE_SEND): {
			ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
			break;
		}
		case (STATE_LISTEN): {
			ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_FAST);
			break;
		}
		case (STATE_IDLE):
		default: {
			ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
			break;
		}
		}

		if (state == STATE_SEND) {
			// Broadcast out marked packet
			makeRadioMessage(&myMessage);
			radioCommandXmit(&testMessage, ROBOT_ID_ALL, &myMessage);
			osTaskDelayUntil(&lastWakeTime, messageDelay);
		} else if (state == STATE_IDLE) {
			// Check for message that sets new radio broadcast rate
			if (radioCommandReceiveNonBlocking(&setBroadcastRate, &broadcastRate)) {
				setDelay = (broadcastSet *) radioCommandGetDataPtr(&broadcastRate);
				if (setDelay->check == CHECK) {
					ledsSetPattern(LED_ALL, LED_PATTERN_ON, LED_BRIGHTNESS_HIGH, LED_RATE_TURBO);
					osTaskDelayUntil(&lastWakeTime, 50);
					messageDelay = setDelay->delay;
					cprintf("Broadcast message delay set to %u ms!\n", messageDelay);
				}
			}
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		} else {
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);
		}
	}
}


int main() {
	// Initialize system
	systemInit();
	systemPrintStartup();

	behaviorSystemInit(behaviorTask, 4096);

	osTaskStartScheduler();
	return (0);
}

