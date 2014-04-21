
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "inc/lm3s8962.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "driverlib/debug.h"
#include "driverlib/pwm.h"

#include "roneos.h"
#include "ir_comms.h"

//#include "../src/crc/crctest.h"	//TESTING

// the IR transmitters transmit at 38 kHz.
#define IR_COMMS_TRANSMIT_FREQUENCY 		38000
#define IR_COMMS_PWM_PERIOD 				(SYSCTL_CLOCK_FREQ / IR_COMMS_TRANSMIT_FREQUENCY)

// the IR receivers receive at 1250bps = 800us per bit
// we use 8x oversampling, so we call the interrupt at 100us, 10000 = 100us for oversampling
#define IR_COMMS_INTERRUPT_PERIOD				10000


#define IR_RECEIVERS_PERIPH 				SYSCTL_PERIPH_GPIOD
#define IR_RECEIVERS_BASE 					GPIO_PORTD_BASE

//start bit to signify the start of a message and for the interrupt to sync
//#define IR_COMMS_RANGE_BITS_LENGTH					11 // single bit per power setting can lead to ossilations
#define IR_COMMS_RANGE_BITS_LENGTH					21
#define IR_COMMS_ORIENTATION_BITS_LENGTH 			8
#define IR_COMMS_CRC_LENGTH							16
#define IR_COMMS_MIN_PAYLOAD_FOR_CRC				9
#define IR_COMMS_BIT_SAMPLE_OFFSET					4

// IRComms interrupt states
#define IR_COMMS_IRQ_IDLE					0
#define IR_COMMS_IRQ_RECEIVE				1
#define IR_COMMS_IRQ_TRANSMIT_INDEX			0



#if defined(RONE_V6)
// no orientation hardware on this robot

#define ORIENTATION_DATAIN_SHIFT_IN			3

#elif defined(RONE_V9)

#define IR_COMMS_PWM_PIN					PWM_OUT_4
#define IR_COMMS_PWM_BASE					PWM_BASE

#define ORIENTATION_IR_CLK_PERIPH			SYSCTL_PERIPH_GPIOB
#define ORIENTATION_IR_CLK_BASE 			GPIO_PORTB_BASE
#define ORIENTATION_IR_CLK_PIN 				GPIO_PIN_3

#define ORIENTATION_XMIT_DATA_PERIPH     	SYSCTL_PERIPH_GPIOB
#define ORIENTATION_XMIT_DATA_BASE       	GPIO_PORTB_BASE
#define ORIENTATION_XMIT_DATA_PIN			GPIO_PIN_4

#elif defined(RONE_V12)

#define IR_COMMS_PWM_PIN					PWM_OUT_4
#define IR_COMMS_PWM_BASE					PWM_BASE

#define ORIENTATION_IR_CLK_PERIPH			SYSCTL_PERIPH_GPIOB
#define ORIENTATION_IR_CLK_BASE 			GPIO_PORTB_BASE
#define ORIENTATION_IR_CLK_PIN 				GPIO_PIN_3

#define ORIENTATION_XMIT_DATA_PERIPH     	SYSCTL_PERIPH_GPIOB
#define ORIENTATION_XMIT_DATA_BASE       	GPIO_PORTB_BASE
#define ORIENTATION_XMIT_DATA_PIN			GPIO_PIN_4

#define ORIENTATION_DATAIN_SHIFT_IN			4

#elif defined(RONE_IRBEACON)

#define IR_COMMS_PWM_PIN					PWM_OUT_4
#define IR_COMMS_PWM_BASE					PWM_BASE

#define ORIENTATION_IR_CLK_PERIPH			SYSCTL_PERIPH_GPIOB
#define ORIENTATION_IR_CLK_BASE 			GPIO_PORTB_BASE
#define ORIENTATION_IR_CLK_PIN 				GPIO_PIN_3

#define ORIENTATION_XMIT_DATA_PERIPH     	SYSCTL_PERIPH_GPIOB
#define ORIENTATION_XMIT_DATA_BASE       	GPIO_PORTB_BASE
#define ORIENTATION_XMIT_DATA_PIN			GPIO_PIN_4

#define ORIENTATION_DATAIN_SHIFT_IN			4

#define IR_COMMS_NUM_OF_RECEIVERS           2

#else
#error "Robot type must be defined: RONE_V6, RONE_V9, RONE_V12, or RONE_IRBEACON"
#endif

// message queues for incoming and outgoing messages
#define IR_COMMS_QUEUE_RECV_SIZE	50
#define IR_COMMS_QUEUE_XMIT_SIZE	10
static osQueueHandle irCommsQueueRecv;
static osQueueHandle irCommsQueueXmit;


// This struct holds the power PWM value and its associated maximum range.
typedef struct IRCommsPWMRangeData {
	const uint8 powerPWM;
	const uint16 rangeMM;
} IRCommsPWMRangeData;

const IRCommsPWMRangeData rangePowerData[IR_COMMS_RANGE_BITS_LENGTH] = {
	//### final 16-bit compressed range bits (presumption values are not used; bit count and linear transformation are used instead) and 8-bit spacing
//		{255,1600}, // 0
//		{255,1500}, // 1
//		{100,1400}, // 2
//		{90,1300}, // 3
//		{81,1200}, // 4
//		{73,1100}, // 5
//		{64,1000}, // 6
//		{57,900}, // 7
//		{49,800}, // 8
//		{42,700}, // 9
//		{36,600}, // 10
//		{24,500}, // 11
//		{16,400}, // 12
//		{8,300}, // 13
//		{4,200}, // 14
//		{2,100}, // 15
//		{0,200}, // 16
//		{0,100}, // 17
//		{0,200}, // 18
//		{0,100}, // 19
//		{0,200}, // 20
//		{0,100}, // 21
//		{0,200}, // 22
//		{0,100}, // 23

//		{255,1600}, // 0
//		{255,1500}, // 1
//		{100,1400}, // 2
//		{100,1300}, // 3
//		{81,1200}, // 4
//		{81,1100}, // 5
//		{64,1000}, // 6
//		{64,900}, // 7
//		{49,800}, // 8
//		{49,700}, // 9
//		{36,600}, // 10
//		{36,500}, // 11
//		{16,400}, // 12
//		{16,300}, // 13
//		{4,200}, // 14
//		{4,100}, // 15
//		{0,200}, // 16
//		{0,100}, // 17
//		{0,200}, // 18
//		{0,100}, // 19
//		{0,200}, // 20
//		{0,100}, // 21
//		{0,200}, // 22
//		{0,100}, // 23

//	{255,0},
//	{255,0},
//	{160,0},
//	{128,0},
//	{64,0},
//	{32,0},
//	{16,0},
//	{2,0},
//	{2,0},
//	{2,0},


//James: These are good values
//		{0,2445},
//		{255,2445},
//		{240,2174},
//		{220,1904},
//		{200,1634},
//		{180,1364},
//		{160,1093},
//		{140,823},
//		{120,553},
//		{100,282},
//		{80,120},

// James: I duplicated the number of bits to eliminate sampling alising

		{0,2445},
		{255,2445},
		{255,2445},
		{240,2174},
		{240,2174},
		{220,1904},
		{220,1904},
		{200,1634},
		{200,1634},
		{180,1364},
		{180,1364},
		{160,1093},
		{160,1093},
		{140,823},
		{140,823},
		{120,553},
		{120,553},
		{100,282},
		{100,282},
		{80,120},
		{80,120},


//		{240,2206},
//		{220,1979},
//		{200,1752},
//		{180,1524},
//		{160,1297},
//		{140,1070},
//		{120,842},
//		{100,615},
//		{80,388},
//		{60,161},
};

static uint8 irCommsMessageLengthPayload;
static uint8 irCommsMessageLengthCRC;
static uint8 irCommsMessageLengthTotal_bytes;
static uint8 irCommsMessageLengthTotal;

static uint8 irCommsXmitPower;

extern const uint16 CRC16_TABLE[];


/*
 * @brief Updates the given CRC.  The CRC is the  cyclic redundancy check, an error-detecting code commonly used in digital networks and storage devices to detect accidental changes to raw data.  (http://en.wikipedia.org/wiki/Cyclic_redundancy_check)
 TODO: Shouldn't CRC be capitalized?
 *
 * @param crc      The current CRC
 * @param data     the data to update the CRC from
 * @param data_len the length of the data
 * @return         The updated CRC
 */
static uint16 CRCUpdate(uint16 crc, const unsigned char *data, unsigned int data_len) {
    unsigned int tblIdx;

    while (data_len--) {
        tblIdx = (crc ^ *data) & 0xff;
        crc = (CRC16_TABLE[tblIdx] ^ (crc >> 8)) & 0xffff;

        data++;
    }
    return crc & 0xffff;
}


/*
 * @brief Calculates the CRC for the message. The CRC is the  cyclic redundancy check, an error-detecting code commonly used in digital networks and storage devices to detect accidental changes to raw data.
 *
 * @param msg[] the message to be calculated
 * @returns the CRC for the message
 */
static uint16 CRCcalculate(uint8 msg[]) {
    uint16 crc = 0x0000;
	if (irCommsMessageLengthPayload < IR_COMMS_MIN_PAYLOAD_FOR_CRC) {
		// the message is short (Probably just the robotID).  make the CRC just the inverse of the message
		crc = ~msg[0];
		// clear out the upper bits so it matches the message CRC
		crc &= ((1L << (uint32)irCommsMessageLengthCRC) - 1L);
	} else {
	    int i;
	    for (i = 0; i < irCommsMessageLengthTotal_bytes; i++) {
	        crc = CRCUpdate(crc, (unsigned char *)&msg[i], 1);
	    }
	    crc = crc ^ 0x0000;
	}
    return crc;
}


/*
 * @brief Latches the orientation data. Forces IR_CLK to clock.
 *
 * @returns void
 */
static void orientationXmitLatch(void) {
	#if (defined(RONE_V9) || defined(RONE_V12))
	MAP_GPIOPinWrite(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN, ORIENTATION_IR_CLK_PIN);
	MAP_GPIOPinWrite(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN, 0);
	#elif defined(RONE_IRBEACON)
	MAP_GPIOPinWrite(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN, ORIENTATION_IR_CLK_PIN);
	MAP_GPIOPinWrite(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN, 0);
	#endif
}


/*
 * @brief Transmits the orientation data using the shift register.
 *
 * @param data byte of data to be shifted into the shift register
 * @returns void
 */
static void orientationXmitSetOutputPins(uint8 data) {
	#if (defined(RONE_V9) || defined(RONE_V12) || defined(RONE_IRBEACON))
	static uint8 dataOld = 0xFF;
	uint8 i;

	if (dataOld != data) {
		dataOld = data;
		MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);

		for (i = 0; i < IR_COMMS_NUM_OF_TRANSMITTERS; i++){
			if (data & 0x80) {
				MAP_GPIOPinWrite(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN, ORIENTATION_XMIT_DATA_PIN);//IR_DATA //SER
			} else {
				MAP_GPIOPinWrite(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN, 0);//IR_DATA //SER
			}
			orientationXmitLatch();
			data = data << 1;
		}
		MAP_GPIOPinWrite(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN, 0);
		orientationXmitLatch();
		MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
	}
	#endif
}


/*
 * @brief Enables ir_comms interrupt.
 *
 * @returns void
 */
void irCommsIntEnable() {
    MAP_IntEnable(INT_TIMER1A);
}


/*
 * @brief Disables ir_comms.
 *
 * @returns void
 */
void irCommsIntDisable() {
    MAP_IntDisable(INT_TIMER1A);
}


/*
 * @brief Sets the transmit power of the ir_comms system for all of the transmitters.
 *
 * @param power the PWM (0-255) of the transmit strength.  note that power = 1 is the same as power 0
 */
void irCommsSetXmitPower(uint8 power) {
	irCommsXmitPower = power;
}


/*
 * @brief TODO:
 *
 * @param rangeIdx
 * @returns  TODO: does this return the minimum or the maximum range of the robot?  Shouldn't it return both, a min and a max range?
 */
uint16 irCommsComputeNbrRange(uint32 rangeBits) {	//### 32-bit
	int8 rangeIdx;

	for (rangeIdx = IR_COMMS_RANGE_BITS_LENGTH - 1; rangeIdx >= 0 ; rangeIdx--) {
		if (rangeBits & (1 << rangeIdx)) {
			break;
		}
	}

	if(rangeIdx < 0) {
		return 0;
	} else {
		rangeIdx = bound(rangeIdx, 0, IR_COMMS_RANGE_BITS_LENGTH - 1);
		return rangePowerData[rangeIdx].rangeMM;
	}
}

/*
 * @brief Set
 *   -the number of bits in the data payload
 *   -how many whole bytes are in the the message,
 *   -how many bits are in the final byte of the message.
 * @param  size, the length of the irComm payload
 */
void irCommsSetSize(uint8 size) {
	irCommsIntDisable();

	irCommsMessageLengthPayload = size;
	if (irCommsMessageLengthPayload < IR_COMMS_MIN_PAYLOAD_FOR_CRC) {
		irCommsMessageLengthCRC = size;
	} else {
		irCommsMessageLengthCRC = IR_COMMS_CRC_LENGTH;
	}
	irCommsMessageLengthTotal = IR_COMMS_RANGE_BITS_LENGTH + IR_COMMS_ORIENTATION_BITS_LENGTH + irCommsMessageLengthPayload + irCommsMessageLengthCRC;
	irCommsMessageLengthTotal_bytes = size >> 3;
	if(size & 0x07) {
		irCommsMessageLengthTotal_bytes++;
	}
	irCommsIntEnable();
}

/*
 * @brief TODO:
 *
 * @param irMessagePtr message to transmit
 * @return TRUE if  _____
 */
boolean irCommsSendMessage_internal(IRCommsMessage *irMessagePtr) {
	portBASE_TYPE val;

	val = osQueueSend(irCommsQueueXmit, (void*)(irMessagePtr), 0);
    if (val == pdPASS) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/*
 * @brief TODO:
 *
 * @param irMessagePtr message to transmit
 * @return TRUE if  _____
 */
boolean irCommsSendMessage(IRCommsMessage* irMessagePtr) {
	// If something is controlling the robot, prevent the user from
	// sending out IR messages. Otherwise send the IR message.
	if (rcMode == RC_MODE_ON) {
		return (FALSE);
	} else {
		return (irCommsSendMessage_internal(irMessagePtr));
	}
}

/*
 * @brief TODO:
 *
 * @param irMessagePtr message to transmit
 * @return TRUE if  _____
 */
boolean irCommsGetMessage_internal(IRCommsMessage* irMessagePtr) {
	portBASE_TYPE val;
	uint8 i;

	val = osQueueReceive(irCommsQueueRecv, (void*)(irMessagePtr), 0);
    if (val == pdPASS) {
        return TRUE;
    } else {
    	irMessagePtr->data[0] = 0;
    	for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
        	irMessagePtr->orientationBitMatrix[i] = 0;
		}
    	irMessagePtr->receiverBits = 0;
        return FALSE;
    }
}

/*
 * @brief TODO:
 *
 * @param irMessagePtr message to transmit
 * @return TRUE if  _____
 */
boolean irCommsGetMessage(IRCommsMessage* irMessagePtr) {
	/*
	 * If another program is controlling the robot, prevent the user
	 * from getting IR messages. Otherwise allow the user to get IR
	 * messages.
	 */
	if (rcMode == RC_MODE_ON) {
		return (FALSE);
	} else {
		return (irCommsGetMessage_internal(irMessagePtr));
	}
}

/*
 * @brief TODO:
 *
 * @param orientationBitsMatrixPtr
 * @param rangeBitsMatrixPtr
 */
void irCommsOrientationBitMatrixPrint(uint8* orientationBitsMatrixPtr) {
	uint8 i;
	char s[10];
	char s2[50];
	char vertLabel[] = "  recv  ";

	cprintf("   xmit    \n");
	cprintf("   76543210\n");
	cprintf("  +--------\n");
	for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
		sprintf(s2, "%c%d|%s\n", vertLabel[i], i, bitString8(s, orientationBitsMatrixPtr[i]));
		cprintf(s2);
	}
}

/*
 * @brief returns the number of bits received in a given ir message
 *
 * @param irMessagePtr the message received
 * @return number of bits received in message
 */
uint8 irCommsGetMessageReceiveBits(IRCommsMessage* irMessagePtr) {
	uint8 i;
	uint8 receiverBits, orientationBits;

	receiverBits = 0;
	for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
		orientationBits = irMessagePtr->orientationBitMatrix[i];
		if (orientationBits) {
			receiverBits |= (1 << i);
		}
	}
	return receiverBits;
}

/*
 * @brief initializes the infra-red comms.  Call this in the main section before using the IR comms.
 */
void irCommsInit(void) {
	// init variables for t variable message size
//	irCommsMessageLength_Bits = IR_COMMS_RANGE_BITS_LENGTH + IR_COMMS_ORIENTATION_BITS_LENGTH + IR_COMMS_MESSAGE_BIT_LENGTH_MAX + IR_COMMS_CRC_LENGTH;
//	irCommsMessageLength_bytes = IR_COMMS_MESSAGE_LENGTH_MAX;
//	irCommsMessageBitCountMax = IR_COMMS_BIT_BUFFER_LENGTH_IN_WORDS * 32;
//	wholeBytesInMessage = IR_COMMS_MESSAGE_BIT_LENGTH_MAX / 8;
//	bitsInFinalByte = IR_COMMS_MESSAGE_BIT_LENGTH_MAX % 8;
//	irCommsMessageLeftJustifyShifts = (irCommsMessageBitCountMax - irCommsMessageLength_Bits);
//	irCommsMessageBitCountOrienatationStart = irCommsMessageLength_Bits - IR_COMMS_RANGE_BITS_LENGTH - IR_COMMS_ORIENTATION_BITS_LENGTH;

	// setup system variables
	irCommsSetXmitPower(IR_COMMS_POWER_MAX);

	// init the OS message queues
	irCommsQueueRecv = osQueueCreate(IR_COMMS_QUEUE_RECV_SIZE, sizeof(IRCommsMessage));
	irCommsQueueXmit = osQueueCreate(IR_COMMS_QUEUE_XMIT_SIZE, sizeof(IRCommsMessage));

	// init the IR receiver port for gpio, set to input
    MAP_SysCtlPeripheralEnable(IR_RECEIVERS_PERIPH);
    MAP_GPIOPinTypeGPIOInput(IR_RECEIVERS_BASE, 0xFF);

	#if defined(RONE_V9)
	// init the IR_DATA port for gpio, set to output
	MAP_SysCtlPeripheralEnable(ORIENTATION_XMIT_DATA_PERIPH);
	MAP_GPIOPinWrite(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN, 0);
	MAP_GPIOPinTypeGPIOOutput(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN);

	// init the IR_CLK port for gpio, set to output
	MAP_SysCtlPeripheralEnable(ORIENTATION_IR_CLK_PERIPH);
	MAP_GPIOPinWrite(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN, 0);
	MAP_GPIOPinTypeGPIOOutput(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN);

	orientationXmitSetOutputPins(0x00);

	#elif defined(RONE_V12)
	// init the IR_DATA port for gpio, set to output
	MAP_SysCtlPeripheralEnable(ORIENTATION_XMIT_DATA_PERIPH);
	MAP_GPIOPinWrite(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN, 0);
	MAP_GPIOPinTypeGPIOOutput(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN);

	// init the IR_CLK port for gpio, set to output
	MAP_SysCtlPeripheralEnable(ORIENTATION_IR_CLK_PERIPH);
	MAP_GPIOPinWrite(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN, 0);
	MAP_GPIOPinTypeGPIOOutput(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN);

	orientationXmitSetOutputPins(0x00);
	#elif defined(RONE_IRBEACON)

	// init the IR_DATA port for gpio, set to output
	MAP_SysCtlPeripheralEnable(ORIENTATION_XMIT_DATA_PERIPH);
	MAP_GPIOPinWrite(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN, 0);
	MAP_GPIOPinTypeGPIOOutput(ORIENTATION_XMIT_DATA_BASE, ORIENTATION_XMIT_DATA_PIN);

	// init the IR_CLK port for gpio, set to output
	MAP_SysCtlPeripheralEnable(ORIENTATION_IR_CLK_PERIPH);
	MAP_GPIOPinWrite(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN, 0);
	MAP_GPIOPinTypeGPIOOutput(ORIENTATION_IR_CLK_BASE, ORIENTATION_IR_CLK_PIN);

	// init the IR beacons
	pwmInitializeBeacon();

	orientationXmitSetOutputPins(0x00);
	#endif

    //set the IR_PWM pin as pwm output
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE0_PWM4);
    MAP_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_0);

    // Set the PWM period.
    MAP_PWMGenConfigure(PWM_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    MAP_PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, IR_COMMS_PWM_PERIOD);

	#if defined(RONE_V9)
    //invert the PWM to control the shift register output enable
    MAP_PWMOutputInvert(PWM_BASE, PWM_OUT_4_BIT, true);
	#elif defined(RONE_V12)
    //invert the PWM to control the shift register output enable
    MAP_PWMOutputInvert(PWM_BASE, PWM_OUT_4_BIT, true);
	#elif defined(RONE_IRBEACON)
    MAP_PWMOutputInvert(PWM_BASE, PWM_OUT_4_BIT, true);
	#endif

    // Enable the PWM generator.
    MAP_PWMGenEnable(PWM_BASE, PWM_GEN_2);
    //enable output state.
    MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);



    // enable a 10000hz (100us) interrupt
    // timer 1a is used for the ir interrupt.  timer 1b is used for the MSP430 message interrupt
    // this code runs first, so it will enable and configure timer1
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_ONE_SHOT);
    // end shared timer init code

    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, SYSCTL_CLOCK_FREQ / IR_COMMS_INTERRUPT_PERIOD);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);

	// Enable the interrupt in the NVIC with the right priority for FreeRTOS
	MAP_IntPrioritySet(INT_TIMER1A, SYSTEM_INTERRUPT_PRIORITY_IR);
    MAP_IntEnable(INT_TIMER1A);
}


/*
 * Useful powers for the IR Beacon:
 * 		Orientation Bits Power:	(Min defined as >4 orientation bits)
 *			Power: 255		Max: 267 cm		Min: 152 cm
 *			Power: 64		Max: 229 cm		Min: 117 cm
 *			Power: 30		Max: 188 cm		Min: 81 cm
 *
 *		Message Bits Power: (No Min, but max is clearly defined)
 *			Power: 90		Max: 325 cm
 *			Power: 20		Max: 229 cm
 *			Power: 12		Max: 188 cm
 */

/*
 * @brief Gets the hardware PWM width for a given transmit power of
 * the ir_comms system.
 *
 * @param power the Power-PWM (0-128) of the transmit strength
 * @returns the hardware PWM corresponding to that Power-PWM
 */
static uint32 irCommsGetPWMWidth(uint16 power) {
	uint16 power16 = bound(power, 0, IR_COMMS_POWER_MAX);
	// increase from 255 to 256 to make division by 512 below work properly
	if (power16 > 0) {
		power16++;
	}
	// Divide the result by 512 in order to get a max duty cycle of 50%
    return (((uint32)IR_COMMS_PWM_PERIOD * (uint32)power16) >> 9);	//=IR_COMMS_PWM_PERIOD * (power16/512)
}


// IR Nav tower functionality

#if defined(RONE_IRBEACON)

#define IR_COMMS_NAV_TOWER_ORIENTATION_BITS_POWER_HIGH	255
#define IR_COMMS_NAV_TOWER_ORIENTATION_BITS_POWER_LOW	30
#define IR_COMMS_NAV_TOWER_DATA_BITS_POWER_HIGH	86
#define IR_COMMS_NAV_TOWER_DATA_BITS_POWER_LOW	12

static boolean highPowerBeacon = FALSE;

void irCommsBeaconHighPower(boolean highPower) {
	highPowerBeacon = highPower;
}


static uint32 pwmCompute(uint8 data, boolean orientationBits, uint8 xmitPower) {
	uint32 pwm = 0;
	// data is 0 or nonzero
	// oriantaion bits is true or false
	// xmit power ranges from 0-255

	//pwmSettingForNextBit = irCommsPWMOnPeriod;
	if (data) {
		if(orientationBits) {
			// The IR beacon transmits orientation bits at max power.  This gives them the same range as the normal bits
			if (highPowerBeacon) {
				xmitPower = IR_COMMS_NAV_TOWER_ORIENTATION_BITS_POWER_HIGH;
			} else {
				xmitPower = IR_COMMS_NAV_TOWER_ORIENTATION_BITS_POWER_LOW;
			}
		} else {
			// The data and range bits are trasmitted at reduced power
			if (highPowerBeacon) {
				xmitPower = IR_COMMS_NAV_TOWER_DATA_BITS_POWER_HIGH;
			} else {
				xmitPower = IR_COMMS_NAV_TOWER_DATA_BITS_POWER_LOW;
			}
		}
		pwm = irCommsGetPWMWidth(xmitPower);	//lookup table power (0~size-1)
	}
	return pwm;
}
#else

static uint32 pwmCompute(uint8 data, boolean orientationBits, uint8 xmitPower) {
	uint32 pwm = 0;
	// data is 0 or nonzero
	// oriantaion bits is true or false
	// xmit power ranges from 0-255

	// This is for robot hardware.
	// The robots transmit all bits at max power
	//TODO this should change, so that robots transmit data bits at reduced power
	if (data) {
		pwm = irCommsGetPWMWidth(xmitPower);	//lookup table power (0~size-1)

		// new style - different power for data and oriantation bits
//		if(orientationBits) {
//			// The IR beacon transmits orientation bits at max power.  This gives them the same range as the normal bits
//			xmitPower = IR_COMMS_NAV_TOWER_ORIENTATION_BITS_POWER_HIGH;
//		} else {
//			// The data and range bits are trasmitted at reduced power
//			xmitPower = IR_COMMS_NAV_TOWER_DATA_BITS_POWER_HIGH;
//		}
//		pwm = irCommsGetPWMWidth(xmitPower);	//lookup table power (0~size-1)

	}
	return pwm;
}

#endif


#define XMIT_IDLE	0
#define XMIT_DATA	1
#define XMIT_RANGE	2

/*
 * @brief Handles ir_comms transmission. Called every 100us from the main IR interrupt.
 */
static void irCommsTransmitHandler(void) {
	static uint8 bitEnableForNextBit = 0xff;	//control each transmitter for orientation bits
	static uint32 pwmSettingForNextBit = 0;	//zero/full power for normal bits, or lookup table power for range bits
	static int8 xmitBitCounter = -1;	//index of normal bit to be transmitted
	static int8 xmitBitSubCounter = 0;	//index of range bit to be transmitted, or 1/8 splitter active when 0
	static uint8 xmitMode = XMIT_IDLE;
	static uint32 messageCRC;
	static IRCommsMessage irMessage;
	portBASE_TYPE val;
	portBASE_TYPE taskWoken = pdFALSE;

	//transmit previous bit first.  This minimizes jitter between bit times
	orientationXmitSetOutputPins(~bitEnableForNextBit);
	MAP_PWMPulseWidthSet(IR_COMMS_PWM_BASE, IR_COMMS_PWM_PIN, pwmSettingForNextBit);

	if (xmitMode == XMIT_IDLE) {
		// xmit is idle.  check for a new message to transmit
		val = osQueueReceiveFromISR(irCommsQueueXmit, (void*)(&irMessage), &taskWoken);	//try to get the next message from xmit queue
		if (val == pdPASS) {	//get message from queue
			messageCRC = CRCcalculate(irMessage.data);
			xmitBitCounter = -1;
			xmitBitSubCounter = 0;
			xmitMode = XMIT_DATA;
		}
	}

	if (xmitMode == XMIT_DATA) {
		// Transmit orientation bits, message bits, and crc bits
		if(xmitBitSubCounter == 0) {
			// Update transmit bit settings only on the every 8th interrupt, i.e. when the subcounter == 0
			if(xmitBitCounter == -1) {	//start bit if needed
				pwmSettingForNextBit = pwmCompute(0xff, FALSE, irCommsXmitPower);
				bitEnableForNextBit = 0xff;	//all transmitters
			} else if(xmitBitCounter < IR_COMMS_ORIENTATION_BITS_LENGTH) {	//orientation bits (1~8)
				pwmSettingForNextBit = pwmCompute(0x01, TRUE, irCommsXmitPower);
				bitEnableForNextBit = 1 << (xmitBitCounter);	//enable only one transmitter (0~7)

			} else if(xmitBitCounter < IR_COMMS_ORIENTATION_BITS_LENGTH + irCommsMessageLengthPayload) {	//message bits (8+1~8+size)
				uint8 index = xmitBitCounter - IR_COMMS_ORIENTATION_BITS_LENGTH;	//(0~size-1)
				uint8 arrayIdx = index >> 3;
				uint8 bitIdx = index & 0x07;
				uint8 data = (irMessage.data[arrayIdx] & (0x1 << bitIdx));
				pwmSettingForNextBit = pwmCompute(data, FALSE, irCommsXmitPower);
				bitEnableForNextBit = 0xff;	//all transmitters

			} else if(xmitBitCounter < IR_COMMS_ORIENTATION_BITS_LENGTH + irCommsMessageLengthPayload + irCommsMessageLengthCRC) {	//crc bits (8+size+1~8+size+size)
				uint8 index = xmitBitCounter - IR_COMMS_ORIENTATION_BITS_LENGTH - irCommsMessageLengthPayload;	//(0~size-1)
				uint8 data = (messageCRC & (0x1 << index));
				pwmSettingForNextBit = pwmCompute(data, FALSE, irCommsXmitPower);
				bitEnableForNextBit = 0xff;	//all transmitters

			} else if(xmitBitCounter == IR_COMMS_ORIENTATION_BITS_LENGTH + irCommsMessageLengthPayload + irCommsMessageLengthCRC) {	//space
				pwmSettingForNextBit = pwmCompute(0x00, FALSE, irCommsXmitPower);
				bitEnableForNextBit = 0xff;	//all transmitters

			} else {	//finished
				xmitMode = XMIT_RANGE;
				xmitBitSubCounter = -1;	//it will become 0
			}
			xmitBitCounter++;
		}
		xmitBitSubCounter = (xmitBitSubCounter + 1) & 0x07;
	}

	if (xmitMode == XMIT_RANGE) {
		// Transmit the range bits every 100us
		//pwmSettingForNextBit = irCommsGetPWMWidth(rangePowerData[xmitBitSubCounter].powerPWM);	//lookup table power (0~size-1)
		pwmSettingForNextBit = pwmCompute(0x01, TRUE, rangePowerData[xmitBitSubCounter].powerPWM);
		bitEnableForNextBit = 0xff;
		xmitBitSubCounter++;	//next in (0~size-1)
		if(xmitBitSubCounter >= IR_COMMS_RANGE_BITS_LENGTH) {
			//end of range bits
			pwmSettingForNextBit = 0;	//zero power
			bitEnableForNextBit = 0xff;	//all transmitters
			xmitMode = XMIT_IDLE;
		}
	}
}


#define RECV_IDLE		0
#define RECV_DATA		1
#define RECV_RANGE		2
#define RECV_WAIT		3
#define RECV_PROCESS	4

static uint32 recvMsgRangeBits[IR_COMMS_NUM_OF_RECEIVERS];	//### 32-bit
static uint8 recvMsgOrientationBits[IR_COMMS_NUM_OF_RECEIVERS];
static uint8 recvMsgPayload[IR_COMMS_NUM_OF_RECEIVERS][IR_COMMS_MESSAGE_LENGTH_MAX];
static uint16 recvMsgCRC[IR_COMMS_NUM_OF_RECEIVERS];

/*
 * @brief this interrupt reads all 8 ir receivers and transmits IR transmit data for interrobot data
 * it is called every 800us
 *
 */
static void irCommsReceiveHandler(void) {
	uint32 currentTime;
	uint8 receiveBits;
	static int8 recvBitCounter = -1;	//index of normal bit to be received
	static uint8 recvBitSubCounter = 0;	//index of range bit to be received, or 1/8 splitter active when 0
	static uint8 recvMode = XMIT_IDLE;
	portBASE_TYPE val;
	portBASE_TYPE taskWoken = pdFALSE;
	boolean gotNewMessage = FALSE;
	uint16 CRC_calculated;
	uint16 CRC_calculatedFirst;
	IRCommsMessage irMessage;
	uint8 i, receiver;

	// read the bits on the port immediately to minimize jitter
	receiveBits = ~((uint8)MAP_GPIOPinRead(IR_RECEIVERS_BASE, 0xFF));
	currentTime = osTaskGetTickCountFromISR();

	if(recvMode == RECV_IDLE) {
		if(receiveBits != 0) {
			// non-zero bits mean a new message has arrived
			for (receiver = 0; receiver < IR_COMMS_NUM_OF_RECEIVERS; receiver++) {
				recvMsgRangeBits[receiver] = 0;
				recvMsgOrientationBits[receiver] = 0;
				for (i = 0; i < irCommsMessageLengthTotal_bytes; i++) {
					recvMsgPayload[receiver][i] = 0;
				}
				recvMsgCRC[receiver] = 0;
			}
			recvBitCounter = -1;
			recvBitSubCounter = 0;
			recvMode = RECV_DATA;
		}
	}

	if(recvMode == RECV_DATA) {
		if((recvBitSubCounter & 0x07) == IR_COMMS_BIT_SAMPLE_OFFSET) {
			// only sample normal bits only when recvSubCounter == IR_COMMS_BIT_SAMPLE_OFFSET
			for (receiver = 0; receiver < IR_COMMS_NUM_OF_RECEIVERS; receiver++) {
				uint8 receiveBit = (receiveBits >> receiver) & 0x01;	//0 or 1
				if(recvBitCounter == -1) {
					//start bit do nothing
				} else if(recvBitCounter < IR_COMMS_ORIENTATION_BITS_LENGTH) {
					//orientation bits (0~7)
					recvMsgOrientationBits[receiver] |= (receiveBit << recvBitCounter);

				} else if(recvBitCounter < IR_COMMS_ORIENTATION_BITS_LENGTH + irCommsMessageLengthPayload) {
					//message bits (8+1~8+size)
					uint8 index = recvBitCounter - IR_COMMS_ORIENTATION_BITS_LENGTH;	//(0~size-1)
					uint8 arrayIdx = index >> 3;
					uint8 bitIdx = index & 0x07;
					recvMsgPayload[receiver][arrayIdx] |= (receiveBit << bitIdx);

				} else if(recvBitCounter < IR_COMMS_ORIENTATION_BITS_LENGTH + irCommsMessageLengthPayload + irCommsMessageLengthCRC) {//crc bits (8+size+1~8+size+size)
					uint8 index = recvBitCounter - IR_COMMS_ORIENTATION_BITS_LENGTH - irCommsMessageLengthPayload;	//(0~size-1)
					recvMsgCRC[receiver] |= ((uint16)receiveBit << index);

				}
			}
			if(recvBitCounter >= IR_COMMS_ORIENTATION_BITS_LENGTH + irCommsMessageLengthPayload + irCommsMessageLengthCRC /*- 1*/) {
				recvMode = RECV_WAIT;
			}
			recvBitCounter++;
		}
	}

	if(recvMode == RECV_WAIT) {
		if((recvBitSubCounter & 0x07) == 0) {
			recvMode = RECV_RANGE;
			recvBitSubCounter = 0;
		}
	}

	if(recvMode == RECV_RANGE) {
		//first receive the range bits and sample at 100us
		if(recvBitSubCounter >= IR_COMMS_RANGE_BITS_LENGTH) {
			recvBitCounter = 0;
			recvBitSubCounter = 0;
			recvMode = RECV_PROCESS;
		} else {
			if(recvBitSubCounter < sizeof(uint32)*8) {	//### only at most 32 range bits can be stored in uint16
				for (receiver = 0; receiver < IR_COMMS_NUM_OF_RECEIVERS; receiver++) {
					uint8 receiveBit = (receiveBits >> receiver) & 0x01;
					recvMsgRangeBits[receiver] |= (((uint32)receiveBit) << recvBitSubCounter);	//(0~15) or (0~size-1)
				}
			}
		}
	}

	if(recvMode == RECV_PROCESS) {
		CRC_calculatedFirst = 0;
		for (receiver = 0; receiver < IR_COMMS_NUM_OF_RECEIVERS; receiver++) {
			CRC_calculated = CRCcalculate(recvMsgPayload[receiver]);
			if (recvMsgCRC[receiver] == CRC_calculated) {
				// this message has a good crc
				if (!gotNewMessage) {
					//this is the first message being processed this interrupt.  Clear the message data
					for (i = 0; i < IR_COMMS_NUM_OF_RECEIVERS; ++i) {
						irMessage.orientationBitMatrix[i] = 0;
					}
					for (i = 0; i < irCommsMessageLengthTotal_bytes; i++) {
						irMessage.data[i] = recvMsgPayload[receiver][i];
					}
					irMessage.receiverBits = 0;

					// Mark the message as received on this IR. Note that this information is NOT
					// duplicated in the orientation bits because it is possible to receive a message
					// without orientation bits.
					irMessage.receiverBits |= (0x1 << receiver);
					irMessage.orientationBitMatrix[receiver] |= recvMsgOrientationBits[receiver];
					irMessage.rangeBits = recvMsgRangeBits[receiver];
					irMessage.timeStamp = currentTime;
					CRC_calculatedFirst = CRC_calculated;
					gotNewMessage = TRUE;
					//DEBUG serial_send_long(" R", i);
					//DEBUG serial_send_long("", i);
				} else {
					//this message has just been received by another receiver
					boolean match = TRUE;
					if(CRC_calculatedFirst != CRC_calculated) {
						//first check crc match
						match = FALSE;
						//DEBUG serial_send_string(" !CRC");
					} else {
						//second check data match
						for (i = 0; i < irCommsMessageLengthTotal_bytes; i++) {
							if (irMessage.data[i] != recvMsgPayload[receiver][i]) {
								match = FALSE;
								//DEBUG serial_send_long(" !data", j);
							}
						}
					}
					if(match) {
						// The messages and crc match, update the orientation and range bits.
						irMessage.orientationBitMatrix[receiver] |= recvMsgOrientationBits[receiver];
						irMessage.receiverBits |= (0x1 << receiver);
						// or the range bits together
						//if(recvMsgRangeBits[receiver] > irMessage.rangeBits) {
						//	irMessage.rangeBits = recvMsgRangeBits[receiver];
						//}
						irMessage.rangeBits |= recvMsgRangeBits[receiver];
						//DEBUG serial_send_long("", i);
					}
				}

				if (gotNewMessage) {	//received a message and add it to the receiver queue to be taken off later by the user
					val = osQueueSendFromISR(irCommsQueueRecv, (void*)(&irMessage), &taskWoken);
					//TODO trap the return val and so something with it
				}
			}
		}
		recvMode = RECV_IDLE;
	}
	recvBitSubCounter++;
}


/*
 * @brief Handles interrupt requests generated by the IR comms.
 *
 * Clears IRQ line for next interrupt, transmit, receive, return to idle mode after message processed.
 * @returns void
 */
void irCommsHandler(void) {
	// Call transmit and receive handler every 100us.
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	irCommsTransmitHandler();
	irCommsReceiveHandler();
}


// Note: I changed first byte to 1 to prevent a message of all 0s from having a correct CRC
const uint16 CRC16_TABLE[256] = {
    0x0001, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040  };


