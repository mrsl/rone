#include <stdio.h>
#include <msp430f2132.h>

#include "typedefs.h"
#include "SPI_8962.h"
#include "Gyroscope.h"
#include "Accelerometer.h"
#include "Magnetometer.h"
#include "BumpSensors.h"
#include "ReflectiveSensors.h"
#include "RFIDReader.h"
#include "IRBeacon.h"
#include "I2C.h"
#include "leds.h"
#include "PowerController.h"
#include "Buttons.h"
#include "SPIMessage.h"
#include "bootStrapLoader.h"

#define DEBUGGING
/* 
 * registers 
 */
#define TIMER_A_INTERRUPT_VECTOR		TAIV
#define TIMER_A_CONTROL_REGISTER		TACTL
#define TIMER_A_COUNT					TAR		 // This is the actual 16 bit counter value
#define TIMER_A_CAPCOMPARE_CONTROL_0	TACCTL0	 // Two functions for each mode (which is set by CAP in the TACCTL register)
#define TIMER_A_CAPCOMPARE_CONTROL_1	TACCTL1  
#define TIMER_A_CAPCOMPARE_CONTROL_2	TACCTL2  
#define TIMER_A_CAPCOMPARE_REGISTER_0	TACCR0
#define TIMER_A_CAPCOMPARE_REGISTER_1	TACCR1 
#define TIMER_A_CAPCOMPARE_REGISTER_2	TACCR2

//Capture/compare interrupt enable
#define CAPCOMPARE_INTERRUPT_ENABLE	CCIE
 
//Timer A clock source select: 2 - sub main clock SMCLK */
#define TIMER_A_SOURCE_SELECT_SMCLK		TASSEL_2
#define TIMER_A_MODE_STOP				MC_0
#define TIMER_A_MODE_UP					MC_1
#define TIMER_A_MODE_CONTINUOUS			MC_2
#define INPUT_DIVIDER_8					ID_3

//4294967295	// 2^32-1
#define TIMER_COUNT_MAX					0xFFFFFFFF


#define I2C_DELAY				50000
#define I2C_MESSAGE_DELAY		1000

#define BLINKY_DELAY_ON				20
#define BLINKY_DELAY_OFF			200
#define SYSTEM_LED_BRIGHTNESS		20
#define SYSTEM_LED_DIMMER_DELAY		150

/* The voltage at which the MSP430 will shut the robot down to prevent damage
 * This is 10x the voltage (e.g. 3.6V = 36) */
#define VBAT_SHUTDOWN_THRESHOLD			35
#define VBAT_SHUTDOWN_BLINK_DELAY		300000
#define VBAT_SHUTDOWN_BLINK_LED_TIMES	3
#define VBAT_SHUTDOWN_BLINK_LED			10
#define VBAT_SHUTDOWN_BLINK_LED_PWM		45

#ifdef RONE_V12
	#define USB_5V_1000mAMP_THRESHHOLD          53
#endif


#define BATTERY_LED_FADE_IN_SPEED	40 //Smaller is faster

#define POWER_OFF_DELAY				50000
#define POWER_ON_DELAY				500000
#define RESET_DELAY					50000

// at 600hz = 0.4s
#define POWER_BUTTON_HOLD_DELAY_OFF 	240

// 600hz = 3.4s
#define POWER_BUTTON_HOLD_DELAY_AIRPLANE 	2040

// 6 600hz ticks
#define POWER_BUTTON_HOLD_DELAY_RESET 	6

#define AIRPLANE_MODE_DELAY			1800	
#define AIRPLANE_LIGHT_DELAY		200000

// read ADC at 10 hz
#define TIMER_ADC_PERIOD			6

uint32 powerButtonStartTicks = 0;
//Sate variable used to control the off button features
//powerButtonFSMState = 0: idle
//powerButtonFSMState = 1: reset state, will reset the robot when the 
//						   button is relesed
//powerButtonFSMState = 2: off state, will turn the robot off (no airplane
//						   mode) when the button is released, will also 
//						   show the current battery state and time-to-airplane
//powerButtonFSMState = 3: airplane mode state, will turn off the robot and 
//						   put the robot into airplane mode
volatile uint8 powerButtonFSMState = 0;

//Start with the robot off and in airplane mode
volatile boolean powerOffRequest = TRUE;
volatile boolean powerOnReset = FALSE;
volatile boolean powerResetRequest = FALSE;
volatile boolean airplaneMode = TRUE;
volatile uint32 airplaneStartTicks;

#ifdef RONE_V12
	// Default to FALSE so that wre can see startup information
	boolean resetFTDIWithButton = FALSE;
#endif

volatile uint16 timerADCUpdate = TIMER_ADC_PERIOD;

volatile uint32 systemTicksVal = 0;
volatile uint8 timer60hz = 0;
volatile uint8 timer10hz = 0;

//Array to hold the LED values for the power off LEDs
//LED_NUM_ELEMENTS_SPI_MSG is found in Leds.h
uint8 powerButtonLEDOverideData[LED_NUM_ELEMENTS_SPI_MSG];
uint8 powerButtonLEDOveride = FALSE;

volatile uint8 batteryLEDFadeIn = 0;
volatile uint8 batteryLEDFadeInCount = 0;

volatile uint32 currentTicks;

volatile uint32 systemLEDRampTimer = 0;
volatile int16 systemLEDRampBrightness = -1;
volatile int16 systemLEDRampBrightnessOld = -1;


void setPowerOffRequest(boolean powerOff){
	powerOffRequest = powerOff;
}

void setPowerResetRequest(boolean powerOffAndOn) {
	powerResetRequest = powerOffAndOn;
}

void watchdogInit(void){
	//TODO: RFID debug
#ifndef RFID_DEBUG
	//Setup sACLK to come from VLOCLK which has a 4kHz to 20kHz
	//range. This is internal and very low power. It is divided
	//by 1.
	//BCSCTL1 &= ~XTS; //Clear the XTS bit, setting it to low frequency mode
	//BCSCTL3 &= ~LFXT1S0; //Clear the LFXT1S bit 0 so that VLOCLK is selected
	BCSCTL3 = LFXT1S1; //Set the LFXT1S1 bit so that VLOCLK is selected
	
	//Setup the watchdog to do PUC resets using the ACLK
	//The ACLK runs at 32kHz and is internal, therefore
	//The watchdog will need to be petted more often than
	//once every 250ms. The division of the clock is 8192.
	//Also turn on the watch dog and pet it.
	WDTCTL = WDTPW+WDTCNTCL+WDTSSEL+WDTIS0; //This resets at about 8192 Counts*(1/ 12000 Hz)=0.68 Sec
#endif
}

void watchdogEnable(void){
	//TODO: RFID debug
#ifndef RFID_DEBUG
	WDTCTL = WDTPW+WDTCNTCL+WDTSSEL+WDTIS0; //This resets at about 8192 Counts*(1/ 12000 Hz)=0.68 Sec
#endif
}

void watchdogDisable(void){
	// Stop WDT
	WDTCTL = WDTPW + WDTHOLD;
}

void watchdogPet(void){
	//Pet the watchdog
	WDTCTL = WDTPW+WDTCNTCL+WDTSSEL+WDTIS0; //See Watchdog enable
}

void timerEnable(void) {
	// Set clock controls to use the SMCLK, count in up mode 
	// (up to TACCR0, restart at 0), and to divide the source clock
	// by 8.
	TIMER_A_CONTROL_REGISTER = TIMER_A_SOURCE_SELECT_SMCLK | 
							   TIMER_A_MODE_UP | 
							   INPUT_DIVIDER_8;
}

/*
 * Disables timer A. This conserves power and prevents the IR beacon interrupt
 * function from being called.
 */
void timerDisable(void) {
	TIMER_A_CONTROL_REGISTER = TIMER_A_SOURCE_SELECT_SMCLK | 
							   TIMER_A_MODE_STOP | 
							   INPUT_DIVIDER_8;
}


void timerInit(void) {
	// Enable the timer 1 interrupt
	TIMER_A_CAPCOMPARE_CONTROL_0 = CAPCOMPARE_INTERRUPT_ENABLE;
	
	// Set TACCR0. TACCR0 + 1 is the number of ticks in the period
	// because the clock ticks from 0 to TACCR0 inclusive.
	//   3333 should be the number of ticks on a 16 MHz clock
	//   to render an interrupt period of 600Hz if the input clock
	//   is being divided by 8. (16Mhz / 8 / 600 Hz = # ticks)
	TACCR0 = 3333 - 1;
	
	// Set clock controls to use the SMCLK, count in up mode 
	// (up to TACCR0, restart at 0), and to divide the source clock
	// by 8.
	timerEnable();
}


uint32 systemTicks(void) {
	return systemTicksVal;
}


uint32 timerDiff(uint32 oldval, uint32 newval) {
	uint32 difference;
	if (newval > oldval) {
		difference = newval - oldval;
	} else {
		difference = (TIMER_COUNT_MAX - oldval) + (newval + 1);
	}
	return difference;
}


boolean airplaneModeGetState(void) {
	return airplaneMode;	
}


void setSystemLEDRampBrightness(int16 newBrightness){
	systemLEDRampBrightness = newBrightness;
}


void setAllLEDData(uint8 *data, uint8 value){
	//Sets all of the values in the array data
	//of length LED_NUM_ELEMENTS_SPI_MSG, to value
	int i;
	for(i = 0; i<LED_NUM_ELEMENTS_SPI_MSG; i++){
		data[i] = value;
	}
}


void setBatteryLEDData(uint8 *data){
	unsigned long int vbatt = (long int)powerVBatGet();
	int i = 10;
	//The range of vbat is from 
	//4.3V = 43 = 5 Lights
	//3.5V = 35 = 0 Lights
	batteryLEDFadeInCount++;
	if(batteryLEDFadeInCount > BATTERY_LED_FADE_IN_SPEED){
		batteryLEDFadeInCount = 0;
		if(batteryLEDFadeIn < 255){
			batteryLEDFadeIn++;
		}
	}
	if(vbatt >= VBAT_SHUTDOWN_THRESHOLD){
		vbatt = (vbatt-VBAT_SHUTDOWN_THRESHOLD)*(1275/(43-VBAT_SHUTDOWN_THRESHOLD));
		while(i < LED_NUM_ELEMENTS_SPI_MSG){
			if(vbatt > 255){
				data[i] = batteryLEDFadeIn;
				vbatt -= 255;
			} else {
				data[i] = (vbatt*batteryLEDFadeIn)>>8;
				break;
			}
			i++;
		}
	}
}


void setTimeToAirplaneLEDData(uint8 *data, uint32 timeLeft){
	uint8 lightsOn = (uint8)(timeLeft/((POWER_BUTTON_HOLD_DELAY_AIRPLANE-POWER_BUTTON_HOLD_DELAY_OFF)/5))+1;
	uint8 i;
	for(i=1; i<6; i++){
		if((i%5)<lightsOn){
			data[i-1] = 120;
		} else {
			data[i-1] = 0;
		}
	}
}


#ifdef RONE_V12
	void resetFTDIWithButtonSet(boolean r){
		resetFTDIWithButton = r;
	}
#endif


void main(void) {
	uint32 i;

	// Enable interupts
	__bis_SR_register(GIE);

	// run at 16mhz
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

#ifdef RFID_DEBUG
	//Start with the robot on
	powerOffRequest = FALSE;
	powerOnReset = TRUE;

	// Start with the robot in airplane mode
	airplaneMode = FALSE;
#else
	//Start with the robot off
	powerOffRequest = TRUE;
	powerOnReset = FALSE;

	// Start with the robot in airplane mode
	airplaneMode = TRUE;

	//check bootloader version and update as needed
	uint8 *msp430BSLVersionPtr = (uint8 *)MSP430_BSL_VERSION_ADRESS;
	if (*msp430BSLVersionPtr != getStoredBSLVersion()){
		updateMSP430BSLVersion();
	}
#endif
	
	//Start the watchdog
	watchdogInit();
	
	//Initalize the power controller module	
	resetInit();
	ADC10Init();

#ifdef RONE_V12
	ftdiResetInit();
	ledResetInit();
	motSleepInit();
	chargeLimitInit();
#endif
	// Tiletrack functions
	reflectiveSensorsInit();
	RFIDReaderInit();

	powerButtonInit();
	powerEnInit();
	powerUSBInit();
#ifdef RONE_V11
	ir_beacon_init();
#endif

	while(1) {
		//Pet the watchdog so that we don't get reset
		watchdogPet();
		
		// Power-off: Disable all modules and go to LPM4 until woken up by power button press
		if (powerOffRequest) {
			// clear the flag
			powerOffRequest = FALSE;
			
			// disable all interrupts except for the power button irq
#ifdef RONE_V11
			irBeaconDisable();
#endif
			RFIDInterruptDisable();
			reflectiveSensorPowerDisable();
			watchdogPet();
			watchdogDisable();
			timerDisable();
			SPI8962Shutdown();
			I2CShutdown();
			ADC10Shutdown();
			
			// wait for I2C to finish
			for(i = 0; (UCB0STAT & UCBBUSY) && (i < I2C_MAX_WHILE_DELAY) ; i++) {}
			
			// reset the robot and shut down the main supply
			resetSet(TRUE);
			#ifdef RONE_V12
				ftdiResetSet(TRUE);
				ledResetSet(TRUE);
				motSleepSet(TRUE);
				chargeLimitSet(FALSE);
			#endif
			//msp430SetRobotMode(MSP430_MODE_PYTHON);
			powerUSBSetEnable(FALSE); // Turn off the USB Power Sense Line comparator
			powerEnSet(FALSE);
			for (i = 0; i < POWER_OFF_DELAY; ++i) {}

			// enable power button interrupt
			powerButtonIRQEnable();
			
			// set soft reset flag
			powerOnReset = TRUE;

			// shutdown the MSP430 and wait for power button interrupt or usb
			if (!powerResetRequest) {
				__bis_SR_register(LPM4_bits + GIE);
 			} else {
				powerResetRequest = FALSE;
				for (i = 0; i < POWER_OFF_DELAY; ++i) {}
			}
		}
					
		// Power-on/reset and initialize all components
		if (powerOnReset) {
			// disable interrupts
			__bic_SR_register(GIE);
			powerButtonIRQDisable();
			watchdogDisable();

			powerOnReset = FALSE;
			airplaneMode = FALSE;	
			
			#ifdef RONE_V12
				// Set Charge Limit to high to force initial charge at 500mA
				chargeLimitSet(TRUE);
			#endif
			
			// Turn on the USB Power Sense Line comparator
			powerUSBSetEnable(TRUE);
			
			// Turn on the main power supply and wait for it to stabilize
			powerEnSet(TRUE);
			for (i = 0; i < POWER_ON_DELAY; i++) {}

			// Initialize i/o and timer
			I2CInit();
			SPI8962Init();
			timerInit();

			//Start the watchdog back up
			watchdogInit();

			// enable normal CPU operation, enable interrupts
			__bis_SR_register(GIE);
			
			// take the 8962 out of reset
			resetSet(FALSE);
			#ifdef RONE_V12
				ftdiResetSet(FALSE);
				ledResetSet(FALSE);
				motSleepSet(FALSE);
			#endif
			
			airplaneStartTicks = 0;

			// wait for I2C to finish
			for(i = 0; (UCB0STAT & UCBBUSY) && (i < I2C_MAX_WHILE_DELAY) ; i++) {}

			ADC10Init();
			powerVBatInit();

			// init the on-chip I/O
			accelInit();
			gyroInit();
			ledInit();
//			magInit();
			RFIDInterruptEnable();
			bumpSensorInit();
			reflectiveSensorsInit();


			/*#ifdef RONE_V12
				Usb5vSenseInit();
			#endif*/
			//bumpDebugInit();
			#ifdef RONE_V11
				irBeaconEnable();
			#endif
			
			//Pet the watchdog and start the program
			watchdogPet();
		}

		//Only update and check the SPI messages and LEDs if 
		//the MSP is not overiding the LEDs
		if (!powerButtonLEDOveride) {
			//TODO: RFID debug
#ifndef RFID_DEBUG
			msp430CheckAndUpdate();
#endif
			if (systemLEDRampBrightness != systemLEDRampBrightnessOld) {
				if (systemLEDRampBrightness >= 0) {
					blinkyLEDSet(systemLEDRampBrightness);
				} else {
					blinkyLEDSet(0);
				}
				systemLEDRampBrightnessOld = systemLEDRampBrightness;
			}
		}

		// Update the VBAT value and the USB 5V Line ADC (V12 Only)
		if (timerADCUpdate == 0) {
			// Disable almost all of the interrupts so that these are atomic
			__bic_SR_register(GIE);
			watchdogDisable();

			// Update the values
			powerVBatReadADC();
			/*#ifdef RONE_V12
				Usb5vReadADC();
			#endif*/
			timerADCUpdate = TIMER_ADC_PERIOD;

			// Re-enable inturupts
			__bis_SR_register(GIE);
			watchdogInit();
		}
		/*
		#ifdef RONE_V12
			if(powerEnGet()){
				if(Usb5vGet() > USB_5V_1000mAMP_THRESHHOLD){
					chargeLimitSet(FALSE); // put LOW
				} else {
					chargeLimitSet(TRUE);  // put HIGH
				}
			}
		#endif*/
		
		//TODO RFID debug
//#ifndef RFID_DEBUG
		// check for a power off request
		currentTicks = systemTicks();
 		if (powerButtonGetValue()) {
			if (powerButtonFSMState == 1) { //Reset Mode
				powerButtonLEDOveride = FALSE;
				// power button is depressed and the state is still in reset
				if (timerDiff(powerButtonStartTicks, currentTicks) > POWER_BUTTON_HOLD_DELAY_OFF) {
					// it was held long enough to turn off - display the battery and time-to-airplane
					powerButtonFSMState = 2;
					powerButtonLEDOveride = TRUE;
					batteryLEDFadeIn = 0;
					//Turn off the system LED so that it is not left stuck on
					blinkyLEDSet(0);
				}
			}
			else if (powerButtonFSMState == 2) { //Off mode
				powerButtonLEDOveride = TRUE;
				//Display the time until airplane on lights in addition to
				//the battery meter in this mode
				setAllLEDData(powerButtonLEDOverideData, 0);
				setBatteryLEDData(powerButtonLEDOverideData);
				setTimeToAirplaneLEDData(powerButtonLEDOverideData, POWER_BUTTON_HOLD_DELAY_AIRPLANE - timerDiff(powerButtonStartTicks, currentTicks));

				ledUpdate(powerButtonLEDOverideData);
				ledTimeoutReset();

				if (timerDiff(powerButtonStartTicks, currentTicks) > POWER_BUTTON_HOLD_DELAY_AIRPLANE) {
					// It was held long enough to go into airplane mode - turn the robot off
					// and put the rone in airplane mode
					powerButtonFSMState = 3;
					powerOffRequest = TRUE;
					airplaneMode = TRUE;
					powerButtonFSMState = 0; //idle again
					powerButtonLEDOveride = TRUE;
				}
			}
			else if (powerButtonFSMState == 3) {
				//This shoudln't happen, because the robot should be off
				powerButtonLEDOveride = TRUE;
				//turn off all the lights in preparation for airplane mode
				ledSetAll(0);
			} else {
				// power button is first pressed.  record the ticks
				powerButtonStartTicks = currentTicks;
				powerButtonFSMState = 1;
				powerButtonLEDOveride = FALSE;
			}
		} else {
			//Release the overide on the LEDs
			powerButtonLEDOveride = FALSE;

			if (powerButtonFSMState == 1) {
				//debounce
				if (timerDiff(powerButtonStartTicks, currentTicks) > POWER_BUTTON_HOLD_DELAY_RESET) {
					// the power button is released after a short delay - reset the rone
					resetSet(TRUE);
					#ifdef RONE_V12
						if (resetFTDIWithButton) {
							ftdiResetSet(TRUE);
						}
						ledResetSet(TRUE);
						motSleepSet(TRUE);
					#endif
					//msp430SetRobotMode(MSP430_MODE_PYTHON);
					for (i = 0; i < RESET_DELAY; i++) {}
				    resetSet(FALSE);
				    #ifdef RONE_V12
						if (resetFTDIWithButton) {
							ftdiResetSet(FALSE);
						}
						ledResetSet(FALSE);
						motSleepSet(FALSE);
					#endif
					//for (i = 0; i < RESET_DELAY; i++) {}
					ledInit();
					powerButtonFSMState = 0; //idle again
				}
			}
			else if (powerButtonFSMState == 2){
				// The power button was released while in the off state - turn the robot off
				// Do not put the rone in airplane mode
				powerOffRequest = TRUE;
				airplaneMode = FALSE;
				powerButtonFSMState = 0; //idle again
			}
		}

		//Turn off the robot as soon as possible if the VBat drops below a threshold
		if (powerVBatGet() < VBAT_SHUTDOWN_THRESHOLD) {
			if(!powerUSBGetState()) {
				resetSet(TRUE);
				#ifdef RONE_V12
					ftdiResetSet(TRUE);
					motSleepSet(TRUE);
				#endif

				// Turn the robot off officially this time
				powerOffRequest = TRUE;
				airplaneMode = FALSE;

				// Go into airplane mode if the battery is beyond dead
				if (powerVBatGet() < (VBAT_SHUTDOWN_THRESHOLD-3)) {
					airplaneMode = TRUE;
				}

				// Disable some interupts
				watchdogPet();
				watchdogDisable();

				// Make sure the LED's don't turn off
				ledTimeoutReset();

				// Turn off the bliky led so it isn't stuck on
				blinkyLEDSet(0);

				// Blink the red light 3 times so that the user knows there is no battery
				uint8 blinkCount;
				setAllLEDData(powerButtonLEDOverideData, 0);
				for (blinkCount = 0; blinkCount < VBAT_SHUTDOWN_BLINK_LED_TIMES; blinkCount++){
					powerButtonLEDOverideData[VBAT_SHUTDOWN_BLINK_LED] = VBAT_SHUTDOWN_BLINK_LED_PWM;
					ledUpdate(powerButtonLEDOverideData);

					for (i=0; i<VBAT_SHUTDOWN_BLINK_DELAY; i++) {}

					powerButtonLEDOverideData[VBAT_SHUTDOWN_BLINK_LED] = 0;
					ledUpdate(powerButtonLEDOverideData);

					for (i=0; i<VBAT_SHUTDOWN_BLINK_DELAY; i++) {}
				}
				watchdogEnable();
			}
		}
//#endif
	} // while
}


// Timer0_A0 interrupt service routine (ISR) runs at 600 Hz
extern uint32 SPIMessageTicks;

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
{
	//Disable the inturpts so that all the counters update
	__bic_SR_register(GIE);
	
	systemTicksVal++;
	timer60hz++;
	timer10hz++;
	
	//enable nested interrupts for SPI transfers
	__bis_SR_register(GIE);
	
//	if (SPIMessageTicks > 0) {
//		// you have received a SPI message recently.  reset the time-outs 
//		SPIMessageTicks--;
//		systemLEDRampTimer = 0;
//		systemLEDRampBrightness = -1;
//	}
//	if (SPIMessageTicks == 0) {
//		// no SPI message in a while.  Wait a bit, then start the system LED ramp
//		systemLEDRampTimer++;
//		if (systemLEDRampTimer > SYSTEM_LED_DIMMER_DELAY) {
//			systemLEDRampBrightness = (systemLEDRampTimer - SYSTEM_LED_DIMMER_DELAY) / 16; 
//			if (systemLEDRampBrightness == SYSTEM_LED_BRIGHTNESS) {
//				systemLEDRampTimer = (SYSTEM_LED_DIMMER_DELAY + 1);
//			}
//		} else {
//			systemLEDRampBrightness = -1;
//		}
//	}

	if (SPIMessageTicks > 0) {
		// you have received a SPI message recently.  reset the time-outs 
		SPIMessageTicks--;
		systemLEDRampTimer = 0;
	}
	if (SPIMessageTicks == 0) {
		// no SPI message in a while.  start the system LED ramp
		systemLEDRampTimer++;
		systemLEDRampBrightness = systemLEDRampTimer / 32; 
		if (systemLEDRampBrightness >= SYSTEM_LED_BRIGHTNESS) {
			systemLEDRampTimer = 0;
		}
	}
	
	if(timer60hz >= 10) {
		//run the 60hz code
		timer60hz = 0;
		
		#ifdef RONE_V11			
			if(irBeaconGetEnable()) {
				irBeaconUpdate();
			}
		#endif
		
		if(timerADCUpdate > 0) {
			timerADCUpdate--;
		}
	}
	
	if(timer10hz >= 60) {
		//Run the 10Hz Code
		timer10hz = 0;
		
		//Update the LEDs so that they will timeout correctly
		ledTimeoutUpdate();
	}
// Used for testing frequency of interrupt function being called
//	pulseFlag = 1 - pulseFlag;
//	blinkySet(pulseFlag);
}

//SPI and I2C RX interrupt vector handler
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
	// SPI CPU board ISR
	if(IFG2 & UCA0RXIFG) {
		SPI8962RX_ISR();
	}
	
	// I2C ISR
	if (UCB0STAT & UCNACKIFG){            // send STOP if slave sends NACK
		UCB0CTL1 |= UCTXSTP;
		UCB0STAT &= ~UCNACKIFG;
	}
}

/* Handle the Port2 Interrupts */
//#pragma vector = PORT2_VECTOR
//__interrupt void port2Interupt(void) {
////#ifdef RONE_V12_TILETRACK
////	if (RFID_PORT_IFLG & RFID_INTERUPT_BITS) {
////		RFIDReaderInterupt();
////	}
////#endif
//}
