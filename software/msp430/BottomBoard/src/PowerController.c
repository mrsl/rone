#include <stdio.h>
#include <msp430f2132.h>

#include "typedefs.h"
#include "SPI_8962.h"
#include "Gyroscope.h"
#include "Accelerometer.h"
#include "BumpSensors.h"
#include "IRBeacon.h"
#include "I2C.h"
#include "leds.h"
#include "System.h"
#include "PowerController.h"

#define POWER_BUTTON_PORT_SEL		P1SEL
#define POWER_BUTTON_PORT_IN		P1IN
#define POWER_BUTTON_PORT_OUT		P1OUT
#define POWER_BUTTON_PORT_DIR		P1DIR
#define POWER_BUTTON_BIT			BIT2

/* 5VUSB_SENSE Pin Inputs */
#ifdef RONE_V11
	#define POWER_USB_PORT_IN			P1IN
	#define POWER_USB_PORT_OUT			P1OUT
	#define POWER_USB_PORT_DIR			P1DIR
	#define POWER_USB_PORT_SEL			P1SEL
	#define POWER_USB_BIT				BIT3
	
	#define POWER_EN_PORT_SEL			P1SEL
	#define POWER_EN_PORT_IN			P1IN
	#define POWER_EN_PORT_OUT			P1OUT
	#define POWER_EN_PORT_DIR			P1DIR
	#define POWER_EN_BIT				BIT1
#endif
#ifdef RONE_V12
	#define POWER_USB_PORT_IN			P2IN
	#define POWER_USB_PORT_OUT			P2OUT
	#define POWER_USB_PORT_DIR			P2DIR
	#define POWER_USB_PORT_SEL			P2SEL
	#define POWER_USB_BIT				BIT3
	
	#define POWER_EN_PORT_SEL			P1SEL
	#define POWER_EN_PORT_IN			P1IN
	#define POWER_EN_PORT_OUT			P1OUT
	#define POWER_EN_PORT_DIR			P1DIR
	#define POWER_EN_BIT				BIT0
	
	#define FTDI_RESET_PORT_SEL			P1SEL
	#define FTDI_RESET_PORT_IN			P1IN
	#define FTDI_RESET_PORT_OUT			P1OUT
	#define FTDI_RESET_PORT_DIR			P1DIR
	#define FTDI_RESET_BIT				BIT3
	
#ifdef RONE_V12_TILETRACK
	#define MOT_SLEEP_PORT_SEL			P1SEL
	#define MOT_SLEEP_PORT_IN			P1IN
	#define MOT_SLEEP_PORT_OUT			P1OUT
	#define MOT_SLEEP_PORT_DIR			P1DIR
	#define MOT_SLEEP_BIT				BIT5
#else
	#define MOT_SLEEP_PORT_SEL			P3SEL
	#define MOT_SLEEP_PORT_IN			P3IN
	#define MOT_SLEEP_PORT_OUT			P3OUT
	#define MOT_SLEEP_PORT_DIR			P3DIR
	#define MOT_SLEEP_BIT				BIT6
#endif /* #ifdef RONE_V12_TILETRACK */

	#define CHRG_LIM_PORT_IN			P2IN
	#define CHRG_LIM_PORT_OUT			P2OUT
	#define CHRG_LIM_PORT_DIR			P2DIR
	#define CHRG_LIM_PORT_SEL			P2SEL
	#define CHRG_LIM_BIT				BIT5
	
	#define USB_5V_RUN_AVG_LEN          3
	uint8 Usb5vRunAvg[USB_5V_RUN_AVG_LEN];
	uint8 Usb5vRunAvgCount = 0;       
#endif

#ifndef RONE_V12_TILETRACK
#define RESET_PORT_SEL		P2SEL
#define RESET_PORT_IN		P2IN
#define RESET_PORT_OUT		P2OUT
#define RESET_PORT_DIR		P2DIR
#define RESET_BIT			BIT4
#else
#define RESET_PORT_SEL		P1SEL
#define RESET_PORT_IN		P1IN
#define RESET_PORT_OUT		P1OUT
#define RESET_PORT_DIR		P1DIR
#define RESET_BIT			BIT4
#endif // #ifndef RONE_V12_TILETRACK

boolean powerOn = FALSE;
uint16 startCount;
uint16 endCount;
#define VBAT_RUN_AVG_LEN	3
//Start off assuming that the battery is fully charged
uint8 vBatRunAvg[VBAT_RUN_AVG_LEN];
uint8 vBatRunAvgCount = 0;


void powerUSBInit(void) {
#ifdef RONE_V11
	//Input with I/O
	POWER_USB_PORT_IN &= ~POWER_USB_BIT;
	POWER_USB_PORT_DIR &= ~POWER_USB_BIT; 
	POWER_USB_PORT_SEL &= ~POWER_USB_BIT;
#endif
#ifdef RONE_V12
	// Set up the comparator to read true if we receive more than about 2.5V on
	// USB 5V. This allows airplaine mode with a 1/3 Voltage Divider
	CACTL1 = CARSEL + CAREF0;  // 0.25*Vcc reference applied to '-' terminal
							   // No interrupts.
	CACTL2 = P2CA0 + CAF;      // Input CA0 on '+' terminal, filter output.

	powerUSBSetEnable(FALSE);
#endif
}

void powerUSBSetEnable(boolean on) {
#ifdef RONE_V11
	// Nothing to do for V11
#endif
#ifdef RONE_V12
	// Turn off/on the comparator
	if(on) {
		CAPD &= ~POWER_USB_BIT;	// Disable digital I/O on P2.3 (technically this step is redundant)
		CACTL1 |= CAON;		  	// Turn on the comparator
	} else {
		CAPD |= POWER_USB_BIT; // Enable digital I/O on P2.3
		CACTL1 &= ~CAON;		// Disable the comparator
	}
#endif
}


// Set the mode of the USB Power Sense to be either Digital, Analog Comparator, or ADC based
// This may take over the USBSetEnable
void powerUSBSetMode(uint8 mode) {
	// TODO: Do this, and add the ADC for fast charge mode
}

boolean powerUSBGetState(void){
#ifdef RONE_V11
	if(((POWER_USB_PORT_IN & POWER_USB_BIT)==POWER_USB_BIT)){
		return TRUE;
	} 
	else {
		return FALSE;
	}
#endif
#ifdef RONE_V12
	// CAOUT is the value of the filtered Comparator
	if((CACTL2 & CAOUT) == CAOUT) {
		return TRUE;
	} else {
		return FALSE;
	}
#endif
}


uint8 powerVBatGet(void){
	//Returns 10x the battery voltage as an integer
	//Takes a running average of the last three reads to filter out noise
	uint8 i;
	uint16 averageVBat = 0;
	for(i=0; i<VBAT_RUN_AVG_LEN; i++){
		averageVBat += (uint16)(vBatRunAvg[i]);
	}
	return (uint8)(averageVBat/VBAT_RUN_AVG_LEN);
}

void ADC10Init(void){
	ADC10CTL0 = SREF_0 | ADC10SHT_2 | ADC10ON; 					// Default ref, 16 x clk, ADC10 on
}

void powerVBatInit(void) {
	uint8 i;
	//Start off assuming that the battery is fully charged
	for(i = 0; i < VBAT_RUN_AVG_LEN; i++){
		vBatRunAvg[i] = 42;
	}
	//Read the battery and use that as all of the running averages
	//Esensially, this should give a starting place unless the read fails
	vBatRunAvgCount = 0;
	powerVBatReadADC();
	for(i = 0; i < VBAT_RUN_AVG_LEN; i++){
		vBatRunAvg[i] = vBatRunAvg[0];
	}
	//Reset the read count to 0
	vBatRunAvgCount = 0;
}


void ADC10Shutdown(void){
	ADC10CTL0 = 0; // ~ADC10ON
}


#define ADC_MAX_DELAY_TIME		500
#define VBAT_CONV_NUMER			(2.5 * 10)
#define VBAT_CONV_DENOM			(1024 * 0.467)
//#define VBAT_CONV_DENOM			(1024 * 0.425)

void powerVBatReadADC(void){
	volatile uint16 i;
	
	// Reconfigure ADC10 for power sense
	ADC10AE0 |= BIT7; 													// A7
	ADC10CTL1 = INCH_7 | ADC10DIV_7 | CONSEQ_0;							// input channel 7, single sequence
	ADC10CTL0 = ADC10SHT_2 | ADC10ON | SREF_1 | REFON | REF2_5V ; 		// 16 x clk, ADC10 on, internal ref, ref on, 2.5 volt internal ref
   	ADC10CTL0 |= ENC + ADC10SC;             							// Sampling and conversion start
	for (i = 0 ; i < ADC_MAX_DELAY_TIME ; i++) {
		if (!(ADC10CTL1 & ADC10BUSY)) {
			// Add the current read to the sliding average if there was a sucsessful read
			vBatRunAvg[vBatRunAvgCount] = (uint8)(ADC10MEM * VBAT_CONV_NUMER / VBAT_CONV_DENOM);
			vBatRunAvgCount = (vBatRunAvgCount + 1) % VBAT_RUN_AVG_LEN;
			break;
		} 
	}

	// Change back to original ADC10 settings
	ADC10Init();
}


void powerEnSet(uint8 val) {
	if (val) {
		POWER_EN_PORT_OUT |= POWER_EN_BIT;
	} else {
		POWER_EN_PORT_OUT &= ~POWER_EN_BIT;
	}
	powerOn = val;	
}


uint8 powerEnGet(void) {
	return powerOn;
}


void powerEnInit(void) {
	powerEnSet(FALSE);
	POWER_EN_PORT_SEL &= ~POWER_EN_BIT; 
	POWER_EN_PORT_DIR |= POWER_EN_BIT;
}


void resetSet(boolean val) {
	if(val) {
		RESET_PORT_DIR |= RESET_BIT;
		RESET_PORT_OUT &= ~RESET_BIT;
	} else {
		RESET_PORT_DIR &= ~RESET_BIT;
	}
}


void resetInit(void) {
	RESET_PORT_SEL &= ~RESET_BIT; 
	resetSet(TRUE);
}

#ifdef RONE_V12
	void ftdiResetSet(boolean val) {
		if(val) {
			FTDI_RESET_PORT_DIR |= FTDI_RESET_BIT; //Force the output to a 0
			FTDI_RESET_PORT_OUT &= ~FTDI_RESET_BIT;
		} else {
			FTDI_RESET_PORT_DIR &= ~FTDI_RESET_BIT; //Let the output float to a 1
		}
	}
	
	
	void ftdiResetInit(void) {
		FTDI_RESET_PORT_SEL &= ~FTDI_RESET_BIT; 
		ftdiResetSet(TRUE);
	}
	
	void motSleepSet(boolean val){ // send TRUE to sleep and FALSE to wake
		if(val) {
			MOT_SLEEP_PORT_DIR &= ~MOT_SLEEP_BIT; //Let the output float to a 0
		} else {
			MOT_SLEEP_PORT_DIR |= MOT_SLEEP_BIT; //Force the output to a 1
			MOT_SLEEP_PORT_OUT |= MOT_SLEEP_BIT;
		}
	}
	
	void motSleepInit(void) {
		MOT_SLEEP_PORT_SEL &= ~MOT_SLEEP_BIT;
		motSleepSet(TRUE);
	}
	
	void chargeLimitSet(boolean val){ // False is low and True is high
		if(val){
			CHRG_LIM_PORT_DIR |= CHRG_LIM_BIT;
			CHRG_LIM_PORT_OUT |= CHRG_LIM_BIT;
		}else{
			CHRG_LIM_PORT_DIR &= ~CHRG_LIM_BIT;
		}
	}
	
	void chargeLimitInit(void){
		CHRG_LIM_PORT_SEL &= ~CHRG_LIM_BIT;
		chargeLimitSet(FALSE);
	}
	
	/*#define USB_5V_CONV_NUMER			(2.5 * 10)
	#define USB_5V_CONV_DENOM			(1024 * 0.333)
	
	void Usb5vReadADC(void){
		volatile uint16 i;
		
		ADC10AE0 |= BIT3; // A3
		ADC10CTL1 = INCH_3 | ADC10DIV_7 | CONSEQ_0;			// input channel 3, single sequence 
	   	ADC10CTL0 |= ENC + ADC10SC;             			// Sampling and conversion start
		for (i = 0 ; i < ADC_MAX_DELAY_TIME ; i++){
			if(!(ADC10CTL1 & ADC10BUSY)){
				//Add the current read to the sliding average if there was a sucsessful read
				Usb5vRunAvg[Usb5vRunAvgCount] = (uint8)(ADC10MEM * USB_5V_CONV_NUMER / USB_5V_CONV_DENOM);
				Usb5vRunAvgCount = (Usb5vRunAvgCount+1)%USB_5V_RUN_AVG_LEN;
				break;
			}
		}
	}
	
	void Usb5vSenseInit(void){
		uint8 i;
		
		//Start off assuming that the battery is fully charged
		for(i = 0; i < USB_5V_RUN_AVG_LEN; i++){
			Usb5vRunAvg[i] = 0;
		}
		//Read the battery and use that as all of the running averages
		//Esensially, this should give a starting place unless the read fails
		Usb5vRunAvgCount = 0;
		Usb5vReadADC();
		for(i = 0; i < USB_5V_RUN_AVG_LEN; i++){
			Usb5vRunAvg[i] = Usb5vRunAvg[0];
		}
		//Reset the read count to 0
		Usb5vRunAvgCount = 0;
	}
	
	uint8 Usb5vGet(void){
		//Returns 10x the battery voltage as an integer
		//Takes a running average of the last three reads to filter out noise
		uint8 i;
		uint16 average5vUsb = 0;
		for(i=0; i<USB_5V_RUN_AVG_LEN; i++){
			average5vUsb += (uint16)(Usb5vRunAvg[i]);
		}
		return (uint8)(average5vUsb/USB_5V_RUN_AVG_LEN);
	}*/
	
#endif


boolean powerButtonGetValue(void) {
	if (POWER_BUTTON_PORT_IN & POWER_BUTTON_BIT) {
		return 0;
	} else {
		return 1;
	}
}


void powerButtonInit(void) {
	POWER_BUTTON_PORT_DIR &= ~POWER_BUTTON_BIT;
	POWER_BUTTON_PORT_SEL &= ~POWER_BUTTON_BIT;
}


void powerButtonIRQEnable(void) {
	P1IE |= POWER_BUTTON_BIT;                             // P1.2 interrupt enabled
  	P1IES |= POWER_BUTTON_BIT;                            // P1.2 Hi-to-lo edge
  	P1IFG &= ~POWER_BUTTON_BIT;                           // P1.2 IFG cleared
}


void powerButtonIRQDisable(void) {
	P1IE &= ~POWER_BUTTON_BIT;                             // P1.2 interrupt disabled
}

//void powerButtonDisable(void) {
//	powerButtonState = FALSE;
//	P1IE &= ~POWER_BUTTON_BIT;
//	P1IES &= ~POWER_BUTTON_BIT;	
//}
//
//void powerButtonEnable(void) {
//	powerButtonState = TRUE;	
//}


#define BUTTON_DEBOUNCE_COUNT   15000


// Port 1 interrupt service routine
// study msp430x21x2_adc10_01.c for help with the low-power modes
// LPM4: disables CPU, ACLK, MCLK, SMCLK, DCO, and crystal oscillator
//			(all clocks are disabled)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
	uint16 i;
	boolean powerOn = FALSE;
	
	if (!powerOn) {
		// Turn on the USB Sense Comparator if we are in airplane mode
		if (airplaneModeGetState()) {
			powerUSBSetEnable(TRUE); // Turn on the USB Power Sense Line
		}
		for (i = 0; i < BUTTON_DEBOUNCE_COUNT; ++i) {}
		// Is the button still down?
		if (powerButtonGetValue()) {
			if (airplaneModeGetState()) {
				if (powerUSBGetState()) {
					powerOn = TRUE;
				}
			} else {
				powerOn = TRUE;
			}
		}
	}
	if (powerOn) {
		// Turn back off of the comparator
		if (airplaneModeGetState()) {
			powerUSBSetEnable(FALSE);
		}

		// Exit LPM4, enable normal CPU operation, enable interrupts
		// aka turn the robot back on
		__bic_SR_register_on_exit(SCG1 + SCG0 + OSCOFF + CPUOFF);
	}
	P1IFG &= ~POWER_BUTTON_BIT;                           // P1.2 IFG cleared
}
