#ifndef POWERCONTROLLER_H_
#define POWERCONTROLLER_H_

// constants to convert raw ADC values of battery and USB to voltage
#define VOLTAGE_BAT_CONV_NUMER			(2.5 * 10)
/* The 0.467 Constant comes from the 1/2 voltage divider + sampling droop */
#define VOLTAGE_BAT_CONV_DENOM			(1024 * 0.467)
#define VOLTAGE_BAT_FULLY_CHARGED		41


// Experimentally determined number for V15
// USB5VSense USB: 		890
//	 		  Charger:	950
//			  Both:		926
// these values are raw ADC values - before before conversion to voltage
#define VOLTAGE_USB_PLUGGED_IN_THRESHOLD		700
#define VOLTAGE_USB_FAST_CHARGE_THRESHOLD		950

#define VOLTAGE_USB_CONV_NUMER			(2.5 * 10)
/* The 0.467 Constant comes from the 1/2 voltage divider + sampling droop */
#define VOLTAGE_USB_CONV_DENOM			(1024 * 0.467)
#define VOLTAGE_USB_CONV_OFFSET			30


void set8962Reset(uint8 state);

void powerEnSet(uint8 val);
uint8 powerEnGet(void);
void powerEnInit(void);

boolean powerButtonGetValue(void);
void powerButtonInit(void);
void powerButtonDisable(void);
void powerButtonIRQEnable(void);
void powerButtonIRQDisable(void);

void resetSet(boolean val);
void resetInit(void);

#ifdef RONE_V12
	void ftdiResetInit(void);
	void ftdiResetSet(boolean val);
	void motSleepSet(boolean val);
	void motSleepInit(void);
	void chargeLimitSet(boolean val);
	void chargeLimitInit(void);
#endif

void ADC10Init(void);
void ADC10Shutdown(void);

void powerUSBInit(void);
uint16 voltageUSBGet(void);
void voltageUSBReadADC(void);

void powerVBatInit(void);
uint8 voltageBatGet(void);
void voltageBatReadADC(void);

#endif /*POWERCONTROLLER_H_*/
