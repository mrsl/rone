#ifndef POWERCONTROLLER_H_
#define POWERCONTROLLER_H_

// Experimentally determined number for V15
// USB5VSense USB: 		890
//	 		  Charger:	950
//			  Both:		926
#define POWER_USB_PLUGGED_IN_THRESHOLD		700
#define POWER_USB_FAST_CHARGE_THRESHOLD		950


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
uint16 powerUSBGetAvg(void);
void powerUSBReadADC(void);

void powerVBatInit(void);
uint8 powerVBatGet(void);
void powerVBatReadADC(void);

#endif /*POWERCONTROLLER_H_*/
