#ifndef POWERCONTROLLER_H_
#define POWERCONTROLLER_H_

void set8962Reset(uint8 state);

void powerEnSet(uint8 val);
uint8 powerEnGet(void);
void powerEnInit(void);

boolean powerButtonGetValue(void);
void powerButtonInit(void);
void powerButtonDisable(void);
void powerButtonIRQEnable(void);
void powerButtonIRQDisable(void);


boolean powerUSBGetState(void);
void powerUSBInit(void);
void powerUSBSetEnable(boolean on);
void powerUSBSetMode(uint8 mode);
void ADC10Init(void);

void resetSet(boolean val);
void resetInit(void);

#ifdef RONE_V12
	void ftdiResetInit(void);
	void ftdiResetSet(boolean val);
	void motSleepSet(boolean val);
	void motSleepInit(void);
	void chargeLimitSet(boolean val);
	void chargeLimitInit(void);
	void Usb5vReadADC(void);
	void Usb5vSenseInit(void);
	uint8 Usb5vGet(void);
#endif

void powerVBatInit(void);
void ADC10Shutdown(void);
uint8 powerVBatGet(void);
void powerVBatReadADC(void);

#endif /*POWERCONTROLLER_H_*/
