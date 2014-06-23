#ifndef SYSTEM_H_
#define SYSTEM_H_

void setPowerOffRequest(boolean powerOff);
void setPowerResetRequest(boolean powerOffAndOn);
uint32 systemTicks(void);
void timerInit(void);
void timerDisable(void);
void setSystemLEDRampBrightness(int16);
#ifdef RONE_V12
	void resetFTDIWithButtonSet(boolean r);
#endif
boolean airplaneModeGetState(void);
uint32 systemTicks(void);
uint32 timerDiff(uint32 oldval, uint32 newval);

#endif /*SYSTEM_H_*/
