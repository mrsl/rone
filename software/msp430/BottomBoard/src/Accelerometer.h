#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#define ACCEL_DATA_LENGTH	6

#ifndef RONE_V12_TILETRACK

void accelInit(void);
void accelUpdate(void);
uint16 accelGetData(uint8 addr);

#else

#define accelInit() 		{}
#define accelUpdate()		{}
#define accelGetData(val)	0

#endif

#endif /*ACCELEROMETER_H_*/
