#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#define MAG_DATA_LENGTH		6

#ifdef RONE_V12_TILETRACK

void magInit(void);
void magUpdate(void);
uint8 magGetDataLeft(uint8 addr);
uint8 magGetDataRight(uint8 addr);

#else

#define magInit() 				{}
#define magUpdate()				{}
#define magGetDataLeft(val)		0
#define magGetDataRight(val)	0

#endif

#endif /*MAGNETOMETER_H_*/
