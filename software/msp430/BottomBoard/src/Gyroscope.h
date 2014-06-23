#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#define GYRO_DATA_LENGTH		6

#ifndef RONE_V12_TILETRACK

void gyroInit(void);
void gyroUpdate(void);
uint16 gyroGetData(uint8 addr);

#else

#define gyroInit() 			{}
#define gyroUpdate()		{}
#define gyroGetData(val)	0

#endif

#endif /*GYROSCOPE_H_*/
