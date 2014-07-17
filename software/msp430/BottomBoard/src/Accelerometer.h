#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#define ACCEL_DATA_LENGTH	6

void accelInit(void);
void accelUpdate(void);
uint16 accelGetData(uint8 addr);

#endif /*ACCELEROMETER_H_*/
