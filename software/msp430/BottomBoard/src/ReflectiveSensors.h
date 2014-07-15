#ifndef REFLECTIVE_H_
#define REFLECTIVE_H_
#ifdef RONE_V12_TILETRACK

#define REFLECTIVE_SENSORS_ENABLE_DELAY 500000

#define REFLECTIVE_POWER_EN_DIR P1DIR
#define REFLECTIVE_POWER_EN_SEL P1DIR
#define REFLECTIVE_POWER_EN_OUT P1OUT
#define REFLECTIVE_POWER_EN_BIT BIT6

#define NUM_REFLECTIVE_PORTS 	5

#define REFLECTIVEA_PORT_DIR 	P2DIR
#define REFLECTIVEB_PORT_DIR 	P3DIR
#define REFLECTIVEA_PORT_SEL	P2SEL
#define REFLECTIVEB_PORT_SEL	P3SEL
#define REFLECTIVE_ADC_PORTS	(BIT0 + BIT1 + BIT2 + BIT4 + BIT6)

void reflectiveSensorsInit();
void reflectiveSensorPowerEnable();
void reflectiveSensorPowerDisable();
void reflectiveSensorsUpdate();
uint8 reflectiveGetData(int i);

#else

#define reflectiveSensorsInit() {}
#define reflectiveSensorPowerEnable() {}
#define reflectiveSensorPowerDisable() {}
#define reflectiveSensorsUpdate() {}
#define reflectiveGetData(i) 0

#endif

#endif /* REFLECTIVE_H_ */
