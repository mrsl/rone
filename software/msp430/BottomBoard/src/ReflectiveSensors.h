#ifndef REFLECTIVE_H_
#define REFLECTIVE_H_
#ifdef RONE_V12_TILETRACK

#define REFLECTIVE_POWER_EN_DIR P1DIR
#define REFLECTIVE_POWER_EN_SEL P1DIR
#define REFLECTIVE_POWER_EN_OUT P1OUT
#define REFLECTIVE_POWER_EN_BIT BIT6

#define NUM_REFLECTIVE_PORTS 	5

#define REFLECTIVEA_PORT_DIR 	P2DIR
#define REFLECTIVEB_PORT_DIR 	P3DIR
#define REFLECTIVEA_PORT_SEL	P2SEL
#define REFLECTIVEB_PORT_SEL	P3SEL
#define REFLECTIVEA_BITS 		(BIT0 + BIT1 + BIT2 + BIT4)
#define REFLECTIVEB_BITS 		(BIT6)

void reflectiveSensorsInit();
void reflectiveSensorPowerEnable();
void reflectiveSensorPowerDisble();
void reflectiveSensorsUpdate();

#else

void reflectiveSensorsInit() {}
void reflectiveSensorPowerEnable() {}
void reflectiveSensorPowerDisble() {}
void reflectiveSensorsUpdate() {}
#endif

#endif /* REFLECTIVE_H_ */
