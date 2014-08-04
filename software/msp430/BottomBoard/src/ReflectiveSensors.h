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
#define REFLECTIVEA_PORT_REN	P2REN
#define REFLECTIVEB_PORT_REN	P3REN
#define REFLECTIVEA_ADC_PORTS	(BIT0 + BIT1 + BIT2 + BIT4)
#define REFLECTIVEB_ADC_PORTS	(BIT6)

#define REFLECTIVE_ADC_CHANNEL_0	INCH_0
#define REFLECTIVE_ADC_CHANNEL_1	INCH_1
#define REFLECTIVE_ADC_CHANNEL_2	INCH_2
#define REFLECTIVE_ADC_CHANNEL_3	INCH_4
#define REFLECTIVE_ADC_CHANNEL_4	INCH_6


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
