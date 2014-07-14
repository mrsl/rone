#ifndef BUMP_H_
#define BUMP_H_

/*	DEFAULT NUMBERING SCHEME STARTS AT FRONT-LEFT, INCREASES CCW  */
#if defined(RONE_V9)


#define BUMPA_PORT_IN			P2IN
#define BUMPA_PORT_OUT			P2OUT
#define BUMPA_PORT_DIR			P2DIR
#define BUMPA_PORT_SEL			P2SEL
#define BUMPA_BITS				(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7)


#elif defined(RONE_V11) || defined(RONE_V12)

#define BUMPA_PORT_IN			P2IN
#define BUMPA_PORT_DIR			P2DIR
#define BUMPA_PORT_SEL			P2SEL
#define BUMPA_BITS				(BIT0 | BIT1 | BIT6 | BIT7)

#define BUMPB_PORT_IN			P1IN
#define BUMPB_PORT_DIR			P1DIR
#define BUMPB_PORT_SEL			P1SEL
#define BUMPB_BITS				(BIT4 | BIT5 | BIT6 | BIT7)

//P2.2 for bump sensors
#define BUMP_PWR_PORT_DIR		P2DIR
#define BUMP_PWR_PORT_SEL		P2SEL
#define BUMP_PWR_PORT_OUT		P2OUT		
#define BUMP_PWR_BIT			BIT2


#endif


#ifndef RONE_V12_TILETRACK

void bumpSensorInit();
void bumpSensorUpdate();
uint8 bumpSensorGet();
void bumpSensorHackFix();

void bumpDebugInit();
void bumpDebugSet(uint8 val);
void bumpDebugClear(uint8 val);

#else

#define bumpSensorInit()	{}
#define bumpSensorUpdate() 	{}
#define bumpSensorGet()		0

#endif //#ifndef RONE_V12_TILETRACK

#endif /*BUMP_H_*/
