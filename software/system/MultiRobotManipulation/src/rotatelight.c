
////////////////////////////////
/**
 * @file https://svn.rice.edu/r/mrone/robotcode/MultiRobotManipulation/src/rotatelight.c
 *
 * @brief: used in AAMAS paper to demonstrate multi-robot manipulation
 *
 * 1-  for light following ( no consensus) : Run exp0a.c
 * 2-  for leader following (consensus) : Run rotatelight.c
 *
 *     TODO: K_PROPORTIONAL has been adjusted. Now K_INTEGRAL has to be adjusted (as you pick a high value and bring it down to stabilize
 *     		the robot, settle on one, and decrease K_PROPORTIONAL slightly if necessary). 0.001 may be the right value (the errors get very large).
 *
 *			Don't bother testing them while stationary-- it doesn't look right even with exact SuperDemo code.
 *     To get a better K_INTEGRAL_MAX, print out the integral values and see what range they tend to fall in. Pick a max based on that.
 *
 *
 */

//Behavior: move forward; if robot bumps into something, enter into flock behavior



//#ifdef PART_LM3S9B92
//	#include "inc/lm3s9b92.h"
//#endif
//#ifdef PART_LM3S8962
//	#include "inc/lm3s8962.h"
//#endif
//
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "inc/hw_qei.h"
//
//#include "driverlib/gpio.h"
//#include "driverlib/pwm.h"
//#include "driverlib/qei.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"
//#include "driverlib/sysctl.h"

#include <stdio.h>
#include <stdlib.h>

#include "roneos.h"
#include "ronelib.h"

#define NEIGHBOR_ROUND_PERIOD		300

#define BEHAVIOR_TASK_PRIORITY		(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD		50

#define K							5
#define SPEED						60
#define TV_MAX						100
#define RV_MAX						4000

#define K_INTEGRAL_MAX				1000 //400 works, 300 doesn't
#define DECAY_INTEGRAL              70

#define OBJECT_MASS					778
#define K_PI_N						100
#define K_PI_D					    100

#define K_I				            30
#define K_P			            	5000
#define K_D				            75

#define PWM_CONTROLLER_PULSE_TIME	4

#define FR_OFFSET				    0
#define RR_OFFSET				    0

#define BEH_MODE_DO_NOTHING					0
#define BEH_MODE_SEARCH_THEN_PHOTOTAXIS		1
#define BEH_MODE_SEARCH_THEN_FLOCK			2
#define BEH_MODE_FLOCKING					3
#define BEH_MODE_PHOTOTAXIS					4
#define BEH_MODE_PWM_CONTROLLER				5
#define BEH_MODE_PWM_CONTROLLER_PULSE		6
#define BEH_MODE_PWM_CONTROLLER1			7
#define BEH_MODE_PULSE_CONTROLLER			8
#define BEH_MODE_RANDOM_ROTATE				9

#define PHOTOTAXIS_DEADZONE			100
#define PHOTOTAXIS_PWM				70

#define FLOCK_TV					20



/******** structs ********/

typedef struct motorVelocityData {
	uint8 motor;
	uint32 time;
	uint32 ticks;
	int32 vel;
	int32 velGoal;
	int32 velGoalRamp;
	int32 error;
	int32 iTerm;
} motorVelocityData;


static int rrtimer;
static int direction;
static int start;

static motorVelocityData motorData[2];


boolean isBumped = FALSE;
int32 sensor_offset = 0;
int32 integralError =  0;


typedef struct Err {
	int32 integralError;
	int32 error;
} Err;

void behPController(Beh* behPtr, int32 diff);
void behPIController(Beh* behPtr, int32 diff , int ErrorlightPtr[], boolean *IntegralWindup) ;
void behPIDController(Beh* behPtr, int32 diff , int ErrorlightPtr[], boolean *IntegralWindup);


enum manipState{ SEARCH_FOR_OBJECT, TRANSPORT_TO_LIGHT, AT_LIGHT};
enum manipState currentState;

void behaviorTask(void *parameters);
void backgroundTask(void *parameters);
int main(void) {
	volatile uint32 val1, val2;

	systemPreInit();
	systemInit();
	systemPrintStartup();
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	neighborsXmitEnable(TRUE);
	val1 = osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);

	if ((val1 != pdTRUE)) {
		cprintf("could not create a task");
	}

	/* Start the scheduler. */
	osTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
   	 task. This is a very bad thing, and should not happen.*/
	return 0;
}



//Beh* behLight(Beh* lightPtr, int32 light_value[],int ErrorlightPtr[],boolean *TaskDone, boolean *IntegralWindup) {
//	// this function compute control law  (tv, rv) to move robot toward light and sets TaskDone to 1 if the robot is at the light
//	boolean done = FALSE;
//
//	//assumption is fl is 0, fr is 1
//	// commented out  for adding controller
//	//if (abs(diff) < 10)
//	//sensor_offset = -diff;
//
//	//else
//	//sensor_offset = 0;
//
//	// end comment
//	int32 tv;
//	int32 rv;
//	//static int32 integralError = 0;
//
//	int32 diff;
//	if ((abs(lightSensorFR) < 1200) &&  (abs(lightSensorFL) < 1200 ) && (abs(lightSensorRL) < 1200 ) &&  (abs(lightSensorRR) < 1200 )) {
//		//ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_MED);
//		//uint32 deltaTicks = encoder_delta_ticks(currentTime, prevTime);
//
//		//if (max ( lightSensorRL , lightSensorRR ) > max(lightSensorFL,lightSensorFR)){
//		//if ((lightSensorRL> lightSensorFL) || (lightSensorRR,lightSensorFR)){
//		if ( min(lightSensorRL,lightSensorRR) > (100 * max (lightSensorFL,lightSensorFR) /100)){
//		//if (lightSensorRL>min(lightSensorRL,lightSensorRR) ){
//			// diff =  (max ( lightSensorRL , lightSensorRR ) -  max(lightSensorFL,lightSensorFR));
//			diff = MILLIRAD_PI /4;}
//		else{
//			diff = lightSensorFL - lightSensorFR;}
//
//		//behPIController(lightPtr, diff, ErrorlightPtr,IntegralWindup);
//		behPController(lightPtr, diff);
//
//		//behPIDController(lightPtr, diff,ErrorlightPtr, IntegralWindup);
//		// #### commented out  for adding controller
//		//lightPtr->tv = SPEED;
//		//lightPtr->rv = (sensor_offset + diff) * K;
//		// #### end comment
//
//		*TaskDone = 0;
//
//	}
//	else {
//		ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_MED);
//		*TaskDone =1;
//		lightPtr->tv = 0;
//		lightPtr->rv = 0;
//	}
//
//	return lightPtr;
//}



void behPController(Beh* behPtr, int32 diff) {
	// sets the rv and tv of the robot using a PI controller
	int tv, rv ;



	rv = bound(diff * K ,-RV_MAX, RV_MAX);
	tv = SPEED/2;



	behPtr->tv = tv;
	behPtr->rv = rv;


}





// LightError [2] : LightError [ 0] = error, LightError[1]= error integral
void behPIController(Beh* behPtr, int32 diff , int ErrorlightPtr[], boolean *IntegralWindup) {
	// sets the rv and tv of the robot using a PI controller
	int tv, rv , dE;
	dE = abs(diff) - ErrorlightPtr[0];
	ErrorlightPtr[0] = abs(diff) ;
	ErrorlightPtr[1] = ErrorlightPtr[1] + abs(diff);
	if (ErrorlightPtr[1] >= K_INTEGRAL_MAX){
		*IntegralWindup = 1;
		ErrorlightPtr[1] = (K_INTEGRAL_MAX * 99) /100 ;
	}

	else{
		if (*IntegralWindup == 1 && ErrorlightPtr[1] != 0 ){

			ErrorlightPtr[1] = (ErrorlightPtr[1] * DECAY_INTEGRAL)/100;
		}
		else{
			*IntegralWindup = 0 ;

		}

	}



	rv = bound(diff * K ,-RV_MAX, RV_MAX);


	//tv = bound(( SPEED  -   abs(K_PI_N *  SPEED * IntegralError/ ( K_INTEGRAL_MAX *K_PI_D ) ) )  ,0, TV_MAX);
	//tv = bound((  (1* K_INTEGRAL_MAX * SPEED)  -   (1 *SPEED * ErrorlightPtr[1])) /  ( 1 * K_INTEGRAL_MAX )  ,0, TV_MAX);
	tv = bound((  (K_INTEGRAL_MAX * K_INTEGRAL_MAX * SPEED)  -   (2 * ErrorlightPtr[1] *SPEED * ErrorlightPtr[1])) /  ( K_INTEGRAL_MAX  * K_INTEGRAL_MAX )  ,0, TV_MAX);

	//behPtr->tv = SPEED;



	behPtr->tv = tv;
	behPtr->rv = rv;


}








// PID Controller
void behPIDController(Beh* behPtr, int32 diff , int ErrorlightPtr[], boolean *IntegralWindup) {
	int tv, rv, tv_PD ,tv_PI ;
	ErrorlightPtr[1] = ErrorlightPtr[1] + abs(diff);
	int dE = abs(diff) - ErrorlightPtr[0];
	ErrorlightPtr[0] = abs(diff) ;
	if (ErrorlightPtr[1] >= K_INTEGRAL_MAX){
		*IntegralWindup = 1;
		ErrorlightPtr[1] = (K_INTEGRAL_MAX * 99) /100 ;
	}

	else{
		if (*IntegralWindup == 1 && ErrorlightPtr[1]!= 0 ){

			ErrorlightPtr[1] = ( ErrorlightPtr[1] * DECAY_INTEGRAL)/100;
		}
		else{
			*IntegralWindup = 0 ;

		}

	}


	rv = bound(diff * K ,-RV_MAX, RV_MAX);

	dE = bound(dE ,0 , 100);

	//tv = bound(( SPEED  -   abs(K_PI_N *  SPEED * IntegralError/ ( K_INTEGRAL_MAX *K_PI_D ) ) )  ,0, TV_MAX);
	tv_PD = (100 * SPEED - (dE * SPEED ) )* K_D / 100;
	tv_PI = (100 * SPEED  -  (  ErrorlightPtr[1] *ErrorlightPtr[1]   *SPEED )) * K_I  /(100 * K_INTEGRAL_MAX * K_INTEGRAL_MAX) ;
	tv = bound( tv_PD +  tv_PI,0, TV_MAX);

	//behPtr->tv = SPEED;



	behPtr->tv = tv;
	behPtr->rv = rv;
}

#define FLOCK_INTEGRAL_MAX		5000
#define FLOCK_INTEGRAL_DECAY	970
#define FLOCK_INTEGRAL_DECAY2	4


void behaviorTask(void* parameters) {
	//initialization
	uint32 lastWakeTime = osTaskGetTickCount();
	uint32 neighborRoundPrev = 0;
	uint32 neighborRound;
	uint8 neighbors;
	uint32 currentTime = osTaskGetTickCount();
	uint32 prevTime = 0;

	boolean reachedLightSource = FALSE;

	int32 integralError = 0;

	Beh behMove;

	NbrList nbrList;
	start=0;

	//begin by moving forward
	behMove.tv = SPEED;
	behMove.rv = 0;
	behMove.active = TRUE;
	motorSetBeh(&behMove);

	boolean targetSeen = 0;
	boolean lightActivated = 0;

	boolean TaskDone = 0;
	uint32 bumpSensorTimeOn = 0;
	uint32 bumpSensorTimeOff = 0;
	static boolean IntegralWindup = 0;

	// search variables
	uint32 segmentNum = 0;
	uint32 TimeStart = currentTime;
	uint32 TimeForThisSegment = 0;
	static int ErrorLightPtr[2];
	ErrorLightPtr[0] = 0;
	ErrorLightPtr[1] = 0;
	//int ErrorlightPtr =&LightError;
	int32 lightSensorFR, lightSensorFL, lightSensorRL, lightSensorRR;
	int32 lightSensorFROffset, lightSensorFLOffset, lightSensorRLOffset, lightSensorRROffset;
	//uint8 state_mike = BEH_MODE_RANDOM_ROTATE;
	//uint8 state_mike = BEH_MODE_PHOTOTAXIS;
	uint8 state_mike = BEH_MODE_SEARCH_THEN_FLOCK;


	int32 alpha, alphaMax;
	int32 flockAngleIntegral = 0;
	uint32 accelCounter = 0;
	int32 lightDiff = 0;
	int32 lightMax = -1;
	uint8 controllerPulseTimer = 0;




	lightSensorFROffset = light_sensor_get_value(LIGHT_SENSOR_FRONT_RIGHT);
	lightSensorFLOffset = light_sensor_get_value(LIGHT_SENSOR_FRONT_LEFT);
	lightSensorRLOffset = light_sensor_get_value(LIGHT_SENSOR_REAR_LEFT);
	lightSensorRROffset = light_sensor_get_value(LIGHT_SENSOR_REAR_RIGHT);
    neighborsIgnore(126);
	motorSetPWM(MOTOR_LEFT, 0);
	motorSetPWM(MOTOR_RIGHT, 0);

	//osTaskDelay(1000);
	for (;;) {
		if (cprintfTerminalGetHost() == CPRINTF_TERMINAL_REMOTE) {
			neighborsGetMutex();
			nbrListCreate(&nbrList);
//			nbrListPrint(&nbrList, "nbrs");
			currentTime = osTaskGetTickCount();

			uint8 bumpBits = bumpSensorsGetBits();
			cprintf("behavior %4d\n",state_mike);

			lightSensorFR = light_sensor_get_value(LIGHT_SENSOR_FRONT_RIGHT)  - lightSensorFROffset;
			lightSensorFL = light_sensor_get_value(LIGHT_SENSOR_FRONT_LEFT) - lightSensorFLOffset;
			lightSensorRL = light_sensor_get_value(LIGHT_SENSOR_REAR_LEFT) - lightSensorRLOffset;
			lightSensorRR = light_sensor_get_value(LIGHT_SENSOR_REAR_RIGHT)  - lightSensorRROffset;

			lightDiff = 0;
			lightMax = -1;

			if((lightSensorRL > lightSensorRR)&&(lightSensorRL > lightSensorFL)&&(lightSensorRL > lightSensorFR)) {
				lightDiff = 0;
				lightMax = LIGHT_SENSOR_REAR_LEFT;
			} else if((lightSensorRR > lightSensorRL)&&(lightSensorRR > lightSensorFL)&&(lightSensorRR > lightSensorFR)) {
				lightDiff = 0;
				lightMax = LIGHT_SENSOR_REAR_RIGHT;
			} else {
				lightDiff = lightSensorFL-lightSensorFR;
				if (lightSensorFR > lightSensorFL) {
					lightMax = LIGHT_SENSOR_FRONT_RIGHT;
				} else {
					lightMax = LIGHT_SENSOR_FRONT_LEFT;
				}
			}
			//cprintf("light FR% 4d  FL% 4d  RL% 4d  RR% 4d  FrontDiff% 4d max% 2d\n", 	lightSensorFR, lightSensorFL, lightSensorRL, lightSensorRR, lightDiff, lightMax);
			//compute control law to move to light
			//cprintf("%d %d\n", lightSensorFR,lightSensorFL );


			//just rotate
			/*if(lightSensorFR>lightSensorFL)
			{
				mikes_stupid_motor_function(1, 0, 60);
				mikes_stupid_motor_function(0, 1, 60);
			}
			else
			{
				mikes_stupid_motor_function(1, 1, 60);
				mikes_stupid_motor_function(0, 0, 60);

			}*/


			/*if(accelerometerGetValue(ACCELEROMETER_Z) < -800) {
				accelCounter++;
				if (accelCounter > 20) {
					state_mike = BEH_MODE_SEARCH_THEN_PHOTOTAXIS;
				}
			} else {
				accelCounter = 0;
			}
*/

			behMove = behInactive;

			switch (state_mike) {
			case BEH_MODE_DO_NOTHING: {


				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOWEST, LED_RATE_MED);
				motorSetPWM(MOTOR_LEFT, 0);
				motorSetPWM(MOTOR_RIGHT, 0);
				break;
			}case BEH_MODE_RANDOM_ROTATE: {

				cprintf("rrtimer % 4d , direction %4d \n",rrtimer,direction);
				if(start==0)
				{
					srand(lightSensorFR+lightSensorFL+lightSensorRL+lightSensorRR);
					 rrtimer=rand()%30;
					direction=rand()%2;

					start=1;
				}
				if(direction==0)
				{
					motorSetPWM(MOTOR_LEFT, -100);
					motorSetPWM(MOTOR_RIGHT, 100);
					rrtimer--;


				}
				else
				{
					motorSetPWM(MOTOR_LEFT, 100);
					motorSetPWM(MOTOR_RIGHT, -100);
				}
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOWEST, LED_RATE_MED);

				rrtimer--;
				if(rrtimer<0)
				{
					state_mike=BEH_MODE_PWM_CONTROLLER;
				}

				break;
			}
			case BEH_MODE_SEARCH_THEN_FLOCK:
			case BEH_MODE_SEARCH_THEN_PHOTOTAXIS: {
//				mikes_stupid_motor_function(1, 1,75);
//				mikes_stupid_motor_function(0, 1, 75);
				motorSetPWM(MOTOR_LEFT, 75);
				motorSetPWM(MOTOR_RIGHT, 75);
				if (state_mike == BEH_MODE_SEARCH_THEN_FLOCK) {
					if(bumpSensorsGetBits() > 0) {
						state_mike = BEH_MODE_FLOCKING;
					}
					ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOWEST, LED_RATE_MED);
				} else if (state_mike == BEH_MODE_SEARCH_THEN_PHOTOTAXIS) {
					if(bumpSensorsGetBits() > 0) {
						state_mike = BEH_MODE_PHOTOTAXIS;
					}
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOWEST, LED_RATE_MED);
				}
				break;
			}
			case BEH_MODE_PWM_CONTROLLER:{
				int32 tvgain, error;


				alpha = behFlockAngle(&nbrList);
				alphaMax = behFlockAngleMax(&nbrList);

				error = alpha / 100;
				flockAngleIntegral = flockAngleIntegral * FLOCK_INTEGRAL_DECAY / 1000;
				flockAngleIntegral += (error * error) * (error > 0 ? 1 : -1);
				flockAngleIntegral = bound(flockAngleIntegral, -FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
				tvgain = 100 - (abs(flockAngleIntegral) * 100)/FLOCK_INTEGRAL_MAX;
				//tvgain = 100 - ((flockAngleIntegral*flockAngleIntegral) * 100)/(FLOCK_INTEGRAL_MAX*FLOCK_INTEGRAL_MAX);
				behMove.tv = FLOCK_TV * tvgain / 100;

				if(abs(flockAngleIntegral)==FLOCK_INTEGRAL_MAX)
				{
					int i=0;
					//cprintf("in maximum flockangle\n");
					if(flockAngleIntegral<0)
					{
						motorSetPWM(MOTOR_LEFT, 100);
						motorSetPWM(MOTOR_RIGHT, -100);
					}
					else
					{
						motorSetPWM(MOTOR_LEFT, -100);
						motorSetPWM(MOTOR_RIGHT, 100);
					}
					flockAngleIntegral=flockAngleIntegral*0;
					controllerPulseTimer = PWM_CONTROLLER_PULSE_TIME;
					state_mike = BEH_MODE_PWM_CONTROLLER_PULSE;
				}
				else {
					int32 rv_I = flockAngleIntegral * K_I/100 ; //WEIGHTED TEST WORKS WITH 2
					int32 rv_P = error * K_P/100; //WEIGHTED TEST WORKS WITH 2

					//cprintf("error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",	alpha, flockAngleIntegral, rv_I, rv_P );

					behMove.rv = (rv_I + rv_P) ;
					//behMove.tv=0;
						//rv = ((100 - (networkSize*100)/4) * rv) / 100;



					behMove.active = TRUE;

					//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));

					motorSetBeh(&behMove);
				}

				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;

			}
			case BEH_MODE_PWM_CONTROLLER_PULSE:{
				controllerPulseTimer--;
				if(controllerPulseTimer == 0) {
					motorSetPWM(MOTOR_LEFT, 0);
					motorSetPWM(MOTOR_RIGHT,0);
					state_mike = BEH_MODE_PWM_CONTROLLER;
					cprintf("end delay\n");
				}
				break;
			}
			case BEH_MODE_PWM_CONTROLLER1:{
				int32 tvgain, error;


				alpha = behFlockAngle(&nbrList);
				alphaMax = behFlockAngleMax(&nbrList)/100;

				error = alpha / 100;
				flockAngleIntegral = flockAngleIntegral * FLOCK_INTEGRAL_DECAY / 1000;
				flockAngleIntegral += (error * error) * (error > 0 ? 1 : -1);
				flockAngleIntegral = bound(flockAngleIntegral, -FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
				tvgain = 100 - (abs(flockAngleIntegral) * 100)/FLOCK_INTEGRAL_MAX;
				//tvgain = 100 - ((flockAngleIntegral*flockAngleIntegral) * 100)/(FLOCK_INTEGRAL_MAX*FLOCK_INTEGRAL_MAX);
				behMove.tv = FLOCK_TV * tvgain / 100;



				if(abs(flockAngleIntegral)==FLOCK_INTEGRAL_MAX*.7)
				{
					int i=0;
					//cprintf("in maximum flockangle\n");
					if(flockAngleIntegral<0)
					{
						motorSetPWM(MOTOR_LEFT, 100);
						motorSetPWM(MOTOR_RIGHT, -100);
					}
					else
					{
						motorSetPWM(MOTOR_LEFT, -100);
						motorSetPWM(MOTOR_RIGHT, 100);
					}
					flockAngleIntegral=flockAngleIntegral*0;
					for( i=0;i<100;i++)
					{
						cprintf("   ");
					}
					motorSetPWM(MOTOR_LEFT, 0);
					motorSetPWM(MOTOR_RIGHT,0);
					cprintf("end delay\n");


				}

				static int previous_ri=0;

				if(flockAngleIntegral>0)
				{
					if(previous_ri<0)
					{
						flockAngleIntegral=0;
					}
				}
				else
				{
					if(previous_ri>0)
					{
						flockAngleIntegral=0;
					}
				}


				//behMove.rv = flockAngleIntegral;

				int32 rv_I = flockAngleIntegral * K_I/100 ; //WEIGHTED TEST WORKS WITH 2
				int32 rv_P = error * K_P/100; //WEIGHTED TEST WORKS WITH 2

				cprintf("error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",	alpha, flockAngleIntegral, rv_I, rv_P );

				behMove.rv = (rv_I + rv_P) ;
				behMove.tv=0;
					//rv = ((100 - (networkSize*100)/4) * rv) / 100;

				behMove.active = TRUE;

				//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));

				motorSetBeh(&behMove);
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;

			}case BEH_MODE_PULSE_CONTROLLER:{
				int32 tvgain, error;
				motorSetPWM(MOTOR_LEFT, 0);
				motorSetPWM(MOTOR_RIGHT,0);
				cprintf("in cont\n");


				/*static int rI;
				static int rP;

				static int previous_alpha,alpha;

				alpha = behFlockAngle(&nbrList);
				//alphaMax = behFlockAngleMax(&nbrList)/100;
				alpha


				if(alpha!=previous_alpha)
				{

					if(alpha>0)
					{
						motorSetPWM(MOTOR_LEFT, -100);
						motorSetPWM(MOTOR_RIGHT, 100);


					}
					else
					{
						motorSetPWM(MOTOR_LEFT, 100);
						motorSetPWM(MOTOR_RIGHT,-100);

					}
					int i;
					for( i=0;i<100;i++)//delay
					{
						//cprintf("   ");
					}

					motorSetPWM(MOTOR_LEFT, 0);
					motorSetPWM(MOTOR_RIGHT,0);

				}
				previous_alpha=alpha;
				cprintf("alpha %4d   alphaprev %4d  rI %4d   \n",	alpha ,previous_alpha,rI);*/

				break;

			}
			case BEH_MODE_FLOCKING: {
				int32 tvgain, error;
				// either push or flock, depending onthe experiment
//				alpha = behFlockAngle(&nbrList);
//				rvBearingController(&behMove, alpha, 90);
//				error = behFlockAngleMax(&nbrList)/ 100;
//				if(abs(error) > 3) {
//					//behMove.tv = 0;
//					if(error > 0) {
//						motorSetPWM(MOTOR_LEFT, -PHOTOTAXIS_PWM);
//						motorSetPWM(MOTOR_RIGHT, PHOTOTAXIS_PWM);
//					} else {
//						motorSetPWM(MOTOR_LEFT, PHOTOTAXIS_PWM);
//						motorSetPWM(MOTOR_RIGHT, -PHOTOTAXIS_PWM);
//					}
//					cprintf("rotate    error % 4d  tv% 4d size% 4d\n",	error, behMove.tv, nbrList.size );
//				} else {
////					behMove.tv = FLOCK_TV;
//					motorSetPWM(MOTOR_LEFT, PHOTOTAXIS_PWM);
//					motorSetPWM(MOTOR_RIGHT, PHOTOTAXIS_PWM);
//					cprintf("translate error % 4d  tv% 4d size% 4d\n",	error, behMove.tv, nbrList.size );
//				}

				alpha = behFlockAngle(&nbrList);
				alphaMax = behFlockAngleMax(&nbrList)/100;

				error = alpha / 100;
				flockAngleIntegral = flockAngleIntegral * FLOCK_INTEGRAL_DECAY / 1000;
				flockAngleIntegral += (error * error) * (error > 0 ? 1 : -1);
				flockAngleIntegral = bound(flockAngleIntegral, -FLOCK_INTEGRAL_MAX, FLOCK_INTEGRAL_MAX);
				tvgain = 100 - (abs(flockAngleIntegral) * 100)/FLOCK_INTEGRAL_MAX;
				//tvgain = 100 - ((flockAngleIntegral*flockAngleIntegral) * 100)/(FLOCK_INTEGRAL_MAX*FLOCK_INTEGRAL_MAX);
				behMove.tv = FLOCK_TV * tvgain / 100;



				//behMove.rv = flockAngleIntegral;

				int32 rv_I = flockAngleIntegral * K_I/100 ; //WEIGHTED TEST WORKS WITH 2
				int32 rv_P = error * K_P/100; //WEIGHTED TEST WORKS WITH 2

				cprintf("error % 4d  flockAngleIntegral% 5d  rv_I% 4d  rv_P% 4d \n",	alpha, flockAngleIntegral, rv_I, rv_P );

				behMove.rv = (rv_I + rv_P) ;

					//rv = ((100 - (networkSize*100)/4) * rv) / 100;



				behMove.active = TRUE;

				//cprintf("error % 4d  flockAngleIntegral% 5d  tvgain% 4d  tv% 4d nbr% 4d\n",	alpha, flockAngleIntegral, tvgain, behMove.tv ,nbrGetID(nbrList ->nbrs));

				motorSetBeh(&behMove);
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;
			}
			case BEH_MODE_PHOTOTAXIS: {
				static int offset=0;
				if(lightMax == LIGHT_SENSOR_REAR_LEFT) {
					// rotate counterclockwise
					motorSetPWM(MOTOR_LEFT, -PHOTOTAXIS_PWM);
					motorSetPWM(MOTOR_RIGHT, PHOTOTAXIS_PWM);
				} else if(lightMax == LIGHT_SENSOR_REAR_RIGHT) {
					// rotate clockwise
					motorSetPWM(MOTOR_LEFT, PHOTOTAXIS_PWM);
					motorSetPWM(MOTOR_RIGHT, -PHOTOTAXIS_PWM);
				} else if(abs(lightDiff) < PHOTOTAXIS_DEADZONE) {
					// stop
					motorSetPWM(MOTOR_LEFT, 60);
					motorSetPWM(MOTOR_RIGHT, 60);
				} else {
					//robot 61=+50 robot 1=+75
					if(lightDiff < 0) {
						// rotate clockwise
						motorSetPWM(MOTOR_LEFT, PHOTOTAXIS_PWM);
						motorSetPWM(MOTOR_RIGHT, -PHOTOTAXIS_PWM);
					} else {
						// rotate counterclockwise
						motorSetPWM(MOTOR_LEFT, -PHOTOTAXIS_PWM);
						motorSetPWM(MOTOR_RIGHT, PHOTOTAXIS_PWM);
					}
				}
				ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_MED);
				break;
			}
			default:
				break;
			}

			//motorSetVelocity(1,200);

			neighborsPutMutex();
			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);

			//TODO: limit tv and rv.  (100 is a good limit?)
		}
		else
		{
			motorSetBeh(&behInactive);
		}

	}
	return;
}






