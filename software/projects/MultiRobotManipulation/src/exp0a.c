////////////////////////////////
/**
 * @file: https://svn.rice.edu/r/mrone/robotcode/MultiRobotManipulation/src/exp0a.c
 *
 * @since July 31st, 2012
 * @author Divya Bhat
 *
 * @brief: used in AAMAS paper to demonstrate multi-robot manipulation
 *
 * 1-  for light following ( no consensus) : Run exp0a.c
 * 2-  for leader following (consensus) : Run rotatelight.c
 *
 *
 *     TODO: K_PROPORTIONAL has been adjusted. Now K_INTEGRAL has to be adjusted (as you pick a high value and bring it down to stabilize
 *     		the robot, settle on one, and decrease K_PROPORTIONAL slightly if necessary). 0.001 may be the right value (the errors get very large).
 *
 *			Don't bother testing them while stationary-- it doesn't look right even with exact SuperDemo code.
 *     To get a better K_INTEGRAL_MAX, print out the integral values and see what range they tend to fall in. Pick a max based on that.
 *
 *  TODO: this code has no commenting.  Each function needs a description so we can easily fix problems.
 *
 */

//Behavior: move forward; if robot bumps into something, enter into flock behavior


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


#define K_I				            75
#define K_D				            25

#define FR_OFFSET				    90 // robot 64 = 85  ,  robot 61 = 90 , robot 62 =  00100
#define RR_OFFSET				    0




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
	neighborsInit(NEIGHBOR_ROUND_PERIOD);
	neighborsXmitEnable(TRUE);
	val1 = osTaskCreate(behaviorTask, "behavior", 4096, NULL, BEHAVIOR_TASK_PRIORITY);
	val2 = osTaskCreate(backgroundTask, "background", 1024, NULL, BACKGROUND_TASK_PRIORITY);

	if ((val1 != pdTRUE)) {
		cprintf("could not create a task");
	}

	/* Start the scheduler. */
	osTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
   	 task. This is a very bad thing, and should not happen.*/
	return 0;
}


// the background task runs all the time
void backgroundTask(void* parameters) {
	for (;;) {
		// delay to let other tasks run at same priority
		osTaskDelay(100);
	}
}


Beh* behLight(Beh* lightPtr, int32 light_value[],int ErrorlightPtr[],boolean *TaskDone, boolean *IntegralWindup) {
	// this function compute control law  (tv, rv) to move robot toward light and sets TaskDone to 1 if the robot is at the light
	boolean done = FALSE;

	//assumption is fl is 0, fr is 1
	// commented out  for adding controller
	//if (abs(diff) < 10)
	//sensor_offset = -diff;

	//else
	//sensor_offset = 0;

	// end comment
	int32 tv;
	int32 rv;
	//static int32 integralError = 0;

	int32 diff;
	if ((abs(light_value[0]) < 1200) &&  (abs(light_value[1]) < 1200 ) && (abs(light_value[2]) < 1200 ) &&  (abs(light_value[3]) < 1200 )) {
		//ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_MED);
		//uint32 deltaTicks = encoder_delta_ticks(currentTime, prevTime);

		//if (max ( light_value[2] , light_value[3] ) > max(light_value[1],light_value[0])){
		//if ((light_value[2]> light_value[1]) || (light_value[3],light_value[0])){
		if ( min(light_value[2],light_value[3]) > (100 * max (light_value[1],light_value[0]) /100)){
		//if (light_value[2]>min(light_value[2],light_value[3]) ){
			// diff =  (max ( light_value[2] , light_value[3] ) -  max(light_value[1],light_value[0]));
			diff = MILLIRAD_PI /4;}
		else{
			diff = light_value[1] - light_value[0];}

		//behPIController(lightPtr, diff, ErrorlightPtr,IntegralWindup);
		behPController(lightPtr, diff);

		//behPIDController(lightPtr, diff,ErrorlightPtr, IntegralWindup);
		// #### commented out  for adding controller
		//lightPtr->tv = SPEED;
		//lightPtr->rv = (sensor_offset + diff) * K;
		// #### end comment

		*TaskDone = 0;

	}
	else {
		ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_LOW, LED_RATE_MED);
		*TaskDone =1;
		lightPtr->tv = 0;
		lightPtr->rv = 0;
	}

	return lightPtr;
}



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
	Beh* behPtr = &behMove;




	int32 light_value_init[4];


	light_value_init[0] = light_sensor_get_value(0); // front right light sensor
	light_value_init[1] = light_sensor_get_value(1); // front left light sensor
	light_value_init[2] = light_sensor_get_value(2); // rear left light sensor
	light_value_init[3] = light_sensor_get_value(3); // rear right sensor
	//compute control law to move to light


	NbrList *nbrList = malloc(sizeof(NbrList));

	//initialize light sensor
	light_sensor_init();

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
	int32 light_value[4];


	for (;;) {
		if (cprintfTerminalGetHost() == CPRINTF_TERMINAL_REMOTE) {
			//		neighborsGetMutex();
			//		nbrListCreate(nbrList);
			currentTime = osTaskGetTickCount();

			uint8 bumpBits = bumpSensorsGetBits();

			if (bumpBits != 0 ) {
				bumpSensorTimeOn++;
				bumpSensorTimeOff = 0;
				isBumped = 1;
			}else{
				bumpSensorTimeOn = 0;
				bumpSensorTimeOff++;
			}


			light_value[0] = light_sensor_get_value(0)  - FR_OFFSET ; // front right light sensor
			light_value[1] = light_sensor_get_value(1); // front left light sensor
			light_value[2] = light_sensor_get_value(2); // rear left light sensor
			light_value[3] = light_sensor_get_value(3)  - RR_OFFSET; // rear right sensor
			//compute control law to move to light

			switch (currentState){
			case SEARCH_FOR_OBJECT:{
				ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_MED);

				if(bumpSensorTimeOn > 5){
					currentState = TRANSPORT_TO_LIGHT;
				}
				//TODO: spiral mode.  For now, move straight
				//behSetTv(&behMove, SPEED);
				//behSetRv(&behMove, 0);

				// have we completed this segment?
				if( currentTime >= TimeStart+TimeForThisSegment ){
					segmentNum++;
					TimeStart = currentTime;
					// we want segments of length diameter*(1,1,2,2,3,3,4,4,5,5,...) to make a spiral path
					//    = (number)*robot_diameter_in_mm*1000 ms/s / SPEED_in_mm/s.
					TimeForThisSegment = ((segmentNum+1)/2)*110*1000/SPEED;
				}
				//Compute control law

				behSetTv(&behMove, SPEED);
				//  we added 100 to the multiplication to make a tighter spiral.
				behSetRv(&behMove,  MILLIRAD_DEG_90*1100/TimeForThisSegment);


				break;}
			case TRANSPORT_TO_LIGHT:{
				ledsSetPattern(LED_BLUE, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_MED);

				/*if(bumpSensorTimeOff > 5){
					currentState = SEARCH_FOR_OBJECT;
					segmentNum = 0;
					TimeStart = currentTime;
					TimeForThisSegment = 0;
					break;

				}*/

				/*if ( TaskDone == 1 ){
					currentState = AT_LIGHT;
					break;
				}
*/
				// move toward the light
				behPtr = behLight(behPtr , light_value, ErrorLightPtr,&TaskDone, &IntegralWindup );
				behSetTv(behPtr, behPtr->tv);
				behSetRv(behPtr, behPtr->rv);

				break;}
			case AT_LIGHT:{
				ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_MED);
				behSetTvRv(behPtr, 0, 0);
				if ( TaskDone != 1 ){
					currentState = TRANSPORT_TO_LIGHT;
				}

				break;		}
			}
			cprintf("%d:%d,%d,%d,%d,%d,%d,%d\n", roneID, lightActivated, targetSeen, isBumped, bumpBits, light_value[0], light_value[1], TaskDone);


			motorSetBeh(&behMove);
			//		neighborsPutMutex();

			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);

			//TODO: limit tv and rv.  (100 is a good limit?)
		}
	}
	return;
}










