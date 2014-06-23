////////////////////////////////
/**
 * @file: https://svn.rice.edu/r/mrone/robotcode/MultiRobotManipulation/src/bug0.c
 *
 * @since July 31st, 2013
 * @author James McLurkin
 *
 * @brief: Bug0 algorithm for MRM
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

#define MOTION_TV					80

#define MODE_IDLE					0
#define MODE_SEARCH_FOR_OBJECT		1
#define MODE_TRANSPORT_TO_LIGHT		2
#define MODE_AT_LIGHT				3
#define MODE_FOLLOW_WALL_LEFT		4
#define MODE_FOLLOW_WALL_RIGHT		5



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

	boolean targetSeen = 0;
	boolean lightActivated = 0;

	boolean TaskDone = 0;
	uint32 bumpSensorTimeOn = 0;
	uint32 bumpSensorTimeOff = 0;
	static boolean IntegralWindup = 0;

	uint8 currentState = MODE_IDLE;

	// search variables
	uint32 segmentNum = 0;
	uint32 TimeStart = currentTime;
	uint32 TimeForThisSegment = 0;
	static int ErrorLightPtr[2];
	ErrorLightPtr[0] = 0;
	ErrorLightPtr[1] = 0;
	//int ErrorlightPtr =&LightError;
	int32 light_value[4];


	neighborsInit(NEIGHBOR_ROUND_PERIOD);

	int32 light_value_init[4];

	light_value_init[0] = lightSensorGetValue(0); // front right light sensor
	light_value_init[1] = lightSensorGetValue(1); // front left light sensor
	light_value_init[2] = lightSensorGetValue(2); // rear left light sensor
	light_value_init[3] = lightSensorGetValue(3); // rear right sensor



	for (;;) {
		if (!rprintfIsHost()) {
			NbrList nbrList;
			Beh behMove = behInactive;

			neighborsGetMutex();
			nbrListCreate(&nbrList);

			currentTime = osTaskGetTickCount();
			buttonModeRGB(&currentState, MODE_FOLLOW_WALL_LEFT, MODE_IDLE, MODE_FOLLOW_WALL_RIGHT);


			uint8 bumpBits = bumpSensorsGetBits();

			if (bumpBits != 0 ) {
				bumpSensorTimeOn++;
				bumpSensorTimeOff = 0;
				isBumped = 1;
			}else{
				bumpSensorTimeOn = 0;
				bumpSensorTimeOff++;
			}

			light_value[0] = lightSensorGetValue(0)  - FR_OFFSET ; // front right light sensor
			light_value[1] = lightSensorGetValue(1); // front left light sensor
			light_value[2] = lightSensorGetValue(2); // rear left light sensor
			light_value[3] = lightSensorGetValue(3)  - RR_OFFSET; // rear right sensor
			//compute control law to move to light

			switch (currentState){
			case MODE_IDLE: {
				ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_SLOW);
				break;
			}
			case MODE_FOLLOW_WALL_LEFT: {
				behWallMove(&behMove, MOTION_TV, WALL_FOLLOW_LEFT);
				ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				break;
			}
			case MODE_FOLLOW_WALL_RIGHT: {
				behWallMove(&behMove, MOTION_TV, WALL_FOLLOW_RIGHT);
				ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_HIGH, LED_RATE_FAST);
				break;
			}
			case MODE_SEARCH_FOR_OBJECT:{
				ledsSetPattern(LED_RED, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_MED);

				if(bumpSensorTimeOn > 5){
					currentState = MODE_TRANSPORT_TO_LIGHT;
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
				break;
			}
			case MODE_TRANSPORT_TO_LIGHT:{
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
				behLight(&behMove , light_value, ErrorLightPtr,&TaskDone, &IntegralWindup );
				//behSetTv(behPtr, behPtr->tv);
				//behSetRv(behPtr, behPtr->rv);
				break;
			}
			case MODE_AT_LIGHT:{
				ledsSetPattern(LED_GREEN, LED_PATTERN_ON, LED_BRIGHTNESS_MED, LED_RATE_MED);
				behSetTvRv(&behMove, 0, 0);
				if ( TaskDone != 1 ){
					currentState = MODE_TRANSPORT_TO_LIGHT;
				}
				break;
			}
			}
			//cprintf("%d:%d,%d,%d,%d,%d,%d,%d\n", roneID, lightActivated, targetSeen, isBumped, bumpBits, light_value[0], light_value[1], TaskDone);


			motorSetBeh(&behMove);
			neighborsPutMutex();

			osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD);

			//TODO: limit tv and rv.  (100 is a good limit?)
		}
	}
	return;
}



/******** boilerplate main function.  probably don't need to change anything here ********/

int main(void) {
	// init the rone hardware and roneos services
	systemInit();

	// init the behavior system and start the behavior thread
	behaviorSystemInit(behaviorTask, 4096);

	// Start the scheduler
	osTaskStartScheduler();

	// should never get here.  If so, you have a bad memory problem in the scheduler
	return 0;
}








