/*
 * @file motor.c
 * @brief functions dealing with the two motors on the rone
 * @since Mar 2, 2011
 * @author: jamesm
 */

#include <stdlib.h>

#ifdef PART_LM3S9B92
#include "inc/lm3s9b92.h"
#endif
#ifdef PART_LM3S8962
#include "inc/lm3s8962.h"
#endif

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_qei.h"

#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "roneos.h"


/******** Defines ********/

#ifdef PART_LM3S9B92
/**MOT0 only includes the mapping for the F port**/
#define PWM0 			GPIO_PA6_PWM0
#define PWM1 			GPIO_PA7_PWM1
#define PWM0_PORT_BASE 	GPIO_PORTA_BASE
#define PWM1_PORT_BASE 	GPIO_PORTA_BASE
#define PWM0_PERIPH 	SYSCTL_PERIPH_GPIOA
#define PWM1_PERIPH 	SYSCTL_PERIPH_GPIOA
#define PWM0_PIN 		GPIO_PIN_6
#define PWM1_PIN 		GPIO_PIN_7

/**MOT1**/
#define PWM2 			GPIO_PH0_PWM2
#define PWM3 			GPIO_PH1_PWM3
#define PWM2_PORT_BASE 	GPIO_PORTH_BASE
#define PWM3_PORT_BASE 	GPIO_PORTH_BASE
#define PWM2_PERIPH 	SYSCTL_PERIPH_GPIOH
#define PWM3_PERIPH 	SYSCTL_PERIPH_GPIOH
#define PWM2_PIN 		GPIO_PIN_0
#define PWM3_PIN 		GPIO_PIN_1

#endif

#ifdef PART_LM3S8962
/**MOT0**/
#define PWM0 			GPIO_PF0_PWM0
#define PWM1 			GPIO_PG1_PWM1
#define PWM0_PORT_BASE 	GPIO_PORTF_BASE
#define PWM1_PORT_BASE 	GPIO_PORTG_BASE
#define PWM0_PERIPH 	SYSCTL_PERIPH_GPIOF
#define PWM1_PERIPH 	SYSCTL_PERIPH_GPIOG
#define PWM0_PIN 		GPIO_PIN_0
#define PWM1_PIN 		GPIO_PIN_1

/**MOT1**/
#define PWM2 			GPIO_PB0_PWM2
#define PWM3 			GPIO_PB1_PWM3
#define PWM2_PORT_BASE 	GPIO_PORTB_BASE
#define PWM3_PORT_BASE 	GPIO_PORTB_BASE
#define PWM2_PERIPH 	SYSCTL_PERIPH_GPIOB
#define PWM3_PERIPH 	SYSCTL_PERIPH_GPIOB
#define PWM2_PIN 		GPIO_PIN_0
#define PWM3_PIN 		GPIO_PIN_1

#endif

#define MOTION_TV_MIN						3
#define MOTION_TV_GAIN						100
#define MOTION_RV_MAX						1000
#define MOTION_RV_GAIN						69
#define MOTION_RV_TURN_INPLACE_GAIN			150
#define ANGLE_ERROR_GAIN					20
#define WAYPOINT_DISTANCE_CAPTURE			5	// 5 mm down from 15 mm
#define WAYPOINT_DISTANCE_HOLD				(WAYPOINT_DISTANCE_CAPTURE * 2)
#define WAYPOINT_ANGLE_CAPTURE				10 // 10 milliradians = 0.57 degrees, down from 150 milliradians = 8.9 degrees
#define WAYPOINT_ANGLE_HOLD					(WAYPOINT_ANGLE_CAPTURE * 2)
#define WAYPOINT_CAPTURE_TV					(MOTION_TV_MIN * 3)
#define MOTION_ROTATE_ONLY_CAPTURE_ANGLE	(MILLIRAD_PI / 1)
#define MOTION_ROTATE_ONLY_RELEASE_ANGLE	50

#define MOTOR_PWM_FREQUENCY 	22000
#define MOTOR_COMMAND_TIMEOUT 	20

#define MOTOR_FORWARD 		0
#define MOTOR_REVERSE 		1

#define MOTORS_NUM		    2

#define MOTOR_KFF 			22
#define MOTOR_KFF_OFFSET 	36
#define MOTOR_KP 			80
#define MOTOR_KD 			15
#define MOTOR_KI   			40

#define MOTOR_VEL_RAMP		16


/******** structs ********/
/*
 * @brief contains information on the robot's state used for smooth motor control
 */
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



/******** functions ********/
//static void motorVelocityDataInit(motorVelocityData* velPtr, uint8 motor);


/******** variables ********/
static uint32 motor_command_timer;
static motorVelocityData motorData[MOTORS_NUM];
static boolean motorCommandMode;



/*
 * 	@brief Enables PWM according to the specified motor.
 *
 * 	@param motor specifies left or right motor (MOTOR_RIGHT/MOTOR_LEFT)
 * 	@returns void
 */
static void motorEnablePwm(int16 motor) {
	unsigned long ulPeriod;
	volatile uint32 ulLoop;  // make this volatile so it doesn't get optimized away

	uint32 port_base_a = 0;
	uint32 port_base_b = 0;
	uint32 port_periph_a = 0;
	uint32 port_periph_b = 0;
	uint8 pin_a = 0;
	uint8 pin_b = 0;
	uint32 pwm_a = 0;
	uint32 pwm_b = 0;
	uint32 pwmgen = 0;
	uint8 pwmoutbit = 0;

	//mot0 = pa6,pa7, mot1 = ph0,ph1 lm3s9b92
	//mot0 = pf0, pg1, mot1 = pe0, pe1
	if (motor == MOTOR_RIGHT) {
		port_base_a = PWM0_PORT_BASE;
		port_base_b = PWM1_PORT_BASE;
		port_periph_a = PWM0_PERIPH;
		port_periph_b = PWM1_PERIPH;
		pin_a = PWM0_PIN;
		pin_b = PWM1_PIN;
		pwm_a = PWM0;
		pwm_b = PWM1;
		pwmgen = PWM_GEN_0;
		pwmoutbit = PWM_OUT_0_BIT | PWM_OUT_1_BIT;
	} else if (motor == MOTOR_LEFT) {
		port_base_a = PWM2_PORT_BASE;
		port_base_b = PWM3_PORT_BASE;
		port_periph_a = PWM2_PERIPH;
		port_periph_b = PWM3_PERIPH;
		pin_a = PWM2_PIN;
		pin_b = PWM3_PIN;
		pwm_a = PWM2;
		pwm_b = PWM3;
		pwmgen = PWM_GEN_1;
		pwmoutbit = PWM_OUT_2_BIT | PWM_OUT_3_BIT;
	} else {
		return;
	}

	//enable the peripherals
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
	MAP_SysCtlPeripheralEnable(port_periph_a);
	MAP_SysCtlPeripheralEnable(port_periph_b);
	// Wait for port to be enabled.
	for (ulLoop = 0; ulLoop < 10; ) {ulLoop++;}

	//set the pins as pwm pins
	GPIOPinConfigure(pwm_a);
	GPIOPinConfigure(pwm_b);
	MAP_GPIOPinTypePWM(port_base_a, pin_a);
	MAP_GPIOPinTypePWM(port_base_b, pin_b);

	// Compute the PWM period based on the system clock.
	ulPeriod = SYSCTL_CLOCK_FREQ / MOTOR_PWM_FREQUENCY;

	// Set the PWM period.
	MAP_PWMGenConfigure(PWM_BASE, pwmgen, PWM_GEN_MODE_UP_DOWN
			| PWM_GEN_MODE_NO_SYNC);
	MAP_PWMGenPeriodSet(PWM_BASE, pwmgen, ulPeriod);

	// Enable the PWM generator.
	MAP_PWMGenEnable(PWM_BASE, pwmgen);
	//enable output state.
	MAP_PWMOutputState(PWM_BASE, pwmoutbit, true);
}

/*
 * 	@brief Initializes variables of the motorVelocityData struct.
 *
 * 	Initializations include motor, ticks, velGoal, iTerm, and error.
 * 	Sets motor and ticks according to the input parameter.
 * 	Sets the others to zero.
 *	@param velPtr a pointer that points to the motorVelocityData struct
 * 	@param motor specifies which motor you want
 * 	@returns void
 */
static void motorVelocityDataInit(motorVelocityData* velPtr, uint8 motor) {
	velPtr->motor = motor;
	velPtr->ticks = encoderGetTicks(motor);
	velPtr->velGoal = 0;
	velPtr->velGoalRamp = 0;
	velPtr->iTerm = 0;
	velPtr->error = 0;
}

/*
 * @brief Initializes motor.
 *
 * Enables PWM and initializes the motorVelocityData struct for both left and right motor.
 * Also sets the command timer to 0
 * @returns void
 */
void motorInit() {
	motorEnablePwm(MOTOR_LEFT);
	motorEnablePwm(MOTOR_RIGHT);
	motorCommandMode = MOTOR_COMMAND_MODE_VELOCITY;
	motorVelocityDataInit(&motorData[MOTOR_LEFT], MOTOR_LEFT);
	motorVelocityDataInit(&motorData[MOTOR_RIGHT], MOTOR_RIGHT);
	motor_command_timer = 0;
}


/*
 * @brief Sets PWM duty cycle for the specified motor and signal.
 *
 * @param motor left or right motor
 * @param signal forward or backward
 * @param dutyCycle duty cycle of PWM
 * @returns 0 if the properties are successfully set; -1 if any of the input parameter is not recognized
 */
static int motorSetPWM_LowLevel(int motor, int signal, int dutyCycle) {
	unsigned long ulPeriod;

	uint32 pwmouta = 0;
	uint32 pwmoutb = 0;
	ulPeriod = SYSCTL_CLOCK_FREQ / MOTOR_PWM_FREQUENCY;

	if (dutyCycle >= 100) {
		dutyCycle = 100;
		ulPeriod = (ulPeriod * 99) / 100;
	} else if (dutyCycle < 0) {
		dutyCycle = 0;
		ulPeriod = 0;
	} else {
		ulPeriod = (ulPeriod * dutyCycle) / 100;
	}

	if (motor == MOTOR_LEFT) {
		pwmouta = PWM_OUT_0;
		pwmoutb = PWM_OUT_1;
	} else if (motor == MOTOR_RIGHT) {
		pwmouta = PWM_OUT_2;
		pwmoutb = PWM_OUT_3;
	} else {
		return -1;
	}

	if (signal == MOTOR_FORWARD) {
		MAP_PWMPulseWidthSet(PWM_BASE, pwmouta, ulPeriod);
	} else if (signal == MOTOR_REVERSE) {
		MAP_PWMPulseWidthSet(PWM_BASE, pwmoutb, ulPeriod);
	} else {
		return -1;
	}
	return 0;
}

/*
 * @brief Resets motor timer.
 *
 * Resets the motor timer to 20. Also disable the charger
 * @returns void
 */
static void motorCommandTimerReset(void) {
	motor_command_timer = MOTOR_COMMAND_TIMEOUT;
	charger_disable();
}

static void motorSetPWM_NonCmd(uint32 motor, int32 dutyCycle) {
	int motReverseLine = 0;
	int motForwardLine = 0;


	/*
	 * As per Mike Ciholas's suggestion, modulate between on and "brake" to
	 * preserve torque, unless we are going for 0 PWM, then just disable the motor.
	 *
	 * "... the actual correct method is to modulate between 'driving' and
	 * 'brake'.  Yes, this sounds counter intuitive, but it is the right
	 * way and makes the motor do first order speed control on its own.
	 *
	 * The key is thinking of the motor as an inductor.  During the 'brake'
	 * time, the voltage across the inductor is near zero, so the change in
	 * current is minimized.  This preserves the torque at slow speeds leading to
	 * good low speed control.
	 *
	 * As long as the PWM frequency is high enough, there is no loss of
	 * performance using 'brake' over 'off'.  In fact, the motor will get
	 * more efficient and generate less electrical noise."
	 */
	if (motor == MOTOR_LEFT) {
		motReverseLine = MOTOR_REVERSE;
		motForwardLine = MOTOR_FORWARD;
	} else if (motor == MOTOR_RIGHT) {
		motReverseLine = MOTOR_REVERSE;
		motForwardLine = MOTOR_FORWARD;
	} else {
		return;
	}

	if (dutyCycle == 0) {
		// stop (coast)
		motorSetPWM_LowLevel(motor, motReverseLine, 0);
		motorSetPWM_LowLevel(motor, motForwardLine, 0);
	} else if (dutyCycle > 0) {
		motorCommandTimerReset();
		// forward. Modulate between brake (both ON) and the reverse being off (forward)
		motorSetPWM_LowLevel(motor, motReverseLine, 100);
		motorSetPWM_LowLevel(motor, motForwardLine, 100-dutyCycle);
	} else if (dutyCycle < 0) {
		motorCommandTimerReset();
		//reverse. Modulate between brake (both ON) and forward being off (reverse)
		motorSetPWM_LowLevel(motor, motReverseLine, 100+dutyCycle);
		motorSetPWM_LowLevel(motor, motForwardLine, 100);
	}
}


/*
 * @brief Sets PWM duty cycle for the specified motor if remote control mode is off.
 *
 * Sets PWM duty cycle for the specified motor for both reverse and forward signals.
 * @param motor left or right motor
 * @param dutyCycle duty cycle of PWM
 * @returns void
 */
void motorSetPWM(uint32 motor, int32 dutyCycle) {

	if (rcMode == RC_MODE_OFF){
		motorSetPWM_rc(motor, dutyCycle);
	}

	//motorCommandMode = MOTOR_COMMAND_MODE_PWM;
	//motorSetPWM_NonCmd(motor, dutyCycle);
}

/*
 * \internal
 * @brief Sets PWM duty cycle for the specified motor. Should not be called by the user.
 *        Call motorSetPWM() instead.
 *
 * Sets PWM duty cycle for the specified motor for both reverse and forward signals.
 * @param motor left or right motor
 * @param dutyCycle duty cycle of PWM
 * @returns void
 * \endinternal
 */
void motorSetPWM_rc(uint32 motor, int32 dutyCycle) {
	motorCommandMode = MOTOR_COMMAND_MODE_PWM;
	motorSetPWM_NonCmd(motor, dutyCycle);
}



/*
 * @brief Converts velocity to pwm
 *
 * @param goalvelocity velocity to be converted
 * @returns converted pwm
 */
static int32 velToPWM(int32 goalvelocity){
	int32 pwm;
	if (goalvelocity > 0){
		pwm = (goalvelocity * MOTOR_KFF) / 100 + MOTOR_KFF_OFFSET;
	} else if (goalvelocity < 0){
		pwm = (goalvelocity * MOTOR_KFF) / 100 - MOTOR_KFF_OFFSET;
	} else {
		pwm = 0;
	}
	return pwm;
}


/*
 * @brief Updates the velocity data for one motor.
 *
 * @param velPtr pointer to motorVelocityData
 * @returns void
 */
static void motorVelocityUpdateOne(motorVelocityData* velPtr){
	int32 vel, dticks;
	int32 ffterm, pterm, dterm, iterm;
	int32 PWM, error, derror;
	uint32 ticks = encoderGetTicks(velPtr->motor);
	uint32 curtime = osTaskGetTickCount();
	uint32 dtime = curtime - velPtr->time;

	//	static uint32 printTimer = 0;
	//    if (velPtr->motor == MOTOR_LEFT){
	//    	printTimer++;
	//    }

	if (dtime == 0) {
		return;
	}

	dticks = encoderDeltaTicks(ticks, velPtr->ticks);
	vel = (dticks * 625) / ((int32)(dtime * 10));
	//    if ((velPtr->motor == MOTOR_LEFT) && ((printTimer % 4) == 0)) {
	////    	serial_send_long(" new=",curticks);
	////    	serial_send_long(" old=",velPtr->ticks);
	//    	serial_send_long(" dt=",encoder_delta_ticks(curticks, velPtr->ticks));
	//    	serial_send_long(" v=",vel);
	//    	serial_send_long(" vg=",motorData[velPtr->motor].velGoal);
	//
	//    }

	// ramp velGoal
	if (abs(velPtr->velGoal - velPtr->velGoalRamp) < MOTOR_VEL_RAMP) {
		velPtr->velGoal = velPtr->velGoalRamp;
	} else if (velPtr->velGoal < velPtr->velGoalRamp) {
		velPtr->velGoal += MOTOR_VEL_RAMP;
	} else if (velPtr->velGoal > velPtr->velGoalRamp) {
		velPtr->velGoal -= MOTOR_VEL_RAMP;
	}

	error    = velPtr->velGoal - vel;
	derror   = error - velPtr->error;
	ffterm   = velToPWM(velPtr->velGoal);
	pterm    = MOTOR_KP * error;
	dterm    = -MOTOR_KD * derror;
	iterm    = velPtr->iTerm + MOTOR_KI * error;
	//			8 bits  (16bits  16bits  16bits)/8bits  = 18bits in numer
	PWM      = ffterm + (pterm + dterm + iterm)/100;

	velPtr->vel = vel;
	velPtr->time = curtime;
	velPtr->ticks = ticks;

	//    if ((velPtr->motor == MOTOR_LEFT) && ((printTimer % 4) == 0)) {
	//    	serial_send_long(" e=",error);
	//    	serial_send_float(" ff=",ffterm);
	//    	serial_send_float(" p=",pterm);
	//    	serial_send_float(" d=",dterm);
	//    	serial_send_float(" i=",iterm);
	//		serial_send_long(" PWM=",PWM);
	//		serial_send_string_crlf("");
	//    }

	if (velPtr->velGoal == 0) {
		iterm = 0.0;
		error = 0.0;
		PWM = 0;
	}
	velPtr->iTerm = iterm;
	velPtr->error = error;

	PWM = bound(PWM, -100, 100);
	motorSetPWM_NonCmd(velPtr->motor, PWM);
}


/*
 * @brief Brakes one motor with the specified duty cycle.
 *
 * @param motor (left or right)
 * @returns void
 */
void motorBrake(uint32 motor) {
	int motForwardLine = 0;
	int motReverseLine = 0;

	if (motor == MOTOR_LEFT) {
		motForwardLine = MOTOR_FORWARD;
		motReverseLine = MOTOR_REVERSE;
	} else if (motor == MOTOR_RIGHT) {
		motForwardLine = MOTOR_FORWARD;
		motReverseLine = MOTOR_REVERSE;
	} else {
		return;
	}
	motorCommandTimerReset();
	// brake by setting both forward and reverse. This brakes the motors.
	motorSetPWM_LowLevel(motor, motForwardLine, 100);
	motorSetPWM_LowLevel(motor, motReverseLine, 100);
}

/*
 * @brief Turns off the motors at a low level/
 *
 * @returns void
 */
void motorTimeoutStop(void) {
	// we have not received a motor command in a while.  Timeout and enable the charger
	motorSetPWM_NonCmd(MOTOR_LEFT, 0);
	motorSetPWM_NonCmd(MOTOR_RIGHT, 0);

	// enable charger
	charger_enable();
}

/*
 * @brief Updates the motor command timer.
 *
 * If no motor command has been received, timeout and enables the charger.
 * This function should be called at 10 hz.
 * @returns void
 */
void motorCommandTimerUpdate(void) {
	if (motor_command_timer > 0) {
		motor_command_timer--;
	}

	if (motor_command_timer == 0) {
		motorTimeoutStop();
	}
}


/*
 * @brief Sets the current velocity of the specified motor.
 *
 * @param motor left or right motor
 * @param velocity motor velocity to be set in mm/s
 * @returns void
 */
void motorSetVelocity_NonCmd(uint32 motor, int32 velocity) {
	if ((motor == MOTOR_LEFT) || (motor == MOTOR_RIGHT)) {
		motorData[motor].velGoalRamp = velocity;
	}
}

/*
 * @brief Sets the velocity of the specified motor if remote control
 *        mode is off.
 *
 * @param motor left or right motor
 * @param velocity motor velocity to be set in mm/s
 * @returns void
 */
void motorSetVelocity(uint32 motor, int32 velocity) {

	if (rcMode == RC_MODE_OFF){
		motorSetVelocity(motor, velocity);
	}
	//motorCommandMode = MOTOR_COMMAND_MODE_VELOCITY;
	//motorSetVelocity_NonCmd(motor, velocity);
}

/*
 * \internal
 * @brief Sets the velocity of the specified motor. Should not be called by user.
 *        Call motorSetVelocity instead.
 *
 * @param motor left or right motor
 * @param velocity motor velocity to be set in mm/s
 * @retuns void
 * \endinternal
 */
void motorSetVelocity_rc(uint32 motor, int32 velocity) {
	motorCommandMode = MOTOR_COMMAND_MODE_VELOCITY;
	motorSetVelocity_NonCmd(motor, velocity);
}


/*
 * @brief Gets the current velocity of the specified motor.
 *
 * @param motor left or right motor
 * @returns the current velocity; 0 if the input parameter is not recognized
 */
int32 motorGetVelocity(uint32 motor) {
	if ((motor == MOTOR_LEFT) || (motor == MOTOR_RIGHT)) {
		return motorData[motor].vel;
	} else {
		return 0;
	}
}


/*
 * @brief Sets the translational and radial velocity of the motor.
 *
 * @returns void
 */
void motorSetTVRV_NonCmd(int32 tv, int32 rv) {
	int32  velLeft = boundAbs(tv - rv * MOTOR_WHEEL_BASE_MM / 2000, VELOCITY_MAX);
	int32 velRight = boundAbs(tv + rv * MOTOR_WHEEL_BASE_MM / 2000, VELOCITY_MAX);
	motorSetVelocity_NonCmd(MOTOR_LEFT, velLeft);
	motorSetVelocity_NonCmd(MOTOR_RIGHT, velRight);
}


/*
 * @brief Sets the translation and radial velocity of the motors if remote control
 *        mode is off.
 *
 * @param tv the translational velocity
 * @param rv the rotational velocity
 * @returns void
 */
void motorSetTVRV(int32 tv, int32 rv) {
	if (rcMode == RC_MODE_OFF) {
		motorSetTVRV_rc(tv, rv);
	}
	//motorCommandMode = MOTOR_COMMAND_MODE_VELOCITY;
	//motorSetTVRV_NonCmd(tv, rv);
}


/*
 * \internal
 * @brief Sets the translation and radial velocity of the motors. Should not be
 *        called by the user. Call motorSetTVRV instead.
 *
 * @param tv the translational velocity
 * @param rv the rotational velocity
 * @returns void
 * \endinternal
 */
void motorSetTVRV_rc(int32 tv, int32 rv) {
	motorCommandMode = MOTOR_COMMAND_MODE_VELOCITY;
	motorSetTVRV_NonCmd(tv, rv);
}



/*
 * @brief Gets the translational and radial velocity of the motor.
 *
 * @param tvPtr pointer to the desired translational velocity
 * @param rvPtr pointer to the desired rotational velocity
 * @returns void
 */
void motorGetTVRV(int32* tvPtr, int32* rvPtr){
	int32 velL = motorData[MOTOR_LEFT].vel;
	int32 velR = motorData[MOTOR_RIGHT].vel;
	*tvPtr = (velL + velR) / 2;
	*rvPtr = (velR - velL) * 1000 / MOTOR_WHEEL_BASE_MM;
}





Pose waypointGoalPose;
boolean waypointDone = TRUE;
boolean waypointRotateOnly = TRUE;
int32 waypointSpeed = 50;

/*
 * @brief Returns whether current waypoint has been reached.
 *
 * @returns waypointDone boolean that keeps track of whether the current waypoint has been reached or not
 */
boolean waypointMoveDone(void) {
	return waypointDone;
}


void waypointMoveInit(void) {
	waypointDone = TRUE;
	waypointRotateOnly = FALSE;
}

/*
 * @brief MotorCommandMode set so that waypoint behavior with absolute coordinates.
 * Sets Goal point and speed according to Parameters
 *
 * @param posePtr - Pointer to position of Goal
 * @param speed
 * @returns void
 */
void waypointMove(Pose* posePtr, int32 speed) {
	motorCommandMode = MOTOR_COMMAND_MODE_WAYPOINT;
	waypointGoalPose = *posePtr;
	waypointSpeed = speed;
	waypointDone = FALSE;
}

/*
 * @brief MotorCommandMode set so that waypoint behavior can be set with relative coordinates.
 * Sets Goal point and speed according to Parameters
 *
 * @param posePtr - Pointer to position of Goal
 * @param speed
 *
 * @returns void
 */
void waypointMoveRelative(Pose* posePtr, int32 speed) {
	Pose pose;
	motorCommandMode = MOTOR_COMMAND_MODE_WAYPOINT;
	encoderGetPose(&pose);
	poseAdd(&waypointGoalPose, &pose, posePtr);
	waypointSpeed = speed;
	waypointDone = FALSE;
}


/*
* @brief MotorCommandMode set so that waypoint behavior  can be set with absolute coordinates for theta.
* Sets Goal point and speed according to Parameters
*
* @param posePtr - Pointer to position of Goal
* @param speed
* @returns void
*/
void waypointMoveTheta(Pose* posePtr, int32 speed) {
	motorCommandMode = MOTOR_COMMAND_MODE_WAYPOINT_THETA;
	waypointGoalPose = *posePtr;
	waypointSpeed = speed;
	waypointDone = FALSE;
}

/*
 * @brief MotorCommandMode set so that waypoint behavior can be set with relative coordinates for theta.
 * Sets Goal point and speed according to Parameters
 *
 * @param posePtr - Pointer to position of Goal
 * @param speed
 *
 * @returns void
 */
void waypointMoveThetaRelative(Pose* posePtr, int32 speed) {
	Pose pose;
	motorCommandMode = MOTOR_COMMAND_MODE_WAYPOINT_THETA;
	encoderGetPose(&pose);
	poseAdd(&waypointGoalPose, &pose, posePtr);
	waypointSpeed = speed;
	waypointDone = FALSE;
}



void waypointMoveUpdate(void) {
	Pose pose;
	int32 distance, tv, rv;

	//cprintf("beat this is a long string %d\n",counter++);
	//cprintf("c %d\n",counter++);
	//cprintf("beat this is a long string\n");

	tv = 0;
	rv = 0;
	if(!waypointDone) {
		encoderGetPose(&pose);
		distance = poseDistance(&pose, &waypointGoalPose);
		//cprintf("DISTANCE: %d\n", distance);
		//motorGetTVRV(&tv, &rv);

		if(distance < WAYPOINT_DISTANCE_CAPTURE) {
			waypointDone = TRUE;
		} else {
			tv = distance * MOTION_TV_GAIN / 100 + MOTION_TV_MIN;
			tv = bound(tv, -waypointSpeed, waypointSpeed);
			int32 xDif = waypointGoalPose.x - pose.x;
			int32 yDif = waypointGoalPose.y - pose.y;
			int32 angle_to_goal = atan2MilliRad(yDif, xDif);
			int32 bearing = smallestAngleDifference(angle_to_goal, pose.theta);

			rv = MOTION_RV_GAIN * bearing / 100;
			rv = bound(rv, -MOTION_RV_MAX, MOTION_RV_MAX);


			if(( bearing > MILLIRAD_HALF_PI)  || ( bearing < -MILLIRAD_HALF_PI)){
				//bearing = MILLIRAD_PI-bearing;
				angle_to_goal = atan2MilliRad(-1*yDif, -1*xDif);
				bearing = smallestAngleDifference(angle_to_goal, pose.theta);

				rv = MOTION_RV_GAIN * bearing / 100;
				rv = bound(rv, -MOTION_RV_MAX, MOTION_RV_MAX);

				tv = -tv;
				//rv = -rv;
			}

			//			// go backwards code
			//			if( bearing > MILLIRAD_HALF_PI){
			//				bearing = MILLIRAD_PI-bearing;
			//				tv = -tv;
			//				rv = -rv;
			//			}
			//			else if( bearing < -MILLIRAD_HALF_PI){
			//				bearing = MILLIRAD_PI+bearing;
			//				tv = -tv;
			//				rv = -rv;
			//			}
			//			// end go backwards code





			//if bearing is larger than 90 degrees, we only want to rotate
			//			if (abs(bearing) > MOTION_ROTATE_ONLY_CAPTURE_ANGLE) {
			//				tv = 0;
			//				waypointRotateOnly = TRUE;
			//			}

			//			//if bearing was larger than 90 degrees, we only want to rotate until the bearing is less than 36
			//			if(waypointRotateOnly){
			//				tv = 0;
			//				if(abs(bearing) < MOTION_ROTATE_ONLY_RELEASE_ANGLE) {
			//					waypointRotateOnly = FALSE;
			//				}
			//				if(abs(bearing) > MOTION_ROTATE_ONLY_CAPTURE_ANGLE){
			//					waypointRotateOnly = TRUE;
			//				}
			//			}

			//			cprintf("Rotate? %d", waypointRotateOnly);

		}
	}
	/* Only send motor commands if the robot isn't in remote control mode. */
	if (rcMode == RC_MODE_OFF) {
		motorSetTVRV_NonCmd(tv, rv);
	}
}

int getSign(int32 val) {
	if (val > 0) {
		return 1;
	} else if (val == 0) {
		return 0;
	} else {
		return -1;
	}
}




void waypointMoveUpdateTheta(void) {
	Pose pose;
	int32 distance, tv, rv;

	//cprintf("beat this is a long string %d\n",counter++);
	//	cprintf("c %d\n",counter++);
	//cprintf("beat this is a long string\n");

	tv = 0;
	rv = 0;
	if(!waypointDone) {
		encoderGetPose(&pose);
		distance = poseDistance(&pose, &waypointGoalPose);
		//cprintf("DISTANCE: %d\n", distance);
		//motorGetTVRV(&tv, &rv);

		if(distance < WAYPOINT_DISTANCE_HOLD) {
			int32 angleError = smallestAngleDifference(waypointGoalPose.theta, pose.theta);
			if (angleError < WAYPOINT_ANGLE_CAPTURE) {
				waypointDone = TRUE;
			} else {
				rv = MOTION_RV_TURN_INPLACE_GAIN * angleError / 100 + getSign(angleError) * ANGLE_ERROR_GAIN;
				rv = bound(rv, -MOTION_RV_MAX, MOTION_RV_MAX);
			}
			/* Only send motor commands if the robot isn't in remote control mode. */
			if (rcMode == RC_MODE_OFF) {
				motorSetTVRV_NonCmd(tv, rv);
			}
		} else {
			waypointMoveUpdate();
		}
	}
}


/*
 * @brief Updates the velocity data for both motors.
 *
 * @returns void
 */
void motorVelocityUpdate(void){
	switch(motorCommandMode) {
	case MOTOR_COMMAND_MODE_WAYPOINT:
		waypointMoveUpdate();
		break;
	case MOTOR_COMMAND_MODE_WAYPOINT_THETA:
		waypointMoveUpdateTheta();
		break;
	default:
		break;
	}
	if (motorCommandMode != MOTOR_COMMAND_MODE_PWM) {
		motorVelocityUpdateOne(&motorData[MOTOR_LEFT]);
		motorVelocityUpdateOne(&motorData[MOTOR_RIGHT]);
	}
}

