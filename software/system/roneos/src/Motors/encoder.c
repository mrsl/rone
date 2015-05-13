/*
 * @file encoder.c
 * @brief functions for wheel encoders
 * @since Mar 2, 2011
 * @author: James McLurkin
 */

#ifdef PART_LM3S9B92
	#include "inc/lm3s9b92.h"
#endif
#ifdef PART_LM3S8962
	#include "inc/lm3s8962.h"
#endif

#include <stdlib.h>

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

/**Defines**/
#ifdef PART_LM3S9B92

	/**QEI0**/
	#define QEI_PORT_BASE_0 GPIO_PORTC_BASE
	#define QEI_PERIPH_0 SYSCTL_PERIPH_GPIOC
	#define QEI_PIN_0_A GPIO_PC4_PHA0
	#define QEI_PIN_0_B GPIO_PF0_PHB0
	#define QEI_GPIO_PINS_0 GPIO_PIN_4

	/**QEI1**/
	#define QEI_PORT_BASE_1 GPIO_PORTE_BASE
	#define QEI_PERIPH_1 SYSCTL_PERIPH_GPIOE
	#define QEI_PIN_1_A GPIO_PE3_PHA1
	#define QEI_PIN_1_B GPIO_PE2_PHB1
	#define QEI_GPIO_PINS_1 GPIO_PIN_2 | GPIO_PIN_3

#endif

#ifdef PART_LM3S8962
	/**QEI0**/
	#define QEI_PORT_BASE_0 GPIO_PORTC_BASE
	#define QEI_PERIPH_0 SYSCTL_PERIPH_GPIOC
	#define QEI_PIN_0_A GPIO_PC4_PHA0
	#define QEI_PIN_0_B GPIO_PC6_PHB0
	#define QEI_GPIO_PINS_0 GPIO_PIN_4 | GPIO_PIN_6

	/**QEI1**/
	#define QEI_PORT_BASE_1 GPIO_PORTE_BASE
	#define QEI_PERIPH_1 SYSCTL_PERIPH_GPIOE
	#define QEI_PIN_1_A GPIO_PE3_PHA1
	#define QEI_PIN_1_B GPIO_PE2_PHB1
	#define QEI_GPIO_PINS_1 GPIO_PIN_2 | GPIO_PIN_3

#endif

#define VEL_CAP_F 10
#define MAX_POS 0xFFFF

Pose pose;
uint32 odometer; // distance traveled since power-up in mm
int32 xUm;
int32 yUm;
int32 thetaUr;
uint32 odometerUm;

uint16 encoderLeftOld = 0;
uint16 encoderRightOld = 0;
int16 thetaOld;
/*
 * @brief Initializes the encoder.
 *
 * Enables the peripherals.
 * Sets the state of the odometer to 0,0,0.
 * @returns void
 */
void encoderInit(void){
    unsigned long ulPeriod;
    volatile uint32 ulLoop;


  ulPeriod = SYSCTL_CLOCK_FREQ / VEL_CAP_F;
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
  MAP_SysCtlPeripheralEnable(QEI_PERIPH_0);
  MAP_SysCtlPeripheralEnable(QEI_PERIPH_1);
#ifdef PART_LM3S9B92
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  //Will this compile with this plat_printf call from pymite?
  //I (sjb2) don't think so...I'll comment it out just in case.
  //plat_printf("QEI0 and QEI1 enabled using alternate (F0) port.\n");
#endif
 // Wait for port to be enabled.

  for(ulLoop = 0; ulLoop < 10; ulLoop++) {}

  GPIOPinConfigure(QEI_PIN_0_A);
  GPIOPinConfigure(QEI_PIN_0_B);
  GPIOPinConfigure(QEI_PIN_1_A);
  GPIOPinConfigure(QEI_PIN_1_B);

  MAP_GPIOPinTypeQEI(QEI_PORT_BASE_0, QEI_GPIO_PINS_0);//mot0
  MAP_GPIOPinTypeQEI(QEI_PORT_BASE_1, QEI_GPIO_PINS_1);//mot1
#ifdef PART_LM3S9B92
  MAP_GPIOPinTypeQEI(GPIO_PORTF_BASE, GPIO_PIN_0);
#endif


  //configure blocks swap the left
  QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_RESET
		| QEI_CONFIG_SWAP , MAX_POS);
  QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_RESET
		| QEI_CONFIG_NO_SWAP , MAX_POS);


  QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, ulPeriod);
  QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, ulPeriod);

  //enable the QEI blocks
  QEIEnable(QEI0_BASE);
  QEIEnable(QEI1_BASE);

  QEIVelocityEnable(QEI0_BASE);
  QEIVelocityEnable(QEI1_BASE);
  //Reset pose to 0.
  encoderPoseClear();

}

/*
 * @brief Gets the current ticks of the specified encoder.
 *
 * Ticks can then be converted to a measurement of distance.
 * @param enc specifies which encoder's ticks you want
 * @returns the current position of the specified encoder; returns 0 if it's unavailable
 * \internal
 * TODO: change all functions to camelCase
 * \endinternal
 */
int32 encoderGetTicks(uint32 enc){
    if(ENCODER_RIGHT == enc) {
        return QEIPositionGet(QEI1_BASE);
    } else if(ENCODER_LEFT == enc) {
        return QEIPositionGet(QEI0_BASE);
    } else {
        return 0;
    }
}

/*
 * @brief Gets the difference between the two input ticks.
 *
 * @param new is the new encoder position
 * @param old is the old encoder position
 * @returns the difference between old and new position with rollover protection
 */
int32 encoderDeltaTicks(uint32 new, uint32 old){
	int32 diff = 0;

    diff = new - old;

    if (diff > 32768) {
    	//Positive difference more than half
    	// values : 2^16=65536, range of encoders = [0,1,...,65535].  Say we recorded [3,2,1,0,65535], a decrease of 1 each time
    	// diff should be [-1,-1,-1,-1], but we record [-1,-1,-1,65535].
        diff = diff - 65536;
    }
    else if (diff < -32768) {
    	// values : 2^16=65536, range of encoders = [0,1,...,65535].  Say we recorded [65533,65534,65535,0], an increase of 1 each time
    	// diff should be [1, 1, 1], but we record [1,1,-65535].
        diff = diff + 65536;
    }
	return diff;
}

/*
 * @brief Gets the current rotating direction of the encoder.
 *
 * @param enc specifies which encoder (right or left) to look up
 * @returns the current rotating direction
 */
int32 encoderGetDirection(uint32 enc){
    if(ENCODER_RIGHT == enc) {
        return QEIDirectionGet(QEI1_BASE);
    } else if(ENCODER_LEFT == enc) {
        return QEIDirectionGet(QEI0_BASE);
    } else {
        return 0;
    }
}

/*
 * @brief Gets the current velocity of the specified encoder.
 *
 * @param enc specifies which encoder (right or left) to look up
 * @returns the current velocity
 */
int32 encoderGetVelocity(uint32 enc){
    if(ENCODER_RIGHT == enc) {
        return QEIVelocityGet(QEI1_BASE) * encoderGetDirection(enc);
    } else if(ENCODER_LEFT == enc) {
        return QEIVelocityGet(QEI0_BASE) * encoderGetDirection(enc);
    } else {
        return 0;
    }
}

/*
 * @brief Updates the encoder's pose.
 *
 * There is an assumption that this function will be called on a regular basis in
 * to ensure accuracy.
 * Keep updating in micrometers.
 * The encoders have a resolution of 0.0625mm. That is, each tick represents
 * a change of 0.0625 mm. Our pose is stored in micrometers and microradians.
 * Each tick corresponds to 62.5 micrometers. Rounding to 63.
 * @returns void
 */
void encoderPoseUpdate(void) {
	uint16 encoderLeft, encoderRight;
	int32 distLUm, distRUm, distUm;
	int32 deltaThetaUr;
	int32 thetaNew, thetaMean;
	int32 avgDeltaThetaMR;

	// Read the encoders.  Use 16-bit unsigned computation to use ALU to handle roll over.
	// Coerce to signed value to get direction.  Reverse direction of right motor
	// because it rotates backwards
	encoderLeft = encoderGetTicks(ENCODER_LEFT);
    encoderRight = encoderGetTicks(ENCODER_RIGHT);

	distLUm = encoderDeltaTicks(encoderLeft, encoderLeftOld) * MOTOR_MICROMETERS_PER_ENCODER_COUNT;
	distRUm = encoderDeltaTicks(encoderRight, encoderRightOld) * MOTOR_MICROMETERS_PER_ENCODER_COUNT;

	encoderLeftOld  = encoderLeft;
	encoderRightOld = encoderRight;

	// Compute the distance traveled in micrometers.
	distUm = (distRUm + distLUm) / 2;

	// compute the arc angle in microradians
	// the wheel base is 78mm  (version 11).  previous versions used 89
	deltaThetaUr = ((distRUm - distLUm) * 1000) / (MOTOR_WHEEL_BASE_MM);

	// This math is an approximation, but slightly better than the naive version
	// rossum.sourceforge/papers/DiffSteer/#d3
	avgDeltaThetaMR = deltaThetaUr / 2000;
	xUm = xUm + (distUm * cosMilliRad(pose.theta + avgDeltaThetaMR) / MILLIRAD_TRIG_SCALER);
	yUm = yUm + (distUm * sinMilliRad(pose.theta + avgDeltaThetaMR) / MILLIRAD_TRIG_SCALER);

	odometerUm += abs(distUm);

	//need a function that normalizes theta
	thetaUr = normalizeAngleMicroRad(thetaUr + deltaThetaUr);
	pose.theta = thetaUr / 1000;
	pose.x = xUm / 1000;
	pose.y = yUm / 1000;
	odometer = odometerUm / 1000;
}

/*
 * @brief Clears the x, y, theta and odometer values of the encoder.
 *
 * @returns void
 */
void encoderPoseClear(void) {
	pose.x = 0;
	pose.y = 0;
	pose.theta = 0;
	odometer = 0;

	xUm = 0;
	yUm = 0;
	thetaUr = 0;
	odometerUm = 0;
}

void encoderGetPose(Pose* posePtr) {
	posePtr->x = pose.x;
	posePtr->y = pose.y;
	posePtr->theta = pose.theta;
}

int32 encoderGetHeading(void) {
	return pose.theta;
}

void encoderSetPose(Pose* posePtr) {
	pose.x = posePtr->x;
	pose.y = posePtr->y;
	pose.theta = posePtr->theta;

	xUm = pose.x * 1000;
	yUm = pose.y * 1000;
	thetaUr = pose.theta * 1000;
}


uint32 encoderGetOdometer(void) {
	return odometer;
}
