/*
 * @file intMath.c
 * @brief fast integer math (no floating point processor on rone)
 * @since Apr 2, 2012
 * @author jamesm
 */
//Note: this file is compiled at O0 to prevent odd errors at gaussianNoise().

#include <math.h>
#include <stdlib.h>

#include "roneos.h"

#include "intMathTrigLookup.inc"

#define MILLIRAD_BYTE_CONV	25


Pose poseOrigin = {0,0,0};

/*
 * @brief Continually decrements the input value by one until it is closest to zero.
 *
 * @param val the value to be decremented
 * @returns the decremented value (within the range of 0 <= val < 1)
 */
uint32 decToZero(uint32 val) {
	if (val > 0) {
		val--;
	}
	return(val);
}

/*
 * @brief Average two values.
 *
 * @param val1 is the first value
 * @param val2 is the second value
 * @returns the average of val1 and val2
 */
int32 average(int32 val1, int32 val2) {
	return ((val1 + val2) / 2);
}

/*
 * @brief Circularly increments the index by 1.
 *
 * Increments the index by 1. Resets index to 0 if it exceeds the maximum index
 * Circular meaning it goes back to 0.
 * @param index is the index to be incremented
 * @param maxIndex is the maximum index
 * @returns the incremented circular index
 */
uint32 circularInc(uint32 index, uint32 maxIndex) {
	index++;
	if (index > maxIndex) {
		index = 0;
	}
	return index;
}

/*
 * @brief Circularly decrements the index by 1.
 *
 * Decrements the index by 1. If the index reaches 0, resets it to maximum index.
 * Circular meaning it goes back to maximum index.
 * @param index the index to be decremented
 * @param maxIndex the maximum index
 * @returns the decremented circular index
 */
uint32 circularDec(uint32 index, uint32 maxIndex) {
	if (index == 0) {
		index = maxIndex;
	}
	else {
		index--;
	}
	return index;
}

/*
 *	@brief Computes the magnitude of the input vector.
 *
 *	@param x the x component of the vector
 *	@param y the y component of the vector
 *	@returns the magnitude of the vector with components x and y
 */
int32 vectorMag(int32 x, int32 y) {
	return sqrtInt(x*x + y*y);
}

/*
 *	@brief Compute the integer square root of a number.
 *
 *	Based on Microchip app note TB040.
 *	Can't take the root of numbers higher than MAX_INT32.
 *	@param val is the number to be computed
 *	@returns the computed integer square root
 */
uint32 sqrtInt(uint32 val) {
	uint32 g;
	uint32 bit = 0x8000;
	uint32 result = 0;
	uint32 test;


	// can't take the root of numbers higher than MAX_INT32
	if (val >= 0x80000000) {
		result = 46341;
	}
	else {
		while (bit != 0) {
			g = result | bit;
			test = g * g;
			if (test == val) {
				// we're done!
				result = g;
				break;
			}else if (test < val) {
				// our guess is too small, keep the temp bit
				result |= bit;
			}
			// shift the bit down one
			bit >>= 1;
		}
	}
	return result;
}

/*
 * @brief Pack a 32-bit dataWord into 8-bit, pointed to by char pointer arrayPtr.
 *
 * @param arrayPtr points to the packed 8-bit dataWord
 * @param dataWord 32-bit data to be packed
 * @returns void
 */
void pack32(uint8 * arrayPtr, uint32 dataWord) {
	*arrayPtr++ = (uint8)((dataWord >> 24) & 0xFF);
	*arrayPtr++ = (uint8)((dataWord >> 16) & 0xFF);
	*arrayPtr++ = (uint8)((dataWord >> 8) & 0xFF);
	*arrayPtr++ = (uint8)((dataWord) & 0xFF);
}


/*
 * @brief Pack a 24-bit dataWord into 8-bit, pointed to by char pointer arrayPtr.
 *
 * @param arrayPtr points to the packed 8-bit dataWord
 * @param dataWord 24-bit data to be packed
 * @returns void
 */
void pack24(uint8 * arrayPtr, uint32 dataWord) {
	*arrayPtr++ = (uint8)((dataWord >> 16) & 0xFF);
	*arrayPtr++ = (uint8)((dataWord >> 8) & 0xFF);
	*arrayPtr++ = (uint8)((dataWord) & 0xFF);
}


/*
 * @brief Pack a 16-bit dataWord into 8-bit, pointed to by pointer arrayPtr.
 *
 * @param arrayPtr points to the packed 8-bit dataWord
 * @param dataWord 16-bit data to be packed
 * @returns void
 */
void pack16(uint8 * arrayPtr, uint32 dataWord) {
	*arrayPtr++ = (uint8)((dataWord >> 8) & 0xFF);
	*arrayPtr++ = (uint8)((dataWord) & 0xFF);
}


/*
 * @brief Unpacks an 8-bit data into 16-bit.
 *
 * @param arrayPtr points to data with 8-bit wordlength
 * @returns unpacked input data with 16-bit wordlength
 */
uint16 unpack16(uint8 * arrayPtr) {
	uint16 dataWord = (((uint32)arrayPtr[0]) << 8) | (((uint32)arrayPtr[1]));
	return(dataWord);
}


/*
 * @brief Unpacks an 8-bit data into 24-bit.
 *
 * @param arrayPtr points to data with 8-bit wordlength
 * @returns unpacked input data with 24-bit wordlength
 */
uint32 unpack24(uint8 * arrayPtr) {
	uint32 dataWord = unpack16(&arrayPtr[1]);
	dataWord |= (((uint32)arrayPtr[0]) << 16);
	return(dataWord);
}


/*
 * @brief Unpacks an 8-bit data into 32 bit.  Implemented in a pedantic
 * way to avoid assumptions of endianness.
 *
 * @param arrayPtr points to data with 8-bit wordlength
 * @returns unpacked input data with 32-bit wordlength
 */
uint32 unpack32(uint8 * arrayPtr) {
	uint32 dataWord = unpack24(&arrayPtr[1]);
	dataWord |= (((uint32)arrayPtr[0]) << 24);
	return(dataWord);
}

/*
 * @brief Normalizes the angle. (Depricated)
 *
 * Normalizes the angle to make it stay in the range of 0 <= angle < millirad_2PI
 * @param angle the angle to be normalized
 * @returns the normalized angle
 */
int16 normalizeAngleMilliRad(int16 angle) {
	while (angle < 0) {
		angle += MILLIRAD_2PI;
	}
	while (angle >= MILLIRAD_2PI) {
		angle -= MILLIRAD_2PI;
	}
	return( (uint16) angle);
}

/*
 * @brief  Normalizes the angle.
 *
 * Normalizes the angle to make it stay in the range of -millirad_PI < angle <= millirad_PI.
 * @param angle the angle to be normalized
 * @returns the normalized angle in milli-radians
 */
int16 normalizeAngleMilliRad2(int16 angle) {
	while (angle <= -MILLIRAD_PI) {
		angle += MILLIRAD_2PI;
	}

	while (angle > MILLIRAD_PI) {
		angle -= MILLIRAD_2PI;
	}
	return angle;
}

// Is this useful or should it be deleted?
///*
// * @brief  Normalizes the angle.
// *
// * Normalizes the angle to make it stay in the range of 0 <= angle < millirad_PI.
// * @param angle the angle to be normalized
// * @returns the normalized angle
// */
//int16 normalizeAngleMilliRad3(int16 angle) {
//	while (angle < 0) {
//		angle = MILLIRAD_2PI + angle;
//	}
//	while (angle > MILLIRAD_PI) {
//		angle = MILLIRAD_2PI - angle;
//	}
//	return angle;
//}

/*
 * @brief Normalizes the angle.
 *
 * Normalizes the angle to make it stay in the range of 0 <= angle < microrad_2PI.
 * @param angle the angle to be normalized
 * @returns the normalized angle in micro-radians
 */
int32 normalizeAngleMicroRad(int32 angle) {
	while (angle < 0) {
		angle += (MILLIRAD_2PI * 1000);
	}

	while (angle >= (MILLIRAD_2PI * 1000)) {
		angle -= (MILLIRAD_2PI * 1000);
	}
	return(angle);
}

/*
 *	@brief Finds the min of 3 input values.
 *
 *	@param	valueA first int32 value
 *	@param	valueB second int32 value
 *	@param	valueC third int32 value
 *	@returns minimum of the 3 input values
 */
int32 min3(int32 valueA, int32 valueB, int32 valueC){
	int32 res = valueA;
	if (valueB < res){
		res = valueB;
	}

	if (valueC < res){
		res = valueC;
	}

	return res;
}

/*
 * @brief Interprets the angle as milli-radian of sine.
 *
 * @param angle the angle to be interpreted
 * @returns if angle is greater than pi/4, angle as milli-radian of sine. else, 0.
 */
int16 sinMilliRad(int16 angle) {
	int16 angleTemp = normalizeAngleMilliRad(angle);
	int16 sign = 1;

	if (angleTemp < MILLIRAD_HALF_PI) {
		/* 0 <= angle < pi/2 */
		//angleTemp = angleTemp;
	}
	else if (angleTemp < MILLIRAD_PI) {
		/* pi/2 <= angle < pi */
		angleTemp = MILLIRAD_PI - angleTemp;
	}
	else if (angleTemp < (MILLIRAD_PI + MILLIRAD_HALF_PI)) {
		/* pi <= angle < 3pi/2 */
		angleTemp = angleTemp - MILLIRAD_PI;
		sign = -1;
	}
	else {
		/* 3pi/2 <= angle < 2pi */
		angleTemp = MILLIRAD_2PI - angleTemp;
		sign = -1;
	}

	angleTemp = angleTemp >> 1;
	if (angleTemp > 786) {
		return(0);
	}
	else {
	 	return(sign * trigMilliRadLookup[angleTemp]);
	}
}

/*
 * Interprets the angle as milli-randian of cosine.
 *
 * @param angle the angle to be interpreted
 * @returns angle as milli-radian of cosine
 */
int16 cosMilliRad(int16 angle) {
 	return sinMilliRad(angle + MILLIRAD_HALF_PI);
}

/*
 * @brief Calculates the smallest angle difference between the two input angles.
 *
 * The difference will be within the range of -MILLIRAD_PI <- difference <= MILLIRAD_PI.
 * @param thetaGoal is first angle
 * @param theta is second angle
 * @returns the difference between thetaGoal and theta
 */
int16 smallestAngleDifference(int16 thetaGoal, int16 theta) {
	int16 thetaDelta;

	if (thetaGoal > theta) {
		// case 1
		thetaDelta = thetaGoal - theta;
		if (thetaDelta > MILLIRAD_PI) {
			// case 2
			thetaDelta -= MILLIRAD_2PI;
		}
	}
	else {
		// thetaGoal <= theta
		// case 3
		thetaDelta = thetaGoal - theta;
		if (thetaDelta < -MILLIRAD_PI) {
			// case 4
			thetaDelta += MILLIRAD_2PI;
		}
	}
	return(thetaDelta);
}


/*
 * @brief Calculates the smallestAngleDifference between two poses.
 *
 * @param poseGoalPtr pointer to goal pose
 * @param posePtr pointer to a pose
 * @returns smallest angle difference between two poses
 */
int32 poseAngleDiff(Pose* poseGoalPtr, Pose* posePtr) {
	return smallestAngleDifference(poseGoalPtr->theta, posePtr->theta);
}


/*
 * @brief Adds two Poses and places result in a Pose.
 *
 * @param poseResPtr pointer to Pose to hold result
 * @param pose1Ptr pointer to first Pose
 * @param pose2Ptr pointer to second Pose
 * @returns void
 */
void poseAdd(Pose* poseResPtr, Pose* pose1Ptr, Pose* pose2Ptr) {
	poseResPtr->x = pose1Ptr->x + pose2Ptr->x;
	poseResPtr->y = pose1Ptr->y + pose2Ptr->y;
	poseResPtr->theta = normalizeAngleMilliRad(pose1Ptr->theta + pose2Ptr->theta);
}

/*
 * @brief Calculates distance between two poses.
 *
 * @param pose1Ptr pointer to first Pose
 * @param pose2Ptr pointer to second Pose
 * @returns distance
 */
int32 poseDistance(Pose* pose1Ptr, Pose* pose2Ptr) {
	int32 x = pose1Ptr->x - pose2Ptr->x;
	int32 y = pose1Ptr->y - pose2Ptr->y;
	return sqrtInt(x * x + y * y);
}


/*
 * @brief Bounds the value with one specified bound as both lower and upper bound.
 *
 * Bounds the input value so that it stays within the range of -bound <= value <= bound.
 * If it exceeds the bound, set it to the bound.
 * @param val is the value to be bounded
 * @param bound is the lower and upper bound
 * @returns the bounded value
 */
int32 boundAbs(int32 val, int32 bound) {
	if (val > bound) {
		val = bound;
	}
	else if (val < -bound){
		val = -bound;
	}
	return(val);
}


/*
 * @brief Finds the min of the two arguments.
 *
 * Finds the min of two arguments.
 * @param x, y is the value to be compared
 * @returns the min value
 */
int32 min(int32 x, int32 y) {
	if (x > y) {
		return y;
	}
	return x;
}

/*
 * @brief Finds the min of the two arguments.
 *
 * Finds the min of two arguments.
 * @param x, y is the value to be compared
 * @returns the min value
 */
int32 max(int32 x, int32 y) {
	if (x > y) {
		return x;
	}
	return y;
}


/*
 * @brief Bounds the value with specified lower and upper bound.
 *
 * Bounds the value so that it stays within the range of lowerBound <= value <= upperBound.
 * If it exceeds the bound, set it to the lower/upper bound.
 * @param val is the value to be bounded
 * @param lowerBound is the lower bound
 * @param upperBound is the upper bound
 * @returns the bounded value
 */
int32 bound(int32 val, int32 lowerBound, int32 upperBound) {
	if (val < lowerBound) {
		val = lowerBound;
	}
	else if (val > upperBound) {
		val = upperBound;
	}
	return val;
}

/*
 * @brief Counts how many bits the value has.
 *
 * Ignores leading zeros.
 * @param val is the value to be counted
 * @returns the number of bits of the input value
 */
uint8 bitsCount(uint32 val) {
	uint32 bitMask;
	uint8 returnVal = 0;

	for (bitMask = 1; bitMask != 0; bitMask = bitMask << 1) {
		if (val & bitMask) {
			returnVal++;
		}
	}
	return(returnVal);
}




/*
 * @brief Runs a simple IIR on the given variable.
 *
 * @param currentVal is the current value to be filtered
 * @param newVal is the new value
 * @param alpha is the ratio of new/old given in units of x/100. 10 would be a slow filter, 90 a fast one
 * @returns the IIR response of these values
 */
//TODO check for overflow
int32 filterIIR(int32 currentVal, int32 newVal, int32 alpha) {
	int32 temp = (newVal * alpha) + ((100 - alpha) * currentVal);
	return temp / 100;
}


/*
 * @brief Calculates an IIR of two angles in milliradians.
 *
 * @param currentVal is the first angle to be averaged
 * @param newVal is the second angle to be averaged
 * @param alpha is the ratio of new/old given in units of x/100. 10 would be a slow filter, 90 a fast one
 * @returns the IIR response of these values
 */
int16 filterIIRAngle(int16 currentVal, int16 newVal, int32 alpha) {
	int32 y, x, average;

	x = ((int32)cosMilliRad(newVal) * alpha) + ((100 - alpha) * (int32)cosMilliRad(currentVal));
	y = ((int32)sinMilliRad(newVal) * alpha) + ((100 - alpha) * (int32)sinMilliRad(currentVal));
	average = atan2MilliRad(y, x);
	return(normalizeAngleMilliRad2(average));
}



/*
 * 	@brief Gets atan2 approximation in miiliradians.
 *
 *  Originally developed by John Aspinal at iRobot. It is quite good.
 *  @param y y-coordinate of the point to be calculated
 *  @param x x-coordinate of the point to be calculated
 *  @returns atan2 approximation of the input point, specified by (x,y) coordinate
 */
int16 atan2MilliRad(int32 y, int32 x) {
	boolean x_fold = FALSE;
	boolean y_fold = FALSE;
	boolean deg_45_fold = FALSE;
	uint32 val, denom, ux, uy;

	/* argument folding */
	if (x < 0) {
		x_fold = TRUE;
		x = -x;
	}
	if (y < 0) {
		y_fold = TRUE;
		y = -y;
	}
	if (y > x) {
		int32 tmp;
		deg_45_fold = TRUE;
		tmp = x;
		x = y;
		y = tmp;
	}

	uy = (uint32)y;
	ux = (uint32)x;

    /* atan2 approximation for 0 <= y/x <= 1
        basic formula is
        (60937 * y * x) /
        ((61 * x * x) + (17 * y * y))
      we check the size of ux to know how much to truncate args so we don't overflow
	  ux and uy must be 8-bit numbers to prevent overflow
      x > y so we only need to check x
    */
	while (ux & ~0x000000FF) {
		ux = ux >> 1;
		uy = uy >> 1;
	}
	val = (60937L * uy * ux);
	denom = (61L * ux * ux) + (17L * uy * uy);

	if (denom != 0) {
		val = val / denom;
		/* argument unfolding */
		if (deg_45_fold) {
		 	val = MILLIRAD_DEG_90 - val;
		}
		if (x_fold) {
		 	val = MILLIRAD_PI - val;
		}
		if (y_fold && (val > 0)) {
		 	val = MILLIRAD_2PI - val;
		}
	}
	else {
		// denom = 0 iff x = y = 0, but then function is undefined.  Return 0.
		val = 0;
	}

	return((int16)val);
}

/*
 * @brief Calculates the average of the two angles in micro-radians.
 *
 * @param theta1 is the first angle to be averaged
 * @param theta2 is the second angle to be averaged
 * @returns the average angle
 */
int32 averageAnglesMicroRad(int32 theta1, int32 theta2) {
	int32 thetaDelta;
    int32 mean;

	if (theta2 > theta1) {
		/* case 1 */
		thetaDelta = theta2 - theta1;
		if (thetaDelta > (MILLIRAD_PI * 1000)) {
			/* case 2 */
			thetaDelta -= (MILLIRAD_2PI * 1000);
		}
	}
	else {
		/* theta2 < theta1 */
		/* case 3 */
		thetaDelta = theta2 - theta1;
		if (thetaDelta < -(MILLIRAD_PI * 1000)) {
			/* case 4 */
			thetaDelta += (MILLIRAD_2PI * 1000);
		}
	}
    mean = theta1 + thetaDelta / 2;
	return(normalizeAngleMicroRad(mean));
}


/*
 * @brief Calculates the average of the two angles in milli-radians.
 *
 * @param angle1 is the first angle to be averaged
 * @param angle2 is the second angle to be averaged
 * @returns the average angle
 */
int16 averageAngles(int16 angle1, int16 angle2) {
	int32 y, x, average;

	x = cosMilliRad(angle1) + cosMilliRad(angle2);
	y = sinMilliRad(angle1) + sinMilliRad(angle2);
	average = atan2MilliRad(y, x);
	return(normalizeAngleMilliRad2(average));
}


/*
 * @brief Calculates the average of the two angles in millirad.
 *
 * @param angleLeft is the first angle to be averaged
 * @param angleRight is the second angle to be averaged
 * @returns the average angle
 */
int16 averageAnglesLeftToRight(int16 angleLeft, int16 angleRight) {
	int16 average = averageAngles(angleLeft, angleRight);

	// Determine right normal vector direction
	if (normalizeAngleMilliRad2(angleLeft) * normalizeAngleMilliRad2(angleRight) > 0) {
		if (normalizeAngleMilliRad2(angleRight) > normalizeAngleMilliRad2(angleLeft)){
			average += MILLIRAD_PI;
		}
	} else {
		if (normalizeAngleMilliRad2(angleLeft) > 0){
			if (normalizeAngleMilliRad2(angleLeft) - normalizeAngleMilliRad2(angleRight) > MILLIRAD_PI){
				average += MILLIRAD_PI;
			}
		} else {
			if (normalizeAngleMilliRad2(angleLeft) - normalizeAngleMilliRad2(angleRight) < MILLIRAD_PI){
				average += MILLIRAD_PI;
			}
		}
	}
	return average;
}

/*
 * @brief Calculates the average of the angles in the array.
 *
 * Calculates the average of the first "size (a number)" of angles in angleArray.
 * @param angleArray[] is the array of angles to be averaged
 * @param size specifies how many elements in the array (starting from the first) should be averaged
 * @returns the average of the angles in the array (returns 0 if given a nonpositive size)
 */
int16 averageArrayAngle(int16 angleArray[], int32 size) {
	int32 ysum = 0;
	int32 xsum = 0;
	int32 average, i;

	if (size > 0) {
		for (i = 0; i < size; i++) {
			ysum += sinMilliRad(angleArray[i]);
			xsum += cosMilliRad(angleArray[i]);
		}
		average = atan2MilliRad(ysum, xsum);
	}
	else {
		average = 0;
	}
	return(normalizeAngleMilliRad(average));
}


/*
 * @brief unit conversion (16bit to 8bit)
 *
 *
 * @param angle the angle as a Byte
 * @returns int16 angle in milli-radians
 */
int16 byteToMillirad(int8 angle) {
    return (int16)angle * MILLIRAD_BYTE_CONV;
}


/*
 * @brief unit conversion (16bit to 8bit)
 *
 *
 * @param angle
 * @returns int8 the angle as a Byte
 */
int8 milliradToByte(int16 angle) {
	return (int8)(angle / MILLIRAD_BYTE_CONV);
}


/*
 * @brief unit conversion (16bit to 8bit)
 *
 *
 * @param angle the angle as a Byte
 * @returns int16 angle in milli-radians
 */
int16 byteToMilliradUnsigned(uint8 angle) {
    return (int16)angle * MILLIRAD_BYTE_CONV;
}


/*
 * @brief unit conversion (16bit to 8bit)
 *
 *
 * @param angle
 * @returns int8 the angle as a Byte
 */
uint8 milliradToByteUnsigned(int16 angle) {
	angle = normalizeAngleMilliRad(angle);
	return (uint8)(angle / MILLIRAD_BYTE_CONV);
}



//TODO: check for use elsewhere
//void posePrint(Pose* posePtr, char* string) {
//	sprintf(string, "{%d,%d,%d}", posePtr->x, posePtr->y, posePtr->theta);
//}



/*
 * @brief Converts string of bytes into a long int.
 *
 * Converts a string of 'len' bytes, MSB first, into a long int. This is
 * not to be confused with 'atoi()': 'atoi()' converts "abcdef12" into
 * '0xabcdef12L', whereas 'stol()' converts '0xab,0xcd,0xef,0x12' into
 * '0xabcdef12L'.
 *
 * @param str a string
 * @param len the number of bytes in str
 * @returns str converted into a long it
 */
uint32 stol(uint8 * str, int len) {
	int i;
	long retval;

	retval = 0L;
	for (i = 0; i < len; i++) {
		retval <<= 8;
		retval |= str[i];
	}

	return (retval);
}


static const int16 ir_orientation_cos[IR_COMMS_NUM_OF_TRANSMITTERS] = {1000,707,0,-707, -1000, -707, 0, 707};
static const int16 ir_orientation_sin[IR_COMMS_NUM_OF_TRANSMITTERS] = {0,707,1000,707,0,-707,-1000,-707};

/*
 * @brief Calculates the resultant angle from the bit vector.  This assumes that bit0 = 0 rad
 *
 * @param bitVector is the vector of bits
 * @returns the average angle
 */
int16 angleFromBitVector(uint8 bitVector) {
	uint8 i;
	int32 x = 0;
	int32 y = 0;
	for (i = 0; i < 8; i++) {
		if (bitVector & 1) {
			x += ir_orientation_cos[i];
			y += ir_orientation_sin[i];
		}
		bitVector >>= 1;
	}
	return normalizeAngleMilliRad2(atan2MilliRad(y, x));
}


//static const int16 ir_orientation_beacon_cos[IR_COMMS_NUM_OF_TRANSMITTERS] = {995,957,882,773,634,471,290,98};
//static const int16 ir_orientation_beacon_sin[IR_COMMS_NUM_OF_TRANSMITTERS] = {98,290,471,634,773,882,957,995};
static const int16 ir_orientation_beacon_cos[IR_COMMS_NUM_OF_TRANSMITTERS] = {1000,975,901,782,623,434,223,0};
static const int16 ir_orientation_beacon_sin[IR_COMMS_NUM_OF_TRANSMITTERS] = {0,223,434,623,782,901,975,1000};

//TODO: Update comment to talk about beacons (how is this different from angle FromBItVector)
/*
 * @brief Calculates the resultant angle from the bit vector.  This assumes that bit0 = 0 rad
 *
 * @param bitVector is the vector of bits
 * @returns the average angle
 */
int16 angleFromBitVectorBeacon(uint8 bitVector) {
	uint8 i;
	int32 x = 0;
	int32 y = 0;
	for (i = 0; i < 8; i++) {
		if (bitVector & 1) {
			x += ir_orientation_beacon_cos[i];
			y += ir_orientation_beacon_sin[i];
		}
		bitVector >>= 1;
	}
	return normalizeAngleMilliRad2(atan2MilliRad(y, x));
}



static const int16 ir_receiver_cos[IR_COMMS_NUM_OF_RECEIVERS] = {924,383,-383,-924,-924,-383,383,924};
static const int16 ir_receiver_sin[IR_COMMS_NUM_OF_RECEIVERS] = {383,924,924,383,-383,-924,-924,-383};

/*
 * @brief Calculates the resultant angle from the bit vector.
 * This assumes that bit0 = 383 rad
 *
 * @param bitVector is the vector of bits
 * @returns the average angle
 */
int16 angleFromBitVectorOffset(uint8 bitVector) {
	uint8 i;
	int32 x = 0;
	int32 y = 0;
	for (i = 0; i < 8; i++) {
		if (bitVector & 1) {
			x += ir_receiver_cos[i];
			y += ir_receiver_sin[i];
		}
		bitVector >>= 1;
	}
	return normalizeAngleMilliRad2(atan2MilliRad(y, x));
}


// This function is to return the number of bits that reflect against an obstacle.
// The number of bits increases as the robot gets closer to the obstacle.

/*
 * @brief Determines the number of bits that reflect against an obstacle.
 *
 * The number of bits increases as the robot gets closer to the obstacle.
 * @param val a uint8 value
 * @returns the number of bits that reflect against an obstacle
 */
uint8 bitsMaxContiguous(uint8 val) {
	uint8 bitMask;
	uint8 bitCount = 0;
	uint8 bitCountMax = 0;
	uint8 bits = 0;
	uint8 bitsMax = 0;

	// special case 0xFF
	if (val == 0xFF) {
		return 0xFF;
	}

	// we have a gap somewhere.
	// first see if there is wraparound, then count the wrap bits from the left
	if ((val & 0x81) == 0x81) {
		// first count from left
		for (bitMask = 0x80; bitMask != 0; bitMask = bitMask >> 1) {
			if (val & bitMask) {
				bits |= bitMask;
				bitCount++;
			} else {
				// found a space. now count from left
				break;
			}
		}
	}

	for (bitMask = 1; bitMask != 0; bitMask = bitMask << 1) {
		if (val & bitMask) {
			bits |= bitMask;
			bitCount++;
		} else {
			// found a space.
			bitCount = 0;
			bits = 0;
		}
		if (bitCount > bitCountMax) {
			bitCountMax = bitCount;
			bitsMax = bits;
		}
	}
	return(bitsMax);
}



const uint8 gaussianLookup[GAUSSIAN_LOOKUP_SIZE] = {
255,
255,
254,
252,
250,
247,
244,
240,
235,
230,
225,
219,
213,
206,
200,
192,
185,
178,
170,
162,
155,
147,
139,
132,
124,
117,
110,
103,
96,
89,
83,
77,
71,
65,
60,
55,
50,
46,
42,
38,
35,
31,
28,
25,
23,
20,
18,
16,
14,
13,
11,
10,
9,
8,
7,
6,
5,
4,
4,
3,
3,
2,
2,
2
};

const uint16 gaussianSumLookup[GAUSSIAN_LOOKUP_SIZE] = {
255,
510,
763,
1016,
1266,
1513,
1756,
1996,
2232,
2462,
2687,
2906,
3119,
3326,
3525,
3718,
3903,
4081,
4251,
4413,
4568,
4715,
4854,
4986,
5110,
5227,
5336,
5439,
5534,
5623,
5706,
5783,
5854,
5919,
5979,
6034,
6085,
6131,
6173,
6211,
6246,
6277,
6305,
6330,
6353,
6373,
6391,
6407,
6422,
6434,
6446,
6455,
6464,
6472,
6478,
6484,
6489,
6494,
6497,
6501,
6504,
6506,
6508,
6510
};

/*
 * @brief Calculates gaussian noise.
 *
 * Optimization errors fixed by setting roneos optimization to 0!!!!!!!
 *
 * @returns gaussian noise +/-GAUSSIAN_LOOKUP_MAX_RETURN centered at 0.
 */
int16 gaussianNoise() {
	volatile uint8 i2, odd;
	volatile uint16 myRand;
	volatile int16 val=0;

	myRand = rand() % gaussianSumLookup[GAUSSIAN_LOOKUP_SIZE-1];
	odd = myRand & 1;

	for (i2=0; i2 < GAUSSIAN_LOOKUP_SIZE; i2++){
		if(gaussianSumLookup[i2]>=myRand){
			if(odd==1){
				val = (-(GAUSSIAN_LOOKUP_MAX_RETURN-gaussianLookup[i2]));
				break;
			}else{
				val = (GAUSSIAN_LOOKUP_MAX_RETURN-gaussianLookup[i2]);
				break;
			}
		}
	}
	return val;
}

#if 0
/*
 * overflowCheck...
 * ----------------
 * Overflow check functions for integer calculations.
 */
uint8 msb8fast(uint8 val) {
	if (val & b11110000) {
		//1111----
		if (val & b11000000) {
			//11------
			if (val & b10000000) {
				//1-------
				return(7);
			}
			else {
				//01------
				return(6);
			}
		}
		else {
			//0011----
			if (val & b00100000) {
				//1-------
				return(5);
			}
			else {
				//01------
				return(4);
			}
		}
	}
	else {
		//0000----
		if (val & b00001100) {
			//000011--
			if (val & b00001000) {
				//1-------
				return(3);
			}
			else {
				//000001--
				return(2);
			}
		}
		else {
			//0011----
			if (val & b00000010) {
				//0000001-
				return(1);
			}
			else {
				//00000001
				return(0);
			}
		}
	}
}


uint8 msb32fast(uint32 val) {
	if (val & 0xFFFF0000) {
		//1111----
		if (val & 0xFF000000) {
			//11------
			return(msb8fast((uint8)(val >> 24)) + 24);
		}
		else {
			//0011----
			return(msb8fast((uint8)(val >> 16)) + 16);
		}
	}
	else {
		//0000----
		if (val & 0x0000FF00) {
			//000011--
			return(msb8fast((uint8)(val >> 8)) + 8);
		}
		else {
			//00000011
			return(msb8fast((uint8)val));
		}
	}
}


uint8 msb32(uint32 val) {
	uint32 bitMask;
	uint8 returnVal = 31;
	for (bitMask = 0x80000000; bitMask != 0; bitMask = bitMask >> 1) {
		if (val & bitMask) {
			break;
		}
		returnVal--;
	}
	return(returnVal);
}


int32 add32func(int32 v1, int32 v2, const char * filePathName, unsigned int lineNum) {
  uint8 p1 = msb32(abs(v1)) + 1;
  uint8 p2 = msb32(abs(v2)) + 1;
  uint8 p = max(p1, p2);
  if ((p + 1) > 31) {
    cprintf(COLOR_ERROR "add32 overflow v1=%d, v2=%d in %s at line %d\n", v1, v2, sysGetFilenameFromPath((char *)filePathName), lineNum);
  	return(0);
  }
  else {
  	return(v1 + v2);
  }
}


int32 sub32func(int32 v1, int32 v2, const char * filePathName, unsigned int lineNum) {
  return(add32func(v1, -v2, filePathName, lineNum));
}


int32 mul32func(int32 v1, int32 v2, const char * filePathName, unsigned int lineNum) {
  uint8 p1 = msb32(abs(v1) + 1);
  uint8 p2 = msb32(abs(v2) + 1);
  if ((p1 + p2) > 31) {
    cprintf(COLOR_ERROR "mul32 overflow v1=%d, v2=%d in %s at line %d\n", v1, v2, sysGetFilenameFromPath((char *)filePathName), lineNum);
	  return(0);
  }
  else {
  	return(v1 * v2);
  }
}



int32 div32func(int32 v1, int32 v2, const char * filePathName, unsigned int lineNum) {
  if (v1 == 0) {
	return(0);
  }

  if (abs(v2) > abs(v1)) {
    // check for underflow
    cprintf(COLOR_ERROR "div32 underflow v1=%d, v2=%d in %s at line %d\n", v1, v2, sysGetFilenameFromPath((char *)filePathName), lineNum);
    return(0);
  }
  else {
//    // check for loss of precision
//    uint8 p1 = msb32(abs(v1));
//    uint8 p2 = msb32(abs(v2));
//    if ((p1 - p2) < 4) {
//      cprintf(COLOR_ERROR "div32 precision loss v1=%d, v2=%d in %s at line %d\n", v1, v2, getFilename((char *)filePathName), lineNum);
//  	  return(0);
//    }
//    else {
//  	  return(v1 / v2);
//    }
	  return(v1 / v2);
  }
}


uint8 msb64(uint64 val) {
	uint64 bitMask;
	uint8 returnVal = 63;
	for (bitMask = 0x8000000000000000LL; bitMask != 0; bitMask = bitMask >> 1) {
		if (val & bitMask) {
			break;
		}
		returnVal--;
	}
	return(returnVal);
}


int64 abs64(int64 val) {
	if (val < 0) {
		return -val;
	}
	else {
		return val;
	}
}


int64 add64func(int64 v1, int64 v2, const char * filePathName, unsigned int lineNum) {
  uint8 p1 = msb64(abs(v1)) + 1;
  uint8 p2 = msb64(abs(v2)) + 1;
  uint8 p = max(p1, p2);
  if ((p + 1) > 63) {
	char temp[200];
	sprintf(temp, COLOR_ERROR "add64 overflow v1=%lld, v2=%lld in %s at line %d\n", v1, v2, sysGetFilenameFromPath((char *)filePathName), lineNum);
    cprintf(temp);
  	return(0);
  }
  else {
  	return(v1 + v2);
  }
}


int64 sub64func(int64 v1, int64 v2, const char * filePathName, unsigned int lineNum) {
  return(add64func(v1, -v2, filePathName, lineNum));
}


int64 mul64func(int64 v1, int64 v2, const char * filePathName, unsigned int lineNum) {
  uint8 p1 = msb64(abs64(v1) + 1);
  uint8 p2 = msb64(abs64(v2) + 1);
  if ((p1 + p2) > 63) {
	char temp[200];
	sprintf(temp, COLOR_ERROR "mul64 overflow v1=%lld, v2=%lld in %s at line %d\n", v1, v2, sysGetFilenameFromPath((char *)filePathName), lineNum);
    cprintf(temp);
    return(0);
  }
  else {
  	return(v1 * v2);
  }
}



int64 div64func(int64 v1, int64 v2, const char * filePathName, unsigned int lineNum) {
  if (v1 == 0) {
	  return(0);
  }

  if (abs64(v2) > abs64(v1)) {
    // check for underflow
	char temp[200];
	sprintf(temp, COLOR_ERROR "div64 underflow v1=%lld, v2=%lld in %s at line %d\n", v1, v2, sysGetFilenameFromPath((char *)filePathName), lineNum);
    cprintf(temp);
    return(0);
  }
  else {
//    // check for loss of precision
//    uint8 p1 = msb64(abs(v1));
//    uint8 p2 = msb64(abs(v2));
//    if ((p1 - p2) < 4) {
//      cprintf(COLOR_ERROR "div64 precision loss v1=%lld, v2=%lld in %s at line %d\n", v1, v2, getFilename((char *)filePathName), lineNum);
//  	  return(0);
//    }
//    else {
//  	  return(v1 / v2);
//    }
	  return(v1 / v2);
  }
}
#endif
