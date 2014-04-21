/**
 * @file intMath.h
 * @brief fast integer math (no floating point processor on rone)
 * @since Apr 2, 2012
 * @author James McLurkin
 *  */

#ifndef intMath_h
#define intMath_h


/******** Defines ********/
#define __PI 3.14159

/* Angles */
#define MILLIRAD_HALF_PI			1571
#define MILLIRAD_PI					(MILLIRAD_HALF_PI * 2)
#define MILLIRAD_2PI				(MILLIRAD_HALF_PI * 4)

#define MILLIRAD_DEG_0				0
#define MILLIRAD_DEG_15				(MILLIRAD_HALF_PI / 6)
#define MILLIRAD_DEG_20				(MILLIRAD_PI / 9)
#define MILLIRAD_DEG_30				(MILLIRAD_HALF_PI / 3)
#define MILLIRAD_DEG_45				(MILLIRAD_HALF_PI / 2)
#define MILLIRAD_DEG_60				(MILLIRAD_PI / 3)
#define MILLIRAD_DEG_90				MILLIRAD_HALF_PI
#define MILLIRAD_DEG_180			MILLIRAD_PI
#define MILLIRAD_DEG_270			(MILLIRAD_PI + MILLIRAD_HALF_PI)
#define MILLIRAD_DEG_360			MILLIRAD_2PI

#define MILLIRAD_TRIG_SCALER		32767

#define GAUSSIAN_LOOKUP_SIZE 64
#define GAUSSIAN_LOOKUP_MAX_RETURN 255


/******** Structs ********/

/**
 * @brief The pose of a robot, it's position and orientation
 */
typedef struct Pose {
	int32 x; 		/**< typically in milli-meters */
	int32 y; 		/**< typically in milli-meters */
	int32 theta;	/**< typically in milli-radians */
} Pose;

extern Pose poseOrigin;



/******** Macros ********/


/******** Functions ********/

//General math

/**
 *	@brief Compute the integer square root of a number.
 *
 *	Based on Microchip app note TB040.
 *	Can't take the root of numbers higher than MAX_INT32.
 *	@param val is the number to be computed
 *	@returns the computed integer square root
 */
uint32 sqrtInt(uint32 val);


/**
 *	@brief Computes the magnitude of the input vector.
 *
 *	@param x the x component of the vector
 *	@param y the y component of the vector
 *	@returns the magnitude of the vector with components x and y
 */
int32 vectorMag(int32 x, int32 y);

/**
 * @brief Continually decrements the input value by one until it is closest to zero.
 *
 * @param val the value to be decremented
 * @returns the decremented value (within the range of 0 <= val < 1)
 */
uint32 decToZero(uint32 val);


/**
 * @brief Average two values.
 *
 * @param val1 is the first value
 * @param val2 is the second value
 * @returns the average of val1 and val2
 */
int32 average(int32 val1, int32 val2);


/**
 * @brief Counts how many bits the value has.
 *
 * Ignores leading zeros.
 * @param val is the value to be counted
 * @returns the number of bits of the input value
 */
uint8 bitsCount(uint32 val);


/**
 * @brief Finds the min of the two arguments.
 *
 * Finds the min of two arguments.
 * @param x is the value to be compared
 * @param y is the value to be compared * @returns the min value
 */
int32 min(int32 x, int32 y);


/**
 * @brief Finds the min of the two arguments.
 *
 * Finds the min of two arguments.
 * @param x is the value to be compared
 * @param y is the value to be compared
 * @returns the min value
 */
int32 max(int32 x, int32 y);


/**
 * @brief Bounds the value with one specified bound as both lower and upper bound.
 *
 * Bounds the input value so that it stays within the range of -bound <= value <= bound.
 * If it exceeds the bound, set it to the bound.
 * @param val is the value to be bounded
 * @param bound is the lower and upper bound
 * @returns the bounded value
 */
int32 boundAbs(int32 val, int32 bound);


/**
 * @brief Bounds the value with specified lower and upper bound.
 *
 * Bounds the value so that it stays within the range of lowerBound <= value <= upperBound.
 * If it exceeds the bound, set it to the lower/upper bound.
 * @param val is the value to be bounded
 * @param lowerBound is the lower bound
 * @param upperBound is the upper bound
 * @returns the bounded value
 */
int32 bound(int32 val, int32 lowerBound, int32 upperBound);

// trig and angles

/**
 * @brief Interprets the angle as milli-radian of sine.
 *
 * @param angle the angle to be interpreted
 * @returns if angle is greater than pi/4, angle as milli-radian of sine. else, 0.
 */
int16 sinMilliRad(int16 angle);


/**
 * Interprets the angle as milli-randian of cosine.
 *
 * @param angle the angle to be interpreted
 * @returns angle as milli-radian of cosine
 */
int16 cosMilliRad(int16 angle);


/**
 * 	@brief Gets atan2 approximation in miiliradians.
 *
 *  Originally developed by John Aspinal at iRobot. It is quite good.
 *  @param y y-coordinate of the point to be calculated
 *  @param x x-coordinate of the point to be calculated
 *  @returns atan2 approximation of the input point, specified by (x,y) coordinate
 */
int16 atan2MilliRad(int32 y, int32 x);


/**
 * @brief Normalizes the angle.
 *
 * Normalizes the angle to make it stay in the range of 0 <= angle < millirad_2PI
 * @param angle the angle to be normalized
 * @returns the normalized angle
 */
int16 normalizeAngleMilliRad(int16 angle);


/**
 * @brief  Normalizes the angle.
 *
 * Normalizes the angle to make it stay in the range of -millirad_PI < angle <= millirad_PI.
 * @param angle the angle to be normalized
 * @returns the normalized angle in milli-radians
 */
int16 normalizeAngleMilliRad2(int16 angle);


/**
 * @brief Normalizes the angle.
 *
 * Normalizes the angle to make it stay in the range of 0 <= angle < microrad_2PI.
 * @param angle the angle to be normalized
 * @returns the normalized angle in micro-radians
 */
int32 normalizeAngleMicroRad(int32 angle);


//TODO: Commented out in C file, Should this be deleted
int16 normalizeAngleMilliRad3(int16 angle);


/**
 * @brief Calculates the smallest angle difference between the two input angles.
 *
 * The difference will be within the range of -MILLIRAD_PI <- difference <= MILLIRAD_PI.
 * @param thetaGoal is first angle
 * @param theta is second angle
 * @returns the difference between thetaGoal and theta
 */
int16 smallestAngleDifference(int16 thetaGoal, int16 theta);


/**
 *	@brief Finds the min of 3 input values.
 *
 *	@param	valueA first int32 value
 *	@param	valueB second int32 value
 *	@param	valueC third int32 value
 *	@returns minimum of the 3 input values
 */
int32 min3(int32 valueA, int32 valueB, int32 valueC);

// Averaging and arrays

//Is this even right? Updated so that params are the same but there's a TODO:check for overflow in the actual code
/**
 * 	@brief IIR (low pass) filter for integer quantities
 *
 *  runs an IIR filter for integer quantities.
 *  @param sample new measurement
 *  @param average
 *  @param alpha IIR constant.  This is divided by 100, so 100 = all newVal
 *  and 0 = all currentVal
 *  @returns filtered value
 */
int32 filterIIR(int32 sample, int32 average, int32 alpha);


/*
 * @brief Calculates an IIR of two angles in milliradians.
 *
 * @param currentVal is the first angle to be averaged
 * @param newVal is the second angle to be averaged
 * @param alpha is the ratio of new/old given in units of x/100. 10 would be a slow filter, 90 a fast one
 * @returns the IIR response of these values
 */
int16 filterIIRAngle(int16 currentVal, int16 newVal, int32 alpha);


/**
 * @brief Calculates the average of the two angles in milli-radians.
 *
 * @param angle1 is the first angle to be averaged
 * @param angle2 is the second angle to be averaged
 * @returns the average angle
 */
int16 averageAngles(int16 angle1, int16 angle2);


/**
 * @brief Calculates the average of the two angles in micro-radians.
 *
 * @param theta1 is the first angle to be averaged
 * @param theta2 is the second angle to be averaged
 * @returns the average angle
 */
int32 averageAnglesMicroRad(int32 theta1, int32 theta2);


/**
 * @brief Calculates the average of the angles in the array.
 *
 * Calculates the average of the first "size (a number)" of angles in angleArray.
 * @param angleArray[] is the array of angles to be averaged
 * @param size specifies how many elements in the array (starting from the first) should be averaged
 * @returns the average of the angles in the array (returns 0 if given a nonpositive size)
 */
int16 averageArrayAngle(int16 angleArray[], int32 size);


/**
 * @brief Calculates the average of the two angles in millirad.
 *
 * @param angleLeft is the left angle to be averaged
 * @param angleRight is the right angle to be averaged
 * @returns the average angle
 */
int16 averageAnglesLeftToRight(int16 angleLeft, int16 angleRight);


/**
 * @brief unit conversion (16bit to 8bit)
 *
 *
 * @param angle -angle that needs to be changed
 * @returns int16 angle in milli-radians
 */
int16 byteToMillirad(int8 angle);


/**
 * @brief unit conversion (16bit to 8bit)
 *
 *
 * @param angle
 * @returns int8 the angle as a Byte
 */
int8 milliradToByte(int16 angle);


// byte packing

/**
 * @brief Pack a 16-bit dataWord into 8-bit, pointed to by pointer arrayPtr.
 *
 * @param arrayPtr points to the packed 8-bit dataWord
 * @param dataWord 16-bit data to be packed
 * @returns void
 */
void pack16(uint8 * arrayPtr, uint32 dataWord);


/**
 * @brief Pack a 24-bit dataWord into 8-bit, pointed to by char pointer arrayPtr.
 *
 * @param arrayPtr points to the packed 8-bit dataWord
 * @param dataWord 24-bit data to be packed
 * @returns void
 */
void pack24(uint8 * arrayPtr, uint32 dataWord);


/**
 * @brief Pack a 32-bit dataWord into 8-bit, pointed to by char pointer arrayPtr.
 *
 * @param arrayPtr points to the packed 8-bit dataWord
 * @param dataWord 32-bit data to be packed
 * @returns void
 */
void pack32(uint8 * arrayPtr, uint32 dataWord);


/**
 * @brief Unpacks an 8-bit data into 16-bit.
 *
 * @param arrayPtr points to data with 8-bit wordlength
 * @returns unpacked input data with 16-bit wordlength
 */
uint16 unpack16(uint8 * arrayPtr);


/**
 * @brief Unpacks an 8-bit data into 24-bit.
 *
 * @param arrayPtr points to data with 8-bit wordlength
 * @returns unpacked input data with 24-bit wordlength
 */
uint32 unpack24(uint8 * arrayPtr);


/**
 * @brief Unpacks an 8-bit data into 32 bit.  Implemented in a pedantic
 * way to avoid assumptions of endianness.
 *
 * @param arrayPtr points to data with 8-bit wordlength
 * @returns unpacked input data with 32-bit wordlength
 */
uint32 unpack32(uint8 * arrayPtr);

// Angle - Byte converting

/**
 * @brief unit conversion (8bit to 16bit)
 *
 *
 * @param angle as a  8 Byte
 * @returns a int16 angle in milli-radians
 */
int16 byteToMillirad(int8  angle);


/**
 * @brief unit conversion (16bit to 8bit)
 *
 *
 */
int8 milliradToByte(int16  angle);


/**
 * @brief unit conversion (8bit to 16bit)
 *
 *
 * @param angle as a  8 Byte
 * @returns a int16 angle in milli-radians
 */
int16 byteToMilliradUnsigned(uint8 angle);


/**
 * @brief unit conversion (16bit to 8bit)
 *
 *
 * @param angle a int16
 * @returns int8 the angle as a Byte
 */
uint8 milliradToByteUnsigned(int16 angle);


//for midi parser

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
uint32 stol(uint8 * str, int len);

// angles from bits

/**
 * @brief Calculates the resultant angle from the bit vector.  This assumes that bit0 = 0 rad
 *
 * @param bitVector is the vector of bits
 * @returns the average angle
 */
int16 angleFromBitVector(uint8 bitVector);


/**
 * @brief Calculates the resultant angle from the bit vector.
 * This assumes that bit0 = 383 rad
 *
 * @param bitVector is the vector of bits
 * @returns the average angle
 */
int16 angleFromBitVectorOffset(uint8 bitVector);


//TODO: Update comment to talk about beacons (how is this different from angle FromBItVector)
/**
 * @brief Calculates the resultant angle from the bit vector.  This assumes that bit0 = 0 rad
 *
 * @param bitVector is the vector of bits
 * @returns the average angle
 */
int16 angleFromBitVectorBeacon(uint8 bitVector);


/**
 * @brief Determines the number of bits that reflect against an obstacle.
 *
 * The number of bits increases as the robot gets closer to the obstacle.
 * @param val a uint8 value
 * @returns the number of bits that reflect against an obstacle
 */
uint8 bitsMaxContiguous(uint8 val);


// Pose math

/**
 * @brief Calculates the smallestAngleDifference between two poses.
 *
 * @param poseGoalPtr pointer to goal pose
 * @param posePtr pointer to a pose
 * @returns smallest angle difference between two poses
 */
int32 poseAngleDiff(Pose* poseGoalPtr, Pose* posePtr);


/**
 * @brief Adds two Poses and places result in a Pose.
 *
 * @param poseResPtr pointer to Pose to hold result
 * @param pose1Ptr pointer to first Pose
 * @param pose2Ptr pointer to second Pose
 * @returns void
 */
void poseAdd(Pose* poseResPtr, Pose* pose1Ptr, Pose* pose2Ptr);


/**
 * @brief Calculates distance between two poses.
 *
 * @param pose1Ptr pointer to first Pose
 * @param pose2Ptr pointer to second Pose
 * @returns distance
 */
int32 poseDistance(Pose* pose1Ptr, Pose* pose2Ptr);

// Gaussian

/*
 * @brief Calculates gaussian noise.
 *
 * Optimization errors fixed by setting roneos optimization to 0!!!!!!!
 *
 * @returns gaussian noise +/-GAUSSIAN_LOOKUP_MAX_RETURN centered at 0.
 */
int16 gaussianNoise();

#if 0
// overflow checking math
#define add32(v1, v2) add32func(v1, v2, __FILE__, __LINE__)
#define sub32(v1, v2) sub32func(v1, v2, __FILE__, __LINE__)
#define mul32(v1, v2) mul32func(v1, v2, __FILE__, __LINE__)
#define div32(v1, v2) div32func(v1, v2, __FILE__, __LINE__)

uint8 msb32(uint32 val);
int32 add32func(int32 v1, int32 v2, const char * filePathName, unsigned int lineNum);
int32 sub32func(int32 v1, int32 v2, const char * filePathName, unsigned int lineNum);
int32 mul32func(int32 v1, int32 v2, const char * filePathName, unsigned int lineNum);
int32 div32func(int32 v1, int32 v2, const char * filePathName, unsigned int lineNum);

#define add64(v1, v2) add64func(v1, v2, __FILE__, __LINE__)
#define sub64(v1, v2) sub64func(v1, v2, __FILE__, __LINE__)
#define mul64(v1, v2) mul64func(v1, v2, __FILE__, __LINE__)
#define div64(v1, v2) div64func(v1, v2, __FILE__, __LINE__)

int64 abs64(int64 val);
uint8 msb64(uint64 val);
int64 add64func(int64 v1, int64 v2, const char * filePathName, unsigned int lineNum);
int64 sub64func(int64 v1, int64 v2, const char * filePathName, unsigned int lineNum);
int64 mul64func(int64 v1, int64 v2, const char * filePathName, unsigned int lineNum);
int64 div64func(int64 v1, int64 v2, const char * filePathName, unsigned int lineNum);
#endif

#endif // #ifndef intMath_h

