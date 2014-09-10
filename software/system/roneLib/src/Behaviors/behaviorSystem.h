/**
 * @file behaviorSystem.h
 *
 * @brief Behaviors associated with robot movement.
 * @details
 * @since September 11, 2011
 * @author James McLurkin
 */

#ifndef BEHAVIORSYSTEM_H_
#define BEHAVIORSYSTEM_H_

#define BEHAVIOR_TASK_PRIORITY		(BACKGROUND_TASK_PRIORITY + 1)
#define BEHAVIOR_TASK_PERIOD		50

/**
 * @brief Structure for robot behaviors.
 */
typedef struct Beh {
	int32 tv;			/**< translational velocity: minimum/maximum tv are ____ (mm/s)*/
	int32 rv;			/**< rotational velocity: minimum/maximum rv are ____ (mradians/s)*/
	boolean active;     /**< boolean that is TRUE if behavior is active, FALSE otherwise*/
} Beh;

extern const Beh behInactive;

// behavior system
//TODO Should brief be more specific?
/**
 * @brief Initializes the behavior system task.
 *
 * @returns void
 */
void behaviorSystemInit(void (*behaviorTask)(void* parameters), uint32 stackSize);


/**
 *  @brief Moves the robot based on the input motor behavior.
 *
 *  If the input behavior is inactive, the motors are not turned on.
 *  @param behPtr for desired motor behavior
 *  @returns void
 */
void motorSetBeh(Beh* behPtr);


/**
 * @brief Gets translational velocity
 *
 * @param behPtr for behavior
 * @returns translational velocity of input behavior
 */
int32 behGetTv(Beh* behPtr);


/**
 *  @brief Sets the input behavior's translational and rotational velocities
 *
 *  @param behPtr for behavior to update
 *  @param tv desired translational velocity
 *  @param rv desired rotational velocity
 *  @returns void
 */
void behSetTvRv(Beh* behPtr, int32 tv, int32 rv);


/**
 *  @brief Sets the input behavior's translational velocity
 *
 *  @param behPtr for behavior to update
 *  @param tv desired translational velocity
 *  @returns void
 */
void behSetTv(Beh* behPtr, int32 tv);


/**
 *  @brief Sets the input behavior's rotational velocity
 *
 *  @param behPtr for behavior to update
 *  @param rv desired rotational velocity
 *  @returns void
 */
void behSetRv(Beh* behPtr, int32 rv);


/**
 *  @brief Gets the input behavior's rotational velocity.
 *
 *  @param behPtr for behavior
 *  @returns the rotational velocity of the input behavior
 */
int32 behGetRv(Beh* behPtr);


/**
 *  @brief Tells whether or not the input behavior is active.
 *
 *  @param behPtr for behavior
 *  @returns a 1 if the behavior is active and 0 if inactive
 */
uint8 behIsActive(Beh* behPtr);


/**
 *  @brief Sets the input behavior to active
 *
 *  @param behPtr for behavior
 *  @returns void
 */
void behSetActive(Beh* behPtr);


/**
 *  @brief Sets the input behavior to inactive
 *
 *  @param behPtr for behavior
 *  @returns void
 */
void behSetInactive(Beh* behPtr);


/**
 *  @brief Determines if the red, green, or blue button has been pressed.
 *
 *  Updates the input modePtr to indicate which color has been pressed.
 *  @param modePtr for the mode to update if a button has been pressed
 *  @param modeRed the mode that should be set if the red button has been pressed
 *  @param modeGreen the mode that should be set if the green button has been pressed
 *  @param modeBlue the mode that should be set if the blue button has been pressed
 *  @returns a boolean TRUE if a button has been pressed (and therefore modePtr updated) and FALSE if not
 */
boolean buttonModeRGB(uint8* modePtr, uint8 modeRed, uint8 modeGreen, uint8 modeBlue);

/**
 * @brief Subsume a low priority behavior with a high priority one
 *
 * Compute behOutPtr by subsuming behInLoPtr with behInHighPtr if behInHighPtr is active.
 * Otherwise, behOutPtr is set to behInLoPtr.
 *
 * @param behInLoPtr low-priority input behavior
 * @param behInHighPtr high-priority input behavior
 * @param behOutPtr output behavior
 * @returns behOutPtr
 */
Beh* behSubsume(Beh* behOutPtr, Beh* behInLoPtr, Beh* behInHighPtr);
#endif /* BEHAVIORSYSTEM_H_ */
