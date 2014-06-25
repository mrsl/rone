/**
 * @file buttons.h
 * @brief Functions for 3 buttons on the top of the robot (R,G,B) that can be programmed
 * by the user.
 *
 * @since Mar 19, 2011
 * @author James McLurkin
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#define BUTTON_RED                  0
#define BUTTON_GREEN                1
#define BUTTON_BLUE                 2


/**
 *	@brief Initializes the buttons.
 *
 *	Initializes the red, green, and blue buttons as input.
 *	@returns void
 */
void buttonsInit(void);


/**
 *	@brief Gets the state of the specified button.
 *
 *	Tells you whether the specified button is on or off
 *	@param	button specifies which button to check (BUTTON_RED, BUTTON_BLUE, or BUTTON_GREEN)
 *	@returns void
 */
uint32 buttonsGet(uint32 button);

#endif /* BUTTONS_H_ */
