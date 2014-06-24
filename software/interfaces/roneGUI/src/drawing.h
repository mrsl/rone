/*
 * drawing.h
 *
 *  Created on: Sep 7, 2011
 *      Author: nathan
 */

#ifndef DRAWING_H_
#define DRAWING_H_

/* colors */
#define COLOR_SIZE 		3

extern const float color_red[COLOR_SIZE];
extern const float color_green[COLOR_SIZE];
extern const float color_blue[COLOR_SIZE];

extern const float color_cyan[COLOR_SIZE];
extern const float color_magenta[COLOR_SIZE];
extern const float color_yellow[COLOR_SIZE];

extern const float color_black[COLOR_SIZE];
extern const float color_lightgray[COLOR_SIZE];
extern const float color_gray[COLOR_SIZE];
extern const float color_darkgray[COLOR_SIZE];
extern const float color_white[COLOR_SIZE];

/*
 * Display list names
 *
 * Display lists are sequences of openGL instructions that are compiled
 * and executed repeatedly.
 */
#define NUM_DRAWING_LISTS	12

#define LIST_CIRCLE_FILLED			(drawingListBase + 0)
#define LIST_SEMICIRCLE			(drawingListBase + 1)
#define LIST_ANNULUS				(drawingListBase + 2)
#define LIST_ANNULUS_SECT			(drawingListBase + 3)
#define LIST_AXES2D				(drawingListBase + 4)
#define LIST_SQUARE				(drawingListBase + 5)
#define LIST_TRIANGLE				(drawingListBase + 6)
#define LIST_TRIANGLE_HALF			(drawingListBase + 7)
#define LIST_PUSHBUTTON_CENTRE		(drawingListBase + 8)
#define LIST_PUSHBUTTON_OUTLINE	(drawingListBase + 9)
#define LIST_IR_COMPONENTS			(drawingListBase + 10)
#define LIST_GRIPPER_CLAWS			(drawingListBase + 11)

extern GLint drawingListBase;

#endif /* DRAWING_H_ */
