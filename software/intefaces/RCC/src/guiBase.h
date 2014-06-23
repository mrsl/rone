#ifndef DRAWING_H_
#define DRAWING_H_

#define COLOR_SIZE 		3

#define NUM_DRAWING_LISTS	1

#define LIST_CIRCLE_FILLED	(drawingListBase + 0)

#define POINT_SIZE 		10
#define DISK_SLICES 		60
#define DISK_LOOPS			3

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

extern GLint drawingListBase;

void drawInit();

#endif
