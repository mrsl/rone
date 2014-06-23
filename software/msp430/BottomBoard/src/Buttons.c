#include "typeDefs.h"
#include "Buttons.h"


#define BUTTONS_MASK	0x01


uint8 buttonBlue = 0;
uint8 buttonGreen = 0;
uint8 buttonRed = 0;

void buttonsUpdate(uint8 data) {
	buttonRed = data & BUTTONS_MASK;
	data >>= 1;
	buttonGreen = data & BUTTONS_MASK;
	data >>= 1;
	buttonBlue = data & BUTTONS_MASK;
}

uint8 buttonsGet(uint8 button) {
	switch (button) {
	case BUTTON_GREEN:
		return buttonGreen;
	case BUTTON_BLUE:
		return buttonBlue;
	case BUTTON_RED:
		return buttonRed;
	default:
		return FALSE;
	}
}
