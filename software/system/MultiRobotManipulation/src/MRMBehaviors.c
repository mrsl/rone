/*
 * File for the behaviors of MRM experiment
 *
 * Author: Lauren Schmidt
 * Created On: Sept. 17, 2013
 */

#include <stdio.h>
#include <stdlib.h>


#include "roneos.h"
#include "ronelib.h"

Beh* behGuide(Beh* behOutput){
	uint8 navigationMode = MODE_IDLE;
	for(;;){
		/*** READ BUTTONS ***/
		if (buttonsGet(BUTTON_RED)) {
			//Show the generated tree
			navigationMode = SHOW_TREE_SOURCE;
			//nbrDataSet(&msgShowTree, 1);
		} else if (buttonsGet(BUTTON_GREEN)) {
			//Set goal robot
			navigationMode = SOURCE;
			//broadcastMsgSetSource(&broadcastMessage, TRUE);
			//nbrDataSetFloat(&msgWeight, 0.0);
		} else if (buttonsGet(BUTTON_BLUE)) {
			//OPEN
		}

		switch (navigationMode) {

			case SOURCE: {
				ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				broadcastMsgSetSource(&broadcastMessage, TRUE);
				//if(printNow) rprintf(" source w%d t%d",prevWeight,type);
				break;
			} //end TREE SOURCE
			case SHOW_TREE_SOURCE: {
				//if(printNow){rprintf("tree");}
				//IF the red button was pushed making you the begining of the tree
				if(type==1){
					ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				} else {
					ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
				}
				break;
			}
			case SHOW_TREE: {
				//if(printNow){rprintf("tree");}
				if(partOfTree){
					//You are part of the tree, turn on blue LEDS
					if(type==1){
						ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
					} else {
						ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_LOW, LED_RATE_FAST);
					}
				}
				break;
			}
			case MODE_IDLE: {
				//do nothing
				break;
			}
			default:{
				//do nothing
				if(printNow){
					//rprintf(" nothing");
				}
			}
			}//end switch
	}//forever for loop
	//Shouldn't get here
	return behOutput;
}
