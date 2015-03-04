/*
 * collectivetransport.h
 *
 *  Created on: Mar 4, 2015
 *      Author: Golnaz
 */

#ifndef COLLECTIVETRANSPORT_H_
#define COLLECTIVETRANSPORT_H_

/* Main includes */
#include <stdlib.h>
#include <stdio.h>
#include "roneos.h"
#include "ronelib.h"
#include <math.h>

/* Our includes */
#include "./tree/centroidLite.h"
#include "./transport/objectControllers.h"
#include "./tree/guide.h"
#include "./tree/leader.h"

#define NEIGHBOR_ROUND_PERIOD	300
#define RPRINTF_SLEEP_TIME		30

typedef enum states state;
enum states {
	IDLE,
	FOLLOW,
	LEADER,
	GUIDE
};

#define MOVE_TV			20

#endif /* COLLECTIVETRANSPORT_H_ */
