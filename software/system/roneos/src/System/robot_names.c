#include "typedefs.h"
/*
 * @brief robot has a name and a numeric ID
 */
typedef struct robotName{
	uint8 id;
	char name[];
} robotName;
