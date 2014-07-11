/**
 * registry.h
 *
 * Header file for registry.c
 */
#ifndef REG_H_
#define REG_H_

#define MAXPORT			100
#define PORTSIZE		64
#define REGISTRYWATCH	500

/**
 * Data from registry
 */
struct regData
{
	int n;				// Number of registry entries found
	int ports[MAXPORT];	// Array of ports that are connected
};

extern int commToNum[MAXPORT]; // Array that maps COM number to robot ID

void commWatch(void *vargp);
void enumCommNames(struct regData *data);

#endif
