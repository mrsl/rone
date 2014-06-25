/**
 * registry.h
 *
 * Header file for registry.c
 */
#ifndef REG_H_
#define REG_H_

#define MAXPORT			1000
#define PORTSIZE		64
#define REGISTRYWATCH	1000

/* Struct to hold registry data over serial ports */
struct regData {
	int n;
	int ports[MAXPORT];
};

/* Array that maps COM number to robot ID */
extern int commToNum[MAXPORT];

void initCommWatch();
void *commWatch(void *vargp);
void enumCommNames(struct regData *data);

#endif
