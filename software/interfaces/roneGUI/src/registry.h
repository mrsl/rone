#ifndef REG_H_
#define REG_H_

#define MAXPORT 50
#define PORTSIZE 64
#define SLEEPTIME 500

struct regData {
	int n;
	int ports[MAXPORT];
};

extern int commToNum[MAXPORT];

void initCommWatch();
void *commWatch(void *vargp);
void enumCommNames(struct regData *data);

#endif
