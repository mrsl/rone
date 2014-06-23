/**
 * main.c
 */
#include "rcc.h"

int port;

/**
 * Main function
 */
int
main(int argc, char **argv)
{
	port = 8000; /* Use port 8000 */

	/* Initialize robot buffers */
	initRobots();
	initCommWatch();

	/* Create web server */
	if (createServer(port) < 0) {
		if (VERBOSE)
		fprintf(stderr, "ERROR: Failed to create server\n");
		exit (-1);
	}

	guiInit(argc, argv);

	return (0);
}
