/**
 * rcc.c
 *
 * Main function of the RCC
 */
#include "rcc.h"

int port;
int verbose = 0;

/**
 * Main function
 */
int main(int argc, char **argv)
{
	int i;
	int err = 0;
	port = 8000; // Use port 8000 as default

	/* Parse command line arguments */
	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = 1;
			continue;
		}
		if (i + 1 != argc) {
			if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
				if (sscanf(argv[i + 1], "%d", &port) != 1) {
					err = 1;
					break;
				}
				continue;
			}
		}
		err = 1;
		break;
	}

	if (err) {
		printf("Usage: %s <-v|--verbose> <-p|--port> [portnum]\n", argv[0]);
		return (-1);
	}

	if (!verbose)
		FreeConsole();

	/* Initialize robot buffers */
	initRobots();
	initCommWatch();

	/* Create web server */
	if (createServer(port) < 0)
		Error("Failed to create server");

	guiInit();

	return (0);
}
