/**
 * rcc.c
 *
 * Main function of the RCC
 */
#include "rcc.h"

int port = 8000;
int verbose = 0;
GLfloat aprilTagX = 0.;
GLfloat aprilTagY = 0.;
char logDir[MAX_PATH] = ".\\logs";
char defaultATServerIP[15] = "192.168.1.155";
char guiPath[MAX_PATH] = "..\\..\\roneGUI\\bin";

/**
 * Main function
 */
int main(int argc, char **argv)
{
	int i;
	int err = 0;

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
				i++;
				continue;
			}
			if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--height") == 0) {
				if (sscanf(argv[i + 1], "%f", &aprilTagY) != 1) {
					err = 1;
					break;
				}
				i++;
				aprilTagY /= 2.;
				continue;
			}
			if (strcmp(argv[i], "-w") == 0 || strcmp(argv[i], "--width") == 0) {
				if (sscanf(argv[i + 1], "%f", &aprilTagX) != 1) {
					err = 1;
					break;
				}
				i++;
				aprilTagX /= 2.;
				continue;
			}
			if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--log") == 0) {
				if (sscanf(argv[i + 1], "%s", logDir) != 1) {
					err = 1;
					break;
				}
				i++;
				aprilTagX /= 2.;
				continue;
			}
			if (strcmp(argv[i], "-g") == 0 || strcmp(argv[i], "--gui") == 0) {
				if (sscanf(argv[i + 1], "%s", guiPath) != 1) {
					err = 1;
					break;
				}
				i++;
				continue;
			}
		}
		err = 1;
		break;
	}

	if (err) {
		printf("Usage: %s <-v|--verbose> <-p|--port> [Port]\n    <-h|--height> [AprilTag max Y] <-w|--width> [AprilTag max X]\n    <-g|--gui> [Path to roneGUI.exe]", argv[0]);
		return (-1);
	}

	if (!verbose)
		FreeConsole();

	/* Initialize robot buffers */
	initRobots();
	makeThread(&commWatch, 0);

	/* Make the logfile directory */
	CreateDirectory(logDir, NULL);

	/* Create web server */
	if (createServer(port) < 0) {
		Error("Failed to create server");
	}

	initAprilTag();
	guiInit();

	return (0);
}
