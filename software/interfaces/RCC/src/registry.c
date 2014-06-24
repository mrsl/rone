#include "rcc.h"

int commToNum[MAXPORT];

/**
 * Creates the thread to watch the registry
 */
void
initCommWatch()
{
	pthread_t tid;

	Pthread_create(&tid, NULL, commWatch, NULL);
}

/**
 * Watches the registry for possible robots to connect to
 */
void
*commWatch(void *vargp)
{
	int i;
	struct regData data;	/* Read data from the registry */

	/* Get rid of pesky compiler warnings */
	vargp = (void *)vargp;

	for (;;) {
		enumCommNames(&data);

		/* Try to connect to each robot */
		for (i = 0; i < data.n; i++) {
			if (robots[commToNum[data.ports[i]]].up == 0)
				initCommCommander(data.ports[i]);
		}

		Sleep(REGISTRYWATCH);
	}
	return (NULL);
}

/**
 * Opens and views the registry and fills a data struct with possible robots
 */
void
enumCommNames(struct regData *data)
{
	int port;
	int numOfComm = 0;

	LONG status;

	HKEY  hKey;
	DWORD dwIndex = 0;
	CHAR  name[48];
	DWORD szName;
	UCHAR portName[48];
	DWORD szPortName;
	DWORD Type;

	/* Try to open the registry entry containing serial port information */
	if(RegOpenKeyEx(HKEY_LOCAL_MACHINE,
		TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"),
		0,
		KEY_READ,
		&hKey) != ERROR_SUCCESS)
		return;

	/* Read registry until failure */
	for (;;) {
		szName = sizeof(name);
		szPortName = sizeof(portName);

		status = RegEnumValue(hKey,
			dwIndex++,
			name,
			&szName,
			NULL,
			&Type,
			portName,
			&szPortName);

		if ((status == ERROR_SUCCESS)) {
			if (strstr(name, "VCP")) {
				if (sscanf((char *)portName, "COM%d", &port) != 1)
					continue;

				data->ports[numOfComm] = port;
				numOfComm++;
			}
		}

		if (status != ERROR_SUCCESS)
			break;
	}

	data->n = numOfComm;

	RegCloseKey(hKey);
}
