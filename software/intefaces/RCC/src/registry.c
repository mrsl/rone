#include "rcc.h"

int commToNum[MAXPORT];

void
initCommWatch()
{
	pthread_t tid;

	Pthread_create(&tid, NULL, commWatch, NULL);
}

void
*commWatch(void *vargp)
{
	int i;
	struct regData data;

	for (;;) {
		enumCommNames(&data);

		for (i = 0; i < data.n; i++) {
			if (robots[commToNum[data.ports[i]]].up == 0)
				initCommCommander(data.ports[i]);
		}

		Sleep(SLEEPTIME);
	}
	return (NULL);
}

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

	if(RegOpenKeyEx(HKEY_LOCAL_MACHINE,
		TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"),
		0,
		KEY_READ,
		&hKey) != ERROR_SUCCESS)
		return;

	do {
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

		if((status == ERROR_SUCCESS)) {
			if (strstr(name, "VCP")) {
				if (sscanf(portName, "COM%d", &port) != 1)
					continue;

				data->ports[numOfComm] = port;
				numOfComm++;
			}
		}

	} while(status == ERROR_SUCCESS);

	data->n = numOfComm;

	RegCloseKey(hKey);
}
