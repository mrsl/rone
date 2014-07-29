/*
 * guiCommands.c
 *
 * Functions for various commands on the GUI
 */
#include "rcc.h"

/* The script template */
const char scriptTemplate[212] =
	"#$language = \"VBScript\"\r\n#$interface = \"1.0\"\r\n\r\nSub Main()\r\n\tcrt.Session.Connect \"/TELNET %s %d\"\r\n\tcrt.Screen.Synchronous = True\r\n\tcrt.Screen.WaitForString \"Enter the robot ID\"\r\n\tcrt.Screen.Send \"%d\" & Chr(13)\r\n";
const char scriptEnd[10] = "End Sub\r\n";
const char scriptAT[62] = "\tcrt.Screen.WaitForString \"Robot AprilTag\"\r\n\tcrt.Screen.Send ";
const char scriptATSend[8] = "\"%d\" & ";
const char scriptEnter[10] = "Chr(13)\r\n";

/**
 * Opens a secureCRT window and connects to the server to the requested ID
 */
int openClientConnection(int robotID, int aprilTagID)
{
	HANDLE hTempFile = INVALID_HANDLE_VALUE;

	DWORD dwRetVal = 0;
	UINT uRetVal = 0;

	TCHAR szTempFileName[MAX_PATH];
	TCHAR lpTempPathBuffer[MAX_PATH];
	char buffer[1024];

	/* Create a temporary file */
	dwRetVal = GetTempPath(MAX_PATH, lpTempPathBuffer);

	if (dwRetVal > MAX_PATH || (dwRetVal == 0)) {
		return (-1);
	}

	uRetVal = GetTempFileName(lpTempPathBuffer,
							  TEXT("SCRIPT"),
							  0,
							  szTempFileName);
	if (uRetVal == 0) {
		return (-1);
	}

	hTempFile = CreateFile((LPTSTR) szTempFileName,
						   GENERIC_WRITE, 0,
						   NULL,
						   CREATE_ALWAYS,
						   FILE_ATTRIBUTE_NORMAL,
						   NULL);

	if (hTempFile == INVALID_HANDLE_VALUE) {
		return (-1);
	}

	if (robotID == 0) {
		hprintf(&hTempFile, scriptTemplate, ipAddress, port, robotID);
		hprintf(&hTempFile, scriptAT);
		if (aprilTagID != -1) {
			hprintf(&hTempFile, scriptATSend, aprilTagID);
		}
		hprintf(&hTempFile, scriptEnter);
		hprintf(&hTempFile, scriptEnd);
	/* Output the script to the temporary file */
	} else {
		hprintf(&hTempFile, scriptTemplate, ipAddress, port, robotID);
		hprintf(&hTempFile, scriptEnd);
	}


	if (!CloseHandle(hTempFile)) {
		return (-1);
	}

	/* Open secureCRT with the script as an argument */
	if (sprintf(buffer, "/SCRIPT \"%s\"", szTempFileName) < 0) {
		return (-1);
	}

	ShellExecute(GetDesktopWindow(),
				 "open",
				 "securecrt.exe",
				 buffer,
				 "",
				 SW_SHOW);

	return (0);
}

/**
 * Opens a direct secureCRT connection to a local robot
 */
int directConnect(int robotID)
{
	char buffer[BUFFERSIZE];

	blacklist(robotID);

	if (sprintf(buffer, "/SERIAL COM%d /BAUD 230400 /NOCTS",
		robots[robotID].port) < 0) {
		return (-1);
	}

	ShellExecute(GetDesktopWindow(),
				 "open",
				 "securecrt.exe",
				 buffer,
				 "",
				 SW_SHOW);

	return (0);
}

/**
 * Kills all secureCRT instances
 */
void killSecureCRT()
{
	unsigned int i;
	char name[64] = "SecureCRT.EXE";
	TCHAR processName[64];
	DWORD processIDs[MAX_PROCESSES];
	DWORD pBytesReturned;
	HANDLE process;

	HMODULE hMod;
	DWORD cbNeeded;

	if (EnumProcesses(processIDs,
					  MAX_PROCESSES,
					  &pBytesReturned) == 0) {
		return;
	}

	for (i = 0; i < pBytesReturned / sizeof(DWORD); i++) {
		process = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_TERMINATE |
							  PROCESS_VM_READ,
							  FALSE,
							  processIDs[i]);
		if (process == NULL) {
			continue;
		}

		if (EnumProcessModules(process,
							   &hMod,
							   sizeof(hMod),
							   &cbNeeded)) {
			GetModuleBaseName(process,
							  hMod,
							  processName,
							  sizeof(processName) / sizeof(TCHAR));

			if (strcmp(name, processName) == 0) {
				TerminateProcess(process, 0);
			}
		}
	}
}

void hostRobot(int robotID)
{
	if (robots[robotID].type == LOCAL && robots[robotID].hSerial != NULL
		&& !robots[robotID].blacklisted) {
		hprintf(robots[robotID].hSerial, "rt\n");
	}
}

void blacklist(int robotID)
{
	if (robots[robotID].blacklisted) {
		robots[robotID].blacklisted = 0;
		if (initCommCommander(robots[robotID].port) < 0) {
			commToNum[robots[robotID].port] = 0;
			robots[robotID].type = UNKNOWN;
		}
	} else {
		robots[robotID].blacklisted = 1;
		if (robots[robotID].hSerial != NULL) {
			CloseHandle(*robots[robotID].hSerial);
			robots[robotID].hSerial = NULL;
		}
	}
}

void commConnect(int robotID)
{
	if (robots[robotID].type != REMOTE && robots[robotID].hSerial != NULL
		&& !robots[robotID].blacklisted) {
		directConnect(robotID);
	}
}

/* TODO: Modularize log files */
void beginLog(int robotID)
{
	char fileName[MAX_PATH], date[BUFFERSIZE];
	datestr(date);

	if (!robots[robotID].log) {
		robots[robotID].log = 1;
		if (robots[robotID].aid != -1) {
			sprintf(fileName, "%s\\%d_AT%d_%s.log", logDir, robotID,
				robots[robotID].aid, date);
		} else {
			sprintf(fileName, "%s\\%d_%s.log", logDir, robotID, date);
		}
		robots[robotID].logH = CreateFile((LPTSTR) fileName,
										  GENERIC_WRITE,
										  0,
										  NULL,
										  CREATE_ALWAYS,
										  FILE_ATTRIBUTE_NORMAL,
										  NULL);
		if (robots[robotID].logH == INVALID_HANDLE_VALUE) {
			robots[robotID].log = 0;
		}
	} else {
		robots[robotID].log = 0;
		CloseHandle(robots[robotID].logH);
	}
}

void beginAprilTagLog(int aprilTagID)
{
	char fileName[MAX_PATH], date[BUFFERSIZE];
	datestr(date);

	if (!aprilTagData[aprilTagID].log) {
		aprilTagData[aprilTagID].log = 1;
		if (aprilTagData[aprilTagID].rid != -1) {
			sprintf(fileName, "%s\\AT%d_%d_%s.log", logDir, aprilTagID,
				aprilTagData[aprilTagID].rid, date);
		} else {
			sprintf(fileName, "%s\\AT%d_%s.log", logDir, aprilTagID, date);
		}
		aprilTagData[aprilTagID].logH = CreateFile((LPTSTR) fileName,
												   GENERIC_WRITE,
											 	   0,
											       NULL,
												   CREATE_ALWAYS,
												   FILE_ATTRIBUTE_NORMAL,
												   NULL);
		if (aprilTagData[aprilTagID].logH == INVALID_HANDLE_VALUE) {
			aprilTagData[aprilTagID].log = 0;
		}
	} else {
		aprilTagData[aprilTagID].log = 0;
		CloseHandle(aprilTagData[aprilTagID].logH);
	}
}

void openLocalConnections()
{
	int i;

	for (i = 0; i < MAXROBOTID; i++) {
		mutexLock(&robots[i].mutex);
		/* If the robot is active */
		if (robots[i].up != 0 && !robots[i].blacklisted) {
			if (robots[i].type == LOCAL || robots[i].type == HOST) {
				openClientConnection(i, -1);
			}
		}
		mutexUnlock(&robots[i].mutex);
	}
}

void openRemoteConnections()
{
	int i;

	for (i = 0; i < MAXROBOTID; i++) {
		mutexLock(&robots[i].mutex);
		/* If the robot is active */
		if (robots[i].up != 0) {
			if (robots[i].type == REMOTE) {
				openClientConnection(i, -1);
			}
		}
		mutexUnlock(&robots[i].mutex);
	}
}

void showRobotInfo(int robotID)
{
	if (robots[robotID].display) {
		robots[robotID].display = 0;
	} else {
		if (!robots[robotID].blacklisted
			|| robots[robotID].type == REMOTE) {
			robots[robotID].display = 1;
		}
	}
}

void showAprilTagInfo(int robotID)
{
	robotID -= 2000;

	if (aprilTagData[robotID].display) {
		aprilTagData[robotID].display = 0;
	} else {
		aprilTagData[robotID].display = 1;
	}
}

int guiConnect(int robotID)
{
	char exeName[BUFFERSIZE];
	char buffer[BUFFERSIZE];

	if (robots[robotID].type == REMOTE || robots[robotID].hSerial == NULL
		|| robots[robotID].blacklisted) {
		return (-1);
	}

	blacklist(robotID);

	if (sprintf(exeName, "%s\\%s", guiPath, "roneGUI.exe") < 0) {
		return (-1);
	}

	if (sprintf(buffer, "-p %d", robots[robotID].port) < 0) {
		return (-1);
	}

	ShellExecute(GetDesktopWindow(),
				 "open",
				 exeName,
				 buffer,
				 "",
				 SW_SHOW);

	return (0);
}
