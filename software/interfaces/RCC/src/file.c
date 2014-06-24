#include "rcc.h"

/* The script template */
const char scriptTemplate[256] = "#$language = \"VBScript\"\r\n#$interface = \"1.0\"\r\n\r\nSub Main()\r\n\tcrt.Session.Connect \"/TELNET %s %d\"\r\n\tcrt.Screen.Synchronous = True\r\n\tcrt.Screen.WaitForString \"Enter the robot ID you wish to view: \"\r\n\tcrt.Screen.Send \"%d\" & Chr(13)\r\nEnd Sub\r\n";

/**
 * Opens a secureCRT window and connects to the server to the requested ID
 */
int
openClientConnection(int robotID)
{
    HANDLE hTempFile = INVALID_HANDLE_VALUE;

    DWORD dwRetVal = 0;

    TCHAR szTempFileName[MAX_PATH];
    TCHAR lpTempPathBuffer[MAX_PATH];
    char buffer[1024];

    /* Create a temporary file */
	dwRetVal = GetTempPath(MAX_PATH, lpTempPathBuffer);

    if (dwRetVal > MAX_PATH || (dwRetVal == 0))
        return (-1);

    if (GetTempFileName(lpTempPathBuffer,
    					TEXT("SCRIPT"),
    					0,
    					szTempFileName) == 0)
    	return (-1);

    if (CreateFile((LPTSTR) szTempFileName,
    				GENERIC_WRITE,
    				0,
					NULL,
					CREATE_ALWAYS,
					FILE_ATTRIBUTE_NORMAL,
					NULL) == INVALID_HANDLE_VALUE);
        return (-1);

	/* Output the script to the temporary file */
	fcprintf(&hTempFile, scriptTemplate, ipAddress, port, robotID);

	if (!CloseHandle(hTempFile))
	   return (-1);

	/* Open secureCRT with the script as an argument */
	if (sprintf(buffer, "/SCRIPT \"%s\"", szTempFileName) < 0)
		return (-1);

	ShellExecute(GetDesktopWindow(), "open",
				 "securecrt.exe", buffer, "", SW_SHOW);

    return (0);
}
