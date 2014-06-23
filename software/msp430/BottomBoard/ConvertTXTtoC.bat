@ECHO OFF
:BEGIN
CLS
CHOICE /N /C:12 /M "Which version do you want to compile? (1 for V11 or 2 for V12)"%1
IF ERRORLEVEL ==2 GOTO TWELVE
IF ERRORLEVEL ==1 GOTO ELEVEN
GOTO END
:ELEVEN
ECHO You have chosen version 11.
CD %~p0
ECHO Running Jar...
java -jar ConvertTXTtoC.jar 11
ECHO The new file will be in the robotcode Systems folder!
PAUSE
GOTO END
:TWELVE
ECHO You have chosen version 12.
CD %~p0
ECHO Running Jar...
java -jar ConvertTXTtoC.jar 12
ECHO The new file will be in the robotcode Systems folder!
PAUSE
GOTO END
:END