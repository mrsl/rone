
1. Eclipse IDE:
http://www.eclipse.org/downloads/packages/eclipse-standard-431/keplersr1


2. Eclipse CDT (Install from within eclipse):
http://www.eclipse.org/cdt/


3. ARM-Eclipse integration:
http://gnuarmeclipse.livius.net/blog/


4. Code Sourcery Lite Cross-complier, Arm EABI:
http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/

Copy the files in ?\robotcode\ronelibs\zz-newprojectfiles\gcclibs
to
Sourcery_CodeBench_Lite_for_ARM_EABI\arm-none-eabi\lib\thumb2
stellaris_fury-names.inc
libcs3stellaris_fury.a

5. Download OpenOCD for JTAG debugging:
http://openocd.sourceforge.net/supported-jtag-interfaces/
http://gnuarmeclipse.livius.net/blog/openocd-install/


6. setup Open OCD for debugging:
6a. Make new GDB Hardware Debugging configuration
6b. In "Main" tab, select the elf file of the program
6c. In the "Debugger" tab:
    a) In the GDB command, select the Sourcery Lite arm-none-eabi-gdb
	b) Check the remote target box
	c) Select "OpenOCD (via pipe)" as the JTAG Device
	d) Copy the following command for the GDB connection string

| openocd -f interface/busblaster.cfg -f target/stellaris.cfg -c "gdb_port pipe; log_output openocd.log" -c "init" -c "reset halt"
openocd -d0 -f interface/busblaster.cfg -f target/stellaris.cfg -c init -c targets -c "reset halt"

6d. In the "Startup" tab:
	a) Un-check the "Reset and Delay (seconds):" box
	b) In the Initialization Commands Text Box
	
monitor adapter_khz 2000
monitor lm3s.cpu configure -rtos auto

    c) In order to break at main: Check the "Set breakpoint at: " checkbox in Runtime Options and type "main"
	d) In order to resume running the program after downloading to the flash: Check the "Resume" checkbox in the Runtime Options section

Useful Website on How to Use GDB with OpenOCD:

http://openocd.sourceforge.net/doc/html/GDB-and-OpenOCD.html#GDB-and-OpenOCD

Eclipse and GDB OpenOCD Debugging with Plugin:
Main:
Select the right files.
Debugger:
a. "Start OpenOCD locally" checked
b. "Executable:" <PATH TO OPENOCD>
c. Other Options: -f interface/busblaster.cfg -f target/stellaris.cfg -c "init"
d. GDB Client Setup:
set mem inaccessible-by-default off
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4

Startup:
a. "Initial Reset." checked with "type:" init
b. "Set breakpoint at main" checked
c. "Pre-run reset" UNCHECKED
d. "Continue" checked

==== Notes ====


6. Bus Blaster JTAG Debugger:

http://dangerousprototypes.com/docs/Bus_Blaster

7. Make files for projects:
(Under construction)




You must define the following symbols in your projects:

== driverlib ==
sourcerygxx
PART_LM3S8962 -or- PART_LM3S9B92

== FreeRTOS ==
GCC_ARMCM3

== roneos ==
GCC_ARMCM3
sourcerygxx
PART_LM3S8962 -or- PART_LM3S9B92
RONE_V6 -or- RONE_V9

== application ==
-nothing-



==== Project settings ====
includes:
"${workspace_loc:/roneLib/inc}"
"${workspace_loc:/roneos/inc}"
"${workspace_loc:/FreeRTOS/inc}"
"${workspace_loc:/driverlib}"

libraries:
roneLib
roneos
FreeRTOS
driverlib
gcc

library search path:
"${workspace_loc:/roneLib/bin}"
"${workspace_loc:/roneos/bin}"
"${workspace_loc:/FreeRTOS/bin}"
"${workspace_loc:/driverlib/bin}"

misc:
 -Werror=implicit-function-declaration -Wno-unused


==== debugger Settings ====
