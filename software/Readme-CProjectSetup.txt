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
