gui.c
-----
Main source file to be executed. Contains the functions directly related to 
GLUT, the user interface program.

gui.h
-----
Main header file that goes on all .c files in the project.

serial.c
--------
Functions that set up, read and write to the serial port to talk to the robot.

drawinput.c
-----------
Draws all input related GUI elements and processes input

drawoutput.c
------------
Draws all output related GUI elements from serial data

drawbase.c
----------
Provides color and shape definitions for OpenGL

drawing.h
---------
Header file for drawbase.c, provides some defines

text.c
------
Uses WGL to provide fonts.

util.c
------
Provides various utility functions used in all files

input.c
-------
Textboxes and other input methods.

output.c
--------
How the GUI parses data received from the serial port into the screen.

io.h
----
Contains structures and defines for all IO

params.h
--------
Provides parameters about the robot and GUI object positions.