/**
 * @file rone.h
 * @brief compiler options, and all the include files for the r-one operating system
 * @since Mar 19, 2011
 * @author jamesm
 *
 */

#ifndef RONE_H_
#define RONEOS_H_

#ifdef PART_LM3S8962
	#if defined(RONE_V6)
		#warning "Compiling for rone_v6"
	#elif defined(RONE_V9)
		#warning "Compiling for rone_v9"
	#elif defined(RONE_V12)
		#warning "Compiling for rone_v12"
	#elif defined(RONE_IRBEACON)
		#warning "Compiling for IR beacon"
	#else
		#error "robot version not defined. one of RONE_V6, RONE_V9, RONE_V12, or RONE_IRBEACON"
	#endif
#endif

// Free RTOS includes
#include "FreeRTOS.h"
#include "Task.h"
#include "queue.h"
#include "semphr.h"

// system includes
#include "../src/System/typedefs.h"
#include "../src/System/intMath.h"
#include "../src/System/system.h"
#include "../src/System/charger.h"
#include "../src/System/spi.h"
#include "../src/System/spi_message.h"
#include "../src/System/pwm.h"
#include "../src/System/msp430Bootloader.h"

#include "../src/InputOutput/buttons.h"
#include "../src/InputOutput/leds.h"
#include "../src/InputOutput/blinkyLed.h"
#include "../src/InputOutput/ir_beacon.h"
#include "../src/InputOutput/radio.h"
#include "../src/InputOutput/radioCommand.h"

#include "../src/InputOutput/Logger/integer.h"
#include "../src/InputOutput/Logger/sd_card.h"
#include "../src/InputOutput/Logger/crc_ccitt.h"
#include "../src/InputOutput/Logger/ffconf.h"
#include "../src/InputOutput/Logger/ff.h"
#include "../src/InputOutput/Logger/diskio.h"

#include "../src/Audio/audio.h"
#include "../src/Audio/Midi.h"
#include "../src/Audio/MIDIFilesOS.h"

#include "../src/IRComms/ir_comms.h"
#include "../src/IRComms/neighbors.h"
#include "../src/IRComms/nbrData.h"
#include "../src/IRComms/nbrDataFloat.h"

#include "../src/Motors/motor.h"
#include "../src/Motors/encoder.h"

#include "../src/Sensors/gyro.h"
#include "../src/Sensors/light_sensor.h"
#include "../src/Sensors/accelerometer.h"
#include "../src/Sensors/bump_sensor.h"
#include "../src/Sensors/reflective_sensors.h"

#include "../src/SerialIO/serial.h"
#include "../src/SerialIO/cprintf.h"
#include "../src/SerialIO/rprintf.h"
#include "../src/SerialIO/serialCommand.h"
#include "../src/SerialIO/serialCommand.h"
#include "../src/SerialIO/systemCommands.h"
#include "../src/SerialIO/basicPrinting.h"

#include "../src/InputOutput/Logger/logger.h"


//#include "../src/crc/crctest.h"	//TESTING

#endif /* RONE_H_ */
