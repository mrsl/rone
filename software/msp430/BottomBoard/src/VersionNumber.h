#ifndef VERSIONNUMBER_H_
#define VERSIONNUMBER_H_

/* 
 * Hardware version numbers need to fit in 2 bits. Start at 1 just because. Need a new version
 * number for each build configuration.
 */
#ifdef RONE_V11
	#define HARDWARE_VERSION_NUMBER    1
#endif

#ifdef RONE_V12
	#define HARDWARE_VERSION_NUMBER    2
#endif

#define	PROGRAM_VERSION_NUMBER		22

/* Bitpack the version ID byte */
#define FULL_ID_VERSION_NUMBER   (((HARDWARE_VERSION_NUMBER & 0x3) << 6) | (PROGRAM_VERSION_NUMBER & 0x3F))

#endif /*VERSIONNUMBER_H_*/
