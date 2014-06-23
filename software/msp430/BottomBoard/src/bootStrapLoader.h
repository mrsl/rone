#ifndef BOOTSTRAPLOADER_H_
#define BOOTSTRAPLOADER_H_

// bootloader version numbers
#define MSP430_BSL_VERSION_ADRESS        0xE001
#define MSP430_BSL_LOCAL_VERSION_NUMBER  0x01

void updateMSP430BSLVersion(void);
void startBSL(void);
uint8 getStoredBSLVersion(void);
#endif /*BOOTSTRAPLOADER_H_*/
