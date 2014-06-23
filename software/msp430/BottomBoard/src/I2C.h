#ifndef I2C_H_
#define I2C_H_

#include "typedefs.h"

// Timeout and Speed Definitions
//#define I2C_MAX_WHILE_DELAY		200000
#define I2C_MAX_WHILE_DELAY		40000

void I2CInit (void);
void I2CShutdown(void);
void I2CSend(uint8 address, uint8 subAddress, uint8 data);
void I2CSendArray(uint8 address, uint8 subAddress, uint8* dataPtr, uint8 length);
void I2CReceiveArray(uint8 address, uint8 subAddress, uint8* dataPtr, uint8 length);
void I2CRX_ISR(void);
void I2CInterruptDisable(void);
void endianSwap(uint8* data);

#endif /*I2C_H_*/
