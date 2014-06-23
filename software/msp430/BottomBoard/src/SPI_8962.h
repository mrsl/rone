#ifndef SPI_8962_H_
#define SPI_8962_H_

void SPI8962Init (void);
void SPI8962Shutdown(void);
void SPI8962InterruptEnable();
boolean SPI8962GetMessage(uint8* message);
void SPI8962SetMessage(uint8* message);
void SPI8962RX_ISR(void);
void SPI8962InterruptDisable(void);
uint8 messageChecksum(uint8* msg, uint8 codeLen, uint8 msgLen);
void shiftBuffer(uint8* msg, uint8 len);

#endif /*SPI_8962_H_*/
