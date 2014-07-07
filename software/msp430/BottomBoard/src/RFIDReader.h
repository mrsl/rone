#ifndef RFIDREADER_H_
#define RFIDREADER_H_

/*
 * The RFID tag reader uses the bump sensor ports to bit-bang an SPI-like
 * serial protocol
 */
#define RFID_PORT_DIR	P2DIR
#define RFID_PORT_IN	P2IN
#define RFID_PORT_REN	P2REN
#define RFID_PORT_OUT	P2OUT
#define RFID_PORT_SEL	P2SEL
#define RFID_PORT_IFLG	P2IFG
#define RFID_PORT_IE	P2IE
#define RFID_PORT_IFES	P2IES

#define RFID_TAG_PRES_BIT	BIT0
#define RFID_CLOCK_BIT		BIT1
#define RFID_DATA_BIT		BIT6
#define RFID_DATA_BIT_IDX	6

#define RFID_INTERUPT_BITS	(RFID_TAG_PRES_BIT | RFID_CLOCK_BIT)
#define RFID_BITS (RFID_TAG_PRES_BIT | RFID_CLOCK_BIT | RFID_DATA_BIT)

#define RFID_TAG_DATA_LEN 		7

//#ifdef RONE_V12_TILETRACK
//
//void RFIDReaderInit();
//void RFIDReaderUpdate();
//uint8 RFIDReaderGet();
//void RFIDInterruptEnable();
//void RFIDInterruptDisable();
//void RFIDReaderInterupt();
//
//#else

#define RFIDReaderInit()	{}
#define RFIDReaderUpdate() 	{}
#define RFIDReaderGet()		0
#define RFIDInterruptEnable() 	{}
#define RFIDInterruptDisable() 	{}

//#endif //#ifndef RONE_V12_TILETRACK

#endif /* RFIDREADER_H_*/
