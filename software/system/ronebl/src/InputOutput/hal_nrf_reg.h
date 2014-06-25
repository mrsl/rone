/* Copyright (c) 2006 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 * $Rev: 1731 $
 *
 */

/**
 *  @file hal_nrf_reg.h
 * @brief Register definitions for the nRF HAL module
 * @defgroup nordic_hal_nrf_reg nRF24L01 Register definitions
 * @{
 * @ingroup nordic_hal_nrf
 * Header file defining register mapping with bit definitions.\ This file is radio-chip dependent, and are included with the hal_nrf.h
 */

#ifndef HAL_NRF_REG_H__
#define HAL_NRF_REG_H__


/** @name - Instruction Set - */
//@{
/* nRF24L01 Instruction Definitions */
#define NRF_R_REGISTER     		0x00  /**< Register read command */
#define NRF_W_REGISTER     		0x20  /**< Register write command */
#define REGISTER_MASK 0x1F
#define NRF_R_RX_PAYLOAD   		0x61  /**< Read RX payload command */
#define NRF_W_TX_PAYLOAD   		0xA0  /**< Write TX payload command */
#define NRF_FLUSH_TX      		0xE1  /**< Flush TX register command */
#define NRF_FLUSH_RX      		0xE2  /**< Flush RX register command */
#define NRF_REUSE_TX_PL   		0xE3  /**< Reuse TX payload command */
#define NRF_R_RX_PAYLOAD_WID   	0x60  /**< Read RX payload command */
#define NRF_W_ACK_PAYLOAD  		0xA8  /**< Write ACK payload command */
#define NRF_W_TX_PAYLOAD_NOACK 	0xB0  /**< Write ACK payload command */
#define NRF_NOP           		0xFF  /**< No Operation command, used for reading status register */
#define NRF_LOCK_UNLOCK   		0x50  /**< Lock/unlcok exclusive features */
//@}


/** @name  - Register Memory Map - */
//@{
/* nRF24L01 * Register Definitions * */
#define NRF_CONFIG        	0x00  /**< nRF24L01 config register */
#define NRF_EN_AA         	0x01  /**< nRF24L01 enable Auto-Acknowledge register */
#define NRF_EN_RXADDR     	0x02  /**< nRF24L01 enable RX addresses register */
#define NRF_SETUP_AW      	0x03  /**< nRF24L01 setup of address width register */
#define NRF_SETUP_RETR    	0x04  /**< nRF24L01 setup of automatic retransmission register */
#define NRF_RF_CH         	0x05  /**< nRF24L01 RF channel register */
#define NRF_RF_SETUP      	0x06  /**< nRF24L01 RF setup register */
#define NRF_STATUS        	0x07  /**< nRF24L01 status register */
#define NRF_OBSERVE_TX    	0x08  /**< nRF24L01 transmit observe register */
#define NRF_RPD            	0x09  /**< nRF24L01 receive power detect register */
#define NRF_RX_ADDR_P0    	0x0A  /**< nRF24L01 receive address data pipe0 */
#define NRF_RX_ADDR_P1    	0x0B  /**< nRF24L01 receive address data pipe1 */
#define NRF_RX_ADDR_P2    	0x0C  /**< nRF24L01 receive address data pipe2 */
#define NRF_RX_ADDR_P3    	0x0D  /**< nRF24L01 receive address data pipe3 */
#define NRF_RX_ADDR_P4    	0x0E  /**< nRF24L01 receive address data pipe4 */
#define NRF_RX_ADDR_P5    	0x0F  /**< nRF24L01 receive address data pipe5 */
#define NRF_TX_ADDR       	0x10  /**< nRF24L01 transmit address */
#define NRF_RX_PW_P0      	0x11  /**< nRF24L01 \# of bytes in rx payload for pipe0 */
#define NRF_RX_PW_P1      	0x12  /**< nRF24L01 \# of bytes in rx payload for pipe1 */
#define NRF_RX_PW_P2      	0x13  /**< nRF24L01 \# of bytes in rx payload for pipe2 */
#define NRF_RX_PW_P3      	0x14  /**< nRF24L01 \# of bytes in rx payload for pipe3 */
#define NRF_RX_PW_P4      	0x15  /**< nRF24L01 \# of bytes in rx payload for pipe4 */
#define NRF_RX_PW_P5      	0x16  /**< nRF24L01 \# of bytes in rx payload for pipe5 */
#define NRF_FIFO_STATUS   	0x17  /**< nRF24L01 FIFO status register */
#define NRF_DYNPD         	0x1C  /**< nRF24L01 Dynamic payload setup */
#define NRF_FEATURE       	0x1D  /**< nRF24L01 Exclusive feature setup */

//@}


///* Bit Mnemonics */
//#define MASK_RX_DR  6
//#define MASK_TX_DS  5
//#define MASK_MAX_RT 4
//#define EN_CRC      3
//#define CRCO        2
//#define PWR_UP      1
//#define PRIM_RX     0
//#define ENAA_P5     5
//#define ENAA_P4     4
//#define ENAA_P3     3
//#define ENAA_P2     2
//#define ENAA_P1     1
//#define ENAA_P0     0
//#define ERX_P5      5
//#define ERX_P4      4
//#define ERX_P3      3
//#define ERX_P2      2
//#define ERX_P1      1
//#define ERX_P0      0
//#define AW          0
//#define ARD         4
//#define ARC         0
//#define PLL_LOCK    4
//#define RF_DR       3
//#define RF_PWR      1
//#define LNA_HCURR   0
//#define RX_DR       6
//#define TX_DS       5
//#define MAX_RT      4
//#define RX_P_NO     1
//#define TX_FULL     0
//#define PLOS_CNT    4
//#define ARC_CNT     0
//#define TX_REUSE    6
//#define FIFO_FULL   5
//#define TX_EMPTY    4
//#define RX_FULL     1
//#define RX_EMPTY    0


/** @name CONFIG register bit definitions */
//@{

#define NRF_CONFIG_MASK_RX_DR    6     /**< CONFIG register bit 6 */
#define NRF_CONFIG_MASK_TX_DS    5     /**< CONFIG register bit 5 */
#define NRF_CONFIG_MASK_MAX_RT   4     /**< CONFIG register bit 4 */
#define NRF_CONFIG_EN_CRC        3     /**< CONFIG register bit 3 */
#define NRF_CONFIG_CRCO          2     /**< CONFIG register bit 2 */
#define NRF_CONFIG_PWR_UP        1     /**< CONFIG register bit 1 */
#define NRF_CONFIG_PRIM_RX       0     /**< CONFIG register bit 0 */
//@}

/** @name RF_SETUP register bit definitions */
//@{
#define NRF_SETUP_PLL_LOCK      4     /**< RF_SETUP register bit 4 */
#define NRF_SETUP_RF_DR         3     /**< RF_SETUP register bit 3 */
#define NRF_SETUP_RF_PWR1       2     /**< RF_SETUP register bit 2 */
#define NRF_SETUP_RF_PWR0       1     /**< RF_SETUP register bit 1 */
#define NRF_SETUP_LNA_HCURR     0     /**< RF_SETUP register bit 0 */
//@}

/* STATUS 0x07 */
/** @name STATUS register bit definitions */
//@{
#define NRF_STATUS_RX_DR         6     /**< STATUS register bit 6 */
#define NRF_STATUS_TX_DS         5     /**< STATUS register bit 5 */
#define NRF_STATUS_MAX_RT        4     /**< STATUS register bit 4 */
#define NRF_STATUS_TX_FULL       0     /**< STATUS register bit 0 */
//@}

/* FIFO_STATUS 0x17 */
/** @name FIFO_STATUS register bit definitions */
//@{
#define NRF_FIFOSTATUS_TX_REUSE      6     /**< FIFO_STATUS register bit 6 */
#define NRF_FIFOSTATUS_TX_FIFO_FULL  5     /**< FIFO_STATUS register bit 5 */
#define NRF_FIFOSTATUS_TX_EMPTY      4     /**< FIFO_STATUS register bit 4 */
#define NRF_FIFOSTATUS_RX_FULL       1     /**< FIFO_STATUS register bit 1 */
#define NRF_FIFOSTATUS_RX_EMPTY      0     /**< FIFO_STATUS register bit 0 */
//@}

#define NRF_ENAA_ENAA_P5     5       /**< dynamic payload enable */
#define NRF_ENAA_ENAA_P4     4       /**< dynamic payload enable */
#define NRF_ENAA_ENAA_P3     3       /**< dynamic payload enable */
#define NRF_ENAA_ENAA_P2     2       /**< dynamic payload enable */
#define NRF_ENAA_ENAA_P1     1       /**< dynamic payload enable */
#define NRF_ENAA_ENAA_P0     0       /**< dynamic payload enable */

#define NRF_DYNPD_DPL_P5      5       /**< dynamic payload enable */
#define NRF_DYNPD_DPL_P4      4       /**< dynamic payload enable */
#define NRF_DYNPD_DPL_P3      3       /**< dynamic payload enable */
#define NRF_DYNPD_DPL_P2      2       /**< dynamic payload enable */
#define NRF_DYNPD_DPL_P1      1       /**< dynamic payload enable */
#define NRF_DYNPD_DPL_P0      0       /**< dynamic payload enable */

#define NRF_FEATURE_EN_DPL      2       /**< dynamic payload enable */
#define NRF_FEATURE_EN_ACK_PAY  1       /**< dynamic payload enable */
#define NRF_FEATURE_EN_DYN_ACK  0       /**< dynamic payload enable */


#endif // HAL_NRF_REG_H__
/** @} */
