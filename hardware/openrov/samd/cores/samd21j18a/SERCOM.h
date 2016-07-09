/*
    Copyright (c) 2014 Arduino.  All right reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _SERCOM_CLASS_
#define _SERCOM_CLASS_

#include "sam.h"

#define SERCOM_FREQ_REF 48000000

typedef enum
{
	UART_EXT_CLOCK = 0,
	UART_INT_CLOCK = 0x1u
} SercomUartMode;

typedef enum
{
	SPI_SLAVE_OPERATION = 0x2u,
	SPI_MASTER_OPERATION = 0x3u
} SercomSpiMode;

typedef enum
{
	SERCOM_EVEN_PARITY = 0,
	SERCOM_ODD_PARITY,
	SERCOM_NO_PARITY
} SercomParityMode;

typedef enum
{
	SERCOM_STOP_BIT_1 = 0,
	SERCOM_STOP_BITS_2
} SercomNumberStopBit;

typedef enum
{
	MSB_FIRST = 0,
	LSB_FIRST
} SercomDataOrder;

typedef enum
{
	UART_CHAR_SIZE_8_BITS = 0,
	UART_CHAR_SIZE_9_BITS,
	UART_CHAR_SIZE_5_BITS = 0x5u,
	UART_CHAR_SIZE_6_BITS,
	UART_CHAR_SIZE_7_BITS
} SercomUartCharSize;

typedef enum
{
	SERCOM_RX_PAD_0 = 0,
	SERCOM_RX_PAD_1,
	SERCOM_RX_PAD_2,
	SERCOM_RX_PAD_3
} SercomRXPad;

typedef enum
{
	UART_TX_PAD_0 = 0x0ul,	// Only for UART
	UART_TX_PAD_2 = 0x1ul,  // Only for UART
	UART_TX_RTS_CTS_PAD_0_2_3 = 0x2ul,  // Only for UART with TX on PAD0, RTS on PAD2 and CTS on PAD3
} SercomUartTXPad;

typedef enum
{
	SAMPLE_RATE_x16 = 0x1,	//Fractional
	SAMPLE_RATE_x8 = 0x3,	//Fractional
} SercomUartSampleRate;

typedef enum
{
	SERCOM_SPI_MODE_0 = 0,	// CPOL : 0  | CPHA : 0
	SERCOM_SPI_MODE_1,		// CPOL : 0  | CPHA : 1
	SERCOM_SPI_MODE_2,		// CPOL : 1  | CPHA : 0
	SERCOM_SPI_MODE_3		// CPOL : 1  | CPHA : 1
} SercomSpiClockMode;

typedef enum
{
	SPI_PAD_0_SCK_1 = 0,
	SPI_PAD_2_SCK_3,
	SPI_PAD_3_SCK_1,
	SPI_PAD_0_SCK_3
} SercomSpiTXPad;

typedef enum
{
	SPI_CHAR_SIZE_8_BITS = 0x0ul,
	SPI_CHAR_SIZE_9_BITS
} SercomSpiCharSize;

// ----------------------------------------
// I2C Definitions
// ----------------------------------------

#define I2C_DEFAULT_BAUD            100000
#define I2C_MAX_BAUD                400000

// Flags
#define I2C_FLAG_STOP 		(1 << 0)	// Issue a stop condition
#define I2C_FLAG_READ 		(1 << 1)	// Perform a read operation
#define I2C_FLAG_BUSY 		(1 << 2)	// Transaction is in progress
#define I2C_FLAG_FAIL 		(1 << 3)	// Transaction failed

namespace I2C
{
	enum EMode : uint8_t 
	{
		SLAVE   = 0x4,
		MASTER  = 0x5
	};
	
	enum EBusState: uint8_t 
	{
		UNKNOWN = 0x0,
		IDLE	= 0x1,
		OWNER	= 0x2,
		BUSY	= 0x3
	};
	
	enum EReadWriteFlag: uint8_t 
	{
		WRITE   = 0x0,
		READ	= 0x1
	};
  
  	enum EAckAction: uint8_t
	{
		ACK 	= 0x0,
		NACK 	= 0x1
	};
	
	enum EMasterCommand: uint8_t 
	{
		NO_ACTION 		= 0x0,
		REPEAT_START	= 0x1,
		BYTE_READ		= 0x2,
		STOP			= 0x3
	};
	
	enum ERetCode: int32_t
	{
		SUCCESS						= 0,                     
		ERR_INVALID_ARG        		= -1,
		ERR_DENIED               	= -2,
		ERR_ALREADY_INITIALIZED  	= -3,
		ERR_NOT_INITIALIZED      	= -4,
		ERR_BAUDRATE_UNAVAILABLE 	= -5,
		ERR_UNSUPPORTED_OP        	= -6,
		ERR_NOT_READY				= -7
	};
	
	enum ETransactionRetCode: int32_t
	{
		SUCCESS                 = 0,   	// Operation successful
		ACK                   	= -1,  	// Received ACK from device on I2C bus
		NACK                  	= -2,  	// Received NACK from device on I2C bus
		ERR_ARBLOST           	= -3,  	// Arbitration lost
		ERR_BAD_ADDRESS       	= -4, 	// Bad address
		ERR_BUS               	= -5,  	// Bus error
		ERR_BUSY              	= -6,  	// Device busy
		ERR_PACKAGE_COLLISION 	= -7,  	// Package collision
		ERR_TIMEOUT				= -8
	};
	
	struct TTransaction
	{
		uint8_t 	slaveAddress;
		uint16_t 	flags;
		uint32_t	length;
		uint8_t		*buffer;
	};
}

// Error definitions


class SERCOM
{
public:
	SERCOM( Sercom *s ) ;

	/* ========== UART ========== */
	void initUART( SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate = 0 ) ;
	void initFrame( SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits ) ;
	void initPads( SercomUartTXPad txPad, SercomRXPad rxPad );

	void resetUART( void ) ;
	void enableUART( void ) ;
	void flushUART( void ) ;
	void clearStatusUART( void ) ;
	bool availableDataUART( void ) ;
	bool isBufferOverflowErrorUART( void ) ;
	bool isFrameErrorUART( void ) ;
	bool isParityErrorUART( void ) ;
	bool isDataRegisterEmptyUART( void ) ;
	uint8_t readDataUART( void ) ;
	int writeDataUART( uint8_t data ) ;
	bool isUARTError() ;
	void acknowledgeUARTError() ;

	/* ========== SPI ========== */
	void initSPI( SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder ) ;
	void initSPIClock( SercomSpiClockMode clockMode, uint32_t baudrate ) ;

	void resetSPI( void ) ;
	void enableSPI( void ) ;
	void disableSPI( void ) ;
	void setDataOrderSPI( SercomDataOrder dataOrder ) ;
	SercomDataOrder getDataOrderSPI( void ) ;
	void setBaudrateSPI( uint8_t divider ) ;
	void setClockModeSPI( SercomSpiClockMode clockMode ) ;
	void writeDataSPI( uint8_t data ) ;
	uint16_t readDataSPI( void ) ;
	bool isBufferOverflowErrorSPI( void ) ;
	bool isDataRegisterEmptySPI( void ) ;
	bool isTransmitCompleteSPI( void ) ;
	bool isReceiveCompleteSPI( void ) ;

	/* ========== I2C ========== */
	int32_t InitMasterMode_I2C( uint32_t baudRateIn, uint16_t optionsIn );
	
	int32_t Enable_I2C();
	int32_t Disable_I2C();
	int32_t Reset_I2C();
	int32_t SetBaudRate_I2C( uint32_t baudRateIn );
	
	
	int32_t WaitForIdleBusState_I2C();
	int32_t WaitForInterrupt_I2C();
	int32_t SendCommand_I2C( i2c::EI2CMasterCommand commandIn );
	
	void WriteCTRLA_I2C( uint16_t dataIn );
	void WriteCTRLB_I2C( uint16_t dataIn );
	
	void PrepareNack_I2C();
	void PrepareAck_I2C();
	
	void SyncReset_I2C();
	void SyncEnable_I2C();
	void SyncSysOp_I2C();
	void SyncBusy_I2C();
	
	void ClearInterruptMB_I2C();
	void ClearInterruptSB_I2C();
	void ClearInterruptERROR_I2C();

	bool IsEnabled_I2C();

	bool CheckInterruptMB_I2C();
	bool CheckInterruptSB_I2C();
	bool CheckInterruptERROR_I2C();
	
	bool IsRXNackReceived_I2C();
	
	bool IsBusError_I2C();
	bool IsArbitrationLost_I2C();

	bool IsBusStateUnknown_I2C();
	bool IsBusStateIdle_I2C();
	bool IsBusStateBusy_I2C();
	bool IsBusStateOwner_I2C();
	
	bool IsAvailable_I2C();

private:
	// Shared private members
	Sercom* 		sercom;

	// ----------------
	// I2C attributes
	TTransaction	m_transaction;

	bool 			m_isAvailable = false;
	uint32_t 		m_startTime = 0;
	const uint16_t  m_kTRise_ns = 215;


	// Methods
	uint8_t CalculateBaudrateSynchronous( uint32_t baudrateIn );
	
	void InitClockNVIC();
};

#endif
