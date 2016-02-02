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


// TWI enums
namespace i2c
{
	enum ETWIMode : uint8_t 
	{
		SLAVE = 0x4u,
		MASTER = 0x5u
	};
	
	enum ETWIBusState: uint8_t 
	{
		UNKNOWN = 0x0ul,
		IDLE,
		OWNER,
		BUSY
	};
	
	enum ETWIReadWriteFlag: uint8_t 
	{
		WRITE = 0x0ul,
		READ
	};
	
	enum ETWIMasterCommand: uint8_t 
	{
		NO_ACTION = 0,
		REPEAT_START,
		BYTE_READ,
		STOP
	} ;
	
	enum ETWIMasterAckAction: uint8_t
	{
		ACK 	= 0,
		NACK 	= 1
	};
}

class SERCOM
{
public:
	SERCOM( Sercom *s ) ;

	/* ========== UART ========== */
	void initUART( SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate = 0 ) ;
	void initFrame( SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits ) ;
	void initPads( SercomUartTXPad txPad, SercomRXPad rxPad ) ;

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

	/* ========== WIRE ========== */
	void InitMasterMode_TWI( uint32_t baudRateIn );
	void InitSlaveMode_TWI( uint8_t addressIn );
	void Reset_TWI();
	void Enable_TWI();
	void Disable_TWI();
	void PrepareNack_TWI();
	void PrepareAck_TWI();
	void SendCommand_TWI( i2c::ETWIMasterCommand commandIn );
	void SetTimeout_TWI( uint32_t timeoutMsIn );
	void SyncReset_TWI();
	void SyncEnable_TWI();
	void SyncSysOp_TWI();
	void SyncBusy_TWI();
	void ClearInterruptMB_TWI();
	void ClearInterruptSB_TWI();
	void MoveToIdleBusState_TWI();

	int StartTransmission_TWI( uint8_t addressIn, i2c::ETWIReadWriteFlag flagIn );
	int ReadAsMaster_TWI( uint8_t *dataOut, int lengthIn, bool sendRepeatedStart = false );
	int WriteAsMaster_TWI( uint8_t *dataIn, int lengthIn, bool sendRepeatedStart = false );
	
	int WriteAsSlave_TWI( uint8_t dataIn );

	bool IsRXNackReceived_TWI();
	bool IsArbitrationLost_TWI();
	bool IsBusError_TWI();
	bool CheckInterruptMB_TWI();
	bool CheckInterruptSB_TWI();
	bool HasBusOwnership_TWI();
	bool IsBusStateIdle_TWI();
	bool IsMasterExtendedSCLTimeout_TWI();
	bool IsMasterMode_TWI();
	bool IsSlaveMode_TWI();

	bool IsDataReady_TWI();
	bool IsStopDetected_TWI();
	bool IsRestartDetected_TWI();
	bool IsAddressMatch_TWI();
	bool IsMasterReadOperation_TWI();
	bool IsDataAvailable_TWI();

private:
	// Shared private members
	Sercom* sercom;
	
	uint32_t m_timeout_ms = 10;
	uint32_t m_startTime = 0;

	// Methods
	uint8_t CalculateBaudrateSynchronous( uint32_t baudrateIn );
	
	void InitClockNVIC();
};

#endif
