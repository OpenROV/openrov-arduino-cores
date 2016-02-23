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

#include "SERCOM.h"
#include "Arduino.h"
#include "variant.h"

#define STANDARD_MODE_FAST_MODE 0x0
#define FAST_MODE_PLUS 0X01
#define HIGHSPEED_MODE 0X02

using namespace i2c;

SERCOM::SERCOM( Sercom *s )
{
	sercom = s;
}

/* 	=========================
 	===== Sercom UART
 	=========================
*/
void SERCOM::initUART( SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate )
{
	InitClockNVIC();
	resetUART();

	//Setting the CTRLA register
	sercom->USART.CTRLA.reg =	SERCOM_USART_CTRLA_MODE( mode ) |
	                            SERCOM_USART_CTRLA_SAMPR( sampleRate );

	//Setting the Interrupt register
	sercom->USART.INTENSET.reg =	SERCOM_USART_INTENSET_RXC |  //Received complete
	                                SERCOM_USART_INTENSET_ERROR; //All others errors

	if( mode == UART_INT_CLOCK )
	{
		uint16_t sampleRateValue;

		if( sampleRate == SAMPLE_RATE_x16 )
		{
			sampleRateValue = 16;
		}
		else
		{
			sampleRateValue = 8;
		}

		// Asynchronous fractional mode (Table 24-2 in datasheet)
		//   BAUD = fref / (sampleRateValue * fbaud)
		// (multiply by 8, to calculate fractional piece)
		uint32_t baudTimes8 = ( SystemCoreClock * 8 ) / ( sampleRateValue * baudrate );

		sercom->USART.BAUD.FRAC.FP   = ( baudTimes8 % 8 );
		sercom->USART.BAUD.FRAC.BAUD = ( baudTimes8 / 8 );
	}
}
void SERCOM::initFrame( SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits )
{
	//Setting the CTRLA register
	sercom->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_FORM( ( parityMode == SERCOM_NO_PARITY ? 0 : 1 ) ) |
	                            dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

	//Setting the CTRLB register
	sercom->USART.CTRLB.reg |=	SERCOM_USART_CTRLB_CHSIZE( charSize ) |
	                            nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |
	                            ( parityMode == SERCOM_NO_PARITY ? 0 : parityMode ) << SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value
}

void SERCOM::initPads( SercomUartTXPad txPad, SercomRXPad rxPad )
{
	//Setting the CTRLA register
	sercom->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_TXPO( txPad ) |
	                            SERCOM_USART_CTRLA_RXPO( rxPad );

	// Enable Transceiver and Receiver
	sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;
}

void SERCOM::resetUART()
{
	// Start the Software Reset
	sercom->USART.CTRLA.bit.SWRST = 1 ;

	while( sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST )
	{
		// Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
	}
}

void SERCOM::enableUART()
{
	//Setting  the enable bit to 1
	sercom->USART.CTRLA.bit.ENABLE = 0x1u;

	//Wait for then enable bit from SYNCBUSY is equal to 0;
	while( sercom->USART.SYNCBUSY.bit.ENABLE );
}

void SERCOM::flushUART()
{
	// Wait for transmission to complete
	while( sercom->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE );
}

void SERCOM::clearStatusUART()
{
	//Reset (with 0) the STATUS register
	sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
}

bool SERCOM::availableDataUART()
{
	//RXC : Receive Complete
	return sercom->USART.INTFLAG.bit.RXC;
}

bool SERCOM::isUARTError()
{
	return sercom->USART.INTFLAG.bit.ERROR;
}

void SERCOM::acknowledgeUARTError()
{
	sercom->USART.INTFLAG.bit.ERROR = 1;
}

bool SERCOM::isBufferOverflowErrorUART()
{
	//BUFOVF : Buffer Overflow
	return sercom->USART.STATUS.bit.BUFOVF;
}

bool SERCOM::isFrameErrorUART()
{
	//FERR : Frame Error
	return sercom->USART.STATUS.bit.FERR;
}

bool SERCOM::isParityErrorUART()
{
	//PERR : Parity Error
	return sercom->USART.STATUS.bit.PERR;
}

bool SERCOM::isDataRegisterEmptyUART()
{
	//DRE : Data Register Empty
	return sercom->USART.INTFLAG.bit.DRE;
}

uint8_t SERCOM::readDataUART()
{
	return sercom->USART.DATA.bit.DATA;
}

int SERCOM::writeDataUART( uint8_t data )
{
	//Flush UART buffer
	flushUART();

	//Put data into DATA register
	sercom->USART.DATA.reg = ( uint16_t )data;
	return 1;
}

/*	 =========================
 	===== Sercom SPI
 	=========================
*/
void SERCOM::initSPI( SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder )
{
	resetSPI();
	InitClockNVIC();

	//Setting the CTRLA register
	sercom->SPI.CTRLA.reg =	SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
	                        SERCOM_SPI_CTRLA_DOPO( mosi ) |
	                        SERCOM_SPI_CTRLA_DIPO( miso ) |
	                        dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;

	//Setting the CTRLB register
	sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE( charSize ) |
	                        SERCOM_SPI_CTRLB_RXEN;	//Active the SPI receiver.


}

void SERCOM::initSPIClock( SercomSpiClockMode clockMode, uint32_t baudrate )
{
	//Extract data from clockMode
	int cpha, cpol;

	if( ( clockMode & ( 0x1ul ) ) == 0 )
	{
		cpha = 0;
	}
	else
	{
		cpha = 1;
	}

	if( ( clockMode & ( 0x2ul ) ) == 0 )
	{
		cpol = 0;
	}
	else
	{
		cpol = 1;
	}

	//Setting the CTRLA register
	sercom->SPI.CTRLA.reg |=	( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |
	                            ( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );

	//Synchronous arithmetic
	sercom->SPI.BAUD.reg = CalculateBaudrateSynchronous( baudrate );
}

void SERCOM::resetSPI()
{
	//Setting the Software Reset bit to 1
	sercom->SPI.CTRLA.bit.SWRST = 1;

	//Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
	while( sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST );
}

void SERCOM::enableSPI()
{
	//Setting the enable bit to 1
	sercom->SPI.CTRLA.bit.ENABLE = 1;

	while( sercom->SPI.SYNCBUSY.bit.ENABLE )
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}
}

void SERCOM::disableSPI()
{
	//Setting the enable bit to 0
	sercom->SPI.CTRLA.bit.ENABLE = 0;

	while( sercom->SPI.SYNCBUSY.bit.ENABLE )
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}
}

void SERCOM::setDataOrderSPI( SercomDataOrder dataOrder )
{
	//Register enable-protected
	disableSPI();

	sercom->SPI.CTRLA.bit.DORD = dataOrder;

	enableSPI();
}

SercomDataOrder SERCOM::getDataOrderSPI()
{
	return ( sercom->SPI.CTRLA.bit.DORD ? LSB_FIRST : MSB_FIRST );
}

void SERCOM::setBaudrateSPI( uint8_t divider )
{
	//Can't divide by 0
	if( divider == 0 )
	{
		return;
	}

	//Register enable-protected
	disableSPI();

	sercom->SPI.BAUD.reg = CalculateBaudrateSynchronous( SERCOM_FREQ_REF / divider );

	enableSPI();
}

void SERCOM::setClockModeSPI( SercomSpiClockMode clockMode )
{
	int cpha, cpol;

	if( ( clockMode & ( 0x1ul ) ) == 0 )
	{
		cpha = 0;
	}
	else
	{
		cpha = 1;
	}

	if( ( clockMode & ( 0x2ul ) ) == 0 )
	{
		cpol = 0;
	}
	else
	{
		cpol = 1;
	}

	//Register enable-protected
	disableSPI();

	sercom->SPI.CTRLA.bit.CPOL = cpol;
	sercom->SPI.CTRLA.bit.CPHA = cpha;

	enableSPI();
}

void SERCOM::writeDataSPI( uint8_t data )
{
	while( sercom->SPI.INTFLAG.bit.DRE == 0 )
	{
		// Waiting Data Registry Empty
	}

	sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register

	while( sercom->SPI.INTFLAG.bit.TXC == 0 || sercom->SPI.INTFLAG.bit.DRE == 0 )
	{
		// Waiting Complete Transmission
	}
}

uint16_t SERCOM::readDataSPI()
{
	while( sercom->SPI.INTFLAG.bit.DRE == 0 || sercom->SPI.INTFLAG.bit.RXC == 0 )
	{
		// Waiting Complete Reception
	}

	return sercom->SPI.DATA.bit.DATA;  // Reading data
}

bool SERCOM::isBufferOverflowErrorSPI()
{
	return sercom->SPI.STATUS.bit.BUFOVF;
}

bool SERCOM::isDataRegisterEmptySPI()
{
	//DRE : Data Register Empty
	return sercom->SPI.INTFLAG.bit.DRE;
}

//bool SERCOM::isTransmitCompleteSPI()
//{
//	//TXC : Transmit complete
//	return sercom->SPI.INTFLAG.bit.TXC;
//}
//
//bool SERCOM::isReceiveCompleteSPI()
//{
//	//RXC : Receive complete
//	return sercom->SPI.INTFLAG.bit.RXC;
//}



// =========================
// ===== Sercom TWI
// =========================

void SERCOM::InitMasterMode_TWI( uint32_t baudRateIn )
{
	// Set up interrupts and clocks
	// TODO: Handle failure
	InitClockNVIC();

	// Reset
	Reset_TWI();

	// Sync
	SyncBusy_TWI();

	// Set sercom peripheral to operate in I2C Master Mode
	sercom->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER;
		// | SERCOM_I2CM_CTRLA_LOWTOUTEN
		// | SERCOM_I2CM_CTRLA_INACTOUT( 0x3 )
		// | SERCOM_I2CM_CTRLA_MEXTTOEN;

	// Enable Smart Mode (The currently set CTRLB.ACKACT is sent automatically when DATA.DATA is read)
	//sercom->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;

	// Calculate baud setting from requested rate
	int32_t tempBaud = ( SystemCoreClock / ( 2 * baudRateIn ) - 1 );

	// Make sure it falls in the proper range
	if( tempBaud < 255 && tempBaud > 0 )
	{
		sercom->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_SPEED( 0 );
		sercom->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD( tempBaud );
	}
	else
	{
		// Otherwise it defaults to max to create the slowest speed
		sercom->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_SPEED( 0 );
		sercom->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD( 255 );
	}
	
	// Just in case
	SyncBusy_TWI();
}

void SERCOM::InitSlaveMode_TWI( uint8_t addressIn )
{
	// TODO: Not yet implemented
}

void SERCOM::Reset_TWI()
{
	// Reset Peripheral
	sercom->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_SWRST;

	// Sync
	SyncReset_TWI();
}

void SERCOM::Enable_TWI()
{
	// Enable the peripheral. Doesn't matter if you use I2CM or I2CS
	sercom->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;

	// Sync
	SyncEnable_TWI();

	// Move to the Idle bus state. Usually starts at Unknown state.
	MoveToIdleBusState_TWI();
}

void SERCOM::Disable_TWI()
{
	// Disable the peripheral. Doesn't matter if you use I2CM or I2CS
	sercom->I2CM.CTRLA.reg |= ~SERCOM_I2CM_CTRLA_ENABLE;

	// Sync
	SyncEnable_TWI();
}

void SERCOM::SyncReset_TWI()
{
	// Sync after setting CTRLA.SWRST
	while( sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SWRST );
}

void SERCOM::SyncEnable_TWI()
{
	// Sync after setting CTRLA.ENABLE
	while( sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_ENABLE );
}

void SERCOM::SyncSysOp_TWI()
{
	// Sync after:
	// 	Writing to STATUS.BUSSTATE in Master Mode
	// 	Writing to ADDR.ADDR in Master Mode
	//	Writing to DATA in Master Mode

	while( sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP );
}

void SERCOM::SyncBusy_TWI()
{
	// Sync after all types of sync events are complete
	while( sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_MASK );
}

void SERCOM::MoveToIdleBusState_TWI()
{
	// Get operation start time for timeout
	m_startTime = millis();

	// Wait for bus to move from Unknown state to Idle
	while( !( sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_BUSSTATE( ETWIBusState::IDLE ) ) )
	{
		// Force bus state change after 10ms timeout
		if( millis() - m_startTime >= m_timeout_ms )
		{
			// Force bus state to idle
			sercom->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE( ETWIBusState::IDLE );
		}
	}

	// TODO: If still not in Idle, something is wrong. Mark I2C interface as unusable.
}

void SERCOM::SetTimeout_TWI( uint32_t timeoutMsIn )
{
	m_timeout_ms = timeoutMsIn;
}

void SERCOM::PrepareNack_TWI()
{
	// Set ACKACT to Nack
	sercom->I2CM.CTRLB.bit.ACKACT = ETWIMasterAckAction::NACK;
}

void SERCOM::PrepareAck_TWI()
{
	// Set ACKACT to Ack
	sercom->I2CM.CTRLB.bit.ACKACT = ETWIMasterAckAction::ACK;
}

void SERCOM::SendCommand_TWI( ETWIMasterCommand commandIn )
{
	// Set CTRLB.CMD to issue the specified master operation
	sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD( commandIn );
}

void SERCOM::ClearInterruptMB_TWI()
{
	// Setting INTFLAG.MB to 1 clears the interrupt
	// Note: The following other actions also clear this bit:
	//	- Writing to ADDR.ADDR
	//	- Writing to DATA.DATA
	//	- Reading DATA.DATA when Smart Mode is enabled
	//	- Writing a valid command to CTRLB.CMD
	sercom->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_MB;
}

void SERCOM::ClearInterruptSB_TWI()
{
	// Setting INTFLAG.SB to 1 clears the interrupt
	// Note: The following other actions also clear this bit:
	//	- Writing to ADDR.ADDR
	//	- Writing to DATA.DATA
	//	- Reading DATA.DATA when Smart Mode is enabled
	//	- Writing a valid command to CTRLB.CMD
	sercom->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_SB;
}

int SERCOM::StartTransmission_TWI( uint8_t addressIn, ETWIReadWriteFlag flagIn )
{
	// Send Start | Address | Write/Read
	sercom->I2CM.ADDR.reg = ( addressIn << 1 ) | flagIn;

	m_startTime = millis();

	if( flagIn == ETWIReadWriteFlag::WRITE )
	{
		// Wait for response on bus - Either An error condition or a slave 
		while( !CheckInterruptMB_TWI() )
		{
			// Handle loss of arbitration
			if( IsArbitrationLost_TWI() )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -2;
			}
			
			// Handle bus error
			if( IsBusError_TWI() )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -3;
			}
		
			// Handle timeout
			if( millis() - m_startTime >= m_timeout_ms )
			{
				// If we time out, we need to change the ACKACT to a NACK
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -1;
			}
		}
		

		
		// Check for NACK
		if( IsRXNackReceived_TWI() )
		{
			// Slave busy. Issue stop command.
			PrepareNack_TWI();
			SendCommand_TWI( ETWIMasterCommand::STOP );
			return -3;
		}
	}
	else
	{
		// Wait for response on bus - Either An error condition or a slave 
		while( !CheckInterruptMB_TWI() && !CheckInterruptSB_TWI() )
		{
			// Handle loss of arbitration
			if( IsArbitrationLost_TWI() )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -2;
			}
			
			// Handle bus error
			if( IsBusError_TWI() )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -3;
			}
			
			// Handle timeout
			if( millis() - m_startTime >= m_timeout_ms )
			{
				// If we time out, we need to change the ACKACT to a NACK
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -1;
			}
		}
		
		// Check for NACK
		if( IsRXNackReceived_TWI() )
		{
			// Slave busy. Issue stop command.
			PrepareNack_TWI();
			SendCommand_TWI( ETWIMasterCommand::STOP );
			return -3;
		}
	}
	
	// All is well. Following read or write commands will clear interrupt bits for us
	return 0;
}

int SERCOM::ReadAsMaster_TWI( uint8_t *dataOut, int lengthIn, bool sendRepeatedStart )
{
	uint8_t bytesRead = 0;

	// Read data into buffer
	while( bytesRead != lengthIn )
	{
		m_startTime = millis();
			
		// Wait for the next byte to come in. We can either fail here via the slave not writing data or loss of arbitration. We force a NACK/STOP in either case to recover.
		while( !CheckInterruptMB_TWI() && !CheckInterruptSB_TWI() )
		{
			// Handle loss of arbitration
			if( IsArbitrationLost_TWI() )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -2;
			}
			
			// Handle bus error
			if( IsBusError_TWI() )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -3;
			}
			
			// Handle timeout
			if( millis() - m_startTime >= m_timeout_ms )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -1;
			}
		}
		
		// Copy data from DATA register to user buffer (clears SB intflag )
		dataOut[ bytesRead ] = sercom->I2CM.DATA.bit.DATA;
		bytesRead++;
		
		if( bytesRead == lengthIn )
		{
			break;
		}
		 
		// Issue read command to get the next byte
		PrepareAck_TWI();
		SendCommand_TWI( ETWIMasterCommand::BYTE_READ );
	}

	// Send NACK/STOP to signal that master is finished reading
	PrepareNack_TWI();
	
	if( sendRepeatedStart )
	{
		SendCommand_TWI( ETWIMasterCommand::REPEAT_START );
	}
	else
	{
		SendCommand_TWI( ETWIMasterCommand::STOP );
	}
	
	return bytesRead;
}

int SERCOM::WriteAsMaster_TWI( uint8_t *dataIn, int lengthIn, bool sendRepeatedStart )
{
	uint16_t bytesRemaining	= lengthIn;
	uint16_t bufferIndex 	= 0;

	// Write bytes
	while( bytesRemaining-- )
	{
		// Write byte of data to the DATA register
		sercom->I2CM.DATA.bit.DATA = dataIn[ bufferIndex++ ];

		m_startTime = millis();

		// Wait for ACK/NACK
		while( !(sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) )
		{
			// Handle loss of arbitration
			if( IsArbitrationLost_TWI() )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -2;
			}
			
			// Handle bus error
			if( IsBusError_TWI() )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -3;
			}
			
			// Handle timeout
			if( millis() - m_startTime >= m_timeout_ms )
			{
				PrepareNack_TWI();
				SendCommand_TWI( ETWIMasterCommand::STOP );
				return -1;
			}
		}

		// Check for NACK from slave
		if( sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK )
		{
			PrepareNack_TWI();
			SendCommand_TWI( ETWIMasterCommand::STOP );
			return -4;
		}
	}
	
	if( sendRepeatedStart )
	{
		SendCommand_TWI( ETWIMasterCommand::REPEAT_START );
	}
	else
	{
		SendCommand_TWI( ETWIMasterCommand::STOP );
	}


	return 0;
}

int SERCOM::WriteAsSlave_TWI( uint8_t dataIn )
{
	// TODO: Not yet implemented
	return -1;
}


bool SERCOM::IsRXNackReceived_TWI()
{
	return ( sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK );
}

bool SERCOM::IsArbitrationLost_TWI()
{
	return ( sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_ARBLOST );
}

bool SERCOM::IsBusError_TWI()
{
	return ( sercom->I2CM.STATUS.bit.BUSERR );
}

bool SERCOM::CheckInterruptMB_TWI()
{
	return ( sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB );
}

bool SERCOM::CheckInterruptSB_TWI()
{
	return ( sercom->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB ); 
}

bool SERCOM::IsBusStateIdle_TWI()
{
	return ( sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_BUSSTATE( ETWIBusState::IDLE ) );
}

bool SERCOM::IsMasterExtendedSCLTimeout_TWI()
{
	return ( sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_MEXTTOUT ); 
}

bool SERCOM::HasBusOwnership_TWI()
{
	return ( sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_BUSSTATE( ETWIBusState::OWNER ) );
}

bool SERCOM::IsMasterMode_TWI()
{
	return ( sercom->I2CS.CTRLA.bit.MODE == ETWIMode::MASTER );
}

bool SERCOM::IsSlaveMode_TWI()
{
	return ( sercom->I2CS.CTRLA.bit.MODE == ETWIMode::SLAVE );
}

// These functions used in slave mode, primarily
bool SERCOM::IsDataReady_TWI()
{
	return ( sercom->I2CS.INTFLAG.bit.DRDY );
}

bool SERCOM::IsStopDetected_TWI()
{
	 return ( sercom->I2CS.INTFLAG.bit.PREC );
}

bool SERCOM::IsRestartDetected_TWI()
{
	return ( sercom->I2CS.STATUS.bit.SR );
}

bool SERCOM::IsAddressMatch_TWI()
{
	return ( sercom->I2CS.INTFLAG.bit.AMATCH );
}

bool SERCOM::IsMasterReadOperation_TWI()
{
	return ( sercom->I2CS.STATUS.bit.DIR );
}

bool SERCOM::IsDataAvailable_TWI()
{
	if( IsMasterMode_TWI() )
	{
		return ( sercom->I2CM.INTFLAG.bit.SB );
	}
	else
	{
		return ( sercom->I2CS.INTFLAG.bit.DRDY );
	}
}


/*	 =========================
 	===== Sercom Shared
 	=========================
*/

uint8_t SERCOM::CalculateBaudrateSynchronous( uint32_t baudRateIn )
{
	return SERCOM_FREQ_REF / ( 2 * baudRateIn ) - 1;
}


void SERCOM::InitClockNVIC()
{
	uint8_t clockId = 0;
	IRQn_Type IdNvic = PendSV_IRQn ; // Dummy init to intercept potential error later

	if( sercom == SERCOM0 )
	{
		clockId = GCM_SERCOM0_CORE;
		IdNvic 	= SERCOM0_IRQn;
	}
	else if( sercom == SERCOM1 )
	{
		clockId = GCM_SERCOM1_CORE;
		IdNvic 	= SERCOM1_IRQn;
	}
	else if( sercom == SERCOM2 )
	{
		clockId = GCM_SERCOM2_CORE;
		IdNvic 	= SERCOM2_IRQn;
	}
	else if( sercom == SERCOM3 )
	{
		clockId = GCM_SERCOM3_CORE;
		IdNvic 	= SERCOM3_IRQn;
	}
	else if( sercom == SERCOM4 )
	{
		clockId = GCM_SERCOM4_CORE;
		IdNvic 	= SERCOM4_IRQn;
	}
	else if( sercom == SERCOM5 )
	{
		clockId = GCM_SERCOM5_CORE;
		IdNvic 	= SERCOM5_IRQn;
	}

	if( IdNvic == PendSV_IRQn )
	{
		// We got a problem here
		return ;
	}

	// Setting NVIC
	NVIC_EnableIRQ( IdNvic );
	NVIC_SetPriority( IdNvic, ( 1 << __NVIC_PRIO_BITS ) - 1 ); /* set Priority */

	//Setting clock
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( clockId ) | // Generic Clock 0 (SERCOMx)
	                    GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
	                    GCLK_CLKCTRL_CLKEN ;

	while( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}
}
