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
// ===== Sercom I2C
// =========================

I2C::ERetCode SERCOM::InitMasterMode_I2C( const I2C::TOptions &optionsIn )
{
	// Set up interrupts and clocks
	InitClockNVIC();
	
	if( IsEnabled_I2C() )
	{
		// Can't initialize while interface is enabled
		return I2C::ERetCode::ERR_DENIED;
	}

	// Reset
	Reset_I2C();

	// Set options
	m_i2cOptions = optionsIn;

	// Set sercom peripheral to operate in I2C Master Mode
	SetBitsCTRLA_I2C( SERCOM_I2CM_CTRLA_MODE_I2C_MASTER
					| ( m_i2cOptions.setSCLSM ? SERCOM_I2CM_CTRLA_SCLSM : 0 )					// Interrupts only occur after ACK/NACK sent
					| ( m_i2cOptions.enableINACTOUT ? SERCOM_I2CM_CTRLA_INACTOUT( 0x3 ) : 0 )	// Inactivity timeout
					| ( m_i2cOptions.enableMEXTOUT ? SERCOM_I2CM_CTRLA_MEXTTOEN : 0 )			// Cumulative transaction time timeout
					| ( m_i2cOptions.enableLOWTOUT ? SERCOM_I2CM_CTRLA_LOWTOUTEN : 0 ) );		// SCL low timeout			

	// Enable Smart Mode (The currently set CTRLB.ACKACT is sent automatically when DATA.DATA is read)
	SetBitsCTRLB_I2C( ( m_i2cOptions.setSMEN ? SERCOM_I2CM_CTRLB_SMEN : 0 ) );

	// Set the baud rate
	if( SetBaudRate_I2C( m_i2cOptions.baudRate ) )
	{
		return I2C::ERetCode::ERR_BAUDRATE_UNAVAILABLE;
	}
	
	m_isInitialized = true;
	return I2C::ERetCode::SUCCESS;
}

void SERCOM::DeinitMasterMode_I2C()
{
	// Clear enable
	ClearBitsCTRLA_I2C( SERCOM_I2CM_CTRLA_ENABLE );

	// Reset
	Reset_I2C();

	m_isInitialized = false;
}

I2C::ERetCode SERCOM::SetBaudRate_I2C( uint32_t baudRateIn )
{
	uint32_t tmp;

	if( IsEnabled_I2C() ) 
	{
		// Can't change baudrate while enabled
		return I2C::ERetCode::ERR_DENIED;
	}
	
	if( baudRateIn > I2C_MAX_BAUD )
	{
		return I2C::ERetCode::ERR_DENIED;
	}

	// NOTE: All clock units are in Hz
	tmp = (uint32_t)( ( SystemCoreClock - 10 * baudRateIn - baudRateIn * SystemCoreClock * ( m_i2cOptions.tRise_ns * 0.000000001 ) ) / ( 2 * baudRateIn ) );
	
	// Add 1 to make sure the baud rate falls slightly below the target baud rate and not above
	tmp += 1;
	
	// Set speed to standard (up to 100KHz) / fast (up to 400KHz)
	SetBitsCTRLA_I2C( SERCOM_I2CM_CTRLA_SPEED( 0x0 ) );

	// Set baud registers
	sercom->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUDLOW( 0x0 ) | SERCOM_I2CM_BAUD_BAUD( tmp );

	return I2C::ERetCode::SUCCESS;
}

void SERCOM::Reset_I2C()
{
	// Reset Peripheral
	SetBitsCTRLA_I2C( SERCOM_I2CM_CTRLA_SWRST );
}

void SERCOM::Enable_I2C()
{
	// Enable the peripheral
	SetBitsCTRLA_I2C( SERCOM_I2CM_CTRLA_ENABLE );

	// Move to the Idle bus state. Usually starts at Unknown state.
	WaitForIdleBusState_I2C();
}

void SERCOM::Disable_I2C()
{
	SyncReset_I2C();
	SyncEnable_I2C();
	
	// Disable the peripheral
	ClearBitsCTRLA_I2C( SERCOM_I2CM_CTRLA_ENABLE );
}

void SERCOM::WaitForIdleBusState_I2C()
{
	// Set timeout value
	m_timer = 0;

	// Wait for bus to move from Unknown state to Idle. Can also be moved to idle by INACTOUT triggering
	while( !IsBusStateIdle_I2C() )
	{
		m_timer++;

		// Bus state never changed, something is wrong and the I2C bus should not be used
		if( m_timer >= m_i2cOptions.timeoutPeriod )
		{
			// Force to idle
			SetBitsSTATUS_I2C( SERCOM_I2CM_STATUS_BUSSTATE( I2C::EBusState::IDLE ) );
		}
	}
}

I2C::ERetCode SERCOM::WaitForInterrupt_I2C( uint8_t &flagsOut )
{
	m_timer = 0;

	do 
	{
		++m_timer;
		flagsOut = ReadRegisterINTFLAG_I2C();

		if(  m_timer > m_i2cOptions.timeoutPeriod  ) 
		{
			// Timeout occurred
			// Not sure how this happens yet, but seemingly possible to reach this state due to triggered SMBus timeouts
			return I2C::ERetCode::ERR_TIMEOUT;
		}
	} while( !( flagsOut & I2C::EInterruptFlags::INTFLAG_MB ) && !( flagsOut & I2C::EInterruptFlags::INTFLAG_SB ) );

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode SERCOM::PerformTransfer_I2C( I2C::TTransfer *transferIn )
{
	I2C::ERetCode ret;

	// Check to see if bus is idle or owner
	if( !IsBusStateIdle_I2C() || !IsBusStateOwner_I2C() )
	{
		// Bus is currently busy
		return I2C::ERetCode::ERR_BUSY;
	}

	// Now busy performing a transfer	
	m_transferActive = true;
	
	// Copy the message to be transferred
	m_pTransfer = transferIn;

	if( m_pTransfer == nullptr )
	{
		return I2C::ERetCode::ERR_BAD_TRANSFER;
	}

	// Start transaction
	ret = StartTransaction_I2C();

	uint8_t flags = 0;

	// Handle any further transactions with this slave (multi-byte reads and writes)
	while( m_transferActive )
	{
		// Wait for interrupts
		ret = WaitForInterrupt_I2C( flags );

		// Check for timeout
		if( ret == I2C::ERetCode::ERR_TIMEOUT )
		{
			m_transferActive = false;
		}
		else
		{
			// Finish Transaction
			ret = FinishTransaction_I2C( flags );
		}
	}

	return ret;
}

I2C::ERetCode SERCOM::StartTransaction_I2C()
{
	// Perform bounds checking on slave address
	if( m_pTransfer->slaveAddress > 0x7F )
	{
		return I2C::ERetCode::ERR_BAD_ADDRESS;
	}

	I2C::ERetCode ret;
	uint8_t flags 	= 0;

	// For reads, prepare ACK/NACK to be sent after reading the data
	if( m_pTransfer->length == 1 && m_i2cOptions.setSCLSM )
	{
		// Only a one byte transaction, respond with NACK to end transaction with slave
		PrepareNack_I2C();
	}
	else
	{
		// More bytes to read after this transaction, send ACK
		PrepareAck_I2C();
	}

	// Set the address and R/W bit
	WriteADDR_I2C( ( m_pTransfer->slaveAddress << 1 ) | m_pTransfer->action );

	// Wait for interrupts
	ret = WaitForInterrupt_I2C( flags );

	if( ret == I2C::ERetCode::ERR_TIMEOUT )
	{
		m_transferActive = false;
		return ret;
	}

	return FinishTransaction_I2C( flags );
}

I2C::ERetCode SERCOM::FinishTransaction_I2C( uint8_t flagsIn )
{
	// Get Bus Status
	sercom_i2cm_status_reg_t status = ReadRegisterSTATUS_I2C();

	if( flagsIn & I2C::EInterruptFlags::INTFLAG_MB )
	{
		if( status & SERCOM_I2CM_STATUS_ARBLOST ) 
		{
			// We can arrive here in multiple ways:
			// 1. Arbitration lost or bus error during address transmit
			// 2. Arbitration lost or bus error during data transmit
			// 3. Arbitration lost or bus error during ACK/NACK of read op

			// Arbitration lost
			ClearInterruptMB_I2C();

			// TODO: May need to specify that we kill it here
			// Now that MB is cleared, the while loop in transfer will time out
			m_transferActive = false;

			if( status & SERCOM_I2CM_STATUS_BUSERR ) 
			{
				// Bus error
				return I2C::ERetCode::ERR_BUS;
			}

			// Otherwise, bad address/non-existent slave
			return I2C::ERetCode::ERR_BAD_ADDRESS;
		}
		else
		{
			// Handle NACK from slave (either an explicit NACK or a non-present slave)
			if( status & SERCOM_I2CM_STATUS_RXNACK ) 
			{
				// Send a stop. Whether it is a failure from sending an address or transmitting data, a NACK with MB means a failed transfer.
				SendBusCommand_I2C( I2C::EBusCommand::STOP );
				m_transferActive = false;

				return I2C::ERetCode::NACK;
			}

			// Handle end of transfer
			if( m_pTransfer->length == 0 ) 
			{
				// If the user intends to perform another transfer after this, don't send a stop. The next ADDR write will automatically send a repeated start
				if( !m_pTransfer->issueRepeatedStart )
				{
					SendBusCommand_I2C( I2C::EBusCommand::STOP );
				}

				m_transferActive = false;
			} 
			else 
			{
				// Write the next byte of data - this will also clear the MB intflag
				WriteDATA_I2C( *m_pTransfer->buffer );

				// Move forward in transfer buffer
				m_pTransfer->buffer++;
				m_pTransfer->length--;

				// Transfer is still active until this function is called again and we've reached length == 0
			}

			return I2C::ERetCode::SUCCESS;
		}
	}
	else if( flagsIn & I2C::EInterruptFlags::INTFLAG_SB )
	{
		if( ( m_pTransfer->length > 0 ) && !( status & SERCOM_I2CM_STATUS_RXNACK ) ) 
		{
			// Slave accepted read request and there are more bytes to read
			m_pTransfer->length--;

			// If we are coming up on the last byte in the transfer, prepare a NACK accordingly
			if( ( m_pTransfer->length == 0 && !m_i2cOptions.setSCLSM ) || ( m_pTransfer->length == 1 && m_i2cOptions.setSCLSM ) ) 
			{
				PrepareNack_I2C();
			}

			// Read byte from DATA register
			// NOTE: If SMEN is enabled, this will automatically trigger an ACK/NACK and clear the SB interrupt flag
			*m_pTransfer->buffer++ = ReadRegisterDATA_I2C();

			// Transaction complete
			if( m_pTransfer->length == 0 ) 
			{
				// If the user intends to perform another transfer after this, don't send a stop. The next ADDR write will automatically send a repeated start
				if( !m_pTransfer->issueRepeatedStart )
				{
					SendBusCommand_I2C( I2C::EBusCommand::STOP );
				}

				m_transferActive = false;
			}
			else
			{
				// If smart mode is off, we need to explicitly send a bus command to get the ACK to fire to receive the next byte
				if( !m_i2cOptions.setSMEN )
				{
					SendBusCommand_I2C( I2C::EBusCommand::BYTE_READ );
				}
			}
		} 
		else 
		{
			// Send a stop. Whether it is a failure from sending an address or transmitting data, a NACK with SB means a failed transfer.
			SendBusCommand_I2C( I2C::EBusCommand::STOP );
			m_transferActive = false;

			ClearInterruptSB_I2C();

			return I2C::ERetCode::NACK;
		}

		ClearInterruptSB_I2C();
	}

	return I2C::ERetCode::SUCCESS;
}

// -----------------------------------------------------------
// Register interactions

bool SERCOM::IsEnabled_I2C()
{
	SyncReset_I2C();
	SyncEnable_I2C();
	
	return ( sercom->I2CM.CTRLA.reg & SERCOM_I2CM_CTRLA_ENABLE );
}

void SERCOM::SyncReset_I2C()
{
	// Sync after setting CTRLA.SWRST
	while( sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SWRST );
}

void SERCOM::SyncEnable_I2C()
{
	// Sync after setting CTRLA.ENABLE
	while( sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_ENABLE );
}

void SERCOM::SyncSysOp_I2C()
{
	// Sync after:
	// 	Writing to STATUS.BUSSTATE in Master Mode
	// 	Writing to ADDR.ADDR in Master Mode
	//	Writing to DATA in Master Mode

	while( sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_SYSOP );
}

void SERCOM::SyncBusy_I2C()
{
	// Sync after all types of sync events are complete
	while( sercom->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_MASK );
}

bool SERCOM::IsBusStateIdle_I2C()
{
	SyncSysOp_I2C();

	return ( ReadValueSTATUS_BUSSTATE_I2C() == I2C::EBusState::IDLE );
}

bool SERCOM::IsBusStateOwner_I2C()
{
	SyncSysOp_I2C();

	return ( ReadValueSTATUS_BUSSTATE_I2C() == I2C::EBusState::OWNER );
}

sercom_i2cm_intflag_reg_t SERCOM::ReadRegisterINTFLAG_I2C()
{
	return sercom->I2CM.INTFLAG.reg;
}

void SERCOM::SendBusCommand_I2C( I2C::EBusCommand commandIn )
{
	// Set CTRLB.CMD to issue the specified master operation
	SetBitsCTRLB_I2C( SERCOM_I2CM_CTRLB_CMD( commandIn ) );
}

void SERCOM::PrepareNack_I2C()
{
	// Set ACKACT to NACK (0x1)
	SetBitsCTRLB_I2C( SERCOM_I2CM_CTRLB_ACKACT );
}

void SERCOM::PrepareAck_I2C()
{
	// Clear ACKACT to ACK (0x0)
	ClearBitsCTRLB_I2C( SERCOM_I2CM_CTRLB_ACKACT );
}

void SERCOM::ClearInterruptMB_I2C()
{
	// Setting INTFLAG.MB to 1 clears the interrupt
	// Note: The following other actions also clear this bit:
	//	- Writing to ADDR.ADDR
	//	- Writing to DATA.DATA
	//	- Reading DATA.DATA when Smart Mode is enabled
	//	- Writing a valid command to CTRLB.CMD
	sercom->I2CM.INTFLAG.reg = SERCOM_I2CM_INTFLAG_MB;
}

void SERCOM::ClearInterruptSB_I2C()
{
	// Setting INTFLAG.SB to 1 clears the interrupt
	// Note: The following other actions also clear this bit:
	//	- Writing to ADDR.ADDR
	//	- Writing to DATA.DATA
	//	- Reading DATA.DATA when Smart Mode is enabled
	//	- Writing a valid command to CTRLB.CMD
	sercom->I2CM.INTFLAG.reg = SERCOM_I2CM_INTFLAG_SB;
}

sercom_i2cm_status_reg_t SERCOM::ReadRegisterSTATUS_I2C()
{
	SyncSysOp_I2C();

	return sercom->I2CM.STATUS.reg;
}

uint8_t SERCOM::ReadValueCTRLA_SCLSM_I2C()
{
	return ( sercom->I2CM.CTRLA.reg & SERCOM_I2CM_CTRLA_SCLSM );
}

uint8_t SERCOM::ReadValueSTATUS_BUSSTATE_I2C()
{
	SyncSysOp_I2C();

	return ( sercom->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_BUSSTATE_Msk ) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos;
}

sercom_i2cm_data_reg_t SERCOM::ReadRegisterDATA_I2C()
{
	return sercom->I2CM.DATA.reg;
}

void SERCOM::WriteDATA_I2C( sercom_i2cm_data_reg_t dataIn )
{
	SyncSysOp_I2C();

	sercom->I2CM.DATA.reg = dataIn;
}

void SERCOM::WriteADDR_I2C( sercom_i2cm_addr_reg_t dataIn )
{
	SyncSysOp_I2C();

	sercom->I2CM.ADDR.reg = dataIn;
}

void SERCOM::SetBitsCTRLA_I2C( sercom_i2cm_ctrla_reg_t dataIn )
{
	SyncReset_I2C();
	SyncEnable_I2C();
	
	sercom->I2CM.CTRLA.reg |= dataIn;
}

void SERCOM::ClearBitsCTRLA_I2C( sercom_i2cm_ctrla_reg_t dataIn )
{
	SyncReset_I2C();
	SyncEnable_I2C();
	
	sercom->I2CM.CTRLA.reg &= ~dataIn;
}

void SERCOM::WriteCTRLA_I2C( sercom_i2cm_ctrla_reg_t dataIn )
{
	SyncReset_I2C();
	SyncEnable_I2C();
	
	sercom->I2CM.CTRLA.reg = dataIn;
}

void SERCOM::SetBitsCTRLB_I2C( sercom_i2cm_ctrlb_reg_t dataIn )
{
	SyncSysOp_I2C();
	
	sercom->I2CM.CTRLB.reg |= dataIn;
}

void SERCOM::ClearBitsCTRLB_I2C( sercom_i2cm_ctrlb_reg_t dataIn )
{
	SyncSysOp_I2C();
	
	sercom->I2CM.CTRLB.reg &= ~dataIn;
}

void SERCOM::WriteCTRLB_I2C( sercom_i2cm_ctrlb_reg_t dataIn )
{
	SyncSysOp_I2C();
	
	sercom->I2CM.CTRLB.reg = dataIn;
}

void SERCOM::SetBitsSTATUS_I2C( sercom_i2cm_status_reg_t dataIn )
{
	SyncSysOp_I2C();
	
	sercom->I2CM.STATUS.reg |= dataIn;
}

void SERCOM::ClearBitsSTATUS_I2C( sercom_i2cm_status_reg_t dataIn )
{
	SyncSysOp_I2C();
	
	sercom->I2CM.STATUS.reg &= ~dataIn;
}

void SERCOM::WriteSTATUS_I2C( sercom_i2cm_status_reg_t dataIn )
{
	SyncSysOp_I2C();
	
	sercom->I2CM.STATUS.reg = dataIn;
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
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( clockId ) | 			// SERCOM clock id
						// GCLK_CLKCTRL_ID( GCM_SERCOMx_SLOW ) |	// SERCOM slow clock. Hopefully this is idempotent
	                    GCLK_CLKCTRL_GEN_GCLK0 | 				// Generic Clock Generator 0 is source
	                    GCLK_CLKCTRL_CLKEN;

	while( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}
}
