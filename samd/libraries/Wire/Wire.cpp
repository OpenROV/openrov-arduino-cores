/*
    TWI/I2C library for Arduino Zero
    Copyright (c) 2015 Arduino LLC. All rights reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

extern "C" {
#include <string.h>
}

#include <Arduino.h>
#include <wiring_private.h>

#include "Wire.h"

#define SAMD21_I2C_TIMEOUT_MS  (10)

#define I2C_FLAG_WRITE      0
#define I2C_FLAG_READ       1

using namespace i2c;

TwoWire::TwoWire( SERCOM *s, uint8_t pinSDA, uint8_t pinSCL )
{
	sercomm						= s;
	m_pinSDA					= pinSDA;
	m_pinSCL					= pinSCL;
	m_transmissionBegun 		= false;
	
}

void TwoWire::begin( void )
{
	// Initialize in master mode
	sercomm->InitMasterMode_TWI( DEFAULT_TWI_CLOCK );
	sercomm->Enable_TWI();

	pinPeripheral( m_pinSDA, g_APinDescription[m_pinSDA].ulPinType );
	pinPeripheral( m_pinSCL, g_APinDescription[m_pinSCL].ulPinType );
}

void TwoWire::setClock( uint32_t baudRateIn )
{
	// Retinitialize with new baud rate
	sercomm->InitMasterMode_TWI( baudRateIn );
	sercomm->Enable_TWI();
}

void TwoWire::end()
{
	sercomm->Disable_TWI();
}

uint8_t TwoWire::requestFrom( uint8_t address, size_t quantity, bool issueRepeatedStart )
{
	if( quantity > BUFFER_LENGTH )
	{
		return 0;
	}

	// Start transmission
	int ret = sercomm->StartTransmission_TWI( address, ETWIReadWriteFlag::READ );
	if( ret < 0 )
	{
		return 0;
	}

	// Read first data
	int bytesRead = sercomm->ReadAsMaster_TWI( m_intermediateReadBuffer, quantity, issueRepeatedStart );

	if( bytesRead < 0 )
	{
		return 0;
	}

	// Move data to ringbuffer
	for( int i = 0; i < bytesRead; ++i )
	{
		m_rxBuffer.store_char( m_intermediateReadBuffer[ i ] );
	}

	return bytesRead;
}

uint8_t TwoWire::requestFrom( uint8_t address, size_t quantity )
{
	return requestFrom( address, quantity, false );
}

void TwoWire::beginTransmission( uint8_t address )
{
	// save address of target and clear buffer
	m_txAddress = address;
	m_txBuffer.clear();

	m_transmissionBegun = true;
}

// Errors:
//  0 : Success
//  1 : Data too long
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Other error
uint8_t TwoWire::endTransmission( bool issueRepeatedStart )
{
	m_transmissionBegun = false ;

	if( sercomm->StartTransmission_TWI( m_txAddress, ETWIReadWriteFlag::WRITE ) < 0 )
	{
		return 2;
	}

	int byteCount = 0;

	// Copy bytes from ringbuffer to intermediate buffer
	while( m_txBuffer.available() )
	{
		m_intermediateWriteBuffer[ byteCount++ ] = m_txBuffer.read_char();
	}
	
	if( byteCount > BUFFER_LENGTH )
	{
		return 1;
	}
	
	int ret = sercomm->WriteAsMaster_TWI( m_intermediateWriteBuffer, byteCount, issueRepeatedStart );
	if( ret < 0 )
	{
		return 4;
	}
	
	return 0;

}

uint8_t TwoWire::endTransmission()
{
	return endTransmission( false );
}

// Write a single byte
size_t TwoWire::write( uint8_t ucData )
{
	// No writing, without begun transmission or a full buffer
	if( !m_transmissionBegun || m_txBuffer.isFull() )
	{
		return 0 ;
	}

	m_txBuffer.store_char( ucData ) ;

	return 1 ;
}

// Write multiple bytes
size_t TwoWire::write( const uint8_t *data, size_t quantity )
{
	//Try to store all data
	for( size_t i = 0; i < quantity; ++i )
	{
		//Return the number of data stored, when the buffer is full (if write return 0)
		if( !write( data[i] ) )
		{
			return i;
		}
	}

	//All data stored
	return quantity;
}

int TwoWire::available( void )
{
	return m_rxBuffer.available();
}

int TwoWire::read( void )
{
	return m_rxBuffer.read_char();
}

int TwoWire::peek( void )
{
	return m_rxBuffer.peek();
}

void TwoWire::flush( void )
{
	// Do nothing, use endTransmission(..) to force
	// data transfer.
}

void TwoWire::onReceive( void( *function )( int ) )
{
	onReceiveCallback = function;
}

void TwoWire::onRequest( void( *function )( void ) )
{
	onRequestCallback = function;
}

void TwoWire::onService( void )
{
	// // We never use this really
	// if( sercom->isSlaveWIRE() )
	// {

	// 	if( sercom->isStopDetectedWIRE() ||
	// 	    ( sercom->isAddressMatch() && sercom->isRestartDetectedWIRE() && !sercom->isMasterReadOperationWIRE() ) ) //Stop or Restart detected
	// 	{
	// 		sercom->prepareAckBitWIRE();
	// 		sercom->prepareCommandBitsWire( WIRE_MASTER_ACT_STOP );

	// 		//Calling onReceiveCallback, if exists
	// 		if( onReceiveCallback )
	// 		{
	// 			onReceiveCallback( available() );
	// 		}

	// 		rxBuffer.clear();
	// 	}
	// 	else if( sercom->isAddressMatch() ) //Address Match
	// 	{
	// 		sercom->prepareAckBitWIRE();
	// 		sercom->prepareCommandBitsWire( WIRE_MASTER_ACT_STOP );

	// 		if( sercom->isMasterReadOperationWIRE() ) //Is a request ?
	// 		{
	// 			// wait for data ready flag,
	// 			// before calling request callback
	// 			while( !sercom->isDataReadyWIRE() );

	// 			//Calling onRequestCallback, if exists
	// 			if( onRequestCallback )
	// 			{
	// 				onRequestCallback();
	// 			}
	// 		}
	// 	}
	// 	else if( sercom->isDataReadyWIRE() ) //Received data
	// 	{
	// 		if( rxBuffer.isFull() )
	// 		{
	// 			sercom->prepareNackBitWIRE();
	// 		}
	// 		else
	// 		{
	// 			//Store data
	// 			rxBuffer.store_char( sercom->readDataWIRE() );

	// 			sercom->prepareAckBitWIRE();
	// 		}

	// 		sercom->prepareCommandBitsWire( WIRE_MASTER_ACT_STOP );
	// 	}
	// }
}

#if WIRE_INTERFACES_COUNT > 0
/*  In case new variant doesn't define these macros,
    we put here the ones for Arduino Zero.

    These values should be different on some variants!
*/

TwoWire Wire( &PERIPH_WIRE, PIN_WIRE_SDA, PIN_WIRE_SCL );

void WIRE_IT_HANDLER( void )
{
	Wire.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 1
TwoWire Wire1( &PERIPH_WIRE1, PIN_WIRE1_SDA, PIN_WIRE1_SCL );

void WIRE1_IT_HANDLER( void )
{
	Wire1.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 2
TwoWire Wire2( &PERIPH_WIRE2, PIN_WIRE2_SDA, PIN_WIRE2_SCL );

void WIRE2_IT_HANDLER( void )
{
	Wire2.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 3
TwoWire Wire3( &PERIPH_WIRE3, PIN_WIRE3_SDA, PIN_WIRE3_SCL );

void WIRE3_IT_HANDLER( void )
{
	Wire3.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 4
TwoWire Wire4( &PERIPH_WIRE4, PIN_WIRE4_SDA, PIN_WIRE4_SCL );

void WIRE4_IT_HANDLER( void )
{
	Wire4.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 5
TwoWire Wire5( &PERIPH_WIRE5, PIN_WIRE5_SDA, PIN_WIRE5_SCL );

void WIRE5_IT_HANDLER( void )
{
	Wire5.onService();
}
#endif

