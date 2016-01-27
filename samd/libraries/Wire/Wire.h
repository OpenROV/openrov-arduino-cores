/*
 * TWI/I2C library for Arduino Zero
 * Copyright (c) 2015 Arduino LLC. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef TwoWire_h
#define TwoWire_h

#include "Stream.h"
#include "variant.h"

#include "SERCOM.h"
#include "RingBuffer.h"

#define BUFFER_LENGTH 32

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

class TwoWire : public Stream
{
  public:
    TwoWire(SERCOM *s, uint8_t pinSDA, uint8_t pinSCL);
    void begin();
    void begin(uint8_t);
    void end();
    void setClock(uint32_t);

    void beginTransmission(uint8_t);
    uint8_t endTransmission(bool stopBit);
    uint8_t endTransmission(void);

    uint8_t requestFrom(uint8_t address, size_t quantity, bool stopBit);
    uint8_t requestFrom(uint8_t address, size_t quantity);

    size_t write(uint8_t data);
    size_t write(const uint8_t * data, size_t quantity);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(void));

    using Print::write;

    void onService(void);

  private:
    Sercom 			*m_pSercom;
    SERCOM 			*sercomm;
    uint8_t 		m_pinSDA;
    uint8_t 		m_pinSCL;
    
    bool 			m_transmissionBegun;

    // RX Buffer
    RingBuffer 		m_rxBuffer;

    //TX buffer
    RingBuffer 		m_txBuffer;
    uint8_t 		m_txAddress;
    
    char 			m_readBuffer[ BUFFER_LENGTH ];

    // Callback user functions
    void (*onRequestCallback)(void);
    void (*onReceiveCallback)(int);

    // TWI clock frequency
    static const uint32_t TWI_CLOCK = 100000;
    
    
    int _read( char *data, int length );
    int _start( uint8_t address, uint8_t rw_flag );
    int _stop();
    int _write( char *data, int length);
};

#if WIRE_INTERFACES_COUNT > 0
  extern TwoWire Wire;
#endif
#if WIRE_INTERFACES_COUNT > 1
  extern TwoWire Wire1;
#endif
#if WIRE_INTERFACES_COUNT > 2
  extern TwoWire Wire2;
#endif
#if WIRE_INTERFACES_COUNT > 3
  extern TwoWire Wire3;
#endif
#if WIRE_INTERFACES_COUNT > 4
  extern TwoWire Wire4;
#endif
#if WIRE_INTERFACES_COUNT > 5
  extern TwoWire Wire5;
#endif

#endif
