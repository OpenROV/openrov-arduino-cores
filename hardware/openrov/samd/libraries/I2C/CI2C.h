#pragma once

#include "SERCOM.h"

// Options
// TODO

class CI2C
{
public:
  // Methods
  CI2C( SERCOM *s, uint8_t pinSDA, uint8_t pinSCL );
  virtual ~CI2C();

  I2C::ERetCode Enable( uint32_t baudRateIn, uint16_t optionsIn );
  I2C::ERetCode Disable();
  I2C::ERetCode Scan();

  bool IsAvailable();

  // Write operations
  I2C::ERetCode WriteByte( uint8_t slaveAddressIn, uint8_t dataIn );
  I2C::ERetCode WriteByte( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t dataIn );
  I2C::ERetCode WriteWord( uint8_t slaveAddressIn, uint16_t dataIn );
  I2C::ERetCode WriteWord( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t dataIn );

  I2C::ERetCode WriteBytes( uint8_t slaveAddressIn, uint8_t *dataIn, uint8_t numberBytesIn );
  I2C::ERetCode WriteBytes( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t *dataIn, uint8_t numberBytesIn );
  I2C::ERetCode WriteWords( uint8_t slaveAddressIn, uint16_t *dataIn, uint8_t numberWordsIn );
  I2C::ERetCode WriteWords( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t *dataIn, uint8_t numberWordsIn );

  // Direct read operations (Uses user provided buffer)
  I2C::ERetCode ReadByte( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t &dataOut );
  I2C::ERetCode ReadWord( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t &dataOut );

  I2C::ERetCode ReadBytes( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t *dataOut, uint8_t numberBytesIn );
  I2C::ERetCode ReadWords( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t *dataOut, uint8_t numberWordsIn );

  // Buffered read operations (Uses internal buffer)
  I2C::ERetCode ReadBytes_Buffered( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t *dataOut, uint8_t numberBytesIn );
  I2C::ERetCode ReadWords_Buffered( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t *dataOut, uint8_t numberWordsIn );

  uint8_t NextByte();
  uint16_t NextWord();

  uint8_t BytesAvailable();

private:
  // Attributes
  I2C::TTransaction m_transaction         = {0};

  const uint8_t kBufferSize_bytes         = 32;

  uint8_t m_pBuffer[ kBufferSize_bytes ]  = {0};
  uint8_t m_bytesAvailable                = 0;

  // Methods
  I2C::ERetCode PerformTransfer();
};
