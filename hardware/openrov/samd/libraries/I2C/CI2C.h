#pragma once

#include "SERCOM.h"

// Options

class CI2C
{
public:
  // Attributes
  TI2CTransaction   m_transaction;
  uint8_t           m_mode;
  uint32_t          m_baudrate;
  
  // Methods
  CI2C();
  virtual ~CI2C();
  
  int32_t Initialize( uint32_t baudRateIn, uint16_t optionsIn );
  int32_t Deinitialize();
  int32_t Enable();
  int32_t Disable();
  int32_t SetBaudRate( uint32_t baudRateIn );
  int32_t Transfer();

private:
};
