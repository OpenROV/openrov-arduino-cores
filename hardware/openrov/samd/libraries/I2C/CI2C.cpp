#include "CI2C.h"

CI2C::CI2C( SERCOM *s, uint8_t pinSDA, uint8_t pinSCL )
{

}

CI2C::~CI2C()
{
	
}

I2C::ERetCode CI2C::Enable( uint32_t baudRateIn, uint16_t optionsIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::Disable()
{

	return I2C::ERetCode::SUCCESS;
}

bool CI2C::IsAvailable()
{
	return true;
}

I2C::ERetCode CI2C::Scan()
{

	return I2C::ERetCode::SUCCESS;
}

// Write operations
I2C::ERetCode CI2C::WriteByte( uint8_t slaveAddressIn, uint8_t dataIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::WriteByte( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t dataIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::WriteWord( uint8_t slaveAddressIn, uint16_t dataIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::WriteWord( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t dataIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::WriteBytes( uint8_t slaveAddressIn, uint8_t *dataIn, uint8_t numberBytesIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::WriteBytes( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t *dataIn, uint8_t numberBytesIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::WriteWords( uint8_t slaveAddressIn, uint16_t *dataIn, uint8_t numberWordsIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::WriteWords( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t *dataIn, uint8_t numberWordsIn )
{

	return I2C::ERetCode::SUCCESS;
}

// Direct read operations (Uses user provided buffer)
I2C::ERetCode CI2C::ReadByte( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t &dataOut )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::ReadWord( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t &dataOut )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::ReadBytes( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t *dataOut, uint8_t numberBytesIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::ReadWords( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t *dataOut, uint8_t numberWordsIn )
{

	return I2C::ERetCode::SUCCESS;
}

// Buffered read operations (Uses internal buffer)
I2C::ERetCode CI2C::ReadBytes_Buffered( uint8_t slaveAddressIn, uint8_t registerIn, uint8_t *dataOut, uint8_t numberBytesIn )
{

	return I2C::ERetCode::SUCCESS;
}

I2C::ERetCode CI2C::ReadWords_Buffered( uint8_t slaveAddressIn, uint8_t registerIn, uint16_t *dataOut, uint8_t numberWordsIn )
{

	return I2C::ERetCode::SUCCESS;
}

uint8_t CI2C::NextByte()
{

	return 0;
}

uint16_t CI2C::NextWord()
{
	return 0;
}


uint8_t CI2C::BytesAvailable()
{
	return m_bytesAvailable;
}

I2C::ERetCode CI2C::PerformTransfer()
{

	return I2C::ERetCode::SUCCESS;
}