/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

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
/*
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * + Pin number +  Trident pin     |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital Low      |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */


#include "variant.h"

// Shorthand to collapse the table some more
#define PIN_ATTRIBUTE_PWM 	(PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)

// Pin Descriptions for Rovduino
const PinDescription g_APinDescription[] =
{ 
		//		| PORT 	| PIN 	| PIN_TYPE				| PIN_ATTRIBUTES	| ADC_CHANNEL		| PWM_CHANNEL	| TIMER_CHANNEL	| EXT. INTERRUPT	|	// Pin	| Description 

		// NOTE!
		// DUM(MY) pins should not be used. Their purpose is to make the array index of these pin descriptors match the pin numbers on the board. 
		// Some of them describe actual pins, just not GPIO pins, such as the oscillator, power, ground, debug, etc pins. Just FYI.
		// They are set up to redundantly describe the unused PA28 pin, which will be safe, just in case you don't see this and try.

/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 0	| DOES NOT EXIST
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 1	| XOSCIN32	
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 2	| XOSCOUT32
				{ PORTA	, 2		, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 3	| N/A
				{ PORTA	, 3		, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 4	| N/A		
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 5	| GNDANA
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 6	| VDDANA
				{ PORTB	, 8		, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 7	| N/A
				{ PORTB	, 9		, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 8	| N/A
				{ PORTA	, 4		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 9	| SERCOM0/PAD[0], 	UART0 TX
				{ PORTA	, 5		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 10	| SERCOM0/PAD[1], 	UART0 RX
				{ PORTA	, 6		, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 11	| N/A
				{ PORTA	, 7		, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 12	| N/A
				

				{ PORTA	, 8		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 13	| SERCOM2/PAD[0],  	I2C1 SDA
				{ PORTA	, 9		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 14	| SERCOM2/PAD[1],  	I2C1 SCL
				{ PORTA	, 10	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 15	| I2C0_INT (Unused)
				{ PORTA	, 11	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 16	| I2C0_PWR
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 17	| VDDIO
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 18	| GND		
				{ PORTB	, 10	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 19	| N/A
				{ PORTB	, 11	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 20	| N/A
				{ PORTA	, 12	, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 21	| SERCOM4/PAD[0],  	UART1_TX
				{ PORTA	, 13	, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 22	| SERCOM4/PAD[1],  	UART1_RX
				{ PORTA	, 14	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 23	| N/A
				{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 24	| N/A


				{ PORTA	, 16	, PIO_SERCOM			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 25	| SERCOM1/PAD[0],  	UART2_TX
				{ PORTA	, 17	, PIO_SERCOM			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 26	| SERCOM1/PAD[1],  	UART2_RX
				{ PORTA	, 18	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 27	| N/A
				{ PORTA	, 19	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 28	| N/A		
				{ PORTA	, 20	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 29	| N/A
				{ PORTA	, 21	, PIO_TIMER_ALT			, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH7		, TCC0_CH7		, EXTERNAL_INT_NONE	}, 	// 30	| TCC0_7,	 		LED_PWM
				{ PORTA	, 22	, PIO_SERCOM			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 31	| SERCOM3/PAD[0], 	UART3 TX
				{ PORTA	, 23	, PIO_SERCOM			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 32	| SERCOM3/PAD[1], 	UART3 RX
				{ PORTA	, 24	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 33	| N/A
				{ PORTA	, 25	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 34	| N/A	
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 35	| GND
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 36	| VDDIO


				{ PORTB	, 22	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 37	| LED2
				{ PORTB	, 23	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 38	| LED1
				{ PORTA	, 27	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 39	| LED0
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 40	| RESETN	
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 41	| N/A
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 42	| GND
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 43	| VDDCORE	
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 44	| VDDIN	
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 45	| DEBUG SWCLK	
/*DUM*/			{ PORTA	, 28	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 46	| DEBUG SWDIO			
				{ PORTB	, 2		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 47	| SERCOM5/PAD[0], 	UART4 TX
				{ PORTB	, 3		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 48	| SERCOM5/PAD[1], 	UART4 RX
};

const void* g_apTCInstances[ TCC_INST_NUM+TC_INST_NUM ]= { TCC0, TCC1, TCC2, TC3, TC4, TC5, TC6, TC7 };

// Instantiate the sercom objects
SERCOM sercom0( SERCOM0 );
SERCOM sercom1( SERCOM1 );
SERCOM sercom2( SERCOM2 );
SERCOM sercom3( SERCOM3 );
SERCOM sercom4( SERCOM4 );
SERCOM sercom5( SERCOM5 );

// Instantiate SerialDebug object
Uart SerialDebug( &sercom0, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX );

void SERCOM0_Handler()
{
  SerialDebug.IrqHandler();
}

// Instantiate SerialMotorB object
Uart SerialMotorA( &sercom4, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX );

void SERCOM4_Handler()
{
  SerialMotorA.IrqHandler();
}

// Instantiate Serial2 object
Uart SerialMotorB( &sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX );

void SERCOM1_Handler()
{
  SerialMotorB.IrqHandler();
}

// Instantiate SerialCpu object
Uart SerialCpu( &sercom3, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX );

void SERCOM3_Handler()
{
  SerialCpu.IrqHandler();
}

// Instantiate Serial4 object
Uart SerialMotorC( &sercom5, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX );

void SERCOM5_Handler()
{
  SerialMotorC.IrqHandler();
}

