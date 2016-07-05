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
		// They are set up to redundantly describe the EN_PROGRAM pin, which will be safe, just in case you don't see this and try.

/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 0
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 1	| XOSCIN32	
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 2	| XOSCOUT32
				
				{ PORTA	, 2		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel0		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 3	| A0
				{ PORTA	, 3		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel1		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 4	| A1
				{ PORTB	, 4		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel12		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 5	| A12
				{ PORTB	, 5		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel13		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 6	| A13
				
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 7	| GNDANA
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 8	| VDDANA		
				
				{ PORTB	, 6		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel14		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 9	| A14
				{ PORTB	, 7		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel15		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 10	| A15
				{ PORTB	, 8		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel2		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 11	| A2
				{ PORTB	, 9		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel3		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 12	| A3
				
				{ PORTA	, 4		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 13	| SERCOM0/PAD[0], 	UART0 TX
				{ PORTA	, 5		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 14	| SERCOM0/PAD[1], 	UART0 RX
				{ PORTA	, 6		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 15	| SERCOM0/PAD[2], 	UART0 RTS
				{ PORTA	, 7		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 16	| SERCOM0/PAD[3], 	UART0 CTS
				
				{ PORTA	, 8		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 17	| SERCOM2/PAD[0],  	I2C1 SDA
				{ PORTA	, 9		, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 18	| SERCOM2/PAD[1],  	I2C1 SCL
				
				{ PORTA	, 10	, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel0		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 19	| A18					
				{ PORTA	, 11	, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel0		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 20	| A19				
				
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 21	| VDDIO
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 22	| GND		
				
				{ PORTB	, 10	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 23	| Servo1 			TC5 0
				{ PORTB	, 11	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 24	| Servo2 			TC5 1
				{ PORTB	, 12	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 25	| Servo3 			TC4 0
				{ PORTB	, 13	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 26	| GPIO0
				
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 27	| N/A		
				
				{ PORTB	, 15	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 28	| GPIO1, 			LED_1
					
				{ PORTA	, 12	, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 29	| SERCOM4/PAD[0],  	UART1_TX
				{ PORTA	, 13	, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 30	| SERCOM4/PAD[1],  	UART1_RX
				
				{ PORTA	, 14	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 31	| GPIO2 			LED_0
				{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 32	| GPIO3
				
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 33	| GND
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 34	| VDDIO		
					
				{ PORTA	, 16	, PIO_SERCOM			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 35	| SERCOM1/PAD[0],  	UART2_TX
				{ PORTA	, 17	, PIO_SERCOM			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 36	| SERCOM1/PAD[1],  	UART2_RX
				
				{ PORTA	, 18	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 37	| GPIO4
				{ PORTA	, 19	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 38	| GPIO5
				
				{ PORTB	, 16	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 39	| GPIO6
				{ PORTB	, 17	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 40	| GPIO7
				{ PORTA	, 20	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 41	| GPIO8
				
				{ PORTA	, 21	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 42	| GPIO9
				
				{ PORTA	, 22	, PIO_SERCOM			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 43	| SERCOM3/PAD[0], 	UART3 TX
				{ PORTA	, 23	, PIO_SERCOM			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 44	| SERCOM3/PAD[1], 	UART3 RX
				
				{ PORTA	, 24	, PIO_TIMER_ALT			, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM1_CH2		, TCC1_CH2		, EXTERNAL_INT_NONE	}, 	// 45	| TCC1_2,	 		PWM0
				
				{ PORTA	, 25	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 46	| GPIO10
				
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 47	| GND
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 48	| VDDIO	
				
				{ PORTB	, 22	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 49	| GPIO11
				{ PORTB	, 23	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 50	| GPIO12
				{ PORTA	, 27	, PIO_OUTPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 51	| GPIO13
				
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 52	| RESETN			
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 53	| SPARE		
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 54	| GND
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 55	| VDDCORE	
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 56	| VDDIN	
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 57	| DEBUG SWCLK	
/*DUM*/			{ PORTA	, 15	, PIO_INPUT				, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 58	| DEBUG SWDIO			
				
				{ PORTB	, 30	, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 59	| SERCOM5/PAD[0], 	UART4 TX
				{ PORTB	, 31	, PIO_SERCOM_ALT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 60	| SERCOM5/PAD[1], 	UART4 RX
				
				{ PORTB	, 0		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel8		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 61	| A8
				{ PORTB	, 1		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel9		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 62	| A9
				{ PORTB	, 2		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel10		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 63	| A10
				{ PORTB	, 3		, PIO_ANALOG			, PIN_ATTR_ANALOG	, ADC_Channel11		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	} 	// 64	| A11
};

const void* g_apTCInstances[ TCC_INST_NUM+TC_INST_NUM ]= { TCC0, TCC1, TCC2, TC3, TC4, TC5, TC6, TC7 };

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 );
SERCOM sercom1( SERCOM1 );
SERCOM sercom2( SERCOM2 );
SERCOM sercom3( SERCOM3 );
SERCOM sercom4( SERCOM4 );
SERCOM sercom5( SERCOM5 );

// Instantiate Serial object
Uart Serial( &sercom0, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX );

void SERCOM0_Handler()
{
  Serial.IrqHandler();
}

// Instantiate Serial1 object
Uart Serial1( &sercom4, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX );

void SERCOM4_Handler()
{
  Serial1.IrqHandler();
}

// Instantiate Serial2 object
Uart Serial2( &sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX );

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// Instantiate Serial3 object
Uart Serial3( &sercom3, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX );

void SERCOM3_Handler()
{
  Serial3.IrqHandler();
}

// Instantiate Serial4 object
Uart Serial4( &sercom5, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX );

void SERCOM5_Handler()
{
  Serial4.IrqHandler();
}

