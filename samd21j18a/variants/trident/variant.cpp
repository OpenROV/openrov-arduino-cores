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
 * + Pin number +  Autonomo pin    |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
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
	//	| PORT 	| PIN 	| PIN_TYPE			| PIN_ATTRIBUTES	| ADC_CHANNEL		| PWM_CHANNEL	| TIMER_CHANNEL	| EXT. INTERRUPT	|	// Pin	| Description 

		// NOTE!
		// DUM(MY) pins should not be used. Their purpose is to make the array index of these pin descriptors match the pin numbers on the board. 
		// Some of them describe actual pins, just not GPIO pins, such as the oscillator, power, ground, debug, etc pins. Just FYI.
		// They are set up to redundantly describe the EN_PROGRAM pin, which will be safe, just in case you don't see this and try.

/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 0
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 1	| XOSCIN32	
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 2	| XOSCOUT32

		{ PORTA	, 2		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel0		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 3	| A0, 				Batt1_V
		{ PORTA	, 3		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel1		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 4	| A1, 				Batt2_V
		{ PORTB	, 4		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel12		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 5	| A12, 				ExtLoad_I
		{ PORTB	, 5		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel13		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 6	| A13, 				ESC1_I
		
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 7	| GNDANA
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 8	| VDDANA		
		
		{ PORTB	, 6		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel14		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 9	| A14, 				BattBus_V
		{ PORTB	, 7		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel15		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 10	| A15, 				ESC2_I
		{ PORTB	, 8		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel2		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 11	| A2, 				NonESC_I
		{ PORTB	, 9		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel3		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 12	| A3, 				ESC3_I
		
		{ PORTA	, 4		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 13	| SERCOM0/PAD[0], 	UART0 TX
		{ PORTA	, 5		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 14	| SERCOM0/PAD[1], 	UART0 RX
		{ PORTA	, 6		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 15	| SERCOM0/PAD[2], 	UART0 RTS
		{ PORTA	, 7		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 16	| SERCOM0/PAD[3], 	UART0 CTS
		
		{ PORTA	, 8		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 17	| SERCOM2/PAD[0],  	I2C1 SDA
		{ PORTA	, 9		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 18	| SERCOM2/PAD[1],  	I2C1 SCL
		
		{ PORTA	, 10	, PIO_TIMER			, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM1_CH0		, TCC1_CH0		, EXTERNAL_INT_NONE	}, 	// 19	| TCC1_0, 			PWM5
		{ PORTA	, 11	, PIO_TIMER			, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM1_CH1		, TCC1_CH1		, EXTERNAL_INT_NONE	}, 	// 20	| TCC1_1, 			PWM6

/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 21	| VDDIO
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 22	| GND		
		
		{ PORTB	, 10	, PIO_TIMER_ALT		, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH4		, TCC0_CH4		, EXTERNAL_INT_NONE	}, 	// 23	| TCC0_4, 			Servo1 (Motor1)
		{ PORTB	, 11	, PIO_TIMER_ALT		, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH5		, TCC0_CH5		, EXTERNAL_INT_NONE	}, 	// 24	| TCC0_5, 			Servo2 (Motor2)
		{ PORTB	, 12	, PIO_TIMER_ALT		, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH6		, TCC0_CH6		, EXTERNAL_INT_NONE	}, 	// 25	| TCC0_6, 			Servo3 (Motor3)
		{ PORTB	, 13	, PIO_TIMER_ALT		, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH7		, TCC0_CH7		, EXTERNAL_INT_NONE	}, 	// 26	| TCC0_7, 			Servo4 (Motor4)
		
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 27	| N/A		
		
		{ PORTB	, 15	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 28	| GPIO6, 			LED_1
			
		{ PORTA	, 12	, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 29	| SERCOM4/PAD[0],  	I2C0 SDA
		{ PORTA	, 13	, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 30	| SERCOM4/PAD[1],  	I2C0 SCL
		
		{ PORTA	, 14	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 31	| GPIO5, 			LED_0
		{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 32	| GPIO4, 			EN_PROGRAM
		
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 33	| GND
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 34	| VDDIO		
			
		{ PORTA	, 16	, PIO_TIMER			, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM2_CH0		, TCC2_CH0		, EXTERNAL_INT_NONE	}, 	// 35	| TCC2_0, 			PWM1 (IntLights)
		{ PORTA	, 17	, PIO_TIMER			, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM2_CH1		, TCC2_CH1		, EXTERNAL_INT_NONE	}, 	// 36	| TCC2_1, 			PWM2 (Lasers)
		
		{ PORTA	, 18	, PIO_TIMER_ALT		, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH2		, TCC0_CH2		, EXTERNAL_INT_NONE	}, 	// 37	| TCC0_2, 			Servo5 (Motor5)
		{ PORTA	, 19	, PIO_TIMER_ALT		, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH3		, TCC0_CH3		, EXTERNAL_INT_NONE	}, 	// 38	| TCC0_3, 			Servo6 (Motor6)
		
		{ PORTB	, 16	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 39	| SERCOM5/PAD[0], 	SPI0 MOSI
		{ PORTB	, 17	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 40	| SERCOM5/PAD[1], 	SPI0 SCK
		{ PORTA	, 20	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 41	| SERCOM5/PAD[2], 	SPI0 MISO
		
		{ PORTA	, 21	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 42	| GPIO3, 			EN_ESC
		
		{ PORTA	, 22	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 43	| SERCOM3/PAD[0], 	UART1 TX
		{ PORTA	, 23	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 44	| SERCOM3/PAD[1], 	UART1 RX
		
		{ PORTA	, 24	, PIO_TIMER_ALT		, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM1_CH2		, TCC1_CH2		, EXTERNAL_INT_NONE	}, 	// 45	| TCC1_2,	 		PWM3 (ExtLights)
		{ PORTA	, 25	, PIO_TIMER_ALT		, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM1_CH3		, TCC1_CH3		, EXTERNAL_INT_NONE	}, 	// 46	| TCC1_2, 			PWM4
		
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 47	| GND
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 48	| VDDIO	

		{ PORTB	, 22	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 49	| GPIO2, 			ESC_PRECHARGE
		{ PORTB	, 23	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 50	| GPIO1, 			EN_EXTI2C
		{ PORTA	, 27	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 51	| GPIO0, 			EN_INTI2C
		
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 52	| RESETN			
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 53	| SPARE		
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 54	| GND
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 55	| VDDCORE	
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 56	| VDDIN	
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 57	| DEBUG SWCLK	
/*DUM*/	{ PORTA	, 15	, PIO_INPUT			, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	},	// 58	| DEBUG SWDIO			
		
		{ PORTB	, 30	, PIO_TIMER			, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH0		, TCC0_CH0		, EXTERNAL_INT_NONE	}, 	// 59	| TCC0_0, 			Servo7 (CamTilt)
		{ PORTB	, 31	, PIO_TIMER			, PIN_ATTRIBUTE_PWM	, No_ADC_Channel	, PWM0_CH1		, TCC0_CH1		, EXTERNAL_INT_NONE	}, 	// 60	| TCC0_1, 			Servo8 (ExtServo)
		
		{ PORTB	, 0		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel8		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 61	| A8,				BattTube1_I
		{ PORTB	, 1		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel9		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 62	| A9,				BattTube2_I
		{ PORTB	, 2		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel10		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 63	| A10,				HUMIDITY
		{ PORTB	, 3		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel11		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	} 	// 64	| A11,				BoardTemp
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
Uart Serial1( &sercom3, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX );

void SERCOM3_Handler()
{
  Serial1.IrqHandler();
}

