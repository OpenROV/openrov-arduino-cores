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

// Pin Descriptions for Rovduino
const PinDescription g_APinDescription[] =
{ 
//	| PORT 	| PIN 	| PIN_TYPE			| PIN_ATTRIBUTES	| ADC_CHANNEL		| PWM_CHANNEL	| TIMER_CHANNEL	| EXT. INTERRUPT	|	// Pin	| Description 

	// External 32khz Oscillator - XOSC32 I/O pins dont need explicit config, handled automatically. Crystal attached
//	{ PORTA	, 0		, N/A				, N/A				, N/A				, N/A			, N/A			, N/A				}, 	// 0 	| XIN32
//	{ PORTA	, 1		, N/A				, N/A				, N/A				, N/A			, N/A			, N/A				}, 	// 1	| XOUT32

	// Analog Power
//	{ N/A	, N/A	, N/A				, N/A				, N/A				, N/A			, N/A			, N/A				}, 	// 6 	| GNDANA
//	{ N/A	, N/a	, N/A				, N/A				, N/A				, N/A			, N/A			, N/A				}, 	// 7	| VDDANA

	// Analog inputs
	{ PORTA	, 2		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel0		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 2	| ADC/AIN[0], A0
	{ PORTA	, 3		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel1		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 3	| ADC/AIN[1], A1
	{ PORTB	, 4		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel12		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 4	| ADC/AIN[12], A12
	{ PORTB	, 5		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel13		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 5	| ADC/AIN[13], A13
	{ PORTB	, 6		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel14		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 8	| ADC/AIN[14], A14
	{ PORTB	, 7		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel15		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 9	| ADC/AIN[15], A15
	{ PORTB	, 8		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel2		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 10	| ADC/AIN[2], A2
	{ PORTB	, 9		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel3		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 11	| ADC/AIN[3], A3
	{ PORTB	, 0		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel8		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 60	| ADC/AIN[8], A8
	{ PORTB	, 1		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel9		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 61	| ADC/AIN[9], A9
	{ PORTB	, 2		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel10		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 62	| ADC/AIN[10], A10
	{ PORTB	, 3		, PIO_ANALOG		, PIN_ATTR_ANALOG	, ADC_Channel11		, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 63	| ADC/AIN[11], A11
	
	// Digital Outputs
	{ PORTB	, 15	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 27	| ADC/AIN[11], A11
	{ PORTA	, 14	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 30	| ADC/AIN[11], A11
	{ PORTA	, 15	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 31	| ADC/AIN[11], A11
	{ PORTA	, 21	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 41	| ADC/AIN[11], A11
	{ PORTB	, 22	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 48	| ADC/AIN[11], A11
	{ PORTB	, 23	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 49	| ADC/AIN[11], A11
	{ PORTA	, 27	, PIO_OUTPUT		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 50	| ADC/AIN[11], A11
	
	// UART0
	{ PORTA	, 4		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 12	| TX, SERCOM0/PAD[0]
  	{ PORTA	, 5		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 13	| RX, SERCOM0/PAD[1]
  	{ PORTA	, 6		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 14	| RTS, SERCOM0/PAD[2]
  	{ PORTA	, 7		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 15	| CTS, SERCOM0/PAD[3]
	  
	// UART1
	{ PORTA	, 22	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 42	| TX, SERCOM3/PAD[0]
  	{ PORTA	, 23	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 43	| RX, SERCOM3/PAD[1]
	  
	// I2C 0
	{ PORTA	, 8		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 16	| TX, SERCOM2/PAD[0]
  	{ PORTA	, 9		, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 17	| RX, SERCOM2/PAD[1]
	  
	// I2C 1
	{ PORTA	, 12	, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 16	| TX, SERCOM4/PAD[0]
  	{ PORTA	, 13	, PIO_SERCOM_ALT	, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 17	| RX, SERCOM4/PAD[1]
	  
	// SPI 0
	{ PORTB	, 16	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 38	| MOSI, SERCOM5/PAD[0]
  	{ PORTB	, 17	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 39	| SCK, SERCOM5/PAD[1]
	{ PORTA	, 20	, PIO_SERCOM		, PIN_ATTR_DIGITAL	, No_ADC_Channel	, NOT_ON_PWM	, NOT_ON_TIMER	, EXTERNAL_INT_NONE	}, 	// 40	| MISO, SERCOM5/PAD[2]  
	
	// PWM Channels 
	
	// TCC0
	{ PORTB	, 10	, PIO_TIMER_ALT		, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM0_CH4	, TCC0_CH4	, EXTERNAL_INT_NONE	}, 	// 22	| ADC/AIN[11], A11
	{ PORTB	, 11	, PIO_TIMER_ALT		, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM0_CH5	, TCC0_CH5	, EXTERNAL_INT_NONE	}, 	// 23	| ADC/AIN[11], A11
	{ PORTB	, 12	, PIO_TIMER_ALT		, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM0_CH6	, TCC0_CH6	, EXTERNAL_INT_NONE	}, 	// 24	| ADC/AIN[11], A11
	{ PORTB	, 13	, PIO_TIMER_ALT		, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM0_CH7	, TCC0_CH7	, EXTERNAL_INT_NONE	}, 	// 25	| ADC/AIN[11], A11
	
	{ PORTA	, 18	, PIO_TIMER_ALT		, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM0_CH2	, TCC0_CH2	, EXTERNAL_INT_NONE	}, 	// 36	| ADC/AIN[11], A11
	{ PORTA	, 19	, PIO_TIMER_ALT		, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM0_CH3	, TCC0_CH3	, EXTERNAL_INT_NONE	}, 	// 37	| ADC/AIN[11], A11
	{ PORTB	, 30	, PIO_TIMER			, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM0_CH0	, TCC0_CH0	, EXTERNAL_INT_NONE	}, 	// 58	| ADC/AIN[11], A11
	{ PORTB	, 31	, PIO_TIMER			, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM0_CH1	, TCC0_CH1	, EXTERNAL_INT_NONE	}, 	// 59	| ADC/AIN[11], A11
	
	// TCC1
	{ PORTA	, 10	, PIO_TIMER			, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM1_CH0	, TCC1_CH0	, EXTERNAL_INT_NONE	}, 	// 18	| ADC/AIN[11], A11
	{ PORTA	, 11	, PIO_TIMER			, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM1_CH1	, TCC1_CH1	, EXTERNAL_INT_NONE	}, 	// 19`	| ADC/AIN[11], A11
	{ PORTA	, 24	, PIO_TIMER_ALT		, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM1_CH2	, TCC1_CH2	, EXTERNAL_INT_NONE	}, 	// 44	| ADC/AIN[11], A11
	{ PORTA	, 25	, PIO_TIMER_ALT		, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM1_CH3	, TCC1_CH3	, EXTERNAL_INT_NONE	}, 	// 45	| ADC/AIN[11], A11
	
	// TCC2
	{ PORTA	, 16	, PIO_TIMER			, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM2_CH0	, TCC2_CH0	, EXTERNAL_INT_NONE	}, 	// 34	| ADC/AIN[11], A11
	{ PORTA	, 17	, PIO_TIMER			, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER)	, No_ADC_Channel	, PWM2_CH1	, TCC2_CH1	, EXTERNAL_INT_NONE	} 	// 35	| ADC/AIN[11], A11
};

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5, TC6, TC7 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial( &sercom0, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;
void SERCOM0_Handler()
{
  Serial.IrqHandler();
}

Uart Serial1( &sercom3, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
void SERCOM3_Handler()
{
  Serial1.IrqHandler();
}

