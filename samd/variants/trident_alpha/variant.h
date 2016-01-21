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

#pragma once

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC       (32768ul)

/** Master clock frequency */
#define VARIANT_MCK           (48000000ul)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (65u) // There are actually 64 pins, but the first pin in the pin descriptors is a dummy used to offset the pins to be 1-based
#define NUM_DIGITAL_PINS     (7u)
#define NUM_ANALOG_INPUTS    (12u)
#define NUM_ANALOG_OUTPUTS   (0u)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

// Interrupts
#define digitalPinToInterrupt(P)   ( g_APinDescription[P].ulExtInt )

// LEDs
#define PIN_LED_0          	(31u)
#define PIN_LED_1          	(28u)

// Other GPIO
#define PIN_EN_PROGRAM     	(32u)
#define PIN_EN_INTI2C   	(51u)
#define PIN_EN_EXTI2C  		(50u)
#define PIN_EN_ESC        	(42u)
#define PIN_ESC_PRECHARGE 	(49u)

#define PIN_PWM_1   (35u)
#define PIN_PWM_2 	(36u)
#define PIN_PWM_3   (45u)
#define PIN_PWM_4 	(46u)
#define PIN_PWM_5   (19u)
#define PIN_PWM_6 	(20u)

// Analog Pins
#define PIN_A0 				(3u)
#define PIN_A1 				(4u)
#define PIN_A2 				(11u)
#define PIN_A3  			(12u)
#define PIN_A8   			(61u)
#define PIN_A9   			(62u)
#define PIN_A10  			(63u)
#define PIN_A11 			(64u)
#define PIN_A12 			(5u)
#define PIN_A13   			(6u)
#define PIN_A14   			(9u)
#define PIN_A15   			(10u)

#define PIN_LED  			PIN_LED_0
#define LED_BUILTIN 		PIN_LED_0

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A8  = PIN_A8 ;
static const uint8_t A9  = PIN_A9 ;
static const uint8_t A10 = PIN_A10 ;
static const uint8_t A11 = PIN_A11 ;
static const uint8_t A12 = PIN_A12 ;
static const uint8_t A13 = PIN_A13 ;
static const uint8_t A14 = PIN_A14 ;
static const uint8_t A15 = PIN_A15 ;

#define ADC_RESOLUTION      12

// UART Interfaces

// UART0 (Primary Serial Port wth CTS+RTS)
#define PIN_SERIAL_RX       (14u)
#define PIN_SERIAL_TX       (13u)
#define PIN_SERIAL_RTS      (15u)
#define PIN_SERIAL_CTS      (16u)
#define PAD_SERIAL_TX       (UART_TX_RTS_CTS_PAD_0_2_3)
#define PAD_SERIAL_RX       (SERCOM_RX_PAD_1)

// UART1 (Programming/aux serial port)
#define PIN_SERIAL1_RX       (44u)
#define PIN_SERIAL1_TX       (43u)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)

// SPI interfaces
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (41u)
// #define PIN_SPI_SS           (43u)
#define PIN_SPI_MOSI         (39u)
#define PIN_SPI_SCK          (40u)

static const uint8_t MISO = PIN_SPI_MISO;
// static const uint8_t SS	  = PIN_SPI_SS ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t SCK  = PIN_SPI_SCK ;


// I2C Interfaces
#define WIRE_INTERFACES_COUNT 2

// SERCOM4
#define PIN_WIRE_SDA         (29u)
#define PIN_WIRE_SCL         (30u)

// SERCOM2
#define PIN_WIRE1_SDA         (17u)
#define PIN_WIRE1_SCL         (18u)

// Unused USB pins
#define PIN_USB_DM 0
#define PIN_USB_DP 0
#define PIN_USB_HOST_ENABLE 0

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*  =========================
 *  ===== SERCOM DEFINITION
 *  =========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial;
extern Uart Serial1;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.

// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial

#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial1

#define PERIPH_WIRE          		    sercom4
#define WIRE_IT_HANDLER      		    SERCOM4_Handler

#define PERIPH_WIRE1          		  sercom2
#define WIRE1_IT_HANDLER      		  SERCOM2_Handler

// TODO: What the hell is this
#define PERIPH_SPI           		sercom5
#define PAD_SPI_TX           		SPI_PAD_2_SCK_3
#define PAD_SPI_RX           		SERCOM_RX_PAD_0

