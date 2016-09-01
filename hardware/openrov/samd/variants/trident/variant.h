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
#define NUM_DIGITAL_PINS     (14u)
#define NUM_ANALOG_INPUTS    (14u)
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

#define PIN_LED  			      PIN_LED_0
#define LED_BUILTIN 		    PIN_LED_0

#define ADC_RESOLUTION      10

// UART Interfaces

// UART0 (Primary Serial Port wth CTS+RTS)
#define PIN_SERIAL_RX       (14u)
#define PIN_SERIAL_TX       (13u)
#define PIN_SERIAL_RTS      (15u)
#define PIN_SERIAL_CTS      (16u)
#define PAD_SERIAL_TX       (UART_TX_RTS_CTS_PAD_0_2_3)
#define PAD_SERIAL_RX       (SERCOM_RX_PAD_1)

// UART1 (Programming/aux serial port)
#define PIN_SERIAL1_RX       (30u)
#define PIN_SERIAL1_TX       (29u)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)

// UART2 (Programming/aux serial port)
#define PIN_SERIAL2_RX       (36u)
#define PIN_SERIAL2_TX       (35u)
#define PAD_SERIAL2_TX       (UART_TX_PAD_0)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_1)

// UART3 (Programming/aux serial port)
#define PIN_SERIAL3_RX       (44u)
#define PIN_SERIAL3_TX       (43u)
#define PAD_SERIAL3_TX       (UART_TX_PAD_0)
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_1)

// UART4 (Programming/aux serial port)
#define PIN_SERIAL4_RX       (60u)
#define PIN_SERIAL4_TX       (59u)
#define PAD_SERIAL4_TX       (UART_TX_PAD_0)
#define PAD_SERIAL4_RX       (SERCOM_RX_PAD_1)

// SPI interfaces
#define SPI_INTERFACES_COUNT 0

// I2C Interfaces
#define WIRE_INTERFACES_COUNT 1

// SERCOM2
#define PIN_WIRE_SDA         (17u)
#define PIN_WIRE_SCL         (18u)

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
extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;

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

#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial2

#define SERIAL_PORT_HARDWARE3       Serial3
#define SERIAL_PORT_HARDWARE_OPEN3  Serial3

#define SERIAL_PORT_HARDWARE4       Serial4
#define SERIAL_PORT_HARDWARE_OPEN4  Serial4

#define PERIPH_WIRE          		    sercom2
#define WIRE_IT_HANDLER      		    SERCOM2_Handler

