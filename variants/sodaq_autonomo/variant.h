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

#ifndef _VARIANT_SODAQ_AUTONOMO_
#define _VARIANT_SODAQ_AUTONOMO_

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
#define PINS_COUNT           (64u)
#define NUM_DIGITAL_PINS     (7u)
#define NUM_ANALOG_INPUTS    (12u)
#define NUM_ANALOG_OUTPUTS   (0u)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// Interrupts
#define digitalPinToInterrupt(P)   ( g_APinDescription[P].ulExtInt )

// LEDs
#define PIN_LED_0          (30u)
#define PIN_LED_1          (28u)
#define PIN_LED             PIN_LED_0
#define LED_BUILTIN         PIN_LED_0

// Other GPIO
#define PIN_EN_PROGRAM          (31u)
#define PIN_EN_INTI2C           (50u)
#define PIN_EN_EXTI2C           (49u)
#define PIN_EN_ESC              (41u)
#define PIN_ESC_PRECHARGE       (48u)

// Analog Pins
#define PIN_A0               (2u)
#define PIN_A1               (3u)
#define PIN_A2               (10u)
#define PIN_A3               (11u)
// #define PIN_A4               (PIN_A0 + 4)
// #define PIN_A5               (PIN_A0 + 5)
// #define PIN_A6               (PIN_A0 + 6)
// #define PIN_A7               (PIN_A0 + 7)
#define PIN_A8               (60u)
#define PIN_A9               (61u)
#define PIN_A10              (62u)
#define PIN_A11              (63u)
#define PIN_A12              (4u)
#define PIN_A13              (5u)
#define PIN_A14              (8u)
#define PIN_A15              (9u)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
// static const uint8_t A4  = PIN_A4 ;
// static const uint8_t A5  = PIN_A5 ;
// static const uint8_t A6  = PIN_A6 ;
// static const uint8_t A7  = PIN_A7 ;
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
#define PIN_SERIAL_RX       (13u)
#define PIN_SERIAL_TX       (12u)
#define PIN_SERIAL_RTS      (14u)
#define PIN_SERIAL_CTS      (15u)
#define PAD_SERIAL_TX       (UART_TX_RTS_CTS_PAD_0_2_3)
#define PAD_SERIAL_RX       (SERCOM_RX_PAD_1)

// UART1 (Programming/aux serial port)
#define PIN_SERIAL1_RX       (43u)
#define PIN_SERIAL1_TX       (42u)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)

// SPI interfaces
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (40u)
// #define PIN_SPI_SS           (43u)
#define PIN_SPI_MOSI         (38u)
#define PIN_SPI_SCK          (39u)

static const uint8_t MISO = PIN_SPI_MISO;
// static const uint8_t SS	  = PIN_SPI_SS ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t SCK  = PIN_SPI_SCK ;


// Other Digital Pins
static const uint8_t VCC_SW  = (16u);
static const uint8_t BEE_VCC = (17u);

// TODO: ???
// Other Analog Pins
static const uint8_t BAT_VOLT = (6u);
static const uint8_t AREF     = (7u);
static const uint8_t DAC0     = PIN_A0; // or (35u) implications for cores/arduino/wiring_analog.c analogWrite() 

// I2C Interfaces
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (28u)
#define PIN_WIRE_SCL         (29u)

#define PIN_WIRE1_SDA         (16u)
#define PIN_WIRE1_SCL         (17u)

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
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB

// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial

#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial1

#define PERIPH_WIRE          sercom4
#define WIRE_IT_HANDLER      SERCOM4_Handler

#define PERIPH_WIRE_1          sercom2
#define WIRE_IT_HANDLER_1      SERCOM2_Handler

// TODO: What the hell is this
#define PERIPH_SPI           sercom5
#define PAD_SPI_TX           SPI_PAD_2_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_0

#endif /* _VARIANT_SODAQ_AUTONOMO */

