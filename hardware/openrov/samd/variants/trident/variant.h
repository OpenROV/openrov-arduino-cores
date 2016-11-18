#pragma once

// Defines
#define VARIANT_MAINOSC       (32768ul)     // Frequency of the board main oscillator
#define VARIANT_MCK           (48000000ul)  // Master clock frequency

// Includes
#include "WVariant.h"

#ifdef __cplusplus
  #include "SERCOM.h"
  #include "Uart.h"
#endif 

#ifdef __cplusplus
  extern "C"
  {
#endif

// Pin setup

// Number of pins defined in PinDescription array
#define PINS_COUNT                  (65u) // There are actually 64 pins, but the first pin in the pin descriptors is a dummy used to offset the pins to be 1-based
#define NUM_DIGITAL_PINS            (14u)
#define NUM_ANALOG_INPUTS           (14u)
#define NUM_ANALOG_OUTPUTS          (0u)

// Helpers for pin operations
#define digitalPinToPort(P)         ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)      ( 1 << g_APinDescription[P].ulPin )
#define portOutputRegister(port)    ( &(port->OUT.reg) )
#define portInputRegister(port)     ( &(port->IN.reg) )
#define portModeRegister(port)      ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)         ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )
#define digitalPinToInterrupt(P)    ( g_APinDescription[P].ulExtInt )

// ADC Configuration
#define ADC_RESOLUTION              10

// UART Interfaces

// UART0 (Debug)
#define PIN_SERIAL_RX       (10u)
#define PIN_SERIAL_TX       (9u)
#define PAD_SERIAL_TX       (UART_TX_PAD_0)
#define PAD_SERIAL_RX       (SERCOM_RX_PAD_1)

// UART1 (ESCA)
#define PIN_SERIAL1_RX       (22u)
#define PIN_SERIAL1_TX       (21u)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)

// UART2 (ESCB)
#define PIN_SERIAL2_RX       (26u)
#define PIN_SERIAL2_TX       (25u)
#define PAD_SERIAL2_TX       (UART_TX_PAD_0)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_1)

// UART3 (PI)
#define PIN_SERIAL3_RX       (32u)
#define PIN_SERIAL3_TX       (31u)
#define PAD_SERIAL3_TX       (UART_TX_PAD_0)
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_1)

// UART4 (ESCC)
#define PIN_SERIAL4_RX       (48u)
#define PIN_SERIAL4_TX       (47u)
#define PAD_SERIAL4_TX       (UART_TX_PAD_0)
#define PAD_SERIAL4_RX       (SERCOM_RX_PAD_1)

// SPI interfaces (None)
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


// C++ Object declarations
#ifdef __cplusplus

    // SERCOM declarations
    extern SERCOM sercom0;
    extern SERCOM sercom1;
    extern SERCOM sercom2;
    extern SERCOM sercom3;
    extern SERCOM sercom4;
    extern SERCOM sercom5;

    // UART Declarations
    extern Uart SerialDebug;
    extern Uart SerialMotorA;
    extern Uart SerialMotorB;
    extern Uart SerialCpu;
    extern Uart SerialMotorC;

    // I2C Declarations made in I2C library
    // I2C interface and interrupt handler
    #define PERIPH_WIRE          		    sercom2
    #define WIRE_IT_HANDLER      		    SERCOM2_Handler
#endif



