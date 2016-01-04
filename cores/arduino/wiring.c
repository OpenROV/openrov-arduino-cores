/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

// System Core Clock is at 1MHz (8MHz/8) at Reset.
// It is switched to 48MHz in the Reset Handler (startup.c)
uint32_t SystemCoreClock = 1000000ul ;

// SAMD21J18A board initialization
// Good to know:
//  - At reset, ResetHandler did the system clock configuration. Core is running at 48MHz.
//  - Watchdog is disabled by default, unless someone plays with NVM User page
//  - During reset, all PORT lines are configured as inputs with input buffers, output buffers and pull disabled.

void init( void )
{
    // Set Systick to 1ms interval, common to all Cortex-M variants
    if ( SysTick_Config( SystemCoreClock / 1000 ) )
    {
        // Capture error
        while ( 1 ) ;
    }

    // PM == power manager

    // Clock SERCOM for Serial
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;

    // Clock TC/TCC for Pulse and Analog
    PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 | PM_APBCMASK_TC6 | PM_APBCMASK_TC7;

    // Clock ADC for Analog (no dac in use)
    PM->APBCMASK.reg |= PM_APBCMASK_ADC;

    // Setup digital IO pins in OUTPUT mode
    pinMode( PIN_LED_0, OUTPUT );
    pinMode( PIN_LED_1, OUTPUT );
    pinMode( PIN_EN_PROGRAM, OUTPUT );
    pinMode( PIN_EN_INTI2C, OUTPUT );
    pinMode( PIN_EN_EXTI2C, OUTPUT );
    pinMode( PIN_EN_ESC, OUTPUT );
    pinMode( PIN_ESC_PRECHARGE, OUTPUT );
  
  
  // Initialize ADC Controller
  
  // Setting clock
  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |    // Divide Clock by 512.
                   ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default

  ADC->SAMPCTRL.reg = 0x3f;                        // Set max Sampling Time Length

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0

  analogReference( AR_DEFAULT ) ; // Analog Reference is AREF pin (3.3v)
  
}

#ifdef __cplusplus
}
#endif
