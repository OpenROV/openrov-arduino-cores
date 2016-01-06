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

#include "sam.h"
#include "variant.h"

#include <stdio.h>

/**
 * \brief SystemInit() configures the needed clocks and according Flash Read Wait States.
 * At reset:
 * - OSC8M clock source is enabled with a divider by 8 (1MHz).
 * - Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
 * We need to:
 * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 * 2) Put XOSC32K as source of Generic Clock Generator 1
 * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 * 4) Enable DFLL48M clock
 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 * 6) Modify PRESCaler value of OSCM to have 8MHz
 * 7) Put OSC8M as source for Generic Clock Generator 3
 */
// Constants for Clock generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)

#define GCLK_0 (0u)
#define GCLK_1 (1u)
#define GCLK_2 (2u)
#define GCLK_3 (3u)

// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u)

void SystemInit( void )
{
	// Turn on the Power Manager, SYSCTRL, and GCLK clocks
	PM->APBAMASK.reg = (PM_APBAMASK_PM | PM_APBAMASK_SYSCTRL | PM_APBAMASK_GCLK);
	
	// Set the Power Manager clock dividers
	PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1;
	PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val;
	PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val;
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val;
  
  	// Set 1 Flash Wait State for 48MHz, cf tables 20.9 and 35.27 in SAMD21 Datasheet
	NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val;

	// Configure internal 8MHz oscillator to run without prescaler 
    SYSCTRL->OSC8M.bit.PRESC 	= SYSCTRL_OSC8M_PRESC_1_Val;
    SYSCTRL->OSC8M.bit.ONDEMAND = 0;
    SYSCTRL->OSC8M.bit.RUNSTDBY = 0;
    SYSCTRL->OSC8M.bit.ENABLE 	= 1;
	
	// Wait for oscillator to be ready
    while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC8MRDY))
	{
		asm( "nop" );
	}
	
	// Reset the GCLK module so it is in a known state
	GCLK->CTRL.reg = GCLK_CTRL_SWRST;
	
	// Sync
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
	{
		asm( "nop" );
	}
	
	// ----------------------------------------------------------------------------------------------
	// Put OSC8M as source of Generic Clock Generator 1

	// Set GCLK1 to divide to 32khz
	GCLK->GENDIV.reg = GCLK_GENDIV_ID( GCLK_1 ) | GCLK_GENDIV_DIV( 250 ); // Generic Clock Generator 1
	
	// Sync
	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		asm( "nop" );
	}
	
	// Write Generic Clock Generator 1 configuration 
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GCLK_1 ) | 	// Generic Clock Generator 1
						GCLK_GENCTRL_SRC_OSC8M | 		// Selected source is External 32KHz Oscillator
						GCLK_GENCTRL_GENEN;				// Enable
	
	// Sync
	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		asm( "nop" );
	}

	// ----------------------------------------------------------------------------------------------
   	// Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GENERIC_CLOCK_MULTIPLEXER_DFLL48M ) | 	// Generic Clock Multiplexer 0
						GCLK_CLKCTRL_GEN_GCLK1 | 								// Generic Clock Generator 1 is source
						GCLK_CLKCTRL_CLKEN;										// Enable	

	// Sync
	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		asm( "nop" );
	}

	// ----------------------------------------------------------------------------------------------
	// Enable DFLL48M clock
	
	/* Workaround for errata 9905 */
	SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
	
	/* wait for the DFLL clock to be ready */
	while (SYSCTRL->PCLKSR.bit.DFLLRDY == false)
	{
		asm( "nop" );
	}
	
	SYSCTRL_DFLLCTRL_Type dfllctrl =
	{
		.bit.WAITLOCK = 0,  /* Output clock when DFLL is locked */
		.bit.BPLCKC = 0,    /* Bypass coarse lock is enabled */
		.bit.QLDIS = 1,      /* Quick Lock is disabled */
		.bit.CCDIS = 0,     /* Chill Cycle is disabled */
		.bit.ONDEMAND = 0,  /* The oscillator is enabled when a peripheral is requesting the oscillator to be used as a clock source */
		.bit.RUNSTDBY = 0,  /* The oscillator is not stopped in standby sleep mode */
		.bit.USBCRM = 0,    /* USB Clock Recovery Mode is enabled. */
		.bit.LLAW = 0,      /* Locks will be lost after waking up from sleep modes if the DFLL clock has been stopped */
		.bit.STABLE = 0,    /* FINE calibration register value will be fixed after a fine lock */
		.bit.MODE = 1,       /* The DFLL operates in closed-loop operation. */
		.bit.ENABLE = 0,    /* The DFLL oscillator is enabled. */
	};
	
	SYSCTRL->DFLLCTRL.reg = dfllctrl.reg;
	
	/* wait for the DFLL clock to be ready */
	while (SYSCTRL->PCLKSR.bit.DFLLRDY == 0)
	{
		asm( "nop" );
	}
	
	/* get the coarse and fine values stored in NVM */
	uint32_t coarse = (*(uint32_t *)(0x806024) >> 26);
	uint32_t fine = (*(uint32_t *)(0x806028) & 0x3FF);
	
	SYSCTRL_DFLLVAL_Type dfllval =
	{
		.bit.COARSE = coarse,
		.bit.FINE = fine,
	};
	
	SYSCTRL->DFLLVAL.reg = dfllval.reg;
	
	SYSCTRL_DFLLMUL_Type dfllmul =
	{
		.bit.MUL = 1500, /* 32KHz * 1500 = 48MHz */
		.bit.CSTEP = (coarse >> 1), /* must be 50% or less of coarse value */
		.bit.FSTEP = (fine >> 1), /* must be 50% or less of fine value */
	};
	
	SYSCTRL->DFLLMUL.reg = dfllmul.reg;
	
	/* enable DFLL */
	SYSCTRL->DFLLCTRL.bit.ENABLE = 1;
	
	/* wait for DFLL closed loop lock */
	while (SYSCTRL->PCLKSR.bit.DFLLLCKF == 0);
	{
		asm( "nop" );
	}
	
	
	
	
	
	
	
	
	
	
	

  /* ----------------------------------------------------------------------------------------------
   * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GCLK_0 ) | GCLK_GENDIV_DIV( 1 ); // Generic Clock Generator 0

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    asm( "nop" );
  }

  /* Write Generic Clock Generator 0 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GCLK_0 ) | // Generic Clock Generator 0
                      GCLK_GENCTRL_SRC_DFLL48M | // Selected source is DFLL 48MHz
                      GCLK_GENCTRL_IDC | // Set 50/50 duty cycle
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    	asm( "nop" );
  }

  SystemCoreClock=VARIANT_MCK ;

  /* ----------------------------------------------------------------------------------------------
   * 8) Load ADC factory calibration values
   */

  // ADC Bias Calibration
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;

  // ADC Linearity bits 4:0
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;

  // ADC Linearity bits 7:5
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
}
