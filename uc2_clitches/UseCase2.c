/******************************************************************************/
/* UseCase2.C: Detection Clitches (in this case on JTAG I/O lines)            */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2006 KEIL - An ARM Company. All rights reserved.             */

#include <LPC214X.H>      // LPC214x I/O definitions

// JTAG Pins
#define PIN_TDI            0x1
#define PIN_TCLK           0x2
#define PIN_TRST           0x4
#define PIN_TMS            0x8

/*
 * Alternate Outputs, set JTAG I/O: TRST, TMS, TDI, TCK (lines encoded in 'v')
 */
static void alt_o (unsigned int v)  {
  unsigned int i;

  FIO0CLR = PIN_TRST | PIN_TMS | PIN_TDI;
  FIO0SET = v;
  FIO0SET = PIN_TCLK;
  for (i = 0; i < 5; i++);     // wait
  FIO0CLR = PIN_TCLK;
}

/*
 * Enhanced version avoids glitches
 */
static void alt_oe (unsigned int v)  {
  unsigned int x, i;

  x = PIN_TRST | PIN_TMS | PIN_TDI;
  x &= ~v;                      // enhancement to avoid glitches    
  FIO0CLR = x;
  FIO0SET = v;
  FIO0SET = PIN_TCLK;
  for (i = 0; i < 5; i++);     // wait
  FIO0CLR = PIN_TCLK;
}

int main (void)  {
  SCS = 3;                        // Use Fast IO

  FIO0PIN  = 0x0000000F;          // JTAG Pins set to 1
  FIO0DIR  = 0x0000000F;          // JTAG Pins are outputs

  while (1)  {
    alt_o (PIN_TMS);              // generate JTAG clock cycles
    alt_oe(PIN_TMS);
    alt_o (PIN_TMS+PIN_TDI);
    alt_oe(PIN_TMS+PIN_TDI);
    alt_o (PIN_TRST);
    alt_oe(PIN_TRST);
  }
}
