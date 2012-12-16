; Copyright (c) 2002-2010,  Microchip Technology Inc.
;
; Microchip licenses this software to you solely for use with Microchip
; products.  The software is owned by Microchip and its licensors, and
; is protected under applicable copyright laws.  All rights reserved.
;
; SOFTWARE IS PROVIDED "AS IS."  MICROCHIP EXPRESSLY DISCLAIMS ANY
; WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT
; NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
; FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL
; MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
; CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR
; EQUIPMENT, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY
; OR SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED
; TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION,
; OR OTHER SIMILAR COSTS.
;
; To the fullest extent allowed by law, Microchip and its licensors
; liability shall not exceed the amount of fees, if any, that you
; have paid directly to Microchip to use this software.
;
; MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
; OF THESE TERMS.

#define USE_SOFTBOOTWP          ; enable software boot block write protection
;#define INVERT_UART             ; If you don't have an RS232 transceiver, you might want this option

#define RXPIN   GP0             ; define which I/O pins to use for TX/RX
#define TXPIN   GP1
#define PORTRX  GPIO            ; define the SFR used to read the RX pin state
#define PORTTX  GPIO            ; define the SFR used to write the TX pin state
#define TRISTX  TRISIO          ; define the SFR used to change the TX pin to an output pin

; Bootloader must start at the beginning of a FLASH erase block. If unspecified,
; bootloader will automatically be located at the end of program memory address space.
;#define BOOTLOADER_ADDRESS   END_FLASH - (ERASE_FLASH_BLOCKSIZE * 38) ; useful for running under debugger (debug executive wants to reside at the end of memory space too)
;#define BOOTLOADER_ADDRESS   0          ; bootloader at beginning, application start/ISR vectors require remapping

#ifdef BOOTLOADER_ADDRESS
  #if BOOTLOADER_ADDRESS == 0
    ; For Bootloader located at program memory address 0, the application firmware must 
    ; provide remapped reset and interrupt vectors outside of the Boot Block. The following 
    ; #defines tell the bootloader firmware where application entry points are to be expected:
    #define AppVector           0x400   ; remapped application start up code should be located here.
    #define AppIntVector        0x404   ; remapped application interrupt handler should be located here
  #endif
#endif

; Because we need to know the total size of the bootloader before the assembler has finished
; compiling the source code, you must estimate the final bootloader size and write it down 
; here ahead of time. This number is in words (one word per instruction).
#define BOOTLOADERSIZE  .434

#define MAJOR_VERSION   .1              ; Bootloader Firmware Version
#define MINOR_VERSION   .5

