; Copyright (c) 2002-2011,  Microchip Technology Inc.
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

#ifndef RXPORT
    #ifdef __16F1828
        #ifdef USE_ALTERNATE_PINS
            ; UART option for TX/RC4 and RX/RC5 pins.
            #define RXPORT  PORTC
            #define RXPIN   RC5
            #define TXTRIS  TRISC
            #define TXPIN   RC4
        #else
            ; UART default on TX/RB7 and RX/RB5 pins.
            #define RXPORT  PORTB
            #define RXPIN   RB5
            #define TXTRIS  TRISB
            #define TXPIN   RB7
        #endif
    #endif
    #ifdef __16LF1828
        #ifdef USE_ALTERNATE_PINS
            ; UART option for TX/RC4 and RX/RC5 pins.
            #define RXPORT  PORTC
            #define RXPIN   RC5
            #define TXTRIS  TRISC
            #define TXPIN   RC4
        #else
            ; UART default on TX/RB7 and RX/RB5 pins.
            #define RXPORT  PORTB
            #define RXPIN   RB5
            #define TXTRIS  TRISB
            #define TXPIN   RB7
        #endif
    #endif
#endif

#ifndef RXPORT
    #ifdef PORTC
        #ifdef RXDTSEL
            #ifdef USE_ALTERNATE_PINS
                ; Some 14-pin devices have UART option for TX/RA0 and RX/RA1 pins. (PIC16F18xx)
                #define RXPORT  PORTA
                #define RXPIN   RA1
                #define TXTRIS  TRISA
                #define TXPIN   RA0
            #else
                #ifndef RC7
                    ; Some 14-pin devices have the UART on TX/RC4 and RX/RC5 pins. (PIC16F18xx)
                    #define RXPORT  PORTC
                    #define RXPIN   RC5
                    #define TXTRIS  TRISC
                    #define TXPIN   RC4
                #else
                    ; Most devices have the UART on TX/RC6 and RX/RC7 pins.
                    #define RXPORT  PORTC
                    #define RXPIN   RC7
                    #define TXTRIS  TRISC
                    #define TXPIN   RC6
                #endif            
            #endif
        #else
            ; Most devices have the UART on TX/RC6 and RX/RC7 pins.
            #define RXPORT  PORTC
            #define RXPIN   RC7
            #define TXTRIS  TRISC
            #define TXPIN   RC6
        #endif
    #else
        #ifdef RXDTSEL
            #ifdef USE_ALTERNATE_PINS
                ; Some 8-pin devices have UART option for TX/RA4 and RX/RA5 pins. (PIC12F1822)
                #define RXPORT  PORTA
                #define RXPIN   RA5
                #define TXTRIS  TRISA
                #define TXPIN   RA4
            #else
                ; Some 8-pin devices have UART on TX/RA0 and RX/RA1 pins. (PIC12F1822)
                #define RXPORT  PORTA
                #define RXPIN   RA1
                #define TXTRIS  TRISA
                #define TXPIN   RA0
            #endif
        #else
            ; PIC16F88 doesn't have PORTC, so the UART pins are on RX/RB2 and TX/RB5
            #define RXPORT  PORTB
            #define RXPIN   2
            #define TXTRIS  TRISB
            #define TXPIN   5    
        #endif
    #endif
#endif

DigitalInput macro
#ifdef ANSELA
    #if RXPORT == PORTA		; on PIC16F1824, TX/RX can be set to pins RA0/RA1
	banksel ANSELA 
	bcf	ANSELA, RXPIN	; set RX pin to digital input

	BXtoB0
    #endif
#endif
#ifdef ANSELB
    #if RXPORT == PORTB		; on PIC16F1828, TX/RX can be set to pins RX/RB5 and TX/RB7
	banksel ANSELB
	bcf	ANSELB, RXPIN	; set RX pin to digital input

	BXtoB0
    #endif
#endif
    endm        

#if BOOTLOADERSIZE < ERASE_FLASH_BLOCKSIZE
    ; This device has a large Erase FLASH Block Size, so we need to reserve a full Erase Block
    ; page for the bootloader. Reserving an entire erase block prevents the PC application
    ; from accidently erasing a portion of the bootloader.
    #define BOOTBLOCKSIZE ERASE_FLASH_BLOCKSIZE
    #ifndef BOOTLOADER_ADDRESS
        #ifdef CONFIG_AS_FLASH
            #define BOOTLOADER_ADDRESS  (END_FLASH - BOOTBLOCKSIZE - ERASE_FLASH_BLOCKSIZE)
        #else
            #define BOOTLOADER_ADDRESS  (END_FLASH - BOOTBLOCKSIZE)
        #endif
    #endif
#else
    #if (BOOTLOADERSIZE % ERASE_FLASH_BLOCKSIZE) == 0
        #define BOOTBLOCKSIZE BOOTLOADERSIZE     
    #else
        #define BOOTBLOCKSIZE (BOOTLOADERSIZE / ERASE_FLASH_BLOCKSIZE + 1) * ERASE_FLASH_BLOCKSIZE
    #endif
    #ifndef BOOTLOADER_ADDRESS
        #define BOOTLOADER_ADDRESS  (END_FLASH - BOOTBLOCKSIZE)
    #endif
#endif
