; Copyright (c) 2009,  Microchip Technology Inc.
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
;
; Author        Date        Comment
; ************************************************************************
; E. Schlunder  06/09/2009  Starting to move all bank switching code to
;                           macros in this include file. This will 
;                           allow us to support the enhanced PIC16F core
;                           devices, which use a "movlb" instruction
;                           instead of STATUS<RP0/RP1> bits.

#ifdef BSR
    ; Enhanced PIC16F core, use movlb instruction instead of diddling STATUS<RP0/RP1>
BXtoB0  macro
    movlb   .0                      ; Bx -> B0
        endm

B0toB1  macro
    movlb   .1                      ; Bx -> B1
        endm

B0toB2  macro
    movlb   .2                      ; Bx -> B2
        endm

B1toB0  macro
    movlb   .0                      ; Bx -> B0
        endm

B1toB3  macro
    movlb   .3                      ; Bx -> B3
        endm

B2toB3  macro
    movlb   .3                      ; Bx -> B3
        endm

B3toB2  macro
    movlb   .2                      ; Bx -> B2
        endm

B2toB0  macro
    movlb   .0                      ; Bx -> B0
        endm

lfsr    macro   GPRADDR
    movlw   low(GPRADDR)
    movwf   FSR0L
    movlw   high(GPRADDR)
    movwf   FSR0H
        endm

    #ifndef INDF
        #ifdef INDF0
            #define INDF INDF0
        #endif
    #endif

    #ifndef FSR
        #define FSR FSR0L
    #endif

    #ifndef BAUDCTL
        #define BAUDCTL BAUDCON
    #endif
#else  
    ; Original PIC16F core, no movlb instruction

BXtoB0  macro
    clrf    STATUS                  ; Bx -> B0
        endm

B0toB1  macro
    bsf     STATUS, RP0             ; B0 -> B1
        endm

B0toB2  macro
    bsf     STATUS, RP1             ; B0 -> B2
        endm

B2toB0  macro
    bcf     STATUS, RP1             ; B2 -> B0
        endm

B1toB0  macro
    bcf     STATUS, RP0             ; B1 -> B0
        endm

B1toB3  macro
    bsf     STATUS, RP1             ; B1 -> B3
        endm

B2toB3  macro
    bsf     STATUS, RP0             ; B2 -> B3
        endm

B3toB2  macro
    bcf     STATUS, RP0             ; B3 to B2
        endm

lfsr    macro   GPRADDR
    movlw   GPRADDR
    movwf   FSR
        endm
#endif

#ifndef EEDATA
    #ifdef EEDATL
        #define EEDATA EEDATL
    #else
        #define EEDATA EEDAT
    #endif
#endif
