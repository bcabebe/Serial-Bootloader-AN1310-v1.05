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
;
; Author        Date        Comment
; ************************************************************************
; E. Schlunder  07/22/2010  Optimized the code down a little bit,
;                           fewer instructions so works up to
;                           57.6Kbps from 8MHz INTOSC now.
;
;                           Fixed bug that could cause GPR overflow 
;                           when initially connecting with a bad 
;                           autobaud result.
; M. Cahill		05/26/2010	Added Bit-bang UART support for 12F617
; E. Schlunder  07/17/2009  Bringing back support for bootloader at 
;                           address 0 for hardware boot block write
;                           protect.
; E. Schlunder  05/08/2009  Upgrade to support new Serial Bootloader 
;                           features and protocol.
;
; Bootloader for PIC16F by Rodger Richey
; Adapted from PIC18F bootloader developed by Ross Fosler
; 03/18/2002    ... First full implementation
; 03/25/2002    Modified receive & parse engine to vector to autobaud on a checksum 
;               error since a checksum error could likely be a communications problem.
;               Modified the protocol to incorporate the autobaud as part of the 
;               first received <STX>. Doing this improves robustness by allowing
;               re-sync under any condition. Previously it was possible to enter a 
;               state where only a hard reset would allow re-syncing.
; 04/09/2002    Fixed bugs: 1) clear carry before shifting ABTIME in Autobaud
;                           2) Increment address in program memory write
;                           3) Increment address in program memory read
; 06/07/2002    Fixed bug in read, byte counter in code is word counter.  Needed
;               to multiply by 2 to get bytes.
;
; *****************************************************************************

; *****************************************************************************
#include "bootconfig.inc"
#include "devices.inc"
#include "bankswitch.inc"
; *****************************************************************************

; *****************************************************************************

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

#ifndef AppVector 
    ; The application startup GOTO instructions will be written just before the Boot Block,
    ; courtesy of the host PC bootloader application.
    #define AppVector (BootloaderStart-.5)
#endif

; *****************************************************************************
#define STX             0x0F
#define ETX             0x04
#define DLE             0x05
#define NTX             0xFF
; *****************************************************************************

; *****************************************************************************
CRCL                equ 0xA0        ; GPR RAM in bank 1
CRCH                equ 0xA1
RXDATA              equ 0xA2
TXDATA              equ 0xA3

#ifdef Bitbang
FWBRG				equ 0x20		; firmware baud rate generator
msFWBRG				equ 0x21		; most significant byte for count overflow
LoopCount			equ 0x22
ShiftReg			equ 0x23        ; shift register for bit-bang transmit routine
BitCount			equ 0x24
#endif

; Framed Packet Format
; <STX>[<COMMAND><ADDRL><ADDRH><ADDRU><0x00><DATALEN><...DATA...>]<CRCL><CRCH><ETX>

STARTBUFFER         equ 0x25
COMMAND             equ STARTBUFFER     ; receive buffer in bank 0
ADDRESS_L           equ STARTBUFFER+1
ADDRESS_H           equ STARTBUFFER+2
ADDRESS_U           equ STARTBUFFER+3
ADDRESS_X           equ STARTBUFFER+4
DATA_COUNTL         equ STARTBUFFER+5
PACKET_DATA         equ STARTBUFFER+6
DATA_COUNTH         equ PACKET_DATA     ; only for certain commands


#if BOOTLOADER_ADDRESS == 0
#ifndef BSR
PCLATH_TEMP	        equ	0x7E		; Interrupt context save/restore temporary memory
W_TEMP		        equ	0x7F
#endif
#endif
; *****************************************************************************

#ifndef Bitbang
#ifdef PORTC
    ; Most parts have the UART on RC7 and RC6 pins.
    #define PORTRX  PORTC
    #define RXPIN   7

    #define TRISTX  TRISC
    #define TXPIN   6
#else
    ; PIC16F88 doesn't have PORTC, so the UART pins are elsewhere...
    #define PORTRX  PORTB
    #define RXPIN   2

    #define TRISTX  TRISB
    #define TXPIN   5
#endif
#endif
 
    errorlevel -302                 ; Do not show any banking warnings
; *****************************************************************************
#if BOOTLOADER_ADDRESS != 0
    ORG     0
    ; The following GOTO is not strictly necessary, but may startup faster
    ; if running at slow clock speeds.
        errorlevel -306             ; Do not show any page boundary warnings
;    nop                                 
;    movlw   high(BootloaderBreakCheck)
;    movwf   PCLATH                  ; Bx
;    goto    BootloaderBreakCheck
        errorlevel +306             ; Do not show any page boundary warnings

    ORG     BOOTLOADER_ADDRESS
BootloaderStart:
    movlw   high(BootloadMode)
    movwf   PCLATH                  ; Bx
    goto    BootloadMode

; *****************************************************************************
; Determine if the application is supposed to be started or if we should
; go into bootloader mode.
;
; If RXD is in BREAK state (vs IDLE) when we come out of MCLR reset, 
; immediately enter bootloader mode, even if there exists some application 
; firmware in program memory.
BootloaderBreakCheck:
    DigitalInput
    BXtoB0
    movlw   high(AppVector)
    movwf   PCLATH                  ; Bx
#ifdef INVERT_UART
    btfss   PORTRX, RXPIN           ; B0 
    goto    AppVector               ; no BREAK state, attempt to start application

    btfsc   PORTRX, RXPIN           ; B0 BREAK found, wait for RXD to go IDLE
    goto    $-1
#else
    btfsc   PORTRX, RXPIN           ; B0  
    goto    AppVector               ; no BREAK state, attempt to start application

    btfss   PORTRX, RXPIN           ; B0 BREAK found, wait for RXD to go IDLE
    goto    $-1
#endif
#else ; BOOTLOADER_ADDRESS == 0 ****************************************************************
    ORG     0
BootloaderStart:
    nop                             ; required to allow debug executive startup when running under ICD
    BXtoB0                          ; Bx -> B0
    goto    BootloaderBreakCheck

    ORG     0x0004
InterruptVector:
#ifndef BSR
	movwf	W_TEMP                  ; Bx save W register temporarily
	swapf	PCLATH, W				; Bx save PCLATH register
	movwf	PCLATH_TEMP				; Bx (SWAPF used to avoid damaging STATUS register)
#endif
    movlw   high(AppIntVector)      ; Bx set PCLATH for making a long jump to the AppIntVector address
    movwf   PCLATH                  ; Bx
    goto    AppIntVector            ; Bx jump to remapped application interrupt vector.

BootloaderBreakCheck:
    DigitalInput
    BXtoB0
#ifdef INVERT_UART
    btfsc   PORTRX, RXPIN           ; B0 
    goto    WaitForRxIdle           ; BREAK detected, startup in Bootloader mode
#else
    btfss   PORTRX, RXPIN           ; B0  
    goto    WaitForRxIdle           ; BREAK detected, startup in Bootloader mode
#endif

    ; Attempt to startup in Application mode.
    ; Read instruction at the application reset vector location. 
    ; If we read 0x3FFF, assume that the application firmware has
    ; not been programmed yet, so don't try going into application mode.
    banksel EEADR                   ; Bx -> B2
    movlw   low(AppVector)          ; Bx load address of application reset vector
    movwf   EEADR                   ; B2 
    movlw   high(AppVector)
    movwf   EEADRH                  ; B2
    movwf   PCLATH                  ; Bx
    call    ReadFlashWord           ; Bx -> B0

    addlw   .1
    btfss   STATUS, Z               ; Bx if the lower byte != 0xFF, 
    goto    AppVector               ; Bx run application.

    movlw   0x3F
    xorwf   FSR, w                  ; Bx if the lower byte == 0xFF but upper byte != 0x3F,
    btfss   STATUS, Z               ; Bx run application
    goto    AppVector

    movlw   high(BootloadMode)
    movwf   PCLATH                  ; Bx

    ; otherwise, assume application firmware is not loaded, 
    ; fall through to bootloader mode...
#ifdef INVERT_UART
WaitForRxIdle:
    btfsc   PORTRX, RXPIN           ; B0 BREAK found, wait for RXD to go IDLE
    goto    WaitForRxIdle
#else
WaitForRxIdle:
    btfss   PORTRX, RXPIN           ; B0 BREAK found, wait for RXD to go IDLE
    goto    WaitForRxIdle
#endif
#endif ; end BOOTLOADER_ADDRESS == 0 ******************************************

BootloadMode:
    DigitalInput
#ifdef Bitbang
    B0toB1                          ; B0 -> B1
	bcf		TRISTX, TXPIN			; B1 Setup TX pin for output
#else
  #ifdef BRG16
    movlw   b'00110000'             ; 1:8 prescaler - no division required later (but no rounding possible)
    movwf   T1CON                   ; B0
  #endif

  #ifdef BSR
    banksel RCSTA
    movlw   b'10010000'             ; Setup UART
    movwf   RCSTA                   ; B0

;   bcf     TRISTX, TXPIN           ; B1 Setup TX pin for output
    movlw   b'00100110'             ; BRGH = 1, TXEN = 1
    movwf   TXSTA                   ; B1
    banksel OPTION_REG
  #else
    movlw   b'10010000'             ; Setup UART
    movwf   RCSTA                   ; B0

    B0toB1                          ; B0 -> B1
;   bcf     TRISTX, TXPIN           ; B1 Setup TX pin for output
    movlw   b'00100110'             ; BRGH = 1, TXEN = 1
    movwf   TXSTA                   ; B1
  #endif

  #ifndef BRG16
    clrwdt                          ; required to avoid reset when modifying TMR0 prescaler assignment
    movlw   b'00000011'             ; 1:16 prescaler for Timer 0, used for auto-baud calculation
    movwf   OPTION_REG              ; B1
  #endif

    B1toB3                          ; B1 -> B3
  #ifdef INVERT_UART
    bsf     BAUDCTL, RXDTP          ; B3
    bsf     BAUDCTL, TXCKP          ; B3
  #endif
  #ifdef BRG16
    bsf     BAUDCTL, BRG16          ; B3
  #endif
#endif
; *****************************************************************************

; *****************************************************************************
DoAutoBaud:
#ifdef Bitbang
    #include "BitBangAutoBaud.inc"  ; Bx -> B0
#else
    #include "SoftwareAutoBaud.inc" ; Bx -> B1
#endif

WaitForHostCommand:                 ; B0/B1
#ifdef Bitbang
    BXtoB0                          ; B1 -> B0
    lfsr    COMMAND                 ; Bx Point to the buffer
    call    ReadHostByte            ; B0 get start of transmission <STX>
    xorlw   STX
    bnz     DoAutoBaud              ; Bx got something unexpected, perform autobaud
#else
#ifdef BSR
    banksel RCSTA
#else
    BXtoB0                          ; B1 -> B0
#endif
    bsf     RCSTA, CREN             ; B0 start receiving

    lfsr    COMMAND                 ; Bx Point to the buffer
    call    ReadHostByte            ; B0 get start of transmission <STX>
    xorlw   STX
    bnz     DoAutoBaud              ; Bx got something unexpected, perform autobaud
#endif
; *****************************************************************************

; *****************************************************************************
; Read and parse packet data.
StartOfLine:
    movlw   STX                     ; send back start of response
    call    SendHostByte            ; B0/B1 -> B1

ReceiveDataLoop:
    call    ReadHostByte            ; Bx -> B0 Get the data
    xorlw   STX                     ; Check for an unexpected STX
    bz      StartOfLine             ; unexpected STX: abort packet and start over.

NoSTX:
    movf    INDF, W                 ; Bx
    xorlw   ETX                     ; Check for a ETX
    bz      VerifyPacketCRC         ; Yes, verify CRC

NoETX:
    movf    INDF, W                 ; Bx
    xorlw   DLE                     ; Check for a DLE
    bnz     AppendDataBuffer

    call    ReadHostByte            ; Bx -> B0 DLE received, get the next byte and store it
    
AppendDataBuffer:
    incf    FSR, f                  ; Bx move to next empty location
    btfss   FSR, 7                  ; Bx have we overflowed the GPR receive buffer?
    goto    ReceiveDataLoop         ; nope, continue receiving data
    goto    DoAutoBaud              ; overflow, baud rate most likely bad, re-attempt autobaud.
    
VerifyPacketCRC:
    B0toB1                          ; B0 -> B1
    decf    FSR, w                  ; Bx
    movwf   TXDATA                  ; B1 save end of packet pointer
    decf    TXDATA, f               ; Bx

    lfsr    COMMAND                 ; Bx reset pointer to beginning of data
    clrf    CRCL                    ; B1 reset CRC accumulator
    clrf    CRCH                    ; B1

VerifyPacketCrcLoop:
    movf    INDF, w                 ; Bx
    call    AddCrcB1                ; B1 add new data to the CRC

    incf    FSR, f                  ; Bx
    movf    FSR, w                  ; Bx
    subwf   TXDATA, w               ; B1
    bnz     VerifyPacketCrcLoop     ; we aren't at the end of the received data yet, loop

    movf    CRCL, w                 ; B1
    subwf   INDF, w                 ; Bx
    bnz     DoAutoBaud              ; invalid CRC, reset baud rate generator to re-sync with host

    incf    FSR, f                  ; Bx
    movf    CRCH, w                 ; B1
    subwf   INDF, w                 ; Bx
    bnz     DoAutoBaud              ; Bx invalid CRC, reset baud rate generator to re-sync with host

; ***********************************************
; Pre-setup, common to all commands.
    clrf    CRCL                    ; B1
    clrf    CRCH                    ; B1

    BXtoB0                          ; B1 -> B0
    movf    ADDRESS_H, W            ; B0 read address pointer from packet data
    movwf   FSR                     ; Bx temporarily save high address byte to FSR
    movf    ADDRESS_L, W            ; B0
    banksel EEADR                   ; Bx -> B2
    movwf   EEADR                   ; B2
    movf    FSR, w                  ; Bx read back high address byte from temporary register
    movwf   EEADRH                  ; B2

    lfsr    PACKET_DATA             ; Bx
    BXtoB0                          ; Bx -> B0

; ***********************************************

 

; ***********************************************
; Test the command field and sub-command.
CheckCommand:
    movlw   (JUMPTABLE_END - JUMPTABLE_BEGIN)
    subwf   COMMAND, w              ; B0 test for valid command number
    bc      DoAutoBaud              ; Bx invalid command - reset baud generator and re-sync with host

    movf    COMMAND, W              ; B0
    ; This jump table must exist entirely within one 256 byte block of program memory.
#if ($ & 0xFF) > (0xFF - .10)
    ; Too close to the end of a 256 byte boundary, push address forward to get code
    ; into the next 256 byte block.
    messg   "Wasting some code space to ensure jump table is aligned."
    ORG     $+(0x100 - ($ & 0xFF))
#endif
    addwf   PCL, F                  ; 0 Bx Jump in command jump table based on COMMAND from host
JUMPTABLE_BEGIN:
    goto    BootloaderInfo          ; 1 B0 0
    goto    ReadFlash               ; 2 Bx 1
    goto    VerifyFlash             ; 3 Bx 2
    goto    EraseFlash              ; 4 Bx 3
    goto    WriteFlash              ; 5 Bx 4
#ifdef __12F617
    goto    SendAcknowledge         ; 6 B0 5
    goto    SendAcknowledge         ; 7 Bx 6
#else
    goto    ReadEeprom              ; 6 B0 5
    goto    WriteEeprom             ; 7 Bx 6
#endif
    goto    SendAcknowledge         ; 8 B0 7 - WriteConfig not supported on PIC16F devices
    nop                             ; 9 B0 8
    movlw   high(AppVector)         ; 10 B0 9
JUMPTABLE_END:
    movwf   PCLATH                  ; Bx
    goto    AppVector               ; B0
#if (JUMPTABLE_BEGIN & 0xFF) > (JUMPTABLE_END & 0xFF)
    error "Jump table is not aligned to fit within a single 256 byte address range."
#endif

#ifndef Bitbang
WaitForRise:                        ; B0
    clrwdt

WaitForRiseLoop:                    ; B0
#ifdef BRG16
    btfsc   PIR1, TMR1IF            ; B0 if TMR1 overflowed, we did not get a good baud capture
    return                          ; abort
#endif

    btfsc   PORTRX, RXPIN           ; B0 Wait for a falling edge
    goto    WaitForRiseLoop         ; B0

WtSR:
    btfss   PORTRX, RXPIN           ; B0 Wait for rising edge
    goto    WtSR                    ; B0
    return
#endif

; 16-bit CCIT CRC
; Adds WREG byte to the CRC checksum CRCH:CRCL. WREG destroyed on return.
AddCrc:                             ; B0/B1 Init: CRCH = HHHH hhhh, CRCL = LLLL llll
    B0toB1                          ; B0 -> B1
AddCrcB1:
    xorwf   CRCH, w                 ; B1 Pre:  HHHH hhhh     WREG =      IIII iiii
    movwf   RXDATA                  ; B1
    movf    CRCL, w                 ; B1 Pre:  LLLL llll     CRCH =      LLLL llll
    movwf   CRCH                    ; B1
    movf    RXDATA, w               ; B1
    movwf   CRCL                    ; B1 Pre:  IIII iiii     CRCL =      IIII iiii
    swapf   CRCL, w                 ; B1 Pre:  IIII iiii     WREG =      iiii IIII
    andlw   0x0F                    ; Pre:  iiii IIII     WREG =      0000 IIII
    xorwf   CRCL, f                 ; B1 Pre:  IIII iiii     CRCL =      IIII jjjj
    swapf   CRCL, w                 ; B1 Pre:  IIII jjjj     WREG =      jjjj IIII
    andlw   0xF0                    ; Pre:  jjjj IIII     WREG =      jjjj 0000
    xorwf   CRCH, f                 ; B1 Pre:  LLLL llll     CRCH =      MMMM llll
    swapf   CRCL, f                 ; B1 Pre:  IIII jjjj     WREG =      jjjj IIII
    bcf     STATUS, C               ; Bx
    rlf     CRCL, w                 ; B1 Pre:  jjjj IIII     WREG =      jjjI IIIj
    btfsc   STATUS, C               ; Bx
    addlw   .1
    xorwf   CRCH, f                 ; B1 Pre:  MMMM llll     CRCH =      XXXN mmmm
    andlw   b'11100000'             ; Pre:  jjjI IIIj     WREG =      jjj0 0000
    xorwf   CRCH, f                 ; B1 Pre:  jjj0 0000     CRCH =      MMMN mmmm
    swapf   CRCL, f                 ; B1 Pre:  IIII jjjj     WREG =      jjjj IIII
    xorwf   CRCL, f                 ; B1 Pre:  MMMN mmmm     CRCL =      JJJI jjjj
    return

; ***********************************************
; Commands
; ***********************************************
 
; Provides information about the Bootloader to the host PC software.
BootInfoBlock:
    db      high(BOOTBLOCKSIZE), low(BOOTBLOCKSIZE)
    db      MINOR_VERSION, MAJOR_VERSION
#ifdef FREE
    db      0x02, 0x01              ; family id : command mask (erase flash command enabled)
#else
    db      0x02, 0x00              ; family id : command mask (no erase flash command)
#endif
    db      high(BootloaderStart), low(BootloaderStart)
    db      0, upper(BootloaderStart)
    db      high(DEVICEID), low(DEVICEID)
BootInfoBlockEnd:

; In:   <STX>[<0x00>]<CRCL><CRCH><ETX>
; Out:  <STX><BOOTBYTESL><BOOTBYTESH><VERL><VERH><STARTBOOTL><STARTBOOTH><STARTBOOTU><0x00><CRCL><CRCH><ETX>
BootloaderInfo:                     ; B0
    movlw   (BootInfoBlockEnd - BootInfoBlock)
    movwf   DATA_COUNTL             ; B0
    clrf    DATA_COUNTH             ; B0

    banksel EEADR
    movlw   low(BootInfoBlock)
    movwf   EEADR                   ; B2/B3(PIC16F193x)
    movlw   high(BootInfoBlock)     ; B2/B3(PIC16F193x)
    movwf   EEADRH                  ; B2/B3(PIC16F193x)

    ;; fall through to ReadFlash code -- send Bootloader Information Block from FLASH.

; In:   <STX>[<0x01><ADDRL><ADDRH><ADDRU><0x00><BYTESL><BYTESH>]<CRCL><CRCH><ETX>
; Out:  <STX>[<DATA>...]<CRCL><CRCH><ETX>
ReadFlash:                          ; Bx -> B0
    call    ReadFlashWord           ; Bx -> B0
    call    SendEscapeByte          ; Bx -> B1
    call    AddCrcB1                ; B1
    movf    FSR, w                  ; Bx read most significant bits from tempory memory
    call    SendEscapeByte          ; Bx -> B1
    call    AddCrcB1                ; B1
    
    banksel EEADR                   ; Bx -> B2
    incf    EEADR, f                ; B2
    btfsc   STATUS, Z               ; Bx
    incf    EEADRH,F                ; B2
    
    BXtoB0                          ; Bx -> B0
    movlw   1
    subwf   DATA_COUNTL, f          ; B0
    btfss   STATUS, C
    decf    DATA_COUNTH, f          ; B0

    movf    DATA_COUNTH, w          ; B0 is DATA_COUNTH:DATA_COUNTL == 0?
    iorwf   DATA_COUNTL, w          ; B0 
    bnz     ReadFlash               ; Bx non-zero, keep reading more data
    goto    SendChecksum            ; Bx zero, exit read loop and send end of packet

ReadFlashWord:                      ; Bx -> B0
    banksel EECON1                  ; Bx -> B3
    bsf     EECON1, EEPGD           ; B3 access program memory instead of eeprom data memory
    bsf     EECON1, RD              ; B3 initiate read operation
    nop                             ; B3 sounds like this instruction slot might be usable, but NOP is safest
    nop                             ; B3 required NOP during program memory read operation
    banksel EEDATA                  ; Bx -> B2
    movf    EEDATH, w               ; B2 read most significant bits of program memory
    movwf   FSR                     ; Bx save it temporarily
    movf    EEDATA, w               ; B2 read least significant byte of program memory
    BXtoB0                          ; Bx -> B0
    return

; In:   <STX>[<0x02><ADDRL><ADDRH><ADDRU><0x00><BLOCKSL><BLOCKSH>]<CRCL><CRCH><ETX>
; Out:  <STX>[<CRCL1><CRCH1>...<CRCLn><CRCHn>]<ETX>
VerifyFlash:                        ; Bx
    call    ReadFlashWord           ; Bx -> B0
    call    AddCrc                  ; B1/B0 -> B1
    movf    FSR, w                  ; Bx read most significant bits from tempory memory
    call    AddCrcB1                ; B1

    banksel EEADR                   ; Bx -> B2
    incf    EEADR, f                ; B2
    btfsc   STATUS, Z               ; Bx
    incf    EEADRH,F                ; B2

    movf    EEADR, w                ; B2
    andlw   (ERASE_FLASH_BLOCKSIZE-1)
    bnz     VerifyFlash             ; Bx

    call    SendCRCWord             ; Bx -> B1

    BXtoB0                          ; Bx -> B0
    movlw   1
    subwf   DATA_COUNTL, f          ; B0
    btfss   STATUS, C               ; Bx
    decf    DATA_COUNTH, f          ; B0

    movf    DATA_COUNTH, w          ; B0 is DATA_COUNTH:DATA_COUNTL == 0?
    iorwf   DATA_COUNTL, w          ; B0
    bnz     VerifyFlash             ; Bx non-zero, keep reading more data
    goto    SendETX                 ; Bx zero, exit read loop and send end of packet

; In:   <STX>[<0x03><ADDRL><ADDRH><ADDRU><0x00><PAGESL>]<CRCL><CRCH><ETX>
; Out:  <STX>[<0x03>]<CRCL><CRCH><ETX>
#ifdef FREE
#ifdef USE_SOFTBOOTWP
    goto    BootloaderStart         ; this code -should- never be executed, but in case of errant 
    goto    BootloaderStart         ; execution or firmware bugs, may protect against accidental erases.
#endif
EraseFlash:                         ; Bx
#ifdef USE_SOFTBOOTWP
  #define ERASE_ADDRESS_MASK  ( (~(ERASE_FLASH_BLOCKSIZE-1)) & (END_FLASH-1) )

    banksel EEADR                   ; Bx -> B2
  #if high(ERASE_ADDRESS_MASK) != 0xFF
    movlw   high(ERASE_ADDRESS_MASK)    ; force starting address to land on a FLASH Erase Block boundary
    andwf   EEADRH, f
  #endif
  #if low(ERASE_ADDRESS_MASK) != 0xFF
    movlw   low(ERASE_ADDRESS_MASK)     ; force starting address to land on a FLASH Erase Block boundary
    andwf   EEADR, f
  #endif

  #if BOOTLOADER_ADDRESS != 0
    ; is the address to erase less than the bootloader address?
    movlw   high(BOOTLOADER_ADDRESS)
    subwf   EEADRH, w
    movlw   low(BOOTLOADER_ADDRESS)
    btfsc   STATUS, Z
    subwf   EEADR, w
    btfss   STATUS, C
    goto    EraseAddressOkay    ; erasing memory before the boot block is okay
  #endif

    ; is the address to erase greater than or equal to the end of the boot block?
    movlw   high(BOOTLOADER_ADDRESS + BOOTBLOCKSIZE)
    subwf   EEADRH, w
    movlw   low(BOOTLOADER_ADDRESS + BOOTBLOCKSIZE)
    btfsc   STATUS, Z
    subwf   EEADR, w
    btfsc   STATUS, C
    goto    EraseAddressOkay    ; erasing memory after the boot block is okay

    banksel EECON1              ; Bx -> B3
    clrf    EECON1              ; inhibit writes for this block
    goto    NextEraseBlock      ; move on to next erase block

    goto    BootloaderStart     ; this code -should- never be executed, but in case of errant 
    goto    BootloaderStart     ; execution or firmware bugs, may protect against accidental writes.
#endif
EraseAddressOkay:
    banksel EECON1                  ; Bx -> B3
    movlw   b'10010100'             ; Bx setup FLASH erase
    movwf   EECON1                  ; B3
    call    StartWriteB3            ; B3 erase the page

NextEraseBlock:
    ; Decrement address by erase block size
    banksel EEADR                   ; Bx -> B2
#if ERASE_FLASH_BLOCKSIZE >= .256
    movlw   high(ERASE_FLASH_BLOCKSIZE)
    subwf   EEADRH, F               ; B2
#else
    movlw   ERASE_FLASH_BLOCKSIZE
    subwf   EEADR, F                ; B2
    btfss   STATUS, C
    decf    EEADRH, f               ; B2
#endif

    banksel DATA_COUNTL             ; Bx -> B1
    decfsz  DATA_COUNTL, F          ; B1
    goto    EraseFlash    
    goto    SendAcknowledge         ; All done, send acknowledgement packet
#endif

#ifdef USE_SOFTBOOTWP
    goto    BootloaderStart         ; this code -should- never be executed, but in case of errant 
    goto    BootloaderStart         ; execution or firmware bugs, may protect against accidental writes.
#endif

; In:   <STX>[<0x04><ADDRL><ADDRH><ADDRU><0x00><BLOCKSL><DATA>...]<CRCL><CRCH><ETX>
; Out:  <STX>[<0x04>]<CRCL><CRCH><ETX>
WriteFlash:                         ; Bx
#ifdef USE_SOFTBOOTWP
  #define WRITE_ADDRESS_MASK  ( (~(WRITE_FLASH_BLOCKSIZE-1)) & (END_FLASH-1) )

    banksel EEADR                   ; Bx -> B2
  #if high(WRITE_ADDRESS_MASK) != 0xFF
    movlw   high(WRITE_ADDRESS_MASK)    ; force starting address to land on a FLASH Write Block boundary
    andwf   EEADRH, f
  #endif
  #if low(WRITE_ADDRESS_MASK) != 0xFF
    movlw   low(WRITE_ADDRESS_MASK)     ; force starting address to land on a FLASH Write Block boundary
    andwf   EEADR, f
  #endif

  #if BOOTLOADER_ADDRESS != 0
    ; is the address to write less than the bootloader address?
    movlw   high(BOOTLOADER_ADDRESS)
    subwf   EEADRH, w
    movlw   low(BOOTLOADER_ADDRESS)
    btfsc   STATUS, Z
    subwf   EEADR, w
    btfss   STATUS, C
    goto    WriteAddressOkay    ; writing before the boot block is okay
  #endif

    ; is the address to write greater than or equal to the end of the boot block?
    movlw   high(BOOTLOADER_ADDRESS + BOOTBLOCKSIZE)
    subwf   EEADRH, w
    movlw   low(BOOTLOADER_ADDRESS + BOOTBLOCKSIZE)
    btfsc   STATUS, Z
    subwf   EEADR, w
    btfsc   STATUS, C
    goto    WriteAddressOkay    ; writing after the boot block is okay

    banksel EECON1              ; Bx -> B3
    clrf    EECON1              ; inhibit writes for this block
    goto    LoadHoldingRegisters; fake the write so we can move on to real writes

    goto    BootloaderStart     ; this code -should- never be executed, but in case of errant 
    goto    BootloaderStart     ; execution or firmware bugs, may protect against accidental writes.
#endif
WriteAddressOkay:
    banksel EECON1                  ; Bx -> B3
    movlw   b'10000100'             ; B3 setup FLASH write
    movwf   EECON1                  ; B3

LoadHoldingRegisters:
    banksel EEDATA                  ; Bx -> B2
    movf    INDF, w                 ; Bx read from buffer memory
    incf    FSR, f                  ; Bx increment buffer memory pointer
    movwf   EEDATA                  ; B2 load the least significant byte holding register

    movf    INDF, w                 ; Bx read from buffer memory
    incf    FSR, f                  ; Bx increment buffer memory pointer
    movwf   EEDATH                  ; B2 load the most significant byte holding register
    call    StartWrite              ; B3 initiate a write

    banksel EEADR
    incf    EEADR, f                ; B2/B3 increment FLASH memory write pointer
    btfsc   STATUS, Z               ; Bx
    incf    EEADRH, f               ; B2/B3

    ; are we at the end of a write block?
    movlw   (WRITE_FLASH_BLOCKSIZE-1)
    andwf   EEADR, w                ; B2/B3
    bnz     LoadHoldingRegisters    ; Bx

    BXtoB0                          ; B2/B3 -> B0
    decfsz  DATA_COUNTL, F          ; B0 finished writing block, is there any more data to write?
    goto    WriteFlash              ; Bx more data to write, repeat.
    goto    SendAcknowledge         ; B0 all done, send ACK packet

#ifndef __12F617
; In:   <STX>[<0x05><ADDRL><ADDRH><0x00><0x00><BYTESL><BYTESH>]<CRCL><CRCH><ETX>
; Out:  <STX>[<DATA>...]<CRCL><CRCH><ETX>
ReadEeprom:                         ; Bx
    banksel EECON1                  ; Bx -> B3
    clrf    EECON1                  ; B3

ReadEepromLoop:
    bsf     EECON1, RD              ; B3 Read the data
    btfsc   EECON1, RD
    goto    $-1                     ; wait for read to complete
    banksel EEDATA                  ; Bx -> B2
    movf    EEDATA, w               ; B2

    banksel EEADR
    incf    EEADR, f                ; B2 increment EEPROM data pointer
    btfsc   STATUS, Z               ; B2 did we overflow?
    incf    EEADRH, f               ; B2 yes, increment high byte of EEPROM data pointer

    call    SendEscapeByte          ; Bx -> B1
    call    AddCrcB1                ; B1

    BXtoB0                          ; Bx -> B0
    decfsz  DATA_COUNTL, F          ; B0
    goto    ReadEeprom              ; Bx Not finished then repeat
    goto    SendChecksum            ; Bx

; In:   <STX>[<0x06><ADDRL><ADDRH><0x00><0x00><BYTESL><BYTESH><DATA>...]<CRCL><CRCH><ETX>
; Out:  <STX>[<0x06>]<CRCL><CRCH><ETX>
WriteEeprom:                        ; Bx
    incf    FSR, f                  ; Bx increment data buffer pointer
    movf    INDF, w                 ; Bx read data from buffer
    banksel EEDATA                  ; Bx -> B2
    movwf   EEDATA                  ; B2 load the least significant byte holding register
    B2toB3                          ; B2 -> B3
    movlw   b'00000100'             ; Setup for EEPROM data writes
    movwf   EECON1                  ; B3
    call    StartWriteB3            ; B3

    btfsc   EECON1, WR              ; B3 wait for write to complete before moving to next address
    goto    $-1

    banksel EEADR
    incf    EEADR, F                ; B2/B3 Adjust EEDATA pointer

    BXtoB0                          ; B2 -> B0
    decfsz  DATA_COUNTL, f          ; B0
    goto    WriteEeprom             ; Bx
    goto    SendAcknowledge         ; B0
#endif

; ***********************************************
; Send an acknowledgement packet back
;
; <STX><COMMAND><CRCL><CRCH><ETX>

; Some devices only have config words as FLASH memory. Some devices don't have EEPROM.
; For these devices, we can save code by jumping directly to sending back an
; acknowledgement packet if the PC application erroneously requests them.
#ifndef FREE
EraseFlash:                         ; only PIC16F88/87 supports explicit FLASH erase commands
#endif
SendAcknowledge:                    ; B0
    movf    COMMAND, w              ; B0
    call    SendEscapeByte          ; B0/B1 -> B1 Send only the command byte (acknowledge packet)
    call    AddCrcB1                ; B1

SendChecksum:                       ; Bx
    call    SendCRCWord             ; Bx -> B1

SendETX:                            ; Bx -> B1
    BXtoB0                          ; Bx -> B0
    movlw   ETX                     ; send end of text condition
    call    SendHostByte            ; B0/B1 -> B1
    goto    WaitForHostCommand      ; B0/B1

; *****************************************************************************


; *****************************************************************************
SendCRCWord:                        ; Bx -> B1
    banksel CRCL
    movf    CRCL, w                 ; B1
    call    SendEscapeByte          ; Bx -> B1
    movf    CRCH, w                 ; B1
    ;; fall through to SendEscapeByte routine below

; Write a byte to the serial port while escaping control characters with a DLE
; first.
SendEscapeByte:                     ; Bx -> B1
    banksel TXDATA
    movwf   TXDATA                  ; B1 Save the data
 
    xorlw   STX                     ; Check for a STX
    bz      WrDLE                   ; No, continue WrNext

    movf    TXDATA, W               ; B1
    xorlw   ETX                     ; Check for a ETX
    bz      WrDLE                   ; No, continue WrNext

    movf    TXDATA, W               ; B1
    xorlw   DLE                     ; Check for a DLE
    bnz     WrNext                  ; No, continue WrNext

WrDLE:
    movlw   DLE                     ; Yes, send DLE first
    call    SendHostByte            ; B0/B1 -> B1

WrNext:
    movf    TXDATA, W               ; B1 Then send STX

SendHostByte:                       ; B0/B1 -> B1
#ifdef Bitbang
	BXtoB0							; B1 -> B0
	clrwdt
; *****************************************************************************
;Transmit:                           
    ; Transmit the byte in W -- bit banged serial transmit
    movwf	msFWBRG					; save W (msFWBRG is unused outside autobaud)
    movwf   ShiftReg                ; put output byte in shift register
    movlw   D'8'                    ; set bit counter
    movwf   BitCount
#ifdef INVERT_UART
    bsf     PORTTX, TXPIN           ; send start bit
#else
    bcf     PORTTX, TXPIN           ; send start bit
#endif
    call    WaitBitTime             ; wait 1 bit time
    goto    $+1                     ; loop equalizer
    nop
    
TXNextBit:
    rrf     ShiftReg, f              ; put next bit in carry
    btfss   STATUS, C                ; test bit
    goto    TXZero

TXOne:
    nop                             ; loop equalizer
#ifdef INVERT_UART
    bcf     PORTTX, TXPIN
#else
    bsf     PORTTX, TXPIN
#endif
    call    WaitBitTime             ; wait 1 bit time
    decfsz  BitCount,f              ; test if done
    goto    TXNextBit               ; no - send next bit
    goto    TXStop

TXZero:
#ifdef INVERT_UART
    bsf     PORTTX, TXPIN
#else
    bcf     PORTTX, TXPIN
#endif
    call    WaitBitTime             ; wait 1 bit time
    decfsz  BitCount,f              ; test if done
    goto    TXNextBit               ; no - send next bit
    goto    TXStop
    
TXStop:
    goto    $+1                     ; loop equalizer
    nop
#ifdef INVERT_UART
    bcf     PORTTX, TXPIN           ; stop bit
#else
    bsf     PORTTX, TXPIN           ; stop bit
#endif
    call    WaitBitTime             ; ensure full bit time for stop bit

    movf	msFWBRG, w				; restore W, needed for CRC calculation

; *****************************************************************************
	B0toB1							; B0 -> B1
	return
#else
    BXtoB0                          ; B1 -> B0
    clrwdt
    btfss   PIR1, TXIF              ; B0 Write only if TXREG is ready
    goto    $-1
    
    banksel TXREG
    movwf   TXREG                   ; B0 Start sending
    B0toB1                          ; B0 -> B1
    return
#endif
; *****************************************************************************

ReadHostByte:                       ; Bx -> B0
#ifdef Bitbang
    BXtoB0                          ; Bx -> B0
; *****************************************************************************

Receive:			                
    ; receive a serial character to W and INDF - bit banged serial receive
    movlw   D'8'                    ; set bit counter
    movwf   BitCount

; set bit timer for half bit time

    movf    FWBRG,w                 ; get bit time
    addlw   D'1'                    ; round up
    movwf   LoopCount               ; set loop counter
    rrf     LoopCount,f             ; divide by 2
    movlw   D'2'                    ; subtract overhead for call and start bit test
    subwf   LoopCount,f        
     
; wait for start bit edge

    clrwdt
#ifdef INVERT_UART
    btfss   PORTRX, RXPIN           ; skip on rising edge
#else
    btfsc   PORTRX, RXPIN           ; skip on falling edge
#endif
    goto    $-2
    
    call    BitWait                 ; wait till middle of start bit
#ifdef INVERT_UART
    btfss   PORTRX, RXPIN           ; receive bit must be zero here
#else
    btfsc   PORTRX, RXPIN           ; receive bit must be zero here
#endif
    goto    Receive                 ; false start - try again
    goto    $+1                     ; loop equalizer
RXLoop:
    call    WaitBitTime             ; full bit time to middle of next bit
    bcf     STATUS,C                ; pre-clear carry
#ifdef INVERT_UART
    btfss   PORTRX, RXPIN           ; test receive bit
#else
    btfsc   PORTRX, RXPIN           ; test receive bit
#endif
    bsf     STATUS,C                ; set carry if it was high
    
    rrf     INDF, f                 ; shift received bit in
    nop                             ; loop equalizer
    decfsz  BitCount,f              ; test if end of bits
    goto    RXLoop                  ; no - get next bit
    
; The following code seems to prevent 115.2Kbps operation from 14.7456MHz EC (and above).
;
;    call    WaitBitTime             ; wait to middle of stop bit
;    btfss   PORTRX,RXPIN            ; test stop bit
;    clrf    INDF                    ; reset on over run

    movf    INDF, w                 ; return with result in W and INDF memory
; *****************************************************************************
    return	
#else
    BXtoB0                          ; Bx -> B0
    clrwdt
    btfss   PIR1, RCIF              ; B0 Wait for data from RS232
    goto    $-1   

#ifdef BSR
    banksel RCREG
#endif
    movf    RCREG, W                ; B0 Save the data
#ifdef BSR
    movlb   .0
#endif
    movwf   INDF                    ; Bx
    return
#endif

#ifdef USE_SOFTBOOTWP
    goto    BootloaderStart         ; this code -should- never be executed, but in case of errant 
    goto    BootloaderStart         ; execution or firmware bugs, may protect against accidental writes.
#endif

StartWrite:                         ; B2/B3
    B2toB3                          ; B2 -> B3
StartWriteB3:
    clrwdt                          ; Bx
    movlw   0x55                    ; B3 Unlock
    movwf   EECON2                  ; B3
    movlw   0xAA                    ; B3
    movwf   EECON2                  ; B3
    bsf     EECON1, WR              ; B3 Start the write
    nop                             ; Bx
    nop                             ; Bx    
    return                          ; B3


#ifdef Bitbang
; *****************************************************************************

WaitBitTime
    movlw  D'4'                     ; overhead for bit processing (8 cycles) and call... 
    subwf  FWBRG,w                  ; ...to bit wait (8 cycles) is 4 loop counts
    movwf  LoopCount
BitWaitLoop
    nop
BitWait
    decfsz LoopCount,f
    goto   BitWaitLoop
    goto   $+1                      ; loop equalizer
    return

#endif

    END
