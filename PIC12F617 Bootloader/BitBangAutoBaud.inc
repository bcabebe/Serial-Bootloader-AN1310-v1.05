; *****************************************************************************
;
; ___    __________            ________
;    \__/          \__________/
;       |                     |
;       |-------- X ----------|
;
;   X = The number of Timer increments between the first and last
;           rising edge of the RS232 calibration character 0x0F. Other 
;       possible calibration characters are 0x01, 0x03, 0x07, 0x1F, 
;       0x3F, 0x7F.
;   [0]  Fosc = CPU oscillator frequency
;   [1]  Pre  = Timer0 Prescale
;   [2]  Tclk = Timer0 clock = Fosc/4
;   [3]  Ti = Timer Interval = Pre/Tclk
;   [4]  Bits = Number of data bits in X
;   [5]  Baud = Baud rate (i.e. Bits/Second)
;   [6]  X = (Bits/Baud)/Ti = Bits/Baud * Tclk/Pre
;   [8]  Baud = Bits/X * Fosc/(4*Pre)             ; from [6] and [2]
;   [9]  SPBRG = [Fosc/(16*(Baud))]-1             ; by definition when BRGH = 1
;   [10] SPBRG = [Fosc/16 * X/Bits * (4*Pre)/Fosc]-1  ; from [8] & [9]
;   [11] SPBRG = [(X*Pre)/(4*Bits)]-1             ; [10] reduced
;   [12] SPBRG = (X/2)-1                          ; from [11] assuming Pre = 2*Bits = 2*8 = 16
; Note: X/2 allows rounding up which changes the SPBRG uncertainty from +0/-1 bit to +/- 0.5 bit

;------------------------------------------------------------------------------
    BXtoB0
    
    clrf    FWBRG                   ; clear firmware baudrate generator
    clrf    msFWBRG
    
#ifdef INVERT_UART
    btfsc   PORTRX, RXPIN           ; ensure RX is IDLE to begin with
#else
    btfss   PORTRX, RXPIN           ; ensure RX is IDLE to start
#endif
    goto    $-1
#ifdef INVERT_UART
    btfss   PORTRX, RXPIN           ; wait for a high on pin - start bit
#else
    btfsc   PORTRX, RXPIN           ; Wait for a low on pin - start bit
#endif
    goto    $-1
#ifdef INVERT_UART
    btfsc   PORTRX, RXPIN           ; look for edge
#else
    btfss   PORTRX, RXPIN           ; Look for first rising edge
#endif
    goto    $-1

; count instruction cycle loops between rising edges of calibration character
;  this is the number of instruction loops in one byte time
;   each loop consists of 8 instruction cycles

STXHigh:
    incfsz  FWBRG,f
    goto    STXH20
    incf    msFWBRG,f
STXH20:
    goto    $+1
#ifdef INVERT_UART
    btfss   PORTRX, RXPIN           ; wait for high on pin
#else
    btfsc   PORTRX, RXPIN           ; Wait for a low on pin
#endif
    goto    STXHigh
STXLow:
    incfsz  FWBRG,f
    goto    STXL20
    incf    msFWBRG,f
STXL20:
    goto    $+1
#ifdef INVERT_UART
    btfsc   PORTRX, RXPIN           ; wait for second edge
#else
    btfss   PORTRX, RXPIN           ; Wait for second rising edge
#endif
    goto    STXLow

; byte count is in FWBRG - Now compute bit count

    movlw   B'00000010'             ; round up byte count
    addwf   FWBRG,f
BTCompute:
    rrf     msFWBRG,f               ; divide byte count by 4
    rrf     FWBRG,f
    rrf     msFWBRG,f
    rrf     FWBRG,f

; *****************************************************************************
