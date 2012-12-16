; ___    __________            ________
;    \__/          \__________/
;       |                     |
;       |-------- p ----------|
;
;   p = The number of instructions between the first and last
;           rising edge of the RS232 control sequence 0x0F. Other 
;       possible control sequences are 0x01, 0x03, 0x07, 0x1F, 
;       0x3F, 0x7F.
;
;   SPBRG = (p / 32) - 1    BRGH = 1, BRG16 = 0
;   SPBRG = (p / 8) - 1     BRGH = 1, BRG16 = 1

#ifdef BSR
    banksel RCSTA
#else
    BXtoB0                          ; Bx -> B0
#endif
    bcf     RCSTA, CREN             ; B0 Stop UART RX, we're going to do autobaud pulse width timing instead
    movf    RCREG, W                ; B0 Empty the UART receive buffer
    movf    RCREG, W                ; B0
#ifdef BSR
    banksel TMR1H
#endif
RetryAutoBaud:
#ifdef BRG16
    clrf    TMR1H                   ; B0 reset timer count value
    clrf    TMR1L                   ; B0
    bcf     PIR1, TMR1IF            ; B0
#else
    bcf     STATUS, C               ; Bx do not rotate in anything but 0
#endif
    call    WaitForRise             ; B0 wait for a start bit to pass by

#ifdef BRG16
    bsf     T1CON, TMR1ON           ; B0 start timer counting for entire D7..D0 data bit period.
#else
    clrf    TMR0                    ; B0 restart counting
#endif
    call    WaitForRise             ; B0 wait for stop bit
#ifdef BRG16
    bcf     T1CON, TMR1ON           ; B0 stop the timer from counting further. 
    btfsc   PIR1, TMR1IF            ; B0 if TMR1 overflowed, we did not get a good baud capture
    goto    RetryAutoBaud           ; try again

    ; save new baud rate generator value
    movf    TMR1L, w                ; B0 warning: must read TMR0L before TMR0H holds real data
  #ifdef BSR
    banksel SPBRG
  #else
    B0toB1                          ; B0 -> B1
  #endif
    movwf   SPBRG                   ; B1
  #ifdef BSR
    banksel TMR1H
  #else
    BXtoB0                          ; B1 -> B0
  #endif
    movf    TMR1H, w                ; B0
  #ifdef BSR
    banksel SPBRGH
  #else
    B0toB1                          ; B0 -> B1
  #endif
    movwf   SPBRGH                  ; B1
#else  ; not BRG16
    movf    TMR0, w
    movwf   FSR
    rrf     FSR, w                  ; Bx divide timer value by 2 and save in WREG
    nop
    nop
    btfss   STATUS, C               ; Bx do we need to round down?
    addlw   0xFF                    ; yes, round down

  #ifdef BSR
    banksel SPBRG
  #else
    B0toB1                          ; B0 -> B1
  #endif
    movwf   SPBRG                   ; B1 set new baud rate
#endif ; BRG16
