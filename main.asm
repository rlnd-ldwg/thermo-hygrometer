; thermohygrometer based on DHT22 and ATtiny13
; copyright (c) 2019 Roland Ludwig
;
; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License along
; with this program; if not, write to the Free Software Foundation, Inc.,
; 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

; v0.1.0 2020-01-21
; v0.1.1 2020-01-22
; v0.1.2 2020-01-26
; v0.1.3 2020-01-27
; v1.0.0 2020-01-28
; v1.0.1 2020-01-01: added blinking hearbeat and error message; sorce code optimization
; v1.0.2 2020-02-07: using sleep mode and watchdog for reactivation

.include "../inc/tn13def.inc"

;--------------------------------------------------------------------------------
; DHT22 sensor
.equ _DHTDDR, DDRB
.equ _DHTPIN, PINB
.equ _DHTPORT, PORTB
.equ _DHTBIT, PINB0

;--------------------------------------------------------------------------------
; LCD with 3-Wire interface
.equ __DISPLAY_LINES__, 2    ; number of display lines

; define control bits
.equ _RS, 1     ; register select: data=1, control=0
.equ _RW, 0     ; the RW pin is set to ground
.equ _EN, 2     ; enable: falling edge
.equ _SD, _RS   ; serial data (shared with lcd register select)
.equ _SC, 4     ; serial clock, RW bit can be used.warning "3-wire version: no display reading possible"

; port definition
.equ DataDDR, DDRB
.equ DataWrite, PORTB
.equ DataRead, PINB
.equ CtrlDDR, DDRB
.equ CtrlWrite, PORTB
.equ CtrlRead, PINB

; LCD register select
.equ _CTRL, 0
.equ _DATA, 1

;--------------------------------------------------------------------------------
; mcu registers
.equ rDivL, 0
.equ rDivH, 1
.equ rSensorData, 2
.equ rChecksum, 3
; .equ rWD_TestL, 4
; .equ rWD_TestH, 5
.equ rTemp1, 16
.equ rTemp2, 17
.equ rModulo , 18
.equ rHearbeat, 19
.equ rData, 22
.equ rCtrl, 23
.equ rCounterL, 24
.equ rCounterH, 25
.equ rLoop1, 26
.equ rLoop2, 27
.equ rAddr, 28
.equ rLoop3, 29
.equ rPointerL, 30      ; ZL
.equ rPointerH, 31      ; ZH

;--------------------------------------------------------------------------------
; delay
.ifndef __F_CPU
    .equ __F_CPU, 6000000   ; 6MHz
.endif

; you can determine the correction factor with DelayCalc.ods
.equ cFactor, 2                             ; correction factor
.equ dLoop, (__F_CPU / 4 / 1000) - cFactor  ; ms loop counter

.if __F_CPU == 6000000  ; clock 6MHz
    .warning "6MHz"
    .macro mDelay_us us
        .if (\us - 1)
            .equ cycles, (\us - 1)
            ldi rCounterL, lo8(cycles)
            ldi rCounterH, hi8(cycles)
            rcall uswait
        .else
            .rept 6
                nop
            .endr
        .endif
    .endm
.elseif __F_CPU >= 12000000 ; clock 12MHz+
    .warning "12MHz+"
    .macro mDelay_us us
        ldi rCounterL, lo8(\us)
        ldi rCounterH, hi8(\us)
        rcall uswait
    .endm
.endif

.macro mDelay_ms ms
    ldi rCounterL, lo8(\ms)
    ldi rCounterH, hi8(\ms)
    rcall mswait
.endm

.macro mDispCtrl cmd ; rData -> data, rCtrl -> register select
    ldi rData, \cmd
    clr rCtrl
    rcall writeLCD
    .if (\cmd < 4)  ; home or clear instruction needs 1,6ms execution time
        mDelay_us 1563
    .endif
.endm

.macro mNewChr addr char
    ldi rAddr, \addr          ; CGRAM address
    ldi rPointerL, lo8(\char)
    ldi rPointerH, hi8(\char)
    rcall newCHR
.endm

.data

bits:   .space 4    ; byte data from sensor -> humidityH:humidityL:temperatureH:temperaturL

.text
; 0x0000 RESET          External Pin, Power-on Reset,Brown-out Reset, Watchdog Reset
; 0x0001 INT0           External Interrupt Request 0
; 0x0002 PCINT0         Pin Change Interrupt Request 0
; 0x0003 TIM0_OVF       Timer/Counter Overflow
; 0x0004 EE_RDY         EEPROM Ready
; 0x0005 ANA_COMP       Analog Comparator
; 0x0006 TIM0_COMPA     Timer/Counter Compare Match A
; 0x0007 TIM0_COMPB     Timer/Counter Compare Match B
; 0x0008 WDTWatchdog    Time-out
; 0x0009 ADC            ADC Conversion Complete

.org 0x0000
    rjmp main
.org WDTaddr*2          ; word address
    reti                ; do nothing expect wakeup the mcu

main:
    cli                     ; no interrupts

    ldi rTemp1, 0           ; reset system status
    out SREG, rTemp1

    ldi rTemp1, lo8(RAMEND) ; init stack pointer
    out SPL, rTemp1
    .if RAMEND > 0xff
        ldi rTemp1, hi8(RAMEND)
        out SPH, rTemp1
    .endif

    ldi rTemp1, 0x21        ; RC Oscillator Calibration
    out OSCCAL, rTemp1

    ldi rTemp1, 0x80        ; set System Clock Prescaler (CLKPCE = 1, to enable setting of prescaler) (1 << CLKPCE)
    out CLKPR, rTemp1
    clr rTemp1                ; to division factor 1
    out CLKPR, rTemp1

    ldi rTemp1, 1<<WDTIE | 1<<WDP3 | 0<<WDP1 | 0<<WDP0    ; ~2 sec
    out WDTCR, rTemp1

; debug
    ; clr rWD_TestL
    ; clr rWD_TestH

    ldi rTemp1, 1<<SE + 1<<SM1  ; sleep mode: power-down
    out MCUCR, rTemp1

    in rTemp1, CtrlDDR      ; switch control port to output
    ori rTemp1, 1<<_RS | 1<<_EN | 1<<_SC
    out CtrlDDR, rTemp1

; initialize LCD
    mDelay_us 15000
    mDispCtrl 0x30
    mDelay_us 4100
    mDispCtrl 0x30
    mDelay_us 100
    mDispCtrl 0x30
    .if __DISPLAY_LINES__ == 1
        mDispCtrl 0b00110000    ; 8 bit, one line, 5x7
    .else
        mDispCtrl 0b00111000    ; 8 bit, multiple lines, 5x7
    .endif
    mDispCtrl 0b00000001    ; clear display
    mDispCtrl 0b00001100    ; display on, cursor off, blink off

; create new characters for heartbeat
    mNewChr 1 heartU
    mNewChr 2 heartF

; write from right to left
    mDispCtrl 0b00000100    ; decrement address, cursor shift left
    sei                     ; allow interrupts
    rcall startMsg
    sleep

mainloop:
    mDispCtrl 0b10001111    ; set cursor to 1st line, last column
    ldi rCtrl, _DATA
    ldi rData, 2            ; filled heart
    rcall writeLCD
    rcall readDHT
    mDispCtrl 0b10001111    ; set cursor to 1st line, last column
    ldi rCtrl, _DATA
    ldi rData, 1            ; unfilled heart
    rcall writeLCD
    sleep
    rjmp mainloop

;--------------------------------------------------------------------------------
; watchdog test
; wdint:
;     mDispCtrl 0b11001111    ; set cursor to 2nd line, last column
;     ldi rCtrl, _DATA
;     movw rDivL, rWD_TestL
; convloop:
;     rcall divBy10
;     ldi rData, 0x30
;     add rData, rModulo
;     rcall writeLCD
;     tst rDivL                ; done ?
;     brne convloop
;     ldi rTemp1, 1
;     add rWD_TestL, rTemp1
;     clr rTemp1
;     adc rWD_TestH, rTemp1
;
; if WDE set you must set WDTIE to prevent the watchdog to reset the system
; WDTCR |= (1<<WDIE); set interrupt enable to prevent systen reset
;     ; in rTemp1, WDTCR
;     ; ori rTemp1, (1<<WDTIE)
;     ; out WDTCR, rTemp1
;
;     reti
;--------------------------------------------------------------------------------

.func writeLCD          ; rs -> rCtrl, data -> rData

writeLCD:
    mDelay_us 37        ; wait 37µs because no busyflag check in 3-wire mode
    ldi rLoop1, 8       ; 8 bit per byte ;-)
.shiftloop:
    cbi CtrlWrite, _SC  ; set clock to low
    lsl rData           ; shift bit to carry
    brcc .bitclr        ; carry == 0 => shift '0'
    sbi CtrlWrite, _SD  ; shift '1'
    rjmp .nxtbit
.bitclr:
    cbi CtrlWrite, _SD
.nxtbit:
    sbi CtrlWrite, _SC  ; rising edge to shift
    dec rLoop1
    brne .shiftloop     ; all bits done ?
    tst rCtrl           ; control or data register
    brne select2DataW
    cbi CtrlWrite, _RS
    rjmp select2CtrlW
select2DataW:
    sbi CtrlWrite, _RS
select2CtrlW:
    sbi CtrlWrite, _EN
    nop                 ; Enable puls wide 333s at 6MHz
    nop
    cbi CtrlWrite, _EN
    ret
.endfunc                ; writeLCD

.func newCHR            ; addr -> rAddr, ^dataarray -> rPointerH:rPointerL

newCHR:
    lsl rAddr           ; rAddr * 8
    lsl rAddr
    lsl rAddr
    ori rAddr, 0x40    ; set CGRAM set address bit

    ldi rLoop2, 8       ; 8 bit per byte ;-)
    clr rCtrl           ; RS -> ctrl
    mov rData, rAddr    ; CGRAM address
    rcall writeLCD

    ldi rCtrl, _DATA    ; RS - data
.shift2loop:
    lpm rData, Z+       ; read data from flash
    rcall writeLCD
    dec rLoop2
    brne .shift2loop    ; 8 bit done ?
    ret
.endfunc                ; newCHR

;--------------------------------------------------------------------------------
.func readDHT

readDHT:
    ldi rPointerL, lo8(bits); RAM for sensor data
    ldi rPointerH, hi8(bits)
    clr rChecksum

    sbi _DHTDDR, _DHTBIT    ; pin -> output
    sbi _DHTPORT, _DHTBIT   ; port -> high
    mDelay_ms 100

    cbi _DHTPORT, _DHTBIT   ; send request port -> low
    mDelay_us 500           ; for 500us
    sbi _DHTPORT, _DHTBIT

    cbi _DHTDDR, _DHTBIT    ; pin -> input
    mDelay_us 40

    sbic _DHTPIN, _DHTBIT
    rjmp errorMsg
    mDelay_us 80
    sbis _DHTPIN, _DHTBIT
    rjmp errorMsg
    mDelay_us 80

    ldi rLoop1, 5           ; receive 5 databytes
DataLoop:
    ldi rLoop2, 8           ; 8 bit per byte ;-)
BitLoop:
WaitHigh:
    sbis _DHTPIN, _DHTBIT
    rjmp WaitHigh           ; wait for high level

    mDelay_us 30            ; if high after 30µs -> bit is set
    clc                     ; carry = 0
    sbic _DHTPIN, _DHTBIT
    sec                     ; carry = 1 -> bit = 1
    rol rSensorData

WaitLow:
    sbic _DHTPIN, _DHTBIT
    rjmp WaitLow            ; wait for low level

    dec rLoop2              ; 8 bit done ?
    brne BitLoop            ; no, next bit

    dec rLoop1
    breq CalcChecksum       ; all bytes done
    st Z+, rSensorData      ; save byte in result
    add rChecksum, rSensorData  ; calculate checksum -> rChecksum
    rjmp DataLoop

CalcChecksum:
    sub rChecksum, rSensorData  ; is checksum correct -> rChecksum == 0
    brne errorMsg

    sbiw rPointerL, 3       ; Z points to first temperatur byte
    ld rTemp1, Z            ; MSB is sign bit
    bst rTemp1, 7           ; save temperature sign in "T" flag, T=1 -> negative temperature
    andi rTemp1, 0x7f       ; mask sign bit
    st Z, rTemp1

    ldi rPointerL, lo8(bits); sensor data is stored here
    ldi rPointerH, hi8(bits)

    ldi rCtrl, _DATA
    ldi rData, 'H'
    rcall writeLCD
    ldi rData, 'R'
    rcall writeLCD
    ldi rData, '%'
    rcall writeLCD
    ld rDivH, Z+        ; humidity
    ld rDivL, Z+
    rcall divBy10       ; decimal place
    ldi rData, 0x30
    add rData, rModulo
    rcall writeLCD
    ldi rData, '.'
    rcall writeLCD
    rcall divBy10       ; unit place
    ldi rData, 0x30
    add rData, rModulo
    rcall writeLCD
    rcall divBy10       ; tens digit
    ldi rData, 0x30
    add rData, rModulo
    rcall writeLCD
    ldi rData, ' '
    rcall writeLCD
    ldi rData, 'C'
    rcall writeLCD
    ldi rData, 0xdf     ; "°"
    rcall writeLCD
    ld rDivH, Z+        ; temperatur
    ld rDivL, Z+
    rcall divBy10       ; decimal place
    ldi rData, 0x30
    add rData, rModulo
    rcall writeLCD
    ldi rData, '.'
    rcall writeLCD
    rcall divBy10       ; unit place
    ldi rData, 0x30
    add rData, rModulo
    rcall writeLCD
    rcall divBy10       ; tens digit
    tst rModulo         ; suppress leading zero
    breq noprint
    ldi rData, 0x30
    add rData, rModulo
    rcall writeLCD
noprint:
    ldi rData, '+'
    brtc notnegative
    ldi rData, '-'
notnegative:
    rcall writeLCD
    ret

.endfunc                ; readDHT

startMsg:
    ldi rPointerL, lo8(Welcome)
    ldi rPointerH, hi8(Welcome)
    rjmp print

errorMsg:
    ldi rPointerL, lo8(SensorError)
    ldi rPointerH, hi8(SensorError)
print:
    mDispCtrl 0b10001111
    ldi rCtrl, _DATA
next:
    lpm rData, Z+
    tst rData
    breq end
    rcall writeLCD
    rjmp next
end:
    ret

.func uswait        ; µs -> rCounterL:rCounterH

;   ldi rCounterL, 1        1
;   ldi rCounterH, 0        1
;   rcall uswait            3

uswait:
    sbiw rCounterL, 1   ; 2
.if (__F_CPU <= 12000000)   ; 6-12 MHz
    brne l0         ; 1|2
    ret             ; 4
.else
    breq return     ; 1|2
    rcall l0        ; 3
.endif
.if (__F_CPU == 20000000)   ; 20 MHz
l0:
    nop             ; 1
    nop             ; 1
    nop             ; 1
    nop             ; 1
.else
l0:
.endif
.if (__F_CPU >= 12000000)   ; 12-20 MHz
    .rept 6
        nop         ; 1
    .endr
.endif
.if (__F_CPU <= 12000000)    ; 6-12 MHz
    rjmp uswait     ; 2
.else
return:
    ret             ; 4
.endif
.endfunc            ; uswait

.func mswait            ; ms -> rCounterL:rCounterH

;    ldi r22,0       1
;    ldi r21,1       1
;    rcall mswait    3

mswait:
    push rLoop1     ; 2
    push r27        ; 2
    rjmp l1         ; 2
l3:
    sbiw rCounterL,1; 2
    brne l1         ; 1|2
    pop r27         ; 2
    pop rLoop1      ; 2
    ret             ; 4

l1:
    nop             ; 1
    ldi rLoop1, lo8(dLoop)  ; 1
    ldi rLoop2, hi8(dLoop)  ; 1

l2:
    sbiw rLoop1,1   ; 2
    brne l2         ; 1|2

    rjmp l3         ; 2
.endfunc            ; mswait

.func divBy10

divBy10:
    ldi     rLoop3, 16  ; 16 bit
    clr     rModulo     ; initialize rModulo
dv1:
    lsl     rDivL       ; shift dividend ...
    rol     rDivH       ; ... to carry
    rol     rModulo     ; carry -> remainder
    ;brcs    dv3        ; overflow remainder
    cpi     rModulo, 10 ; shift until remainder >= 10
    brcs    dv2         ; if remainder < 10, set to "0"
dv3:
    inc     rDivL       ; else set to "1" and reduce remainder by 10
    subi    rModulo ,10
dv2:
    dec     rLoop3      ; 16 bit done ?
    brne    dv1
    ret
.endfunc

heartU:      .byte 0x00, 0x0a, 0x15, 0x11, 0x0a, 0x04, 0x00, 0x00    ; unfilled heart
heartF:      .byte 0x00, 0x0a, 0x1f, 0x1f, 0x0e, 0x04, 0x00, 0x00    ; filled hear
Welcome:     .asciz "): dnoces a tsuJ"
SensorError: .asciz "  ! rorre rosneS"

.end
