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

.include "../inc/tn13def.inc"

;--------------------------------------------------------------------------------
; DHT22 sensor
.equ _DHTDDR, DDRB
.equ _DHTPIN, PINB
.equ _DHTPORT, PORTB
.equ _DHTBIT, PINB0

;--------------------------------------------------------------------------------
; LCD with 3-Wire interface
.set __DISPLAY_LINES__, 2	; number of display lines

; define control bits
.equ _RS, 1     ; register select: data=1, control=0
.equ _RW, 0     ; the RW pin is set to ground
.equ _EN, 2     ; enable: falling edge
.equ _SD, _RS   ; serial data (shared with lcd register select)
.equ _SC, 4     ; serial clock, RW bit can be used.warning "3-wire version: no display reading possible"

.equ CtrlOut, (1<<_RS | 1<<_EN | 1<<_SC)   ; bit pattern for DDR control port

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
.set rDivL, 0
.set rDivH, 1
.set rSensorData, 2
.set rChecksum, 3
.set rTemp1, 16
.set rTemp2, 17
.set rRemainder , 18
.set rData, 22
.set rCtrl, 23
.set rCounterL, 24
.set rCounterH, 25
.set rLoop1, 26
.set rLoop2, 27
.set rAddr, 28
.set rLoop3, 29
.set rPointerL, 30      ; ZL
.set rPointerH, 31      ; ZH

;--------------------------------------------------------------------------------
; delay
.ifndef __F_CPU
    .set __F_CPU, 6000000   ; 6MHz
.endif

; you can determine the corection factor with DelayCalc.ods
.equ cFactor, 2                             ; correction factor
.equ dLoop, (__F_CPU / 4 / 1000) - cFactor  ; ms loop counter

.if (__F_CPU == 6000000)        ; clock 6MHz
    .warning "6MHz"
    .macro mDelay_us us
        .if (\us - 1)
            .set cycles, (\us - 1)
            ldi rCounterL, lo8(cycles)
            ldi rCounterH, hi8(cycles)
            rcall uswait
        .else
            .rept 6
                nop
            .endr
        .endif
    .endm
.elseif (__F_CPU >= 12000000)   ; clock 12MHz+
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
        .warning "clear | home instruction"
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

bits:   .space 4    ; 4 byte data from sensor -> humidityH:humidityL:temperatureH:temperatur:L

.text
.org 0x0000

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

    ldi rTemp1, 0x80        ; System Clock Prescaler
    out CLKPR, rTemp1
    clr rTemp1
    out CLKPR, rTemp1

    in rTemp1, CtrlDDR      ; switch control port to output
    ori rTemp1, CtrlOut
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

mainloop:
    mDispCtrl 0b10001111    ; set cursor to 1st line, last column
    rcall readDHT
    mDelay_ms 1000
    rjmp mainloop

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

.func newCHR            ; addr -> rAddr, ^dataarray -> rPointerH:rPointerL)

newCHR:
    lsl rAddr           ; rAddr * 8
    lsl rAddr
    lsl rAddr
    ori rAddr, 0x40    ; set CGRAM set address bit

    ldi rLoop2, 8       ; 8 bit per byte ;-)
    clr rCtrl           ; RS -> ctrl
    mov rData, rAddr    ; CGRAM address
    rcall writeLCD

.shift2loop:
    ldi rCtrl, _DATA    ; RS - data
    lpm rData, Z+       ; read data from flash
	; ld r22, Z+      ; read data from RAM
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

    CBI _DHTDDR, _DHTBIT    ; pin -> input
    mDelay_us 40

    sbic _DHTPIN, _DHTBIT
    rjmp error
    mDelay_us 80
    sbis _DHTPIN, _DHTBIT
    rjmp error
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
    breq end                ; all bytes done
    st Z+, rSensorData      ; save byte in result
    add rChecksum, rSensorData  ; calculate checksum -> rChecksum
    rjmp DataLoop

end:
    sub rChecksum, rSensorData  ; is checksum correct -> rChecksum == 0

    sbiw rPointerL, 3       ; Z points to first temperatur byte
    ld rTemp1, Z            ; MSB is sign bit
    mov r2, rTemp1          ; r2 & 0x80 mask sing bit
    andi rTemp2, 0x7f		; delete sign bit
    st Z, rTemp2

output:
    ldi rPointerL, lo8(bits)    ; RAM for sensor data
    ldi rPointerH, hi8(bits)

    ldi rTemp1, 0x01        ; ok -> ♥
    tst rChecksum           ; error ?
    breq ok
    inc rTemp1              ; display error sign on LCD
ok:
    ldi rCtrl, _DATA
    mov rData, rTemp1
    rcall writeLCD
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
    add rData, rRemainder
    rcall writeLCD
    ldi rData, '.'
    rcall writeLCD
    rcall divBy10       ; unit place
    ldi rData, 0x30
    add rData, rRemainder
    rcall writeLCD
    rcall divBy10       ; tens digit
    ldi rData, 0x30
    add rData, rRemainder
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
    add rData, rRemainder
    rcall writeLCD
    ldi rData, '.'
    rcall writeLCD
    rcall divBy10       ; unit place
    ldi rData, 0x30
    add rData, rRemainder
    rcall writeLCD
    rcall divBy10       ; tens digit
    tst rRemainder      ; suppress leading zero
    breq nozero
    ldi rData, 0x30
    add rData, rRemainder
    rcall writeLCD
nozero:
    ldi rData, '+'
    mov rTemp2, rSensorData
    andi rTemp2, 0x80   ; mask sign bit
    breq postemp
    ldi rData, '-'
postemp:
    rcall writeLCD

    ; sub r3, r2
    ; mov rCounterL, r3         ; return checksum

return_del:

    ret
error:
    inc rChecksum       ; error
    rjmp output

.endfunc                ; readDHT

.global uswait
.func uswait        ; void uswait(int µs -> rCounterL:rCounterH)

;   ldi rCounterL, 1        1
;   ldi rCounterH, 0        1
;   rcall uswait            3

uswait:
    sbiw rCounterL, 1   ; 2
.if (__F_CPU <= 12000000)   ; 6-12 MHz
    brne loop       ; 1|2
    ret             ; 4
.else
    breq return     ; 1|2
    rcall loop      ; 3
.endif
.if (__F_CPU == 20000000)   ; 20 MHz
loop:
    nop             ; 1
    nop             ; 1
    nop             ; 1
    nop             ; 1
.else
loop:
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

.global mswait
.func mswait            ; void mswait(const int ms -> rCounterL:rCounterH)

;    ldi r22,0       1
;    ldi r21,1       1
;    rcall mswait    3

mswait:
    push rLoop1        ; 2
    push r27        ; 2
    rjmp l1         ; 2
l3:
    sbiw rCounterL,1      ; 2
    brne l1         ; 1|2
    pop r27         ; 2
    pop rLoop1         ; 2
    ret             ; 4

l1:
    nop             ; 1
    ldi rLoop1, lo8(dLoop) ; 1
    ldi r27, hi8(dLoop) ; 1

l2:
    sbiw rLoop1,1      ; 2
    brne l2         ; 1|2

    rjmp l3         ; 2
.endfunc            ; mswait


.func divBy10
divBy10:
div:    ldi     rLoop3, 16  ; 16 bit
        clr     rRemainder  ; initialize rRemainder
dv1:    lsl     rDivL       ; shift dividend ...
        rol     rDivH       ; ... to carry
        rol     rRemainder  ; carry -> remainder
        ;brcs    dv3         ; overflow remainder
        cpi     rRemainder, 10  ; shift until remainder >= 10
        brcs    dv2         ; if remainder < 10, set to "0"
dv3:    inc     rDivL       ; else set to "1" and reduce remainder by 10
        subi    rRemainder ,10
dv2:    dec     rLoop3      ; 16 bit done ?
        brne    dv1
        ret
.endfunc

LineAddress:    .byte 0x00, 0x40, 0x10, 0x50    ; line # 1, 2, 3, 4 (16x?)
heartU:         .byte 0x00, 0x0a, 0x15, 0x11, 0x11, 0x0a, 0x04, 0x00    ; unfilled heart
heartF:         .byte 0x00, 0x0a, 0x1f, 0x1f, 0x1f, 0x0e, 0x04, 0x00    ; filled hear

; CRCErr:		.asciz "CRC error"
; SensorErr:	.asciz "Sensor error"
; Test:           .asciz "Test1234!"

.end
