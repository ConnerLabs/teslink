#include <p18f4431.inc>

; Teslink receiver
; V1.0
; Steve Conner, May 2014
;
; This code is released in the public domain
; I make no warranties regarding the suitability of this code for any purpose
; and disclaim all liability to the extent permitted by law.
;
; This version is configured as follows:
; Processor: PIC18F4431 with 10MHz oscillator and 4x PLL (40MHz clock)
;
; TOSLINK receiver to pin 26 (RC7/RX)
; Interrupter signal from pin 20 (RD1/SDO)
; Data valid LED on pin 18 (RC3)
; Subcode valid LED on pin 15 (RC0)
; Duty cycle limit warning LED on pin 16 (RC1)
;
; Subcode data size of 5 bytes
; Byte 0: Always link code
; Byte 1: DC bus voltage command bits 9:8 (right justified)
; Byte 2: DC bus voltage command bits 7:0
; Byte 3: Fine tune voltage command bits 9:8 (right justified)
; Byte 4: Fine tune voltage command bits 7:0
;
; DC bus voltage is output as 2.4kHz 10 bit PWM on pins 33 and 34 (RB0/PWM0, RB1/PWM1)
; Fine tune voltage is output as 2.4kHz 10 bit PWM on pins 37 and 38 (RB4/PWM5, RB5/PWM4)
;
; Fail safe state:
; Interrupter: OFF
; DC bus voltage: 0
; Fine tune: 50%
;
; These values must match the ones programmed into the transmitter
; LINK_CODE can be any 8-bit value except 0
#define SUBCODE_SIZE .9
#define LINK_CODE 'T'

; Duty cycle limiter
; Maximum allowable pulse width in units of 5us.
; 400us = 80
#define DL_MAX_LENGTH .120

; Maximum allowable duty cycle as a fraction
; eg DL_DUTY_NUM=1, DL_DC_DEN_X8=1: 1/8 = 12.5%
; Duty cycle numerator
#define DL_DUTY_NUM .2

; Duty cycle denominator (will be multiplied by 8)
#define DL_DC_DEN_X8 .3

; CONFIG1H
  CONFIG  OSC = HSPLL           ; Oscillator Selection bits (HS oscillator, PLL enabled (clock frequency = 4 x FOSC1))
  CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
  CONFIG  IESO = ON             ; Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)

; CONFIG2L
  CONFIG  PWRTEN = OFF          ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  BOREN = ON            ; Brown-out Reset Enable bits (Brown-out Reset enabled)
; BORV = No Setting

; CONFIG2H
  CONFIG  WDTEN = ON           ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
  CONFIG  WDPS = 1          ; Watchdog Timer Postscale Select bits (1:32768)
  CONFIG  WINEN = OFF           ; Watchdog Timer Window Enable bit (WDT window disabled)

; CONFIG3L
  CONFIG  PWMPIN = OFF          ; PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
  CONFIG  LPOL = HIGH           ; Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
  CONFIG  HPOL = HIGH           ; High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
  CONFIG  T1OSCMX = ON          ; Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)

; CONFIG3H
  CONFIG  FLTAMX = RC1          ; FLTA MUX bit (FLTA input is multiplexed with RC1)
  CONFIG  SSPMX = RD1           ; SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RD3 and RD2, respectively. SDO output is multiplexed with RD1.)
  CONFIG  PWM4MX = RB5          ; PWM4 MUX bit (PWM4 output is multiplexed with RB5)
  CONFIG  EXCLKMX = RC3         ; TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RC3)
  CONFIG  MCLRE = ON            ; MCLR Pin Enable bit (Enabled)

; CONFIG4L
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
  CONFIG  LVP = OFF              ; Low-Voltage ICSP Enable bit (Low-voltage ICSP enabled)

; CONFIG5L
  CONFIG  CP0 = OFF             ; Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
  CONFIG  CP1 = OFF             ; Code Protection bit (Block 1 (001000-001FFF) not code-protected)
  CONFIG  CP2 = OFF             ; Code Protection bit (Block 2 (002000-002FFFh) not code-protected)
  CONFIG  CP3 = OFF             ; Code Protection bit (Block 3 (003000-003FFFh) not code-protected)

; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protection bit (Data EEPROM not code-protected)

; CONFIG6L
  CONFIG  WRT0 = OFF            ; Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
  CONFIG  WRT1 = OFF            ; Write Protection bit (Block 1 (001000-001FFF) not write-protected)
  CONFIG  WRT2 = OFF            ; Write Protection bit (Block 2 (002000-002FFFh) not write-protected)
  CONFIG  WRT3 = OFF            ; Write Protection bit (Block 3 (003000-003FFFh) not write-protected)

; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM not write-protected)

; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Table Read Protection bit (Block 2 (002000-002FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Table Read Protection bit (Block 3 (003000-003FFFh) not protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)


	org	0

; variables
	CBLOCK	0x008
	uart1
	uart2

	uart_field
	uart_err
	uart_timeout

	spi_out

	spi_out_prev

	spi_mod

	temp1
	temp2
	temp3
	temp4

	mdecstate
	mdec_in
	mdec_out
	mdec_err

	dibit_index
	dibit_value

	doffset

	valid_state

    duty_acc
    duty_ones
    duty_phase

    duty_out

	subcode_data: SUBCODE_SIZE
	ENDC

; timer setup for SSP clock
; For 40MHz CPU clock:
; 18 = 200kHz interrupter bit rate
	movlw	18	
	movwf	PR2
	bsf		T2CON,2


; SSP setup- configure as master

;	bcf		TRISC,5
;	bcf		TRISC,3
;	bcf		TRISC,0

; sorry mr Toslink- pin was getting set as an output
	movlw	B'11110100'
	movwf	TRISC

;    clrf    TRISC
    clrf    PORTC


	bcf		TRISD,1
	bcf 	TRISD,3

	movlw	B'00000000'
	movwf	SSPSTAT

	movlw	B'00100011'
	movwf	SSPCON

; UART setup
	
	movlw 	B'01001010'
	movwf	BAUDCTL

; For 40MHz CPU clock:
; 9 = 1Mbit/s 
	movlw	9
	movwf	SPBRG

	movlw 	0
	movwf	SPBRGH

	movlw	B'00100110'	
	movwf	TXSTA

	movlw	B'10010000'
	movwf	RCSTA

; PWM output setup

; Turn PWM off in case it was previously on.
	bcf		PTCON1,PTEN

; set PWM pins to 0
	bcf		PORTB,1
	bcf		PORTB,3
	bcf		PORTB,4
	

; PWM1, 3, 5, 7 enabled, independent mode
	bcf		TRISB,1

; Ignoring PWM3 for now because it is connected to PGM on the demo board
	bsf		TRISB,3

	bcf		TRISB,4

	movlw	B'01011111'
	movwf	PWMCON0

; 10 bit resolution, 39kHz PWM freq
; or something
	movlw	3
	movwf	PTPERH
	movlw	H'FF'
	movwf	PTPERL

; prescaler
	bsf		PTCON0,2

; Turn PWM on
	bsf		PTCON1,PTEN


; variable setup
reinit
	clrf	mdecstate
	clrf	spi_out
	clrf	spi_out_prev
	clrf	uart_err
	clrf	valid_state	

	movlw	D'255'
	movwf	uart_timeout

    movlw   DL_DC_DEN_X8
    movwf   duty_phase

; Initialise outputs to safe state
; turn off the data valid LED
	bcf		PORTC,3
	bcf		PORTC,0
    bcf     PORTC,1

; Send all 0s out of the SSP
	bcf		PORTD,1	; clear the SSP output pin
	bcf		SSPCON,SSPEN ; reset the SSP
	bsf		SSPCON,SSPEN
	movlw	0
	movwf	SSPBUF

; Initial values for subcode outputs
; Clear the link code
	movlw	0
	movwf	subcode_data
; (The other subcode channels get their initial values in out_upd)

; Transfer subcode values to outputs
	rcall	out_upd

; todo: analog and digital GPIOs



; test loop
; loop polls for data arriving in the UART
; (and also the timer to allow unsticking if data stops coming)
loop
	dcfsnz	uart_timeout,F
	bra		reinit	; if a timeout has happened, immediately restart
	btfss	PIR1,RCIF
	bra		loop

; save the data
; is it a field 0 byte?
	btfsc	RCREG,0
	goto	field1

; if field 0, load the SPI immediately, as field 0 is the timing reference.
; The contents of spi_out were calculated based on the values of fields 0 and 1 
; previously received, so we do not need to do any error checking now.
	bcf		SSPCON,WCOL
	movf	spi_out,W
	movwf	SSPBUF

; check for write collision (previous SPI data not finished transmitting)
	btfss	SSPCON,WCOL
	bra		spidone

; If a write collision occurred, the data we just wrote was ignored.
; So, reset the SPI and write it again.
; There will be a little dropout while the SPI is reset, as the SDO pin
; changes back to a GPIO, so fill it in by loading the last bit of the
; previous SPI packet onto this pin.
	bcf		PORTD,1
	btfsc	spi_out_prev,0
	bsf		PORTD,1

	bcf		SSPCON,SSPEN
	bsf		SSPCON,SSPEN
	movf	spi_out,W
	movwf	SSPBUF
	
	
spidone
	movlw	D'255'
	movwf	uart_timeout
	movff	spi_out, spi_out_prev

; store field 0
	movf	RCREG,W
	movwf	uart1
	clrf	uart_field
	bra		fielddone	
	
; store field 1
field1
	movf	RCREG,W
	movwf	uart2
	movlw	1
	movwf	uart_field
	
fielddone
; Time critical stuff done, get on with less important issues
; Capture the UART error flags for later reference
	movf	RCSTA,W		
	andlw	B'110'		
	iorwf	uart_err,F		
	
; Now decode the data
	movf	uart_field,w
	bnz		dec_f1

; Field 0 decoding
; Check the parity for field 0
	movf	uart1,W
	andlw	B'11111101'	; the calculation doesn't include the parity bit itself
	rcall	parity

; compare result with field 0's parity bit
	movwf	temp1
	rlncf	temp1,W	; the parity bit is bit 1
	xorwf	uart1,W

; move the result 2 bits left
	movwf	temp1
	rlncf	temp1,F
	rlncf	temp1,W
	andlw	B'1000'

; combine it with the uart_err register
	iorwf	uart_err,F

; Skip field 1 decoding as we are doing field 0
	bra		decdone

; Field 1 decoding (and Manchester decoding)
dec_f1
; Check the parity for field 1
	movf	uart2,W
	andlw	B'11111101'	; the calculation doesn't include the parity bit itself
	rcall	parity

; compare result with field 0's parity bit
	movwf	temp1
	rlncf	temp1,W	; the parity bit is bit 1
	xorwf	uart2,W

; move the result 3 bits left
	movwf	temp1
	rlncf	temp1,F
	rlncf	temp1,F
	rlncf	temp1,W
	andlw	B'10000'

; combine it with the uart_err register
	iorwf	uart_err,F


; Error check
; If uart_err=0 there have been no errors, so go ahead and update the outputs.
; Otherwise go to a fail-safe mode
; For now just light an LED if the data is good
	movf	uart_err,W
	bz		ecok

; errors
;	bcf		PORTC,3
;	clrf	uart_err
;	bra		ecdone
	bra		reinit
; No errors
ecok
	bsf		PORTC,3	; light LED

ecdone
; Clear watchdog timer
; can we use watchdog in debug mode? :|
	clrwdt

; Demodulate the SPI
	movf	uart1,W
	andlw	B'11110000'
	movwf	spi_mod

	swapf	uart2,W
	andlw	B'00001111'
	iorwf	spi_mod,F

	movf	spi_mod,W
	xorlw	B'01010101'
	movwf	spi_out

; Check the SPI duty cycle
; duty_out will be all 1s if the duty cycle is within limits
; or all 0s if it is excessive
    rcall   dclimit
    movf    duty_out,W
    andwf   spi_out,f

; Also drive a duty limit warning light
    btfsc   duty_out,0
    bcf     PORTC,1
    btfss   duty_out,0
    bsf     PORTC,1

; If validation failed, then send a failsafe value instead, which is 0 for the SPI.
	movf	valid_state,W
	btfsc	STATUS,Z
	clrf	spi_out
	
; Manchester decode and subcode state machine
; Combine the data from the UART fields into the 4 bit manchester encoded word
; Most significant 2 bits are stored in uart1 bits 3:2, least significant in uart2 bits 3:2
	movf	uart2,W
	andlw	B'00001100'
	movwf	mdec_in

	rrncf	mdec_in,F
	rrncf	mdec_in,F

	movf	uart1,W
	andlw	B'00001100'
	iorwf	mdec_in

	movlw	B'00001111'
	andwf	mdec_in,F

; Pass it through the Manchester decoding table
	movf	mdec_in,W
	rcall	mdec
	movwf	dibit_value

; Detect sync or invalid words
	movf	dibit_value,W
	andlw	B'11000000'
	bz		not_sync

; Subcode validation

; subcode_valid returns 1 in W if valid, 0 if invalid

	rcall	subcode_valid
	addlw	0
	bz		not_valid

; Subcode data is valid, update the outputs
	bsf		PORTC,0	; subcode valid LED
	movlw	1
	movwf	valid_state

	bra		valid_or_not

; Subcode data is not valid
not_valid
	bcf		PORTC,0	; subcode valid LED
	clrf	valid_state

valid_or_not
; Update the subcode outputs (failsafe values will be sent if validation failed)
	rcall	out_upd

; Clear the link code so it has to be received afresh for validity
	clrf	subcode_data

; Reset the state to 0, whether the data is valid or not
	clrf	mdecstate
;	bcf		PORTC,0

; Skip the rest of the Manchester decoding, as the sync (or invalid) word produces
; no output
	bra		mdecskip


; Handle a regular data word
not_sync

; calculate data area offset and dibit index from state counter
	movf	mdecstate,W
	movwf	dibit_index
	movlw	B'11'
	andwf	dibit_index,f
	
	movf	mdecstate,W
	movwf	doffset

	bcf		STATUS,C
	rrcf	doffset,f
	bcf		STATUS,C
	rrcf	doffset,f

; Collect 4 dibits to make an 8-bit word
; The least significant ones come first with dibit_index=0
; So, we load the dibits into the 2 most significant places
; and rotate them right

	rrncf	mdec_out,F
	rrncf	mdec_out,F

	rlncf	dibit_value,F
	rlncf	dibit_value,F
	swapf	dibit_value,W
	andlw	B'11000000'

	bcf		mdec_out,7
	bcf		mdec_out,6
	iorwf	mdec_out,F

; If dibit index=3, the word is complete, so write it out to the subcode data area
	movf	dibit_index,W
	sublw	3
	bnz		no_write

; Do a bounds check to ensure we haven't gone off the end of the subcode data area
	movf	doffset,w
	sublw	SUBCODE_SIZE
	bn		no_write

; write		
	lfsr	0,subcode_data
	movf	doffset,w
	movff	mdec_out,PLUSW0

no_write	

; increment the state
	incf	mdecstate,F

mdecskip
decdone

	bra		loop

org 0200
; Output update subroutine
; Move data from the subcode data area to the output registers
; If valid_state=0 then send the failsafe/init values
out_upd
	movf	valid_state,W
	bnz		out_valid
; Initial values
; DC bus voltage command -> 0
	movlw	0
	movwf	(subcode_data+1)

	movlw	0
	movwf	(subcode_data+2)

; Fine tune -> 50%
	movlw	2
	movwf	(subcode_data+3)

	movlw	0
	movwf	(subcode_data+4)

	bra		out_invalid

; debug trap
out_valid
	movf	(subcode_data+1),W
	bz		out_invalid
	nop		; insert breakpoint here

out_invalid

; adjust the data by shifting two places left
	bsf		PWMCON1,UDIS

	movf	(subcode_data+1),W
	movwf	temp3

	movf	(subcode_data+2),W
	movwf	temp4

	rlcf	temp4,f
	rlcf	temp3,f
	rlcf	temp4,f
	rlcf	temp3,f

	movf	temp3,w
	andlw	B'1111'
	movwf	PDC0H

	movf	temp4,w
	andlw	B'11111100'
;	iorlw	B'10'
	movwf	PDC0L

	movf	(subcode_data+3),W
	movwf	temp3

	movf	(subcode_data+4),W
	movwf	temp4

	rlcf	temp4,f
	rlcf	temp3,f
	rlcf	temp4,f
	rlcf	temp3,f

	movf	temp3,w
	andlw	B'1111'
	movwf	PDC2H

	movf	temp4,w
	andlw	B'11111100'
;	iorlw	B'10'
	movwf	PDC2L


	
	bcf		PWMCON1,UDIS

	return

; Manchester decoding table
; Converts 4 manchester encoded bits to 2 decoded ones
; Bit 7 of output set -> input invalid
; Bit 6 of output set -> input is sync word
mdec
	movwf	temp1
	movlw	HIGH(mdeclut)
	movwf	PCLATH
	rlncf	temp1,w
	andlw	B'11111'
	addwf	PCL,f

mdeclut
	retlw	B'01000000' ; 0000 -> sync
	retlw	B'10000000' ; 0001 -> invalid
	retlw	B'10000000' ; 0010 -> invalid
	retlw	B'10000000' ; 0011 -> invalid
	retlw	B'10000000' ; 0100 -> invalid
	retlw	B'00000000' ; 0101 -> 00
	retlw	B'00000001' ; 0110 -> 01
	retlw	B'10000000' ; 0111 -> invalid
	retlw	B'10000000' ; 1000 -> invalid
	retlw	B'00000010' ; 1001 -> 10
	retlw	B'00000011' ; 1010 -> 11
	retlw	B'10000000' ; 1011 -> invalid
	retlw	B'10000000' ; 1100 -> invalid
	retlw	B'10000000' ; 1101 -> invalid
	retlw	B'10000000' ; 1110 -> invalid
	retlw	B'10000000' ; 1111 -> invalid


;	retlw	B'0101'
;	retlw	B'0110'
;	retlw	B'1001'
;	retlw	B'1010'
;	retlw	B'0000'
;	retlw	B'0000'
;	retlw	B'0000'
;	retlw	B'0000'

; calculate parity
; Uses the algorithm
;   p = p ^ (p >> 4 | p << 4);
;   p = p ^ (p >> 2);
;   p = p ^ (p >> 1);
;   return p & 1;
; 	

parity
	movwf	temp1
	swapf	temp1,F ; temp1 contains (p>>4 | p<<4)

	xorwf	temp1,F ; temp1 contains p ^ (p>>4|p<<4)
	movf	temp1,W

	movwf	temp2
	rrcf	temp2,F
	rrcf	temp2,W
	andlw	B'00111111'	; W contains (p >> 2)
	xorwf	temp1,F
	movf	temp1,W

	movwf	temp2
	rrcf	temp2,W
	andlw	B'01111111'	; W contains (p >> 1)
	xorwf	temp1,W

	andlw	B'1'
	return




; alternate version suggested by bjorn
;	movwf	temp1
;	swapf	temp1,F
;	xorwf	temp1,W
;	movwf	temp1
;	rrcf	temp1,F
;	rrcf	temp1,F
;	xorwf	temp1,F
;	rrcf	temp1,W
;	xorwf	temp1,W
;	andlw	B'00000001'

; Subcode validator
; To be called on reception of the sync word or invalid word
; returns 1 in W if valid, 0 if invalid
subcode_valid
; If bit 7 of dibit_value set, we just received an invalid word instead of the sync word.
	btfsc	dibit_value,7
	retlw	0

; If doffset is not equal to (SUBCODE_SIZE-1) then we didn't receive the correct number
; of subcode packets since the last sync word.
	movf	doffset,W
	sublw	(SUBCODE_SIZE-1)
	btfss	STATUS,Z
	retlw	0

; If the transmitted link code is not equal to the one programmed in the receiver then
; something went wrong
	movf	subcode_data,W
	sublw	LINK_CODE
	btfss	STATUS,Z
	retlw	0


; If we survived to here then the subcode is valid
	retlw	1

; Duty cycle limiter
dclimit
; Count the ones in spi_out
    movf    spi_out,w
    rcall   ones_count
    movwf   duty_ones
    swapf   spi_out,w
    rcall   ones_count
    addwf   duty_ones,f

; Add the ones to the duty accumulator
    movf    duty_ones,w
    addwf   duty_acc,f

; If overflowed, saturate it to the max possible value.
    bnn     dcl_no
    movlw   .127
    movwf   duty_acc

dcl_no
; If the duty accumulator is beyond the max allowable value,
; blank out the pulse output.
    setf    duty_out
    movlw   DL_MAX_LENGTH
    cpfslt  duty_acc
    clrf    duty_out     ; clear the pulse output

; Periodically subtract some ones from the accumulator
; thus setting the average duty cycle (they can only go in as
; fast as they are taken out)

    decfsz  duty_phase,f
    bra     dcl_nn

    movlw   DL_DC_DEN_X8
    movwf   duty_phase

    movlw   DL_DUTY_NUM
    subwf   duty_acc,f
    bnn     dcl_nn
    clrf    duty_acc
 dcl_nn

    return

org 0300
; Lookup table returns the number of ones in a nibble
ones_count
	movwf	temp1
	movlw	HIGH(oclut)
	movwf	PCLATH
	rlncf	temp1,w
	andlw	B'11111'
	addwf	PCL,f

oclut
    retlw	0 ; 0000 -> 0
	retlw	1 ; 0001 -> 1
	retlw	1 ; 0010 -> 1
	retlw	2 ; 0011 -> 2
	retlw	1 ; 0100 -> 1
	retlw	2 ; 0101 -> 2
	retlw	2 ; 0110 -> 2
	retlw	3 ; 0111 -> 3
	retlw	1 ; 1000 -> 1
	retlw	2 ; 1001 -> 2
	retlw	2 ; 1010 -> 2
	retlw	3 ; 1011 -> 3
	retlw	2 ; 1100 -> 2
	retlw	3 ; 1101 -> 3
	retlw	3 ; 1110 -> 3
	retlw	4 ; 1111 -> 4


; End of program
	end
	
	

	
