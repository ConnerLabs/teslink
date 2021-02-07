#include <p18f4431.inc>
; Teslink transmitter
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
; Interrupter signal to pin 21 (RD2/SDI)
; DC bus voltage command to pin 2 (RA0/AN0)
; Fine tune voltage command to pin 3 (RA1/AN1)
;
; Subcode data size of 5 bytes
; Byte 0: Always link code
; Byte 1: DC bus voltage command bits 9:8 (right justified)
; Byte 2: DC bus voltage command bits 7:0
; Byte 3: Fine tune voltage command bits 9:8 (right justified)
; Byte 4: Fine tune voltage command bits 7:0
;
; These values must match the ones programmed into the transmitter
; LINK_CODE can be any 8-bit value except 0
#define SUBCODE_SIZE .9
#define LINK_CODE 'T'

; DDS
#define BURST_BEGIN	0
#define BURST_ON 1
#define BURST_END 2
#define BURST_OFF 3
#define	BURST_DISABLE 4

#define NO_BIT 9

; CONFIG1H
  CONFIG  OSC = HSPLL           ; Oscillator Selection bits (HS oscillator, PLL enabled (clock frequency = 4 x FOSC1))
  CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
  CONFIG  IESO = ON             ; Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)

; CONFIG2L
  CONFIG  PWRTEN = OFF          ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  BOREN = ON            ; Brown-out Reset Enable bits (Brown-out Reset enabled)
; BORV = No Setting

; CONFIG2H
  CONFIG  WDTEN = OFF           ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
  CONFIG  WDPS = 32768          ; Watchdog Timer Postscale Select bits (1:32768)
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
	CBLOCK
;0x008
	uart1
	uart2
	spi_in
	spi_out

	spi_mod

	temp1
	temp2
	temp3
	temp4

	mencstate

	mencword

	dibit
	doffset
	txdelay

	prf_inc_h
	prf_inc_l

	len_inc_h
	len_inc_l

	dds_acc_u
	dds_acc_h
	dds_acc_l

	len_acc

	int_state

	dds_bit
	begin_bit
	begin_bit2

	dds_pulse_out

	debounce_ctr

    adc_phase

	subcode_data: SUBCODE_SIZE

   

	ENDC

; Knobs for DDS
	#define PRF_HI (subcode_data+5)
	#define PRF_LO (subcode_data+6)

	#define LEN_HI (subcode_data+7)
	#define LEN_LO (subcode_data+8)

; timer setup for SSP clock


	movlw	18	; 200kHz bit rate with 10MHz osc + PLL
;	movlw	13	; 200kHz bit rate with 8MHz osc + PLL
	movwf	PR2
	bsf		T2CON,2


; SSP setup- configure as master

	
;	bcf		TRISC,5
;	bcf		TRISC,3

	movlw	0
	movwf	TRISC

	bcf		TRISD,1
	bcf 	TRISD,3


	movlw	B'00000000'
	movwf	SSPSTAT


	movlw	B'00100011'
	movwf	SSPCON


; UART setup
; this is the transmitter, the UART only needs to send
	
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

; ADC setup

; ADCON0: Single shot mode, single channel, SCM1, A/D On
	movlw	B'000001'
	movwf	ADCON0

; ADCON1: Vref+ = VDD, Vref- =VSS, FIFO disabled
; these are the power-on defaults

; ADCON2: Right justified result, no acquisition delay, clock= Fosc/64
	movlw	B'10000110'
	movwf	ADCON2

; ADCON3: Interrupt select don't care, trigger source disabled
; (power-on defaults)

; ADCHS: Channel assignment defaults ok

; ANSEL0,1: defaults ok


; variable setup
	clrf	mencstate
	clrf	temp4
    clrf    adc_phase

; link code
	movlw	LINK_CODE
	movwf	subcode_data

; sensible-ish default subcode values
	movlw	0
	movwf	(subcode_data+1)
	movlw	0
	movwf	(subcode_data+2)
	movlw	2
	movwf	(subcode_data+3)
	movlw	0
	movwf	(subcode_data+4)


; a write to start the process off
	movf	spi_out,W
	movwf	SSPBUF

; test loop
loop	
	btfss	SSPSTAT,BF
;	btfss	PIR1,SSPIF
	bra		loop
	movf	SSPBUF,W
	movwf	spi_in
	movf	spi_out,W
	movwf	SSPBUF

; fire a byte into the UART for test purposes
	movf	uart1,W
	movwf	TXREG

; timing test
	bsf		PORTC,0

; Do half of the calculations here.
; Manchester encoding state machine

; calculate data area offset and dibit index from state counter
; unless state is 0 in which case skip it as we are sending the
; sync word in place of data
	movf	mencstate,W
	bz		sync

	movwf	dibit
	decf	dibit,f
	movlw	B'11'
	andwf	dibit,f
	
	movf	mencstate,W
	movwf	doffset
	decf	doffset,f

	bcf		STATUS,C
	rrcf	doffset,f
	bcf		STATUS,C
	rrcf	doffset,f

; dereference data
	lfsr	0,subcode_data
	movf	doffset,w
	movff	PLUSW0,temp2

; extract the appropriate 2 bits
	incf	dibit,f

e2loop
	dcfsnz	dibit,f
	bra		e2stop
	rrncf	temp2,f
	rrncf	temp2,f
	bra		e2loop

e2stop		
	
; manchester encode it	
	movf	temp2,w
	andlw	B'11'

	rcall	menc
	bra		mencdone

sync
	movlw	0

; store the manchester encoded result
; 4 bits go into each of the UART outputs
mencdone
	movwf	mencword

; increment the manchester state for next time
	incf	mencstate,F

; reset to 0 once all data is read out
	movf	mencstate,W
	sublw	((4*SUBCODE_SIZE)+1)
	bnz		mdnoclr
	clrf	mencstate
	
mdnoclr

; Load inputs to subcode data area
	rcall	input_update

; timing test
	bcf		PORTC,0


; poll to see when it's time to send the second UART byte
u2poll
	btfss	TXSTA,TRMT
	bra		u2poll

	bsf		PORTC,0
; delay here
; 10 instructions = 1 UART bit
; 60 instructions total, 3 per count
;	movlw	20
;	movwf	txdelay
;	decfsz	txdelay,F
;	goto	$-1

; instead of delay we calculate the interrupter DDS :)
; load the PRF increment and multiply it by 17
; To multiply by 17, shift left 4 bits then add to itself
	movff	PRF_LO,prf_inc_l
	movff	PRF_HI,prf_inc_h
	bcf		STATUS,C
	rlcf	prf_inc_l,f
	rlcf	prf_inc_h,f
	rlcf	prf_inc_l,f
	rlcf	prf_inc_h,f
	rlcf	prf_inc_l,f
	rlcf	prf_inc_h,f
	rlcf	prf_inc_l,f
	rlcf	prf_inc_h,f

	movf	PRF_LO,w
	addwf	prf_inc_l,f
	movf	PRF_HI,w
	addwfc	prf_inc_h,f ; 15 instructions

; Now add the increment to the DDS accumulator 8 times
; We will do this in 2 sections with a break to send out the second UART byte
	movlw	4 ; first section is 4 loops
	movwf	dds_bit
	movlw	NO_BIT
	movwf	begin_bit2 ; 19 instructions of delay to here
bit_loop ; 10 instructions per loop
	movf	prf_inc_l,w	
	addwf	dds_acc_l,f
	movf	prf_inc_h,w	
	addwfc	dds_acc_h,f
	movlw	0
	addwfc	dds_acc_u,f
	btfsc	STATUS,C
	movff	dds_bit,begin_bit2
	decfsz	dds_bit,f
	goto	bit_loop
	

	bcf		PORTC,0

	
; send second UART byte
	movf	uart2,W
	movwf	TXREG

; Do other half of calculations
; timing test
	bsf		PORTC,0

; Finish DDS calculations
	movlw	4 ; another 4 loops
	movwf	dds_bit
	movlw	NO_BIT
	movwf	begin_bit ; 19 instructions of delay to here
bit_loop2 ; 10 instructions per loop
	movf	prf_inc_l,w	
	addwf	dds_acc_l,f
	movf	prf_inc_h,w	
	addwfc	dds_acc_h,f
	movlw	0
	addwfc	dds_acc_u,f
	btfsc	STATUS,C
	movff	dds_bit,begin_bit
	decfsz	dds_bit,f
	goto	bit_loop2

; did the DDS roll over?
; If it did, one of begin_bit and begin_bit2 will not be NO_BIT
; if it didn't, both will be NO_BIT
	movf	begin_bit,W
	sublw	NO_BIT
	bnz		dds_roll

	movf	begin_bit2,W
	sublw	NO_BIT
	bnz		dds_roll2

	bra		dds_state

; Add 4 as this rollover came from the first batch of 4 additions.
dds_roll2
	movf	begin_bit2,W
	addlw	4
	movwf	begin_bit

; If we get here, the DDS rolled over and the bit period of the rollover
; (0-7) is stored in begin_bit
dds_roll
; to trigger a burst, kick the DDS state machine into state burst_begin
	movlw	BURST_BEGIN
	movwf	int_state

; DDS state machine
dds_state
	rcall	dds_sm
	movf	dds_pulse_out,W
	movwf	spi_out

; Modulate the SPI input
	;movf	spi_in,W
    iorwf   spi_in,W    ; combine onboard DDS with external input
	xorlw	B'01010101'
	movwf	spi_mod

; clear the UART registers
	clrf	uart1
	clrf	uart2

; Split into two nibbles and load each into one of the two UART bytes
	movf	spi_mod,W
	andlw	B'11110000'
	movwf	uart1

	swapf	spi_mod,W
	andlw	B'11110000'
	movwf	uart2

; Add the manchester encoded data
; Most significant 2 bits
	movf	mencword,W
	andlw	B'00001100'
	iorwf	uart1,F

; and least significant 2 bits
	rlncf	mencword,F
	rlncf	mencword,W
	andlw	B'00001100'
	iorwf	uart2,F



; Set the parity and field ID bits
; field ID
	bcf		uart1,0
	bsf		uart2,0

; parity
	bcf		uart1,1
	movf	uart1,W
	rcall	parity
	bz		p1odd
	bsf		uart1,1
p1odd

	bcf		uart2,1
	movf	uart2,W
	rcall	parity
	bz		p2odd
	bsf		uart2,1
p2odd
	
	

; return to loop start
;	bcf		PIR1,SSPIF
; timing test
	bcf		PORTC,0
	bra		loop


	

; Manchester encoding table
; Converts 2 unmodulated bits to 4 Manchester encoded ones
; inputs 4-7 return the sync word

org	0200
menc
	movwf	temp1
	movlw	HIGH($)
	movwf	PCLATH
	rlncf	temp1,w
	andlw	B'1111'
	addwf	PCL,f
	retlw	B'0101'
	retlw	B'0110'
	retlw	B'1001'
	retlw	B'1010'
	retlw	B'0000'
	retlw	B'0000'
	retlw	B'0000'
	retlw	B'0000'

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


; Input update subroutine
input_update

; Computed goto on the value of doffset, which shows the
; subcode data location just sent.
	movlw	HIGH($)
	movwf	PCLATH
	rlncf	doffset,w
	andlw	B'1111'
	addwf	PCL,f

; for doffset=0
; Subcode 0 was just sent, 1 going out next time
; Retrieve DC bus voltage result and store to 1+2, set mux to fine tune
	bra		iu_store_12

; for doffset=1
; 2 going out
; Start ADC conversion
	bra		iu_start

; for doffset=2
; 3 going out
; Retrieve fine tune result and store to 3+4, set mux to DC bus
	bra		iu_store_34

; for doffset=3
; 4 going out
; Start ADC conversion
	bra		iu_start

; remaining channels
	bra     iu_store_56
    bra     iu_start
    bra     iu_store_78
    bra     iu_start

iu_start
; set the ADC Go bit
	bsf		ADCON0,1
	return

iu_store_12
; store the result
	movf	ADRESH,W
	movwf	(subcode_data+1)
	movf	ADRESL,W
	movwf	(subcode_data+2)
	
; set the mux to channel 1
	bsf		ADCON0,ACMOD0
    bcf		ADCON0,ACMOD1
	return

iu_store_34
; store the result
	movf	ADRESH,W
	movwf	(subcode_data+3)
	movf	ADRESL,W
	movwf	(subcode_data+4)

; set the mux to channel 2
	bcf		ADCON0,ACMOD0
    bsf		ADCON0,ACMOD1
	return


iu_store_56
; store the result
	movf	ADRESH,W
	movwf	LEN_HI
	movf	ADRESL,W
	movwf	LEN_LO

; set the mux to channel 3
	bsf		ADCON0,ACMOD0
    bsf		ADCON0,ACMOD1
	return

iu_store_78
; store the result
	movf	ADRESH,W
	movwf	PRF_HI
	movf	ADRESL,W
	movwf	PRF_LO

   ; set the mux to channel 0
	bcf		ADCON0,ACMOD0
    bcf		ADCON0,ACMOD1

	return

; DDS state machine
org	0300
dds_sm
	movlw	HIGH($)
	movwf	PCLATH
	rlncf	int_state,w
	andlw	B'1111'
	addwf	PCL,f

	bra		burst_begin
	bra		burst_on
	bra		burst_end
	bra		burst_off
	bra		burst_disable
	return
	return
	return

; burst_begin
; generate the first 8 bits of the burst and transition to burst_on
burst_begin
	bsf		PORTC,1
	bcf		PORTC,1
; first calculate the scaled burst length
; We want a value of 0-60 approx, the input is 0-1023, so we will shift left 4 places
; and use the high word
	movff	LEN_LO,len_inc_l
	movff	LEN_HI,len_inc_h
	bcf		STATUS,C
	rlcf	len_inc_l,f
	rlcf	len_inc_h,f
	rlcf	len_inc_l,f
	rlcf	len_inc_h,f
	rlcf	len_inc_l,f
	rlcf	len_inc_h,f
	rlcf	len_inc_l,f
	rlcf	len_inc_h,w

;	movlw	2
	movwf	len_acc
	incf	len_acc,f
	

	
; generate them
	clrf	dds_pulse_out
;	movlw	8
;	movwf	dds_bit

; Subtract the leading zeros
;	decf	begin_bit,f
;	movf	begin_bit,w
;	subwf	dds_bit,f

	movf	begin_bit,w
;	sublw	8
	movwf	dds_bit
; now generate the ones
bb_loop
	bsf		STATUS,C
	rlcf	dds_pulse_out,f
	dcfsnz	len_acc,f
	goto	bb_tiny
	decfsz	dds_bit,f
	goto	bb_loop



; Less than 0 bits remaining? (whole pulse generated in this state)

; are there fewer than 8 bits remaining?
	movlw	8
	subwf	len_acc,w
	bn		bb_short

; transition to burst_on
	movlw 	BURST_ON
	movwf	int_state	
	return

; short pulse- transit straight to burst_end
bb_short
	movlw 	BURST_END
	movwf	int_state	
	return

; tiny pulse- transit straight to burst_off
bb_tiny
; zero padding required	if dds_bit is >= 1
	movlw 	BURST_OFF
	movwf	int_state
	
	movlw	0
	cpfsgt	dds_bit
	return

	bcf		STATUS,C
	rlcf	dds_pulse_out,f
	decfsz	dds_bit,f
	goto	bb_tiny
	
	return
	

; burst_on
; generate all 1s, decrement the burst length accumulator,
; if less than 8 bits remaining, transition to burst_end
burst_on
	bsf		PORTC,1
	bcf		PORTC,1
	bsf		PORTC,1
	bcf		PORTC,1


	movlw	8
	subwf	len_acc,f
	
	movlw	8
	subwf	len_acc,w
	bnn		bonot

	movlw 	BURST_END
	movwf	int_state
	
bonot
	setf	dds_pulse_out	
;	clrf	dds_pulse_out
	return

; burst_end
; generate the last 8 bits and transition to burst_off
burst_end
	bsf		PORTC,1
	bcf		PORTC,1
	bsf		PORTC,1
	bcf		PORTC,1
	bsf		PORTC,1
	bcf		PORTC,1


	setf	dds_pulse_out

	movlw	HIGH($)
	movwf	PCLATH
	rlncf	len_acc,w
	andlw	B'1111'
	addwf	PCL,f
	bcf		dds_pulse_out,7
	bcf		dds_pulse_out,6
	bcf		dds_pulse_out,5
	bcf		dds_pulse_out,4
	bcf		dds_pulse_out,3
	bcf		dds_pulse_out,2
	bcf		dds_pulse_out,1
	bcf		dds_pulse_out,0

;	clrf	dds_pulse_out

; transition to burst_off
	movlw 	BURST_OFF
	movwf	int_state	
	return

; burst_off
; generate all 0s
burst_off
	clrf	dds_pulse_out
; if the fire button is released, transit to burst_disable
	movlw	BURST_DISABLE
	btfsc	PORTB,0		
	movwf	int_state
	return

; burst_disable
; Reset the DDS and output 0s
; If the fire button stays pressed during the debounce delay,
; transit to burst_begin
burst_disable
	clrf	dds_acc_u
	clrf	dds_acc_h
	clrf	dds_acc_l
	clrf	dds_pulse_out

; if fire button released, reset the debounce counter
	movlw	10
	btfsc	PORTB,0
	movwf	debounce_ctr
	btfsc	PORTB,0	
	return

; if fire button pressed, decrement the debounce counter
	decfsz	debounce_ctr,f
	return
		
; if debounce counter reaches zero, begin a burst immediately
	movlw	4
	movwf	begin_bit
	movlw	BURST_BEGIN
	movwf	int_state
;	bra		burst_begin
	
	return


; Program end
	end
