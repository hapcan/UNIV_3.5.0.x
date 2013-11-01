	LIST
;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2013 hapcan.com
;
;   This program is free software: you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation, either version 3 of the License, or
;   (at your option) any later version.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program.  If not, see <http://www.gnu.org/licenses/>.
;==============================================================================
;   Filename:           univ3_IR_NEC-revX.inc
;   Associated diagram: none
;   Author:             Jacek Siwilo   
;------------------------------------------------------------------------------ 
;   Description:        NEC transmission coding
;
;   Code format:     
;                       frame shape         header_address_!address_command_!command__bit0_header2 - standard frame (lsb first)
;                                           header_address-L_address-H_command_!command__bit0_header2 - extended frame (lsb first)
;                       carier frequency    38kHz
;                       duty cycle          25%
;                       "0"                 560,-560
;                       "1"                 560,-1680
;                       header              9000,-4480
;                       header2             9000,-2250,560,-560
;                       address             8bits (Standard NEC) or 16bit (Extended NEC)
;                       command             8bits
;                       frame cycle         108ms
;                       frame sent          1 time + (header2)repeated
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     10.2013   Original version
;==============================================================================

;------------------------------------------------------------------------------
; Local Definitions  
;------------------------------------------------------------------------------
;         (all registers must be in the same memory bank or in access ram)
;------------------------------------------------------------------------------
;Input Register Definitions
NEC_INPORT                  SET     IRINPORT    ;input port for infrared receiver
NEC_INPIN                   SET     IRINPIN     ;input pin for infrared receiver
;Output Register Definitions
NEC_OUTPORT                 SET     IROUTPORT   ;output port for infrared LED
NEC_OUTPIN                  SET     IROUTPIN    ;output pin for infrared LED
;Working Register Definitions
NEC_TYPE                    SET     IRCODEREG0  ;code type
NEC_ADDRESS_L               SET     IRCODEREG1  ;address register or LSB of address
NEC_ADDRESS_H               SET     IRCODEREG2  ;negated address register or MSB of address
NEC_COMMAND                 SET     IRCODEREG3  ;command register 
NEC_COMMAND_NEG             SET     IRCODEREG4  ;negated command register 
NEC_BIT_COUNTER             SET		IRCODEREG5  ;number of bits in adrress/command
NEC_TIME_COUNTER_H          SET     IRCODEREG6  ;impulse/space length counter MSB
NEC_TIME_COUNTER            SET     IRCODEREG7  ;impulse/space length counter LSB
NEC_ZEROs_COUNTER           SET     IRCODEREG8  ;counts zero bits to define frame length
NEC_COUNTER0                SET     IRCODEREG9  ;counter0
NEC_COUNTER1                SET     IRCODEREG10 ;counter1


;==============================================================================
;==============================================================================
;       N E C   C O D E   T R A N S M I S S I O N                        
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:			TRANSMIT STANDARD NEC INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It transmits NEC standard code frames.
;                   
;    Input:         1. Device address in NEC_ADDRESS register
;                   2. Command in NEC_COMMAND register
;   Output:         1. Pulses with carrier on NEC_OUTPIN pin of
;                      NEC_OUTPORT port (active high state)
;------------------------------------------------------------------------------
NEC_Std_Transmit

    ;header
        call    NEC_Transmit_Header
    ;address
        movlw   .8                          ;address has 8 bits
        movwf   NEC_BIT_COUNTER
        call    NEC_Transmit_Address
    ;address complement
        comf    NEC_ADDRESS_L               ;negate addres
        movlw   .8                          ;address has 8 bits
        movwf   NEC_BIT_COUNTER
        call    NEC_Transmit_Address
    ;command
        movlw   .8                          ;command has 8 bits
        movwf   NEC_BIT_COUNTER
        call    NEC_Transmit_Command
    ;command complement
        comf    NEC_COMMAND                 ;negate command
        movlw   .8                          ;command has 8 bits
        movwf   NEC_BIT_COUNTER
        call    NEC_Transmit_Command
    ;stop bit
        call    NEC_Transmit_0
    ;break after sent message
        call    NEC_Transmit_Std_Break      ;standard code break
    ;header2
        call    NEC_Transmit_Header2

    return

;------------------------------------------------------------------------------
; Routine:			TRANSMIT EXTENDED NEC INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It transmits NEC extended code frames.
;                   
;    Input:         1. Device address in NEC_ADDRESS H & NEC_ADDRESS L registers
;                   2. Command in NEC_COMMAND register
;   Output:         1. Pulses with carrier on NEC_OUTPIN pin of
;                      NEC_OUTPORT port (active high state)
;------------------------------------------------------------------------------
NEC_Ext_Transmit
        clrf    NEC_ZEROs_COUNTER           ;clear "0"s counter
    ;header
        call    NEC_Transmit_Header
    ;address_L
        movlw   .8                          ;address_L has 8 bits
        movwf   NEC_BIT_COUNTER
        call    NEC_Transmit_Address
    ;address_H
        movff   NEC_ADDRESS_H,NEC_ADDRESS_L ;move addres to the right reg
        movlw   .8                          ;address_H has 8 bits
        movwf   NEC_BIT_COUNTER
        call    NEC_Transmit_Address
    ;command
        movlw   .8                          ;command has 8 bits
        movwf   NEC_BIT_COUNTER
        call    NEC_Transmit_Command
    ;command complement
        comf    NEC_COMMAND                 ;negate command
        movlw   .8                          ;command has 8 bits
        movwf   NEC_BIT_COUNTER
        call    NEC_Transmit_Command
    ;stop bit
        call    NEC_Transmit_0
    ;break after sent message
        call    NEC_Transmit_Ext_Break      ;extended code break
    ;header2
        call    NEC_Transmit_Header2

    return

;------------------------------------------------------------------------------
; Routine:		NEC TX CODE SUBROUTINES
;------------------------------------------------------------------------------
NEC_Transmit_Header       
        ;burst
        movlw   .171                        ;9000us
        movwf   NEC_COUNTER1
        call    NEC_Frequency
        decfsz  NEC_COUNTER1
        bra     $ - 6
        movlw   .171
        movwf   NEC_COUNTER1
        call    NEC_Frequency
        decfsz  NEC_COUNTER1
        bra     $ - 6
        ;space
        movlw   .8                          ;4500us
        movwf   NEC_COUNTER1
        call    NEC_Wait560
        decfsz  NEC_COUNTER1
        bra $ - 6
    return
;--------------
NEC_Transmit_Header2
        ;burst
        movlw   .171                        ;9000us
        movwf   NEC_COUNTER1
        call    NEC_Frequency
        decfsz  NEC_COUNTER1
        bra     $ - 6
        movlw   .171
        movwf   NEC_COUNTER1
        call    NEC_Frequency
        decfsz  NEC_COUNTER1
        bra     $ - 6
        ;space
        movlw   .4                          ;2250us
        movwf   NEC_COUNTER1
        call    NEC_Wait560
        decfsz  NEC_COUNTER1
        bra $ - 6
       ;burst
        call    NEC_Transmit_0              ;560us,-560us
    return
;--------------
NEC_Transmit_Address
        btfss   NEC_ADDRESS_L,0
        call    NEC_Transmit_0
        btfsc   NEC_ADDRESS_L,0
        call    NEC_Transmit_1
        rrncf   NEC_ADDRESS_L
        decfsz  NEC_BIT_COUNTER
        bra     NEC_Transmit_Address
    return
;--------------
NEC_Transmit_Command
        btfss   NEC_COMMAND,0
        call    NEC_Transmit_0
        btfsc   NEC_COMMAND,0
        call    NEC_Transmit_1
        rrncf   NEC_COMMAND
        decfsz  NEC_BIT_COUNTER
        bra     NEC_Transmit_Command
    return
;--------------
NEC_Transmit_Std_Break                      ;between messages 40000us
        movlw   .72
        movwf   NEC_COUNTER1
        call    NEC_Wait560            
        decfsz  NEC_COUNTER1
        bra     $ - 6
    return
;--------------
NEC_Transmit_Ext_Break                      ;break after frame: 21ms + number of zeros x 1120us
        movlw   .38
        movwf   NEC_COUNTER1
        call    NEC_Wait560            
        decfsz  NEC_COUNTER1
        bra     $ - 6
        ;ZERO_COUNTER x 0,560ms
        tstfsz  NEC_ZEROs_COUNTER
        bra     $ + 4
    return                                  ;there were not any "0"s
        rcall   NEC_Wait560
        rcall   NEC_Wait560
        decfsz  NEC_ZEROs_COUNTER
        bra		$ - 6
    return

;-------------------------------
NEC_Transmit_0
		incf    NEC_ZEROs_COUNTER           ;"0"s counter
        ;burst                              ;560us
        movlw   .21
        movwf   NEC_COUNTER1
        call    NEC_Frequency
        decfsz  NEC_COUNTER1
        bra     $ - 6
        ;space                              ;560us
        call    NEC_Wait560
    return
;--------------
NEC_Transmit_1
        ;burst                              ;560us
        movlw   .21
        movwf   NEC_COUNTER1
        call    NEC_Frequency
        decfsz  NEC_COUNTER1
        bra     $ - 6
        ;space                              ;1680us
        call    NEC_Wait560
        call    NEC_Wait560
        call    NEC_Wait560
    return
;--------------
NEC_Frequency                               ;T=26.250us, duty 25%
		bsf		NEC_OUTPORT,NEC_OUTPIN      ;6.500us burst
        movlw	.17
        decfsz	WREG
        bra     $ - 2
		bcf		NEC_OUTPORT,NEC_OUTPIN      ;19.750us break
        movlw	.50
        decfsz	WREG
        bra     $ - 2
    return
;--------------
NEC_Wait560                                 ;556us
        movlw   .57
        movwf   NEC_COUNTER0
        movlw	.25
        decfsz	WREG
        bra     $ - 2
        decfsz	NEC_COUNTER0
        bra     $ - 8
    return

;==============================================================================
;==============================================================================
;       N E C   C O D E   R E C E I V I N G                        
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:			RECEIVE NEC INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It receives NEC standard and extended codes. The header of
;                   the code must have been already received. This routine
;                   receives only bits without frame header.
;    Input:         1. Pulses without carrier on NEC_INPIN pin of  NEC_INPORT
;                      port (active low state)
;   Output:         1. NEC_ADDRESS and NEC_COMMAND registers with received code
;                   2. NEC_TYPE = 7 for standard code (address is 8 bit wide) - NEC_ADDRESS
;                      NEC_TYPE = 8 for extended code (address is 16 bit wide - NEC_ADDRESS_NEG(MSB)+NEC_ADDRESS(LSB))
;                   3. WREG = 0 if reading was ok
;                      WREG = 1 if reading was wrong
;------------------------------------------------------------------------------
NEC_Received
        banksel NEC_BIT_COUNTER
		movlw	.32                         ;32 bits left
		movwf	NEC_BIT_COUNTER

;-------------------------------
NEC_ReadBits
		clrf	NEC_ADDRESS_H               ;clear address register
		clrf	NEC_ADDRESS_L               ;clear address register
		clrf	NEC_COMMAND                 ;clear command register
		clrf    NEC_TIME_COUNTER            ;clear ipmluse/space length counter
;--------------
        ;measure space after header
        call    NEC_Measure_Space
        ;check the length of the space
        movlw	.95                         ;95x40=3800us min for "0" (4500us)
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_Received_Error
        movlw	.131                        ;130x40=5200us max for "0"
        cpfslt	NEC_TIME_COUNTER            ;skip if less
        bra		NEC_Received_Error

;--------------
NEC_Frame_Loop
        ;measure burst
        call    NEC_Measure_Impuls
        ;check the length of the impulse
        movlw	.11                         ;11x40=440us min for "0" (560us)
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_Received_Error
        movlw	.18                         ;17x40=680us max for "0"
        cpfslt	NEC_TIME_COUNTER            ;skip if less
        bra		NEC_Received_Error
;--------------
        ;measure space
        call    NEC_Measure_Space
        ;check the length of the space
        movlw	.10                         ;10x40=400us min for "0" (560us)
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_Received_Error
        movlw	.17                         ;17x40=680us max for "0"
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_0                       ;received bit "0"
        movlw	.36     					;36x40=1440us min for "1"
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_Received_Error
        movlw	.48                         ;48x40=1920us max for "1"
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_1                       ;received bit "1"
        bra		NEC_Received_Error

;--------------
NEC_0				
        rrcf	NEC_COMMAND_NEG             ;rotate right			
        bcf     NEC_COMMAND_NEG,7           ;set 0
        bra     NEC_decrement
NEC_1				
        rrcf	NEC_COMMAND_NEG             ;rotate right		
        bsf     NEC_COMMAND_NEG,7           ;set 1
;--------------
NEC_decrement
        dcfsnz	NEC_BIT_COUNTER             ;decrement impulse counter
        bra		NEC_Received_OK
        movlw	.24                         ;all bits of address?
        cpfseq	NEC_BIT_COUNTER             ;skip if euqal
        bra     $ + 8
        movff   NEC_COMMAND_NEG,NEC_ADDRESS_L ;save address_L
        clrf    NEC_COMMAND_NEG
        movlw	.16                         ;all bits of negated address?
        cpfseq	NEC_BIT_COUNTER             ;skip if euqal
        bra     $ + 8
        movff   NEC_COMMAND_NEG,NEC_ADDRESS_H ;save negated address_H
        clrf    NEC_COMMAND_NEG
        movlw	.8                          ;all bits of command?
        cpfseq	NEC_BIT_COUNTER             ;skip if euqal
        bra     $ + 8
        movff   NEC_COMMAND_NEG,NEC_COMMAND ;save command
        clrf    NEC_COMMAND_NEG

    bra     NEC_Frame_Loop                  ;not yet 

;-------------------------------
NEC_Received_OK
        ;wait for stop bit ending
NEC_Loop_StopImpulse
		infsnz	NEC_TIME_COUNTER            ;increment every 40us
		bra		NEC_Received_Error          ;impulse longer than 255*40us, so exit
        movlw   .104                        ;waste time to read pin every 40us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfss   NEC_INPORT,NEC_INPIN        ;impulse still on?
        bra     NEC_Loop_StopImpulse        ;yes, so stay in loop
        ;check received bytes
        comf    NEC_COMMAND_NEG,W
        xorwf   NEC_COMMAND,W               ;negated COMMAND equals COMMAND_NEG?
        bnz     NEC_Received_Error          ;no, so exit
        comf    NEC_ADDRESS_H,W
        xorwf   NEC_ADDRESS_L,W             ;negated ADDRESS_H equals ADDRESS_L?
        bnz     NEC_Received_Extended       ;no, so this is extended code
NEC_Received_Standard                       ;code included in NEC_ADDRESS & NEC_COMMAND regs 
        movlw   0x07
        movwf   NEC_TYPE  
    retlw   0x00  
NEC_Received_Extended                       ;code included in NEC_ADDRESS_NEG - MSB of address
        movlw   0x08                        ;               & NEC_ADDRESS     - LSB of address
        movwf   NEC_TYPE                    ;               & NEC_COMMAND     - command reg    
    retlw   0x00
;-------------------------------
NEC_Received_Error
    retlw   0x01


;------------------------------------------------------------------------------
; Routine:			MEASURE NEC REPETITION FRAME
;------------------------------------------------------------------------------
; Overview:			It waits for 'frame repetition' and checks if it has
;                   format 9000,-2250,560
;    Input:         1. Pulses without carrier on NEC_INPIN pin of
;                      NEC_INPORT port (active low state)
;   Output:         1. WREG = 0 if re impuls began
;                      WREG = 1 if another impuls didn't appear
;------------------------------------------------------------------------------
NEC_Frame_Repetition                        ;108ms + 15%
        banksel NEC_TIME_COUNTER

    ;measure break 108ms
        movlw   .47                         ;47*256*10.25us = 123ms
		movwf   NEC_TIME_COUNTER_H            
		clrf    NEC_TIME_COUNTER
NEC_Frame_Repetition_BreakLoop
        decfsz  NEC_TIME_COUNTER            ;decrement every 10us
        bra     $ + 6
		dcfsnz	NEC_TIME_COUNTER_H
        bra     NEC_Frame_RepetitionError   ;space longer than frame repetition time
        movlw   .25                         ;waste time to read pin every 10.25us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfsc   NEC_INPORT,SIRC_INPIN       ;still space?
        bra     NEC_Frame_Repetition_BreakLoop  ;yes, so stay in loop

    ;measure burst 9000us
        rcall   NEC_Measure_Impuls
        ;check the length of the impulse
        movlw	.192                        ;192x40=7680us min for NEC (9000-15%)
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_Frame_RepetitionError
        movlw	.255                        ;255x40=10200us max for NEC (9000+15%)
        cpfslt	NEC_TIME_COUNTER            ;skip if less
        bra		NEC_Frame_RepetitionError
    ;measure space 2250us
        rcall   NEC_Measure_Space
        ;check the length of the space
        movlw	.48                         ;48x40=1920us min for NEC (2250-15%)
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_Frame_RepetitionError
        movlw	.64                         ;64x40=2560us max for NEC (2250+15%)
        cpfslt	NEC_TIME_COUNTER            ;skip if less
        bra		NEC_Frame_RepetitionError
    ;measure burst 560us
        rcall   NEC_Measure_Impuls
        ;check the length of the impulse
        movlw	.11                         ;11x40=440us min for "0" (560us)
        cpfsgt	NEC_TIME_COUNTER            ;skip if more
        bra		NEC_Received_Error
        movlw	.18                         ;17x40=680us max for "0"
        cpfslt	NEC_TIME_COUNTER            ;skip if less
        bra		NEC_Frame_RepetitionError
    retlw   0x00

NEC_Frame_RepetitionError
    retlw   0x01

;------------------------------------------------------------------------------
; Routine:		NEC RX CODE SUBROUTINES
;------------------------------------------------------------------------------
;measure burst
NEC_Measure_Impuls
		clrf    NEC_TIME_COUNTER            ;clear ipmluse/space length counter
NEC_Measure_Impuls_Loop
		infsnz	NEC_TIME_COUNTER            ;increment every 40us
    return                                  ;impulse longer than 255*40us, so exit
        movlw   .104                        ;waste time to read pin every 40us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfss   NEC_INPORT,NEC_INPIN        ;impulse still on?
        bra     NEC_Measure_Impuls_Loop     ;yes, so stay in loop
    return
;-------------------------------
;measure space
NEC_Measure_Space
		clrf    NEC_TIME_COUNTER            ;clear ipmluse/space length counter
NEC_Measure_Space_Loop
		infsnz	NEC_TIME_COUNTER            ;increment every 40us
    return                                  ;space longer than 255*40us, so exit
        movlw   .104                        ;waste time to read pin every 40us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfsc   NEC_INPORT,NEC_INPIN        ;space still on?
        bra     NEC_Measure_Space_Loop      ;yes, so stay in loop
    return