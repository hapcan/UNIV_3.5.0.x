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
;   Filename:           univ3_IR_Sony_SIRC-revX.inc
;   Associated diagram: none
;   Author:             Jacek Siwilo   
;------------------------------------------------------------------------------   
;   Description:        Sony SIRC 12bit and 15bit transmission coding
;    
;   Code format:     
;                       frame shape         header_command_address (lsb first)
;                       carier frequency    40.0kHz
;                       duty cycle          20%
;                       "0"                 600,-600 us
;                       "1"                 1200,-600 us
;                       header              2400,-600 us
;                       address             5bits (SIRC 12bit), 8bits (SIRC 15bit), 13bits (SIRC 20bit)
;                       command             7bits
;                       frame cycle         45ms
;                       frame sent          3 times
;==============================================================================                   
;   Revision History
;   Rev:  Date:     Details:
;   0     09.2013   Original version
;==============================================================================

;------------------------------------------------------------------------------
; Local Definitions
;------------------------------------------------------------------------------
;Input Register Definitions
SIRC_INPORT                 SET     IRINPORT    ;input port for infrared receiver
SIRC_INPIN                  SET     IRINPIN     ;input pin for infrared receiver
;Output Register Definitions
SIRC_OUTPORT                SET     IROUTPORT   ;output port for infrared LED
SIRC_OUTPIN                 SET     IROUTPIN    ;output pin for infrared LED
;Working Register Definitions
SIRC_TYPE                   SET     IRCODEREG0  ;code type
SIRC_ADDRESS_H              SET     IRCODEREG1  ;address register MSB
SIRC_ADDRESS_L              SET     IRCODEREG2  ;address register LSB
SIRC_COMMAND                SET     IRCODEREG3  ;command register   
SIRC_BIT_COUNTER            SET     IRCODEREG4  ;number of bits in adrress/command
SIRC_TIME_COUNTER_H         SET     IRCODEREG5  ;impulse/space length counter MSB
SIRC_TIME_COUNTER           SET     IRCODEREG6  ;impulse/space length counter LSB
SIRC_MSG_COUNTER            SET     IRCODEREG7  ;number of frames to transmit
SIRC_ZEROs_COUNTER          SET     IRCODEREG8  ;counts zero bits to define frame length
SIRC_COUNTER0               SET     IRCODEREG9  ;counter0
SIRC_COUNTER1               SET     IRCODEREG10 ;counter1


;==============================================================================
;==============================================================================
;       S O N Y   C O D E   T R A N S M I S S I O N                        
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:			TRANSMIT SIRC 12bit INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It transmits three times SIRC 12 bit code frames.
;                   
;    Input:         1. Device address in SIRC_ADDRESS_L register
;                   2. Command in SIRC_COMMAND register
;   Output:         1. Pulses with carrier on SIRC_OUTPIN pin of
;                      SIRC_OUTPORT port (active high state)
;------------------------------------------------------------------------------
SIRC_12bit_Transmit
        movlw   .3                          ;message is repeated three times
        movwf   SIRC_MSG_COUNTER

SIRC_12bit_TransmitRepeat
        clrf    SIRC_ZEROs_COUNTER          ;clear "0"s counter
    ;header
        call    SIRC_Transmit_Header
    ;command
        movlw   .7                          ;command bits
        movwf   SIRC_BIT_COUNTER
        call    SIRC_Transmit_Command
        rrncf   SIRC_COMMAND                ;get initial value of command register
    ;address
        movlw   .5                          ;address bits
        movwf   SIRC_BIT_COUNTER
        call    SIRC_Transmit_Address_L
        rrncf   SIRC_ADDRESS_L              ;get initial value of address register
        rrncf   SIRC_ADDRESS_L
        rrncf   SIRC_ADDRESS_L
    ;break after sent message
        movlw   .35                         ;35x600us + 600us x number of zeros
        call    SIRC_Transmit_Break

        decfsz  SIRC_MSG_COUNTER
        bra     SIRC_12bit_TransmitRepeat
    return

;------------------------------------------------------------------------------
; Routine:			TRANSMIT SIRC 15bit INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It transmits three times SIRC 15 bit code frames.
;                   
;    Input:         1. Device address in SIRC_ADDRESS_L register
;                   2. Command in SIRC_COMMAND register
;   Output:         1. Pulses with carrier on SIRC_OUTPIN pin of
;                      SIRC_OUTPORT port (active high state)
;------------------------------------------------------------------------------
SIRC_15bit_Transmit
        movlw   .3                          ;message is repeated three times
        movwf   SIRC_MSG_COUNTER

SIRC_15bit_TransmitRepeat
        clrf    SIRC_ZEROs_COUNTER          ;clear "0"s counter
    ;header
        call    SIRC_Transmit_Header
    ;command
        movlw   .7                          ;command bits
        movwf   SIRC_BIT_COUNTER
        call    SIRC_Transmit_Command
        rrncf   SIRC_COMMAND                ;get initial value of command register
    ;address
        movlw   .8                          ;address bits
        movwf   SIRC_BIT_COUNTER
        call    SIRC_Transmit_Address_L
    ;break after sent message
        movlw   .26                         ;26x600us + 600us x number of zeros
        call    SIRC_Transmit_Break

        decfsz  SIRC_MSG_COUNTER
        bra     SIRC_15bit_TransmitRepeat
    return

;------------------------------------------------------------------------------
; Routine:			TRANSMIT SIRC 20bit INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It transmits three times SIRC 20 bit code frames.
;                   
;    Input:         1. Device address in SIRC_ADDRESS_H & SIRC_ADDRESS_L registers
;                   2. Command in SIRC_COMMAND register
;   Output:         1. Pulses with carrier on SIRC_OUTPIN pin of
;                      SIRC_OUTPORT port (active high state)
;------------------------------------------------------------------------------
SIRC_20bit_Transmit
        movlw   .3                          ;message is repeated three times
        movwf   SIRC_MSG_COUNTER

SIRC_20bit_TransmitRepeat
        clrf    SIRC_ZEROs_COUNTER          ;clear "0"s counter
    ;header
        call    SIRC_Transmit_Header
    ;command
        movlw   .7                          ;command bits
        movwf   SIRC_BIT_COUNTER
        call    SIRC_Transmit_Command
        rrncf   SIRC_COMMAND                ;get initial value of command register
    ;address
        movlw   .8                          ;address_L bits
        movwf   SIRC_BIT_COUNTER
        call    SIRC_Transmit_Address_L
        movlw   .5                          ;address_H bits
        movwf   SIRC_BIT_COUNTER
        call    SIRC_Transmit_Address_H
        rrncf   SIRC_ADDRESS_H              ;get initial value of address register
        rrncf   SIRC_ADDRESS_H
        rrncf   SIRC_ADDRESS_H
    ;break after sent message
        movlw   .11                         ;11x600us + 600us x number of zeros
        call    SIRC_Transmit_Break

        decfsz  SIRC_MSG_COUNTER
        bra     SIRC_20bit_TransmitRepeat
    return

;------------------------------------------------------------------------------
; Routine:		SIRC CODE SUBROUTINES
;------------------------------------------------------------------------------
SIRC_Transmit_Header                        ;2,412ms
		movlw   .96
		movwf   SIRC_COUNTER1
		call    SIRC_Frequency
		decfsz  SIRC_COUNTER1
		bra     $ - 6
    return
;--------------
SIRC_Transmit_Command
		call	SIRC_Space
        btfss   SIRC_COMMAND,0
        call	SIRC_Transmit_0
        btfsc   SIRC_COMMAND,0
        call	SIRC_Transmit_1
        rrncf   SIRC_COMMAND
        decfsz  SIRC_BIT_COUNTER
        bra     SIRC_Transmit_Command
    return
;--------------
SIRC_Transmit_Address_L
		call	SIRC_Space
        btfss   SIRC_ADDRESS_L,0
        call	SIRC_Transmit_0
        btfsc   SIRC_ADDRESS_L,0
        call	SIRC_Transmit_1
        rrncf   SIRC_ADDRESS_L
        decfsz  SIRC_BIT_COUNTER
        bra     SIRC_Transmit_Address_L
    return
;--------------
SIRC_Transmit_Address_H
		call	SIRC_Space
        btfss   SIRC_ADDRESS_H,0
        call	SIRC_Transmit_0
        btfsc   SIRC_ADDRESS_H,0
        call	SIRC_Transmit_1
        rrncf   SIRC_ADDRESS_H
        decfsz  SIRC_BIT_COUNTER
        bra     SIRC_Transmit_Address_H
    return
;--------------	
SIRC_Transmit_Break					        ;break after frame: WREG x 600us + number of zeros x 600us
        movwf   SIRC_COUNTER1	
        call    SIRC_Space                  ;wait 600us
        decfsz  SIRC_COUNTER1
        bra     $ - 6
        ;ZERO_COUNTER x 0,600ms
        tstfsz  SIRC_ZEROs_COUNTER
        bra     $ + 4
    return                                  ;there were not any "0"s
        call    SIRC_Space                  ;wait 600us
        decfsz  SIRC_ZEROs_COUNTER
        bra		$ - 6
    return

;-------------------------------
SIRC_Space                                  ;586us
        movlw   .60
        movwf   SIRC_COUNTER0
        movlw	.25
        decfsz	WREG
        bra     $ - 2
        decfsz	SIRC_COUNTER0
        bra     $ - 8
    return
;--------------
SIRC_Transmit_0                             ;604us
		incf    SIRC_ZEROs_COUNTER          ;"0"s counter
        movlw   .24                         ;24x25us
        movwf   SIRC_COUNTER0
        call    SIRC_Frequency
        decfsz  SIRC_COUNTER0
        bra     $ - 6
    return
;--------------
SIRC_Transmit_1                             ;1206us
        movlw	.48                         ;48x25us
        movwf   SIRC_COUNTER0
        call    SIRC_Frequency
        decfsz  SIRC_COUNTER0
        bra $ - 6
    return
;--------------
SIRC_Frequency                              ;T=25.125us, duty 25%
		bsf		SIRC_OUTPORT,SIRC_OUTPIN    ;6.125us burst
        movlw	.16
        decfsz	WREG
        bra     $ - 2
		bcf		SIRC_OUTPORT,SIRC_OUTPIN    ;19.000us break
        movlw	.48
        decfsz	WREG
        bra     $ - 2
    return


;==============================================================================
;==============================================================================
;       S O N Y   C O D E   R E C E I V I N G                        
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:			RECEIVE SIRC 12, 15 & 20 bit INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It receives SIRC 12, 15 & 20bit codes. The header of the code
;                   must have been already received. This routine receives only
;                   12, 15 or 20 bits without frame header.
;    Input:         1. Pulses without carrier on SIRC_INPIN pin of SIRC_INPORT
;                      port (active low state)
;   Output:         1. SIRC_ADDRESS and SIRC_COMMAND registers with received code
;                   2. SIRC_TYPE = 3 for SIRC 12bit
;                      SIRC_TYPE = 4 for SIRC 15bit
;                      SIRC_TYPE = 5 for SIRC 20bit 
;                   3. WREG = 0 if reading was ok
;                      WREG = 1 if reading was wrong
;------------------------------------------------------------------------------
SIRC_Received
        banksel SIRC_BIT_COUNTER
		movlw	.20                         ;20 bits left
		movwf	SIRC_BIT_COUNTER

;-------------------------------
SIRC_ReadBits
		clrf	SIRC_ADDRESS_H              ;clear address register
		clrf	SIRC_ADDRESS_L              ;clear address register
		clrf	SIRC_COMMAND                ;clear command register
SIRC_Frame_Loop
		clrf    SIRC_TIME_COUNTER           ;clear ipmluse/space length counter
;--------------
        ;wait for a burst
SIRC_Loop_Space
        movlw   .20
        cpfslt  SIRC_TIME_COUNTER
		bra		SIRC_12or15b?               ;space longer than 20*40us, so check for 12 or 15 bit codes
		incf	SIRC_TIME_COUNTER           ;increment every 40us
        movlw   .104                        ;waste time to read pin every 40us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfsc   SIRC_INPORT,SIRC_INPIN      ;still space?
        bra     SIRC_Loop_Space             ;yes, so stay in loop
;--------------
        ;wait when the burst ends
		clrf    SIRC_TIME_COUNTER           ;clear ipmluse/space length counter
SIRC_Loop_Impulse
		infsnz	SIRC_TIME_COUNTER           ;increment every 40us
		bra		SIRC_Received_Error         ;impulse longer than 255*40us, so exit
        movlw   .104                        ;waste time to read pin every 40us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfss   SIRC_INPORT,SIRC_INPIN      ;impulse still on?
        bra     SIRC_Loop_Impulse           ;yes, so stay in loop
;--------------
        ;check the length of the impulse
        movlw	.12                         ;12x40=480us min for "0" (600us)
        cpfsgt	SIRC_TIME_COUNTER            ;skip if more
        bra		SIRC_Received_Error
        movlw	.17                         ;17x40=680us max for "0"
        cpfsgt	SIRC_TIME_COUNTER           ;skip if more
        bra		SIRC_0                      ;received bit "0"
        movlw	.26     					;26x40=1040us min for "1"
        cpfsgt	SIRC_TIME_COUNTER           ;skip if more
        bra		SIRC_Received_Error
        movlw	.34                         ;34x40=1360us max for "1"
        cpfsgt	SIRC_TIME_COUNTER           ;skip if more
        bra		SIRC_1                      ;received bit "1"
        bra		SIRC_Received_Error
;--------------
SIRC_0				
        rrcf	SIRC_ADDRESS_H              ;rotate right
        rrcf    SIRC_ADDRESS_L				
        bcf     SIRC_ADDRESS_H,7            ;set 0
        bra     SIRC_decrement
SIRC_1				
        rrcf	SIRC_ADDRESS_H              ;rotate right
        rrcf    SIRC_ADDRESS_L				
        bsf     SIRC_ADDRESS_H,7            ;set 1
;--------------
SIRC_decrement
        dcfsnz	SIRC_BIT_COUNTER            ;decrement impulse counter
        bra		SIRC_Received_20bit
        movlw	.13                         ;all bits of command?
        cpfseq	SIRC_BIT_COUNTER            ;skip if euqal
    bra     SIRC_Frame_Loop                 ;not yet 
SIRC_command
        movf	SIRC_ADDRESS_H,W			;received command move to command register
        movwf	SIRC_COMMAND
        bcf     STATUS,C                    ;adjust 7 bits of command to the right
        rrcf    SIRC_COMMAND
        clrf	SIRC_ADDRESS_H					
    bra     SIRC_Frame_Loop

;-------------------------------
SIRC_12or15b?
        movlw   .8                          ;8 bits left of 20 (12bit SIRC)?
        cpfseq  SIRC_BIT_COUNTER            ;skip if euqal
        bra     $ + 4                       ;it's not 12bit
        bra     SIRC_Received_12bit
        movlw   .5                          ;5 bits left of 20 (15bit SIRC)?
        cpfseq  SIRC_BIT_COUNTER            ;skip if euqal
        bra     SIRC_Received_Error         ;it's not 15bit neither
        bra     SIRC_Received_15bit
;--------------
SIRC_Received_12bit
        movlw   .11                         ;adjust 11 bits of address to the right (11 positions)
        movwf   SIRC_BIT_COUNTER
        bcf     STATUS,C
        rrcf	SIRC_ADDRESS_H              ;rotate right
        rrcf    SIRC_ADDRESS_L
        decfsz	SIRC_BIT_COUNTER
        bra     $ - 8
        movlw   0x03
        movwf   SIRC_TYPE  
    retlw   0x00
;--------------
SIRC_Received_15bit
        movlw   .8                          ;adjust 8 bits of address to the right (8 positions)
        movwf   SIRC_BIT_COUNTER
        bcf     STATUS,C
        rrcf	SIRC_ADDRESS_H              ;rotate right
        rrcf    SIRC_ADDRESS_L
        decfsz	SIRC_BIT_COUNTER
        bra     $ - 8
        movlw   0x04
        movwf   SIRC_TYPE  
    retlw   0x00
;--------------
SIRC_Received_20bit
        movlw   .3                          ;adjust 3 bits of address to the right (3 positions)
        movwf   SIRC_BIT_COUNTER
        bcf     STATUS,C
        rrcf	SIRC_ADDRESS_H              ;rotate right
        rrcf    SIRC_ADDRESS_L
        decfsz	SIRC_BIT_COUNTER
        bra     $ - 8
        movlw   0x05
        movwf   SIRC_TYPE           
    retlw   0x00
;-------------------------------
SIRC_Received_Error
    retlw   0x01


;------------------------------------------------------------------------------
; Routine:			MEASURE SIRC FRAME REPETITION TIME
;------------------------------------------------------------------------------
; Overview:			It waits for 'frame repetition time' and checks if another
;                   frame begins.
;    Input:         1. Pulses without carrier on SIRC_INPIN pin of
;                      SIRC_INPORT port (active low state)
;   Output:         1. WREG = 0 if another impuls began
;                      WREG = 1 if another impuls didn't appear
;------------------------------------------------------------------------------
SIRC_Frame_Repetition                       ;45ms + 15%
        banksel SIRC_TIME_COUNTER
        movlw   .20                         ;20*256*10.25us = 52ms
		movwf   SIRC_TIME_COUNTER_H            
		clrf    SIRC_TIME_COUNTER
SIRC_Frame_Repetition_Loop
        decfsz  SIRC_TIME_COUNTER           ;decrement every 10us
        bra     $ + 6
		dcfsnz	SIRC_TIME_COUNTER_H
    retlw   0x01                            ;space longer than frame repetition time
        movlw   .25                         ;waste time to read pin every 10.25us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfsc   SIRC_INPORT,SIRC_INPIN      ;still space?
        bra     SIRC_Frame_Repetition_Loop  ;yes, so stay in loop
    retlw   0x00