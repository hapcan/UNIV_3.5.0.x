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
;   Filename:           univ3_IR_Philips_RC5-revX.inc
;   Associated diagram: none
;   Author:             Jacek Siwilo   
;------------------------------------------------------------------------------  
;   Description:        Philips RC5 transmission coding
;    
;   Code format:     
;                       frame shape         header_!commandBit<6>_togglebit_address_command (!!!MSB bit goes first)
;                       carier frequency    36kHz
;                       duty cycle          25%
;                       "0"                 889,-889 us
;                       "1"                 -889,889 us
;                       header              -889,889 us
;                       address             5bits
;                       command             7bits
;                       frame cycle         114ms
;                       frame sent          2 times
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
RC5_INPORT                  SET     IRINPORT    ;input port for infrared receiver
RC5_INPIN                   SET     IRINPIN     ;input pin for infrared receiver
;Output Register Definitions
RC5_OUTPORT                 SET     IROUTPORT   ;output port for infrared LED
RC5_OUTPIN                  SET     IROUTPIN    ;output pin for infrared LED
;Working Register Definitions
RC5_TYPE                    SET     IRCODEREG0  ;code type
RC5_ADDRESS                 SET     IRCODEREG1  ;address register
RC5_COMMAND                 SET     IRCODEREG2  ;command register 
RC5_BIT_COUNTER             SET		IRCODEREG3  ;number of bits in adrress/command
RC5_FLAGS                   SET     IRCODEREG4  ;used to remember some information
RC5_TIME_COUNTER_H          SET     IRCODEREG5  ;impulse/space length counter MSB
RC5_TIME_COUNTER            SET     IRCODEREG6  ;impulse/space length counter LSB
RC5_MSG_COUNTER             SET     IRCODEREG7  ;number of frames to transmit
RC5_COUNTER0                SET     IRCODEREG8  ;counter0
RC5_COUNTER1                SET     IRCODEREG9  ;counter1


;==============================================================================
;==============================================================================
;       R C 5   C O D E   T R A N S M I S S I O N                        
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:			TRANSMIT RC5 INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It transmits twice RC5 code frames.
;                   
;    Input:         1. Device address in RC5_ADDRESS register
;                   2. Command in RC5_COMMAND register
;   Output:         1. Pulses with carrier on RC5_OUTPIN pin of
;                      RC5_OUTPORT port (active high state)
;------------------------------------------------------------------------------
RC5_Transmit
		movlw	.2                          ;message is sent twice
		movwf	RC5_MSG_COUNTER

RC5_TransmitRepeat
    ;header
        call    RC5_Transmit_Header
    ;commandBit<6>
        btg     RC5_COMMAND,6               ;1 in command means '0' to transmit and vice versa
        movlw   .1                          ;command's bit <6>
        movwf   RC5_BIT_COUNTER
        call    RC5_Transmit_Command
        btg     RC5_COMMAND,7               ;get initial value of bit 6 (but rotated left)
    ;toggle bit
        rcall   RC5_Transmit_Toggle
    ;address
		movlw	.5                          ;address has 5 bits
        movwf	RC5_BIT_COUNTER
        call    RC5_Transmit_Address
		rlncf	RC5_ADDRESS                 ;get initial value of address register
		rlncf	RC5_ADDRESS
		rlncf	RC5_ADDRESS
    ;command
        movlw   .6                          ;command's other 6 bits
        movwf   RC5_BIT_COUNTER
        call    RC5_Transmit_Command
        rlncf   RC5_COMMAND                 ;get initial value of command register
    ;break after sent message
        call    RC5_Transmit_Break

		decfsz	RC5_MSG_COUNTER
        bra     RC5_TransmitRepeat
    return

;------------------------------------------------------------------------------
RC5_Transmit_Header       
		call	RC5_Transmit_1               
    return
;--------------
RC5_Transmit_Toggle       
        btfss   RC5_FLAGS,6
        bra     $ + .8
		rcall   RC5_Transmit_0
        bcf     RC5_FLAGS,6
    return
		rcall   RC5_Transmit_1
        bsf     RC5_FLAGS,6              
    return
;--------------
RC5_Transmit_Address                         ;start transmission from MSB - bit4
		btfss	RC5_ADDRESS,4
		call	RC5_Transmit_0
		btfsc	RC5_ADDRESS,4
		call	RC5_Transmit_1
		rlncf	RC5_ADDRESS
		decfsz	RC5_BIT_COUNTER
		bra		RC5_Transmit_Address
     return
;--------------
RC5_Transmit_Command                         ;start transmission from MSB - bit6
        btfss   RC5_COMMAND,6
        call    RC5_Transmit_0
        btfsc   RC5_COMMAND,6
        call    RC5_Transmit_1
        rlncf   RC5_COMMAND
        decfsz  RC5_BIT_COUNTER
        bra     RC5_Transmit_Command
    return
;--------------
RC5_Transmit_Break                          ;between messages 114ms - 25ms = 89ms
        movlw	.100
        movwf	RC5_COUNTER1
        call    RC5_Wait889
        decfsz	RC5_COUNTER1
        bra		$ - 6
    return

;-------------------------------
RC5_Transmit_0
        ;burst                              ;889us
        movlw   .32	
        movwf   RC5_COUNTER0
        call    RC5_Frequency
        decfsz  RC5_COUNTER0
        bra     $ - 6
        ;space                              ;889us
        call    RC5_Wait889
    return
;--------------
RC5_Transmit_1
        ;space                              ;889us
        call    RC5_Wait889
        ;burst                              ;889us
        movlw   .32	
        movwf   RC5_COUNTER0
        call    RC5_Frequency
        decfsz  RC5_COUNTER0
        bra     $ - 6
    return

;-------------------------------
RC5_Frequency                               ;T=27.75us, duty 25%
		bsf		RC5_OUTPORT,RC5_OUTPIN      ;6.875us burst
        movlw	.18
        decfsz	WREG
        bra     $ - 2
		bcf		RC5_OUTPORT,RC5_OUTPIN      ;20.875us break
        movlw	.53
        decfsz	WREG
        bra     $ - 2
    return
;--------------
RC5_Wait889                                 ;wait 868us
        movlw   .89
        movwf   RC5_COUNTER0
        movlw	.25
        decfsz	WREG
        bra     $ - 2
        decfsz	RC5_COUNTER0
        bra     $ - 8
    return


;==============================================================================
;==============================================================================
;       R C 5   C O D E   R E C E I V I N G                        
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:			RECEIVE RC5 INFRARED CODES
;------------------------------------------------------------------------------
; Overview:			It receives RC5 codes. The header of the code must
;                   have been already received. This routine receives only
;                   12 bits for RC5 code or 11 bits for RC5x code
;    Input:         1. Pulses without carrier on RC5_INPIN pin of
;                      RC5_INPORT port (active low state)
;   Output:         1. IRADDRESS and IRCOMMAND registers with received code
;                   2. NEC_TYPE = 6
;                   3. WREG = 0 if reading was ok
;                      WREG = 1 if reading was wrong
;------------------------------------------------------------------------------
RC5_Received
        banksel RC5_BIT_COUNTER
		movlw	.13                         ;13 bits left
		movwf	RC5_BIT_COUNTER
        bsf		RC5_FLAGS,7                 ;set RC5 code flag
		bsf     RC5_FLAGS,0                 ;last was 1
		bra		RC5_ReadBits
RC5x_Received
        banksel RC5_BIT_COUNTER
		movlw	.12                         ;12 bits left
		movwf	RC5_BIT_COUNTER
        bcf		RC5_FLAGS,7                 ;clear RC5 flag (means RC5x code)
		bcf     RC5_FLAGS,0                 ;last was 0
		bra		RC5_ReadBits

;-------------------------------
RC5_ReadBits
		clrf	RC5_ADDRESS					;clear address register
		clrf	RC5_COMMAND					;clear command register
RC5_Frame_Loop
		clrf    RC5_TIME_COUNTER            ;clear ipmluse/space length counter
;--------------
        ;wait for a burst
RC5_Loop_Space
		infsnz	RC5_TIME_COUNTER            ;increment every 40us
		bra		RC5_Received_Error          ;space longer than 255*40us, so exit
        movlw   .104                        ;waste time to read pin every 40us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfsc   RC5_INPORT,RC5_INPIN        ;still space?
        bra     RC5_Loop_Space              ;yes, so stay in loop
;--------------
        ;wait when the burst ends
RC5_Loop_Impulse
		infsnz	RC5_TIME_COUNTER            ;increment every 40us
		bra		RC5_Received_Error          ;impulse longer than 255*40us, so exit
        movlw   .104                        ;waste time to read pin every 40us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfss   RC5_INPORT,RC5_INPIN        ;impulse still on?
        bra     RC5_Loop_Impulse            ;yes, so stay in loop
;--------------
        ;check the length of the impulse
        movlw	.38                         ;38x40=1520us min for 2T (T=889us)
        cpfsgt	RC5_TIME_COUNTER            ;skip if more
        bra		RC5_Received_Error
        movlw	.51                         ;51x40=2040us max for 2T
        cpfsgt	RC5_TIME_COUNTER            ;skip if more
        bra		RC5_2T                      ;time 2T = 1778us
        movlw	.57     					;57x40=2280us min for 3T
        cpfsgt	RC5_TIME_COUNTER            ;skip if more
        bra		RC5_Received_Error
        movlw	.75                         ;75x40=3000us max for 3T
        cpfsgt	RC5_TIME_COUNTER            ;skip if more
        bra		RC5_3T                      ;time 3T = 2667us
        movlw	.76                         ;76x40=3040us min for 4T
        cpfsgt	RC5_TIME_COUNTER            ;skip if more
        bra		RC5_Received_Error
        movlw	.102                        ;102x40=4080us max for 4T
        cpfsgt	RC5_TIME_COUNTER            ;skip if more
        bra		RC5_4T                      ;time 4T = 3556us
        bra		RC5_Received_Error
;--------------
RC5_2T
        btfss	RC5_FLAGS,0                 ;what was last
        bra		RC5_0                       ;last was 0
        bra		RC5_1                       ;last was 1
RC5_3T
        btfss	RC5_FLAGS,0                 ;what was last
        bra		RC5_1                       ;last was 0
        bra		RC5_10                      ;last was 1
RC5_4T
        btfss	RC5_FLAGS,0                 ;what was last
        bra		RC5_10                      ;last was 0
        bra		RC5_Received_Error          ;last was 1
;--------------
RC5_0								
        bcf		RC5_COMMAND,0               ;set 0
        rcall	RC5_update
		bcf     RC5_FLAGS,0                 ;last was 0
        bra		RC5_dec
RC5_1								
        bsf		RC5_COMMAND,0				;set 1
        rcall	RC5_update
		bsf     RC5_FLAGS,0                 ;last was 1
        bra		RC5_dec
RC5_10								
        bsf		RC5_COMMAND,0               ;set 1 and 0
        rcall	RC5_update
        dcfsnz  RC5_BIT_COUNTER             ;decrement impulse counter
        bra		RC5_Received_Error          ;all bits read already
        bcf		RC5_COMMAND,0
        rcall	RC5_update
		bcf     RC5_FLAGS,0                 ;last was 0
        bra		RC5_dec
;--------------
RC5_update
        movlw   .7                          ;all bits of address?
        cpfseq  RC5_BIT_COUNTER             ;skip if euqal
        bra     $ + 4                       ;not yet
        bra     RC5_addr
        rlncf   RC5_COMMAND                 ;rotate left, destination in the same register
        return
RC5_addr	
        movf    RC5_COMMAND,W				;received address move to address register
        movwf   RC5_ADDRESS
        clrf    RC5_COMMAND					
        return

RC5_dec
        dcfsnz  RC5_BIT_COUNTER             ;decrement impulse counter
        bra     RC5_Received_OK
    bra     RC5_Frame_Loop

;-------------------------------
RC5_Received_OK
		bcf		RC5_ADDRESS,7				;prepare address
		bcf		RC5_ADDRESS,6
		bcf		RC5_ADDRESS,5
        rrncf	RC5_COMMAND                 ;step back
		bcf		RC5_COMMAND,7
        btfss	RC5_FLAGS,7                 ;skip if flag is RC5 code
        bsf		RC5_COMMAND,6               ;this is RC5x code
        movlw   0x06
        movwf   RC5_TYPE  
    retlw   0x00

RC5_Received_Error
    retlw   0x01


;------------------------------------------------------------------------------
; Routine:			MEASURE RC5 FRAME REPETITION TIME
;------------------------------------------------------------------------------
; Overview:			It waits for 'frame repetition time' and checks if another
;                   frame begins.
;    Input:         1. Pulses without carrier on RC5_INPIN pin of
;                      RC5_INPORT port (active low state)
;   Output:         1. WREG = 0 if another impuls began
;                      WREG = 1 if another impuls didn't appear
;------------------------------------------------------------------------------
RC5_Frame_Repetition                        ;114ms + 15%
        banksel RC5_TIME_COUNTER
        movlw   .50                         ;50*256*10.25us = 131ms
		movwf   RC5_TIME_COUNTER_H            
		clrf    RC5_TIME_COUNTER
RC5_Frame_Repetition_Loop
        decfsz  RC5_TIME_COUNTER
        bra     $ + 6
		dcfsnz	RC5_TIME_COUNTER_H          ;increment every 40us
    retlw   0x01                            ;space longer than frame repetition time
        movlw   .25                         ;waste time to read pin every 10.25us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfsc   RC5_INPORT,RC5_INPIN        ;still space?
        bra     RC5_Frame_Repetition_Loop   ;yes, so stay in loop
    retlw   0x00                            ;no, impulse begins