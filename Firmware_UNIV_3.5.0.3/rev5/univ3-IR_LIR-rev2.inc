    LIST
;==============================================================================
;   MIT License
;
;   Copyright (c) 2022 HAPCAN Home Automation Project (http://hapcan.com)
;
;   Permission is hereby granted, free of charge, to any person obtaining a copy
;   of this software and associated documentation files (the "Software"), to deal
;   in the Software without restriction, including without limitation the rights
;   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;   copies of the Software, and to permit persons to whom the Software is
;   furnished to do so, subject to the following conditions:
;
;   The above copyright notice and this permission notice shall be included in all
;   copies or substantial portions of the Software.
;
;   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;   SOFTWARE.
;==============================================================================
;   Filename:           univ3_IR_LIR-revX.inc
;   Associated diagram: none
;   Author:             Jacek Siwilo   
;------------------------------------------------------------------------------   
;   Description:        Infrared LIR format (Applied Digital, Inc.)
;                       transmission and receiving for 32MHz clock
;    
;   The Code format:    Byte 0 � carrier frequency data
;                       Bytes 1 to 223 � code data
;                       Bytes 224 to 255 � code name

;   The first byte (Byte0) encodes the carrier frequency with following formula:
;   Freq (in kHz) = 4608 / (Byte0 + 1)
;   The next 223 bytes encode the signal data timing. Each byte contains two
;   fields: The highest bit <7> indicates whether the carrier should be On
;   when = '1' or Off when = '0', and the lower 7 bits <6:0> indicate for how
;   much time, in multiples of 40 us. 
;   Possible time for 1 byte: 0x01-0x7F (carrier off 40us-5080us)
;                             0x81-0xFF (carrier on 40us-5080us)
;   The data byte equal to 0x00 means end of the code.
;
;==============================================================================                   
;   Revision History
;   Rev:  Date:     Details:
;   0     06.2013   Original version
;   1     03.2014   Code length extended from 200 bytes to 224 bytes
;   2     04.2014   Code length extended from 224 bytes to 672 bytes
;                   (3 consecutive codes can be sent at once)
;==============================================================================

;------------------------------------------------------------------------------
; Local Definitions
;------------------------------------------------------------------------------
;These registers must be located in ACCESS BANK
LIR_ADDR_IN_RAM              SET     IRCODEADDR   ;RX address in RAM where LIR code will be saved
;Input Register Definitions
LIR_INTPUT_PORT              SET     IRINPORT     ;output port for infrared LED
LIR_INTPUT_PIN               SET     IRINPIN      ;output pin for infrared LED
;Output Register Definitions
LIR_OUTPUT_PORT              SET     IROUTPORT    ;output port for infrared LED
LIR_OUTPUT_PIN               SET     IROUTPIN     ;output pin for infrared LED
;Working Register Definitions
LIR_FREQUENCY                SET     IRCODEREG0   ;TX & RX frequency register (first in the LIR code)
LIR_PERIOD_ON                SET     IRCODEREG1   ;TX counted IR carrier period (IRED ON)
LIR_PERIOD_OFF               SET     IRCODEREG2   ;TX counted IR carrier period (IRED OFF)
LIR_CODE_BYTE_VAL            SET     IRCODEREG3   ;TX & RX value of byte in the LIR code
LIR_REPEATS_BASE             SET     IRCODEREG4   ;TX base to count period repeats
LIR_REPEATS_BASE_REM_HI      SET     IRCODEREG5   ;TX working reg in Calculate_Repeats_Bas
LIR_REPEATS_BASE_REM_LO      SET     IRCODEREG6   ;TX working reg in Calculate_Repeats_Bas
LIR_REPEATS_H                SET     IRCODEREG7   ;TX period repeats number (MSB byte); RX counter MSB
LIR_REPEATS_L                SET     IRCODEREG8   ;TX period repeats number (LSB byte); RX counter LSB
LIR_CODE_BYTE_CNTH           SET     IRCODEREG9   ;TX & RX counter high byte
LIR_CODE_BYTE_CNTL           SET     IRCODEREG10  ;TX & RX counter low byte

;==============================================================================
;==============================================================================
;       L I R   C O D E   T R A N S M I S S I O N                        
;==============================================================================
;==============================================================================
;   Carrier frequency:  Freq [kHz] = 4608/(first byte+1) -originally
;                       Freq [kHz] = ROUND((148*(first byte+1)-852)/256)
;                                                  -in this file for 32MHz clock
;
;   Carrier frequency range:
;                       Byte 0 - from 08h to FEh gives 500kHz to 18kHz
;   
;   Duty cycle          25%
;==============================================================================

;------------------------------------------------------------------------------
; Routine:          SEND LIR IR CODE
;------------------------------------------------------------------------------
; Overview:         Sends code written in memory at address IR_CODE_ADDRESS
;------------------------------------------------------------------------------
LIR_SendCode     
        rcall   LIR_CheckFrequency               ;check if frequency is ok
        tstfsz  WREG                             ;skip if result 0 = ok
    retlw   0x01  
        rcall   LIR_CalculateCarrierFrequency    ;calculate frequency value from first byte
        rcall   LIR_CalculateRepeatsBase         ;calculate number of period repeats
        rcall   LIR_SendBytes                    ;send the whole code (199 bytes)
ExitSend_LIR_code
    retlw   0x00  

;------------------------------------------------------------------------------
; Routine:          CHECK LIR FREQUENCY
;------------------------------------------------------------------------------
; Overview:         Checks if frequency register (first in the code) has value
;                   between 0x08 and 0xFE
;                   register (first register in LIR IR code)
;    Input:         1. TBLPTRU,TBLPTRH,TBLPTRL registers - address of first
;                      byte of LIR code in flash memory (frequency register)
;   Output:         1. WREG = 0 if frequency reg is ok
;                      WREG = 1 if frequency reg is wrong
;------------------------------------------------------------------------------
LIR_CheckFrequency
        banksel LIR_FREQUENCY
        tblrd*                              ;read memory, memory address must be set to first byte of LIR code (frequency byte)
        movff   TABLAT,LIR_FREQUENCY
        incf    LIR_FREQUENCY
        movlw   0x08
        cpfsgt  LIR_FREQUENCY               ;make sure 0x08 < LIR_FREQUENCY+1
    retlw   0x01                            ;reg value wrong
        decf    LIR_FREQUENCY               ;go back to original value
    retlw   0x00                            ;reg value ok

;------------------------------------------------------------------------------
; Routine:          CALCULATE LIR CARIER FREQUENCY
;------------------------------------------------------------------------------
; Overview:         Calculates IR code carrier frequency from frequency
;                   register (first register in LIR IR code)
;    Input:         1. LIR_FREQUENCY - firt byte in LIR code(frequency register)
;   Output:         1. LIR_PERIOD_ON,LIR_PERIOD_OFF registers - period register
;                      to be used by LIR_Send_Bytes routine
;------------------------------------------------------------------------------
LIR_CalculateCarrierFrequency               ;LIR_PERIOD_ON = ROUND(ROUND(((148*(first byte+1)-852)/256)/4)
        banksel LIR_FREQUENCY               ;LIR_PERIOD_OFF = ROUND(((148*(first byte+1)-852)/256) - LIR_PERIOD_ON
    ;calculate number of LED ON and OFF clock cycles
        incf    LIR_FREQUENCY               ;first byte+1
        movlw   .148                      
        mulwf   LIR_FREQUENCY               ;(first byte+1)*148
        decf    LIR_FREQUENCY               ;restore original value
        decf    PRODH                       ;(first byte+1)*148-852 (-256 in this line)
        decf    PRODH                       ;(-256)
        decf    PRODH                       ;(-256)
        movlw   .84                         ;(-84)
        subwf   PRODL
        btfss   STATUS,C                    ;skip if not borrow
        decf    PRODH                       
        btfsc   PRODL,7                     ;ROUND(((148*(first byte+1)-852)/256) round up if PRODL bit 7 equals 1
        incf    PRODH                       ;PERIOD ON+OFF
        ;calculate IR LED ON register value
        bcf     STATUS,C                    ;divide period by 4 to get on value (25% duty)
        rrcf    PRODH,W
        bcf     STATUS,C
        rrcf    WREG
        btfsc   STATUS,C                    ;ROUND(ROUND(((148*(first byte+1)-852)/256)/4)
        incf    WREG                        ;increment if last rotated bit was 1
        movwf   LIR_PERIOD_ON               ;move to PERIOD_ON
        ;calculate IR LED OFF register value
        subwf   PRODH,W                     ;get PERIOD_OFF = PERIOD - PERIOD_ON 
        movwf   LIR_PERIOD_OFF
    return

;------------------------------------------------------------------------------
; Routine:          CALCULATE LIR REPEAT BASE
;------------------------------------------------------------------------------
; Overview:         Calculates number of period repeats to achieve 8*40us time
;                   It is used to calculate number of repeats for particular
;                   byte in the LIR code.
;    Input:         1. LIR_FREQUENCY - firt byte in LIR code(frequency register)
;   Output:         1. LIR_REPEATS_BASE register - number of period repeats to
;                      achieve 8*40us time for given frequency
;------------------------------------------------------------------------------
LIR_CalculateRepeatsBase                    ;LIR_REPEATS_BASE = ROUND(1475/(first byte+1))
        banksel LIR_FREQUENCY
        movlw   0xB8                        ;11bit dividend 1475=0x05C3 rotated left 5 positions = 0xB860
        movwf   LIR_REPEATS_H               ;use these register temporarily
        movlw   0x60
        movwf   LIR_REPEATS_L               ;use these register temporarily
        incf    LIR_FREQUENCY               ;8bit divider
        clrf    LIR_REPEATS_BASE            ;8bit result
        clrf    LIR_REPEATS_BASE_REM_HI     ;16bit result remainder
        clrf    LIR_REPEATS_BASE_REM_LO

        movlw   .11                         ;shift 11 bits
        movwf   LIR_CODE_BYTE_CNTL
LIR_CRB_Loop
        ;rotate bit by bit
        bcf     STATUS,C
        rlcf    LIR_REPEATS_L,F             ;rotate dividend through 0
        rlcf    LIR_REPEATS_H,F
        rlcf    LIR_REPEATS_BASE_REM_LO,F   ;rotate result remainder through 0
        rlcf    LIR_REPEATS_BASE_REM_HI,F  
        ;check if divider can be deducted from remainder
        movf    LIR_FREQUENCY,W             ;check if LIR_REPEATS_BASE_REMAINDER - LIR_FREQUENCY > 0
        subwf   LIR_REPEATS_BASE_REM_LO,W
        btfss   STATUS,C
        bra     LIR_CRB_DividerGreater      ;C=0, LIR_REPEATS_BASE_REMAINDER - LIR_FREQUENCY < 0
        ;YES-divider can be deducted from remainder
        movf    LIR_FREQUENCY,W             ;C=1, LIR_REPEATS_BASE_REMAINDER - LIR_FREQUENCY > 0
        subwf   LIR_REPEATS_BASE_REM_LO,F    
        bra     LIR_CRB_Shift               ;Shift C=1 into result
        ;NO-divider can't be deducted from remainder
LIR_CRB_DividerGreater
        tstfsz  LIR_REPEATS_BASE_REM_HI
        bra     $ + 4                       ;MSB > 0 
        bra     LIR_CRB_Shift               ;Shift C=0 into result  
        ;YES-divider can be deducted from remainder with borrow from MSB
        movf    LIR_FREQUENCY,W             ;deduct divider from remainder
        subwf   LIR_REPEATS_BASE_REM_LO,F    
        decf    LIR_REPEATS_BASE_REM_HI,F   ;decrement MSB if borrow
        bsf     STATUS,C
        bra     LIR_CRB_Shift               ;Shift C=1 into result
LIR_CRB_Shift
        rlcf    LIR_REPEATS_BASE            ;shift C into result
        decfsz  LIR_CODE_BYTE_CNTL
        bra     LIR_CRB_Loop

        ;round result
        bcf     STATUS,C
        rlcf    LIR_REPEATS_BASE_REM_LO,W   ;multiply remainder by 2
        btfss   STATUS,C                    ;skip if overflow
        cpfsgt  LIR_FREQUENCY               ;skip if divider is greater than 2*remainder
        incf    LIR_REPEATS_BASE            ;increment result (means round up)
        decf    LIR_FREQUENCY               ;restore original value
    return

;------------------------------------------------------------------------------
; Routine:          SEND LIR BYTES
;------------------------------------------------------------------------------
; Overview:         Sends up to 671 bytes from 3 LIR codes. Sending is stopped
;                   when byte 0x00 is read or when all 671 bytes are sent.
;    Input:         1. TBLPTRU,TBLPTRH,TBLPTRL registers - address of first
;                      byte of LIR code in flash memory (frequency register)
;                      (TBLPTR regs change their values in this routine)
;                   2. LIR_PERIOD_ON,LIR_PERIOD_OFF - period register
;                   3. LIR_REPEATS_BASE register - number of period repeats to
;                      achieve 8*40us time for given LIR IR code
;   Output:         1. Pulses on LIR_OUTPUT_PIN pin of LIR_OUTPUT_PORT port
;------------------------------------------------------------------------------
LIR_SendBytes
        banksel LIR_FREQUENCY
        tblrd*+                             ;get second byte of LIR IR code
        movlw   0x02                        ;max number of bytes 671 = 0x29F
        movwf   LIR_CODE_BYTE_CNTH
        movlw   0x9F 
        movwf   LIR_CODE_BYTE_CNTL
LIR_SendBytesLoop
        tblrd*+                             ;read memory
        movff   TABLAT,LIR_CODE_BYTE_VAL
        tstfsz  LIR_CODE_BYTE_VAL           ;byte equal zero?
        bra     $ + 4
    return                                  ;yes, so exit
        ;SPACE or IMPULSE?
        btfss   LIR_CODE_BYTE_VAL,7         ;impulse or space, skip if impulse
        rcall   LIR_SendSpace
        btfsc   LIR_CODE_BYTE_VAL,7         ;impulse or space, skip if space
        rcall   LIR_SendImpulse
        ;all bytes done?
        rcall   LIR_UpdateSendAddress       ;update address if next LIR code must be sent
        decf    LIR_CODE_BYTE_CNTL          ;decrement 16bit counter
        btfss   STATUS,C                    ;borrow?
        decf    LIR_CODE_BYTE_CNTH          ;yes
        tstfsz  LIR_CODE_BYTE_CNTL          ;zero
        bra     LIR_SendBytesLoop           ;no, keep sending
        tstfsz  LIR_CODE_BYTE_CNTH          ;zero
        bra     LIR_SendBytesLoop           ;no, keep sending
    return

;-------------------------------
LIR_SendSpace
        movlw   .106                        ;Wait_40us
        decfsz  WREG
        bra     $ - 2
        decfsz  LIR_CODE_BYTE_VAL
        bra     LIR_SendSpace
    return

;-------------------------------
LIR_SendImpulse
        ;calculate number of repeats for current frequency (LIR_REPEATS_BASE * LIR_CODE_BYTE_VAL)/8
        bcf     LIR_CODE_BYTE_VAL,7
        movf    LIR_REPEATS_BASE,W          ;take repeats base
        mulwf   LIR_CODE_BYTE_VAL           ;and multiply with LIR code register value
        movff   PRODH,LIR_REPEATS_H         ;move result to repeat counter regs
        movff   PRODL,LIR_REPEATS_L
        bcf     STATUS, C                   ;divide result by 8
        rrcf    LIR_REPEATS_H
        rrcf    LIR_REPEATS_L
        bcf     STATUS, C
        rrcf    LIR_REPEATS_H
        rrcf    LIR_REPEATS_L
        bcf     STATUS, C
        rrcf    LIR_REPEATS_H
        rrcf    LIR_REPEATS_L

LIR_SendImpulseLoop
        decf    LIR_REPEATS_L
        btfss   STATUS,C                    ;LSB decremented through zero?
        decf    LIR_REPEATS_H               ;yes C=0, so decrement MSB byte
        btfsc   STATUS,C                    ;MSB decremented through zero?
        bra     $ + 4                       ;no C=1, so keep going
    return 
        ;LIR_Send_Impulse_On
        bsf     LIR_OUTPUT_PORT,LIR_OUTPUT_PIN      ;impulse - ON
        movf    LIR_PERIOD_ON,W
        decfsz  WREG
        bra     $ - 2
        ;LIR_Send_Impulse_Off
        bcf     LIR_OUTPUT_PORT,LIR_OUTPUT_PIN      ;break - OFF
        movf    LIR_PERIOD_OFF,W
        decfsz  WREG
        bra     $ - 2
    bra     LIR_SendImpulseLoop

;-------------------------------
LIR_UpdateSendAddress 
        movlw   0xC1                        ;byte counter equal 449 = 0x1C1? (first cell passed?)
        xorwf   LIR_CODE_BYTE_CNTL,W
        bnz     $ + .8
        movlw   0x01
        xorwf   LIR_CODE_BYTE_CNTH,W
        bz      LIR_UpdateSendAddressNow

        movlw   0xE1                        ;byte counter equal 225 = 0x0E1? (second cell passed?)
        xorwf   LIR_CODE_BYTE_CNTL,W
        bnz     $ + .8
        movlw   0x00
        xorwf   LIR_CODE_BYTE_CNTH,W
        bz      LIR_UpdateSendAddressNow
    return

LIR_UpdateSendAddressNow
        movlw   .32                         ;add 32 to current address
        addwf   TBLPTRL
        movlw   .0
        addwfc  TBLPTRH
        addwfc  TBLPTRU
    return

;==============================================================================
;==============================================================================
;       L I R   C O D E   R E C E I V I N G                        
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:          WAIT FOR IR CODE
;------------------------------------------------------------------------------
; Overview:         Waits about 6s for IR impulse. Exits immediately when
;                   impulse is received
;    Input:         1. Pulses on LIR_INPUT_PIN pin of LIR_INPUT_PORT port
;   Output:         1. WREG = 0 if impulse was received
;                      WREG = 1 if impulse wasn't received
;------------------------------------------------------------------------------
LIR_WaitForImpulse
        banksel LIR_FREQUENCY
        movlw   .146                        ;wait 146*256*LIR_WasteTime
        movwf   LIR_REPEATS_H
        clrf    LIR_REPEATS_L
        btfss   LIR_INTPUT_PORT,LIR_INTPUT_PIN  ;impulse on?
        bra     LIR_WaitForImpulseOK        ;yes
        rcall   LIR_WFI_WasteTime           ;no
        decfsz  LIR_REPEATS_L
        bra     $ - .8                      ;internal loop
        decfsz  LIR_REPEATS_H
        bra     $ - .12                     ;external loop
    bra     LIR_WaitForImpulseWRONG         ;no impulse received within 6s

;-------------------------------
LIR_WFI_WasteTime                           ;160us
        clrf    WREG
        nop
        nop    
        decfsz  WREG
        bra     $ - 6
        return

;-------------------------------
LIR_WaitForImpulseOK
    retlw   0x00

LIR_WaitForImpulseWRONG
    retlw   0x01

;------------------------------------------------------------------------------
; Routine:          RECEIVE IR CARRIER
;------------------------------------------------------------------------------
; Overview:         Reads carrier of the received infrared code
;    Input:         1. Pulses on LIR_INPUT_PIN pin of LIR_INPUT_PORT port
;                      (active low state)
;                   2. LIR_ADDR_IN_RAM - address of first byte in RAM for
;                      received IR code (0xXXX)
;   Output:         1. LIR_FREQUENCY register with adequate value
;                   2. RAM - LIR_FREQUENCY value written into RAM location
;                   3. WREG = 0 if reading was ok
;                      WREG = 1 if reading was wrong
;------------------------------------------------------------------------------
LIR_ReceiveCarrier
        rcall   LIR_WaitForImpulse          ;wait a few seconds for signal
        tstfsz  WREG
    bra     LIR_ReceiveCarrierWRONG         ;no impulse received

;-------------------------------
        banksel LIR_FREQUENCY
        ;now wait for break and then start counting period time
LIR_ReceiveImpulse
        clrf    LIR_FREQUENCY
        infsnz  LIR_FREQUENCY
    bra     LIR_ReceiveCarrierWRONG         ;impulse too long
        btfss   LIR_INTPUT_PORT,LIR_INTPUT_PIN  ;impulse still on?
        bra     $ - 6                       ;yes, so wait for impulse end
        ;start counting period time (break+impulse)
        clrf    LIR_FREQUENCY
        infsnz  LIR_FREQUENCY               ;count break time
    bra     LIR_ReceiveCarrierWRONG         ;break too long
        btfsc   LIR_INTPUT_PORT,LIR_INTPUT_PIN  ;impulse still off?
        bra     $ - 6                       ;yes, loop until input high again
        nop                                 ;time compensation for higher frequencies
        nop
        nop
        nop
        nop
        nop
        infsnz  LIR_FREQUENCY               ;count impulse time
    bra     LIR_ReceiveCarrierWRONG         ;impulse too long
        btfss   LIR_INTPUT_PORT,LIR_INTPUT_PIN  ;impulse still on?
        bra     $ - 6                       ;yes, loop until input high again

;-------------------------------
        ;calculate LIR frequency value LIR_FREQUENCY=ROUND((counted_period+1)*46/16)-1
        incf    LIR_FREQUENCY               ;counted_period+1
        movlw   .46
        mulwf   LIR_FREQUENCY               ;(counted_period+1)*46
        bcf     STATUS, C                   ;(counted_period+1)*46/16
        rrcf    PRODH
        rrcf    PRODL
        bcf     STATUS, C                 
        rrcf    PRODH
        rrcf    PRODL
        bcf     STATUS, C                 
        rrcf    PRODH
        rrcf    PRODL
        bcf     STATUS, C                 
        rrcf    PRODH
        rrcf    PRODL
        btfsc   STATUS,C                    ;ROUND((counted_period+1)*46/16)
        incf    PRODL                       ;increment if last rotated bit was 1
        decf    PRODL,W                     ;ROUND((counted_period+1)*46/16)-1
        movwf   LIR_FREQUENCY
        ;test result if 8<=LIR_FREQUENCY<=254
        incf    LIR_FREQUENCY
        movlw   0x08
        cpfsgt  LIR_FREQUENCY               ;make sure 0x08 < LIR_FREQUENCY+1
    bra     LIR_ReceiveCarrierWRONG         ;reg value wrong
        decf    LIR_FREQUENCY               ;go back to original value
        ;save frequency to RAM
        lfsr    FSR0,LIR_ADDR_IN_RAM        ;point at first reg in ram memory
        movff   LIR_FREQUENCY,INDF0         ;first byte in the code indicates frequency

;-------------------------------
LIR_ReceiveCarrierOK
    retlw   0x00

LIR_ReceiveCarrierWRONG
    retlw   0x01

;------------------------------------------------------------------------------
; Routine:          RECEIVE FILTERED IR CODE
;------------------------------------------------------------------------------
; Overview:         Receives filtered (code without carrier)and saves IR LIR
;                   format code into given RAM location
;    Input:         1. Pulses without carrier on LIR_INPUT_PIN pin of
;                      LIR_INPUT_PORT port (active low state)
;                   2. LIR_ADDR_IN_RAM - address of first byte in RAM for
;                      received IR code (0xXXX)
;   Output:         1. 672 bytes of LIR format code starting from address of
;                      RAM memory indicated by LIR_ADDR_IN_RAM reg
;                   2. WREG = 0 if reading was ok
;                      WREG = 1 if reading was wrong
;------------------------------------------------------------------------------
LIR_ReceiveFilteredCode
        rcall   LIR_WaitForImpulse          ;wait a few seconds for signal
        tstfsz  WREG
    bra     LIR_ReceiveCodeWRONG            ;no impulse received

;-------------------------------
        banksel LIR_FREQUENCY
        ;now wait for break and then start counting period time
        movlw   0x02                        ;max number of bytes 672 = 0x2A0
        movwf   LIR_CODE_BYTE_CNTH
        movlw   0xA0 
        movwf   LIR_CODE_BYTE_CNTL
        lfsr    FSR0,LIR_ADDR_IN_RAM        ;point at first reg in ram memory
LIR_RCo_CodeLoop
        ;start counting impulse time
        incf    FSR0L                       ;get next reg in ram
        bnc     $ + .4                      ;go to next ram bank if overflow
        incf    FSR0H
        movlw   .128                        ;start counting from value 128 (possible 127 steps)
        movwf   LIR_CODE_BYTE_VAL
        btfsc   LIR_INTPUT_PORT,LIR_INTPUT_PIN  ;impulse or space?
        bra     LIR_RCo_ByteLoopSpace       ;space

;start counting impulse time
LIR_RCo_ByteLoopImp
        infsnz  LIR_CODE_BYTE_VAL           ;increment every 40us
        bra     LIR_RCo_ByteOverflowImp     ;impulse longer than 127*40us
        movlw   .105                        ;waste  time to read pin every 40us
        movwf   WREG
        decfsz  WREG
        bra     $ - 2
        btfss   LIR_INTPUT_PORT,LIR_INTPUT_PIN  ;impulse still on?
        bra     LIR_RCo_ByteLoopImp         ;yes, so stay in loop
        movff   LIR_CODE_BYTE_VAL,INDF0     ;move value into ram
        bra     LIR_RCo_CodeLoopEnd
LIR_RCo_ByteOverflowImp
        setf    INDF0                       ;set 0xFF in ram
        bra     LIR_RCo_CodeLoopEnd         ;go back to loop

;start counting space time
LIR_RCo_ByteLoopSpace
        infsnz  LIR_CODE_BYTE_VAL           ;increment every 40us
        bra     LIR_RCo_ByteOverflowSpace   ;space longer than 127*40us
        movlw   .105                        ;waste  time to read pin every 40us
        movwf   WREG
        decfsz  WREG
        bra     $ - 2
        btfsc   LIR_INTPUT_PORT,LIR_INTPUT_PIN  ;still space?
        bra     LIR_RCo_ByteLoopSpace       ;yes, so stay in loop
        bcf     LIR_CODE_BYTE_VAL,7         ;clear bit that indicates space
        movff   LIR_CODE_BYTE_VAL,INDF0     ;move value into ram
        bra     LIR_RCo_CodeLoopEnd
LIR_RCo_ByteOverflowSpace
        setf    INDF0                       ;set 0x7F in ram
        bcf     INDF0,7                     ;clear bit that indicates space
        bra     LIR_RCo_CodeLoopEnd         ;go back to loop

LIR_RCo_CodeLoopEnd
        decf    LIR_CODE_BYTE_CNTL          ;decrement 16bit counter
        btfss   STATUS,C                    ;borrow?
        decf    LIR_CODE_BYTE_CNTH          ;yes
        tstfsz  LIR_CODE_BYTE_CNTL          ;zero
        bra     LIR_RCo_CodeLoop            ;no
        tstfsz  LIR_CODE_BYTE_CNTH          ;zero
        bra     LIR_RCo_CodeLoop            ;no
        bra     LIR_ReceiveCodeOK           ;yes, so exit with ok flag

;-------------------------------
LIR_ReceiveCodeOK
    retlw   0x00

LIR_ReceiveCodeWRONG
    retlw   0x01

;------------------------------------------------------------------------------
