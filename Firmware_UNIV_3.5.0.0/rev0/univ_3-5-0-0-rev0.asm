;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2013 Jacek Siwilo
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
;   Filename:              univ_3-5-0-0.asm
;   Associated diagram:    univ_3-5-0-x.sch
;   Author:                Jacek Siwilo                          
;   Note:                  Infrared transmitter
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     05.2013   Original version
;==============================================================================
;===  FIRMWARE DEFINITIONS  =================================================== 
;==============================================================================
    #define    ATYPE    .5                            ;application type [0-255]
    #define    AVERS    .0                         ;application version [0-255]
    #define    FVERS    .0                            ;firmware version [0-255]

    #define    FREV     .0                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include <univ_3-5-0-0-rev0.inc>                         ;project variables
ROUTINES   code    0x003000
    #include <univ3-routines-rev0.inc>                     ;UNIV 3 CPU routines
    #include <univ3-IR_LIR-rev0.inc>                               ;IR LIR code
    #include <univ3-fake_bootloader-rev2.inc>    ;fake bootloader for debugging

;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x6a, 0x1a, 0xf9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF			
;==============================================================================
;===  FIRMWARE ID  ============================================================
;==============================================================================
FIRMID      code    0x001010
    DB      0x30, 0x00, 0x03,ATYPE,AVERS,FVERS,FREV>>8,FREV
;            |     |     |     |     |     |     |_____|_____ firmware revision
;            |     |     |     |     |     |__________________ firmware version
;            |     |     |     |     |_____________________ application version
;            |     |     |     |______________________________ application type
;            |     |     |________________________________ hardware version '3'
;            |_____|______________________________________ hardware type 'UNIV'
;==============================================================================
;===  MOVED VECTORS  ==========================================================
;==============================================================================
;PROGRAM RESET VECTOR
FIRMRESET   code    0x1020
        goto    Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x1030
        call    HighInterrupt
        retfie
;PROGRAM LOW PRIORITY INTERRUPT VECTOR
FIRMLOWINT  code    0x1040
        call    LowInterrupt
        retfie

;==============================================================================
;===  FIRMWARE STARTS  ========================================================
;==============================================================================
FIRMSTART   code    0x001050
;------------------------------------------------------------------------------
;---  LOW PRIORITY INTERRUPT  -------------------------------------------------
;------------------------------------------------------------------------------
LowInterrupt
        movff   STATUS,STATUS_LOW           ;save STATUS register
        movff   WREG,WREG_LOW               ;save working register
        movff   BSR,BSR_LOW                 ;save BSR register
        movff   FSR0L,FSR0L_LOW             ;save other registers used in high int
        movff   FSR0H,FSR0H_LOW
        movff   FSR1L,FSR1L_LOW
        movff   FSR1H,FSR1H_LOW

    ;main firmware ready flag
        movlb   0x1
        btfss   FIRMREADY,0
        bra     ExitLowInterrupt            ;main firmware is not ready yet
    ;CAN buffer
        movlb   0x1
        btfsc   CANFULL,0                   ;check if CAN received anything
        call    CANInterrupt                ;proceed with CAN interrupt

ExitLowInterrupt
        movff   BSR_LOW,BSR                 ;restore BSR register
        movff   WREG_LOW,WREG               ;restore working register
        movff   STATUS_LOW,STATUS           ;restore STATUS register
        movff   FSR0L_LOW,FSR0L             ;restore other registers used in high int
        movff   FSR0H_LOW,FSR0H
        movff   FSR1L_LOW,FSR1L
        movff   FSR1H_LOW,FSR1H
    return

;------------------------------------------------------------------------------
;---  HIGH PRIORITY INTERRUPT  ------------------------------------------------
;------------------------------------------------------------------------------
HighInterrupt
        movff   STATUS,STATUS_HIGH          ;save STATUS register
        movff   WREG,WREG_HIGH              ;save working register
        movff   BSR,BSR_HIGH                ;save BSR register
        movff   FSR0L,FSR0L_HIGH            ;save other registers used in high int
        movff   FSR0H,FSR0H_HIGH
        movff   FSR1L,FSR1L_HIGH
        movff   FSR1H,FSR1H_HIGH

    ;main firmware ready flag
        movlb   0x1
        btfss   FIRMREADY,0
        bra     ExitHighInterrupt           ;main firmware is not ready yet
    ;Timer0
		btfsc	INTCON,TMR0IF			    ;Timer0 interrupt?
		rcall	Timer0Interrupt

ExitHighInterrupt
        movff   BSR_HIGH,BSR                ;restore BSR register
        movff   WREG_HIGH,WREG              ;restore working register
        movff   STATUS_HIGH,STATUS          ;restore STATUS register
        movff   FSR0L_HIGH,FSR0L            ;restore other registers used in high int
        movff   FSR0H_HIGH,FSR0H
        movff   FSR1L_HIGH,FSR1L
        movff   FSR1H_HIGH,FSR1H
    return

;------------------------------------------------------------------------------
; Routine:			CAN INTERRUPT
;------------------------------------------------------------------------------
; Overview:			Checks CAN message for response and RTR and saves to FIFO
;------------------------------------------------------------------------------
CANInterrupt
		banksel CANFRAME2
		btfsc	CANFRAME2,0				    ;response message?
	return                                  ;yes, so ignore it and exit
		btfsc	CANFRAME2,1                 ;RTR (Remote Transmit Request)?
	return                                  ;yes, so ignore it and exit
        call    Copy_RXB_RXFIFOIN           ;copies received message to CAN RX FIFO input buffer
        call    WriteToCanRxFIFO		    ;saves message to FIFO
    return

;------------------------------------------------------------------------------
; Routine:			TIMER 0 INTERRUPT
;------------------------------------------------------------------------------
; Overview:			1000ms periodical interrupt
;------------------------------------------------------------------------------
Timer0Interrupt:
        call    Timer0Initialization32MHz   ;restart 1000ms Timer 
        call    UpdateUpTime                ;counts time from restart
        call    UpdateTransmitTimer         ;increment transmit timer (seconds after last transmission)
    return

;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Main:
    ;disable global interrupts for startup
        bcf     INTCON,GIEH                 ;disable high interrupt
        bcf     INTCON,GIEL                 ;disable low interrupt
    ;multiply oscilator clock to 32MHz
        bsf     OSCTUNE,PLLEN               ;enable PLL
    ;initiate ports                         ;default all pins set as analog (portA) or digital (portB,C) inputs 
        ;PORT A
        movlb   0xF                         ;bank 16
        movlw   b'11111111'                 ;0 -digital input, 1 -analog input
        movwf   ANCON0
        clrf    LATA                        ;all low
        movlw   b'11111111'                 ;all inputs
        movwf   TRISA        
        ;PORT B
        movlw   b'11111011'                 ;0 -digital input, 1 -analog input
        movwf   ANCON1
        clrf    LATB                        ;all low
        movlw   b'00001001'                 ;all output except canrx & IRRx
        movwf   TRISB
        ;PORT C
        clrf    LATC                        ;all low
        movlw   b'00000000'                 ;all output 
        movwf   TRISC
    ;firmware initialization
        call    GeneralInitialization       ;read eeprom config, clear other registers
        call    FIFOInitialization          ;prepare FIFO buffers
		call	Timer0Initialization32MHz   ;Timer 0 initialization for 1s periodical interrupt  
    ;firmware ready
        movlb   0x1
        bsf     FIRMREADY,0                 ;set flag "firmware started and ready for interrupts"
    ;enable global interrupts
        bsf     INTCON,GIEH                 ;enable high interrupt
        bsf     INTCON,GIEL                 ;enable low interrupt

Loop:                                       ;main loop
        clrwdt                              ;clear Watchdog timer
        call   ReceiveProcedure             ;check if any msg in RX FIFO and if so - process the msg
        call   TransmitProcedure            ;check if any msg in TX FIFO and if so - transmit it
    bra     Loop

;==============================================================================
;===  FIRMWARE ROUTINES  ======================================================
;==============================================================================
;------------------------------------------------------------------------------
; Routine:			NODE STATUS
;------------------------------------------------------------------------------
; Overview:			It prepares status messages when status request was
;                   received
;------------------------------------------------------------------------------
NodeStatusRequest
    return                                  ;nothing to send
;------------------------------------------------------------------------------
; Routine:			DO INSTRUCTION
;------------------------------------------------------------------------------
; Overview:			Executes instruction immediately or sets timer for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionRequest
;Recognize instruction
        banksel INSTR1
		movlw	0x00			        	;instruction 00?
		xorwf	INSTR1,W
		bz		Instr00
		movlw	0x01			    	    ;instruction 01?
		xorwf	INSTR1,W
		bz		Instr01
		movlw	0x02			    	    ;instruction 02?
		xorwf	INSTR1,W
		bz		Instr02
		movlw	0x03			        	;instruction 03?
		xorwf	INSTR1,W
		bz		Instr03
		movlw	0x04			    	    ;instruction 04?
		xorwf	INSTR1,W
		bz		Instr04
		movlw	0x05			    	    ;instruction 05?
		xorwf	INSTR1,W
		bz		Instr05
	bra		EndDoInstructionRequest	        ;exit if unknown instruction

;-------------------------------
;Instruction execution
Instr00									    ;receive carrier or code and reply
		movlw	0x00			    	    ;carrier
		xorwf	INSTR2,W
		bnz		$ + .20
        bcf     INTCON,GIEH                 ;disable high interrupt
        bcf     INTCON,GIEL                 ;disable low interrupt
        call    LIR_ReceiveCarrier
        call    Status_LearntCarrier
        bsf     INTCON,GIEH                 ;enable high interrupt
        bsf     INTCON,GIEL                 ;enable low interrupt
		bra		EndDoInstructionRequest
		movlw	0x01			        	;code
		xorwf	INSTR2,W
		bnz		$ + .18
        bcf     INTCON,GIEH                 ;disable high interrupt
        bcf     INTCON,GIEL                 ;disable low interrupt
        call    LIR_ReceiveFilteredCode
        call    Status_LearntCode
        bsf     INTCON,GIEH                 ;enable high interrupt
        bsf     INTCON,GIEL                 ;enable low interrupt
		bra		EndDoInstructionRequest
Instr01									    ;send code indicated in INSTR2 from memory
        clrf    TBLPTRU
        movlw   0x97                        ;address of first LIR code in memory - 1
        addwf   INSTR2,W                    ;add number of code from INST2 (1-100)
        movwf   TBLPTRH
        clrf    TBLPTRL
        bcf     INTCON,GIEH                 ;disable high interrupt
        bcf     INTCON,GIEL                 ;disable low interrupt
        call    LIR_SendCode
        bsf     INTCON,GIEH                 ;enable high interrupt
        bsf     INTCON,GIEL                 ;enable low interrupt
		bra		EndDoInstructionRequest
Instr02									    ;
		bra		EndDoInstructionRequest
Instr03									    ;
		bra		EndDoInstructionRequest
Instr04									    ;
		bra		EndDoInstructionRequest
Instr05									    ;
		bra		EndDoInstructionRequest

;-------------------------------
EndDoInstructionRequest
		setf	INSTR1			    	    ;clear instruction
		setf	INSTR2			    
		setf	INSTR3			    
		setf	INSTR4			    
		setf	INSTR5			    
		setf	INSTR6			    
		setf	INSTR7			    
		setf	INSTR8			    
	return			        	

;------------------------------------------------------------------------------
; Routine:			SEND NODE STATUSES
;------------------------------------------------------------------------------
;Response when node was asked to learn the IR code or carrier an their where wrong
Status_LearntError
        banksel TXFIFOIN0
        movlw   0x30			            ;set IR RX frame
		movwf   TXFIFOIN0
        movlw   0x30
		movwf   TXFIFOIN1
		bsf		TXFIFOIN1,0					;response bit
		movff	NODENR,TXFIFOIN2            ;node id
		movff	GROUPNR,TXFIFOIN3
        movlw   0xF0                        ;error frame
        movwf   TXFIFOIN4
		setf	TXFIFOIN5			        ;unused
		setf	TXFIFOIN6
		setf	TXFIFOIN7
		setf	TXFIFOIN8
		setf	TXFIFOIN9
		setf	TXFIFOIN10
		setf	TXFIFOIN11
		call	WriteToCanTxFIFO
EndStatus_LearntError
    return
;------------------------------------------------------------------------------
;Response when node was asked to learn the IR code
Status_LearntCode
        tstfsz  WREG                        ;code learnt ok?
        bra     Status_LearntError          ;no
        banksel TXFIFOIN0
        movlw   .29                         ;number of frames to send
        movwf   R0
        lfsr    FSR2,IRCODEADDR             ;point at address in am where code is
        ;frame header
        movlw   0x30			            ;set IR RX frame
		movwf   TXFIFOIN0
        movlw   0x30
		movwf   TXFIFOIN1
		bsf		TXFIFOIN1,0					;response bit
		movff	NODENR,TXFIFOIN2            ;node id
		movff	GROUPNR,TXFIFOIN3
        clrf    TXFIFOIN4                   ;frame no
Status_LearntCodeLoop
        banksel TXFIFOIN0
        incf    TXFIFOIN4                   ;frame no - update value
		movff	POSTINC2,TXFIFOIN5			;move byte from code into transmit frame
        movff   POSTINC2,TXFIFOIN6
		movff	POSTINC2,TXFIFOIN7
        movff   POSTINC2,TXFIFOIN8
        movff   POSTINC2,TXFIFOIN9
        movff   POSTINC2,TXFIFOIN10
        movff   POSTINC2,TXFIFOIN11
		call	WriteToCanTxFIFO
        decfsz  R0
        bra     Status_LearntCodeLoop
EndStatus_LearntCode
    return
;------------------------------------------------------------------------------
;Response when node was asked to learn the IR code
Status_LearntCarrier
        tstfsz  WREG                        ;carrier learnt ok?
        bra     Status_LearntError          ;no
        banksel TXFIFOIN0
        movlw   0x30			            ;set IR RX frame
		movwf   TXFIFOIN0
        movlw   0x30
		movwf   TXFIFOIN1
		bsf		TXFIFOIN1,0					;response bit
		movff	NODENR,TXFIFOIN2            ;node id
		movff	GROUPNR,TXFIFOIN3
        clrf    TXFIFOIN4                   ;carrier frame
		movff	LIR_FREQUENCY,TXFIFOIN5     ;carrier value
		setf	TXFIFOIN6			        ;unused
		setf	TXFIFOIN7
		setf	TXFIFOIN8
		setf	TXFIFOIN9
		setf	TXFIFOIN10
		setf	TXFIFOIN11
		call	WriteToCanTxFIFO
EndStatus_LearntCarrier
    return

;==============================================================================
;===  END  OF  PROGRAM  =======================================================
;==============================================================================
    END