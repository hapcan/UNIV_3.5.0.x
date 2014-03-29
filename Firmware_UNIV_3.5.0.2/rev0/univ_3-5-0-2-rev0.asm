;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2014 hapcan.com
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
;   Filename:              univ_3-5-0-2.asm
;   Associated diagram:    univ_3-5-0-x.sch
;   Author:                Jacek Siwilo                          
;   Note:                  Infrared transmitter /receiver
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     03.2014   Original version
;==============================================================================
;===  FIRMWARE DEFINITIONS  =================================================== 
;==============================================================================
    #define    ATYPE    .5                            ;application type [0-255]
    #define    AVERS    .0                         ;application version [0-255]
    #define    FVERS    .2                            ;firmware version [0-255]

    #define    FREV     .0                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include "univ_3-5-0-2-rev0.inc"                         ;project variables
    #include "univ3-fake_bootloader-rev2.inc"    ;fake bootloader for debugging
INCLUDEDFILES   code  
    #include "univ3-routines-rev3.inc"                     ;UNIV 3 CPU routines
    #include "univ3-IR_LIR-rev1.inc"                        ;LIR infrared codes
    #include "univ3-IR_Sony_SIRC-rev0.inc"                 ;Sony infrared codes
    #include "univ3-IR_Philips_RC5-rev0.inc"            ;Philips infrared codes
    #include "univ3-IR_NEC-rev0.inc"                        ;NEC infrared codes

;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x62, 0xbc, 0x05, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF			
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
FIRMRESET   code    0x001020
        goto    Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x001030
        call    HighInterrupt
        retfie
;PROGRAM LOW PRIORITY INTERRUPT VECTOR
FIRMLOWINT  code    0x001040
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
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitLowInterrupt            ;main firmware is not ready yet
    ;CAN buffer
        banksel CANFULL
        btfsc   CANFULL,0               ;check if CAN received anything
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
        banksel FIRMREADY
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
        banksel TIMER0_1000ms
        setf    TIMER0_1000ms               ;timer 0 interrupt occurred flag
    return

;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Main:
    ;disable global interrupts for startup
        call    DisAllInt                   ;disable all interrupt
    ;multiply oscilator clock to 32MHz
        bsf     OSCTUNE,PLLEN               ;enable PLL
    ;firmware initialization
        rcall   PortInitialization          ;prepare processor ports
        call    GeneralInitialization       ;read eeprom config, clear other registers
        call    FIFOInitialization          ;prepare FIFO buffers
		call	Timer0Initialization32MHz   ;Timer 0 initialization for 1s periodical interrupt  
    ;firmware ready
        banksel FIRMREADY
        bsf     FIRMREADY,0                 ;set flag "firmware started and ready for interrupts"
    ;enable global interrupts
        call    EnAllInt                    ;enable all interrupts

Loop:                                       ;main loop
        clrwdt                              ;clear Watchdog timer
        call    ReceiveProcedure            ;check if any msg in RX FIFO and if so - process the msg
        call    TransmitProcedure           ;check if any msg in TX FIFO and if so - transmit it
        call    IR_ReceiveCode              ;checks if any IR signal is being transmitted
        rcall   OnceA1000ms                 ;do routines only after 1000ms interrupt
    bra     Loop

;-------------------------------
OnceA1000ms                                 ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER0_1000ms
        tstfsz  TIMER0_1000ms               ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    UpdateHealthRegs            ;saves health maximums to eeprom
        banksel TIMER0_1000ms
        clrf    TIMER0_1000ms
    return

;==============================================================================
;===  FIRMWARE ROUTINES  ======================================================
;==============================================================================
;------------------------------------------------------------------------------
; Routine:			PORT INITIALIZATION
;------------------------------------------------------------------------------
; Overview:			It sets processor pins. All unused pins should be set as
;                   outputs and driven low
;------------------------------------------------------------------------------
PortInitialization                          ;default all pins set as analog (portA,B) or digital (portB,C) inputs 
    ;PORT A
        banksel ANCON0                      ;select memory bank
        ;0-digital, 1-analog input
        movlw   b'00000011'                 ;(x,x,x,AN4,AN3,AN2,AN1-boot_mode,AN0-volt)
        movwf   ANCON0
        ;output level
        clrf    LATA                        ;all low
        ;0-output, 1-input
        movlw   b'00000011'                 ;all outputs except, bit<1>-boot_mode, bit<0>-volt
        movwf   TRISA        
    ;PORT B
        ;0-digital, 1-analog input
        movlw   b'00000000'                 ;(x,x,x,x,x,AN10,AN9,AN8)
        movwf   ANCON1
        ;output level
        clrf    LATB                        ;all low
        ;0-output, 1-input
        movlw   b'00001001'                 ;all output except canrx & IRRx
        movwf   TRISB
        ;pull up
        bcf     INTCON2,RBPU                ;enable pull ups on B port
        bsf     WPUB,WPUB0                  ;enable pull up on RB0 pin (IRRx)
    ;PORT C
        ;output level
        clrf    LATC                        ;all low
        ;0-output, 1-input
        movlw   b'00000000'                 ;all output 
        movwf   TRISC
    return

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
        banksel INSTR1                      ;allow only known values
        movlw   0x09                        ;INSTR less than?
        cpfslt  INSTR1
    bra     ExitDoInstructionRequest        ;no, so exit
        movf    INSTR1,W                    ;recognize instruction
        call    ComputedGoto                ;modify PC according to WREG value
        bra     Instr00                     ;instruction 00
        bra     Instr01                     ;instruction 01
        bra     Instr02                     ;instruction 02
        bra     Instr03                     ;instruction 03
        bra     Instr04                     ;instruction 04
        bra     Instr05                     ;instruction 05
        bra     Instr06                     ;instruction 06
        bra     Instr07                     ;instruction 07
        bra     Instr08                     ;instruction 08
ExitDoInstructionRequest
    return 

;-------------------------------
;Instruction execution
Instr00									    ;receive carrier or code and reply
		movlw	0x00			    	    ;carrier
		xorwf	INSTR2,W
		bnz		$ + .20
        call    DisAllInt                   ;disable all interrupt
        call    LIR_ReceiveCarrier
        call    Status_LearntCarrier
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest
		movlw	0x01			        	;code
		xorwf	INSTR2,W
		bnz		$ + .18
        call    DisAllInt                   ;disable all interrupt
        call    LIR_ReceiveFilteredCode
        call    Status_LearntCode
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest
;---------------
Instr01									    ;send code indicated in INSTR2 from memory
        clrf    TBLPTRU
        movlw   0x97                        ;address of first LIR code in memory - 1
        addwf   INSTR2,W                    ;add number of code from INST2 (1-100)
        movwf   TBLPTRH
        clrf    TBLPTRL
        call    DisAllInt                   ;disable all interrupt
        call    LIR_SendCode
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest
;---------------
Instr02
        movlw   0x30                        ;only infrared receiver message can
        xorwf   RXFIFO0,W                   ;  execute this instruction,
        bnz     ExitDoInstructionRequest    ;  is this IRRX frame (0x3030)?
        movlw   0x30
        xorwf   RXFIFO1,W
        bnz     ExitDoInstructionRequest
        ;send the same code as received
        movff   RXFIFO6,INSTR1               
        movff   RXFIFO7,INSTR2
        movff   RXFIFO8,INSTR3
        movff   RXFIFO9,INSTR4
        movlw   0x03                        ;make sure, the code type is not less than 3
        cpfslt  INSTR1
        bra     DoInstructionRequest        ;send received code
        bra     ExitDoInstructionRequest
;---------------
Instr03									    ;send SIRC 12bit
        movff   INSTR2,SIRC_ADDRESS_L       ;code address_L
        movff   INSTR3,SIRC_COMMAND         ;code command
        call    DisAllInt                   ;disable all interrupt
        call    SIRC_12bit_Transmit
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest 
;---------------
Instr04									    ;send SIRC 15bit
        movff   INSTR2,SIRC_ADDRESS_L       ;code address_L
        movff   INSTR3,SIRC_COMMAND         ;code command
        call    DisAllInt                   ;disable all interrupt
        call    SIRC_15bit_Transmit
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest
;---------------
Instr05									    ;send SIRC 20bit
        movff   INSTR2,SIRC_ADDRESS_H       ;code address_H
        movff   INSTR3,SIRC_ADDRESS_L       ;code address_L
        movff   INSTR4,SIRC_COMMAND         ;code command
        call    DisAllInt                   ;disable all interrupt
        call    SIRC_20bit_Transmit
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest
;---------------
Instr06									    ;send RC5x code
        movff   INSTR2,RC5_ADDRESS          ;code address
        movff   INSTR3,RC5_COMMAND          ;code command
        call    DisAllInt                   ;disable all interrupt
        call    RC5_Transmit
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest
;---------------
Instr07									    ;send Standard NEC code
        movff   INSTR2,NEC_ADDRESS_L        ;code address
        movff   INSTR3,NEC_COMMAND          ;code command
        call    DisAllInt                   ;disable all interrupt
        call    NEC_Std_Transmit
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest
;---------------
Instr08						                ;send Extended NEC code
        movff   INSTR2,NEC_ADDRESS_H        ;code address_H
        movff   INSTR3,NEC_ADDRESS_L        ;code address_L
        movff   INSTR4,NEC_COMMAND          ;code command
        call    DisAllInt                   ;disable all interrupt
        call    NEC_Ext_Transmit
        call    EnAllInt                    ;enable all interrupts
        bra     ExitDoInstructionRequest       	

;------------------------------------------------------------------------------
; Routine:			SEND LEARNING CODE STATUS
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
        tstfsz  WREG                        ;learnt code ok?
        bra     Status_LearntError          ;no
        banksel TXFIFOIN0
        movlw   .32                         ;number of frames to send
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
;Response when node was asked to learn the IR carrier
Status_LearntCarrier
        tstfsz  WREG                        ;learnt carrier ok?
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

;------------------------------------------------------------------------------
; Routine:			RECEIVE INFRARED CODE
;------------------------------------------------------------------------------
; Overview:			It checks if any infrared code is being received,
;                   recognizes it by the code header and forms CAN message.
;------------------------------------------------------------------------------
IR_ReceiveCode
        btfsc   IRINPORT,IRINPIN            ;impulse present?
    return                                  ;no, so exit

        banksel IRTIMECOUNTER
        call    DisAllInt                   ;disable all interrupts
        clrf    IRTIMECOUNTER               ;clear ipmluse/space length counter
;--------------
;count header timer
IRLoopCountHeader
		infsnz	IRTIMECOUNTER               ;increment every 40us
		bra		IR_ReceiveCodeExit          ;impulse longer than 255*40us, so exit
        movlw   .104                        ;waste time to read pin every 40us
		movwf	WREG
		decfsz	WREG
		bra		$ - 2
        btfss   IRINPORT,IRINPIN            ;impulse still on?
        bra     IRLoopCountHeader           ;yes, so stay in loop
;--------------
;check what header was received
IRCheckHeader                               ;checks header with +-15% precision
        movlw   .19                         ;19x40=760us min for RC5 (889-15%)
        cpfsgt  IRTIMECOUNTER               ;skip if more
        bra     IR_ReceiveCodeError
        movlw   .25                         ;25x40=680us max for RC5 (889+15%)
        cpfsgt  IRTIMECOUNTER               ;skip if more
        bra     IR_RC5
        movlw   .38                         ;38x40=1520us min RC5x (2*889-15%)
        cpfsgt  IRTIMECOUNTER               ;skip if more
        bra     IR_ReceiveCodeError
        movlw   .50                         ;50x40=2000us max RC5x (2*889+15%)
        cpfsgt  IRTIMECOUNTER               ;skip if more
        bra     IR_RC5x
        movlw   .51                         ;51x40=2040us min for SIRC (2400-15%)
        cpfsgt  IRTIMECOUNTER               ;skip if more
        bra     IR_ReceiveCodeError
        movlw   .69                         ;69x40=2760us max for SIRC (2400+15%)
        cpfsgt  IRTIMECOUNTER               ;skip if more
        bra     IR_SIRC
        movlw   .192                        ;192x40=7680us min for NEC (9000-15%)
        cpfsgt  IRTIMECOUNTER               ;skip if more
        bra     IR_ReceiveCodeError
        movlw   .255                        ;255x40=10200us max for SIRC (9000+15%)
        cpfsgt  IRTIMECOUNTER               ;skip if more
        bra     IR_NEC
        bra     IR_ReceiveCodeError
;--------------
;receive the rest of IR code
IR_RC5
        call    RC5_Received                ;receive all bits of the code
        tstfsz  WREG                        ;received code ok?
        bra     $ + 6                       ;no, wait for another frame
        rcall   IR_RC5_OrderRegs            ;prepare registers to send
        rcall   IR_SendToCAN                ;yes, so send it to CAN
        call    RC5_Frame_Repetition        ;wait for another frame
        tstfsz  WREG                        ;another frame began?
        bra     IR_ReceiveCodeExit          ;no, so exit
        bra     IR_ReceiveCode              ;yes, so start reading
IR_RC5x
        call    RC5x_Received
        tstfsz  WREG                        ;received code ok?
        bra     $ + 4                       ;no, wait for another frame
        rcall   IR_RC5_OrderRegs            ;prepare registers to send
        rcall   IR_SendToCAN                ;yes, so send it to CAN
        call    RC5_Frame_Repetition        ;wait for another frame
        tstfsz  WREG                        ;another frame began?
        bra     IR_ReceiveCodeExit          ;no, so exit
        bra     IR_ReceiveCode              ;yes, so start reading
IR_SIRC
        call    SIRC_Received
        tstfsz  WREG                        ;received code ok?
        bra     $ + 6                       ;no, wait for another frame
        rcall   IR_SIRC_OrderRegs           ;prepare registers to send
        rcall   IR_SendToCAN                ;yes, so send it to CAN
        call    SIRC_Frame_Repetition       ;wait for another frame
        tstfsz  WREG                        ;another frame began?
        bra     IR_ReceiveCodeExit          ;no, so exit
        bra     IR_ReceiveCode              ;yes, so start reading
IR_NEC
        call    NEC_Received
        tstfsz  WREG                        ;received code ok?
        bra     IR_ReceiveCodeExit          ;no, so exit
        rcall   IR_NEC_OrderRegs            ;prepare registers to send
        rcall   IR_SendToCAN                ;yes, so send it to CAN
        call    NEC_Frame_Repetition        ;wait for another frame
        tstfsz  WREG                        ;another frame began?
        bra     IR_ReceiveCodeExit          ;no, so exit
        bra     $ - .8                      ;yes, wait for another repetition
;--------------
IR_ReceiveCodeError
        call    RC5_Frame_Repetition        ;wait the longest repetition time
        tstfsz  WREG                        ;another frame began?
        bra     IR_ReceiveCodeExit          ;no, so exit
        bra     IR_ReceiveCode              ;yes, so start reading
;--------------
IR_ReceiveCodeExit
        call    IR_SendStopFrame            ;sends CAN frame indicating no IR signal
        call    EnAllInt                    ;enable all interrupts
    return

;------------------------------------------------------------------------------
IR_RC5_OrderRegs                            ;TYPE_ADDRESS_COMMAND_unused
        movff   RC5_TYPE,IRCODETYPE
        movff   RC5_ADDRESS,IRCODE0
        movff   RC5_COMMAND,IRCODE1
        setf    IRCODE2                     ;unused register
    return

;-------------------------------
IR_SIRC_OrderRegs
        movff   SIRC_TYPE,IRCODETYPE
        movlw   0x03                        ;12bit?
        xorwf   IRCODETYPE,W
        bz      IR_SIRC_12bit
        movlw   0x04                        ;15bit?
        xorwf   IRCODETYPE,W
        bz      IR_SIRC_15bit
        movlw   0x05                        ;20bit?
        xorwf   IRCODETYPE,W
        bz      IR_SIRC_20bit
    return
IR_SIRC_12bit                               ;TYPE_ADDRESS-L_COMMAND_unused
        movff   SIRC_ADDRESS_L,IRCODE0
        movff   SIRC_COMMAND,IRCODE1
        setf    IRCODE2                     ;unused register
    return
IR_SIRC_15bit                               ;TYPE_ADDRESS-L_COMMAND_unused
        movff   SIRC_ADDRESS_L,IRCODE0
        movff   SIRC_COMMAND,IRCODE1
        setf    IRCODE2                     ;unused register
    return
IR_SIRC_20bit                               ;TYPE_ADDRESS-H_ADDRESS-L_COMMAND
        movff   SIRC_ADDRESS_H,IRCODE0
        movff   SIRC_ADDRESS_L,IRCODE1
        movff   SIRC_COMMAND,IRCODE2
    return

;-------------------------------
IR_NEC_OrderRegs                            ;TYPE_ADDRESS_COMMAND_unused
        movff   NEC_TYPE,IRCODETYPE
        movlw   0x07                        ;standard frame (8 bit address)?
        xorwf   IRCODETYPE,W
        bz      IR_NEC_Std 
        movlw   0x08                        ;extended frame (16 bit address)?
        xorwf   IRCODETYPE,W
        bz      IR_NEC_Ext
    return
IR_NEC_Std                                  ;TYPE_ADDRESS-L_COMMAND_unused
        movff   NEC_TYPE,IRCODETYPE
        movff   NEC_ADDRESS_L,IRCODE0
        movff   NEC_COMMAND,IRCODE1
        setf    IRCODE2                     ;unused register
        bra     IR_NEC_OrderRegsExit
IR_NEC_Ext                                  ;TYPE_ADDRESS-H_ADDRESS-L_COMMAND
        movff   NEC_TYPE,IRCODETYPE
        movff   NEC_ADDRESS_H,IRCODE0
        movff   NEC_ADDRESS_L,IRCODE1
        movff   NEC_COMMAND,IRCODE2
IR_NEC_OrderRegsExit
        movff   IRCODETYPE,IRCODETYPE_S     ;the code cannot be received twice, so
        movff   IRCODE0,IRCODE0_S           ; prepare registers for IR_ReceivedTwice?
        movff   IRCODE1,IRCODE1_S           ; routine to pass
        movff   IRCODE2,IRCODE2_S
    return

;------------------------------------------------------------------------------
IR_SendToCAN
        rcall   IR_ReceivedTwice?           ;the code must be received twice to be sent
        tstfsz  WREG                        ;received twice?
    return                                  ;no it wasn't received twice, so exit
        rcall   IR_AlreadySent?             ;the code must be different than recently sent
        tstfsz  WREG                        ;code the same?
    return                                  ;yes, so exit
        movff   IRCODETYPE,TXFIFOIN6        ;ir code type
        movff   IRCODE0,TXFIFOIN7           ;ir code bits
        movff   IRCODE1,TXFIFOIN8
        movff   IRCODE2,TXFIFOIN9
        rcall   IR_SendCodeNow 
        ;indicate code as sent already
        movff   IRCODETYPE,IRCODETYPE_TX
        movff   IRCODE0,IRCODE0_TX
        movff   IRCODE1,IRCODE1_TX
        movff   IRCODE2,IRCODE2_TX
    return

;------------------------------------------------------------------------------
IR_ReceivedTwice?
        banksel IRCODETYPE
        movf    IRCODETYPE,W					
        cpfseq  IRCODETYPE_S                ;compare code type with saved
        bra     IR_ReceivedTwice_No         ;not the same
        movf    IRCODE0,W					
        cpfseq  IRCODE0_S                   ;compare bits with saved address
        bra     IR_ReceivedTwice_No         ;not the same
        movf    IRCODE1,W
        cpfseq  IRCODE1_S                   ;compare bits with saved command
        bra     IR_ReceivedTwice_No         ;not the same
        movf    IRCODE2,W
        cpfseq  IRCODE2_S                   ;compare bits with saved command
        bra     IR_ReceivedTwice_No         ;not the same
    retlw   0x00                            ;code is the same like last one
IR_ReceivedTwice_No                         ;save current code
        movff   IRCODETYPE,IRCODETYPE_S     ;move code type to saved code type register
        movff   IRCODE0,IRCODE0_S           ;move code bits to saved bits reg
        movff   IRCODE1,IRCODE1_S	
        movff   IRCODE2,IRCODE2_S
    retlw   0x01

;------------------------------------------------------------------------------
IR_AlreadySent?
        banksel IRCODETYPE
        movf    IRCODETYPE,W					
        cpfseq  IRCODETYPE_TX               ;compare code type with saved
        bra     IR_AlreadySent_No           ;not the same
        movf    IRCODE0,W
        cpfseq  IRCODE0_TX                  ;compare bits with already sent
        bra     IR_AlreadySent_No           ;not the same
        movf    IRCODE1,W
        cpfseq  IRCODE1_TX                  ;compare bits with already sent
        bra     IR_AlreadySent_No           ;not the same
        movf    IRCODE2,W
        cpfseq  IRCODE2_TX                  ;compare bits with already sent
        bra     IR_AlreadySent_No           ;not the same
    retlw   0x01                            ;the code was sent already
IR_AlreadySent_No
    retlw   0x00                            ;the code is different than recently sent

;------------------------------------------------------------------------------
IR_SendCodeNow                              ;send to CAN received infrared code
        banksel TXFIFOIN0
        movlw   0x30			            ;set IR RX frame
		movwf   TXFIFOIN0
        movlw   0x30
		movwf   TXFIFOIN1
		movff	NODENR,TXFIFOIN2            ;node id
		movff	GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        setf    TXFIFOIN5
        ;TXFIFOIN6-TXFIFOIN9 must be set in parent routine
		setf	TXFIFOIN10                  ;unused
		setf	TXFIFOIN11
		call	WriteToCanTxFIFO            ;put msg to TX FIFO 
        call    TransmitProcedure           ;check if any msg in TX FIFO and if so - transmit it
    return

;------------------------------------------------------------------------------
IR_SendStopFrame
        banksel IRCODETYPE
        btfsc   IRCODETYPE_TX,7             ;the stop code recently sent?
    return                                  ;yes, so exit
        bsf     IRCODETYPE_TX,7             ;means "stop frame"
        movff   IRCODETYPE_TX,TXFIFOIN6     ;ir code type
        movff   IRCODE0_TX,TXFIFOIN7        ;ir code bits
        movff   IRCODE1_TX,TXFIFOIN8
        movff   IRCODE2_TX,TXFIFOIN9
        rcall   IR_SendCodeNow
    return

;==============================================================================
;===  END  OF  PROGRAM  =======================================================
;==============================================================================
    END