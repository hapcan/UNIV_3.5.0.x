	NOLIST
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

;==============================================================================
;                         PROJECT REGISTER DEFINITIONS
;==============================================================================

R0          EQU     H'0000'     ;Status_LearntCode routine


;infrared code registers
IRCODEADDR  EQU     0x700       ;first byte of 256 byte block for IR code in RAM
IROUTPORT   EQU     PORTB       ;pin with IR transmitter
IROUTPIN    EQU     5
IRINPORT    EQU     PORTB       ;pin with IR receiver
IRINPIN     EQU     0
IRCODEREG0  EQU     H'0040'     ;reserved registers for IR routines
IRCODEREG1  EQU     H'0041'
IRCODEREG2  EQU     H'0042'
IRCODEREG3  EQU     H'0043'
IRCODEREG4  EQU     H'0044'
IRCODEREG5  EQU     H'0045'
IRCODEREG6  EQU     H'0046'
IRCODEREG7  EQU     H'0047'
IRCODEREG8  EQU     H'0048'
IRCODEREG9  EQU     H'0049'


;temporary saved registers
STATUS_HIGH EQU     H'0060'
WREG_HIGH   EQU     H'0061'
BSR_HIGH    EQU     H'0062'
FSR0L_HIGH  EQU     H'0063'
FSR0H_HIGH  EQU     H'0064'
FSR1L_HIGH  EQU     H'0065'
FSR1H_HIGH  EQU     H'0066'
STATUS_LOW  EQU     H'0067'
WREG_LOW    EQU     H'0068'
BSR_LOW     EQU     H'0069'
FSR0L_LOW   EQU     H'006A'
FSR0H_LOW   EQU     H'006B'
FSR1L_LOW   EQU     H'006C'
FSR1H_LOW   EQU     H'006D'


;==============================================================================
;                             ROUTINES REGISTERS
;==============================================================================
;------------------------------
;   FIFO registers
;------------------------------
;CAN RX FIFO input buffer
RXFIFOIN0   EQU		H'0070'     
RXFIFOIN1   EQU		H'0071'
RXFIFOIN2   EQU		H'0072'
RXFIFOIN3   EQU		H'0073'
RXFIFOIN4   EQU		H'0074'
RXFIFOIN5   EQU		H'0075'
RXFIFOIN6   EQU		H'0076'
RXFIFOIN7   EQU		H'0077'
RXFIFOIN8   EQU		H'0078'
RXFIFOIN9   EQU		H'0079'
RXFIFOIN10  EQU		H'007A'
RXFIFOIN11  EQU		H'007B'
;CAN RX FIFO output buffer
RXFIFO0     EQU		H'007C'     
RXFIFO1     EQU		H'007D'
RXFIFO2     EQU		H'007E'
RXFIFO3     EQU		H'007F'
RXFIFO4     EQU		H'0080'
RXFIFO5     EQU		H'0081'
RXFIFO6     EQU		H'0082'
RXFIFO7     EQU		H'0083'
RXFIFO8     EQU		H'0084'
RXFIFO9     EQU		H'0085'
RXFIFO10    EQU		H'0086'
RXFIFO11    EQU		H'0087'
;CAN TX FIFO input buffer
TXFIFOIN0   EQU		H'0088'     
TXFIFOIN1   EQU		H'0089'
TXFIFOIN2   EQU		H'008A'
TXFIFOIN3   EQU		H'008B'
TXFIFOIN4   EQU		H'008C'
TXFIFOIN5   EQU		H'008D'
TXFIFOIN6   EQU		H'008E'
TXFIFOIN7   EQU		H'008F'
TXFIFOIN8   EQU		H'0090'
TXFIFOIN9   EQU		H'0091'
TXFIFOIN10  EQU		H'0092'
TXFIFOIN11  EQU		H'0093'
;CAN TX FIFO output buffer
TXFIFO0     EQU		H'0094'     
TXFIFO1     EQU		H'0095'
TXFIFO2     EQU		H'0096'
TXFIFO3     EQU		H'0097'
TXFIFO4     EQU		H'0098'
TXFIFO5     EQU		H'0099'
TXFIFO6     EQU		H'009A'
TXFIFO7     EQU		H'009B'
TXFIFO8     EQU		H'009C'
TXFIFO9     EQU		H'009D'
TXFIFO10    EQU		H'009E'
TXFIFO11    EQU		H'009F'
;CAN FIFO RX registers
RXTOP       EQU     H'00A0'     ;FIFO RX stack top
RXCNT		EQU		H'00A1'     ;FIFO RX current stack depth
RXCNTMX		EQU		H'00A2'     ;FIFO RX maximum reached stack depth
RXFIFOR0	EQU		H'00A3'     ;working register
RXFIFOR1	EQU		H'00A4'     ;working register
;CAN FIFO TX registers
TXTOP       EQU     H'00A5'     ;FIFO RX stack top
TXCNT		EQU		H'00A6'     ;FIFO RX current stack depth
TXCNTMX		EQU		H'00A7'     ;FIFO RX maximum reached stack depth
TXFIFOR0	EQU		H'00A8'     ;working register
TXFIFOR1	EQU		H'00A9'     ;working register


;------------------------------
;   HEALTH CHECK REGISTERS
;------------------------------
CANINTCNT   EQU     H'00AC'     ;counts can reinitialization events
TRSMTTIMER  EQU     H'00AD'     ;counts seconds when transmit buffers are busy

;------------------------------
;   WORKING REGISTERS
;------------------------------
ROUT0       EQU     H'00B8'     ;CanTransmit routine
ROUT1       EQU     H'00B9'     ;ProcessSystemMessage routine (HealthAskClear)
ROUT2       EQU     H'00BA'     ;CheckAgainstBoxes routine

;------------------------------
;   UPTIME REGISTERS
;------------------------------
UPTIME0     EQU     H'00BC'
UPTIME1     EQU     H'00BD'
UPTIME2     EQU     H'00BE'
UPTIME3     EQU     H'00BF'

;------------------------------
;   BOX REGISTERS
;------------------------------
BOXADR      EQU     H'00D0'
BOXEN       EQU     H'00D1'
BOXFIL1     EQU     H'00E0'     ;message filter
BOXFIL2     EQU     H'00E1'
BOXFIL3     EQU     H'00E2'
BOXFIL4     EQU     H'00E3'
BOXFIL5     EQU     H'00E4'
BOXFIL6     EQU     H'00E5'
BOXFIL7     EQU     H'00E6'
BOXFIL8     EQU     H'00E7'
BOXFIL9     EQU     H'00E8'
BOXFIL10    EQU     H'00E9'
BOXFIL11    EQU     H'00EA'
BOXFIL12    EQU     H'00EB'
BOXFIL1C    EQU     H'00EC'     ;message filter condition
BOXFIL2C    EQU     H'00ED'
BOXFIL3C    EQU     H'00EE'
BOXFIL4C    EQU     H'00EF'
BOXFIL5C    EQU     H'00F0'
BOXFIL6C    EQU     H'00F1'
BOXFIL7C    EQU     H'00F2'
BOXFIL8C    EQU     H'00F3'
BOXFIL9C    EQU     H'00F4'
BOXFIL10C   EQU     H'00F5'
BOXFIL11C   EQU     H'00F6'
BOXFIL12C   EQU     H'00F7'
BOXINSTR1   EQU     H'00F8'     ;instruction in box
BOXINSTR2   EQU     H'00F9'
BOXINSTR3   EQU     H'00FA'
BOXINSTR4   EQU     H'00FB'
BOXINSTR5   EQU     H'00FC'
BOXINSTR6   EQU     H'00FD'
BOXINSTR7   EQU     H'00FE'
BOXINSTR8   EQU     H'00FF'
;saved instruction registers
INSTR1		EQU		H'0050'
INSTR2		EQU		H'0051'
INSTR3		EQU		H'0052'
INSTR4		EQU		H'0053'
INSTR5		EQU		H'0054'
INSTR6		EQU		H'0055'
INSTR7		EQU		H'0056'
INSTR8		EQU		H'0057'
;TIMER		EQU		H'0058'
;SUBTIMER1	EQU		H'0059'
;SUBTIMER2	EQU		H'005A'
;------------------------------
;   CHANNEL REGISTERS
;------------------------------
Instr1Ch1	EQU		H'00D0'
Instr2Ch1	EQU		H'00D1'
Instr3Ch1	EQU		H'00D2'
TimerCh1	EQU		H'00D3'
SubTmr1Ch1	EQU		H'00D4'
SubTmr2Ch1	EQU		H'00D5'
Instr1Ch2	EQU		H'00D6'
Instr2Ch2	EQU		H'00D7'
Instr3Ch2	EQU		H'00D8'
TimerCh2	EQU		H'00D9'
SubTmr1Ch2	EQU		H'00DA'
SubTmr2Ch2	EQU		H'00DB'
Instr1Ch3	EQU		H'00DC'
Instr2Ch3	EQU		H'00DD'
Instr3Ch3	EQU		H'00DE'
TimerCh3	EQU		H'00DF'
SubTmr1Ch3	EQU		H'00E0'
SubTmr2Ch3	EQU		H'00E1'
Instr1Ch4	EQU		H'00E2'
Instr2Ch4	EQU		H'00E3'
Instr3Ch4	EQU		H'00E4'
TimerCh4	EQU		H'00E5'
SubTmr1Ch4	EQU		H'00E6'
SubTmr2Ch4	EQU		H'00E7'
Instr1Ch5	EQU		H'00E8'
Instr2Ch5	EQU		H'00E9'
Instr3Ch5	EQU		H'00EA'
TimerCh5	EQU		H'00EB'
SubTmr1Ch5	EQU		H'00EC'
SubTmr2Ch5	EQU		H'00ED'
Instr1Ch6	EQU		H'00EE'
Instr2Ch6	EQU		H'00EF'
Instr3Ch6	EQU		H'00F0'
TimerCh6	EQU		H'00F1'
SubTmr1Ch6	EQU		H'00F2'
SubTmr2Ch6	EQU		H'00F3'
Instr1Ch7	EQU		H'00F4'
Instr2Ch7	EQU		H'00F5'
Instr3Ch7	EQU		H'00F6'
TimerCh7	EQU		H'00F7'
SubTmr1Ch7	EQU		H'00F8'
SubTmr2Ch7	EQU		H'00F9'
Instr1Ch8	EQU		H'00FA'
Instr2Ch8	EQU		H'00FB'
Instr3Ch8	EQU		H'00FC'
TimerCh8	EQU		H'00FD'
SubTmr1Ch8	EQU		H'00FE'
SubTmr2Ch8	EQU		H'00FF'

;==============================================================================
;                             COPPIED EEPROM
;==============================================================================
;eeprom module config
CONFIG0     EQU		H'0208'
CONFIG1     EQU		H'0209'
CONFIG2     EQU		H'020A'
CONFIG3     EQU		H'020B'
CONFIG4     EQU		H'020C'
CONFIG5     EQU		H'020D'
CONFIG6     EQU		H'020E'
CONFIG7     EQU		H'020F'
CONFIG8     EQU		H'0210'
CONFIG9     EQU		H'0211'
CONFIG10    EQU		H'0212'
CONFIG11    EQU		H'0213'
CONFIG12    EQU		H'0214'
CONFIG13    EQU		H'0215'
CONFIG14    EQU		H'0216'
CONFIG15    EQU		H'0217'
CONFIG16    EQU		H'0218'
CONFIG17    EQU		H'0219'
CONFIG18    EQU		H'021A'
CONFIG19    EQU		H'021B'
CONFIG20    EQU		H'021C'
CONFIG21    EQU		H'021D'
CONFIG22    EQU		H'021E'
CONFIG23    EQU		H'021F'
;module id
NODENR      EQU     H'0226'     ;node number
GROUPNR     EQU     H'0227'     ;node group number
;health check regs
;res    	EQU		H'0228'
;res	    EQU		H'0229'
;res    	EQU		H'022A'
RXCNTMXEE	EQU		H'022B'
TXCNTMXEE	EQU		H'022C'
CANINTCNTEE EQU     H'022D' 
RXERRCNTEE	EQU		H'022E'
TXERRCNTEE  EQU		H'022F'
;enable box registers
ENBOX08		EQU		H'0240'     ;boxes 1-8
ENBOX16		EQU		H'0241'
ENBOX24		EQU		H'0242'
ENBOX32		EQU		H'0243'
ENBOX40		EQU		H'0244'
ENBOX48		EQU		H'0245'
ENBOX56		EQU		H'0246'
ENBOX64		EQU		H'0247'
ENBOX72		EQU		H'0248'
ENBOX80		EQU		H'0249'
ENBOX88		EQU		H'024A'
ENBOX96		EQU		H'024B'
ENBOX104	EQU		H'024C'
ENBOX112	EQU		H'024D'
ENBOX120	EQU		H'024E'
ENBOX128	EQU		H'024F'     ;boxes 121-128


;==============================================================================
;                   BOOTLOADER REGISTERS - DO NOT CHANGE!!!
;==============================================================================
;CAN receive buffer
RxBCON      EQU		H'0100'
CANFRAME1   EQU		H'0101'
CANFRAME2   EQU		H'0102'
CANNODE     EQU		H'0103'
CANGROUP    EQU		H'0104'
CANDLC      EQU		H'0105'
CAND0       EQU		H'0106'
CAND1       EQU		H'0107'
CAND2       EQU		H'0108'
CAND3       EQU		H'0109'
CAND4       EQU		H'010A'
CAND5       EQU		H'010B'
CAND6       EQU		H'010C'
CAND7       EQU		H'010D'
CANFULL     EQU     H'010E'     ;CAN buffer full flag
FIRMREADY   EQU     H'010F'     ;firmware started completely flag
;------------------------------
;UART receive buffer
UART0       EQU		H'0110'
UART1       EQU		H'0111'
UART2       EQU		H'0112'
UART3       EQU		H'0113'
UART4       EQU		H'0114'
UART5       EQU		H'0115'
UART6       EQU		H'0116'
UART7       EQU		H'0117'
UART8       EQU		H'0118'
UART9       EQU		H'0119'
UART10      EQU		H'011A'
UART11      EQU		H'011B'
UART12      EQU		H'011C'
UART13      EQU		H'011D'
UART14      EQU		H'011E'
UART15      EQU		H'011F'
UART16      EQU		H'0120'
UART17      EQU		H'0121'
UART18      EQU		H'0122'
UART19      EQU		H'0123'
UART20      EQU		H'0124'
UART21      EQU		H'0125'
UART22      EQU		H'0126'
UART23      EQU		H'0127'
UART24      EQU		H'0128'
UART25      EQU		H'0129'
UART26      EQU		H'012A'
UART27      EQU		H'012B'
UART28      EQU		H'012C'
UART29      EQU		H'012D'
UARTOVF     EQU		H'012E'     ;overflow indicator
UARTCNT     EQU		H'012F'     ;uart buffer counter
;------------------------------
;bootloader other regs
UARTON      EQU		H'016F'     ;indicates that UART bootloader is on when 0xFF
;------------------------------
;bootloader shadow registers for high interrupt
STATUS_H    EQU     H'0170'
WREG_H      EQU     H'0171'
BSR_H       EQU     H'0172'
PIR1_H      EQU     H'0173'
PIR4_H      EQU     H'0174'
PIR5_H      EQU     H'0175'
CANSTAT_H   EQU     H'0176'
CANCON_H    EQU     H'0177'
FSR0L_H     EQU		H'0178'
FSR0H_H     EQU		H'0179'
FSR1L_H     EQU		H'017A'
FSR1H_H     EQU		H'017B'
TABLAT_H    EQU		H'017C'
TBLPTRL_H   EQU		H'017D'
TBLPTRH_H   EQU		H'017E'
TBLPTRU_H   EQU		H'017F'
INTCON_H    EQU		H'0180'
EECON1_H    EQU		H'0181'
EEDATA_H    EQU		H'0182'
EEADRH_H    EQU		H'0183'
EEADR_H     EQU		H'0184'
TRISA_H     EQU		H'0185'
ADCON2_H    EQU     H'0186'
ADCON1_H    EQU     H'0187'
ADCON0_H    EQU     H'0188'
ANCON0_H    EQU     H'0189'
ADRESL_H    EQU     H'018A'
ADRESH_H    EQU     H'018B'
RCSTA1_H    EQU     H'018C'
TXSTA1_H    EQU     H'018D'
PMD1_H      EQU     H'018E'
COMSTAT_H   EQU     H'018F'
;------------------------------
;bootloader shadow registers for low interrupt
STATUS_L    EQU     H'0190'
WREG_L      EQU     H'0191'
BSR_L       EQU     H'0192'
PIR1_L      EQU     H'0193'
PIR4_L      EQU     H'0194'
PIR5_L      EQU     H'0195'
CANSTAT_L   EQU     H'0196'
CANCON_L    EQU     H'0197'
FSR0L_L     EQU		H'0198'
FSR0H_L     EQU		H'0199'
FSR1L_L     EQU		H'019A'
FSR1H_L     EQU		H'019B'
TABLAT_L    EQU		H'019C'
TBLPTRL_L   EQU		H'019D'
TBLPTRH_L   EQU		H'019E'
TBLPTRU_L   EQU		H'019F'
INTCON_L    EQU		H'01A0'
EECON1_L    EQU		H'01A1'
EEDATA_L    EQU		H'01A2'
EEADRH_L    EQU		H'01A3'
EEADR_L     EQU		H'01A4'
TRISA_L     EQU		H'01A5'
ADCON2_L    EQU     H'01A6'
ADCON1_L    EQU     H'01A7'
ADCON0_L    EQU     H'01A8'
ANCON0_L    EQU     H'01A9'
ADRESL_L    EQU     H'01AA'
ADRESH_L    EQU     H'01AB'
RCSTA1_L    EQU     H'01AC'
TXSTA1_L    EQU     H'01AD'
PMD1_L      EQU     H'01AE'
COMSTAT_L   EQU     H'01AF'
;==============================================================================
;==============================================================================
;==============================================================================
    LIST
