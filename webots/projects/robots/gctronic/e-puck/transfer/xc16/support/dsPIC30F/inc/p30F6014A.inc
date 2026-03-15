;==========================================================================
;  PIC30F6014A Standard Assembly Include File
; 
;  (c) Copyright 2014 Microchip Technology, All rights reserved
;  Part support version v1.24.A(02-Jan-2015)
; 
 ; This software is developed by Microchip Technology Inc. and its
 ; subsidiaries ("Microchip").
 ; 
 ; Redistribution and use in source and binary forms, with or without
 ; modification, are permitted provided that the following conditions are 
 ; met:
 ; 
 ; 1.      Redistributions of source code must retain the above copyright
 ;         notice, this list of conditions and the following disclaimer.
 ; 2.      Redistributions in binary form must reproduce the above 
 ;         copyright notice, this list of conditions and the following 
 ;         disclaimer in the documentation and/or other materials provided 
 ;         with the distribution.
 ; 3.      Microchip's name may not be used to endorse or promote products
 ;         derived from this software without specific prior written 
 ;         permission.
 ; 
 ; THIS SOFTWARE IS PROVIDED BY MICROCHIP "AS IS" AND ANY EXPRESS OR IMPLIED
 ; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ; MERCHANTABILITY AND FITNESS FOR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 ; SHALL MICROCHIP BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 ; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT LIMITED TO
 ; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA OR PROFITS;
 ; OR BUSINESS INTERRUPTION) HOWSOEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 ; WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 ; OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ; ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ; 
 ;

.NOLIST

;==========================================================================
;  This header file defines configurations, registers, and other useful
;  bits of information for the PIC30F6014A microcontroller.  These names
;  are taken to match the data sheets as closely as possible.
;
;  Note that the processor must be selected before this file is included.
;  The processor may be selected the following ways:
;
;       1. Command line switch:
;               C:\> pic30-as file.s -p30F6014A
;       2. Placing a ".equ" directive before the ".include":
;               .equ __30F6014A, 1
;               .include "p30F6014A.inc"
;       3. Setting the processor in the MPLAB IDE Project Dialog
;==========================================================================

;==========================================================================
;
;       Verify Processor
;
;==========================================================================

.IFNDEF __30F6014A
   .ERROR "Include file does not match processor setting."
.ENDIF


;==========================================================================
;
;       Register Definitions
;
;==========================================================================

;----- Register Equates (Low Byte, High Byte) -----------------------------

        .extern WREG0
        .equiv  WREG0L, _WREG0
        .equiv  WREG0H, _WREG0+1

        .extern WREG1
        .equiv  WREG1L, _WREG1
        .equiv  WREG1H, _WREG1+1

        .extern WREG2
        .equiv  WREG2L, _WREG2
        .equiv  WREG2H, _WREG2+1

        .extern WREG3
        .equiv  WREG3L, _WREG3
        .equiv  WREG3H, _WREG3+1

        .extern WREG4
        .equiv  WREG4L, _WREG4
        .equiv  WREG4H, _WREG4+1

        .extern WREG5
        .equiv  WREG5L, _WREG5
        .equiv  WREG5H, _WREG5+1

        .extern WREG6
        .equiv  WREG6L, _WREG6
        .equiv  WREG6H, _WREG6+1

        .extern WREG7
        .equiv  WREG7L, _WREG7
        .equiv  WREG7H, _WREG7+1

        .extern WREG8
        .equiv  WREG8L, _WREG8
        .equiv  WREG8H, _WREG8+1

        .extern WREG9
        .equiv  WREG9L, _WREG9
        .equiv  WREG9H, _WREG9+1

        .extern WREG10
        .equiv  WREG10L, _WREG10
        .equiv  WREG10H, _WREG10+1

        .extern WREG11
        .equiv  WREG11L, _WREG11
        .equiv  WREG11H, _WREG11+1

        .extern WREG12
        .equiv  WREG12L, _WREG12
        .equiv  WREG12H, _WREG12+1

        .extern WREG13
        .equiv  WREG13L, _WREG13
        .equiv  WREG13H, _WREG13+1

        .extern WREG14
        .equiv  WREG14L, _WREG14
        .equiv  WREG14H, _WREG14+1

        .extern WREG15
        .equiv  WREG15L, _WREG15
        .equiv  WREG15H, _WREG15+1

        .extern SPLIM
        .equiv  SPLIML, _SPLIM
        .equiv  SPLIMH, _SPLIM+1

        .extern ACCAL
        .equiv  ACCALL, _ACCAL
        .equiv  ACCALH, _ACCAL+1

        .extern ACCAH
        .equiv  ACCAHL, _ACCAH
        .equiv  ACCAHH, _ACCAH+1

        .extern ACCAU
        .equiv  ACCAUL, _ACCAU
        .equiv  ACCAUH, _ACCAU+1

        .extern ACCBL
        .equiv  ACCBLL, _ACCBL
        .equiv  ACCBLH, _ACCBL+1

        .extern ACCBH
        .equiv  ACCBHL, _ACCBH
        .equiv  ACCBHH, _ACCBH+1

        .extern ACCBU
        .equiv  ACCBUL, _ACCBU
        .equiv  ACCBUH, _ACCBU+1

        .extern PCL
        .equiv  PCLL, _PCL
        .equiv  PCLH, _PCL+1

        .extern PCH
        .equiv  PCHL, _PCH
        .equiv  PCHH, _PCH+1

        .extern TBLPAG
        .equiv  TBLPAGL, _TBLPAG
        .equiv  TBLPAGH, _TBLPAG+1

        .extern PSVPAG
        .equiv  PSVPAGL, _PSVPAG
        .equiv  PSVPAGH, _PSVPAG+1

        .extern RCOUNT
        .equiv  RCOUNTL, _RCOUNT
        .equiv  RCOUNTH, _RCOUNT+1

        .extern DCOUNT
        .equiv  DCOUNTL, _DCOUNT
        .equiv  DCOUNTH, _DCOUNT+1

        .extern DOSTARTL
        .equiv  DOSTARTLL, _DOSTARTL
        .equiv  DOSTARTLH, _DOSTARTL+1

        .extern DOSTARTH
        .equiv  DOSTARTHL, _DOSTARTH
        .equiv  DOSTARTHH, _DOSTARTH+1

        .extern DOENDL
        .equiv  DOENDLL, _DOENDL
        .equiv  DOENDLH, _DOENDL+1

        .extern DOENDH
        .equiv  DOENDHL, _DOENDH
        .equiv  DOENDHH, _DOENDH+1

        .extern SR
        .equiv  SRL, _SR
        .equiv  SRH, _SR+1

        .extern CORCON
        .equiv  CORCONL, _CORCON
        .equiv  CORCONH, _CORCON+1

        .extern MODCON
        .equiv  MODCONL, _MODCON
        .equiv  MODCONH, _MODCON+1

        .extern XMODSRT
        .equiv  XMODSRTL, _XMODSRT
        .equiv  XMODSRTH, _XMODSRT+1

        .extern XMODEND
        .equiv  XMODENDL, _XMODEND
        .equiv  XMODENDH, _XMODEND+1

        .extern YMODSRT
        .equiv  YMODSRTL, _YMODSRT
        .equiv  YMODSRTH, _YMODSRT+1

        .extern YMODEND
        .equiv  YMODENDL, _YMODEND
        .equiv  YMODENDH, _YMODEND+1

        .extern XBREV
        .equiv  XBREVL, _XBREV
        .equiv  XBREVH, _XBREV+1

        .extern DISICNT
        .equiv  DISICNTL, _DISICNT
        .equiv  DISICNTH, _DISICNT+1

        .extern INTCON1
        .equiv  INTCON1L, _INTCON1
        .equiv  INTCON1H, _INTCON1+1

        .extern INTCON2
        .equiv  INTCON2L, _INTCON2
        .equiv  INTCON2H, _INTCON2+1

        .extern IFS0
        .equiv  IFS0L, _IFS0
        .equiv  IFS0H, _IFS0+1

        .extern IFS1
        .equiv  IFS1L, _IFS1
        .equiv  IFS1H, _IFS1+1

        .extern IFS2
        .equiv  IFS2L, _IFS2
        .equiv  IFS2H, _IFS2+1

        .extern IEC0
        .equiv  IEC0L, _IEC0
        .equiv  IEC0H, _IEC0+1

        .extern IEC1
        .equiv  IEC1L, _IEC1
        .equiv  IEC1H, _IEC1+1

        .extern IEC2
        .equiv  IEC2L, _IEC2
        .equiv  IEC2H, _IEC2+1

        .extern IPC0
        .equiv  IPC0L, _IPC0
        .equiv  IPC0H, _IPC0+1

        .extern IPC1
        .equiv  IPC1L, _IPC1
        .equiv  IPC1H, _IPC1+1

        .extern IPC2
        .equiv  IPC2L, _IPC2
        .equiv  IPC2H, _IPC2+1

        .extern IPC3
        .equiv  IPC3L, _IPC3
        .equiv  IPC3H, _IPC3+1

        .extern IPC4
        .equiv  IPC4L, _IPC4
        .equiv  IPC4H, _IPC4+1

        .extern IPC5
        .equiv  IPC5L, _IPC5
        .equiv  IPC5H, _IPC5+1

        .extern IPC6
        .equiv  IPC6L, _IPC6
        .equiv  IPC6H, _IPC6+1

        .extern IPC7
        .equiv  IPC7L, _IPC7
        .equiv  IPC7H, _IPC7+1

        .extern IPC8
        .equiv  IPC8L, _IPC8
        .equiv  IPC8H, _IPC8+1

        .extern IPC9
        .equiv  IPC9L, _IPC9
        .equiv  IPC9H, _IPC9+1

        .extern IPC10
        .equiv  IPC10L, _IPC10
        .equiv  IPC10H, _IPC10+1

        .extern INTTREG
        .equiv  INTTREGL, _INTTREG
        .equiv  INTTREGH, _INTTREG+1

        .extern CNEN1
        .equiv  CNEN1L, _CNEN1
        .equiv  CNEN1H, _CNEN1+1

        .extern CNEN2
        .equiv  CNEN2L, _CNEN2
        .equiv  CNEN2H, _CNEN2+1

        .extern CNPU1
        .equiv  CNPU1L, _CNPU1
        .equiv  CNPU1H, _CNPU1+1

        .extern CNPU2
        .equiv  CNPU2L, _CNPU2
        .equiv  CNPU2H, _CNPU2+1

        .extern TMR1
        .equiv  TMR1L, _TMR1
        .equiv  TMR1H, _TMR1+1

        .extern PR1
        .equiv  PR1L, _PR1
        .equiv  PR1H, _PR1+1

        .extern T1CON
        .equiv  T1CONL, _T1CON
        .equiv  T1CONH, _T1CON+1

        .extern TMR2
        .equiv  TMR2L, _TMR2
        .equiv  TMR2H, _TMR2+1

        .extern TMR3HLD
        .equiv  TMR3HLDL, _TMR3HLD
        .equiv  TMR3HLDH, _TMR3HLD+1

        .extern TMR3
        .equiv  TMR3L, _TMR3
        .equiv  TMR3H, _TMR3+1

        .extern PR2
        .equiv  PR2L, _PR2
        .equiv  PR2H, _PR2+1

        .extern PR3
        .equiv  PR3L, _PR3
        .equiv  PR3H, _PR3+1

        .extern T2CON
        .equiv  T2CONL, _T2CON
        .equiv  T2CONH, _T2CON+1

        .extern T3CON
        .equiv  T3CONL, _T3CON
        .equiv  T3CONH, _T3CON+1

        .extern TMR4
        .equiv  TMR4L, _TMR4
        .equiv  TMR4H, _TMR4+1

        .extern TMR5HLD
        .equiv  TMR5HLDL, _TMR5HLD
        .equiv  TMR5HLDH, _TMR5HLD+1

        .extern TMR5
        .equiv  TMR5L, _TMR5
        .equiv  TMR5H, _TMR5+1

        .extern PR4
        .equiv  PR4L, _PR4
        .equiv  PR4H, _PR4+1

        .extern PR5
        .equiv  PR5L, _PR5
        .equiv  PR5H, _PR5+1

        .extern T4CON
        .equiv  T4CONL, _T4CON
        .equiv  T4CONH, _T4CON+1

        .extern T5CON
        .equiv  T5CONL, _T5CON
        .equiv  T5CONH, _T5CON+1

        .extern IC1BUF
        .equiv  IC1BUFL, _IC1BUF
        .equiv  IC1BUFH, _IC1BUF+1

        .extern IC1CON
        .equiv  IC1CONL, _IC1CON
        .equiv  IC1CONH, _IC1CON+1

        .extern IC2BUF
        .equiv  IC2BUFL, _IC2BUF
        .equiv  IC2BUFH, _IC2BUF+1

        .extern IC2CON
        .equiv  IC2CONL, _IC2CON
        .equiv  IC2CONH, _IC2CON+1

        .extern IC3BUF
        .equiv  IC3BUFL, _IC3BUF
        .equiv  IC3BUFH, _IC3BUF+1

        .extern IC3CON
        .equiv  IC3CONL, _IC3CON
        .equiv  IC3CONH, _IC3CON+1

        .extern IC4BUF
        .equiv  IC4BUFL, _IC4BUF
        .equiv  IC4BUFH, _IC4BUF+1

        .extern IC4CON
        .equiv  IC4CONL, _IC4CON
        .equiv  IC4CONH, _IC4CON+1

        .extern IC5BUF
        .equiv  IC5BUFL, _IC5BUF
        .equiv  IC5BUFH, _IC5BUF+1

        .extern IC5CON
        .equiv  IC5CONL, _IC5CON
        .equiv  IC5CONH, _IC5CON+1

        .extern IC6BUF
        .equiv  IC6BUFL, _IC6BUF
        .equiv  IC6BUFH, _IC6BUF+1

        .extern IC6CON
        .equiv  IC6CONL, _IC6CON
        .equiv  IC6CONH, _IC6CON+1

        .extern IC7BUF
        .equiv  IC7BUFL, _IC7BUF
        .equiv  IC7BUFH, _IC7BUF+1

        .extern IC7CON
        .equiv  IC7CONL, _IC7CON
        .equiv  IC7CONH, _IC7CON+1

        .extern IC8BUF
        .equiv  IC8BUFL, _IC8BUF
        .equiv  IC8BUFH, _IC8BUF+1

        .extern IC8CON
        .equiv  IC8CONL, _IC8CON
        .equiv  IC8CONH, _IC8CON+1

        .extern OC1RS
        .equiv  OC1RSL, _OC1RS
        .equiv  OC1RSH, _OC1RS+1

        .extern OC1R
        .equiv  OC1RL, _OC1R
        .equiv  OC1RH, _OC1R+1

        .extern OC1CON
        .equiv  OC1CONL, _OC1CON
        .equiv  OC1CONH, _OC1CON+1

        .extern OC2RS
        .equiv  OC2RSL, _OC2RS
        .equiv  OC2RSH, _OC2RS+1

        .extern OC2R
        .equiv  OC2RL, _OC2R
        .equiv  OC2RH, _OC2R+1

        .extern OC2CON
        .equiv  OC2CONL, _OC2CON
        .equiv  OC2CONH, _OC2CON+1

        .extern OC3RS
        .equiv  OC3RSL, _OC3RS
        .equiv  OC3RSH, _OC3RS+1

        .extern OC3R
        .equiv  OC3RL, _OC3R
        .equiv  OC3RH, _OC3R+1

        .extern OC3CON
        .equiv  OC3CONL, _OC3CON
        .equiv  OC3CONH, _OC3CON+1

        .extern OC4RS
        .equiv  OC4RSL, _OC4RS
        .equiv  OC4RSH, _OC4RS+1

        .extern OC4R
        .equiv  OC4RL, _OC4R
        .equiv  OC4RH, _OC4R+1

        .extern OC4CON
        .equiv  OC4CONL, _OC4CON
        .equiv  OC4CONH, _OC4CON+1

        .extern OC5RS
        .equiv  OC5RSL, _OC5RS
        .equiv  OC5RSH, _OC5RS+1

        .extern OC5R
        .equiv  OC5RL, _OC5R
        .equiv  OC5RH, _OC5R+1

        .extern OC5CON
        .equiv  OC5CONL, _OC5CON
        .equiv  OC5CONH, _OC5CON+1

        .extern OC6RS
        .equiv  OC6RSL, _OC6RS
        .equiv  OC6RSH, _OC6RS+1

        .extern OC6R
        .equiv  OC6RL, _OC6R
        .equiv  OC6RH, _OC6R+1

        .extern OC6CON
        .equiv  OC6CONL, _OC6CON
        .equiv  OC6CONH, _OC6CON+1

        .extern OC7RS
        .equiv  OC7RSL, _OC7RS
        .equiv  OC7RSH, _OC7RS+1

        .extern OC7R
        .equiv  OC7RL, _OC7R
        .equiv  OC7RH, _OC7R+1

        .extern OC7CON
        .equiv  OC7CONL, _OC7CON
        .equiv  OC7CONH, _OC7CON+1

        .extern OC8RS
        .equiv  OC8RSL, _OC8RS
        .equiv  OC8RSH, _OC8RS+1

        .extern OC8R
        .equiv  OC8RL, _OC8R
        .equiv  OC8RH, _OC8R+1

        .extern OC8CON
        .equiv  OC8CONL, _OC8CON
        .equiv  OC8CONH, _OC8CON+1

        .extern I2CRCV
        .equiv  I2CRCVL, _I2CRCV
        .equiv  I2CRCVH, _I2CRCV+1

        .extern I2CTRN
        .equiv  I2CTRNL, _I2CTRN
        .equiv  I2CTRNH, _I2CTRN+1

        .extern I2CBRG
        .equiv  I2CBRGL, _I2CBRG
        .equiv  I2CBRGH, _I2CBRG+1

        .extern I2CCON
        .equiv  I2CCONL, _I2CCON
        .equiv  I2CCONH, _I2CCON+1

        .extern I2CSTAT
        .equiv  I2CSTATL, _I2CSTAT
        .equiv  I2CSTATH, _I2CSTAT+1

        .extern I2CADD
        .equiv  I2CADDL, _I2CADD
        .equiv  I2CADDH, _I2CADD+1

        .extern U1MODE
        .equiv  U1MODEL, _U1MODE
        .equiv  U1MODEH, _U1MODE+1

        .extern U1STA
        .equiv  U1STAL, _U1STA
        .equiv  U1STAH, _U1STA+1

        .extern U1TXREG
        .equiv  U1TXREGL, _U1TXREG
        .equiv  U1TXREGH, _U1TXREG+1

        .extern U1RXREG
        .equiv  U1RXREGL, _U1RXREG
        .equiv  U1RXREGH, _U1RXREG+1

        .extern U1BRG
        .equiv  U1BRGL, _U1BRG
        .equiv  U1BRGH, _U1BRG+1

        .extern U2MODE
        .equiv  U2MODEL, _U2MODE
        .equiv  U2MODEH, _U2MODE+1

        .extern U2STA
        .equiv  U2STAL, _U2STA
        .equiv  U2STAH, _U2STA+1

        .extern U2TXREG
        .equiv  U2TXREGL, _U2TXREG
        .equiv  U2TXREGH, _U2TXREG+1

        .extern U2RXREG
        .equiv  U2RXREGL, _U2RXREG
        .equiv  U2RXREGH, _U2RXREG+1

        .extern U2BRG
        .equiv  U2BRGL, _U2BRG
        .equiv  U2BRGH, _U2BRG+1

        .extern SPI1STAT
        .equiv  SPI1STATL, _SPI1STAT
        .equiv  SPI1STATH, _SPI1STAT+1

        .extern SPI1CON
        .equiv  SPI1CONL, _SPI1CON
        .equiv  SPI1CONH, _SPI1CON+1

        .extern SPI1BUF
        .equiv  SPI1BUFL, _SPI1BUF
        .equiv  SPI1BUFH, _SPI1BUF+1

        .extern SPI2STAT
        .equiv  SPI2STATL, _SPI2STAT
        .equiv  SPI2STATH, _SPI2STAT+1

        .extern SPI2CON
        .equiv  SPI2CONL, _SPI2CON
        .equiv  SPI2CONH, _SPI2CON+1

        .extern SPI2BUF
        .equiv  SPI2BUFL, _SPI2BUF
        .equiv  SPI2BUFH, _SPI2BUF+1

        .extern DCICON1
        .equiv  DCICON1L, _DCICON1
        .equiv  DCICON1H, _DCICON1+1

        .extern DCICON2
        .equiv  DCICON2L, _DCICON2
        .equiv  DCICON2H, _DCICON2+1

        .extern DCICON3
        .equiv  DCICON3L, _DCICON3
        .equiv  DCICON3H, _DCICON3+1

        .extern DCISTAT
        .equiv  DCISTATL, _DCISTAT
        .equiv  DCISTATH, _DCISTAT+1

        .extern TSCON
        .equiv  TSCONL, _TSCON
        .equiv  TSCONH, _TSCON+1

        .extern RSCON
        .equiv  RSCONL, _RSCON
        .equiv  RSCONH, _RSCON+1

        .extern RXBUF0
        .equiv  RXBUF0L, _RXBUF0
        .equiv  RXBUF0H, _RXBUF0+1

        .extern RXBUF1
        .equiv  RXBUF1L, _RXBUF1
        .equiv  RXBUF1H, _RXBUF1+1

        .extern RXBUF2
        .equiv  RXBUF2L, _RXBUF2
        .equiv  RXBUF2H, _RXBUF2+1

        .extern RXBUF3
        .equiv  RXBUF3L, _RXBUF3
        .equiv  RXBUF3H, _RXBUF3+1

        .extern TXBUF0
        .equiv  TXBUF0L, _TXBUF0
        .equiv  TXBUF0H, _TXBUF0+1

        .extern TXBUF1
        .equiv  TXBUF1L, _TXBUF1
        .equiv  TXBUF1H, _TXBUF1+1

        .extern TXBUF2
        .equiv  TXBUF2L, _TXBUF2
        .equiv  TXBUF2H, _TXBUF2+1

        .extern TXBUF3
        .equiv  TXBUF3L, _TXBUF3
        .equiv  TXBUF3H, _TXBUF3+1

        .extern ADCBUF0
        .equiv  ADCBUF0L, _ADCBUF0
        .equiv  ADCBUF0H, _ADCBUF0+1

        .extern ADCBUF1
        .equiv  ADCBUF1L, _ADCBUF1
        .equiv  ADCBUF1H, _ADCBUF1+1

        .extern ADCBUF2
        .equiv  ADCBUF2L, _ADCBUF2
        .equiv  ADCBUF2H, _ADCBUF2+1

        .extern ADCBUF3
        .equiv  ADCBUF3L, _ADCBUF3
        .equiv  ADCBUF3H, _ADCBUF3+1

        .extern ADCBUF4
        .equiv  ADCBUF4L, _ADCBUF4
        .equiv  ADCBUF4H, _ADCBUF4+1

        .extern ADCBUF5
        .equiv  ADCBUF5L, _ADCBUF5
        .equiv  ADCBUF5H, _ADCBUF5+1

        .extern ADCBUF6
        .equiv  ADCBUF6L, _ADCBUF6
        .equiv  ADCBUF6H, _ADCBUF6+1

        .extern ADCBUF7
        .equiv  ADCBUF7L, _ADCBUF7
        .equiv  ADCBUF7H, _ADCBUF7+1

        .extern ADCBUF8
        .equiv  ADCBUF8L, _ADCBUF8
        .equiv  ADCBUF8H, _ADCBUF8+1

        .extern ADCBUF9
        .equiv  ADCBUF9L, _ADCBUF9
        .equiv  ADCBUF9H, _ADCBUF9+1

        .extern ADCBUFA
        .equiv  ADCBUFAL, _ADCBUFA
        .equiv  ADCBUFAH, _ADCBUFA+1

        .extern ADCBUFB
        .equiv  ADCBUFBL, _ADCBUFB
        .equiv  ADCBUFBH, _ADCBUFB+1

        .extern ADCBUFC
        .equiv  ADCBUFCL, _ADCBUFC
        .equiv  ADCBUFCH, _ADCBUFC+1

        .extern ADCBUFD
        .equiv  ADCBUFDL, _ADCBUFD
        .equiv  ADCBUFDH, _ADCBUFD+1

        .extern ADCBUFE
        .equiv  ADCBUFEL, _ADCBUFE
        .equiv  ADCBUFEH, _ADCBUFE+1

        .extern ADCBUFF
        .equiv  ADCBUFFL, _ADCBUFF
        .equiv  ADCBUFFH, _ADCBUFF+1

        .extern ADCON1
        .equiv  ADCON1L, _ADCON1
        .equiv  ADCON1H, _ADCON1+1

        .extern ADCON2
        .equiv  ADCON2L, _ADCON2
        .equiv  ADCON2H, _ADCON2+1

        .extern ADCON3
        .equiv  ADCON3L, _ADCON3
        .equiv  ADCON3H, _ADCON3+1

        .extern ADCHS
        .equiv  ADCHSL, _ADCHS
        .equiv  ADCHSH, _ADCHS+1

        .extern ADPCFG
        .equiv  ADPCFGL, _ADPCFG
        .equiv  ADPCFGH, _ADPCFG+1

        .extern ADCSSL
        .equiv  ADCSSLL, _ADCSSL
        .equiv  ADCSSLH, _ADCSSL+1

        .extern TRISA
        .equiv  TRISAL, _TRISA
        .equiv  TRISAH, _TRISA+1

        .extern PORTA
        .equiv  PORTAL, _PORTA
        .equiv  PORTAH, _PORTA+1

        .extern LATA
        .equiv  LATAL, _LATA
        .equiv  LATAH, _LATA+1

        .extern TRISB
        .equiv  TRISBL, _TRISB
        .equiv  TRISBH, _TRISB+1

        .extern PORTB
        .equiv  PORTBL, _PORTB
        .equiv  PORTBH, _PORTB+1

        .extern LATB
        .equiv  LATBL, _LATB
        .equiv  LATBH, _LATB+1

        .extern TRISC
        .equiv  TRISCL, _TRISC
        .equiv  TRISCH, _TRISC+1

        .extern PORTC
        .equiv  PORTCL, _PORTC
        .equiv  PORTCH, _PORTC+1

        .extern LATC
        .equiv  LATCL, _LATC
        .equiv  LATCH, _LATC+1

        .extern TRISD
        .equiv  TRISDL, _TRISD
        .equiv  TRISDH, _TRISD+1

        .extern PORTD
        .equiv  PORTDL, _PORTD
        .equiv  PORTDH, _PORTD+1

        .extern LATD
        .equiv  LATDL, _LATD
        .equiv  LATDH, _LATD+1

        .extern TRISF
        .equiv  TRISFL, _TRISF
        .equiv  TRISFH, _TRISF+1

        .extern PORTF
        .equiv  PORTFL, _PORTF
        .equiv  PORTFH, _PORTF+1

        .extern LATF
        .equiv  LATFL, _LATF
        .equiv  LATFH, _LATF+1

        .extern TRISG
        .equiv  TRISGL, _TRISG
        .equiv  TRISGH, _TRISG+1

        .extern PORTG
        .equiv  PORTGL, _PORTG
        .equiv  PORTGH, _PORTG+1

        .extern LATG
        .equiv  LATGL, _LATG
        .equiv  LATGH, _LATG+1

        .extern C1RXF0SID
        .equiv  C1RXF0SIDL, _C1RXF0SID
        .equiv  C1RXF0SIDH, _C1RXF0SID+1

        .extern C1RXF0EIDH
        .equiv  C1RXF0EIDHL, _C1RXF0EIDH
        .equiv  C1RXF0EIDHH, _C1RXF0EIDH+1

        .extern C1RXF0EIDL
        .equiv  C1RXF0EIDLL, _C1RXF0EIDL
        .equiv  C1RXF0EIDLH, _C1RXF0EIDL+1

        .extern C1RXF1SID
        .equiv  C1RXF1SIDL, _C1RXF1SID
        .equiv  C1RXF1SIDH, _C1RXF1SID+1

        .extern C1RXF1EIDH
        .equiv  C1RXF1EIDHL, _C1RXF1EIDH
        .equiv  C1RXF1EIDHH, _C1RXF1EIDH+1

        .extern C1RXF1EIDL
        .equiv  C1RXF1EIDLL, _C1RXF1EIDL
        .equiv  C1RXF1EIDLH, _C1RXF1EIDL+1

        .extern C1RXF2SID
        .equiv  C1RXF2SIDL, _C1RXF2SID
        .equiv  C1RXF2SIDH, _C1RXF2SID+1

        .extern C1RXF2EIDH
        .equiv  C1RXF2EIDHL, _C1RXF2EIDH
        .equiv  C1RXF2EIDHH, _C1RXF2EIDH+1

        .extern C1RXF2EIDL
        .equiv  C1RXF2EIDLL, _C1RXF2EIDL
        .equiv  C1RXF2EIDLH, _C1RXF2EIDL+1

        .extern C1RXF3SID
        .equiv  C1RXF3SIDL, _C1RXF3SID
        .equiv  C1RXF3SIDH, _C1RXF3SID+1

        .extern C1RXF3EIDH
        .equiv  C1RXF3EIDHL, _C1RXF3EIDH
        .equiv  C1RXF3EIDHH, _C1RXF3EIDH+1

        .extern C1RXF3EIDL
        .equiv  C1RXF3EIDLL, _C1RXF3EIDL
        .equiv  C1RXF3EIDLH, _C1RXF3EIDL+1

        .extern C1RXF4SID
        .equiv  C1RXF4SIDL, _C1RXF4SID
        .equiv  C1RXF4SIDH, _C1RXF4SID+1

        .extern C1RXF4EIDH
        .equiv  C1RXF4EIDHL, _C1RXF4EIDH
        .equiv  C1RXF4EIDHH, _C1RXF4EIDH+1

        .extern C1RXF4EIDL
        .equiv  C1RXF4EIDLL, _C1RXF4EIDL
        .equiv  C1RXF4EIDLH, _C1RXF4EIDL+1

        .extern C1RXF5SID
        .equiv  C1RXF5SIDL, _C1RXF5SID
        .equiv  C1RXF5SIDH, _C1RXF5SID+1

        .extern C1RXF5EIDH
        .equiv  C1RXF5EIDHL, _C1RXF5EIDH
        .equiv  C1RXF5EIDHH, _C1RXF5EIDH+1

        .extern C1RXF5EIDL
        .equiv  C1RXF5EIDLL, _C1RXF5EIDL
        .equiv  C1RXF5EIDLH, _C1RXF5EIDL+1

        .extern C1RXM0SID
        .equiv  C1RXM0SIDL, _C1RXM0SID
        .equiv  C1RXM0SIDH, _C1RXM0SID+1

        .extern C1RXM0EIDH
        .equiv  C1RXM0EIDHL, _C1RXM0EIDH
        .equiv  C1RXM0EIDHH, _C1RXM0EIDH+1

        .extern C1RXM0EIDL
        .equiv  C1RXM0EIDLL, _C1RXM0EIDL
        .equiv  C1RXM0EIDLH, _C1RXM0EIDL+1

        .extern C1RXM1SID
        .equiv  C1RXM1SIDL, _C1RXM1SID
        .equiv  C1RXM1SIDH, _C1RXM1SID+1

        .extern C1RXM1EIDH
        .equiv  C1RXM1EIDHL, _C1RXM1EIDH
        .equiv  C1RXM1EIDHH, _C1RXM1EIDH+1

        .extern C1RXM1EIDL
        .equiv  C1RXM1EIDLL, _C1RXM1EIDL
        .equiv  C1RXM1EIDLH, _C1RXM1EIDL+1

        .extern C1TX2SID
        .equiv  C1TX2SIDL, _C1TX2SID
        .equiv  C1TX2SIDH, _C1TX2SID+1

        .extern C1TX2EID
        .equiv  C1TX2EIDL, _C1TX2EID
        .equiv  C1TX2EIDH, _C1TX2EID+1

        .extern C1TX2DLC
        .equiv  C1TX2DLCL, _C1TX2DLC
        .equiv  C1TX2DLCH, _C1TX2DLC+1

        .extern C1TX2B1
        .equiv  C1TX2B1L, _C1TX2B1
        .equiv  C1TX2B1H, _C1TX2B1+1

        .extern C1TX2B2
        .equiv  C1TX2B2L, _C1TX2B2
        .equiv  C1TX2B2H, _C1TX2B2+1

        .extern C1TX2B3
        .equiv  C1TX2B3L, _C1TX2B3
        .equiv  C1TX2B3H, _C1TX2B3+1

        .extern C1TX2B4
        .equiv  C1TX2B4L, _C1TX2B4
        .equiv  C1TX2B4H, _C1TX2B4+1

        .extern C1TX2CON
        .equiv  C1TX2CONL, _C1TX2CON
        .equiv  C1TX2CONH, _C1TX2CON+1

        .extern C1TX1SID
        .equiv  C1TX1SIDL, _C1TX1SID
        .equiv  C1TX1SIDH, _C1TX1SID+1

        .extern C1TX1EID
        .equiv  C1TX1EIDL, _C1TX1EID
        .equiv  C1TX1EIDH, _C1TX1EID+1

        .extern C1TX1DLC
        .equiv  C1TX1DLCL, _C1TX1DLC
        .equiv  C1TX1DLCH, _C1TX1DLC+1

        .extern C1TX1B1
        .equiv  C1TX1B1L, _C1TX1B1
        .equiv  C1TX1B1H, _C1TX1B1+1

        .extern C1TX1B2
        .equiv  C1TX1B2L, _C1TX1B2
        .equiv  C1TX1B2H, _C1TX1B2+1

        .extern C1TX1B3
        .equiv  C1TX1B3L, _C1TX1B3
        .equiv  C1TX1B3H, _C1TX1B3+1

        .extern C1TX1B4
        .equiv  C1TX1B4L, _C1TX1B4
        .equiv  C1TX1B4H, _C1TX1B4+1

        .extern C1TX1CON
        .equiv  C1TX1CONL, _C1TX1CON
        .equiv  C1TX1CONH, _C1TX1CON+1

        .extern C1TX0SID
        .equiv  C1TX0SIDL, _C1TX0SID
        .equiv  C1TX0SIDH, _C1TX0SID+1

        .extern C1TX0EID
        .equiv  C1TX0EIDL, _C1TX0EID
        .equiv  C1TX0EIDH, _C1TX0EID+1

        .extern C1TX0DLC
        .equiv  C1TX0DLCL, _C1TX0DLC
        .equiv  C1TX0DLCH, _C1TX0DLC+1

        .extern C1TX0B1
        .equiv  C1TX0B1L, _C1TX0B1
        .equiv  C1TX0B1H, _C1TX0B1+1

        .extern C1TX0B2
        .equiv  C1TX0B2L, _C1TX0B2
        .equiv  C1TX0B2H, _C1TX0B2+1

        .extern C1TX0B3
        .equiv  C1TX0B3L, _C1TX0B3
        .equiv  C1TX0B3H, _C1TX0B3+1

        .extern C1TX0B4
        .equiv  C1TX0B4L, _C1TX0B4
        .equiv  C1TX0B4H, _C1TX0B4+1

        .extern C1TX0CON
        .equiv  C1TX0CONL, _C1TX0CON
        .equiv  C1TX0CONH, _C1TX0CON+1

        .extern C1RX1SID
        .equiv  C1RX1SIDL, _C1RX1SID
        .equiv  C1RX1SIDH, _C1RX1SID+1

        .extern C1RX1EID
        .equiv  C1RX1EIDL, _C1RX1EID
        .equiv  C1RX1EIDH, _C1RX1EID+1

        .extern C1RX1DLC
        .equiv  C1RX1DLCL, _C1RX1DLC
        .equiv  C1RX1DLCH, _C1RX1DLC+1

        .extern C1RX1B1
        .equiv  C1RX1B1L, _C1RX1B1
        .equiv  C1RX1B1H, _C1RX1B1+1

        .extern C1RX1B2
        .equiv  C1RX1B2L, _C1RX1B2
        .equiv  C1RX1B2H, _C1RX1B2+1

        .extern C1RX1B3
        .equiv  C1RX1B3L, _C1RX1B3
        .equiv  C1RX1B3H, _C1RX1B3+1

        .extern C1RX1B4
        .equiv  C1RX1B4L, _C1RX1B4
        .equiv  C1RX1B4H, _C1RX1B4+1

        .extern C1RX1CON
        .equiv  C1RX1CONL, _C1RX1CON
        .equiv  C1RX1CONH, _C1RX1CON+1

        .extern C1RX0SID
        .equiv  C1RX0SIDL, _C1RX0SID
        .equiv  C1RX0SIDH, _C1RX0SID+1

        .extern C1RX0EID
        .equiv  C1RX0EIDL, _C1RX0EID
        .equiv  C1RX0EIDH, _C1RX0EID+1

        .extern C1RX0DLC
        .equiv  C1RX0DLCL, _C1RX0DLC
        .equiv  C1RX0DLCH, _C1RX0DLC+1

        .extern C1RX0B1
        .equiv  C1RX0B1L, _C1RX0B1
        .equiv  C1RX0B1H, _C1RX0B1+1

        .extern C1RX0B2
        .equiv  C1RX0B2L, _C1RX0B2
        .equiv  C1RX0B2H, _C1RX0B2+1

        .extern C1RX0B3
        .equiv  C1RX0B3L, _C1RX0B3
        .equiv  C1RX0B3H, _C1RX0B3+1

        .extern C1RX0B4
        .equiv  C1RX0B4L, _C1RX0B4
        .equiv  C1RX0B4H, _C1RX0B4+1

        .extern C1RX0CON
        .equiv  C1RX0CONL, _C1RX0CON
        .equiv  C1RX0CONH, _C1RX0CON+1

        .extern C1CTRL
        .equiv  C1CTRLL, _C1CTRL
        .equiv  C1CTRLH, _C1CTRL+1

        .extern C1CFG1
        .equiv  C1CFG1L, _C1CFG1
        .equiv  C1CFG1H, _C1CFG1+1

        .extern C1CFG2
        .equiv  C1CFG2L, _C1CFG2
        .equiv  C1CFG2H, _C1CFG2+1

        .extern C1INTF
        .equiv  C1INTFL, _C1INTF
        .equiv  C1INTFH, _C1INTF+1

        .extern C1INTE
        .equiv  C1INTEL, _C1INTE
        .equiv  C1INTEH, _C1INTE+1

        .extern C1EC
        .equiv  C1ECL, _C1EC
        .equiv  C1ECH, _C1EC+1

        .extern C1RERRCNT
        .equiv  C1RERRCNTL, _C1RERRCNT
        .equiv  C1RERRCNTH, _C1RERRCNT+1

        .extern C1TERRCNT
        .equiv  C1TERRCNTL, _C1TERRCNT
        .equiv  C1TERRCNTH, _C1TERRCNT+1

        .extern C2RXF0SID
        .equiv  C2RXF0SIDL, _C2RXF0SID
        .equiv  C2RXF0SIDH, _C2RXF0SID+1

        .extern C2RXF0EIDH
        .equiv  C2RXF0EIDHL, _C2RXF0EIDH
        .equiv  C2RXF0EIDHH, _C2RXF0EIDH+1

        .extern C2RXF0EIDL
        .equiv  C2RXF0EIDLL, _C2RXF0EIDL
        .equiv  C2RXF0EIDLH, _C2RXF0EIDL+1

        .extern C2RXF1SID
        .equiv  C2RXF1SIDL, _C2RXF1SID
        .equiv  C2RXF1SIDH, _C2RXF1SID+1

        .extern C2RXF1EIDH
        .equiv  C2RXF1EIDHL, _C2RXF1EIDH
        .equiv  C2RXF1EIDHH, _C2RXF1EIDH+1

        .extern C2RXF1EIDL
        .equiv  C2RXF1EIDLL, _C2RXF1EIDL
        .equiv  C2RXF1EIDLH, _C2RXF1EIDL+1

        .extern C2RXF2SID
        .equiv  C2RXF2SIDL, _C2RXF2SID
        .equiv  C2RXF2SIDH, _C2RXF2SID+1

        .extern C2RXF2EIDH
        .equiv  C2RXF2EIDHL, _C2RXF2EIDH
        .equiv  C2RXF2EIDHH, _C2RXF2EIDH+1

        .extern C2RXF2EIDL
        .equiv  C2RXF2EIDLL, _C2RXF2EIDL
        .equiv  C2RXF2EIDLH, _C2RXF2EIDL+1

        .extern C2RXF3SID
        .equiv  C2RXF3SIDL, _C2RXF3SID
        .equiv  C2RXF3SIDH, _C2RXF3SID+1

        .extern C2RXF3EIDH
        .equiv  C2RXF3EIDHL, _C2RXF3EIDH
        .equiv  C2RXF3EIDHH, _C2RXF3EIDH+1

        .extern C2RXF3EIDL
        .equiv  C2RXF3EIDLL, _C2RXF3EIDL
        .equiv  C2RXF3EIDLH, _C2RXF3EIDL+1

        .extern C2RXF4SID
        .equiv  C2RXF4SIDL, _C2RXF4SID
        .equiv  C2RXF4SIDH, _C2RXF4SID+1

        .extern C2RXF4EIDH
        .equiv  C2RXF4EIDHL, _C2RXF4EIDH
        .equiv  C2RXF4EIDHH, _C2RXF4EIDH+1

        .extern C2RXF4EIDL
        .equiv  C2RXF4EIDLL, _C2RXF4EIDL
        .equiv  C2RXF4EIDLH, _C2RXF4EIDL+1

        .extern C2RXF5SID
        .equiv  C2RXF5SIDL, _C2RXF5SID
        .equiv  C2RXF5SIDH, _C2RXF5SID+1

        .extern C2RXF5EIDH
        .equiv  C2RXF5EIDHL, _C2RXF5EIDH
        .equiv  C2RXF5EIDHH, _C2RXF5EIDH+1

        .extern C2RXF5EIDL
        .equiv  C2RXF5EIDLL, _C2RXF5EIDL
        .equiv  C2RXF5EIDLH, _C2RXF5EIDL+1

        .extern C2RXM0SID
        .equiv  C2RXM0SIDL, _C2RXM0SID
        .equiv  C2RXM0SIDH, _C2RXM0SID+1

        .extern C2RXM0EIDH
        .equiv  C2RXM0EIDHL, _C2RXM0EIDH
        .equiv  C2RXM0EIDHH, _C2RXM0EIDH+1

        .extern C2RXM0EIDL
        .equiv  C2RXM0EIDLL, _C2RXM0EIDL
        .equiv  C2RXM0EIDLH, _C2RXM0EIDL+1

        .extern C2RXM1SID
        .equiv  C2RXM1SIDL, _C2RXM1SID
        .equiv  C2RXM1SIDH, _C2RXM1SID+1

        .extern C2RXM1EIDH
        .equiv  C2RXM1EIDHL, _C2RXM1EIDH
        .equiv  C2RXM1EIDHH, _C2RXM1EIDH+1

        .extern C2RXM1EIDL
        .equiv  C2RXM1EIDLL, _C2RXM1EIDL
        .equiv  C2RXM1EIDLH, _C2RXM1EIDL+1

        .extern C2TX2SID
        .equiv  C2TX2SIDL, _C2TX2SID
        .equiv  C2TX2SIDH, _C2TX2SID+1

        .extern C2TX2EID
        .equiv  C2TX2EIDL, _C2TX2EID
        .equiv  C2TX2EIDH, _C2TX2EID+1

        .extern C2TX2DLC
        .equiv  C2TX2DLCL, _C2TX2DLC
        .equiv  C2TX2DLCH, _C2TX2DLC+1

        .extern C2TX2B1
        .equiv  C2TX2B1L, _C2TX2B1
        .equiv  C2TX2B1H, _C2TX2B1+1

        .extern C2TX2B2
        .equiv  C2TX2B2L, _C2TX2B2
        .equiv  C2TX2B2H, _C2TX2B2+1

        .extern C2TX2B3
        .equiv  C2TX2B3L, _C2TX2B3
        .equiv  C2TX2B3H, _C2TX2B3+1

        .extern C2TX2B4
        .equiv  C2TX2B4L, _C2TX2B4
        .equiv  C2TX2B4H, _C2TX2B4+1

        .extern C2TX2CON
        .equiv  C2TX2CONL, _C2TX2CON
        .equiv  C2TX2CONH, _C2TX2CON+1

        .extern C2TX1SID
        .equiv  C2TX1SIDL, _C2TX1SID
        .equiv  C2TX1SIDH, _C2TX1SID+1

        .extern C2TX1EID
        .equiv  C2TX1EIDL, _C2TX1EID
        .equiv  C2TX1EIDH, _C2TX1EID+1

        .extern C2TX1DLC
        .equiv  C2TX1DLCL, _C2TX1DLC
        .equiv  C2TX1DLCH, _C2TX1DLC+1

        .extern C2TX1B1
        .equiv  C2TX1B1L, _C2TX1B1
        .equiv  C2TX1B1H, _C2TX1B1+1

        .extern C2TX1B2
        .equiv  C2TX1B2L, _C2TX1B2
        .equiv  C2TX1B2H, _C2TX1B2+1

        .extern C2TX1B3
        .equiv  C2TX1B3L, _C2TX1B3
        .equiv  C2TX1B3H, _C2TX1B3+1

        .extern C2TX1B4
        .equiv  C2TX1B4L, _C2TX1B4
        .equiv  C2TX1B4H, _C2TX1B4+1

        .extern C2TX1CON
        .equiv  C2TX1CONL, _C2TX1CON
        .equiv  C2TX1CONH, _C2TX1CON+1

        .extern C2TX0SID
        .equiv  C2TX0SIDL, _C2TX0SID
        .equiv  C2TX0SIDH, _C2TX0SID+1

        .extern C2TX0EID
        .equiv  C2TX0EIDL, _C2TX0EID
        .equiv  C2TX0EIDH, _C2TX0EID+1

        .extern C2TX0DLC
        .equiv  C2TX0DLCL, _C2TX0DLC
        .equiv  C2TX0DLCH, _C2TX0DLC+1

        .extern C2TX0B1
        .equiv  C2TX0B1L, _C2TX0B1
        .equiv  C2TX0B1H, _C2TX0B1+1

        .extern C2TX0B2
        .equiv  C2TX0B2L, _C2TX0B2
        .equiv  C2TX0B2H, _C2TX0B2+1

        .extern C2TX0B3
        .equiv  C2TX0B3L, _C2TX0B3
        .equiv  C2TX0B3H, _C2TX0B3+1

        .extern C2TX0B4
        .equiv  C2TX0B4L, _C2TX0B4
        .equiv  C2TX0B4H, _C2TX0B4+1

        .extern C2TX0CON
        .equiv  C2TX0CONL, _C2TX0CON
        .equiv  C2TX0CONH, _C2TX0CON+1

        .extern C2RX1SID
        .equiv  C2RX1SIDL, _C2RX1SID
        .equiv  C2RX1SIDH, _C2RX1SID+1

        .extern C2RX1EID
        .equiv  C2RX1EIDL, _C2RX1EID
        .equiv  C2RX1EIDH, _C2RX1EID+1

        .extern C2RX1DLC
        .equiv  C2RX1DLCL, _C2RX1DLC
        .equiv  C2RX1DLCH, _C2RX1DLC+1

        .extern C2RX1B1
        .equiv  C2RX1B1L, _C2RX1B1
        .equiv  C2RX1B1H, _C2RX1B1+1

        .extern C2RX1B2
        .equiv  C2RX1B2L, _C2RX1B2
        .equiv  C2RX1B2H, _C2RX1B2+1

        .extern C2RX1B3
        .equiv  C2RX1B3L, _C2RX1B3
        .equiv  C2RX1B3H, _C2RX1B3+1

        .extern C2RX1B4
        .equiv  C2RX1B4L, _C2RX1B4
        .equiv  C2RX1B4H, _C2RX1B4+1

        .extern C2RX1CON
        .equiv  C2RX1CONL, _C2RX1CON
        .equiv  C2RX1CONH, _C2RX1CON+1

        .extern C2RX0SID
        .equiv  C2RX0SIDL, _C2RX0SID
        .equiv  C2RX0SIDH, _C2RX0SID+1

        .extern C2RX0EID
        .equiv  C2RX0EIDL, _C2RX0EID
        .equiv  C2RX0EIDH, _C2RX0EID+1

        .extern C2RX0DLC
        .equiv  C2RX0DLCL, _C2RX0DLC
        .equiv  C2RX0DLCH, _C2RX0DLC+1

        .extern C2RX0B1
        .equiv  C2RX0B1L, _C2RX0B1
        .equiv  C2RX0B1H, _C2RX0B1+1

        .extern C2RX0B2
        .equiv  C2RX0B2L, _C2RX0B2
        .equiv  C2RX0B2H, _C2RX0B2+1

        .extern C2RX0B3
        .equiv  C2RX0B3L, _C2RX0B3
        .equiv  C2RX0B3H, _C2RX0B3+1

        .extern C2RX0B4
        .equiv  C2RX0B4L, _C2RX0B4
        .equiv  C2RX0B4H, _C2RX0B4+1

        .extern C2RX0CON
        .equiv  C2RX0CONL, _C2RX0CON
        .equiv  C2RX0CONH, _C2RX0CON+1

        .extern C2CTRL
        .equiv  C2CTRLL, _C2CTRL
        .equiv  C2CTRLH, _C2CTRL+1

        .extern C2CFG1
        .equiv  C2CFG1L, _C2CFG1
        .equiv  C2CFG1H, _C2CFG1+1

        .extern C2CFG2
        .equiv  C2CFG2L, _C2CFG2
        .equiv  C2CFG2H, _C2CFG2+1

        .extern C2INTF
        .equiv  C2INTFL, _C2INTF
        .equiv  C2INTFH, _C2INTF+1

        .extern C2INTE
        .equiv  C2INTEL, _C2INTE
        .equiv  C2INTEH, _C2INTE+1

        .extern C2EC
        .equiv  C2ECL, _C2EC
        .equiv  C2ECH, _C2EC+1

        .extern C2RERRCNT
        .equiv  C2RERRCNTL, _C2RERRCNT
        .equiv  C2RERRCNTH, _C2RERRCNT+1

        .extern C2TERRCNT
        .equiv  C2TERRCNTL, _C2TERRCNT
        .equiv  C2TERRCNTH, _C2TERRCNT+1

        .extern RCON
        .equiv  RCONL, _RCON
        .equiv  RCONH, _RCON+1

        .extern OSCCON

        .extern OSCCONL
        .equiv  OSCCONLL, _OSCCONL
        .equiv  OSCCONLH, _OSCCONL+1

        .extern OSCCONH
        .equiv  OSCCONHL, _OSCCONH
        .equiv  OSCCONHH, _OSCCONH+1

        .extern OSCTUN
        .equiv  OSCTUNL, _OSCTUN
        .equiv  OSCTUNH, _OSCTUN+1

        .extern BSRAM
        .equiv  BSRAML, _BSRAM
        .equiv  BSRAMH, _BSRAM+1

        .extern SSRAM
        .equiv  SSRAML, _SSRAM
        .equiv  SSRAMH, _SSRAM+1

        .extern NVMCON
        .equiv  NVMCONL, _NVMCON
        .equiv  NVMCONH, _NVMCON+1

        .extern NVMADR
        .equiv  NVMADRL, _NVMADR
        .equiv  NVMADRH, _NVMADR+1

        .extern NVMADRU
        .equiv  NVMADRUL, _NVMADRU
        .equiv  NVMADRUH, _NVMADRU+1

        .extern NVMKEY
        .equiv  NVMKEYL, _NVMKEY
        .equiv  NVMKEYH, _NVMKEY+1

        .extern PMD1
        .equiv  PMD1L, _PMD1
        .equiv  PMD1H, _PMD1+1

        .extern PMD2
        .equiv  PMD2L, _PMD2
        .equiv  PMD2H, _PMD2+1

;----- SR Bits -----------------------------------------------------
        .equiv C,       0x0000
        .equiv Z,       0x0001
        .equiv OV,      0x0002
        .equiv N,       0x0003
        .equiv RA,      0x0004
        .equiv DC,      0x0008
        .equiv DA,      0x0009
        .equiv SAB,     0x000A
        .equiv OAB,     0x000B
        .equiv SB,      0x000C
        .equiv SA,      0x000D
        .equiv OB,      0x000E
        .equiv OA,      0x000F

        .equiv IPL0,    0x0005
        .equiv IPL1,    0x0006
        .equiv IPL2,    0x0007


;----- CORCON Bits -----------------------------------------------------
        .equiv IF,      0x0000
        .equiv RND,     0x0001
        .equiv PSV,     0x0002
        .equiv IPL3,    0x0003
        .equiv ACCSAT,  0x0004
        .equiv SATDW,   0x0005
        .equiv SATB,    0x0006
        .equiv SATA,    0x0007
        .equiv EDT,     0x000B
        .equiv US,      0x000C

        .equiv DL0,     0x0008
        .equiv DL1,     0x0009
        .equiv DL2,     0x000A


;----- MODCON Bits -----------------------------------------------------
        .equiv YMODEN,  0x000E
        .equiv XMODEN,  0x000F

        .equiv XWM0,    0x0000
        .equiv XWM1,    0x0001
        .equiv XWM2,    0x0002
        .equiv XWM3,    0x0003
        .equiv YWM0,    0x0004
        .equiv YWM1,    0x0005
        .equiv YWM2,    0x0006
        .equiv YWM3,    0x0007
        .equiv BWM0,    0x0008
        .equiv BWM1,    0x0009
        .equiv BWM2,    0x000A
        .equiv BWM3,    0x000B


;----- XBREV Bits -----------------------------------------------------
        .equiv BREN,    0x000F

        .equiv XB0,     0x0000
        .equiv XB1,     0x0001
        .equiv XB2,     0x0002
        .equiv XB3,     0x0003
        .equiv XB4,     0x0004
        .equiv XB5,     0x0005
        .equiv XB6,     0x0006
        .equiv XB7,     0x0007
        .equiv XB8,     0x0008
        .equiv XB9,     0x0009
        .equiv XB10,    0x000A
        .equiv XB11,    0x000B
        .equiv XB12,    0x000C
        .equiv XB13,    0x000D
        .equiv XB14,    0x000E


;----- INTCON1 Bits -----------------------------------------------------
        .equiv OSCFAIL, 0x0001
        .equiv STKERR,  0x0002
        .equiv ADDRERR, 0x0003
        .equiv MATHERR, 0x0004
        .equiv COVTE,   0x0008
        .equiv OVBTE,   0x0009
        .equiv OVATE,   0x000A
        .equiv NSTDIS,  0x000F


;----- INTCON2 Bits -----------------------------------------------------
        .equiv INT0EP,  0x0000
        .equiv INT1EP,  0x0001
        .equiv INT2EP,  0x0002
        .equiv INT3EP,  0x0003
        .equiv INT4EP,  0x0004
        .equiv DISI,    0x000E
        .equiv ALTIVT,  0x000F


;----- IFS0 Bits -----------------------------------------------------
        .equiv INT0IF,  0x0000
        .equiv IC1IF,   0x0001
        .equiv OC1IF,   0x0002
        .equiv T1IF,    0x0003
        .equiv IC2IF,   0x0004
        .equiv OC2IF,   0x0005
        .equiv T2IF,    0x0006
        .equiv T3IF,    0x0007
        .equiv SPI1IF,  0x0008
        .equiv U1RXIF,  0x0009
        .equiv U1TXIF,  0x000A
        .equiv ADIF,    0x000B
        .equiv NVMIF,   0x000C
        .equiv SI2CIF,  0x000D
        .equiv MI2CIF,  0x000E
        .equiv CNIF,    0x000F


;----- IFS1 Bits -----------------------------------------------------
        .equiv INT1IF,  0x0000
        .equiv IC7IF,   0x0001
        .equiv IC8IF,   0x0002
        .equiv OC3IF,   0x0003
        .equiv OC4IF,   0x0004
        .equiv T4IF,    0x0005
        .equiv T5IF,    0x0006
        .equiv INT2IF,  0x0007
        .equiv U2RXIF,  0x0008
        .equiv U2TXIF,  0x0009
        .equiv SPI2IF,  0x000A
        .equiv C1IF,    0x000B
        .equiv IC3IF,   0x000C
        .equiv IC4IF,   0x000D
        .equiv IC5IF,   0x000E
        .equiv IC6IF,   0x000F


;----- IFS2 Bits -----------------------------------------------------
        .equiv OC5IF,   0x0000
        .equiv OC6IF,   0x0001
        .equiv OC7IF,   0x0002
        .equiv OC8IF,   0x0003
        .equiv INT3IF,  0x0004
        .equiv INT4IF,  0x0005
        .equiv C2IF,    0x0006
        .equiv DCIIF,   0x0009
        .equiv LVDIF,   0x000A


;----- IEC0 Bits -----------------------------------------------------
        .equiv INT0IE,  0x0000
        .equiv IC1IE,   0x0001
        .equiv OC1IE,   0x0002
        .equiv T1IE,    0x0003
        .equiv IC2IE,   0x0004
        .equiv OC2IE,   0x0005
        .equiv T2IE,    0x0006
        .equiv T3IE,    0x0007
        .equiv SPI1IE,  0x0008
        .equiv U1RXIE,  0x0009
        .equiv U1TXIE,  0x000A
        .equiv ADIE,    0x000B
        .equiv NVMIE,   0x000C
        .equiv SI2CIE,  0x000D
        .equiv MI2CIE,  0x000E
        .equiv CNIE,    0x000F


;----- IEC1 Bits -----------------------------------------------------
        .equiv INT1IE,  0x0000
        .equiv IC7IE,   0x0001
        .equiv IC8IE,   0x0002
        .equiv OC3IE,   0x0003
        .equiv OC4IE,   0x0004
        .equiv T4IE,    0x0005
        .equiv T5IE,    0x0006
        .equiv INT2IE,  0x0007
        .equiv U2RXIE,  0x0008
        .equiv U2TXIE,  0x0009
        .equiv SPI2IE,  0x000A
        .equiv C1IE,    0x000B
        .equiv IC3IE,   0x000C
        .equiv IC4IE,   0x000D
        .equiv IC5IE,   0x000E
        .equiv IC6IE,   0x000F


;----- IEC2 Bits -----------------------------------------------------
        .equiv OC5IE,   0x0000
        .equiv OC6IE,   0x0001
        .equiv OC7IE,   0x0002
        .equiv OC8IE,   0x0003
        .equiv INT3IE,  0x0004
        .equiv INT4IE,  0x0005
        .equiv C2IE,    0x0006
        .equiv DCIIE,   0x0009
        .equiv LVDIE,   0x000A


;----- IPC0 Bits -----------------------------------------------------
        .equiv INT0IP0, 0x0000
        .equiv INT0IP1, 0x0001
        .equiv INT0IP2, 0x0002
        .equiv IC1IP0,  0x0004
        .equiv IC1IP1,  0x0005
        .equiv IC1IP2,  0x0006
        .equiv OC1IP0,  0x0008
        .equiv OC1IP1,  0x0009
        .equiv OC1IP2,  0x000A
        .equiv T1IP0,   0x000C
        .equiv T1IP1,   0x000D
        .equiv T1IP2,   0x000E


;----- IPC1 Bits -----------------------------------------------------
        .equiv IC2IP0,  0x0000
        .equiv IC2IP1,  0x0001
        .equiv IC2IP2,  0x0002
        .equiv OC2IP0,  0x0004
        .equiv OC2IP1,  0x0005
        .equiv OC2IP2,  0x0006
        .equiv T2IP0,   0x0008
        .equiv T2IP1,   0x0009
        .equiv T2IP2,   0x000A
        .equiv T3IP0,   0x000C
        .equiv T3IP1,   0x000D
        .equiv T3IP2,   0x000E


;----- IPC2 Bits -----------------------------------------------------
        .equiv SPI1IP0, 0x0000
        .equiv SPI1IP1, 0x0001
        .equiv SPI1IP2, 0x0002
        .equiv U1RXIP0, 0x0004
        .equiv U1RXIP1, 0x0005
        .equiv U1RXIP2, 0x0006
        .equiv U1TXIP0, 0x0008
        .equiv U1TXIP1, 0x0009
        .equiv U1TXIP2, 0x000A
        .equiv ADIP0,   0x000C
        .equiv ADIP1,   0x000D
        .equiv ADIP2,   0x000E


;----- IPC3 Bits -----------------------------------------------------
        .equiv NVMIP0,  0x0000
        .equiv NVMIP1,  0x0001
        .equiv NVMIP2,  0x0002
        .equiv SI2CIP0, 0x0004
        .equiv SI2CIP1, 0x0005
        .equiv SI2CIP2, 0x0006
        .equiv MI2CIP0, 0x0008
        .equiv MI2CIP1, 0x0009
        .equiv MI2CIP2, 0x000A
        .equiv CNIP0,   0x000C
        .equiv CNIP1,   0x000D
        .equiv CNIP2,   0x000E


;----- IPC4 Bits -----------------------------------------------------
        .equiv INT1IP0, 0x0000
        .equiv INT1IP1, 0x0001
        .equiv INT1IP2, 0x0002
        .equiv IC7IP0,  0x0004
        .equiv IC7IP1,  0x0005
        .equiv IC7IP2,  0x0006
        .equiv IC8IP0,  0x0008
        .equiv IC8IP1,  0x0009
        .equiv IC8IP2,  0x000A
        .equiv OC3IP0,  0x000C
        .equiv OC3IP1,  0x000D
        .equiv OC3IP2,  0x000E


;----- IPC5 Bits -----------------------------------------------------
        .equiv OC4IP0,  0x0000
        .equiv OC4IP1,  0x0001
        .equiv OC4IP2,  0x0002
        .equiv T4IP0,   0x0004
        .equiv T4IP1,   0x0005
        .equiv T4IP2,   0x0006
        .equiv T5IP0,   0x0008
        .equiv T5IP1,   0x0009
        .equiv T5IP2,   0x000A
        .equiv INT2IP0, 0x000C
        .equiv INT2IP1, 0x000D
        .equiv INT2IP2, 0x000E


;----- IPC6 Bits -----------------------------------------------------
        .equiv U2RXIP0, 0x0000
        .equiv U2RXIP1, 0x0001
        .equiv U2RXIP2, 0x0002
        .equiv U2TXIP0, 0x0004
        .equiv U2TXIP1, 0x0005
        .equiv U2TXIP2, 0x0006
        .equiv SPI2IP0, 0x0008
        .equiv SPI2IP1, 0x0009
        .equiv SPI2IP2, 0x000A
        .equiv C1IP0,   0x000C
        .equiv C1IP1,   0x000D
        .equiv C1IP2,   0x000E


;----- IPC7 Bits -----------------------------------------------------
        .equiv IC3IP0,  0x0000
        .equiv IC3IP1,  0x0001
        .equiv IC3IP2,  0x0002
        .equiv IC4IP0,  0x0004
        .equiv IC4IP1,  0x0005
        .equiv IC4IP2,  0x0006
        .equiv IC5IP0,  0x0008
        .equiv IC5IP1,  0x0009
        .equiv IC5IP2,  0x000A
        .equiv IC6IP0,  0x000C
        .equiv IC6IP1,  0x000D
        .equiv IC6IP2,  0x000E


;----- IPC8 Bits -----------------------------------------------------
        .equiv OC5IP0,  0x0000
        .equiv OC5IP1,  0x0001
        .equiv OC5IP2,  0x0002
        .equiv OC6IP0,  0x0004
        .equiv OC6IP1,  0x0005
        .equiv OC6IP2,  0x0006
        .equiv OC7IP0,  0x0008
        .equiv OC7IP1,  0x0009
        .equiv OC7IP2,  0x000A
        .equiv OC8IP0,  0x000C
        .equiv OC8IP1,  0x000D
        .equiv OC8IP2,  0x000E


;----- IPC9 Bits -----------------------------------------------------
        .equiv INT3IP0, 0x0000
        .equiv INT3IP1, 0x0001
        .equiv INT3IP2, 0x0002
        .equiv INT4IP0, 0x0004
        .equiv INT4IP1, 0x0005
        .equiv INT4IP2, 0x0006
        .equiv C2IP0,   0x0008
        .equiv C2IP1,   0x0009
        .equiv C2IP2,   0x000A


;----- IPC10 Bits -----------------------------------------------------
        .equiv DCIIP0,  0x0004
        .equiv DCIIP1,  0x0005
        .equiv DCIIP2,  0x0006
        .equiv LVDIP0,  0x0008
        .equiv LVDIP1,  0x0009
        .equiv LVDIP2,  0x000A


;----- INTTREG Bits -----------------------------------------------------
        .equiv VHOLD,   0x000D
        .equiv TMODE,   0x000E
        .equiv IRQTOCPU, 0x000F

        .equiv VECNUM0, 0x0000
        .equiv VECNUM1, 0x0001
        .equiv VECNUM2, 0x0002
        .equiv VECNUM3, 0x0003
        .equiv VECNUM4, 0x0004
        .equiv VECNUM5, 0x0005
        .equiv ILR0,    0x0008
        .equiv ILR1,    0x0009
        .equiv ILR2,    0x000A
        .equiv ILR3,    0x000B


;----- CNEN1 Bits -----------------------------------------------------
        .equiv CN0IE,   0x0000
        .equiv CN1IE,   0x0001
        .equiv CN2IE,   0x0002
        .equiv CN3IE,   0x0003
        .equiv CN4IE,   0x0004
        .equiv CN5IE,   0x0005
        .equiv CN6IE,   0x0006
        .equiv CN7IE,   0x0007
        .equiv CN8IE,   0x0008
        .equiv CN9IE,   0x0009
        .equiv CN10IE,  0x000A
        .equiv CN11IE,  0x000B
        .equiv CN12IE,  0x000C
        .equiv CN13IE,  0x000D
        .equiv CN14IE,  0x000E
        .equiv CN15IE,  0x000F


;----- CNEN2 Bits -----------------------------------------------------
        .equiv CN16IE,  0x0000
        .equiv CN17IE,  0x0001
        .equiv CN18IE,  0x0002
        .equiv CN19IE,  0x0003
        .equiv CN20IE,  0x0004
        .equiv CN21IE,  0x0005
        .equiv CN22IE,  0x0006
        .equiv CN23IE,  0x0007


;----- CNPU1 Bits -----------------------------------------------------
        .equiv CN0PUE,  0x0000
        .equiv CN1PUE,  0x0001
        .equiv CN2PUE,  0x0002
        .equiv CN3PUE,  0x0003
        .equiv CN4PUE,  0x0004
        .equiv CN5PUE,  0x0005
        .equiv CN6PUE,  0x0006
        .equiv CN7PUE,  0x0007
        .equiv CN8PUE,  0x0008
        .equiv CN9PUE,  0x0009
        .equiv CN10PUE, 0x000A
        .equiv CN11PUE, 0x000B
        .equiv CN12PUE, 0x000C
        .equiv CN13PUE, 0x000D
        .equiv CN14PUE, 0x000E
        .equiv CN15PUE, 0x000F


;----- CNPU2 Bits -----------------------------------------------------
        .equiv CN16PUE, 0x0000
        .equiv CN17PUE, 0x0001
        .equiv CN18PUE, 0x0002
        .equiv CN19PUE, 0x0003
        .equiv CN20PUE, 0x0004
        .equiv CN21PUE, 0x0005
        .equiv CN22PUE, 0x0006
        .equiv CN23PUE, 0x0007


;----- T1CON Bits -----------------------------------------------------
        .equiv TCS,     0x0001
        .equiv TSYNC,   0x0002
        .equiv TGATE,   0x0006
        .equiv TSIDL,   0x000D
        .equiv TON,     0x000F

        .equiv TCKPS0,  0x0004
        .equiv TCKPS1,  0x0005


;----- T2CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
        .equiv T32,     0x0003
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005


;----- T3CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005


;----- T4CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
       ;.equiv T32,     0x0003
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005


;----- T5CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005


;----- IC1CON Bits -----------------------------------------------------
        .equiv ICBNE,   0x0003
        .equiv ICOV,    0x0004
        .equiv ICTMR,   0x0007
        .equiv ICSIDL,  0x000D


;----- IC2CON Bits -----------------------------------------------------
       ;.equiv ICBNE,   0x0003
       ;.equiv ICOV,    0x0004
       ;.equiv ICTMR,   0x0007
       ;.equiv ICSIDL,  0x000D

        .equiv ICM0,    0x0000
        .equiv ICM1,    0x0001
        .equiv ICM2,    0x0002
        .equiv ICI0,    0x0005
        .equiv ICI1,    0x0006


;----- IC3CON Bits -----------------------------------------------------
       ;.equiv ICBNE,   0x0003
       ;.equiv ICOV,    0x0004
       ;.equiv ICTMR,   0x0007
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006


;----- IC4CON Bits -----------------------------------------------------
       ;.equiv ICBNE,   0x0003
       ;.equiv ICOV,    0x0004
       ;.equiv ICTMR,   0x0007
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006


;----- IC5CON Bits -----------------------------------------------------
       ;.equiv ICBNE,   0x0003
       ;.equiv ICOV,    0x0004
       ;.equiv ICTMR,   0x0007
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006


;----- IC6CON Bits -----------------------------------------------------
       ;.equiv ICBNE,   0x0003
       ;.equiv ICOV,    0x0004
       ;.equiv ICTMR,   0x0007
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006


;----- IC7CON Bits -----------------------------------------------------
       ;.equiv ICBNE,   0x0003
       ;.equiv ICOV,    0x0004
       ;.equiv ICTMR,   0x0007
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006


;----- IC8CON Bits -----------------------------------------------------
       ;.equiv ICBNE,   0x0003
       ;.equiv ICOV,    0x0004
       ;.equiv ICTMR,   0x0007
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006


;----- OC1CON Bits -----------------------------------------------------
        .equiv OCTSEL,  0x0003
        .equiv OCFLT,   0x0004
        .equiv OCSIDL,  0x000D

        .equiv OCM0,    0x0000
        .equiv OCM1,    0x0001
        .equiv OCM2,    0x0002


;----- OC2CON Bits -----------------------------------------------------
       ;.equiv OCTSEL,  0x0003
       ;.equiv OCFLT,   0x0004
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002


;----- OC3CON Bits -----------------------------------------------------
       ;.equiv OCTSEL,  0x0003
       ;.equiv OCFLT,   0x0004
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002


;----- OC4CON Bits -----------------------------------------------------
       ;.equiv OCTSEL,  0x0003
       ;.equiv OCFLT,   0x0004
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002


;----- OC5CON Bits -----------------------------------------------------
       ;.equiv OCTSEL,  0x0003
       ;.equiv OCFLT,   0x0004
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002


;----- OC6CON Bits -----------------------------------------------------
       ;.equiv OCTSEL,  0x0003
       ;.equiv OCFLT,   0x0004
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002


;----- OC7CON Bits -----------------------------------------------------
       ;.equiv OCTSEL,  0x0003
       ;.equiv OCFLT,   0x0004
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002


;----- OC8CON Bits -----------------------------------------------------
       ;.equiv OCTSEL,  0x0003
       ;.equiv OCFLT,   0x0004
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002


;----- I2CRCV Bits -----------------------------------------------------
        .equiv I2CRCV0, 0x0000
        .equiv I2CRCV1, 0x0001
        .equiv I2CRCV2, 0x0002
        .equiv I2CRCV3, 0x0003
        .equiv I2CRCV4, 0x0004
        .equiv I2CRCV5, 0x0005
        .equiv I2CRCV6, 0x0006
        .equiv I2CRCV7, 0x0007


;----- I2CTRN Bits -----------------------------------------------------
        .equiv I2CTRN0, 0x0000
        .equiv I2CTRN1, 0x0001
        .equiv I2CTRN2, 0x0002
        .equiv I2CTRN3, 0x0003
        .equiv I2CTRN4, 0x0004
        .equiv I2CTRN5, 0x0005
        .equiv I2CTRN6, 0x0006
        .equiv I2CTRN7, 0x0007


;----- I2CCON Bits -----------------------------------------------------
        .equiv SEN,     0x0000
        .equiv RSEN,    0x0001
        .equiv PEN,     0x0002
        .equiv RCEN,    0x0003
        .equiv ACKEN,   0x0004
        .equiv ACKDT,   0x0005
        .equiv STREN,   0x0006
        .equiv GCEN,    0x0007
        .equiv SMEN,    0x0008
        .equiv DISSLW,  0x0009
        .equiv A10M,    0x000A
        .equiv IPMIEN,  0x000B
        .equiv SCLREL,  0x000C
        .equiv I2CSIDL, 0x000D
        .equiv I2CEN,   0x000F


;----- I2CSTAT Bits -----------------------------------------------------
        .equiv TBF,     0x0000
        .equiv RBF,     0x0001
        .equiv R_NOT_W, 0x0002
        .equiv S,       0x0003
        .equiv P,       0x0004
        .equiv D_NOT_A, 0x0005
        .equiv I2COV,   0x0006
        .equiv IWCOL,   0x0007
        .equiv ADD10,   0x0008
        .equiv GCSTAT,  0x0009
        .equiv BCL,     0x000A
        .equiv TRSTAT,  0x000E
        .equiv ACKSTAT, 0x000F

        .equiv R_W,     0x0002
        .equiv D_A,     0x0005


;----- U1MODE Bits -----------------------------------------------------
        .equiv STSEL,   0x0000
        .equiv ABAUD,   0x0005
        .equiv LPBACK,  0x0006
        .equiv WAKE,    0x0007
        .equiv ALTIO,   0x000A
        .equiv USIDL,   0x000D
        .equiv UARTEN,  0x000F

        .equiv PDSEL0,  0x0001
        .equiv PDSEL1,  0x0002


;----- U1STA Bits -----------------------------------------------------
        .equiv URXDA,   0x0000
        .equiv OERR,    0x0001
        .equiv FERR,    0x0002
        .equiv PERR,    0x0003
        .equiv RIDLE,   0x0004
        .equiv ADDEN,   0x0005
        .equiv TRMT,    0x0008
        .equiv UTXBF,   0x0009
        .equiv UTXEN,   0x000A
        .equiv UTXBRK,  0x000B
        .equiv UTXISEL, 0x000F

        .equiv URXISEL0, 0x0006
        .equiv URXISEL1, 0x0007


;----- U1TXREG Bits -----------------------------------------------------
        .equiv UTXREG0, 0x0000
        .equiv UTXREG1, 0x0001
        .equiv UTXREG2, 0x0002
        .equiv UTXREG3, 0x0003
        .equiv UTXREG4, 0x0004
        .equiv UTXREG5, 0x0005
        .equiv UTXREG6, 0x0006
        .equiv UTXREG7, 0x0007
        .equiv UTXREG8, 0x0008

        .equiv UTX8,    0x0008


;----- U1RXREG Bits -----------------------------------------------------
        .equiv URXREG0, 0x0000
        .equiv URXREG1, 0x0001
        .equiv URXREG2, 0x0002
        .equiv URXREG3, 0x0003
        .equiv URXREG4, 0x0004
        .equiv URXREG5, 0x0005
        .equiv URXREG6, 0x0006
        .equiv URXREG7, 0x0007
        .equiv URXREG8, 0x0008

        .equiv URX8,    0x0008


;----- U2MODE Bits -----------------------------------------------------
       ;.equiv STSEL,   0x0000
       ;.equiv ABAUD,   0x0005
       ;.equiv LPBACK,  0x0006
       ;.equiv WAKE,    0x0007
       ;.equiv USIDL,   0x000D
       ;.equiv UARTEN,  0x000F

       ;.equiv PDSEL0,  0x0001
       ;.equiv PDSEL1,  0x0002


;----- U2STA Bits -----------------------------------------------------
       ;.equiv URXDA,   0x0000
       ;.equiv OERR,    0x0001
       ;.equiv FERR,    0x0002
       ;.equiv PERR,    0x0003
       ;.equiv RIDLE,   0x0004
       ;.equiv ADDEN,   0x0005
       ;.equiv TRMT,    0x0008
       ;.equiv UTXBF,   0x0009
       ;.equiv UTXEN,   0x000A
       ;.equiv UTXBRK,  0x000B
       ;.equiv UTXISEL, 0x000F

       ;.equiv URXISEL0, 0x0006
       ;.equiv URXISEL1, 0x0007


;----- U2TXREG Bits -----------------------------------------------------
       ;.equiv UTXREG0, 0x0000
       ;.equiv UTXREG1, 0x0001
       ;.equiv UTXREG2, 0x0002
       ;.equiv UTXREG3, 0x0003
       ;.equiv UTXREG4, 0x0004
       ;.equiv UTXREG5, 0x0005
       ;.equiv UTXREG6, 0x0006
       ;.equiv UTXREG7, 0x0007
       ;.equiv UTXREG8, 0x0008

       ;.equiv UTX8,    0x0008


;----- U2RXREG Bits -----------------------------------------------------
       ;.equiv URXREG0, 0x0000
       ;.equiv URXREG1, 0x0001
       ;.equiv URXREG2, 0x0002
       ;.equiv URXREG3, 0x0003
       ;.equiv URXREG4, 0x0004
       ;.equiv URXREG5, 0x0005
       ;.equiv URXREG6, 0x0006
       ;.equiv URXREG7, 0x0007
       ;.equiv URXREG8, 0x0008

       ;.equiv URX8,    0x0008


;----- SPI1STAT Bits -----------------------------------------------------
        .equiv SPIRBF,  0x0000
        .equiv SPITBF,  0x0001
        .equiv SPIROV,  0x0006
        .equiv SPISIDL, 0x000D
        .equiv SPIEN,   0x000F


;----- SPI1CON Bits -----------------------------------------------------
        .equiv PPRE0,   0x0000
        .equiv PPRE1,   0x0001
        .equiv SPRE0,   0x0002
        .equiv SPRE1,   0x0003
        .equiv SPRE2,   0x0004
        .equiv MSTEN,   0x0005
        .equiv CKP,     0x0006
        .equiv SSEN,    0x0007
        .equiv CKE,     0x0008
        .equiv SMP,     0x0009
        .equiv MODE16,  0x000A
        .equiv DISSDO,  0x000B
        .equiv SPIFSD,  0x000D
        .equiv FRMEN,   0x000E



;----- SPI2STAT Bits -----------------------------------------------------
       ;.equiv SPIRBF,  0x0000
       ;.equiv SPITBF,  0x0001
       ;.equiv SPIROV,  0x0006
       ;.equiv SPISIDL, 0x000D
       ;.equiv SPIEN,   0x000F


;----- SPI2CON Bits -----------------------------------------------------
       ;.equiv PPRE0,   0x0000
       ;.equiv PPRE1,   0x0001
       ;.equiv SPRE0,   0x0002
       ;.equiv SPRE1,   0x0003
       ;.equiv SPRE2,   0x0004
       ;.equiv MSTEN,   0x0005
       ;.equiv CKP,     0x0006
       ;.equiv SSEN,    0x0007
       ;.equiv CKE,     0x0008
       ;.equiv SMP,     0x0009
       ;.equiv MODE16,  0x000A
       ;.equiv DISSDO,  0x000B
       ;.equiv SPIFSD,  0x000D
       ;.equiv FRMEN,   0x000E



;----- DCICON1 Bits -----------------------------------------------------
        .equiv DJST,    0x0005
        .equiv CSDOM,   0x0006
        .equiv UNFM,    0x0007
        .equiv COFSD,   0x0008
        .equiv CSCKE,   0x0009
        .equiv CSCKD,   0x000A
        .equiv DLOOP,   0x000B
        .equiv DCISIDL, 0x000D
        .equiv DCIEN,   0x000F

        .equiv COFSM0,  0x0000
        .equiv COFSM1,  0x0001


;----- DCICON2 Bits -----------------------------------------------------
        .equiv WS0,     0x0000
        .equiv WS1,     0x0001
        .equiv WS2,     0x0002
        .equiv WS3,     0x0003
        .equiv COFSG0,  0x0005
        .equiv COFSG1,  0x0006
        .equiv COFSG2,  0x0007
        .equiv COFSG3,  0x0008
        .equiv BLEN0,   0x000A
        .equiv BLEN1,   0x000B


;----- DCICON3 Bits -----------------------------------------------------
        .equiv BCG0,    0x0000
        .equiv BCG1,    0x0001
        .equiv BCG2,    0x0002
        .equiv BCG3,    0x0003
        .equiv BCG4,    0x0004
        .equiv BCG5,    0x0005
        .equiv BCG6,    0x0006
        .equiv BCG7,    0x0007
        .equiv BCG8,    0x0008
        .equiv BCG9,    0x0009
        .equiv BCG10,   0x000A
        .equiv BCG11,   0x000B


;----- DCISTAT Bits -----------------------------------------------------
        .equiv TMPTY,   0x0000
        .equiv TUNF,    0x0001
        .equiv RFUL,    0x0002
        .equiv ROV,     0x0003

        .equiv SLOT0,   0x0008
        .equiv SLOT1,   0x0009
        .equiv SLOT2,   0x000A
        .equiv SLOT3,   0x000B


;----- TSCON Bits -----------------------------------------------------
        .equiv TSE0,    0x0000
        .equiv TSE1,    0x0001
        .equiv TSE2,    0x0002
        .equiv TSE3,    0x0003
        .equiv TSE4,    0x0004
        .equiv TSE5,    0x0005
        .equiv TSE6,    0x0006
        .equiv TSE7,    0x0007
        .equiv TSE8,    0x0008
        .equiv TSE9,    0x0009
        .equiv TSE10,   0x000A
        .equiv TSE11,   0x000B
        .equiv TSE12,   0x000C
        .equiv TSE13,   0x000D
        .equiv TSE14,   0x000E
        .equiv TSE15,   0x000F


;----- RSCON Bits -----------------------------------------------------
        .equiv RSE0,    0x0000
        .equiv RSE1,    0x0001
        .equiv RSE2,    0x0002
        .equiv RSE3,    0x0003
        .equiv RSE4,    0x0004
        .equiv RSE5,    0x0005
        .equiv RSE6,    0x0006
        .equiv RSE7,    0x0007
        .equiv RSE8,    0x0008
        .equiv RSE9,    0x0009
        .equiv RSE10,   0x000A
        .equiv RSE11,   0x000B
        .equiv RSE12,   0x000C
        .equiv RSE13,   0x000D
        .equiv RSE14,   0x000E
        .equiv RSE15,   0x000F


;----- ADCON1 Bits -----------------------------------------------------
        .equiv DONE,    0x0000
        .equiv SAMP,    0x0001
        .equiv ASAM,    0x0002
        .equiv ADSIDL,  0x000D
        .equiv ADON,    0x000F

        .equiv SSRC0,   0x0005
        .equiv SSRC1,   0x0006
        .equiv SSRC2,   0x0007
        .equiv FORM0,   0x0008
        .equiv FORM1,   0x0009


;----- ADCON2 Bits -----------------------------------------------------
        .equiv ALTS,    0x0000
        .equiv BUFM,    0x0001
        .equiv BUFS,    0x0007
        .equiv CSCNA,   0x000A

        .equiv SMPI0,   0x0002
        .equiv SMPI1,   0x0003
        .equiv SMPI2,   0x0004
        .equiv SMPI3,   0x0005
        .equiv VCFG0,   0x000D
        .equiv VCFG1,   0x000E
        .equiv VCFG2,   0x000F


;----- ADCON3 Bits -----------------------------------------------------
        .equiv ADRC,    0x0007

        .equiv ADCS0,   0x0000
        .equiv ADCS1,   0x0001
        .equiv ADCS2,   0x0002
        .equiv ADCS3,   0x0003
        .equiv ADCS4,   0x0004
        .equiv ADCS5,   0x0005
        .equiv SAMC0,   0x0008
        .equiv SAMC1,   0x0009
        .equiv SAMC2,   0x000A
        .equiv SAMC3,   0x000B
        .equiv SAMC4,   0x000C


;----- ADCHS Bits -----------------------------------------------------
        .equiv CH0NA,   0x0004
        .equiv CH0NB,   0x000C

        .equiv CH0SA0,  0x0000
        .equiv CH0SA1,  0x0001
        .equiv CH0SA2,  0x0002
        .equiv CH0SA3,  0x0003
        .equiv CH0SB0,  0x0008
        .equiv CH0SB1,  0x0009
        .equiv CH0SB2,  0x000A
        .equiv CH0SB3,  0x000B


;----- ADPCFG Bits -----------------------------------------------------
        .equiv PCFG0,   0x0000
        .equiv PCFG1,   0x0001
        .equiv PCFG2,   0x0002
        .equiv PCFG3,   0x0003
        .equiv PCFG4,   0x0004
        .equiv PCFG5,   0x0005
        .equiv PCFG6,   0x0006
        .equiv PCFG7,   0x0007
        .equiv PCFG8,   0x0008
        .equiv PCFG9,   0x0009
        .equiv PCFG10,  0x000A
        .equiv PCFG11,  0x000B
        .equiv PCFG12,  0x000C
        .equiv PCFG13,  0x000D
        .equiv PCFG14,  0x000E
        .equiv PCFG15,  0x000F


;----- ADCSSL Bits -----------------------------------------------------
        .equiv CSSL0,   0x0000
        .equiv CSSL1,   0x0001
        .equiv CSSL2,   0x0002
        .equiv CSSL3,   0x0003
        .equiv CSSL4,   0x0004
        .equiv CSSL5,   0x0005
        .equiv CSSL6,   0x0006
        .equiv CSSL7,   0x0007
        .equiv CSSL8,   0x0008
        .equiv CSSL9,   0x0009
        .equiv CSSL10,  0x000A
        .equiv CSSL11,  0x000B
        .equiv CSSL12,  0x000C
        .equiv CSSL13,  0x000D
        .equiv CSSL14,  0x000E
        .equiv CSSL15,  0x000F


;----- TRISA Bits -----------------------------------------------------
        .equiv TRISA6,  0x0006
        .equiv TRISA7,  0x0007
        .equiv TRISA9,  0x0009
        .equiv TRISA10, 0x000A
        .equiv TRISA12, 0x000C
        .equiv TRISA13, 0x000D
        .equiv TRISA14, 0x000E
        .equiv TRISA15, 0x000F


;----- PORTA Bits -----------------------------------------------------
        .equiv RA6,     0x0006
        .equiv RA7,     0x0007
        .equiv RA9,     0x0009
        .equiv RA10,    0x000A
        .equiv RA12,    0x000C
        .equiv RA13,    0x000D
        .equiv RA14,    0x000E
        .equiv RA15,    0x000F


;----- LATA Bits -----------------------------------------------------
        .equiv LATA6,   0x0006
        .equiv LATA7,   0x0007
        .equiv LATA9,   0x0009
        .equiv LATA10,  0x000A
        .equiv LATA12,  0x000C
        .equiv LATA13,  0x000D
        .equiv LATA14,  0x000E
        .equiv LATA15,  0x000F


;----- TRISB Bits -----------------------------------------------------
        .equiv TRISB0,  0x0000
        .equiv TRISB1,  0x0001
        .equiv TRISB2,  0x0002
        .equiv TRISB3,  0x0003
        .equiv TRISB4,  0x0004
        .equiv TRISB5,  0x0005
        .equiv TRISB6,  0x0006
        .equiv TRISB7,  0x0007
        .equiv TRISB8,  0x0008
        .equiv TRISB9,  0x0009
        .equiv TRISB10, 0x000A
        .equiv TRISB11, 0x000B
        .equiv TRISB12, 0x000C
        .equiv TRISB13, 0x000D
        .equiv TRISB14, 0x000E
        .equiv TRISB15, 0x000F


;----- PORTB Bits -----------------------------------------------------
        .equiv RB0,     0x0000
        .equiv RB1,     0x0001
        .equiv RB2,     0x0002
        .equiv RB3,     0x0003
        .equiv RB4,     0x0004
        .equiv RB5,     0x0005
        .equiv RB6,     0x0006
        .equiv RB7,     0x0007
        .equiv RB8,     0x0008
        .equiv RB9,     0x0009
        .equiv RB10,    0x000A
        .equiv RB11,    0x000B
        .equiv RB12,    0x000C
        .equiv RB13,    0x000D
        .equiv RB14,    0x000E
        .equiv RB15,    0x000F


;----- LATB Bits -----------------------------------------------------
        .equiv LATB0,   0x0000
        .equiv LATB1,   0x0001
        .equiv LATB2,   0x0002
        .equiv LATB3,   0x0003
        .equiv LATB4,   0x0004
        .equiv LATB5,   0x0005
        .equiv LATB6,   0x0006
        .equiv LATB7,   0x0007
        .equiv LATB8,   0x0008
        .equiv LATB9,   0x0009
        .equiv LATB10,  0x000A
        .equiv LATB11,  0x000B
        .equiv LATB12,  0x000C
        .equiv LATB13,  0x000D
        .equiv LATB14,  0x000E
        .equiv LATB15,  0x000F


;----- TRISC Bits -----------------------------------------------------
        .equiv TRISC1,  0x0001
        .equiv TRISC2,  0x0002
        .equiv TRISC3,  0x0003
        .equiv TRISC4,  0x0004
        .equiv TRISC13, 0x000D
        .equiv TRISC14, 0x000E
        .equiv TRISC15, 0x000F


;----- PORTC Bits -----------------------------------------------------
        .equiv RC1,     0x0001
        .equiv RC2,     0x0002
        .equiv RC3,     0x0003
        .equiv RC4,     0x0004
        .equiv RC13,    0x000D
        .equiv RC14,    0x000E
        .equiv RC15,    0x000F


;----- LATC Bits -----------------------------------------------------
        .equiv LATC1,   0x0001
        .equiv LATC2,   0x0002
        .equiv LATC3,   0x0003
        .equiv LATC4,   0x0004
        .equiv LATC13,  0x000D
        .equiv LATC14,  0x000E
        .equiv LATC15,  0x000F


;----- TRISD Bits -----------------------------------------------------
        .equiv TRISD0,  0x0000
        .equiv TRISD1,  0x0001
        .equiv TRISD2,  0x0002
        .equiv TRISD3,  0x0003
        .equiv TRISD4,  0x0004
        .equiv TRISD5,  0x0005
        .equiv TRISD6,  0x0006
        .equiv TRISD7,  0x0007
        .equiv TRISD8,  0x0008
        .equiv TRISD9,  0x0009
        .equiv TRISD10, 0x000A
        .equiv TRISD11, 0x000B
        .equiv TRISD12, 0x000C
        .equiv TRISD13, 0x000D
        .equiv TRISD14, 0x000E
        .equiv TRISD15, 0x000F


;----- PORTD Bits -----------------------------------------------------
        .equiv RD0,     0x0000
        .equiv RD1,     0x0001
        .equiv RD2,     0x0002
        .equiv RD3,     0x0003
        .equiv RD4,     0x0004
        .equiv RD5,     0x0005
        .equiv RD6,     0x0006
        .equiv RD7,     0x0007
        .equiv RD8,     0x0008
        .equiv RD9,     0x0009
        .equiv RD10,    0x000A
        .equiv RD11,    0x000B
        .equiv RD12,    0x000C
        .equiv RD13,    0x000D
        .equiv RD14,    0x000E
        .equiv RD15,    0x000F


;----- LATD Bits -----------------------------------------------------
        .equiv LATD0,   0x0000
        .equiv LATD1,   0x0001
        .equiv LATD2,   0x0002
        .equiv LATD3,   0x0003
        .equiv LATD4,   0x0004
        .equiv LATD5,   0x0005
        .equiv LATD6,   0x0006
        .equiv LATD7,   0x0007
        .equiv LATD8,   0x0008
        .equiv LATD9,   0x0009
        .equiv LATD10,  0x000A
        .equiv LATD11,  0x000B
        .equiv LATD12,  0x000C
        .equiv LATD13,  0x000D
        .equiv LATD14,  0x000E
        .equiv LATD15,  0x000F


;----- TRISF Bits -----------------------------------------------------
        .equiv TRISF0,  0x0000
        .equiv TRISF1,  0x0001
        .equiv TRISF2,  0x0002
        .equiv TRISF3,  0x0003
        .equiv TRISF4,  0x0004
        .equiv TRISF5,  0x0005
        .equiv TRISF6,  0x0006
        .equiv TRISF7,  0x0007
        .equiv TRISF8,  0x0008


;----- PORTF Bits -----------------------------------------------------
        .equiv RF0,     0x0000
        .equiv RF1,     0x0001
        .equiv RF2,     0x0002
        .equiv RF3,     0x0003
        .equiv RF4,     0x0004
        .equiv RF5,     0x0005
        .equiv RF6,     0x0006
        .equiv RF7,     0x0007
        .equiv RF8,     0x0008


;----- LATF Bits -----------------------------------------------------
        .equiv LATF0,   0x0000
        .equiv LATF1,   0x0001
        .equiv LATF2,   0x0002
        .equiv LATF3,   0x0003
        .equiv LATF4,   0x0004
        .equiv LATF5,   0x0005
        .equiv LATF6,   0x0006
        .equiv LATF7,   0x0007
        .equiv LATF8,   0x0008


;----- TRISG Bits -----------------------------------------------------
        .equiv TRISG0,  0x0000
        .equiv TRISG1,  0x0001
        .equiv TRISG2,  0x0002
        .equiv TRISG3,  0x0003
        .equiv TRISG6,  0x0006
        .equiv TRISG7,  0x0007
        .equiv TRISG8,  0x0008
        .equiv TRISG9,  0x0009
        .equiv TRISG12, 0x000C
        .equiv TRISG13, 0x000D
        .equiv TRISG14, 0x000E
        .equiv TRISG15, 0x000F


;----- PORTG Bits -----------------------------------------------------
        .equiv RG0,     0x0000
        .equiv RG1,     0x0001
        .equiv RG2,     0x0002
        .equiv RG3,     0x0003
        .equiv RG6,     0x0006
        .equiv RG7,     0x0007
        .equiv RG8,     0x0008
        .equiv RG9,     0x0009
        .equiv RG12,    0x000C
        .equiv RG13,    0x000D
        .equiv RG14,    0x000E
        .equiv RG15,    0x000F


;----- LATG Bits -----------------------------------------------------
        .equiv LATG0,   0x0000
        .equiv LATG1,   0x0001
        .equiv LATG2,   0x0002
        .equiv LATG3,   0x0003
        .equiv LATG6,   0x0006
        .equiv LATG7,   0x0007
        .equiv LATG8,   0x0008
        .equiv LATG9,   0x0009
        .equiv LATG12,  0x000C
        .equiv LATG13,  0x000D
        .equiv LATG14,  0x000E
        .equiv LATG15,  0x000F


;----- C1RXF0SID Bits -----------------------------------------------------
        .equiv EXIDE,   0x0000
        .equiv SID0,    0x0002
        .equiv SID1,    0x0003
        .equiv SID2,    0x0004
        .equiv SID3,    0x0005
        .equiv SID4,    0x0006
        .equiv SID5,    0x0007
        .equiv SID6_C1RXF0SID, 0x0008
        .equiv SID7_C1RXF0SID, 0x0009
        .equiv SID8_C1RXF0SID, 0x000A
        .equiv SID9_C1RXF0SID, 0x000B
        .equiv SID10_C1RXF0SID, 0x000C



;----- C1RXF0EIDH Bits -----------------------------------------------------
        .equiv EID6,    0x0000
        .equiv EID7,    0x0001
        .equiv EID8,    0x0002
        .equiv EID9,    0x0003
        .equiv EID10,   0x0004
        .equiv EID11,   0x0005
        .equiv EID12,   0x0006
        .equiv EID13,   0x0007
        .equiv EID14_C1RXF0EIDH, 0x0008
        .equiv EID15_C1RXF0EIDH, 0x0009
        .equiv EID16_C1RXF0EIDH, 0x000A
        .equiv EID17_C1RXF0EIDH, 0x000B


;----- C1RXF0EIDL Bits -----------------------------------------------------
        .equiv EID0,    0x000A
        .equiv EID1,    0x000B
        .equiv EID2,    0x000C
        .equiv EID3,    0x000D
        .equiv EID4,    0x000E
        .equiv EID5,    0x000F



;----- C1RXF1SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RXF1SID, 0x0008
        .equiv SID7_C1RXF1SID, 0x0009
        .equiv SID8_C1RXF1SID, 0x000A
        .equiv SID9_C1RXF1SID, 0x000B
        .equiv SID10_C1RXF1SID, 0x000C



;----- C1RXF1EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RXF1EIDH, 0x0008
        .equiv EID15_C1RXF1EIDH, 0x0009
        .equiv EID16_C1RXF1EIDH, 0x000A
        .equiv EID17_C1RXF1EIDH, 0x000B


;----- C1RXF1EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C1RXF2SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RXF2SID, 0x0008
        .equiv SID7_C1RXF2SID, 0x0009
        .equiv SID8_C1RXF2SID, 0x000A
        .equiv SID9_C1RXF2SID, 0x000B
        .equiv SID10_C1RXF2SID, 0x000C



;----- C1RXF2EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RXF2EIDH, 0x0008
        .equiv EID15_C1RXF2EIDH, 0x0009
        .equiv EID16_C1RXF2EIDH, 0x000A
        .equiv EID17_C1RXF2EIDH, 0x000B


;----- C1RXF2EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C1RXF3SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RXF3SID, 0x0008
        .equiv SID7_C1RXF3SID, 0x0009
        .equiv SID8_C1RXF3SID, 0x000A
        .equiv SID9_C1RXF3SID, 0x000B
        .equiv SID10_C1RXF3SID, 0x000C



;----- C1RXF3EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RXF3EIDH, 0x0008
        .equiv EID15_C1RXF3EIDH, 0x0009
        .equiv EID16_C1RXF3EIDH, 0x000A
        .equiv EID17_C1RXF3EIDH, 0x000B


;----- C1RXF3EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C1RXF4SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RXF4SID, 0x0008
        .equiv SID7_C1RXF4SID, 0x0009
        .equiv SID8_C1RXF4SID, 0x000A
        .equiv SID9_C1RXF4SID, 0x000B
        .equiv SID10_C1RXF4SID, 0x000C



;----- C1RXF4EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RXF4EIDH, 0x0008
        .equiv EID15_C1RXF4EIDH, 0x0009
        .equiv EID16_C1RXF4EIDH, 0x000A
        .equiv EID17_C1RXF4EIDH, 0x000B


;----- C1RXF4EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C1RXF5SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RXF5SID, 0x0008
        .equiv SID7_C1RXF5SID, 0x0009
        .equiv SID8_C1RXF5SID, 0x000A
        .equiv SID9_C1RXF5SID, 0x000B
        .equiv SID10_C1RXF5SID, 0x000C



;----- C1RXF5EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RXF5EIDH, 0x0008
        .equiv EID15_C1RXF5EIDH, 0x0009
        .equiv EID16_C1RXF5EIDH, 0x000A
        .equiv EID17_C1RXF5EIDH, 0x000B


;----- C1RXF5EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C1RXM0SID Bits -----------------------------------------------------
        .equiv MIDE,    0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RXM0SID, 0x0008
        .equiv SID7_C1RXM0SID, 0x0009
        .equiv SID8_C1RXM0SID, 0x000A
        .equiv SID9_C1RXM0SID, 0x000B
        .equiv SID10_C1RXM0SID, 0x000C



;----- C1RXM0EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RXM0EIDH, 0x0008
        .equiv EID15_C1RXM0EIDH, 0x0009
        .equiv EID16_C1RXM0EIDH, 0x000A
        .equiv EID17_C1RXM0EIDH, 0x000B


;----- C1RXM0EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C1RXM1SID Bits -----------------------------------------------------
       ;.equiv MIDE,    0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RXM1SID, 0x0008
        .equiv SID7_C1RXM1SID, 0x0009
        .equiv SID8_C1RXM1SID, 0x000A
        .equiv SID9_C1RXM1SID, 0x000B
        .equiv SID10_C1RXM1SID, 0x000C



;----- C1RXM1EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RXM1EIDH, 0x0008
        .equiv EID15_C1RXM1EIDH, 0x0009
        .equiv EID16_C1RXM1EIDH, 0x000A
        .equiv EID17_C1RXM1EIDH, 0x000B


;----- C1RXM1EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C1TX2SID Bits -----------------------------------------------------
        .equiv TXIDE,   0x0000
        .equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1TX2SID, 0x000B
        .equiv SID7_C1TX2SID, 0x000C
        .equiv SID8_C1TX2SID, 0x000D
        .equiv SID9_C1TX2SID, 0x000E
        .equiv SID10_C1TX2SID, 0x000F



;----- C1TX2EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1TX2EID, 0x000C
        .equiv EID15_C1TX2EID, 0x000D
        .equiv EID16_C1TX2EID, 0x000E
        .equiv EID17_C1TX2EID, 0x000F



;----- C1TX2DLC Bits -----------------------------------------------------
        .equiv TXRB0,   0x0007
        .equiv TXRB1,   0x0008
        .equiv TXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C1TX2B1 Bits -----------------------------------------------------
        .equiv TXB2D00, 0x0000
        .equiv TXB2D01, 0x0001
        .equiv TXB2D02, 0x0002
        .equiv TXB2D03, 0x0003
        .equiv TXB2D04, 0x0004
        .equiv TXB2D05, 0x0005
        .equiv TXB2D06, 0x0006
        .equiv TXB2D07, 0x0007
        .equiv TXB2D10, 0x0008
        .equiv TXB2D11, 0x0009
        .equiv TXB2D12, 0x000A
        .equiv TXB2D13, 0x000B
        .equiv TXB2D14, 0x000C
        .equiv TXB2D15, 0x000D
        .equiv TXB2D16, 0x000E
        .equiv TXB2D17, 0x000F


;----- C1TX2B2 Bits -----------------------------------------------------
        .equiv TXB2D20, 0x0000
        .equiv TXB2D21, 0x0001
        .equiv TXB2D22, 0x0002
        .equiv TXB2D23, 0x0003
        .equiv TXB2D24, 0x0004
        .equiv TXB2D25, 0x0005
        .equiv TXB2D26, 0x0006
        .equiv TXB2D27, 0x0007
        .equiv TXB2D30, 0x0008
        .equiv TXB2D31, 0x0009
        .equiv TXB2D32, 0x000A
        .equiv TXB2D33, 0x000B
        .equiv TXB2D34, 0x000C
        .equiv TXB2D35, 0x000D
        .equiv TXB2D36, 0x000E
        .equiv TXB2D37, 0x000F


;----- C1TX2B3 Bits -----------------------------------------------------
        .equiv TX2B2D40, 0x0000
        .equiv TX2B2D41, 0x0001
        .equiv TX2B2D42, 0x0002
        .equiv TX2B2D43, 0x0003
        .equiv TX2B2D44, 0x0004
        .equiv TX2B2D45, 0x0005
        .equiv TX2B2D46, 0x0006
        .equiv TX2B2D47, 0x0007
        .equiv TXB2D50, 0x0008
        .equiv TXB2D51, 0x0009
        .equiv TXB2D52, 0x000A
        .equiv TXB2D53, 0x000B
        .equiv TXB2D54, 0x000C
        .equiv TXB2D55, 0x000D
        .equiv TXB2D56, 0x000E
        .equiv TXB2D57, 0x000F


;----- C1TX2B4 Bits -----------------------------------------------------
        .equiv TX2B2D60, 0x0000
        .equiv TX2B2D61, 0x0001
        .equiv TX2B2D62, 0x0002
        .equiv TX2B2D63, 0x0003
        .equiv TX2B2D64, 0x0004
        .equiv TX2B2D65, 0x0005
        .equiv TX2B2D66, 0x0006
        .equiv TX2B2D67, 0x0007
        .equiv TX2B2D70, 0x0008
        .equiv TX2B2D71, 0x0009
        .equiv TX2B2D72, 0x000A
        .equiv TX2B2D73, 0x000B
        .equiv TX2B2D74, 0x000C
        .equiv TX2B2D75, 0x000D
        .equiv TX2B2D76, 0x000E
        .equiv TX2B2D77, 0x000F


;----- C1TX2CON Bits -----------------------------------------------------
        .equiv TXREQ,   0x0003
        .equiv TXERR,   0x0004
        .equiv TXLARB,  0x0005
        .equiv TXABT,   0x0006

        .equiv TXPRI0,  0x0000
        .equiv TXPRI1,  0x0001


;----- C1TX1SID Bits -----------------------------------------------------
       ;.equiv TXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1TX1SID, 0x000B
        .equiv SID7_C1TX1SID, 0x000C
        .equiv SID8_C1TX1SID, 0x000D
        .equiv SID9_C1TX1SID, 0x000E
        .equiv SID10_C1TX1SID, 0x000F



;----- C1TX1EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1TX1EID, 0x000C
        .equiv EID15_C1TX1EID, 0x000D
        .equiv EID16_C1TX1EID, 0x000E
        .equiv EID17_C1TX1EID, 0x000F



;----- C1TX1DLC Bits -----------------------------------------------------
       ;.equiv TXRB0,   0x0007
       ;.equiv TXRB1,   0x0008
       ;.equiv TXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F


        .equiv DLC0_C1TX1DLC, 0x0003
        .equiv DLC1_C1TX1DLC, 0x0004
        .equiv DLC2_C1TX1DLC, 0x0005
        .equiv DLC3_C1TX1DLC, 0x0006


;----- C1TX1B1 Bits -----------------------------------------------------
        .equiv TXB1D00, 0x0000
        .equiv TXB1D01, 0x0001
        .equiv TXB1D02, 0x0002
        .equiv TXB1D03, 0x0003
        .equiv TXB1D04, 0x0004
        .equiv TXB1D05, 0x0005
        .equiv TXB1D06, 0x0006
        .equiv TXB1D07, 0x0007
        .equiv TXB1D10, 0x0008
        .equiv TXB1D11, 0x0009
        .equiv TXB1D12, 0x000A
        .equiv TXB1D13, 0x000B
        .equiv TXB1D14, 0x000C
        .equiv TXB1D15, 0x000D
        .equiv TXB1D16, 0x000E
        .equiv TXB1D17, 0x000F


;----- C1TX1B2 Bits -----------------------------------------------------
        .equiv TXB1D20, 0x0000
        .equiv TXB1D21, 0x0001
        .equiv TXB1D22, 0x0002
        .equiv TXB1D23, 0x0003
        .equiv TXB1D24, 0x0004
        .equiv TXB1D25, 0x0005
        .equiv TXB1D26, 0x0006
        .equiv TXB1D27, 0x0007
        .equiv TXB1D30, 0x0008
        .equiv TXB1D31, 0x0009
        .equiv TXB1D32, 0x000A
        .equiv TXB1D33, 0x000B
        .equiv TXB1D34, 0x000C
        .equiv TXB1D35, 0x000D
        .equiv TXB1D36, 0x000E
        .equiv TXB1D37, 0x000F


;----- C1TX1B3 Bits -----------------------------------------------------
        .equiv TXB1D40, 0x0000
        .equiv TXB1D41, 0x0001
        .equiv TXB1D42, 0x0002
        .equiv TXB1D43, 0x0003
        .equiv TXB1D44, 0x0004
        .equiv TXB1D45, 0x0005
        .equiv TXB1D46, 0x0006
        .equiv TXB1D47, 0x0007
        .equiv TXB1D50, 0x0008
        .equiv TXB1D51, 0x0009
        .equiv TXB1D52, 0x000A
        .equiv TXB1D53, 0x000B
        .equiv TXB1D54, 0x000C
        .equiv TXB1D55, 0x000D
        .equiv TXB1D56, 0x000E
        .equiv TXB1D57, 0x000F


;----- C1TX1B4 Bits -----------------------------------------------------
        .equiv TXB1D60, 0x0000
        .equiv TXB1D61, 0x0001
        .equiv TXB1D62, 0x0002
        .equiv TXB1D63, 0x0003
        .equiv TXB1D64, 0x0004
        .equiv TXB1D65, 0x0005
        .equiv TXB1D66, 0x0006
        .equiv TXB1D67, 0x0007
        .equiv TXB1D70, 0x0008
        .equiv TXB1D71, 0x0009
        .equiv TXB1D72, 0x000A
        .equiv TXB1D73, 0x000B
        .equiv TXB1D74, 0x000C
        .equiv TXB1D75, 0x000D
        .equiv TXB1D76, 0x000E
        .equiv TXB1D77, 0x000F


;----- C1TX1CON Bits -----------------------------------------------------
       ;.equiv TXREQ,   0x0003
       ;.equiv TXERR,   0x0004
       ;.equiv TXLARB,  0x0005
       ;.equiv TXABT,   0x0006

       ;.equiv TXPRI0,  0x0000
       ;.equiv TXPRI1,  0x0001


;----- C1TX0SID Bits -----------------------------------------------------
       ;.equiv TXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1TX0SID, 0x000B
        .equiv SID7_C1TX0SID, 0x000C
        .equiv SID8_C1TX0SID, 0x000D
        .equiv SID9_C1TX0SID, 0x000E
        .equiv SID10_C1TX0SID, 0x000F



;----- C1TX0EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1TX0EID, 0x000C
        .equiv EID15_C1TX0EID, 0x000D
        .equiv EID16_C1TX0EID, 0x000E
        .equiv EID17_C1TX0EID, 0x000F



;----- C1TX0DLC Bits -----------------------------------------------------
       ;.equiv TXRB0,   0x0007
       ;.equiv TXRB1,   0x0008
       ;.equiv TXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F


        .equiv DLC0_C1TX0DLC, 0x0003
        .equiv DLC1_C1TX0DLC, 0x0004
        .equiv DLC2_C1TX0DLC, 0x0005
        .equiv DLC3_C1TX0DLC, 0x0006


;----- C1TX0B1 Bits -----------------------------------------------------
        .equiv TXB0D00, 0x0000
        .equiv TXB0D01, 0x0001
        .equiv TXB0D02, 0x0002
        .equiv TXB0D03, 0x0003
        .equiv TXB0D04, 0x0004
        .equiv TXB0D05, 0x0005
        .equiv TXB0D06, 0x0006
        .equiv TXB0D07, 0x0007
        .equiv TXB0D10, 0x0008
        .equiv TXB0D11, 0x0009
        .equiv TXB0D12, 0x000A
        .equiv TXB0D13, 0x000B
        .equiv TXB0D14, 0x000C
        .equiv TXB0D15, 0x000D
        .equiv TXB0D16, 0x000E
        .equiv TXB0D17, 0x000F


;----- C1TX0B2 Bits -----------------------------------------------------
        .equiv TXB0D20, 0x0000
        .equiv TXB0D21, 0x0001
        .equiv TXB0D22, 0x0002
        .equiv TXB0D23, 0x0003
        .equiv TXB0D24, 0x0004
        .equiv TXB0D25, 0x0005
        .equiv TXB0D26, 0x0006
        .equiv TXB0D27, 0x0007
        .equiv TXB0D30, 0x0008
        .equiv TXB0D31, 0x0009
        .equiv TXB0D32, 0x000A
        .equiv TXB0D33, 0x000B
        .equiv TXB0D34, 0x000C
        .equiv TXB0D35, 0x000D
        .equiv TXB0D36, 0x000E
        .equiv TXB0D37, 0x000F


;----- C1TX0B3 Bits -----------------------------------------------------
        .equiv TXB0D40, 0x0000
        .equiv TXB0D41, 0x0001
        .equiv TXB0D42, 0x0002
        .equiv TXB0D43, 0x0003
        .equiv TXB0D44, 0x0004
        .equiv TXB0D45, 0x0005
        .equiv TXB0D46, 0x0006
        .equiv TXB0D47, 0x0007
        .equiv TXB0D50, 0x0008
        .equiv TXB0D51, 0x0009
        .equiv TXB0D52, 0x000A
        .equiv TXB0D53, 0x000B
        .equiv TXB0D54, 0x000C
        .equiv TXB0D55, 0x000D
        .equiv TXB0D56, 0x000E
        .equiv TXB0D57, 0x000F


;----- C1TX0B4 Bits -----------------------------------------------------
        .equiv TXB0D60, 0x0000
        .equiv TXB0D61, 0x0001
        .equiv TXB0D62, 0x0002
        .equiv TXB0D63, 0x0003
        .equiv TXB0D64, 0x0004
        .equiv TXB0D65, 0x0005
        .equiv TXB0D66, 0x0006
        .equiv TXB0D67, 0x0007
        .equiv TXB0D70, 0x0008
        .equiv TXB0D71, 0x0009
        .equiv TXB0D72, 0x000A
        .equiv TXB0D73, 0x000B
        .equiv TXB0D74, 0x000C
        .equiv TXB0D75, 0x000D
        .equiv TXB0D76, 0x000E
        .equiv TXB0D77, 0x000F


;----- C1TX0CON Bits -----------------------------------------------------
       ;.equiv TXREQ,   0x0003
       ;.equiv TXERR,   0x0004
       ;.equiv TXLARB,  0x0005
       ;.equiv TXABT,   0x0006

       ;.equiv TXPRI0,  0x0000
       ;.equiv TXPRI1,  0x0001


;----- C1RX1SID Bits -----------------------------------------------------
        .equiv RXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RX1SID, 0x0008
        .equiv SID7_C1RX1SID, 0x0009
        .equiv SID8_C1RX1SID, 0x000A
        .equiv SID9_C1RX1SID, 0x000B
        .equiv SID10_C1RX1SID, 0x000C



;----- C1RX1EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RX1EID, 0x0008
        .equiv EID15_C1RX1EID, 0x0009
        .equiv EID16_C1RX1EID, 0x000A
        .equiv EID17_C1RX1EID, 0x000B


;----- C1RX1DLC Bits -----------------------------------------------------
        .equiv RXRB0,   0x0004
        .equiv RXRB1,   0x0008
        .equiv RXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F


        .equiv DLC0_C1RX1DLC, 0x0000
        .equiv DLC1_C1RX1DLC, 0x0001
        .equiv DLC2_C1RX1DLC, 0x0002
        .equiv DLC3_C1RX1DLC, 0x0003


;----- C1RX1B1 Bits -----------------------------------------------------
       ;.equiv TXB1D00, 0x0000
       ;.equiv TXB1D01, 0x0001
       ;.equiv TXB1D02, 0x0002
       ;.equiv TXB1D03, 0x0003
       ;.equiv TXB1D04, 0x0004
       ;.equiv TXB1D05, 0x0005
       ;.equiv TXB1D06, 0x0006
       ;.equiv TXB1D07, 0x0007
       ;.equiv TXB1D10, 0x0008
       ;.equiv TXB1D11, 0x0009
       ;.equiv TXB1D12, 0x000A
       ;.equiv TXB1D13, 0x000B
       ;.equiv TXB1D14, 0x000C
       ;.equiv TXB1D15, 0x000D
       ;.equiv TXB1D16, 0x000E
       ;.equiv TXB1D17, 0x000F


;----- C1RX1B2 Bits -----------------------------------------------------
       ;.equiv TXB1D20, 0x0000
       ;.equiv TXB1D21, 0x0001
       ;.equiv TXB1D22, 0x0002
       ;.equiv TXB1D23, 0x0003
       ;.equiv TXB1D24, 0x0004
       ;.equiv TXB1D25, 0x0005
       ;.equiv TXB1D26, 0x0006
       ;.equiv TXB1D27, 0x0007
       ;.equiv TXB1D30, 0x0008
       ;.equiv TXB1D31, 0x0009
       ;.equiv TXB1D32, 0x000A
       ;.equiv TXB1D33, 0x000B
       ;.equiv TXB1D34, 0x000C
       ;.equiv TXB1D35, 0x000D
       ;.equiv TXB1D36, 0x000E
       ;.equiv TXB1D37, 0x000F


;----- C1RX1B3 Bits -----------------------------------------------------
       ;.equiv TXB1D40, 0x0000
       ;.equiv TXB1D41, 0x0001
       ;.equiv TXB1D42, 0x0002
       ;.equiv TXB1D43, 0x0003
       ;.equiv TXB1D44, 0x0004
       ;.equiv TXB1D45, 0x0005
       ;.equiv TXB1D46, 0x0006
       ;.equiv TXB1D47, 0x0007
       ;.equiv TXB1D50, 0x0008
       ;.equiv TXB1D51, 0x0009
       ;.equiv TXB1D52, 0x000A
       ;.equiv TXB1D53, 0x000B
       ;.equiv TXB1D54, 0x000C
       ;.equiv TXB1D55, 0x000D
       ;.equiv TXB1D56, 0x000E
       ;.equiv TXB1D57, 0x000F


;----- C1RX1B4 Bits -----------------------------------------------------
       ;.equiv TXB1D60, 0x0000
       ;.equiv TXB1D61, 0x0001
       ;.equiv TXB1D62, 0x0002
       ;.equiv TXB1D63, 0x0003
       ;.equiv TXB1D64, 0x0004
       ;.equiv TXB1D65, 0x0005
       ;.equiv TXB1D66, 0x0006
       ;.equiv TXB1D67, 0x0007
       ;.equiv TXB1D70, 0x0008
       ;.equiv TXB1D71, 0x0009
       ;.equiv TXB1D72, 0x000A
       ;.equiv TXB1D73, 0x000B
       ;.equiv TXB1D74, 0x000C
       ;.equiv TXB1D75, 0x000D
       ;.equiv TXB1D76, 0x000E
       ;.equiv TXB1D77, 0x000F


;----- C1RX1CON Bits -----------------------------------------------------
        .equiv RXRTRRO, 0x0003
        .equiv RXFUL,   0x0007

        .equiv FILHIT0, 0x0000
        .equiv FILHIT1, 0x0001
        .equiv FILHIT2, 0x0002


;----- C1RX0SID Bits -----------------------------------------------------
       ;.equiv RXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C1RX0SID, 0x0008
        .equiv SID7_C1RX0SID, 0x0009
        .equiv SID8_C1RX0SID, 0x000A
        .equiv SID9_C1RX0SID, 0x000B
        .equiv SID10_C1RX0SID, 0x000C



;----- C1RX0EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C1RX0EID, 0x0008
        .equiv EID15_C1RX0EID, 0x0009
        .equiv EID16_C1RX0EID, 0x000A
        .equiv EID17_C1RX0EID, 0x000B


;----- C1RX0DLC Bits -----------------------------------------------------
       ;.equiv RXRB0,   0x0004
       ;.equiv RXRB1,   0x0008
       ;.equiv RXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F


        .equiv DLC0_C1RX0DLC, 0x0000
        .equiv DLC1_C1RX0DLC, 0x0001
        .equiv DLC2_C1RX0DLC, 0x0002
        .equiv DLC3_C1RX0DLC, 0x0003


;----- C1RX0B1 Bits -----------------------------------------------------
       ;.equiv TXB0D00, 0x0000
       ;.equiv TXB0D01, 0x0001
       ;.equiv TXB0D02, 0x0002
       ;.equiv TXB0D03, 0x0003
       ;.equiv TXB0D04, 0x0004
       ;.equiv TXB0D05, 0x0005
       ;.equiv TXB0D06, 0x0006
       ;.equiv TXB0D07, 0x0007
       ;.equiv TXB0D10, 0x0008
       ;.equiv TXB0D11, 0x0009
       ;.equiv TXB0D12, 0x000A
       ;.equiv TXB0D13, 0x000B
       ;.equiv TXB0D14, 0x000C
       ;.equiv TXB0D15, 0x000D
       ;.equiv TXB0D16, 0x000E
       ;.equiv TXB0D17, 0x000F


;----- C1RX0B2 Bits -----------------------------------------------------
       ;.equiv TXB0D20, 0x0000
       ;.equiv TXB0D21, 0x0001
       ;.equiv TXB0D22, 0x0002
       ;.equiv TXB0D23, 0x0003
       ;.equiv TXB0D24, 0x0004
       ;.equiv TXB0D25, 0x0005
       ;.equiv TXB0D26, 0x0006
       ;.equiv TXB0D27, 0x0007
       ;.equiv TXB0D30, 0x0008
       ;.equiv TXB0D31, 0x0009
       ;.equiv TXB0D32, 0x000A
       ;.equiv TXB0D33, 0x000B
       ;.equiv TXB0D34, 0x000C
       ;.equiv TXB0D35, 0x000D
       ;.equiv TXB0D36, 0x000E
       ;.equiv TXB0D37, 0x000F


;----- C1RX0B3 Bits -----------------------------------------------------
       ;.equiv TXB0D40, 0x0000
       ;.equiv TXB0D41, 0x0001
       ;.equiv TXB0D42, 0x0002
       ;.equiv TXB0D43, 0x0003
       ;.equiv TXB0D44, 0x0004
       ;.equiv TXB0D45, 0x0005
       ;.equiv TXB0D46, 0x0006
       ;.equiv TXB0D47, 0x0007
       ;.equiv TXB0D50, 0x0008
       ;.equiv TXB0D51, 0x0009
       ;.equiv TXB0D52, 0x000A
       ;.equiv TXB0D53, 0x000B
       ;.equiv TXB0D54, 0x000C
       ;.equiv TXB0D55, 0x000D
       ;.equiv TXB0D56, 0x000E
       ;.equiv TXB0D57, 0x000F


;----- C1RX0B4 Bits -----------------------------------------------------
       ;.equiv TXB0D60, 0x0000
       ;.equiv TXB0D61, 0x0001
       ;.equiv TXB0D62, 0x0002
       ;.equiv TXB0D63, 0x0003
       ;.equiv TXB0D64, 0x0004
       ;.equiv TXB0D65, 0x0005
       ;.equiv TXB0D66, 0x0006
       ;.equiv TXB0D67, 0x0007
       ;.equiv TXB0D70, 0x0008
       ;.equiv TXB0D71, 0x0009
       ;.equiv TXB0D72, 0x000A
       ;.equiv TXB0D73, 0x000B
       ;.equiv TXB0D74, 0x000C
       ;.equiv TXB0D75, 0x000D
       ;.equiv TXB0D76, 0x000E
       ;.equiv TXB0D77, 0x000F


;----- C1RX0CON Bits -----------------------------------------------------
       ;.equiv FILHIT0, 0x0000
        .equiv JTOFF,   0x0001
        .equiv DBEN,    0x0002
       ;.equiv RXRTRRO, 0x0003
       ;.equiv RXFUL,   0x0007

        .equiv FILHIT,  0x0000
        .equiv RXB0DBEN, 0x0002


;----- C1CTRL Bits -----------------------------------------------------
        .equiv CANCKS,  0x000B
        .equiv ABAT,    0x000C
        .equiv CANSIDL, 0x000D
        .equiv CANFRZ,  0x000E
        .equiv CANCAP,  0x000F

        .equiv ICODE0,  0x0001
        .equiv ICODE1,  0x0002
        .equiv ICODE2,  0x0003
        .equiv OPMODE0, 0x0005
        .equiv OPMODE1, 0x0006
        .equiv OPMODE2, 0x0007
        .equiv REQOP0,  0x0008
        .equiv REQOP1,  0x0009
        .equiv REQOP2,  0x000A
        .equiv CSIDL,   0x000D


;----- C1CFG1 Bits -----------------------------------------------------
        .equiv BRP0,    0x0000
        .equiv BRP1,    0x0001
        .equiv BRP2,    0x0002
        .equiv BRP3,    0x0003
        .equiv BRP4,    0x0004
        .equiv BRP5,    0x0005
        .equiv SJW0,    0x0006
        .equiv SJW1,    0x0007


;----- C1CFG2 Bits -----------------------------------------------------
        .equiv SAM,     0x0006
        .equiv SEG2PHTS, 0x0007
        .equiv WAKFIL,  0x000E

        .equiv PRSEG0,  0x0000
        .equiv PRSEG1,  0x0001
        .equiv PRSEG2,  0x0002
        .equiv SEG1PH0, 0x0003
        .equiv SEG1PH1, 0x0004
        .equiv SEG1PH2, 0x0005
        .equiv SEG2PH0, 0x0008
        .equiv SEG2PH1, 0x0009
        .equiv SEG2PH2, 0x000A


;----- C1INTF Bits -----------------------------------------------------
        .equiv RXB0IF,  0x0000
        .equiv RXB1IF,  0x0001
        .equiv TXB0IF,  0x0002
        .equiv TXB1IF,  0x0003
        .equiv TXB2IF,  0x0004
        .equiv ERRIF,   0x0005
        .equiv WAKIF,   0x0006
        .equiv IRXIF,   0x0007
        .equiv EWARN,   0x0008
        .equiv RXWARN,  0x0009
        .equiv TXWARN,  0x000A
        .equiv RXBP,    0x000B
        .equiv TXBP,    0x000C
        .equiv TXBO,    0x000D
        .equiv RXB1OVFL, 0x000E
        .equiv RXB0OVFL, 0x000F

        .equiv RX0IF,   0x0000
        .equiv RX1IF,   0x0001
        .equiv TX0IF,   0x0002
        .equiv TX1IF,   0x0003
        .equiv TX2IF,   0x0004
        .equiv IVRIF,   0x0007
        .equiv RXWAR,   0x0009
        .equiv TXWAR,   0x000A
        .equiv RXEP,    0x000B
        .equiv TXEP,    0x000C
        .equiv RX1OVR,  0x000E
        .equiv RX0OVR,  0x000F


;----- C1INTE Bits -----------------------------------------------------
        .equiv RXB0IE,  0x0000
        .equiv RXB1IE,  0x0001
        .equiv TXB0IE,  0x0002
        .equiv TXB1IE,  0x0003
        .equiv TXB2IE,  0x0004
        .equiv ERRIE,   0x0005
        .equiv WAKIE,   0x0006
        .equiv IRXIE,   0x0007

        .equiv RX0IE,   0x0000
        .equiv RX1IE,   0x0001
        .equiv TX0IE,   0x0002
        .equiv TX1IE,   0x0003
        .equiv TX2IE,   0x0004
        .equiv IVRIE,   0x0007


;----- C1EC Bits -----------------------------------------------------
        .equiv REC0,    0x0000
        .equiv REC1,    0x0001
        .equiv REC2,    0x0002
        .equiv REC3,    0x0003
        .equiv REC4,    0x0004
        .equiv REC5,    0x0005
        .equiv REC6,    0x0006
        .equiv REC7,    0x0007
        .equiv TEC0,    0x0008
        .equiv TEC1,    0x0009
        .equiv TEC2,    0x000A
        .equiv TEC3,    0x000B
        .equiv TEC4,    0x000C
        .equiv TEC5,    0x000D
        .equiv TEC6,    0x000E
        .equiv TEC7,    0x000F


;----- C2RXF0SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RXF0SID, 0x0008
        .equiv SID7_C2RXF0SID, 0x0009
        .equiv SID8_C2RXF0SID, 0x000A
        .equiv SID9_C2RXF0SID, 0x000B
        .equiv SID10_C2RXF0SID, 0x000C



;----- C2RXF0EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RXF0EIDH, 0x0008
        .equiv EID15_C2RXF0EIDH, 0x0009
        .equiv EID16_C2RXF0EIDH, 0x000A
        .equiv EID17_C2RXF0EIDH, 0x000B


;----- C2RXF0EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2RXF1SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RXF1SID, 0x0008
        .equiv SID7_C2RXF1SID, 0x0009
        .equiv SID8_C2RXF1SID, 0x000A
        .equiv SID9_C2RXF1SID, 0x000B
        .equiv SID10_C2RXF1SID, 0x000C



;----- C2RXF1EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RXF1EIDH, 0x0008
        .equiv EID15_C2RXF1EIDH, 0x0009
        .equiv EID16_C2RXF1EIDH, 0x000A
        .equiv EID17_C2RXF1EIDH, 0x000B


;----- C2RXF1EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2RXF2SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RXF2SID, 0x0008
        .equiv SID7_C2RXF2SID, 0x0009
        .equiv SID8_C2RXF2SID, 0x000A
        .equiv SID9_C2RXF2SID, 0x000B
        .equiv SID10_C2RXF2SID, 0x000C



;----- C2RXF2EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RXF2EIDH, 0x0008
        .equiv EID15_C2RXF2EIDH, 0x0009
        .equiv EID16_C2RXF2EIDH, 0x000A
        .equiv EID17_C2RXF2EIDH, 0x000B


;----- C2RXF2EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2RXF3SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RXF3SID, 0x0008
        .equiv SID7_C2RXF3SID, 0x0009
        .equiv SID8_C2RXF3SID, 0x000A
        .equiv SID9_C2RXF3SID, 0x000B
        .equiv SID10_C2RXF3SID, 0x000C



;----- C2RXF3EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RXF3EIDH, 0x0008
        .equiv EID15_C2RXF3EIDH, 0x0009
        .equiv EID16_C2RXF3EIDH, 0x000A
        .equiv EID17_C2RXF3EIDH, 0x000B


;----- C2RXF3EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2RXF4SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RXF4SID, 0x0008
        .equiv SID7_C2RXF4SID, 0x0009
        .equiv SID8_C2RXF4SID, 0x000A
        .equiv SID9_C2RXF4SID, 0x000B
        .equiv SID10_C2RXF4SID, 0x000C



;----- C2RXF4EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RXF4EIDH, 0x0008
        .equiv EID15_C2RXF4EIDH, 0x0009
        .equiv EID16_C2RXF4EIDH, 0x000A
        .equiv EID17_C2RXF4EIDH, 0x000B


;----- C2RXF4EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2RXF5SID Bits -----------------------------------------------------
       ;.equiv EXIDE,   0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RXF5SID, 0x0008
        .equiv SID7_C2RXF5SID, 0x0009
        .equiv SID8_C2RXF5SID, 0x000A
        .equiv SID9_C2RXF5SID, 0x000B
        .equiv SID10_C2RXF5SID, 0x000C



;----- C2RXF5EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RXF5EIDH, 0x0008
        .equiv EID15_C2RXF5EIDH, 0x0009
        .equiv EID16_C2RXF5EIDH, 0x000A
        .equiv EID17_C2RXF5EIDH, 0x000B


;----- C2RXF5EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2RXM0SID Bits -----------------------------------------------------
       ;.equiv MIDE,    0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RXM0SID, 0x0008
        .equiv SID7_C2RXM0SID, 0x0009
        .equiv SID8_C2RXM0SID, 0x000A
        .equiv SID9_C2RXM0SID, 0x000B
        .equiv SID10_C2RXM0SID, 0x000C



;----- C2RXM0EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RXM0EIDH, 0x0008
        .equiv EID15_C2RXM0EIDH, 0x0009
        .equiv EID16_C2RXM0EIDH, 0x000A
        .equiv EID17_C2RXM0EIDH, 0x000B


;----- C2RXM0EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2RXM1SID Bits -----------------------------------------------------
       ;.equiv MIDE,    0x0000
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RXM1SID, 0x0008
        .equiv SID7_C2RXM1SID, 0x0009
        .equiv SID8_C2RXM1SID, 0x000A
        .equiv SID9_C2RXM1SID, 0x000B
        .equiv SID10_C2RXM1SID, 0x000C



;----- C2RXM1EIDH Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RXM1EIDH, 0x0008
        .equiv EID15_C2RXM1EIDH, 0x0009
        .equiv EID16_C2RXM1EIDH, 0x000A
        .equiv EID17_C2RXM1EIDH, 0x000B


;----- C2RXM1EIDL Bits -----------------------------------------------------
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2TX2SID Bits -----------------------------------------------------
       ;.equiv TXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2TX2SID, 0x000B
        .equiv SID7_C2TX2SID, 0x000C
        .equiv SID8_C2TX2SID, 0x000D
        .equiv SID9_C2TX2SID, 0x000E
        .equiv SID10_C2TX2SID, 0x000F



;----- C2TX2EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2TX2EID, 0x000C
        .equiv EID15_C2TX2EID, 0x000D
        .equiv EID16_C2TX2EID, 0x000E
        .equiv EID17_C2TX2EID, 0x000F



;----- C2TX2DLC Bits -----------------------------------------------------
       ;.equiv TXRB0,   0x0007
       ;.equiv TXRB1,   0x0008
       ;.equiv TXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F


        .equiv DLC0_C2TX2DLC, 0x0003
        .equiv DLC1_C2TX2DLC, 0x0004
        .equiv DLC2_C2TX2DLC, 0x0005
        .equiv DLC3_C2TX2DLC, 0x0006


;----- C2TX2B1 Bits -----------------------------------------------------
       ;.equiv TXB2D00, 0x0000
       ;.equiv TXB2D01, 0x0001
       ;.equiv TXB2D02, 0x0002
       ;.equiv TXB2D03, 0x0003
       ;.equiv TXB2D04, 0x0004
       ;.equiv TXB2D05, 0x0005
       ;.equiv TXB2D06, 0x0006
       ;.equiv TXB2D07, 0x0007
       ;.equiv TXB2D10, 0x0008
       ;.equiv TXB2D11, 0x0009
       ;.equiv TXB2D12, 0x000A
       ;.equiv TXB2D13, 0x000B
       ;.equiv TXB2D14, 0x000C
       ;.equiv TXB2D15, 0x000D
       ;.equiv TXB2D16, 0x000E
       ;.equiv TXB2D17, 0x000F


;----- C2TX2B2 Bits -----------------------------------------------------
       ;.equiv TXB2D20, 0x0000
       ;.equiv TXB2D21, 0x0001
       ;.equiv TXB2D22, 0x0002
       ;.equiv TXB2D23, 0x0003
       ;.equiv TXB2D24, 0x0004
       ;.equiv TXB2D25, 0x0005
       ;.equiv TXB2D26, 0x0006
       ;.equiv TXB2D27, 0x0007
       ;.equiv TXB2D30, 0x0008
       ;.equiv TXB2D31, 0x0009
       ;.equiv TXB2D32, 0x000A
       ;.equiv TXB2D33, 0x000B
       ;.equiv TXB2D34, 0x000C
       ;.equiv TXB2D35, 0x000D
       ;.equiv TXB2D36, 0x000E
       ;.equiv TXB2D37, 0x000F


;----- C2TX2B3 Bits -----------------------------------------------------
        .equiv TXB2D40, 0x0000
        .equiv TXB2D41, 0x0001
        .equiv TXB2D42, 0x0002
        .equiv TXB2D43, 0x0003
        .equiv TXB2D44, 0x0004
        .equiv TXB2D45, 0x0005
        .equiv TXB2D46, 0x0006
        .equiv TXB2D47, 0x0007
       ;.equiv TXB2D50, 0x0008
       ;.equiv TXB2D51, 0x0009
       ;.equiv TXB2D52, 0x000A
       ;.equiv TXB2D53, 0x000B
       ;.equiv TXB2D54, 0x000C
       ;.equiv TXB2D55, 0x000D
       ;.equiv TXB2D56, 0x000E
       ;.equiv TXB2D57, 0x000F


;----- C2TX2B4 Bits -----------------------------------------------------
        .equiv TXB2D60, 0x0000
        .equiv TXB2D61, 0x0001
        .equiv TXB2D62, 0x0002
        .equiv TXB2D63, 0x0003
        .equiv TXB2D64, 0x0004
        .equiv TXB2D65, 0x0005
        .equiv TXB2D66, 0x0006
        .equiv TXB2D67, 0x0007
        .equiv TXB2D70, 0x0008
        .equiv TXB2D71, 0x0009
        .equiv TXB2D72, 0x000A
        .equiv TXB2D73, 0x000B
        .equiv TXB2D74, 0x000C
        .equiv TXB2D75, 0x000D
        .equiv TXB2D76, 0x000E
        .equiv TXB2D77, 0x000F


;----- C2TX2CON Bits -----------------------------------------------------
       ;.equiv TXREQ,   0x0003
       ;.equiv TXERR,   0x0004
       ;.equiv TXLARB,  0x0005
       ;.equiv TXABT,   0x0006

       ;.equiv TXPRI0,  0x0000
       ;.equiv TXPRI1,  0x0001


;----- C2TX1SID Bits -----------------------------------------------------
       ;.equiv TXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2TX1SID, 0x000B
        .equiv SID7_C2TX1SID, 0x000C
        .equiv SID8_C2TX1SID, 0x000D
        .equiv SID9_C2TX1SID, 0x000E
        .equiv SID10_C2TX1SID, 0x000F



;----- C2TX1EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2TX1EID, 0x000C
        .equiv EID15_C2TX1EID, 0x000D
        .equiv EID16_C2TX1EID, 0x000E
        .equiv EID17_C2TX1EID, 0x000F



;----- C2TX1DLC Bits -----------------------------------------------------
       ;.equiv TXRB0,   0x0007
       ;.equiv TXRB1,   0x0008
       ;.equiv TXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F


        .equiv DLC0_C2TX1DLC, 0x0003
        .equiv DLC1_C2TX1DLC, 0x0004
        .equiv DLC2_C2TX1DLC, 0x0005
        .equiv DLC3_C2TX1DLC, 0x0006


;----- C2TX1B1 Bits -----------------------------------------------------
       ;.equiv TXB1D00, 0x0000
       ;.equiv TXB1D01, 0x0001
       ;.equiv TXB1D02, 0x0002
       ;.equiv TXB1D03, 0x0003
       ;.equiv TXB1D04, 0x0004
       ;.equiv TXB1D05, 0x0005
       ;.equiv TXB1D06, 0x0006
       ;.equiv TXB1D07, 0x0007
       ;.equiv TXB1D10, 0x0008
       ;.equiv TXB1D11, 0x0009
       ;.equiv TXB1D12, 0x000A
       ;.equiv TXB1D13, 0x000B
       ;.equiv TXB1D14, 0x000C
       ;.equiv TXB1D15, 0x000D
       ;.equiv TXB1D16, 0x000E
       ;.equiv TXB1D17, 0x000F


;----- C2TX1B2 Bits -----------------------------------------------------
       ;.equiv TXB1D20, 0x0000
       ;.equiv TXB1D21, 0x0001
       ;.equiv TXB1D22, 0x0002
       ;.equiv TXB1D23, 0x0003
       ;.equiv TXB1D24, 0x0004
       ;.equiv TXB1D25, 0x0005
       ;.equiv TXB1D26, 0x0006
       ;.equiv TXB1D27, 0x0007
       ;.equiv TXB1D30, 0x0008
       ;.equiv TXB1D31, 0x0009
       ;.equiv TXB1D32, 0x000A
       ;.equiv TXB1D33, 0x000B
       ;.equiv TXB1D34, 0x000C
       ;.equiv TXB1D35, 0x000D
       ;.equiv TXB1D36, 0x000E
       ;.equiv TXB1D37, 0x000F


;----- C2TX1B3 Bits -----------------------------------------------------
       ;.equiv TXB1D40, 0x0000
       ;.equiv TXB1D41, 0x0001
       ;.equiv TXB1D42, 0x0002
       ;.equiv TXB1D43, 0x0003
       ;.equiv TXB1D44, 0x0004
       ;.equiv TXB1D45, 0x0005
       ;.equiv TXB1D46, 0x0006
       ;.equiv TXB1D47, 0x0007
       ;.equiv TXB1D50, 0x0008
       ;.equiv TXB1D51, 0x0009
       ;.equiv TXB1D52, 0x000A
       ;.equiv TXB1D53, 0x000B
       ;.equiv TXB1D54, 0x000C
       ;.equiv TXB1D55, 0x000D
       ;.equiv TXB1D56, 0x000E
       ;.equiv TXB1D57, 0x000F


;----- C2TX1B4 Bits -----------------------------------------------------
       ;.equiv TXB1D60, 0x0000
       ;.equiv TXB1D61, 0x0001
       ;.equiv TXB1D62, 0x0002
       ;.equiv TXB1D63, 0x0003
       ;.equiv TXB1D64, 0x0004
       ;.equiv TXB1D65, 0x0005
       ;.equiv TXB1D66, 0x0006
       ;.equiv TXB1D67, 0x0007
       ;.equiv TXB1D70, 0x0008
       ;.equiv TXB1D71, 0x0009
       ;.equiv TXB1D72, 0x000A
       ;.equiv TXB1D73, 0x000B
       ;.equiv TXB1D74, 0x000C
       ;.equiv TXB1D75, 0x000D
       ;.equiv TXB1D76, 0x000E
       ;.equiv TXB1D77, 0x000F


;----- C2TX1CON Bits -----------------------------------------------------
       ;.equiv TXREQ,   0x0003
       ;.equiv TXERR,   0x0004
       ;.equiv TXLARB,  0x0005
       ;.equiv TXABT,   0x0006

       ;.equiv TXPRI0,  0x0000
       ;.equiv TXPRI1,  0x0001


;----- C2TX0SID Bits -----------------------------------------------------
       ;.equiv TXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2TX0SID, 0x000B
        .equiv SID7_C2TX0SID, 0x000C
        .equiv SID8_C2TX0SID, 0x000D
        .equiv SID9_C2TX0SID, 0x000E
        .equiv SID10_C2TX0SID, 0x000F



;----- C2TX0EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2TX0EID, 0x000C
        .equiv EID15_C2TX0EID, 0x000D
        .equiv EID16_C2TX0EID, 0x000E
        .equiv EID17_C2TX0EID, 0x000F



;----- C2TX0DLC Bits -----------------------------------------------------
       ;.equiv TXRB0,   0x0007
       ;.equiv TXRB1,   0x0008
       ;.equiv TXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F



;----- C2TX0B1 Bits -----------------------------------------------------
       ;.equiv TXB0D00, 0x0000
       ;.equiv TXB0D01, 0x0001
       ;.equiv TXB0D02, 0x0002
       ;.equiv TXB0D03, 0x0003
       ;.equiv TXB0D04, 0x0004
       ;.equiv TXB0D05, 0x0005
       ;.equiv TXB0D06, 0x0006
       ;.equiv TXB0D07, 0x0007
       ;.equiv TXB0D10, 0x0008
       ;.equiv TXB0D11, 0x0009
       ;.equiv TXB0D12, 0x000A
       ;.equiv TXB0D13, 0x000B
       ;.equiv TXB0D14, 0x000C
       ;.equiv TXB0D15, 0x000D
       ;.equiv TXB0D16, 0x000E
       ;.equiv TXB0D17, 0x000F


;----- C2TX0B2 Bits -----------------------------------------------------
       ;.equiv TXB0D20, 0x0000
       ;.equiv TXB0D21, 0x0001
       ;.equiv TXB0D22, 0x0002
       ;.equiv TXB0D23, 0x0003
       ;.equiv TXB0D24, 0x0004
       ;.equiv TXB0D25, 0x0005
       ;.equiv TXB0D26, 0x0006
       ;.equiv TXB0D27, 0x0007
       ;.equiv TXB0D30, 0x0008
       ;.equiv TXB0D31, 0x0009
       ;.equiv TXB0D32, 0x000A
       ;.equiv TXB0D33, 0x000B
       ;.equiv TXB0D34, 0x000C
       ;.equiv TXB0D35, 0x000D
       ;.equiv TXB0D36, 0x000E
       ;.equiv TXB0D37, 0x000F


;----- C2TX0B3 Bits -----------------------------------------------------
       ;.equiv TXB0D40, 0x0000
       ;.equiv TXB0D41, 0x0001
       ;.equiv TXB0D42, 0x0002
       ;.equiv TXB0D43, 0x0003
       ;.equiv TXB0D44, 0x0004
       ;.equiv TXB0D45, 0x0005
       ;.equiv TXB0D46, 0x0006
       ;.equiv TXB0D47, 0x0007
       ;.equiv TXB0D50, 0x0008
       ;.equiv TXB0D51, 0x0009
       ;.equiv TXB0D52, 0x000A
       ;.equiv TXB0D53, 0x000B
       ;.equiv TXB0D54, 0x000C
       ;.equiv TXB0D55, 0x000D
       ;.equiv TXB0D56, 0x000E
       ;.equiv TXB0D57, 0x000F


;----- C2TX0B4 Bits -----------------------------------------------------
       ;.equiv TXB0D60, 0x0000
       ;.equiv TXB0D61, 0x0001
       ;.equiv TXB0D62, 0x0002
       ;.equiv TXB0D63, 0x0003
       ;.equiv TXB0D64, 0x0004
       ;.equiv TXB0D65, 0x0005
       ;.equiv TXB0D66, 0x0006
       ;.equiv TXB0D67, 0x0007
       ;.equiv TXB0D70, 0x0008
       ;.equiv TXB0D71, 0x0009
       ;.equiv TXB0D72, 0x000A
       ;.equiv TXB0D73, 0x000B
       ;.equiv TXB0D74, 0x000C
       ;.equiv TXB0D75, 0x000D
       ;.equiv TXB0D76, 0x000E
       ;.equiv TXB0D77, 0x000F


;----- C2TX0CON Bits -----------------------------------------------------
       ;.equiv TXREQ,   0x0003
       ;.equiv TXERR,   0x0004
       ;.equiv TXLARB,  0x0005
       ;.equiv TXABT,   0x0006

       ;.equiv TXPRI0,  0x0000
       ;.equiv TXPRI1,  0x0001


;----- C2RX1SID Bits -----------------------------------------------------
       ;.equiv RXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RX1SID, 0x0008
        .equiv SID7_C2RX1SID, 0x0009
        .equiv SID8_C2RX1SID, 0x000A
        .equiv SID9_C2RX1SID, 0x000B
        .equiv SID10_C2RX1SID, 0x000C



;----- C2RX1EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RX1EID, 0x0008
        .equiv EID15_C2RX1EID, 0x0009
        .equiv EID16_C2RX1EID, 0x000A
        .equiv EID17_C2RX1EID, 0x000B


;----- C2RX1DLC Bits -----------------------------------------------------
       ;.equiv RXRB0,   0x0004
       ;.equiv RXRB1,   0x0008
       ;.equiv RXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F


        .equiv DLC0_C2RX1DLC, 0x0000
        .equiv DLC1_C2RX1DLC, 0x0001
        .equiv DLC2_C2RX1DLC, 0x0002
        .equiv DLC3_C2RX1DLC, 0x0003


;----- C2RX1B1 Bits -----------------------------------------------------
       ;.equiv TXB1D00, 0x0000
       ;.equiv TXB1D01, 0x0001
       ;.equiv TXB1D02, 0x0002
       ;.equiv TXB1D03, 0x0003
       ;.equiv TXB1D04, 0x0004
       ;.equiv TXB1D05, 0x0005
       ;.equiv TXB1D06, 0x0006
       ;.equiv TXB1D07, 0x0007
       ;.equiv TXB1D10, 0x0008
       ;.equiv TXB1D11, 0x0009
       ;.equiv TXB1D12, 0x000A
       ;.equiv TXB1D13, 0x000B
       ;.equiv TXB1D14, 0x000C
       ;.equiv TXB1D15, 0x000D
       ;.equiv TXB1D16, 0x000E
       ;.equiv TXB1D17, 0x000F


;----- C2RX1B2 Bits -----------------------------------------------------
       ;.equiv TXB1D20, 0x0000
       ;.equiv TXB1D21, 0x0001
       ;.equiv TXB1D22, 0x0002
       ;.equiv TXB1D23, 0x0003
       ;.equiv TXB1D24, 0x0004
       ;.equiv TXB1D25, 0x0005
       ;.equiv TXB1D26, 0x0006
       ;.equiv TXB1D27, 0x0007
       ;.equiv TXB1D30, 0x0008
       ;.equiv TXB1D31, 0x0009
       ;.equiv TXB1D32, 0x000A
       ;.equiv TXB1D33, 0x000B
       ;.equiv TXB1D34, 0x000C
       ;.equiv TXB1D35, 0x000D
       ;.equiv TXB1D36, 0x000E
       ;.equiv TXB1D37, 0x000F


;----- C2RX1B3 Bits -----------------------------------------------------
       ;.equiv TXB1D40, 0x0000
       ;.equiv TXB1D41, 0x0001
       ;.equiv TXB1D42, 0x0002
       ;.equiv TXB1D43, 0x0003
       ;.equiv TXB1D44, 0x0004
       ;.equiv TXB1D45, 0x0005
       ;.equiv TXB1D46, 0x0006
       ;.equiv TXB1D47, 0x0007
       ;.equiv TXB1D50, 0x0008
       ;.equiv TXB1D51, 0x0009
       ;.equiv TXB1D52, 0x000A
       ;.equiv TXB1D53, 0x000B
       ;.equiv TXB1D54, 0x000C
       ;.equiv TXB1D55, 0x000D
       ;.equiv TXB1D56, 0x000E
       ;.equiv TXB1D57, 0x000F


;----- C2RX1B4 Bits -----------------------------------------------------
       ;.equiv TXB1D60, 0x0000
       ;.equiv TXB1D61, 0x0001
       ;.equiv TXB1D62, 0x0002
       ;.equiv TXB1D63, 0x0003
       ;.equiv TXB1D64, 0x0004
       ;.equiv TXB1D65, 0x0005
       ;.equiv TXB1D66, 0x0006
       ;.equiv TXB1D67, 0x0007
       ;.equiv TXB1D70, 0x0008
       ;.equiv TXB1D71, 0x0009
       ;.equiv TXB1D72, 0x000A
       ;.equiv TXB1D73, 0x000B
       ;.equiv TXB1D74, 0x000C
       ;.equiv TXB1D75, 0x000D
       ;.equiv TXB1D76, 0x000E
       ;.equiv TXB1D77, 0x000F


;----- C2RX1CON Bits -----------------------------------------------------
       ;.equiv RXRTRRO, 0x0003
       ;.equiv RXFUL,   0x0007

       ;.equiv FILHIT0, 0x0000
       ;.equiv FILHIT1, 0x0001
       ;.equiv FILHIT2, 0x0002


;----- C2RX0SID Bits -----------------------------------------------------
       ;.equiv RXIDE,   0x0000
       ;.equiv SRR,     0x0001
       ;.equiv SID0,    0x0002
       ;.equiv SID1,    0x0003
       ;.equiv SID2,    0x0004
       ;.equiv SID3,    0x0005
       ;.equiv SID4,    0x0006
       ;.equiv SID5,    0x0007
        .equiv SID6_C2RX0SID, 0x0008
        .equiv SID7_C2RX0SID, 0x0009
        .equiv SID8_C2RX0SID, 0x000A
        .equiv SID9_C2RX0SID, 0x000B
        .equiv SID10_C2RX0SID, 0x000C



;----- C2RX0EID Bits -----------------------------------------------------
       ;.equiv EID6,    0x0000
       ;.equiv EID7,    0x0001
       ;.equiv EID8,    0x0002
       ;.equiv EID9,    0x0003
       ;.equiv EID10,   0x0004
       ;.equiv EID11,   0x0005
       ;.equiv EID12,   0x0006
       ;.equiv EID13,   0x0007
        .equiv EID14_C2RX0EID, 0x0008
        .equiv EID15_C2RX0EID, 0x0009
        .equiv EID16_C2RX0EID, 0x000A
        .equiv EID17_C2RX0EID, 0x000B


;----- C2RX0DLC Bits -----------------------------------------------------
       ;.equiv RXRB0,   0x0004
       ;.equiv RXRB1,   0x0008
       ;.equiv RXRTR,   0x0009
       ;.equiv EID0,    0x000A
       ;.equiv EID1,    0x000B
       ;.equiv EID2,    0x000C
       ;.equiv EID3,    0x000D
       ;.equiv EID4,    0x000E
       ;.equiv EID5,    0x000F


        .equiv DLC0_C2RX0DLC, 0x0000
        .equiv DLC1_C2RX0DLC, 0x0001
        .equiv DLC2_C2RX0DLC, 0x0002
        .equiv DLC3_C2RX0DLC, 0x0003


;----- C2RX0B1 Bits -----------------------------------------------------
       ;.equiv TXB0D00, 0x0000
       ;.equiv TXB0D01, 0x0001
       ;.equiv TXB0D02, 0x0002
       ;.equiv TXB0D03, 0x0003
       ;.equiv TXB0D04, 0x0004
       ;.equiv TXB0D05, 0x0005
       ;.equiv TXB0D06, 0x0006
       ;.equiv TXB0D07, 0x0007
       ;.equiv TXB0D10, 0x0008
       ;.equiv TXB0D11, 0x0009
       ;.equiv TXB0D12, 0x000A
       ;.equiv TXB0D13, 0x000B
       ;.equiv TXB0D14, 0x000C
       ;.equiv TXB0D15, 0x000D
       ;.equiv TXB0D16, 0x000E
       ;.equiv TXB0D17, 0x000F


;----- C2RX0B2 Bits -----------------------------------------------------
       ;.equiv TXB0D20, 0x0000
       ;.equiv TXB0D21, 0x0001
       ;.equiv TXB0D22, 0x0002
       ;.equiv TXB0D23, 0x0003
       ;.equiv TXB0D24, 0x0004
       ;.equiv TXB0D25, 0x0005
       ;.equiv TXB0D26, 0x0006
       ;.equiv TXB0D27, 0x0007
       ;.equiv TXB0D30, 0x0008
       ;.equiv TXB0D31, 0x0009
       ;.equiv TXB0D32, 0x000A
       ;.equiv TXB0D33, 0x000B
       ;.equiv TXB0D34, 0x000C
       ;.equiv TXB0D35, 0x000D
       ;.equiv TXB0D36, 0x000E
       ;.equiv TXB0D37, 0x000F


;----- C2RX0B3 Bits -----------------------------------------------------
       ;.equiv TXB0D40, 0x0000
       ;.equiv TXB0D41, 0x0001
       ;.equiv TXB0D42, 0x0002
       ;.equiv TXB0D43, 0x0003
       ;.equiv TXB0D44, 0x0004
       ;.equiv TXB0D45, 0x0005
       ;.equiv TXB0D46, 0x0006
       ;.equiv TXB0D47, 0x0007
       ;.equiv TXB0D50, 0x0008
       ;.equiv TXB0D51, 0x0009
       ;.equiv TXB0D52, 0x000A
       ;.equiv TXB0D53, 0x000B
       ;.equiv TXB0D54, 0x000C
       ;.equiv TXB0D55, 0x000D
       ;.equiv TXB0D56, 0x000E
       ;.equiv TXB0D57, 0x000F


;----- C2RX0B4 Bits -----------------------------------------------------
       ;.equiv TXB0D60, 0x0000
       ;.equiv TXB0D61, 0x0001
       ;.equiv TXB0D62, 0x0002
       ;.equiv TXB0D63, 0x0003
       ;.equiv TXB0D64, 0x0004
       ;.equiv TXB0D65, 0x0005
       ;.equiv TXB0D66, 0x0006
       ;.equiv TXB0D67, 0x0007
       ;.equiv TXB0D70, 0x0008
       ;.equiv TXB0D71, 0x0009
       ;.equiv TXB0D72, 0x000A
       ;.equiv TXB0D73, 0x000B
       ;.equiv TXB0D74, 0x000C
       ;.equiv TXB0D75, 0x000D
       ;.equiv TXB0D76, 0x000E
       ;.equiv TXB0D77, 0x000F


;----- C2RX0CON Bits -----------------------------------------------------
       ;.equiv FILHIT0, 0x0000
       ;.equiv JTOFF,   0x0001
       ;.equiv DBEN,    0x0002
       ;.equiv RXRTRRO, 0x0003
       ;.equiv RXFUL,   0x0007

       ;.equiv FILHIT,  0x0000
       ;.equiv RXB0DBEN, 0x0002


;----- C2CTRL Bits -----------------------------------------------------
       ;.equiv CANCKS,  0x000B
       ;.equiv ABAT,    0x000C
       ;.equiv CANSIDL, 0x000D
       ;.equiv CANFRZ,  0x000E
       ;.equiv CANCAP,  0x000F

       ;.equiv ICODE0,  0x0001
       ;.equiv ICODE1,  0x0002
       ;.equiv ICODE2,  0x0003
       ;.equiv OPMODE0, 0x0005
       ;.equiv OPMODE1, 0x0006
       ;.equiv OPMODE2, 0x0007
       ;.equiv REQOP0,  0x0008
       ;.equiv REQOP1,  0x0009
       ;.equiv REQOP2,  0x000A
       ;.equiv CSIDL,   0x000D


;----- C2CFG1 Bits -----------------------------------------------------
       ;.equiv BRP0,    0x0000
       ;.equiv BRP1,    0x0001
       ;.equiv BRP2,    0x0002
       ;.equiv BRP3,    0x0003
       ;.equiv BRP4,    0x0004
       ;.equiv BRP5,    0x0005
       ;.equiv SJW0,    0x0006
       ;.equiv SJW1,    0x0007


;----- C2CFG2 Bits -----------------------------------------------------
       ;.equiv SAM,     0x0006
       ;.equiv SEG2PHTS, 0x0007
       ;.equiv WAKFIL,  0x000E

       ;.equiv PRSEG0,  0x0000
       ;.equiv PRSEG1,  0x0001
       ;.equiv PRSEG2,  0x0002
       ;.equiv SEG1PH0, 0x0003
       ;.equiv SEG1PH1, 0x0004
       ;.equiv SEG1PH2, 0x0005
       ;.equiv SEG2PH0, 0x0008
       ;.equiv SEG2PH1, 0x0009
       ;.equiv SEG2PH2, 0x000A


;----- C2INTF Bits -----------------------------------------------------
       ;.equiv RXB0IF,  0x0000
       ;.equiv RXB1IF,  0x0001
       ;.equiv TXB0IF,  0x0002
       ;.equiv TXB1IF,  0x0003
       ;.equiv TXB2IF,  0x0004
       ;.equiv ERRIF,   0x0005
       ;.equiv WAKIF,   0x0006
       ;.equiv IRXIF,   0x0007
       ;.equiv EWARN,   0x0008
       ;.equiv RXWARN,  0x0009
       ;.equiv TXWARN,  0x000A
       ;.equiv RXBP,    0x000B
       ;.equiv TXBP,    0x000C
       ;.equiv TXBO,    0x000D
       ;.equiv RXB1OVFL, 0x000E
       ;.equiv RXB0OVFL, 0x000F

       ;.equiv RX0IF,   0x0000
       ;.equiv RX1IF,   0x0001
       ;.equiv TX0IF,   0x0002
       ;.equiv TX1IF,   0x0003
       ;.equiv TX2IF,   0x0004
       ;.equiv IVRIF,   0x0007
       ;.equiv RXWAR,   0x0009
       ;.equiv TXWAR,   0x000A
       ;.equiv RXEP,    0x000B
       ;.equiv TXEP,    0x000C
       ;.equiv RX1OVR,  0x000E
       ;.equiv RX0OVR,  0x000F


;----- C2INTE Bits -----------------------------------------------------
       ;.equiv RXB0IE,  0x0000
       ;.equiv RXB1IE,  0x0001
       ;.equiv TXB0IE,  0x0002
       ;.equiv TXB1IE,  0x0003
       ;.equiv TXB2IE,  0x0004
       ;.equiv ERRIE,   0x0005
       ;.equiv WAKIE,   0x0006
       ;.equiv IRXIE,   0x0007

       ;.equiv RX0IE,   0x0000
       ;.equiv RX1IE,   0x0001
       ;.equiv TX0IE,   0x0002
       ;.equiv TX1IE,   0x0003
       ;.equiv TX2IE,   0x0004
       ;.equiv IVRIE,   0x0007


;----- C2EC Bits -----------------------------------------------------
       ;.equiv REC0,    0x0000
       ;.equiv REC1,    0x0001
       ;.equiv REC2,    0x0002
       ;.equiv REC3,    0x0003
       ;.equiv REC4,    0x0004
       ;.equiv REC5,    0x0005
       ;.equiv REC6,    0x0006
       ;.equiv REC7,    0x0007
       ;.equiv TEC0,    0x0008
       ;.equiv TEC1,    0x0009
       ;.equiv TEC2,    0x000A
       ;.equiv TEC3,    0x000B
       ;.equiv TEC4,    0x000C
       ;.equiv TEC5,    0x000D
       ;.equiv TEC6,    0x000E
       ;.equiv TEC7,    0x000F


;----- RCON Bits -----------------------------------------------------
        .equiv POR,     0x0000
        .equiv BOR,     0x0001
        .equiv IDLE,    0x0002
        .equiv SLEEP,   0x0003
        .equiv WDTO,    0x0004
        .equiv SWDTEN,  0x0005
        .equiv SWR,     0x0006
        .equiv EXTR,    0x0007
        .equiv LVDEN,   0x000C
        .equiv BGST,    0x000D
        .equiv IOPUWR,  0x000E
        .equiv TRAPR,   0x000F

        .equiv LVDL0,   0x0008
        .equiv LVDL1,   0x0009
        .equiv LVDL2,   0x000A
        .equiv LVDL3,   0x000B


;----- OSCCON Bits -----------------------------------------------------
        .equiv OSWEN,   0x0000
        .equiv LPOSCEN, 0x0001
        .equiv CF,      0x0003
        .equiv LOCK,    0x0005

        .equiv POST0,   0x0006
        .equiv POST1,   0x0007
        .equiv NOSC0,   0x0008
        .equiv NOSC1,   0x0009
        .equiv NOSC2,   0x000A
        .equiv COSC0,   0x000C
        .equiv COSC1,   0x000D
        .equiv COSC2,   0x000E


;----- OSCTUN Bits -----------------------------------------------------
        .equiv TUN0,    0x0000
        .equiv TUN1,    0x0001
        .equiv TUN2,    0x0002
        .equiv TUN3,    0x0003


;----- BSRAM Bits -----------------------------------------------------
        .equiv RL_BSR,  0x0000
        .equiv IR_BSR,  0x0001
        .equiv IW_BSR,  0x0002


;----- SSRAM Bits -----------------------------------------------------
        .equiv RL_SSR,  0x0000
        .equiv IR_SSR,  0x0001
        .equiv IW_SSR,  0x0002


;----- NVMCON Bits -----------------------------------------------------
        .equiv TWRI,    0x0008
        .equiv WRERR,   0x000D
        .equiv WREN,    0x000E
        .equiv WR,      0x000F

        .equiv PROGOP0, 0x0000
        .equiv PROGOP1, 0x0001
        .equiv PROGOP2, 0x0002
        .equiv PROGOP3, 0x0003
        .equiv PROGOP4, 0x0004
        .equiv PROGOP5, 0x0005
        .equiv PROGOP6, 0x0006


;----- PMD1 Bits -----------------------------------------------------
        .equiv ADCMD,   0x0000
        .equiv C1MD,    0x0001
        .equiv C2MD,    0x0002
        .equiv SPI1MD,  0x0003
        .equiv SPI2MD,  0x0004
        .equiv U1MD,    0x0005
        .equiv U2MD,    0x0006
        .equiv I2CMD,   0x0007
        .equiv DCIMD,   0x0008
        .equiv T1MD,    0x000B
        .equiv T2MD,    0x000C
        .equiv T3MD,    0x000D
        .equiv T4MD,    0x000E
        .equiv T5MD,    0x000F


;----- PMD2 Bits -----------------------------------------------------
        .equiv OC1MD,   0x0000
        .equiv OC2MD,   0x0001
        .equiv OC3MD,   0x0002
        .equiv OC4MD,   0x0003
        .equiv OC5MD,   0x0004
        .equiv OC6MD,   0x0005
        .equiv OC7MD,   0x0006
        .equiv OC8MD,   0x0007
        .equiv IC1MD,   0x0008
        .equiv IC2MD,   0x0009
        .equiv IC3MD,   0x000A
        .equiv IC4MD,   0x000B
        .equiv IC5MD,   0x000C
        .equiv IC6MD,   0x000D
        .equiv IC7MD,   0x000E
        .equiv IC8MD,   0x000F




;  Some useful macros for PWRSAV instructions

   .equiv SLEEP_MODE, 0x0
   .equiv IDLE_MODE, 0x1


;==========================================================================
;
;   Configuration Words
;
;   Configuration words exist in Program Space and their locations are
;   defined in the device linker script. They can be set in source code
;   or in the MPLAB IDE. Each configuration word should be specified
;   only once (multiple settings may be combined.)
;
;==========================================================================
;
;  Setting configuration words using macros:
;
;  The following macro named 'config' can be used to set configuration words:
;
        .macro config REG, VALUE
        .pushsection \REG.sec, code
        .global \REG
\REG:   .pword \VALUE
        .popsection
        .endm
;
;  For example, to set CONFIG_REG using the macro above, the following line
;  can be pasted only at the beginning of the assembly source code,
;  immediately below the '.include' directive.
;
;        config __CONFIG_REG, SETTING_A & SETTING_B
;
;  Note that the 'config' macro takes two arguments: the first is the name
;  of a configuration word (in this case, __CONFIG_REG), the second argument
;  is a boolean expression that may include multiple settings.
;  The example above would enable SETTING_A and also SETTING_B.
;
;  A description of all possible settings for each configuration word
;  appears below.
;
;==========================================================================
;
;  Setting configuration words without using macros:
;
;  To set configuration words without using the 'config' macro,
;  use the following format:
;
;           .section __CONFIG_REG.sec, code
;           .global __CONFIG_REG
;__CONFIG_REG: .pword SETTING_A & SETTING_B
;
;  This source code is equivalent to the expanded macro from the previous
;  example.
;
;==========================================================================


;----- FUID0 (0x8005c0) --------------------------------------------------
;
;  The following settings are available for FUID0:
;

;----- FUID1 (0x8005c2) --------------------------------------------------
;
;  The following settings are available for FUID1:
;

;----- FUID2 (0x8005c4) --------------------------------------------------
;
;  The following settings are available for FUID2:
;

;----- FUID3 (0x8005c6) --------------------------------------------------
;
;  The following settings are available for FUID3:
;

;----- FOSC (0xf80000) --------------------------------------------------
;
;  The following settings are available for FOSC:
;
;   Oscillator:
;     LPOSC                Low-Power 32KHz Osc (No change to Primary Osc Mode bits)
;     LP                   Low-Power 32KHz Osc (No change to Primary Osc Mode bits)
;     FRC                  Internal Fast RC (No change to Primary Osc Mode bits)
;     LPRC                 Internal Low-Power RC (No change to Primary Osc Mode bits)
;     XTL                  XTL
;     HS                   HS
;     XT                   XT
;     ERCIO                ERCIO
;     ERC                  ERC
;     EC                   EC
;     ECIO                 ECIO
;     FRC_PLL4             FRC w/PLL 4x
;     FRC_PLL16            FRC w/PLL 16x
;     XT_PLL4              XT w/PLL 4x
;     XT_PLL8              XT w/PLL 8x
;     XT_PLL16             XT w/PLL 16x
;     FRC_PLL8             FRC w/PLL 8x
;     ECIO_PLL4            ECIO w/PLL 4x
;     ECIO_PLL8            ECIO w/PLL 8x
;     ECIO_PLL16           ECIO w/PLL 16x
;     HS2_PLL4             HS2 w/PLL 4x
;     HS2_PLL8             HS2 w/PLL 8x
;     HS2_PLL16            HS2 w/PLL 16x
;     HS3_PLL4             HS3 w/PLL 4x
;     HS3_PLL8             HS3 w/PLL 8x
;     HS3_PLL16            HS3 w/PLL 16x
;
;   Clock Switching and Monitor:
;     CSW_FSCM_ON          Sw Enabled, Mon Enabled
;     CSW_ON_FSCM_OFF      Sw Enabled, Mon Disabled
;     CSW_FSCM_OFF         Sw Disabled, Mon Disabled
;
        .equiv LPOSC,           0xF8E0 ; Low-Power 32KHz Osc (No change to Primary Osc Mode bits)
        .equiv LP,              0xF8E0 ; Low-Power 32KHz Osc (No change to Primary Osc Mode bits)
        .equiv FRC,             0xF9E0 ; Internal Fast RC (No change to Primary Osc Mode bits)
        .equiv LPRC,            0xFAE0 ; Internal Low-Power RC (No change to Primary Osc Mode bits)
        .equiv XTL,             0xFBE0 ; XTL
        .equiv HS,              0xFBE2 ; HS
        .equiv XT,              0xFBE4 ; XT
        .equiv ERCIO,           0xFBE8 ; ERCIO
        .equiv ERC,             0xFBE9 ; ERC
        .equiv EC,              0xFBEB ; EC
        .equiv ECIO,            0xFBEC ; ECIO
        .equiv FRC_PLL4,        0xFFE1 ; FRC w/PLL 4x
        .equiv FRC_PLL16,       0xFFE3 ; FRC w/PLL 16x
        .equiv XT_PLL4,         0xFFE5 ; XT w/PLL 4x
        .equiv XT_PLL8,         0xFFE6 ; XT w/PLL 8x
        .equiv XT_PLL16,        0xFFE7 ; XT w/PLL 16x
        .equiv FRC_PLL8,        0xFFEA ; FRC w/PLL 8x
        .equiv ECIO_PLL4,       0xFFED ; ECIO w/PLL 4x
        .equiv ECIO_PLL8,       0xFFEE ; ECIO w/PLL 8x
        .equiv ECIO_PLL16,      0xFFEF ; ECIO w/PLL 16x
        .equiv HS2_PLL4,        0xFFF1 ; HS2 w/PLL 4x
        .equiv HS2_PLL8,        0xFFF2 ; HS2 w/PLL 8x
        .equiv HS2_PLL16,       0xFFF3 ; HS2 w/PLL 16x
        .equiv HS3_PLL4,        0xFFF5 ; HS3 w/PLL 4x
        .equiv HS3_PLL8,        0xFFF6 ; HS3 w/PLL 8x
        .equiv HS3_PLL16,       0xFFF7 ; HS3 w/PLL 16x

        .equiv CSW_FSCM_ON,     0x3FFF ; Sw Enabled, Mon Enabled
        .equiv CSW_ON_FSCM_OFF, 0x7FFF ; Sw Enabled, Mon Disabled
        .equiv CSW_FSCM_OFF,    0xBFFF ; Sw Disabled, Mon Disabled


;----- FWDT (0xf80002) --------------------------------------------------
;
;  The following settings are available for FWDT:
;
;   WDT Prescaler B:
;     WDTPSB_1             1:1
;     WDTPSB_2             1:2
;     WDTPSB_3             1:3
;     WDTPSB_4             1:4
;     WDTPSB_5             1:5
;     WDTPSB_6             1:6
;     WDTPSB_7             1:7
;     WDTPSB_8             1:8
;     WDTPSB_9             1:9
;     WDTPSB_10            1:10
;     WDTPSB_11            1:11
;     WDTPSB_12            1:12
;     WDTPSB_13            1:13
;     WDTPSB_14            1:14
;     WDTPSB_15            1:15
;     WDTPSB_16            1:16
;
;   WDT Prescaler A:
;     WDTPSA_1             1:1
;     WDTPSA_8             1:8
;     WDTPSA_64            1:64
;     WDTPSA_512           1:512
;
;   Watchdog Timer:
;     WDT_OFF              Disabled
;     WDT_ON               Enabled
;
        .equiv WDTPSB_1,        0xFFF0 ; 1:1
        .equiv WDTPSB_2,        0xFFF1 ; 1:2
        .equiv WDTPSB_3,        0xFFF2 ; 1:3
        .equiv WDTPSB_4,        0xFFF3 ; 1:4
        .equiv WDTPSB_5,        0xFFF4 ; 1:5
        .equiv WDTPSB_6,        0xFFF5 ; 1:6
        .equiv WDTPSB_7,        0xFFF6 ; 1:7
        .equiv WDTPSB_8,        0xFFF7 ; 1:8
        .equiv WDTPSB_9,        0xFFF8 ; 1:9
        .equiv WDTPSB_10,       0xFFF9 ; 1:10
        .equiv WDTPSB_11,       0xFFFA ; 1:11
        .equiv WDTPSB_12,       0xFFFB ; 1:12
        .equiv WDTPSB_13,       0xFFFC ; 1:13
        .equiv WDTPSB_14,       0xFFFD ; 1:14
        .equiv WDTPSB_15,       0xFFFE ; 1:15
        .equiv WDTPSB_16,       0xFFFF ; 1:16

        .equiv WDTPSA_1,        0xFFCF ; 1:1
        .equiv WDTPSA_8,        0xFFDF ; 1:8
        .equiv WDTPSA_64,       0xFFEF ; 1:64
        .equiv WDTPSA_512,      0xFFFF ; 1:512

        .equiv WDT_OFF,         0x7FFF ; Disabled
        .equiv WDT_ON,          0xFFFF ; Enabled


;----- FBORPOR (0xf80004) --------------------------------------------------
;
;  The following settings are available for FBORPOR:
;
;   POR Timer Value:
;     PWRT_OFF             Timer Disabled
;     PWRT_4               4ms
;     PWRT_16              16ms
;     PWRT_64              64ms
;
;   Brown Out Voltage:
;     BORV_45              4.5V
;     BORV_42              4.2V
;     BORV_27              2.7V
;     NONE                 Reserved
;
;   PBOR Enable:
;     PBOR_OFF             Disabled
;     PBOR_ON              Enabled
;
;   Master Clear Enable:
;     MCLR_DIS             Disabled
;     MCLR_EN              Enabled
;
        .equiv PWRT_OFF,        0xFFFC ; Timer Disabled
        .equiv PWRT_4,          0xFFFD ; 4ms
        .equiv PWRT_16,         0xFFFE ; 16ms
        .equiv PWRT_64,         0xFFFF ; 64ms

        .equiv BORV_45,         0xFFCF ; 4.5V
        .equiv BORV_42,         0xFFDF ; 4.2V
        .equiv BORV_27,         0xFFEF ; 2.7V
        .equiv NONE,            0xFFFF ; Reserved

        .equiv PBOR_OFF,        0xFF7F ; Disabled
        .equiv PBOR_ON,         0xFFFF ; Enabled

        .equiv MCLR_DIS,        0x7FFF ; Disabled
        .equiv MCLR_EN,         0xFFFF ; Enabled


;----- FBS (0xf80006) --------------------------------------------------
;
;  The following settings are available for FBS:
;
;   Boot Segment Program Memory Write Protect:
;     WR_PROTECT_BOOT_ON   Boot Segment Program Memory is write protected
;     WR_PROTECT_BOOT_OFF  Boot Segment Program Memory may be written
;
;   Boot Segment Program Flash Memory Code Protection:
;     HIGH_LARGE_BOOT_CODE High Security, Large-sized Boot Flash
;     HIGH_MEDIUM_BOOT_CODEHigh Security, Medium-sized Boot Flash
;     HIGH_SMALL_BOOT_CODE High Security, Small-sized Boot Flash
;     STRD_LARGE_BOOT_CODE Standard Security, Large-sized Boot Flash
;     STRD_MEDIUM_BOOT_CODEStandard Security, Medium-sized Boot Flash
;     STRD_SMALL_BOOT_CODE Standard Security, Small-sized Boot Flash
;     NO_BOOT_CODE         No Boot Segment
;
;   Boot Segment Data EEPROM Protection:
;     SMALL_BOOT_EEPROM    Boot EEPROM Enabled
;     NO_BOOT_EEPROM       No Boot EEPROM
;
;   Boot Segment Data RAM Protection:
;     LARGE_BOOT_RAM       Large-sized Boot RAM
;     MED_BOOT_RAM         Medium-sized Boot RAM
;     SMALL_BOOT_RAM       Small-sized Boot RAM
;     NO_BOOT_RAM          No Boot RAM
;
        .equiv WR_PROTECT_BOOT_ON, 0xFFFE ; Boot Segment Program Memory is write protected
        .equiv WR_PROTECT_BOOT_OFF, 0xFFFF ; Boot Segment Program Memory may be written

        .equiv HIGH_LARGE_BOOT_CODE, 0xFFF1 ; High Security, Large-sized Boot Flash
        .equiv HIGH_MEDIUM_BOOT_CODE, 0xFFF3 ; High Security, Medium-sized Boot Flash
        .equiv HIGH_SMALL_BOOT_CODE, 0xFFF5 ; High Security, Small-sized Boot Flash
        .equiv STRD_LARGE_BOOT_CODE, 0xFFF9 ; Standard Security, Large-sized Boot Flash
        .equiv STRD_MEDIUM_BOOT_CODE, 0xFFFB ; Standard Security, Medium-sized Boot Flash
        .equiv STRD_SMALL_BOOT_CODE, 0xFFFD ; Standard Security, Small-sized Boot Flash
        .equiv NO_BOOT_CODE,    0xFFFF ; No Boot Segment

        .equiv SMALL_BOOT_EEPROM, 0xFEFF ; Boot EEPROM Enabled
        .equiv NO_BOOT_EEPROM,  0xFFFF ; No Boot EEPROM

        .equiv LARGE_BOOT_RAM,  0xCFFF ; Large-sized Boot RAM
        .equiv MED_BOOT_RAM,    0xDFFF ; Medium-sized Boot RAM
        .equiv SMALL_BOOT_RAM,  0xEFFF ; Small-sized Boot RAM
        .equiv NO_BOOT_RAM,     0xFFFF ; No Boot RAM


;----- FSS (0xf80008) --------------------------------------------------
;
;  The following settings are available for FSS:
;
;   Secure Segment Program Write Protect:
;     WR_PROT_SEC_ON       Enabled
;     WR_PROT_SEC_OFF      Disabled
;
;   Secure Segment Program Flash Memory Code Protection:
;     HIGH_LARGE_SEC_CODE  High Security, Large-sized Secure Flash
;     HIGH_MEDIUM_SEC_CODE High Security, Medium-sized Secure Flash
;     HIGH_SMALL_SEC_CODE  High Security, Small-sized Secure Flash
;     STRD_LAR_SEC_CODE    Standard Security, Large-sized Secure Flash
;     STRD_MED_SEC_CODE    Standard Security, Medium-sized Secure Flash
;     STRD_SMALL_SEC_CODE  Standard Security, Small-sized Secure Flash
;     NO_SEC_CODE          No Secure Segment
;
;   Secure Segment Data EEPROM Protection:
;     LAR_SEC_EEPROM       Large-sized Secure Data EEPROM
;     MED_SEC_EEPROM       Medium-sized Secure Data EEPROM
;     SMALL_SEC_EEPROM     Small-sized Secure Data EEPROM
;     NO_SEC_EEPROM        No Segment Data EEPROM
;
;   Secure Segment Data RAM Protection:
;     LARGE_SEC_RAM        Large-sized Secure RAM
;     MED_SEC_RAM          Medium-sized Secure RAM
;     SMALL_SEC_RAM        Small-sized Secure RAM
;     NO_SEC_RAM           No Secure RAM
;
        .equiv WR_PROT_SEC_ON,  0xFFFE ; Enabled
        .equiv WR_PROT_SEC_OFF, 0xFFFF ; Disabled

        .equiv HIGH_LARGE_SEC_CODE, 0xFFF1 ; High Security, Large-sized Secure Flash
        .equiv HIGH_MEDIUM_SEC_CODE, 0xFFF3 ; High Security, Medium-sized Secure Flash
        .equiv HIGH_SMALL_SEC_CODE, 0xFFF5 ; High Security, Small-sized Secure Flash
        .equiv STRD_LAR_SEC_CODE, 0xFFF9 ; Standard Security, Large-sized Secure Flash
        .equiv STRD_MED_SEC_CODE, 0xFFFB ; Standard Security, Medium-sized Secure Flash
        .equiv STRD_SMALL_SEC_CODE, 0xFFFD ; Standard Security, Small-sized Secure Flash
        .equiv NO_SEC_CODE,     0xFFFF ; No Secure Segment

        .equiv LAR_SEC_EEPROM,  0xFCFF ; Large-sized Secure Data EEPROM
        .equiv MED_SEC_EEPROM,  0xFDFF ; Medium-sized Secure Data EEPROM
        .equiv SMALL_SEC_EEPROM, 0xFEFF ; Small-sized Secure Data EEPROM
        .equiv NO_SEC_EEPROM,   0xFFFF ; No Segment Data EEPROM

        .equiv LARGE_SEC_RAM,   0xCFFF ; Large-sized Secure RAM
        .equiv MED_SEC_RAM,     0xDFFF ; Medium-sized Secure RAM
        .equiv SMALL_SEC_RAM,   0xEFFF ; Small-sized Secure RAM
        .equiv NO_SEC_RAM,      0xFFFF ; No Secure RAM


;----- FGS (0xf8000a) --------------------------------------------------
;
;  The following settings are available for FGS:
;
;   General Code Segment Write Protect:
;     GWRP_ON              Enabled
;     GWRP_OFF             Disabled
;
;   General Segment Code Protection:
;     GSS_HIGH             High Security Code Protection Enabled
;     GSS_ON               Enabled
;     GSS_STD              Enabled
;     CODE_PROT_ON         Enabled
;     GSS_OFF              Disabled
;     CODE_PROT_OFF        Disabled
;
        .equiv GWRP_ON,         0xFFFE ; Enabled
        .equiv GWRP_OFF,        0xFFFF ; Disabled

        .equiv GSS_HIGH,        0xFFFB ; High Security Code Protection Enabled
        .equiv GSS_ON,          0xFFFD ; Enabled
        .equiv GSS_STD,         0xFFFD ; Enabled
        .equiv CODE_PROT_ON,    0xFFFD ; Enabled
        .equiv GSS_OFF,         0xFFFF ; Disabled
        .equiv CODE_PROT_OFF,   0xFFFF ; Disabled


;----- FICD (0xf8000c) --------------------------------------------------
;
;  The following settings are available for FICD:
;
;   Comm Channel Select:
;     ICS_PGD3             Use EMUC3 and EMUD3
;     PGD3                 Use EMUC3 and EMUD3
;     ICS_PGD2             Use EMUC2 and EMUD2
;     PGD2                 Use EMUC2 and EMUD2
;     ICS_PGD1             Use EMUC1 and EMUD1
;     PGD1                 Use EMUC1 and EMUD1
;     ICS_PGD              Use PGC/EMUC and PGD/EMUD
;     PGD                  Use PGC/EMUC and PGD/EMUD
;
        .equiv ICS_PGD3,        0xFFFC ; Use EMUC3 and EMUD3
        .equiv PGD3,            0xFFFC ; Use EMUC3 and EMUD3
        .equiv ICS_PGD2,        0xFFFD ; Use EMUC2 and EMUD2
        .equiv PGD2,            0xFFFD ; Use EMUC2 and EMUD2
        .equiv ICS_PGD1,        0xFFFE ; Use EMUC1 and EMUD1
        .equiv PGD1,            0xFFFE ; Use EMUC1 and EMUD1
        .equiv ICS_PGD,         0xFFFF ; Use PGC/EMUC and PGD/EMUD
        .equiv PGD,             0xFFFF ; Use PGC/EMUC and PGD/EMUD


.LIST
