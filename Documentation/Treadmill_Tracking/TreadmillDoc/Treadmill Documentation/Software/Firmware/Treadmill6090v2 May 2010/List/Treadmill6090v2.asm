
;CodeVisionAVR C Compiler V2.04.6 Standard
;(C) Copyright 1998-2010 Pavel Haiduc, HP InfoTech s.r.l.
;http://www.hpinfotech.com

;Chip type                : ATmega644P
;Program type             : Application
;Clock frequency          : 20.000000 MHz
;Memory model             : Small
;Optimize for             : Speed
;(s)printf features       : int, width
;(s)scanf features        : int, width
;External RAM size        : 0
;Data Stack size          : 200 byte(s)
;Heap size                : 0 byte(s)
;Promote 'char' to 'int'  : No
;'char' is unsigned       : Yes
;8 bit enums              : No
;global 'const' stored in FLASH: Yes
;Enhanced core instructions    : On
;Smart register allocation     : On
;Automatic register allocation : On

	#pragma AVRPART ADMIN PART_NAME ATmega644P
	#pragma AVRPART MEMORY PROG_FLASH 65536
	#pragma AVRPART MEMORY EEPROM 2048
	#pragma AVRPART MEMORY INT_SRAM SIZE 4096
	#pragma AVRPART MEMORY INT_SRAM START_ADDR 0x100

	#define CALL_SUPPORTED 1

	.LISTMAC
	.EQU EERE=0x0
	.EQU EEWE=0x1
	.EQU EEMWE=0x2
	.EQU UDRE=0x5
	.EQU RXC=0x7
	.EQU EECR=0x1F
	.EQU EEDR=0x20
	.EQU EEARL=0x21
	.EQU EEARH=0x22
	.EQU SPSR0=0x2D
	.EQU SPDR0=0x2E
	.EQU SMCR=0x33
	.EQU MCUSR=0x34
	.EQU MCUCR=0x35
	.EQU WDTCSR=0x60
	.EQU UCSR0A=0xC0
	.EQU UDR0=0xC6
	.EQU SPL=0x3D
	.EQU SPH=0x3E
	.EQU SREG=0x3F
	.EQU GPIOR0=0x1E

	.DEF R0X0=R0
	.DEF R0X1=R1
	.DEF R0X2=R2
	.DEF R0X3=R3
	.DEF R0X4=R4
	.DEF R0X5=R5
	.DEF R0X6=R6
	.DEF R0X7=R7
	.DEF R0X8=R8
	.DEF R0X9=R9
	.DEF R0XA=R10
	.DEF R0XB=R11
	.DEF R0XC=R12
	.DEF R0XD=R13
	.DEF R0XE=R14
	.DEF R0XF=R15
	.DEF R0X10=R16
	.DEF R0X11=R17
	.DEF R0X12=R18
	.DEF R0X13=R19
	.DEF R0X14=R20
	.DEF R0X15=R21
	.DEF R0X16=R22
	.DEF R0X17=R23
	.DEF R0X18=R24
	.DEF R0X19=R25
	.DEF R0X1A=R26
	.DEF R0X1B=R27
	.DEF R0X1C=R28
	.DEF R0X1D=R29
	.DEF R0X1E=R30
	.DEF R0X1F=R31

	.EQU __SRAM_START=0x0100
	.EQU __SRAM_END=0x10FF
	.EQU __DSTACK_SIZE=0x00C8
	.EQU __HEAP_SIZE=0x0000
	.EQU __CLEAR_SRAM_SIZE=__SRAM_END-__SRAM_START+1

	.MACRO __CPD1N
	CPI  R30,LOW(@0)
	LDI  R26,HIGH(@0)
	CPC  R31,R26
	LDI  R26,BYTE3(@0)
	CPC  R22,R26
	LDI  R26,BYTE4(@0)
	CPC  R23,R26
	.ENDM

	.MACRO __CPD2N
	CPI  R26,LOW(@0)
	LDI  R30,HIGH(@0)
	CPC  R27,R30
	LDI  R30,BYTE3(@0)
	CPC  R24,R30
	LDI  R30,BYTE4(@0)
	CPC  R25,R30
	.ENDM

	.MACRO __CPWRR
	CP   R@0,R@2
	CPC  R@1,R@3
	.ENDM

	.MACRO __CPWRN
	CPI  R@0,LOW(@2)
	LDI  R30,HIGH(@2)
	CPC  R@1,R30
	.ENDM

	.MACRO __ADDB1MN
	SUBI R30,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDB2MN
	SUBI R26,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDW1MN
	SUBI R30,LOW(-@0-(@1))
	SBCI R31,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW2MN
	SUBI R26,LOW(-@0-(@1))
	SBCI R27,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	SBCI R22,BYTE3(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1N
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	SBCI R22,BYTE3(-@0)
	SBCI R23,BYTE4(-@0)
	.ENDM

	.MACRO __ADDD2N
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	SBCI R24,BYTE3(-@0)
	SBCI R25,BYTE4(-@0)
	.ENDM

	.MACRO __SUBD1N
	SUBI R30,LOW(@0)
	SBCI R31,HIGH(@0)
	SBCI R22,BYTE3(@0)
	SBCI R23,BYTE4(@0)
	.ENDM

	.MACRO __SUBD2N
	SUBI R26,LOW(@0)
	SBCI R27,HIGH(@0)
	SBCI R24,BYTE3(@0)
	SBCI R25,BYTE4(@0)
	.ENDM

	.MACRO __ANDBMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ANDWMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ANDI R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ANDD1N
	ANDI R30,LOW(@0)
	ANDI R31,HIGH(@0)
	ANDI R22,BYTE3(@0)
	ANDI R23,BYTE4(@0)
	.ENDM

	.MACRO __ANDD2N
	ANDI R26,LOW(@0)
	ANDI R27,HIGH(@0)
	ANDI R24,BYTE3(@0)
	ANDI R25,BYTE4(@0)
	.ENDM

	.MACRO __ORBMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ORWMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ORI  R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ORD1N
	ORI  R30,LOW(@0)
	ORI  R31,HIGH(@0)
	ORI  R22,BYTE3(@0)
	ORI  R23,BYTE4(@0)
	.ENDM

	.MACRO __ORD2N
	ORI  R26,LOW(@0)
	ORI  R27,HIGH(@0)
	ORI  R24,BYTE3(@0)
	ORI  R25,BYTE4(@0)
	.ENDM

	.MACRO __DELAY_USB
	LDI  R24,LOW(@0)
__DELAY_USB_LOOP:
	DEC  R24
	BRNE __DELAY_USB_LOOP
	.ENDM

	.MACRO __DELAY_USW
	LDI  R24,LOW(@0)
	LDI  R25,HIGH(@0)
__DELAY_USW_LOOP:
	SBIW R24,1
	BRNE __DELAY_USW_LOOP
	.ENDM

	.MACRO __GETD1S
	LDD  R30,Y+@0
	LDD  R31,Y+@0+1
	LDD  R22,Y+@0+2
	LDD  R23,Y+@0+3
	.ENDM

	.MACRO __GETD2S
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	LDD  R24,Y+@0+2
	LDD  R25,Y+@0+3
	.ENDM

	.MACRO __PUTD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R31
	STD  Y+@0+2,R22
	STD  Y+@0+3,R23
	.ENDM

	.MACRO __PUTD2S
	STD  Y+@0,R26
	STD  Y+@0+1,R27
	STD  Y+@0+2,R24
	STD  Y+@0+3,R25
	.ENDM

	.MACRO __PUTDZ2
	STD  Z+@0,R26
	STD  Z+@0+1,R27
	STD  Z+@0+2,R24
	STD  Z+@0+3,R25
	.ENDM

	.MACRO __CLRD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R30
	STD  Y+@0+2,R30
	STD  Y+@0+3,R30
	.ENDM

	.MACRO __POINTB1MN
	LDI  R30,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW1MN
	LDI  R30,LOW(@0+(@1))
	LDI  R31,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTD1M
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __POINTW1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	.ENDM

	.MACRO __POINTD1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	LDI  R22,BYTE3(2*@0+(@1))
	LDI  R23,BYTE4(2*@0+(@1))
	.ENDM

	.MACRO __POINTB2MN
	LDI  R26,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW2MN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTBRM
	LDI  R@0,LOW(@1)
	.ENDM

	.MACRO __POINTWRM
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __POINTBRMN
	LDI  R@0,LOW(@1+(@2))
	.ENDM

	.MACRO __POINTWRMN
	LDI  R@0,LOW(@2+(@3))
	LDI  R@1,HIGH(@2+(@3))
	.ENDM

	.MACRO __POINTWRFN
	LDI  R@0,LOW(@2*2+(@3))
	LDI  R@1,HIGH(@2*2+(@3))
	.ENDM

	.MACRO __GETD1N
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __GETD2N
	LDI  R26,LOW(@0)
	LDI  R27,HIGH(@0)
	LDI  R24,BYTE3(@0)
	LDI  R25,BYTE4(@0)
	.ENDM

	.MACRO __GETB1MN
	LDS  R30,@0+(@1)
	.ENDM

	.MACRO __GETB1HMN
	LDS  R31,@0+(@1)
	.ENDM

	.MACRO __GETW1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	.ENDM

	.MACRO __GETD1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	LDS  R22,@0+(@1)+2
	LDS  R23,@0+(@1)+3
	.ENDM

	.MACRO __GETBRMN
	LDS  R@0,@1+(@2)
	.ENDM

	.MACRO __GETWRMN
	LDS  R@0,@2+(@3)
	LDS  R@1,@2+(@3)+1
	.ENDM

	.MACRO __GETWRZ
	LDD  R@0,Z+@2
	LDD  R@1,Z+@2+1
	.ENDM

	.MACRO __GETD2Z
	LDD  R26,Z+@0
	LDD  R27,Z+@0+1
	LDD  R24,Z+@0+2
	LDD  R25,Z+@0+3
	.ENDM

	.MACRO __GETB2MN
	LDS  R26,@0+(@1)
	.ENDM

	.MACRO __GETW2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	.ENDM

	.MACRO __GETD2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	LDS  R24,@0+(@1)+2
	LDS  R25,@0+(@1)+3
	.ENDM

	.MACRO __PUTB1MN
	STS  @0+(@1),R30
	.ENDM

	.MACRO __PUTW1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	.ENDM

	.MACRO __PUTD1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	STS  @0+(@1)+2,R22
	STS  @0+(@1)+3,R23
	.ENDM

	.MACRO __PUTB1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	CALL __EEPROMWRB
	.ENDM

	.MACRO __PUTW1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	CALL __EEPROMWRW
	.ENDM

	.MACRO __PUTD1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	CALL __EEPROMWRD
	.ENDM

	.MACRO __PUTBR0MN
	STS  @0+(@1),R0
	.ENDM

	.MACRO __PUTBMRN
	STS  @0+(@1),R@2
	.ENDM

	.MACRO __PUTWMRN
	STS  @0+(@1),R@2
	STS  @0+(@1)+1,R@3
	.ENDM

	.MACRO __PUTBZR
	STD  Z+@1,R@0
	.ENDM

	.MACRO __PUTWZR
	STD  Z+@2,R@0
	STD  Z+@2+1,R@1
	.ENDM

	.MACRO __GETW1R
	MOV  R30,R@0
	MOV  R31,R@1
	.ENDM

	.MACRO __GETW2R
	MOV  R26,R@0
	MOV  R27,R@1
	.ENDM

	.MACRO __GETWRN
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __PUTW1R
	MOV  R@0,R30
	MOV  R@1,R31
	.ENDM

	.MACRO __PUTW2R
	MOV  R@0,R26
	MOV  R@1,R27
	.ENDM

	.MACRO __ADDWRN
	SUBI R@0,LOW(-@2)
	SBCI R@1,HIGH(-@2)
	.ENDM

	.MACRO __ADDWRR
	ADD  R@0,R@2
	ADC  R@1,R@3
	.ENDM

	.MACRO __SUBWRN
	SUBI R@0,LOW(@2)
	SBCI R@1,HIGH(@2)
	.ENDM

	.MACRO __SUBWRR
	SUB  R@0,R@2
	SBC  R@1,R@3
	.ENDM

	.MACRO __ANDWRN
	ANDI R@0,LOW(@2)
	ANDI R@1,HIGH(@2)
	.ENDM

	.MACRO __ANDWRR
	AND  R@0,R@2
	AND  R@1,R@3
	.ENDM

	.MACRO __ORWRN
	ORI  R@0,LOW(@2)
	ORI  R@1,HIGH(@2)
	.ENDM

	.MACRO __ORWRR
	OR   R@0,R@2
	OR   R@1,R@3
	.ENDM

	.MACRO __EORWRR
	EOR  R@0,R@2
	EOR  R@1,R@3
	.ENDM

	.MACRO __GETWRS
	LDD  R@0,Y+@2
	LDD  R@1,Y+@2+1
	.ENDM

	.MACRO __PUTBSR
	STD  Y+@1,R@0
	.ENDM

	.MACRO __PUTWSR
	STD  Y+@2,R@0
	STD  Y+@2+1,R@1
	.ENDM

	.MACRO __MOVEWRR
	MOV  R@0,R@2
	MOV  R@1,R@3
	.ENDM

	.MACRO __INWR
	IN   R@0,@2
	IN   R@1,@2+1
	.ENDM

	.MACRO __OUTWR
	OUT  @2+1,R@1
	OUT  @2,R@0
	.ENDM

	.MACRO __CALL1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	ICALL
	.ENDM

	.MACRO __CALL1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	CALL __GETW1PF
	ICALL
	.ENDM

	.MACRO __CALL2EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	CALL __EEPROMRDW
	ICALL
	.ENDM

	.MACRO __GETW1STACK
	IN   R26,SPL
	IN   R27,SPH
	ADIW R26,@0+1
	LD   R30,X+
	LD   R31,X
	.ENDM

	.MACRO __GETD1STACK
	IN   R26,SPL
	IN   R27,SPH
	ADIW R26,@0+1
	LD   R30,X+
	LD   R31,X+
	LD   R22,X
	.ENDM

	.MACRO __NBST
	BST  R@0,@1
	IN   R30,SREG
	LDI  R31,0x40
	EOR  R30,R31
	OUT  SREG,R30
	.ENDM


	.MACRO __PUTB1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RNS
	MOVW R26,R@0
	ADIW R26,@1
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	CALL __PUTDP1
	.ENDM


	.MACRO __GETB1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R30,Z
	.ENDM

	.MACRO __GETB1HSX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	.ENDM

	.MACRO __GETW1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R31,Z
	MOV  R30,R0
	.ENDM

	.MACRO __GETD1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R1,Z+
	LD   R22,Z+
	LD   R23,Z
	MOVW R30,R0
	.ENDM

	.MACRO __GETB2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R26,X
	.ENDM

	.MACRO __GETW2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	.ENDM

	.MACRO __GETD2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R1,X+
	LD   R24,X+
	LD   R25,X
	MOVW R26,R0
	.ENDM

	.MACRO __GETBRSX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	LD   R@0,Z
	.ENDM

	.MACRO __GETWRSX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	LD   R@0,Z+
	LD   R@1,Z
	.ENDM

	.MACRO __GETBRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	LD   R@0,X
	.ENDM

	.MACRO __GETWRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	LD   R@0,X+
	LD   R@1,X
	.ENDM

	.MACRO __LSLW8SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	CLR  R30
	.ENDM

	.MACRO __PUTB1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __CLRW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __CLRD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R30
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __PUTB2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z,R26
	.ENDM

	.MACRO __PUTW2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z,R27
	.ENDM

	.MACRO __PUTD2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z+,R27
	ST   Z+,R24
	ST   Z,R25
	.ENDM

	.MACRO __PUTBSRX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	ST   Z,R@0
	.ENDM

	.MACRO __PUTWSRX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	ST   Z+,R@0
	ST   Z,R@1
	.ENDM

	.MACRO __PUTB1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __MULBRR
	MULS R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRRU
	MUL  R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRR0
	MULS R@0,R@1
	.ENDM

	.MACRO __MULBRRU0
	MUL  R@0,R@1
	.ENDM

	.MACRO __MULBNWRU
	LDI  R26,@2
	MUL  R26,R@0
	MOVW R30,R0
	MUL  R26,R@1
	ADD  R31,R0
	.ENDM

;NAME DEFINITIONS FOR GLOBAL VARIABLES ALLOCATED TO REGISTERS
	.DEF _tick=R5
	.DEF _ADNS=R4
	.DEF _gTemp=R7
	.DEF _gTemp2=R6
	.DEF _ADNS0=R9
	.DEF _ADNS1=R8
	.DEF _sstate=R11
	.DEF _r_char=R10
	.DEF _serialFlag=R13
	.DEF _t_char=R12

	.CSEG
	.ORG 0x00

;START OF CODE MARKER
__START_OF_CODE:

;INTERRUPT VECTORS
	JMP  __RESET
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  _timebase
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  _sample
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  _serial_receive0
	JMP  _uart0_send
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00

_0x3:
	.DB  0x1
_0x4:
	.DB  0x2
_0x5:
	.DB  0x80
_0x6:
	.DB  0x80
_0x7:
	.DB  0x80
_0x8:
	.DB  0x80
_0x9:
	.DB  0x8
_0xDE:
	.DB  0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7
	.DB  0x9,0xA,0xB,0xE,0xF,0x10,0x11,0x16
	.DB  0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x2C,0x2D
	.DB  0x3D
_0x12C:
	.DB  0x0,0x0,0x0

__GLOBAL_INI_TBL:
	.DW  0x01
	.DW  _reportState
	.DW  _0x3*2

	.DW  0x01
	.DW  _binTarget
	.DW  _0x4*2

	.DW  0x01
	.DW  _bin0x
	.DW  _0x5*2

	.DW  0x01
	.DW  _bin0y
	.DW  _0x6*2

	.DW  0x01
	.DW  _bin1x
	.DW  _0x7*2

	.DW  0x01
	.DW  _bin1y
	.DW  _0x8*2

	.DW  0x01
	.DW  _binScale
	.DW  _0x9*2

	.DW  0x03
	.DW  0x0B
	.DW  _0x12C*2

_0xFFFFFFFF:
	.DW  0

__RESET:
	CLI
	CLR  R30
	OUT  EECR,R30

;INTERRUPT VECTORS ARE PLACED
;AT THE START OF FLASH
	LDI  R31,1
	OUT  MCUCR,R31
	OUT  MCUCR,R30

;DISABLE WATCHDOG
	LDI  R31,0x18
	WDR
	IN   R26,MCUSR
	CBR  R26,8
	OUT  MCUSR,R26
	STS  WDTCSR,R31
	STS  WDTCSR,R30

;CLEAR R2-R14
	LDI  R24,(14-2)+1
	LDI  R26,2
	CLR  R27
__CLEAR_REG:
	ST   X+,R30
	DEC  R24
	BRNE __CLEAR_REG

;CLEAR SRAM
	LDI  R24,LOW(__CLEAR_SRAM_SIZE)
	LDI  R25,HIGH(__CLEAR_SRAM_SIZE)
	LDI  R26,LOW(__SRAM_START)
	LDI  R27,HIGH(__SRAM_START)
__CLEAR_SRAM:
	ST   X+,R30
	SBIW R24,1
	BRNE __CLEAR_SRAM

;GLOBAL VARIABLES INITIALIZATION
	LDI  R30,LOW(__GLOBAL_INI_TBL*2)
	LDI  R31,HIGH(__GLOBAL_INI_TBL*2)
__GLOBAL_INI_NEXT:
	LPM  R24,Z+
	LPM  R25,Z+
	SBIW R24,0
	BREQ __GLOBAL_INI_END
	LPM  R26,Z+
	LPM  R27,Z+
	LPM  R0,Z+
	LPM  R1,Z+
	MOVW R22,R30
	MOVW R30,R0
__GLOBAL_INI_LOOP:
	LPM  R0,Z+
	ST   X+,R0
	SBIW R24,1
	BRNE __GLOBAL_INI_LOOP
	MOVW R30,R22
	RJMP __GLOBAL_INI_NEXT
__GLOBAL_INI_END:

;GPIOR0 INITIALIZATION
	LDI  R30,0x00
	OUT  GPIOR0,R30

;STACK POINTER INITIALIZATION
	LDI  R30,LOW(__SRAM_END-__HEAP_SIZE)
	OUT  SPL,R30
	LDI  R30,HIGH(__SRAM_END-__HEAP_SIZE)
	OUT  SPH,R30

;DATA STACK POINTER INITIALIZATION
	LDI  R28,LOW(__SRAM_START+__DSTACK_SIZE)
	LDI  R29,HIGH(__SRAM_START+__DSTACK_SIZE)

	JMP  _main

	.ESEG
	.ORG 0

	.DSEG
	.ORG 0x1C8

	.CSEG
;/*4-Axis Optical Motion Tracking System
;Interface to a pair of the AVAGO ADNS-6090 Optical Mouse Camera Chips for Motion Tracking
;
;Implemented in CodeVisionAVR IDE, v2.04.6 standard
;treadmill6090v2.hex included for direct upload to atmel chip (so you don't have to purchase CodeVision unless you want to change the code)
;Use Atmel's AVR Studio to upload the hex file to the ATMega644p
;
;Version 1.1
;May 5, 2010
;
;(c) Gus K Lott III, PhD
;
;Neurobiological Instrumentation Engineer
;HHMI - Janelia Farm Research Campus
;19700 Helix Dr., Ashburn, VA 20147
;lottg@janelia.hhmi.org
;
;*/
;
;//Defines for the ports on which the cameras connect
;#include <mega324.h>
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x01
	.EQU __sm_mask=0x0E
	.EQU __sm_powerdown=0x04
	.EQU __sm_powersave=0x06
	.EQU __sm_standby=0x0C
	.EQU __sm_ext_standby=0x0E
	.EQU __sm_adc_noise_red=0x02
	.SET power_ctrl_reg=smcr
	#endif
;#define gMISO_0 PORTB.4
;#define gdMISO_0 DDRB.4
;#define gMISO_1 PORTB.3
;#define gdMISO_1 DDRB.3
;#define gSCLK PORTB.6
;#define gdSCLK DDRB.6
;#define gMOSI PORTB.5
;#define gdMOSI DDRB.5
;#define gRESET PORTB.1
;#define gdRESET DDRB.1
;#define gNCS PORTB.2
;#define gdNCS DDRB.2
;
;//Ports for Real-Time Trigger Signals
;#define gCLK PORTB.0
;#define gdCLK DDRB.0
;#define gTRG PORTD.4
;#define gdTRG DDRD.4
;#define gaTRG PORTC.0
;#define gdaTRG DDRC.0
;
;#define RBUFFL 900
;
;
;void initialize(void);
;void puts_int(unsigned char newT);
;void pause50us(void);
;void pause10us(void);
;void ADNS_write(unsigned char data);
;void ADNS_read(void);
;void EEPROMDump(void);
;void firmUpload(void);
;void writeEEPROM(void);
;void commandExec(void);
;void grabFrames(void);
;void grabMotion(void);
;void resetADNS(void);
;void dumpRegisters(void);
;void setAnalogOut(void);
;
;//ADNS and gTemp must be register
;unsigned char register tick, ADNS, gTemp;
;unsigned char register gTemp2, ADNS0, ADNS1, sstate=0;
;unsigned char x[2],y[2],motion[2],squal[2],shut_low[2],shut_high[2],maxpix[2];
;unsigned char r_char, serialFlag=0, t_char, t_buffer[RBUFFL], r_buffer[RBUFFL], vTarget=0, vidTime=0, vidFlag=0;
;unsigned char Vf, Vs, Om, SampleCount=0, reportState=1;

	.DSEG
;unsigned int time_ms, serialTimeout=0, t_index=0, t_length=0, r_index=0;
;unsigned char binTime=0, binTarget=2, bin0x=128, bin0y=128, bin1x=128, bin1y=128, binFlag=0, binTemp=0, binScale=8;
;unsigned char bin0xc=0, bin0yc=0, bin1xc=0, bin1yc=0;
;
;
;
;
;
;
;
;
;//Timer Interrupt dedicated to sample rate control
;interrupt [TIM1_COMPA] void sample(void){
; 0000 0050 interrupt [14] void sample(void){

	.CSEG
_sample:
	ST   -Y,R0
	ST   -Y,R1
	ST   -Y,R15
	ST   -Y,R22
	ST   -Y,R23
	ST   -Y,R24
	ST   -Y,R25
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
; 0000 0051         gCLK=1;     //Raise sample clock (gCLK Port)
	SBI  0x5,0
; 0000 0052         gTRG=1;     //Keep Trigger high during experiment
	SBI  0xB,4
; 0000 0053         #asm("sei")
	sei
; 0000 0054         grabMotion();
	RCALL _grabMotion
; 0000 0055 }
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	LD   R27,Y+
	LD   R26,Y+
	LD   R25,Y+
	LD   R24,Y+
	LD   R23,Y+
	LD   R22,Y+
	LD   R15,Y+
	LD   R1,Y+
	LD   R0,Y+
	RETI
;
;//Time base control for periods greater than one milisecond (such as serial timeout, video frame period, and analog output binning
;interrupt [TIM2_COMPA] void timebase(void)
; 0000 0059 {
_timebase:
	ST   -Y,R0
	ST   -Y,R1
	ST   -Y,R15
	ST   -Y,R22
	ST   -Y,R23
	ST   -Y,R24
	ST   -Y,R25
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
; 0000 005A            tick++;
	INC  R5
; 0000 005B            gaTRG=0;
	CBI  0x8,0
; 0000 005C            if (tick==5)  //5 ticks per ms, update ms clocks
	LDI  R30,LOW(5)
	CP   R30,R5
	BREQ PC+3
	JMP _0x10
; 0000 005D            {
; 0000 005E                 tick=0;
	CLR  R5
; 0000 005F                 time_ms++;
	LDI  R26,LOW(_time_ms)
	LDI  R27,HIGH(_time_ms)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
; 0000 0060                 vidTime++;
	LDS  R30,_vidTime
	SUBI R30,-LOW(1)
	STS  _vidTime,R30
; 0000 0061                 binTime++;
	LDS  R30,_binTime
	SUBI R30,-LOW(1)
	STS  _binTime,R30
; 0000 0062 
; 0000 0063                 if (serialTimeout>1) serialTimeout--;
	LDS  R26,_serialTimeout
	LDS  R27,_serialTimeout+1
	SBIW R26,2
	BRLO _0x11
	LDI  R26,LOW(_serialTimeout)
	LDI  R27,HIGH(_serialTimeout)
	LD   R30,X+
	LD   R31,X+
	SBIW R30,1
	ST   -X,R31
	ST   -X,R30
; 0000 0064                 if (serialTimeout==1){
_0x11:
	LDS  R26,_serialTimeout
	LDS  R27,_serialTimeout+1
	SBIW R26,1
	BRNE _0x12
; 0000 0065                         serialTimeout=0;
	LDI  R30,LOW(0)
	STS  _serialTimeout,R30
	STS  _serialTimeout+1,R30
; 0000 0066                         sstate=0;
	CLR  R11
; 0000 0067                 }
; 0000 0068                 if (vidTime==vTarget&vTarget!=0){
_0x12:
	LDS  R30,_vTarget
	LDS  R26,_vidTime
	CALL __EQB12
	MOV  R0,R30
	LDS  R26,_vTarget
	LDI  R30,LOW(0)
	CALL __NEB12
	AND  R30,R0
	BREQ _0x13
; 0000 0069                         vidTime=0;
	LDI  R30,LOW(0)
	STS  _vidTime,R30
; 0000 006A                         vidFlag=1;
	LDI  R30,LOW(1)
	STS  _vidFlag,R30
; 0000 006B                 }
; 0000 006C                 if (binTime==binTarget&binTarget!=0){
_0x13:
	LDS  R30,_binTarget
	LDS  R26,_binTime
	CALL __EQB12
	MOV  R0,R30
	LDS  R26,_binTarget
	LDI  R30,LOW(0)
	CALL __NEB12
	AND  R30,R0
	BREQ _0x14
; 0000 006D                         binTime=0;
	LDI  R30,LOW(0)
	STS  _binTime,R30
; 0000 006E                         //latch in bin values
; 0000 006F                         bin0xc=bin0x;
	LDS  R30,_bin0x
	STS  _bin0xc,R30
; 0000 0070                         bin0x=128;
	LDI  R30,LOW(128)
	STS  _bin0x,R30
; 0000 0071                         bin0yc=bin0y;
	LDS  R30,_bin0y
	STS  _bin0yc,R30
; 0000 0072                         bin0y=128;
	LDI  R30,LOW(128)
	STS  _bin0y,R30
; 0000 0073                         bin1xc=bin1x;
	LDS  R30,_bin1x
	STS  _bin1xc,R30
; 0000 0074                         bin1x=128;
	LDI  R30,LOW(128)
	STS  _bin1x,R30
; 0000 0075                         bin1yc=bin1y;
	LDS  R30,_bin1y
	STS  _bin1yc,R30
; 0000 0076                         bin1y=128;
	LDI  R30,LOW(128)
	STS  _bin1y,R30
; 0000 0077                         setAnalogOut();
	RCALL _setAnalogOut
; 0000 0078                         gaTRG=1;
	SBI  0x8,0
; 0000 0079                 }
; 0000 007A            }
_0x14:
; 0000 007B }
_0x10:
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	LD   R27,Y+
	LD   R26,Y+
	LD   R25,Y+
	LD   R24,Y+
	LD   R23,Y+
	LD   R22,Y+
	LD   R15,Y+
	LD   R1,Y+
	LD   R0,Y+
	RETI
;
;//Communications Interrupts - USART receive complete (data from user on PC)
;interrupt [USART0_RXC] void serial_receive0(void) // Non-Blocking ISR Driven Read to an input command Buffer
; 0000 007F {
_serial_receive0:
	ST   -Y,R0
	ST   -Y,R1
	ST   -Y,R15
	ST   -Y,R22
	ST   -Y,R23
	ST   -Y,R24
	ST   -Y,R25
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
; 0000 0080         r_char=UDR0;
	LDS  R10,198
; 0000 0081 
; 0000 0082         switch(sstate){
	MOV  R30,R11
; 0000 0083                 case 0:
	CPI  R30,0
	BRNE _0x1A
; 0000 0084                         sstate=r_char;
	MOV  R11,R10
; 0000 0085                         r_index=0;
	LDI  R30,LOW(0)
	STS  _r_index,R30
	STS  _r_index+1,R30
; 0000 0086                         break;
	RJMP _0x19
; 0000 0087                 case 1: //High speed mode (7kHz) - No Shutter or SQual values
_0x1A:
	CPI  R30,LOW(0x1)
	BRNE _0x1B
; 0000 0088                         r_buffer[r_index]=r_char;
	RJMP _0x12A
; 0000 0089                         serialFlag=1;
; 0000 008A                         break;
; 0000 008B                 case 128: //reading in new EEPROM contents from user
_0x1B:
	CPI  R30,LOW(0x80)
	BRNE _0x1C
; 0000 008C                         r_buffer[r_index++]=r_char;
	LDI  R26,LOW(_r_index)
	LDI  R27,HIGH(_r_index)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_r_buffer)
	SBCI R31,HIGH(-_r_buffer)
	ST   Z,R10
; 0000 008D                         if (r_index==1986){
	LDS  R26,_r_index
	LDS  R27,_r_index+1
	CPI  R26,LOW(0x7C2)
	LDI  R30,HIGH(0x7C2)
	CPC  R27,R30
	BRNE _0x1D
; 0000 008E                                 #asm("sei")
	sei
; 0000 008F                                 writeEEPROM();
	RCALL _writeEEPROM
; 0000 0090                                 firmUpload();
	RCALL _firmUpload
; 0000 0091                         }
; 0000 0092                 case 246: //Data return state.  0 = rotation coordinates, 1 = raw camera x/y
_0x1D:
	RJMP _0x1E
_0x1C:
	CPI  R30,LOW(0xF6)
	BRNE _0x1F
_0x1E:
; 0000 0093                         reportState=r_char;
	STS  _reportState,R10
; 0000 0094                         sstate=0;
	CLR  R11
; 0000 0095                         break;
	RJMP _0x19
; 0000 0096                 case 247: //Bin Time for Analog Output
_0x1F:
	CPI  R30,LOW(0xF7)
	BRNE _0x20
; 0000 0097                         r_buffer[r_index++]=r_char;
	LDI  R26,LOW(_r_index)
	LDI  R27,HIGH(_r_index)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_r_buffer)
	SBCI R31,HIGH(-_r_buffer)
	ST   Z,R10
; 0000 0098                         if(r_index==2) serialFlag=1;
	LDS  R26,_r_index
	LDS  R27,_r_index+1
	SBIW R26,2
	BRNE _0x21
	LDI  R30,LOW(1)
	MOV  R13,R30
; 0000 0099                         break;
_0x21:
	RJMP _0x19
; 0000 009A                 case 248:       //Write Arbitrary byte to ADNS
_0x20:
	CPI  R30,LOW(0xF8)
	BRNE _0x22
; 0000 009B                         r_buffer[r_index++]=r_char;
	LDI  R26,LOW(_r_index)
	LDI  R27,HIGH(_r_index)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_r_buffer)
	SBCI R31,HIGH(-_r_buffer)
	ST   Z,R10
; 0000 009C                         if (r_index==2) serialFlag=1;
	LDS  R26,_r_index
	LDS  R27,_r_index+1
	SBIW R26,2
	BRNE _0x23
	LDI  R30,LOW(1)
	MOV  R13,R30
; 0000 009D                         break;
_0x23:
	RJMP _0x19
; 0000 009E                 case 249:       //Read Arbitrary byte from ADNS
_0x22:
	CPI  R30,LOW(0xF9)
	BREQ _0x12A
; 0000 009F                         r_buffer[r_index]=r_char;
; 0000 00A0                         serialFlag=1;
; 0000 00A1                         break;
; 0000 00A2                 case 250: //Turn off video
	CPI  R30,LOW(0xFA)
	BREQ _0x12A
; 0000 00A3                         r_buffer[r_index]=r_char;
; 0000 00A4                         serialFlag=1;
; 0000 00A5                         break;
; 0000 00A6                 case 251: //Read out a frame of pixels from the camera and send it to the user
	CPI  R30,LOW(0xFB)
	BREQ _0x12A
; 0000 00A7                         r_buffer[r_index]=r_char;
; 0000 00A8                         serialFlag=1;
; 0000 00A9                         break;
; 0000 00AA                 case 252: //Dump Internal Registers
	CPI  R30,LOW(0xFC)
	BREQ _0x12A
; 0000 00AB                         r_buffer[r_index]=r_char;
; 0000 00AC                         serialFlag=1;
; 0000 00AD                         break;
; 0000 00AE                 case 253: //Set Sample Rate of Motion Acquisition (2 byte sample period)
	CPI  R30,LOW(0xFD)
	BRNE _0x28
; 0000 00AF                         r_buffer[r_index++]=r_char;
	LDI  R26,LOW(_r_index)
	LDI  R27,HIGH(_r_index)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_r_buffer)
	SBCI R31,HIGH(-_r_buffer)
	ST   Z,R10
; 0000 00B0                         if (r_index==2) serialFlag=1;
	LDS  R26,_r_index
	LDS  R27,_r_index+1
	SBIW R26,2
	BRNE _0x29
	LDI  R30,LOW(1)
	MOV  R13,R30
; 0000 00B1                         break;
_0x29:
	RJMP _0x19
; 0000 00B2                 case 254: //Stop Data Acquisition
_0x28:
	CPI  R30,LOW(0xFE)
	BREQ _0x12A
; 0000 00B3                         r_buffer[r_index]=r_char;
; 0000 00B4                         serialFlag=1;
; 0000 00B5                         break;
; 0000 00B6                 case 255: //Start Data Acquisition
	CPI  R30,LOW(0xFF)
	BRNE _0x19
; 0000 00B7                         r_buffer[r_index]=r_char;
_0x12A:
	LDS  R30,_r_index
	LDS  R31,_r_index+1
	SUBI R30,LOW(-_r_buffer)
	SBCI R31,HIGH(-_r_buffer)
	ST   Z,R10
; 0000 00B8                         serialFlag=1;
	LDI  R30,LOW(1)
	MOV  R13,R30
; 0000 00B9                         break;
; 0000 00BA         }
_0x19:
; 0000 00BB         //Turn on board LED
; 0000 00BC         serialTimeout=500;  //500ms Timeout on serial port commands so the system doesn't hang
	LDI  R30,LOW(500)
	LDI  R31,HIGH(500)
	STS  _serialTimeout,R30
	STS  _serialTimeout+1,R31
; 0000 00BD 
; 0000 00BE }
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	LD   R27,Y+
	LD   R26,Y+
	LD   R25,Y+
	LD   R24,Y+
	LD   R23,Y+
	LD   R22,Y+
	LD   R15,Y+
	LD   R1,Y+
	LD   R0,Y+
	RETI
;
;// ISR Driven Non-Blocking UART Write from FIFO Buffer of length RBUFFL  (Data register empty interrupt)
;interrupt [USART0_DRE] void uart0_send(void)
; 0000 00C2 {
_uart0_send:
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
; 0000 00C3 
; 0000 00C4         if (t_index==t_length){  //End of Buffer
	LDS  R30,_t_length
	LDS  R31,_t_length+1
	LDS  R26,_t_index
	LDS  R27,_t_index+1
	CP   R30,R26
	CPC  R31,R27
	BRNE _0x2C
; 0000 00C5                 UCSR0B=UCSR0B&0b11011111;  //Turn Off Send Loop if @ end of buffer
	LDS  R30,193
	ANDI R30,0xDF
	STS  193,R30
; 0000 00C6         }
; 0000 00C7         else{
	RJMP _0x2D
_0x2C:
; 0000 00C8                 //Keep streaming out buffer
; 0000 00C9                 t_index++;
	LDI  R26,LOW(_t_index)
	LDI  R27,HIGH(_t_index)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
; 0000 00CA                 if (t_index==RBUFFL) t_index=0;
	LDS  R26,_t_index
	LDS  R27,_t_index+1
	CPI  R26,LOW(0x384)
	LDI  R30,HIGH(0x384)
	CPC  R27,R30
	BRNE _0x2E
	LDI  R30,LOW(0)
	STS  _t_index,R30
	STS  _t_index+1,R30
; 0000 00CB                 UDR0=t_buffer[t_index];
_0x2E:
	LDS  R30,_t_index
	LDS  R31,_t_index+1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LD   R30,Z
	STS  198,R30
; 0000 00CC         }
_0x2D:
; 0000 00CD }
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	LD   R27,Y+
	LD   R26,Y+
	RETI
;
;//This function is called on chip reset and power up
;void main(void){
; 0000 00D0 void main(void){
_main:
; 0000 00D1         unsigned int i;
; 0000 00D2 
; 0000 00D3         initialize();
;	i -> R16,R17
	RCALL _initialize
; 0000 00D4 
; 0000 00D5         //Reset Camera Chips
; 0000 00D6         gNCS=0; //Open Communication Interface
	CBI  0x5,2
; 0000 00D7         ADNS_write(0x0a|128);
	LDI  R30,LOW(138)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 00D8         ADNS_write(0b00011100);
	LDI  R30,LOW(28)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 00D9         pause50us();
	RCALL _pause50us
; 0000 00DA         i=0;
	__GETWRN 16,17,0
; 0000 00DB         ADNS_write(0x2C|128);
	LDI  R30,LOW(172)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 00DC         ADNS_write(i);
	ST   -Y,R16
	RCALL _ADNS_write
; 0000 00DD         pause50us();
	RCALL _pause50us
; 0000 00DE         ADNS_write(0x2D|128);
	LDI  R30,LOW(173)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 00DF         ADNS_write(~i);
	MOV  R30,R16
	COM  R30
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 00E0         pause50us();
	RCALL _pause50us
; 0000 00E1         ADNS_write(0x09|128);
	LDI  R30,LOW(137)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 00E2         ADNS_write(0b00000111);
	LDI  R30,LOW(7)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 00E3         gNCS=1;  //Close Communication Interface
	SBI  0x5,2
; 0000 00E4 
; 0000 00E5 
; 0000 00E6         //The "Operating System."
; 0000 00E7         //This while loop spins while interrupts are handled
; 0000 00E8         while(1){
_0x33:
; 0000 00E9                 if (serialFlag==1) commandExec();
	LDI  R30,LOW(1)
	CP   R30,R13
	BRNE _0x36
	RCALL _commandExec
; 0000 00EA                 if (vidFlag==1) grabFrames();
_0x36:
	LDS  R26,_vidFlag
	CPI  R26,LOW(0x1)
	BRNE _0x37
	RCALL _grabFrames
; 0000 00EB         }
_0x37:
	RJMP _0x33
; 0000 00EC }
_0x38:
	RJMP _0x38
;
;void setAnalogOut(void){
; 0000 00EE void setAnalogOut(void){
_setAnalogOut:
; 0000 00EF 
; 0000 00F0         //Make sure chip select is asserted
; 0000 00F1         PORTD.5=0; //CS (active low)
	CBI  0xB,5
; 0000 00F2         //Raise latch to block out changes
; 0000 00F3         PORTC.3=1; //LDAC (active low)
	SBI  0x8,3
; 0000 00F4 
; 0000 00F5         //Address DAC A (Vx0)
; 0000 00F6         PORTD.7=0; //A0
	CBI  0xB,7
; 0000 00F7         PORTC.2=0; //A1
	CBI  0x8,2
; 0000 00F8         //Drop Write input for transition to latch
; 0000 00F9         PORTD.6=0; //WR (active low)
	CBI  0xB,6
; 0000 00FA         //update Vx0
; 0000 00FB         PORTA = bin0xc;
	LDS  R30,_bin0xc
	OUT  0x2,R30
; 0000 00FC         //latch value into register
; 0000 00FD         PORTD.6=1; //Raise WR
	SBI  0xB,6
; 0000 00FE 
; 0000 00FF         //Address DAC B (Vy0)
; 0000 0100         PORTD.7=1; //A0
	SBI  0xB,7
; 0000 0101         PORTC.2=0; //A1
	CBI  0x8,2
; 0000 0102         //Drop Write input for transition to latch
; 0000 0103         PORTD.6=0; //WR (active low)
	CBI  0xB,6
; 0000 0104         //update Vy0
; 0000 0105         PORTA=bin0yc;
	LDS  R30,_bin0yc
	OUT  0x2,R30
; 0000 0106         //latch value into register
; 0000 0107         PORTD.6=1; //Raise WR
	SBI  0xB,6
; 0000 0108 
; 0000 0109         //Address DAC C (Vy1)
; 0000 010A         PORTD.7=0; //A0
	CBI  0xB,7
; 0000 010B         PORTC.2=1; //A1
	SBI  0x8,2
; 0000 010C         //Drop Write input for transition to latch
; 0000 010D         PORTD.6=0; //WR (active low)
	CBI  0xB,6
; 0000 010E         //update Vy0
; 0000 010F         PORTA=bin1yc;
	LDS  R30,_bin1yc
	OUT  0x2,R30
; 0000 0110         //latch value into register
; 0000 0111         PORTD.6=1; //Raise WR
	SBI  0xB,6
; 0000 0112 
; 0000 0113         //Address DAC D (Vx1)
; 0000 0114         PORTD.7=1; //A0
	SBI  0xB,7
; 0000 0115         PORTC.2=1; //A1
	SBI  0x8,2
; 0000 0116         //Drop Write input for transition to latch
; 0000 0117         PORTD.6=0; //WR (active low)
	CBI  0xB,6
; 0000 0118         //update Vy0
; 0000 0119         PORTA=bin1xc;
	LDS  R30,_bin1xc
	OUT  0x2,R30
; 0000 011A         //latch value into register
; 0000 011B         PORTD.6=1; //Raise WR
	SBI  0xB,6
; 0000 011C 
; 0000 011D         //Drop latch to Update all DAC registers
; 0000 011E         PORTC.3=0; //LDAC (active low)
	CBI  0x8,3
; 0000 011F         //deassert chip select
; 0000 0120         PORTD.5=1; //CS (active low)
	SBI  0xB,5
; 0000 0121 
; 0000 0122 }
	RET
;
;void grabMotion(void){
; 0000 0124 void grabMotion(void){
_grabMotion:
; 0000 0125 
; 0000 0126         gNCS=0;
	CBI  0x5,2
; 0000 0127 
; 0000 0128         ADNS_write(0x50);
	LDI  R30,LOW(80)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 0129         TCNT0=0;
	LDI  R30,LOW(0)
	OUT  0x26,R30
; 0000 012A         while(TCNT0<24){};  //~75us allowing for events to be handled
_0x63:
	IN   R30,0x26
	CPI  R30,LOW(0x18)
	BRLO _0x63
; 0000 012B         ADNS_read();
	CALL _ADNS_read
; 0000 012C         motion[0]=ADNS0;
	STS  _motion,R9
; 0000 012D         motion[1]=ADNS1;
	__PUTBMRN _motion,1,8
; 0000 012E         ADNS_read();
	CALL _ADNS_read
; 0000 012F         x[0]=ADNS0;
	STS  _x,R9
; 0000 0130         x[1]=ADNS1;
	__PUTBMRN _x,1,8
; 0000 0131         ADNS_read();
	CALL _ADNS_read
; 0000 0132         y[0]=ADNS0;
	STS  _y,R9
; 0000 0133         y[1]=ADNS1;
	__PUTBMRN _y,1,8
; 0000 0134         ADNS_read();  //Can stop here to increase sample speed
	CALL _ADNS_read
; 0000 0135         squal[0]=ADNS0;
	STS  _squal,R9
; 0000 0136         squal[1]=ADNS1;
	__PUTBMRN _squal,1,8
; 0000 0137         ADNS_read();
	CALL _ADNS_read
; 0000 0138         shut_high[0]=ADNS0;
	STS  _shut_high,R9
; 0000 0139         shut_high[1]=ADNS1;
	__PUTBMRN _shut_high,1,8
; 0000 013A         ADNS_read();
	CALL _ADNS_read
; 0000 013B         shut_low[0]=ADNS0;
	STS  _shut_low,R9
; 0000 013C         shut_low[1]=ADNS1;
	__PUTBMRN _shut_low,1,8
; 0000 013D         gNCS=1;
	SBI  0x5,2
; 0000 013E 
; 0000 013F         //Calculate Fly Based Coordinates (or not)
; 0000 0140         switch(reportState){
	LDS  R30,_reportState
; 0000 0141         case 0:
	CPI  R30,0
	BRNE _0x6B
; 0000 0142                 Vf=(unsigned char)((signed char)y[0]+(signed char)y[1])+128;
	__GETB1MN _y,1
	LDS  R26,_y
	ADD  R30,R26
	SUBI R30,-LOW(128)
	STS  _Vf,R30
; 0000 0143                 Vs=(unsigned char)((signed char)y[0]-(signed char)y[1])+128;
	__GETB2MN _y,1
	LDS  R30,_y
	SUB  R30,R26
	SUBI R30,-LOW(128)
	STS  _Vs,R30
; 0000 0144                 Om=(unsigned char)((signed char)x[0]+(signed char)x[1])+128;
	__GETB1MN _x,1
	LDS  R26,_x
	ADD  R30,R26
	SUBI R30,-LOW(128)
	STS  _Om,R30
; 0000 0145                 x[0]=Vf;
	LDS  R30,_Vf
	STS  _x,R30
; 0000 0146                 y[0]=Vs;
	LDS  R30,_Vs
	STS  _y,R30
; 0000 0147                 x[1]=Om;
	LDS  R30,_Om
	__PUTB1MN _x,1
; 0000 0148                 y[1]=128;
	LDI  R30,LOW(128)
	RJMP _0x12B
; 0000 0149                 break;
; 0000 014A         case 1:
_0x6B:
	CPI  R30,LOW(0x1)
	BRNE _0x6A
; 0000 014B                 x[0]+=128;
	LDS  R30,_x
	SUBI R30,-LOW(128)
	STS  _x,R30
; 0000 014C                 y[0]+=128;
	LDS  R30,_y
	SUBI R30,-LOW(128)
	STS  _y,R30
; 0000 014D                 x[1]+=128;
	__GETB1MN _x,1
	SUBI R30,-LOW(128)
	__PUTB1MN _x,1
; 0000 014E                 y[1]+=128;
	__GETB1MN _y,1
	SUBI R30,-LOW(128)
_0x12B:
	__PUTB1MN _y,1
; 0000 014F                 break;
; 0000 0150         }
_0x6A:
; 0000 0151 
; 0000 0152 
; 0000 0153         //Velocity Bins
; 0000 0154         if (x[0]>128) bin0x=bin0x+((x[0]-128)<<binScale);
	LDS  R26,_x
	CPI  R26,LOW(0x81)
	BRLO _0x6D
	SUBI R26,LOW(128)
	LDS  R30,_binScale
	CALL __LSLB12
	LDS  R26,_bin0x
	ADD  R30,R26
	STS  _bin0x,R30
; 0000 0155         if (x[0]<128) bin0x=bin0x-((128-x[0])<<binScale);
_0x6D:
	LDS  R26,_x
	CPI  R26,LOW(0x80)
	BRSH _0x6E
	LDI  R30,LOW(128)
	SUB  R30,R26
	MOV  R26,R30
	LDS  R30,_binScale
	CALL __LSLB12
	LDS  R26,_bin0x
	SUB  R26,R30
	STS  _bin0x,R26
; 0000 0156         if (y[0]>128) bin0y=bin0y+((y[0]-128)<<binScale);
_0x6E:
	LDS  R26,_y
	CPI  R26,LOW(0x81)
	BRLO _0x6F
	SUBI R26,LOW(128)
	LDS  R30,_binScale
	CALL __LSLB12
	LDS  R26,_bin0y
	ADD  R30,R26
	STS  _bin0y,R30
; 0000 0157         if (y[0]<128) bin0y=bin0y-((128-y[0])<<binScale);
_0x6F:
	LDS  R26,_y
	CPI  R26,LOW(0x80)
	BRSH _0x70
	LDI  R30,LOW(128)
	SUB  R30,R26
	MOV  R26,R30
	LDS  R30,_binScale
	CALL __LSLB12
	LDS  R26,_bin0y
	SUB  R26,R30
	STS  _bin0y,R26
; 0000 0158         if (x[1]>128) bin1x=bin1x+((x[1]-128)<<binScale);
_0x70:
	__GETB2MN _x,1
	CPI  R26,LOW(0x81)
	BRLO _0x71
	__GETB1MN _x,1
	SUBI R30,LOW(128)
	MOV  R26,R30
	LDS  R30,_binScale
	CALL __LSLB12
	LDS  R26,_bin1x
	ADD  R30,R26
	STS  _bin1x,R30
; 0000 0159         if (x[1]<128) bin1x=bin1x-((128-x[1])<<binScale);
_0x71:
	__GETB2MN _x,1
	CPI  R26,LOW(0x80)
	BRSH _0x72
	__GETB2MN _x,1
	LDI  R30,LOW(128)
	SUB  R30,R26
	MOV  R26,R30
	LDS  R30,_binScale
	CALL __LSLB12
	LDS  R26,_bin1x
	SUB  R26,R30
	STS  _bin1x,R26
; 0000 015A         if (y[1]>128) bin1y=bin1y+((y[1]-128)<<binScale);
_0x72:
	__GETB2MN _y,1
	CPI  R26,LOW(0x81)
	BRLO _0x73
	__GETB1MN _y,1
	SUBI R30,LOW(128)
	MOV  R26,R30
	LDS  R30,_binScale
	CALL __LSLB12
	LDS  R26,_bin1y
	ADD  R30,R26
	STS  _bin1y,R30
; 0000 015B         if (y[1]<128) bin1y=bin1y-((128-y[1])<<binScale);
_0x73:
	__GETB2MN _y,1
	CPI  R26,LOW(0x80)
	BRSH _0x74
	__GETB2MN _y,1
	LDI  R30,LOW(128)
	SUB  R30,R26
	MOV  R26,R30
	LDS  R30,_binScale
	CALL __LSLB12
	LDS  R26,_bin1y
	SUB  R26,R30
	STS  _bin1y,R26
; 0000 015C 
; 0000 015D         //Clean Packet Structure so that only byte 0 has a value of 0 in any case
; 0000 015E         if (SampleCount==0) SampleCount=1;
_0x74:
	LDS  R30,_SampleCount
	CPI  R30,0
	BRNE _0x75
	LDI  R30,LOW(1)
	STS  _SampleCount,R30
; 0000 015F         shut_high[0]+=1;
_0x75:
	LDS  R30,_shut_high
	SUBI R30,-LOW(1)
	STS  _shut_high,R30
; 0000 0160         shut_high[1]+=1;
	__GETB1MN _shut_high,1
	SUBI R30,-LOW(1)
	__PUTB1MN _shut_high,1
; 0000 0161         if (shut_low[0]==0) shut_low[0]=1;
	LDS  R30,_shut_low
	CPI  R30,0
	BRNE _0x76
	LDI  R30,LOW(1)
	STS  _shut_low,R30
; 0000 0162         if (shut_low[1]==0) shut_low[1]=1;
_0x76:
	__GETB1MN _shut_low,1
	CPI  R30,0
	BRNE _0x77
	LDI  R30,LOW(1)
	__PUTB1MN _shut_low,1
; 0000 0163         squal[0]+=1;
_0x77:
	LDS  R30,_squal
	SUBI R30,-LOW(1)
	STS  _squal,R30
; 0000 0164         squal[1]+=1;
	__GETB1MN _squal,1
	SUBI R30,-LOW(1)
	__PUTB1MN _squal,1
; 0000 0165 
; 0000 0166         //Output Data to Serial Port Buffer
; 0000 0167         t_length=0;
	LDI  R30,LOW(0)
	STS  _t_length,R30
	STS  _t_length+1,R30
; 0000 0168         t_buffer[t_length++]=0;
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LDI  R26,LOW(0)
	STD  Z+0,R26
; 0000 0169         t_buffer[t_length++]=SampleCount++;
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	MOVW R26,R30
	LDS  R30,_SampleCount
	SUBI R30,-LOW(1)
	STS  _SampleCount,R30
	SUBI R30,LOW(1)
	ST   X,R30
; 0000 016A         t_buffer[t_length++]=x[0];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LDS  R26,_x
	STD  Z+0,R26
; 0000 016B         t_buffer[t_length++]=y[0];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LDS  R26,_y
	STD  Z+0,R26
; 0000 016C         t_buffer[t_length++]=x[1];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	__GETB2MN _x,1
	STD  Z+0,R26
; 0000 016D         t_buffer[t_length++]=y[1];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	__GETB2MN _y,1
	STD  Z+0,R26
; 0000 016E 
; 0000 016F         t_buffer[t_length++]=squal[0];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LDS  R26,_squal
	STD  Z+0,R26
; 0000 0170         t_buffer[t_length++]=squal[1];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	__GETB2MN _squal,1
	STD  Z+0,R26
; 0000 0171         t_buffer[t_length++]=shut_high[0];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LDS  R26,_shut_high
	STD  Z+0,R26
; 0000 0172         t_buffer[t_length++]=shut_low[0];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LDS  R26,_shut_low
	STD  Z+0,R26
; 0000 0173         t_buffer[t_length++]=shut_high[1];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	__GETB2MN _shut_high,1
	STD  Z+0,R26
; 0000 0174         t_buffer[t_length++]=shut_low[1];
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
	SBIW R30,1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	__GETB2MN _shut_low,1
	STD  Z+0,R26
; 0000 0175 
; 0000 0176         //Initialize interrupt driven serial port output (takes about 75us in parallel with this loop to complete)
; 0000 0177         t_index=0;
	LDI  R30,LOW(0)
	STS  _t_index,R30
	STS  _t_index+1,R30
; 0000 0178         t_length=11;
	LDI  R30,LOW(11)
	LDI  R31,HIGH(11)
	STS  _t_length,R30
	STS  _t_length+1,R31
; 0000 0179         UDR0 =t_buffer[t_index];
	LDS  R30,_t_index
	LDS  R31,_t_index+1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LD   R30,Z
	STS  198,R30
; 0000 017A         UCSR0B=UCSR0B|0b00100000;
	LDS  R30,193
	ORI  R30,0x20
	STS  193,R30
; 0000 017B 
; 0000 017C         gCLK=0;  //Lower sample clock (gCLK port)
	CBI  0x5,0
; 0000 017D 
; 0000 017E }
	RET
;
;void commandExec(void){
; 0000 0180 void commandExec(void){
_commandExec:
; 0000 0181         TCCR1B=0b00001000;  //Turn off any motion tracking
	LDI  R30,LOW(8)
	STS  129,R30
; 0000 0182         vTarget=0;      //Turn off any video acquisition
	LDI  R30,LOW(0)
	STS  _vTarget,R30
; 0000 0183         vidFlag=0;
	STS  _vidFlag,R30
; 0000 0184         gTRG=0;
	CBI  0xB,4
; 0000 0185 
; 0000 0186         switch (sstate){
	MOV  R30,R11
; 0000 0187                 case 247:       //Set AO bin speed/scale
	CPI  R30,LOW(0xF7)
	BRNE _0x7F
; 0000 0188                         binTarget=r_buffer[1];
	__GETB1MN _r_buffer,1
	STS  _binTarget,R30
; 0000 0189                         binScale=r_buffer[2];
	__GETB1MN _r_buffer,2
	STS  _binScale,R30
; 0000 018A                         break;
	RJMP _0x7E
; 0000 018B                 case 248:       //Write Arbitrary byte to ADNS
_0x7F:
	CPI  R30,LOW(0xF8)
	BRNE _0x80
; 0000 018C                         gNCS=0;
	CBI  0x5,2
; 0000 018D                         ADNS_write(r_buffer[1]|0x80);
	__GETB1MN _r_buffer,1
	ORI  R30,0x80
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 018E                         pause50us();
	RCALL _pause50us
; 0000 018F                         ADNS_write(r_buffer[2]);
	__GETB1MN _r_buffer,2
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 0190                         gNCS=1;
	SBI  0x5,2
; 0000 0191                         break;
	RJMP _0x7E
; 0000 0192                 case 249:       //Read Arbitrary byte from ADNS
_0x80:
	CPI  R30,LOW(0xF9)
	BRNE _0x85
; 0000 0193                         gNCS=0;
	CBI  0x5,2
; 0000 0194                         ADNS_write(r_buffer[1]);
	__GETB1MN _r_buffer,1
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 0195                         pause50us();
	RCALL _pause50us
; 0000 0196                         ADNS_read();
	CALL _ADNS_read
; 0000 0197                         gNCS=1;
	SBI  0x5,2
; 0000 0198                         puts_int(ADNS0);
	ST   -Y,R9
	RCALL _puts_int
; 0000 0199                         break;
	RJMP _0x7E
; 0000 019A                 case 250:
_0x85:
	CPI  R30,LOW(0xFA)
	BRNE _0x8A
; 0000 019B                         vTarget=0;        //End Video, Reset Chip
	LDI  R30,LOW(0)
	STS  _vTarget,R30
; 0000 019C                         resetADNS();
	RCALL _resetADNS
; 0000 019D                         break;
	RJMP _0x7E
; 0000 019E                 case 251://Handle Video from the camera
_0x8A:
	CPI  R30,LOW(0xFB)
	BRNE _0x8B
; 0000 019F                         vTarget=50;
	LDI  R30,LOW(50)
	STS  _vTarget,R30
; 0000 01A0                         break;
	RJMP _0x7E
; 0000 01A1                 case 252:  //Return status information from cameras
_0x8B:
	CPI  R30,LOW(0xFC)
	BRNE _0x8C
; 0000 01A2                         dumpRegisters();
	RCALL _dumpRegisters
; 0000 01A3                         break;
	RJMP _0x7E
; 0000 01A4                 case 253:       //Set Sample Period
_0x8C:
	CPI  R30,LOW(0xFD)
	BRNE _0x8D
; 0000 01A5                         OCR1AH=r_buffer[1];
	__GETB1MN _r_buffer,1
	STS  137,R30
; 0000 01A6                         OCR1AL=r_buffer[2];
	__GETB1MN _r_buffer,2
	STS  136,R30
; 0000 01A7                         break;
	RJMP _0x7E
; 0000 01A8                 case 254: //Stop Motion Acquisition & video
_0x8D:
	CPI  R30,LOW(0xFE)
	BRNE _0x8E
; 0000 01A9                         vTarget = 0;
	LDI  R30,LOW(0)
	STS  _vTarget,R30
; 0000 01AA                         TCCR1B=0b00001000;
	LDI  R30,LOW(8)
	STS  129,R30
; 0000 01AB                         while(t_index!=t_length){}
_0x8F:
	LDS  R30,_t_length
	LDS  R31,_t_length+1
	LDS  R26,_t_index
	LDS  R27,_t_index+1
	CP   R30,R26
	CPC  R31,R27
	BRNE _0x8F
; 0000 01AC                         #asm("rjmp 0")
	rjmp 0
; 0000 01AD                         break;
	RJMP _0x7E
; 0000 01AE                 case 255: //Start Motion Acquisition
_0x8E:
	CPI  R30,LOW(0xFF)
	BRNE _0x7E
; 0000 01AF                         gNCS=0;
	CBI  0x5,2
; 0000 01B0                         /*ADNS_write(0x0b|128);  //Configure for fixed frame rate
; 0000 01B1                         ADNS_write(0b10001000);
; 0000 01B2                         pause50us();
; 0000 01B3                         ADNS_write(0x19|128);
; 0000 01B4                         ADNS_write(0x05);
; 0000 01B5                         pause50us();
; 0000 01B6                         ADNS_write(0x1a|128);
; 0000 01B7                         ADNS_write(0x0D);    */
; 0000 01B8 
; 0000 01B9                         pause50us();
	RCALL _pause50us
; 0000 01BA                         ADNS_write(0x12|128);  //Motion Clear Register
	LDI  R30,LOW(146)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 01BB                         ADNS_write(0xaa);
	LDI  R30,LOW(170)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 01BC                         pause50us();
	RCALL _pause50us
; 0000 01BD 
; 0000 01BE                         gNCS=1;
	SBI  0x5,2
; 0000 01BF                         TCNT1H=0;
	LDI  R30,LOW(0)
	STS  133,R30
; 0000 01C0                         TCNT1L=0;
	STS  132,R30
; 0000 01C1                         TCCR1B=0b00001010;
	LDI  R30,LOW(10)
	STS  129,R30
; 0000 01C2                         break;
; 0000 01C3 
; 0000 01C4         }
_0x7E:
; 0000 01C5         //Turn off board LED
; 0000 01C6         sstate=0;
	CLR  R11
; 0000 01C7         serialFlag=0;
	CLR  R13
; 0000 01C8 }
	RET
;
;
;void grabFrames(void){
; 0000 01CB void grabFrames(void){
_grabFrames:
; 0000 01CC         unsigned int i;
; 0000 01CD         PORTB.0=1;
	ST   -Y,R17
	ST   -Y,R16
;	i -> R16,R17
	SBI  0x5,0
; 0000 01CE         TCCR1B=0b00001000;  //Stop any Motion Acquisition
	LDI  R30,LOW(8)
	STS  129,R30
; 0000 01CF 
; 0000 01D0         gNCS=0;
	CBI  0x5,2
; 0000 01D1         ADNS_write(0x0a|128);
	LDI  R30,LOW(138)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 01D2         ADNS_write(0b00011100);
	LDI  R30,LOW(28)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 01D3         TCNT0=0;
	LDI  R30,LOW(0)
	OUT  0x26,R30
; 0000 01D4         while(TCNT0<16){};  //~50us allowing for events to be handled
_0x9B:
	IN   R30,0x26
	CPI  R30,LOW(0x10)
	BRLO _0x9B
; 0000 01D5         ADNS_write(0x13|128);
	LDI  R30,LOW(147)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 01D6         ADNS_write(0x83);
	LDI  R30,LOW(131)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 01D7         gNCS=1;
	SBI  0x5,2
; 0000 01D8 
; 0000 01D9         time_ms=0;
	LDI  R30,LOW(0)
	STS  _time_ms,R30
	STS  _time_ms+1,R30
; 0000 01DA         while(time_ms<3){} //Wait 10us + 3 frame periods, 3ms should cover that.  Can be less
_0xA0:
	LDS  R26,_time_ms
	LDS  R27,_time_ms+1
	SBIW R26,3
	BRLO _0xA0
; 0000 01DB 
; 0000 01DC         gNCS=0;
	CBI  0x5,2
; 0000 01DD         ADNS_write(0x40);
	LDI  R30,LOW(64)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 01DE         TCNT0=0;
	LDI  R30,LOW(0)
	OUT  0x26,R30
; 0000 01DF         while(TCNT0<16){};  //~50us allowing for events to be handled
_0xA5:
	IN   R30,0x26
	CPI  R30,LOW(0x10)
	BRLO _0xA5
; 0000 01E0 
; 0000 01E1         for (i=0; i<RBUFFL; i++){
	__GETWRN 16,17,0
_0xA9:
	__CPWRN 16,17,900
	BRSH _0xAA
; 0000 01E2                 ADNS_read();
	CALL _ADNS_read
; 0000 01E3                 puts_int(ADNS0);
	ST   -Y,R9
	RCALL _puts_int
; 0000 01E4                 puts_int(ADNS1);
	ST   -Y,R8
	RCALL _puts_int
; 0000 01E5                 TCNT0=0;
	LDI  R30,LOW(0)
	OUT  0x26,R30
; 0000 01E6                 while(TCNT0<4){};  //~10us allowing for events to be handled
_0xAB:
	IN   R30,0x26
	CPI  R30,LOW(0x4)
	BRLO _0xAB
; 0000 01E7         }
	__ADDWRN 16,17,1
	RJMP _0xA9
_0xAA:
; 0000 01E8 
; 0000 01E9         //Send SQUAL
; 0000 01EA         //ADNS_write(0x05);
; 0000 01EB         //pause50us();
; 0000 01EC         //ADNS_read();
; 0000 01ED         //puts_int(ADNS0);
; 0000 01EE         //puts_int(ADNS1);
; 0000 01EF 
; 0000 01F0         gNCS=1;
	SBI  0x5,2
; 0000 01F1 
; 0000 01F2         vidFlag=0;
	LDI  R30,LOW(0)
	STS  _vidFlag,R30
; 0000 01F3         PORTB.0=0;
	CBI  0x5,0
; 0000 01F4 }
	RJMP _0x2000003
;
;//Write the contents of r_buffer to the EEPROM for uploading the local firmware if Avago upgrades
;void writeEEPROM(void){
; 0000 01F7 void writeEEPROM(void){
_writeEEPROM:
; 0000 01F8         unsigned int i;
; 0000 01F9 
; 0000 01FA         for (i=0; i<1986; i++){
	ST   -Y,R17
	ST   -Y,R16
;	i -> R16,R17
	__GETWRN 16,17,0
_0xB3:
	__CPWRN 16,17,1986
	BRSH _0xB4
; 0000 01FB                 PINB.0=1;
	SBI  0x3,0
; 0000 01FC                 while(EECR.1==1){};
_0xB7:
	SBIC 0x1F,1
	RJMP _0xB7
; 0000 01FD                 while(SPMCSR&0x01==1){};
_0xBA:
	IN   R30,0x37
	SBRC R30,0
	RJMP _0xBA
; 0000 01FE                 EEARH=i>>8;
	MOV  R30,R17
	ANDI R31,HIGH(0x0)
	OUT  0x22,R30
; 0000 01FF                 EEARL=i;
	OUT  0x21,R16
; 0000 0200                 EEDR=r_buffer[i];
	LDI  R26,LOW(_r_buffer)
	LDI  R27,HIGH(_r_buffer)
	ADD  R26,R16
	ADC  R27,R17
	LD   R30,X
	OUT  0x20,R30
; 0000 0201 
; 0000 0202                 EECR=0b00000100;
	LDI  R30,LOW(4)
	OUT  0x1F,R30
; 0000 0203                 EECR=0b00000110;
	LDI  R30,LOW(6)
	OUT  0x1F,R30
; 0000 0204         }
	__ADDWRN 16,17,1
	RJMP _0xB3
_0xB4:
; 0000 0205 }
_0x2000003:
	LD   R16,Y+
	LD   R17,Y+
	RET
;
;//Upload EEPROM Contents into ADNS Chip
;void firmUpload(void){
; 0000 0208 void firmUpload(void){
_firmUpload:
; 0000 0209         unsigned int i, address;
; 0000 020A 
; 0000 020B         gNCS=0;
	CALL __SAVELOCR4
;	i -> R16,R17
;	address -> R18,R19
	CBI  0x5,2
; 0000 020C         //Write 0x1d to register 0x14 (SROM_enable register)
; 0000 020D         ADNS_write(0x14|128);
	LDI  R30,LOW(148)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 020E         ADNS_write(0x1D);
	LDI  R30,LOW(29)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 020F         gNCS=1;
	SBI  0x5,2
; 0000 0210         //Wait at least 1 frame period
; 0000 0211         time_ms=0;
	LDI  R30,LOW(0)
	STS  _time_ms,R30
	STS  _time_ms+1,R30
; 0000 0212         while(time_ms<5){};
_0xC1:
	LDS  R26,_time_ms
	LDS  R27,_time_ms+1
	SBIW R26,5
	BRLO _0xC1
; 0000 0213         gNCS=0;
	CBI  0x5,2
; 0000 0214         //Write 0x18 to register 0x14 (SROM_enable register)
; 0000 0215         ADNS_write(0x14|128);
	LDI  R30,LOW(148)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 0216         ADNS_write(0x18);
	LDI  R30,LOW(24)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 0217         gNCS=1;
	SBI  0x5,2
; 0000 0218         pause50us();
	RCALL _pause50us
; 0000 0219         gNCS=0;
	CBI  0x5,2
; 0000 021A 
; 0000 021B         //Begin burst mode writing
; 0000 021C         ADNS_write(0x60|128);
	LDI  R30,LOW(224)
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 021D 
; 0000 021E         //EEPROM Address register to 0
; 0000 021F 
; 0000 0220         while(EECR.1==1) {PINB.0=1;}
_0xCA:
	SBIS 0x1F,1
	RJMP _0xCC
	SBI  0x3,0
	RJMP _0xCA
_0xCC:
; 0000 0221         address=0;
	__GETWRN 18,19,0
; 0000 0222         EEARH=address>>8;
	MOV  R30,R19
	ANDI R31,HIGH(0x0)
	OUT  0x22,R30
; 0000 0223         EEARL=address;
	OUT  0x21,R18
; 0000 0224 
; 0000 0225         for (i=0; i<1986; i++){
	__GETWRN 16,17,0
_0xD0:
	__CPWRN 16,17,1986
	BRSH _0xD1
; 0000 0226                 pause10us();
	RCALL _pause10us
; 0000 0227 
; 0000 0228                 while(EECR.1==1){};
_0xD2:
	SBIC 0x1F,1
	RJMP _0xD2
; 0000 0229 
; 0000 022A                 //Execute Read Command w/ increment
; 0000 022B                 EECR=0b00000001;
	LDI  R30,LOW(1)
	OUT  0x1F,R30
; 0000 022C                 address++;
	__ADDWRN 18,19,1
; 0000 022D                 EEARH=address>>8;
	MOV  R30,R19
	ANDI R31,HIGH(0x0)
	OUT  0x22,R30
; 0000 022E                 EEARL=address;
	OUT  0x21,R18
; 0000 022F 
; 0000 0230                 ADNS=EEDR;
	IN   R4,32
; 0000 0231                 ADNS_write(ADNS);
	ST   -Y,R4
	RCALL _ADNS_write
; 0000 0232                 pause10us();
	RCALL _pause10us
; 0000 0233         }
	__ADDWRN 16,17,1
	RJMP _0xD0
_0xD1:
; 0000 0234 
; 0000 0235         gNCS=1;
	SBI  0x5,2
; 0000 0236         pause50us();
	RCALL _pause50us
; 0000 0237         pause50us();
	RCALL _pause50us
; 0000 0238         pause50us();
	RCALL _pause50us
; 0000 0239 }
	CALL __LOADLOCR4
	ADIW R28,4
	RET
;
;//Reset Chip, takes 200ms
;void resetADNS(void){
; 0000 023C void resetADNS(void){
_resetADNS:
; 0000 023D         //Toggle Reset
; 0000 023E         gRESET=1;
	SBI  0x5,1
; 0000 023F         pause50us();
	RCALL _pause50us
; 0000 0240         gRESET=0;
	CBI  0x5,1
; 0000 0241 
; 0000 0242         //wait for inputs & motion data to be valid
; 0000 0243         time_ms=0;
	LDI  R30,LOW(0)
	STS  _time_ms,R30
	STS  _time_ms+1,R30
; 0000 0244         while(time_ms<200){};
_0xDB:
	LDS  R26,_time_ms
	LDS  R27,_time_ms+1
	CPI  R26,LOW(0xC8)
	LDI  R30,HIGH(0xC8)
	CPC  R27,R30
	BRLO _0xDB
; 0000 0245 
; 0000 0246 }
	RET
;
;//DUMP Status Values from Each Camera to the User (currently only camera 0)
;void dumpRegisters(void){
; 0000 0249 void dumpRegisters(void){
_dumpRegisters:
; 0000 024A         unsigned char regLoc[25]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x09,0x0a,0x0b,0x0e,0x0f,0x10,0x11,0x16,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x2c,0x2d,0x3d};
; 0000 024B         unsigned char i;
; 0000 024C 
; 0000 024D         gNCS=0;
	SBIW R28,25
	LDI  R24,25
	LDI  R26,LOW(0)
	LDI  R27,HIGH(0)
	LDI  R30,LOW(_0xDE*2)
	LDI  R31,HIGH(_0xDE*2)
	CALL __INITLOCB
	ST   -Y,R17
;	regLoc -> Y+1
;	i -> R17
	CBI  0x5,2
; 0000 024E 
; 0000 024F         for (i=0; i<25; i++){
	LDI  R17,LOW(0)
_0xE2:
	CPI  R17,25
	BRSH _0xE3
; 0000 0250                 ADNS_write(regLoc[i]);
	MOV  R30,R17
	LDI  R31,0
	MOVW R26,R28
	ADIW R26,1
	ADD  R26,R30
	ADC  R27,R31
	LD   R30,X
	ST   -Y,R30
	RCALL _ADNS_write
; 0000 0251                 TCNT0=0;
	LDI  R30,LOW(0)
	OUT  0x26,R30
; 0000 0252                 while(TCNT0<16){};  //~50us allowing for events to be handled
_0xE4:
	IN   R30,0x26
	CPI  R30,LOW(0x10)
	BRLO _0xE4
; 0000 0253                 ADNS_read();
	CALL _ADNS_read
; 0000 0254                 puts_int(ADNS0);
	ST   -Y,R9
	RCALL _puts_int
; 0000 0255                 TCNT0=0;
	LDI  R30,LOW(0)
	OUT  0x26,R30
; 0000 0256                 while(TCNT0<16){};  //~50us allowing for events to be handled
_0xE7:
	IN   R30,0x26
	CPI  R30,LOW(0x10)
	BRLO _0xE7
; 0000 0257         }
	SUBI R17,-1
	RJMP _0xE2
_0xE3:
; 0000 0258 
; 0000 0259 
; 0000 025A         gNCS=1;
	SBI  0x5,2
; 0000 025B }
	LDD  R17,Y+0
	ADIW R28,26
	RET
;
;//Dump EEPROM for firmware verification
;void EEPROMDump(void){
; 0000 025E void EEPROMDump(void){
; 0000 025F         unsigned int i;
; 0000 0260 
; 0000 0261         for (i=0; i<1986; i++){
;	i -> R16,R17
; 0000 0262                 while(EECR.1==1){};
; 0000 0263                 //Execute Read Command w/ increment
; 0000 0264                 EEARH=i>>8;
; 0000 0265                 EEARL=i;
; 0000 0266                 EECR=0b00000001;
; 0000 0267                 puts_int(EEDR);
; 0000 0268                 pause50us();
; 0000 0269         }
; 0000 026A }
;
;//Pause for communication protocol
;void pause50us(void){
; 0000 026D void pause50us(void){
_pause50us:
; 0000 026E         unsigned char i=0;
; 0000 026F         while(i<250){
	ST   -Y,R17
;	i -> R17
	LDI  R17,0
_0xF2:
	CPI  R17,250
	BRSH _0xF4
; 0000 0270                 i++;
	SUBI R17,-1
; 0000 0271         }
	RJMP _0xF2
_0xF4:
; 0000 0272 }
	RJMP _0x2000002
;//Pause for communication protocol
;void pause10us(void){
; 0000 0274 void pause10us(void){
_pause10us:
; 0000 0275         unsigned char i=0;
; 0000 0276         while(i<40){
	ST   -Y,R17
;	i -> R17
	LDI  R17,0
_0xF5:
	CPI  R17,40
	BRSH _0xF7
; 0000 0277                 i++;
	SUBI R17,-1
; 0000 0278         }
	RJMP _0xF5
_0xF7:
; 0000 0279 }
_0x2000002:
	LD   R17,Y+
	RET
;
;//Initiate an interrupt driven UART output of a buffer's contents
;//or add a character to the FIFO buffer for transmit
;void puts_int(unsigned char newT)
; 0000 027E {
_puts_int:
; 0000 027F         UCSR0B=UCSR0B&0b11011111;  //Turn off DRE interrupt so that an interrupt doesn't happend during this fast buffer access and cause index/length mismatch
;	newT -> Y+0
	LDS  R30,193
	ANDI R30,0xDF
	STS  193,R30
; 0000 0280         t_length++;
	LDI  R26,LOW(_t_length)
	LDI  R27,HIGH(_t_length)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
; 0000 0281         if (t_length==RBUFFL) t_length=0;
	LDS  R26,_t_length
	LDS  R27,_t_length+1
	CPI  R26,LOW(0x384)
	LDI  R30,HIGH(0x384)
	CPC  R27,R30
	BRNE _0xF8
	LDI  R30,LOW(0)
	STS  _t_length,R30
	STS  _t_length+1,R30
; 0000 0282         t_buffer[t_length]=newT;
_0xF8:
	LDS  R30,_t_length
	LDS  R31,_t_length+1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LD   R26,Y
	STD  Z+0,R26
; 0000 0283 
; 0000 0284         if (((UCSR0A&0b00100000)==0b00100000))
	LDS  R30,192
	ANDI R30,LOW(0x20)
	CPI  R30,LOW(0x20)
	BRNE _0xF9
; 0000 0285         {       //if not outputting, start subsystem
; 0000 0286                 t_index++;
	LDI  R26,LOW(_t_index)
	LDI  R27,HIGH(_t_index)
	LD   R30,X+
	LD   R31,X+
	ADIW R30,1
	ST   -X,R31
	ST   -X,R30
; 0000 0287                 if (t_index==RBUFFL) t_index=0;
	LDS  R26,_t_index
	LDS  R27,_t_index+1
	CPI  R26,LOW(0x384)
	LDI  R30,HIGH(0x384)
	CPC  R27,R30
	BRNE _0xFA
	LDI  R30,LOW(0)
	STS  _t_index,R30
	STS  _t_index+1,R30
; 0000 0288                 UDR0=t_buffer[t_index];
_0xFA:
	LDS  R30,_t_index
	LDS  R31,_t_index+1
	SUBI R30,LOW(-_t_buffer)
	SBCI R31,HIGH(-_t_buffer)
	LD   R30,Z
	STS  198,R30
; 0000 0289                 UCSR0B=UCSR0B|0b00100000;
	LDS  R30,193
	ORI  R30,0x20
	STS  193,R30
; 0000 028A         }
; 0000 028B         UCSR0B=UCSR0B|0b00100000;  //Re-enable DRE interrupt
_0xF9:
	LDS  R30,193
	ORI  R30,0x20
	STS  193,R30
; 0000 028C }
	JMP  _0x2000001
;
;
;
;
;void initialize(void){
; 0000 0291 void initialize(void){
_initialize:
; 0000 0292 
; 0000 0293 
; 0000 0294         //Timer1 Setup for Sample Period, 16-bits for more timer resolution
; 0000 0295         TCCR1A=0b00000000;
	LDI  R30,LOW(0)
	STS  128,R30
; 0000 0296         TCCR1B=0b00001000;
	LDI  R30,LOW(8)
	STS  129,R30
; 0000 0297         TCCR1C=0b00000000;
	LDI  R30,LOW(0)
	STS  130,R30
; 0000 0298         TCNT1H=0;
	STS  133,R30
; 0000 0299         TCNT1L=0;
	STS  132,R30
; 0000 029A         OCR1AH=2;   //4kHz
	LDI  R30,LOW(2)
	STS  137,R30
; 0000 029B         OCR1AL=112;
	LDI  R30,LOW(112)
	STS  136,R30
; 0000 029C         //OCR1AH=1;   //5kHz
; 0000 029D         //OCR1AL=243;
; 0000 029E 
; 0000 029F         TIMSK1=0b00000010;
	LDI  R30,LOW(2)
	STS  111,R30
; 0000 02A0 
; 0000 02A1         //Timer2 Setup for 1ms timebase (this counts 0.2 ms exactly), 5 ticks = 1ms
; 0000 02A2         TCCR2A=0b00000010;
	STS  176,R30
; 0000 02A3         TCCR2B=0b00000011;
	LDI  R30,LOW(3)
	STS  177,R30
; 0000 02A4         TCNT2=0b00000000;
	LDI  R30,LOW(0)
	STS  178,R30
; 0000 02A5         OCR2A=125;
	LDI  R30,LOW(125)
	STS  179,R30
; 0000 02A6         TIMSK2=0b00000010;
	LDI  R30,LOW(2)
	STS  112,R30
; 0000 02A7 
; 0000 02A8         //Timer0 setup for delay timing
; 0000 02A9         TCCR0A=0b00000000;
	LDI  R30,LOW(0)
	OUT  0x24,R30
; 0000 02AA         TIMSK0=0b00000000;
	STS  110,R30
; 0000 02AB         TCCR0B=0b00000011;
	LDI  R30,LOW(3)
	OUT  0x25,R30
; 0000 02AC         TCNT0=0;
	LDI  R30,LOW(0)
	OUT  0x26,R30
; 0000 02AD 
; 0000 02AE         //Setup UART0 for send/receive w/ user
; 0000 02AF         UBRR0H=0x00;
	STS  197,R30
; 0000 02B0         UBRR0L=1;  //1=1.25M, 4=0.5M, 21 = 115.2k, 42 = 57.6k
	LDI  R30,LOW(1)
	STS  196,R30
; 0000 02B1         UCSR0A=0b00000010;
	LDI  R30,LOW(2)
	STS  192,R30
; 0000 02B2         UCSR0B=0b10011000;
	LDI  R30,LOW(152)
	STS  193,R30
; 0000 02B3         UCSR0C=0b00000110;
	LDI  R30,LOW(6)
	STS  194,R30
; 0000 02B4 
; 0000 02B5         //Setup UART1 for comms with Other devices
; 0000 02B6         //UBRR1H=0x00;
; 0000 02B7         //UBRR1L=42;  //1=1.25M, 4=0.5M, 21 = 115.2k, 42 = 57.6k
; 0000 02B8         //UCSR1A=0b00000010;
; 0000 02B9         //UCSR1B=0b10011000;
; 0000 02BA         //UCSR1C=0b00000110;
; 0000 02BB 
; 0000 02BC         //Activate interrupts
; 0000 02BD         #asm("sei")
	sei
; 0000 02BE 
; 0000 02BF         //Raise Global NCS to reset ADNS comm interface
; 0000 02C0         gdNCS=1;
	SBI  0x4,2
; 0000 02C1         gNCS=1;
	SBI  0x5,2
; 0000 02C2         //Setup Read Ports for input
; 0000 02C3         gdMISO_0=0;  //Camera 0 (MISO)
	CBI  0x4,4
; 0000 02C4         gdMISO_1=0;  //Camera 1 (MISO)
	CBI  0x4,3
; 0000 02C5         //Setup Commoun Output ports for Writing to Cameras
; 0000 02C6         gdSCLK=1;
	SBI  0x4,6
; 0000 02C7         gSCLK=0;  //SCLK
	CBI  0x5,6
; 0000 02C8         gdMOSI=1;
	SBI  0x4,5
; 0000 02C9         gMOSI=0;  //MOSI
	CBI  0x5,5
; 0000 02CA         //Lower Active High Reset Pin
; 0000 02CB         gdRESET=1;
	SBI  0x4,1
; 0000 02CC         gRESET=0;
	CBI  0x5,1
; 0000 02CD 
; 0000 02CE         gdCLK=1;
	SBI  0x4,0
; 0000 02CF         gCLK=0;
	CBI  0x5,0
; 0000 02D0         gdTRG=1;
	SBI  0xA,4
; 0000 02D1         gTRG=0;
	CBI  0xB,4
; 0000 02D2         gdaTRG=1;
	SBI  0x7,0
; 0000 02D3         gaTRG=0;
	CBI  0x8,0
; 0000 02D4 
; 0000 02D5         //DAC Communications
; 0000 02D6         DDRA=0xff;
	LDI  R30,LOW(255)
	OUT  0x1,R30
; 0000 02D7         DDRD=0xff;
	OUT  0xA,R30
; 0000 02D8         DDRC=0xff;
	OUT  0x7,R30
; 0000 02D9         PORTD.5=1; //CS
	SBI  0xB,5
; 0000 02DA 
; 0000 02DB         //Reset ADNS6090 Camera Chips
; 0000 02DC         time_ms=0;
	LDI  R30,LOW(0)
	STS  _time_ms,R30
	STS  _time_ms+1,R30
; 0000 02DD         while (time_ms==0){}
_0x11D:
	LDS  R30,_time_ms
	LDS  R31,_time_ms+1
	SBIW R30,0
	BREQ _0x11D
; 0000 02DE         gRESET=1; //Raise Reset pin
	SBI  0x5,1
; 0000 02DF         time_ms=0;
	LDI  R30,LOW(0)
	STS  _time_ms,R30
	STS  _time_ms+1,R30
; 0000 02E0         while(time_ms<200){}
_0x122:
	LDS  R26,_time_ms
	LDS  R27,_time_ms+1
	CPI  R26,LOW(0xC8)
	LDI  R30,HIGH(0xC8)
	CPC  R27,R30
	BRLO _0x122
; 0000 02E1         gRESET=0; //Drop Reset Pin
	CBI  0x5,1
; 0000 02E2         time_ms=0;
	LDI  R30,LOW(0)
	STS  _time_ms,R30
	STS  _time_ms+1,R30
; 0000 02E3         while(time_ms<180){}  //Motion Data is Valid 180ms after reset drops
_0x127:
	LDS  R26,_time_ms
	LDS  R27,_time_ms+1
	CPI  R26,LOW(0xB4)
	LDI  R30,HIGH(0xB4)
	CPC  R27,R30
	BRLO _0x127
; 0000 02E4 
; 0000 02E5 }
	RET
;
;
;//Tight Assembly Code loop for bit banging protocol @1MHz to WRITE a byte to the bus
;//byte to write is stored in ADNS (r4) but is passed as an input argument to the function
;void ADNS_write(unsigned char data){
; 0000 02EA void ADNS_write(unsigned char data){
_ADNS_write:
; 0000 02EB         ADNS=data;
;	data -> Y+0
	LDD  R4,Y+0
; 0000 02EC         #asm
; 0000 02ED                 mov _gTemp,_ADNS
                mov _gTemp,_ADNS
; 0000 02EE                 clr r22
                clr r22
; 0000 02EF         gWriteLoop:
        gWriteLoop:
; 0000 02F0                 cbi $05,6       ;PortB = $05, Drop clock
                cbi $05,6       ;PortB = $05, Drop clock
; 0000 02F1                 nop
                nop
; 0000 02F2                 nop
                nop
; 0000 02F3                 nop
                nop
; 0000 02F4                 nop
                nop
; 0000 02F5                 nop
                nop
; 0000 02F6                 nop
                nop
; 0000 02F7 

; 0000 02F8                 sbrs _gTemp,7       ;Conditional statement to clock out bit 7
                sbrs _gTemp,7       ;Conditional statement to clock out bit 7
; 0000 02F9                 cbi $05,5
                cbi $05,5
; 0000 02FA                 sbrc _gTemp,7
                sbrc _gTemp,7
; 0000 02FB                 sbi $05,5
                sbi $05,5
; 0000 02FC 

; 0000 02FD                 sbi $05,6       ;Raise Clock
                sbi $05,6       ;Raise Clock
; 0000 02FE                 nop
                nop
; 0000 02FF                 nop
                nop
; 0000 0300                 nop
                nop
; 0000 0301                 nop
                nop
; 0000 0302                 nop
                nop
; 0000 0303                 nop
                nop
; 0000 0304                 lsl _gTemp          ;Logial Shift register left to place next bit in 7th position
                lsl _gTemp          ;Logial Shift register left to place next bit in 7th position
; 0000 0305                 inc r22
                inc r22
; 0000 0306                 cpi r22,8
                cpi r22,8
; 0000 0307                 breq gWriteOut
                breq gWriteOut
; 0000 0308                 rjmp gWriteLoop
                rjmp gWriteLoop
; 0000 0309         gWriteOut:
        gWriteOut:
; 0000 030A 

; 0000 030B         #endasm
; 0000 030C 
; 0000 030D }
_0x2000001:
	ADIW R28,1
	RET
;
;//Tight Assembly code 1MHz bit banging protocol to READ a pair of parallel bytes into the ADNS0 and ADNS1  registers
;void ADNS_read(void){
; 0000 0310 void ADNS_read(void){
_ADNS_read:
; 0000 0311         #asm
; 0000 0312                 ;r22 is free for arithmatic  (according to codevision documentation)
                ;r22 is free for arithmatic  (according to codevision documentation)
; 0000 0313                 clr _ADNS0          ;clear ADNS data register
                clr _ADNS0          ;clear ADNS data register
; 0000 0314                 clr _ADNS1
                clr _ADNS1
; 0000 0315                 ldi r22,0x01
                ldi r22,0x01
; 0000 0316                 mov _gTemp2,r22      ;gTemp2 is mask
                mov _gTemp2,r22      ;gTemp2 is mask
; 0000 0317                 clr r22         ;Increment through loop
                clr r22         ;Increment through loop
; 0000 0318 

; 0000 0319         gReadLoop:
        gReadLoop:
; 0000 031A                 lsl _ADNS0          ;Shift ADNS0 to left by 1
                lsl _ADNS0          ;Shift ADNS0 to left by 1
; 0000 031B                 lsl _ADNS1          ;Shift ADNS1 to left by 1
                lsl _ADNS1          ;Shift ADNS1 to left by 1
; 0000 031C                 cbi $05,6       ;drop clock to clock out bit from ADNS chip
                cbi $05,6       ;drop clock to clock out bit from ADNS chip
; 0000 031D                 nop
                nop
; 0000 031E                 inc r22
                inc r22
; 0000 031F                 nop
                nop
; 0000 0320                 nop
                nop
; 0000 0321                 nop
                nop
; 0000 0322                 clr _gTemp
                clr _gTemp
; 0000 0323                 sbi $05,6       ;raise clock
                sbi $05,6       ;raise clock
; 0000 0324                 nop
                nop
; 0000 0325                 nop
                nop
; 0000 0326                 nop
                nop
; 0000 0327                 in _gTemp,$03       ;read in the pin values on port b
                in _gTemp,$03       ;read in the pin values on port b
; 0000 0328 

; 0000 0329                 sbrc _gTemp,4
                sbrc _gTemp,4
; 0000 032A                 or _ADNS0,_gTemp2        ;if bit 4 is set, MISO0 is high, set LSB of ADNS0
                or _ADNS0,_gTemp2        ;if bit 4 is set, MISO0 is high, set LSB of ADNS0
; 0000 032B                 sbrc _gTemp,3
                sbrc _gTemp,3
; 0000 032C                 or _ADNS1,_gTemp2        ;if bit 3 is set, MISO1 is high, set LSB of ADNS1
                or _ADNS1,_gTemp2        ;if bit 3 is set, MISO1 is high, set LSB of ADNS1
; 0000 032D 

; 0000 032E 

; 0000 032F                 cpi r22,8
                cpi r22,8
; 0000 0330                 breq gReadOut
                breq gReadOut
; 0000 0331                 rjmp gReadLoop
                rjmp gReadLoop
; 0000 0332         gReadOut:
        gReadOut:
; 0000 0333         #endasm
; 0000 0334 
; 0000 0335 }
	RET
;

	.DSEG
_x:
	.BYTE 0x2
_y:
	.BYTE 0x2
_motion:
	.BYTE 0x2
_squal:
	.BYTE 0x2
_shut_low:
	.BYTE 0x2
_shut_high:
	.BYTE 0x2
_t_buffer:
	.BYTE 0x384
_r_buffer:
	.BYTE 0x384
_vTarget:
	.BYTE 0x1
_vidTime:
	.BYTE 0x1
_vidFlag:
	.BYTE 0x1
_Vf:
	.BYTE 0x1
_Vs:
	.BYTE 0x1
_Om:
	.BYTE 0x1
_SampleCount:
	.BYTE 0x1
_reportState:
	.BYTE 0x1
_time_ms:
	.BYTE 0x2
_serialTimeout:
	.BYTE 0x2
_t_index:
	.BYTE 0x2
_t_length:
	.BYTE 0x2
_r_index:
	.BYTE 0x2
_binTime:
	.BYTE 0x1
_binTarget:
	.BYTE 0x1
_bin0x:
	.BYTE 0x1
_bin0y:
	.BYTE 0x1
_bin1x:
	.BYTE 0x1
_bin1y:
	.BYTE 0x1
_binScale:
	.BYTE 0x1
_bin0xc:
	.BYTE 0x1
_bin0yc:
	.BYTE 0x1
_bin1xc:
	.BYTE 0x1
_bin1yc:
	.BYTE 0x1

	.CSEG

	.CSEG
__LSLB12:
	TST  R30
	MOV  R0,R30
	MOV  R30,R26
	BREQ __LSLB12R
__LSLB12L:
	LSL  R30
	DEC  R0
	BRNE __LSLB12L
__LSLB12R:
	RET

__EQB12:
	CP   R30,R26
	LDI  R30,1
	BREQ __EQB12T
	CLR  R30
__EQB12T:
	RET

__NEB12:
	CP   R30,R26
	LDI  R30,1
	BRNE __NEB12T
	CLR  R30
__NEB12T:
	RET

__SAVELOCR4:
	ST   -Y,R19
__SAVELOCR3:
	ST   -Y,R18
__SAVELOCR2:
	ST   -Y,R17
	ST   -Y,R16
	RET

__LOADLOCR4:
	LDD  R19,Y+3
__LOADLOCR3:
	LDD  R18,Y+2
__LOADLOCR2:
	LDD  R17,Y+1
	LD   R16,Y
	RET

__INITLOCB:
__INITLOCW:
	ADD  R26,R28
	ADC  R27,R29
__INITLOC0:
	LPM  R0,Z+
	ST   X+,R0
	DEC  R24
	BRNE __INITLOC0
	RET

;END OF CODE MARKER
__END_OF_CODE:
