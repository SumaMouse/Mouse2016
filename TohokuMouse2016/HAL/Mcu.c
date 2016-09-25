#include	<machine.h>
#include	"iodefine.h"
#include	"Common.h"

#include	"Mcu.h"

static void waitClockSet(u8 count);

void	McuInit(void) {

	/* ���W�X�^���C�g�v���e�N�V�����ݒ� */
	SYSTEM.PRCR.WORD = 0xA503;			//���W�X�^���C�g����
	
	
	/* �T�u�N���b�N��~�ݒ�(48Pin�łł̓T�u�N���b�N���������ߐݒ�K�v) */
	SYSTEM.SOSCCR.BYTE = 0x00;
	while(SYSTEM.SOSCCR.BYTE != 0x00);	//���������̂�҂�
	RTC.RCR4.BYTE = 0x01;				//���C���N���b�N�I��
	RTC.RCR3.BYTE = 0x02;				//�T�u�N���b�N��~
	while(RTC.RCR3.BYTE != 0x02);		//���������̂�҂�
	RTC.RCR2.BIT.START = 0;				//RTC��~
	while(RTC.RCR2.BIT.START != 0);		//���������̂�҂�


	/* �N���b�N����҂��J�E���g�p�^�C�}������ */
    MSTP(TMR01) = 0;					//TMR01�X�g�b�v����
	TMR0.TCCR.BYTE = 0x0C;				//�J�E���g�N���b�N=PCLKA/1024=1.95kHz	(������Ԃ�PCLKB��125kHz(typ.))
	
	/* ����N���b�N�ݒ� (ExternalClock=12MHz) */
	SYSTEM.MOSCCR.BYTE = 0x01;			//�O�����U���~
	while(SYSTEM.MOSCCR.BYTE != 0x01);	//���������̂�҂�
	
	SYSTEM.MOSCWTCR.BYTE = 0x0D;		//�O�����U�q����҂����Ԑݒ� 22.288ms
	
	SYSTEM.MOSCCR.BYTE = 0x00;			//�O�����U�퓮��
	while(SYSTEM.MOSCCR.BYTE != 0x00);	//���������̂�҂�
	
	waitClockSet(52);					//�O�����U����҂�
	
	SYSTEM.PLLCR2.BYTE = 0x01;			//PLL��~
	SYSTEM.PLLCR.WORD = 0x0F00;			//fPLL=192MHz
	SYSTEM.PLLWTCR.BYTE = 0x0A;			//PLL���U����҂����� 1.68ms
	SYSTEM.PLLCR2.BYTE = 0x00;			//PLL�J�n
	
	waitClockSet(4);					//PLL���U����҂�
	
	SYSTEM.SCKCR.LONG = 0x21C22211ul;	//ICLK=fPLL/2, PCLKA=fPLL/4, PCLKB=fPLL/4, BLK=fPLL/4
	SYSTEM.SCKCR2.WORD = 0x0033;		//IECLK=fPLL/8, UCLK=fPLL/4
	SYSTEM.SCKCR3.WORD = 0x0400;		//����N���b�N��PLL�N���b�N��I��
	
	/* �����܂łŃN���b�N�ݒ芮���FICLK=96MHz, PCLKB=48MHz */
	

	/* 1ms���荞�ݗp */
	TMR0.TCCR.BYTE = 0x0D;	// PCLK/1024�ŃJ�E���g
	TMR0.TCR.BYTE = 0x48;	// �R���y�A�}�b�`A�ŃJ�E���^�N���A�ACMIEA����
	TMR0.TCORA = 47;		// 1.002667ms Cycle
	
	IEN(TMR0,CMIA0) = 1;	// TMR0��CMIA0���荞�ݗv������
	IPR(TMR0,CMIA0) = 3;	// TMR0��CMIA0���荞�݃��x���ݒ�

	/* TickTimer�p */
	TMR1.TCCR.BYTE = 0x0C;	// PCLK/64�ŃJ�E���g�A1�J�E���g=1.3333us
	
	/* �p���X�o�� */
	MSTP(MTU) = 0;			//�X�g�b�v����
	
	MTU3.TCR.BYTE = 0x22;	//b0-2:TPSC = 2		PCLK/16
							//b3-4:CKEG = 0		Up Edge count
							//b5-7:CCLR = 1		Clear TGRA compare match
	MTU3.TMDR.BYTE = 0x02;	//b0-3:MD = 2		PWM mode1
							//b4:BFA = 0
							//b5:BFB = 0
							//b6:BFE = 0
	MTU3.TIORH.BYTE = 0x12;	

	MTU3.TGRA = 1000;
	MTU3.TGRB = 100;
	
//	MTU.TSTR.BIT.CST3 = 1;


	MTU4.TCR.BYTE = 0x22;	//b0-2:TPSC = 2		PCLK/16
							//b3-4:CKEG = 0		Up Edge count
							//b5-7:CCLR = 1		Clear TGRA compare match
	MTU4.TMDR.BYTE = 0x02;	//b0-3:MD = 2		PWM mode1
							//b4:BFA = 0
							//b5:BFB = 0
							//b6:BFE = 0
	MTU4.TIORH.BYTE = 0x12;	

	MTU4.TGRA = 1000;
	MTU4.TGRB = 100;
	
	MTU.TOER.BIT.OE4A = 1;
	
//	MTU.TSTR.BIT.CST4 = 1;


	/* AD�ϊ� */
	MSTP(S12AD) = 0;			//�X�g�b�v����
	
	S12AD.ADCSR.BYTE = 0x0C;	// b0:EXTRG = 0
								// b1:TRGE = 0
								// b2,3:CKS = 11
								// b4:ADIE = 0
								// b6:ADCS = 0
								// b7:ADST = 0

	S12AD.ADCER.WORD = 0x0020;	// b5:ACE = 1
								// b15:ADRFMT = 0

	
	/* �V���A���ʐM */
	MSTP(SCI1) = 0;			//�X�g�b�v����

	SCI1.SCR.BYTE = 0x00;	/* �����N���b�N�A����M�֎~ */
	SCI1.SMR.BYTE = 0x00;	/* PCLK�N���b�N�ASTOP1bit�A�p���e�B�����A8Bit�f�[�^�A���������� */
	SCI1.BRR = 77;			/* 19200bps */
	SCI1.SSR.BYTE = 0xC0;
	
	/* ���M���� */
	SCI1.SCR.BYTE = 0x20;

	IEN(SCI1,RXI1) = 1;	/* SCI1��RXI1���荞�ݗv������ */
	IPR(SCI1,RXI1) = 2;	/* SCI1��RXI1���荞�݃��x���ݒ� */
	IR(SCI1,RXI1) = 0;
	
	
	/* �Ȉ�SPI�ʐM */
	MSTP(SCI5) = 0;			//�X�g�b�v����
	
	SCI5.SCR.BYTE = 0x00;	/* �����N���b�N�A����M�֎~ */
	SCI5.SMR.BYTE = 0x80;	/* PCLK�N���b�N�A�N���b�N������ */
	SCI5.SPMR.BYTE = 0x00;	/* SS�[�q���g�p�A�}�X�^�[���[�h�A�N���b�N���]�Ȃ��A�N���b�N�x��Ȃ� */
	SCI5.BRR = 23;			/* 500kbps */
	SCI5.SSR.BYTE = 0xC0;
	
	/* ����M���� */
	SCI5.SCR.BYTE = 0x30;
	
	
	/* PBx,PCx�ؑ֐ݒ� */
	PORT.PSRB.BYTE = 0x00u;
	
	/* �|�[�g������ */
	PORT1.PODR.BIT.B5 = 0;		//RIGHT MOTOR CW/CCW
	PORT2.PODR.BIT.B6 = 1;		//TXD1
	PORT2.PODR.BIT.B7 = 1;		//LED0
	PORT3.PODR.BIT.B0 = 1;		//RXD1
	PORT3.PODR.BIT.B1 = 1;		//LED1
	PORTA.PODR.BIT.B1 = 1;		//SSPI CLK
	PORTA.PODR.BIT.B3 = 0;		//SSPI DI
	PORTA.PODR.BIT.B4 = 1;		//SSPI DO
	PORTA.PODR.BIT.B6 = 1;		//SSPI CS
	PORTB.PODR.BIT.B0 = 1;		//LED2
	PORTB.PODR.BIT.B1 = 0;		//LEFT MOTOR CW/CCW
	PORTE.PODR.BIT.B1 = 1;		//RSPI CLK
	PORTE.PODR.BIT.B3 = 0;		//RSPI DI
	PORTE.PODR.BIT.B2 = 1;		//RSPI DO
	PORTC.PODR.BIT.B4 = 1;		//RSPI CS
	PORTC.PODR.BIT.B7 = 0;		//MOTOR SLEEP

	PORT1.PDR.BIT.B5 = 1;		//RIGHT MOTOR CW/CCW
	PORT2.PDR.BIT.B6 = 1;		//TXD1
	PORT2.PDR.BIT.B7 = 1;		//LED0
	PORT3.PDR.BIT.B0 = 0;		//RXD1
	PORT3.PDR.BIT.B1 = 1;		//LED1
	PORTA.PDR.BIT.B1 = 1;		//SSPI CLK
	PORTA.PDR.BIT.B3 = 0;		//SSPI DI
	PORTA.PDR.BIT.B4 = 1;		//SSPI DO
	PORTA.PDR.BIT.B6 = 1;		//SSPI CS
	PORTB.PDR.BIT.B0 = 1;		//LED2
	PORTB.PDR.BIT.B1 = 1;		//LEFT MOTOR CW/CCW
	PORTE.PDR.BIT.B1 = 1;		//RSPI CLK
	PORTE.PDR.BIT.B3 = 0;		//RSPI DI
	PORTE.PDR.BIT.B2 = 1;		//RSPI DO
	PORTC.PDR.BIT.B4 = 1;		//RSPI CS
	PORTC.PDR.BIT.B7 = 1;		//MOTOR SLEEP

	MPC.PWPR.BIT.B0WI = 0;		//PFSWE�������݋���
	MPC.PWPR.BIT.PFSWE = 1;		//PFS�������݋���
	MPC.P14PFS.BIT.PSEL = 1;	//RIGHT MOTOR PWM
	MPC.P26PFS.BIT.PSEL = 10;	//TXD1
	MPC.P30PFS.BIT.PSEL = 10;	//RDX1
	MPC.PA1PFS.BIT.PSEL = 10;	//SSPI CLK
	MPC.PA3PFS.BIT.PSEL = 10;	//SSPI DI
	MPC.PA4PFS.BIT.PSEL = 10;	//SSPI DO
	MPC.PB3PFS.BIT.PSEL = 2;	//LEFT MOTOR PWM
	MPC.PE1PFS.BIT.PSEL = 14;	//RSPI CLK
	MPC.PE2PFS.BIT.PSEL = 14;	//RSPI DO
	MPC.PE3PFS.BIT.PSEL = 13;	//RSPI DI
	MPC.PWPR.BIT.PFSWE = 0;		//PFS�������݋֎~

	PORT2.PMR.BIT.B6 = 1;		//TXD1
	PORT3.PMR.BIT.B0 = 1;		//RXD1
	PORT1.PMR.BIT.B4 = 1;		//RIGHT MOTOR PWM
	PORTA.PMR.BIT.B1 = 1;		//SSPI CLK
	PORTA.PMR.BIT.B3 = 1;		//SSPI DI
	PORTA.PMR.BIT.B4 = 1;		//SSPI DO
	PORTB.PMR.BIT.B3 = 1;		//LEFT MOTOR PWM
	PORTE.PMR.BIT.B1 = 1;		//RSPI CLK
	PORTE.PMR.BIT.B2 = 1;		//RSPI DO
	PORTE.PMR.BIT.B3 = 1;		//RSPI DI

}

static void waitClockSet(u8 count) {	/* 1count = 0.512ms(typ.)@PCLKB=125kHz */
	u8 l_start = TMR0.TCNT;
	u8 l_delta = 0;
	
	while(l_delta < count) {
		l_delta = (u8)(TMR0.TCNT - l_start);
	}
}

