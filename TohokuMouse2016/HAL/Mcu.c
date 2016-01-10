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
	TMR0.TCCR.BYTE = 0x0D;	/* PCLK/1024�ŃJ�E���g */
	TMR0.TCR.BYTE = 0x48;	/* �R���y�A�}�b�`A�ŃJ�E���^�N���A�ACMIEA���� */
	TMR0.TCORA = 47;		/* 1.002667ms Cycle */
	
	IEN(TMR0,CMIA0) = 1;	/* TMR0��CMIA0���荞�ݗv������ */
	IPR(TMR0,CMIA0) = 2;	/* TMR0��CMIA0���荞�݃��x���ݒ� */
	
	
}

static void waitClockSet(u8 count) {	/* 1count = 0.512ms(typ.) */
	u8 l_start = TMR0.TCNT;
	u8 l_delta = 0;
	
	while(l_delta < count) {
		l_delta = (u8)(TMR0.TCNT - l_start);
	}
}
