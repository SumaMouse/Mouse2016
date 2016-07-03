#include	"iodefine.h"
#include	"common.h"

#include	"spi.h"
#include	"timer.h"

/* Clear PERF,MODF,OVRF */
static void ClearSpsr(void) {
	if ((RSPI1.SPSR.BYTE & 0x0D) != 0) {
		RSPI1.SPSR.BYTE = 0xA0;
		while((RSPI1.SPSR.BYTE & 0x0D) != 0);
	}	
}


void RspiInit(void) {

	volatile u8 Dummy;
	volatile u32 t;

	/* MTU���j�b�g���W���[���X�g�b�v���� */
	MSTP(RSPI1) = 0;

	RSPI1.SPCR.BYTE = 0x09;		/* bit7:0 ��M���荞�݋֎~ */
								/* bit6:0 RSPI�@�\���� */
								/* bit5:0 ���M���荞�݋֎~ */
								/* bit4:0 �G���[���荞�݋֎~ */
								/* bit3:1 �}�X�^���[�h */
								/* bit2:0 ���[�h�t�H�[���g�G���[���o�֎~ */
								/* bit1:0 �S2�d�������ʐM */
								/* bit0:1 �N���b�N������(3����) */

	RSPI1.SPPCR.BYTE = 0x30;	/* bit7-6:0 Reserve */
								/* bit5:1 MOSI�o�͒l��MOIFV�ɏ]�� */
								/* bit4:1 MOSI�A�C�h���Œ�l��Hi */
								/* bit3:0 Reserve */
								/* bit2:0 RSPI�o�͒[�q��CMOS�o�� */
								/* bit1:0 �ʏ탂�[�h(���[�v�o�b�N����) */
								/* bit0:0 �ʏ탂�[�h(���[�v�o�b�N����) */
								
	RSPI1.SPBR = 5;				/* 1Mbps */
	TimerWait1_333us(20);
	
	RSPI1.SPDCR.BYTE = 0x20;	/* bit7-6:0 Reserve */
								/* bit5:1 SPDR���W�X�^�ւ�LONG�A�N�Z�X */
								/* bit4:0 SPDR�͎�M�o�b�t�@��ǂ݂��� */
								/* bit3-2:0 SSL�S�[�q�o�� */
								/* bit1-0:1 �i�[�t���[����1 */

	RSPI1.SPCR2.BYTE = 0x00;	/* bit7-4:0 Reserve */
								/* bit3:0 �p���e�B���Ȑf�f���� */
								/* bit2:0 �A�C�h�����荞�݋֎~ */
								/* bit1:0 �����p���e�B */
								/* bit0:0 �p���e�B���� */

	ClearSpsr();

	RSPI1.SPCMD0.WORD = 0x0F09;	/* bit15:0 RSPCK 1RSPCK */
								/* bit14:0 NegationDelay 1RSPCK */
								/* bit13:0 Next Access Delay 1RSPCK */
								/* bit12:0 MSB First */
								/* bit11-8:15 Data size=16bit */
								/* bit7:0 */
								/* bit6-4:0 */
								/* bit3-2:3 4���� */
								/* bit1:0 */
								/* bit0:1 */

	RSPI1.SPCR.BYTE = 0xC9;		/* bit7:1 ��M���荞�݋��� */
								/* bit6:1 RSPI�@�\�L�� */
								/* bit5:0 ���M���荞�݋��� */
								/* bit4:0 �G���[���荞�݋֎~ */
								/* bit3:1 �}�X�^���[�h */
								/* bit2:0 ���[�h�t�H�[���g�G���[���o�֎~ */
								/* bit1:0 �S2�d�������ʐM */
								/* bit0:1 �N���b�N������(3����) */	
	
	Dummy = RSPI1.SPCR.BYTE;

}


u16 RspiTxRx(u16 txData, u16* rxData) {

	volatile u8 t;

	u16 err = 0;
	u32 data = 0;
	
	/* ���M�o�b�t�@��Ԋm�F */
	if (RSPI1.SPSR.BIT.IDLNF == 0) {

		/* ���M�f�[�^�ݒ� */
		RSPI1.SPDR.LONG = (u32)txData;
		
		
		/* �f�[�^�]�������҂�@1MHz */
		TimerWait1_333us(20);

		ClearSpsr();

		/* �f�[�^�擾 */
		data = RSPI1.SPDR.LONG;
	}

	/* ��M�f�[�^�R�s�[ */
	(*rxData) = (u16)data;
	
	return err;
}

void RspiSetCS(u16 level) {
	PORTC.PODR.BIT.B4 = level;
}

