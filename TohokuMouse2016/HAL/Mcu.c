#include	<machine.h>
#include	"iodefine.h"
#include	"Common.h"

#include	"Mcu.h"

static void waitClockSet(u8 count);

void	McuInit(void) {

	/* レジスタライトプロテクション設定 */
	SYSTEM.PRCR.WORD = 0xA503;			//レジスタライト許可
	
	
	/* サブクロック停止設定(48Pin版ではサブクロックが無いため設定必要) */
	SYSTEM.SOSCCR.BYTE = 0x00;
	while(SYSTEM.SOSCCR.BYTE != 0x00);	//書き換わるのを待つ
	RTC.RCR4.BYTE = 0x01;				//メインクロック選択
	RTC.RCR3.BYTE = 0x02;				//サブクロック停止
	while(RTC.RCR3.BYTE != 0x02);		//書き換わるのを待つ
	RTC.RCR2.BIT.START = 0;				//RTC停止
	while(RTC.RCR2.BIT.START != 0);		//書き換わるのを待つ


	/* クロック安定待ちカウント用タイマ初期化 */
    MSTP(TMR01) = 0;					//TMR01ストップ解除
	TMR0.TCCR.BYTE = 0x0C;				//カウントクロック=PCLKA/1024=1.95kHz	(初期状態でPCLKBは125kHz(typ.))
	
	/* 動作クロック設定 (ExternalClock=12MHz) */
	SYSTEM.MOSCCR.BYTE = 0x01;			//外部発振器停止
	while(SYSTEM.MOSCCR.BYTE != 0x01);	//書き換わるのを待つ
	
	SYSTEM.MOSCWTCR.BYTE = 0x0D;		//外部発振子安定待ち時間設定 22.288ms
	
	SYSTEM.MOSCCR.BYTE = 0x00;			//外部発振器動作
	while(SYSTEM.MOSCCR.BYTE != 0x00);	//書き換わるのを待つ
	
	waitClockSet(52);					//外部発振安定待ち
	
	SYSTEM.PLLCR2.BYTE = 0x01;			//PLL停止
	SYSTEM.PLLCR.WORD = 0x0F00;			//fPLL=192MHz
	SYSTEM.PLLWTCR.BYTE = 0x0A;			//PLL発振安定待ち時間 1.68ms
	SYSTEM.PLLCR2.BYTE = 0x00;			//PLL開始
	
	waitClockSet(4);					//PLL発振安定待ち
	
	SYSTEM.SCKCR.LONG = 0x21C22211ul;	//ICLK=fPLL/2, PCLKA=fPLL/4, PCLKB=fPLL/4, BLK=fPLL/4
	SYSTEM.SCKCR2.WORD = 0x0033;		//IECLK=fPLL/8, UCLK=fPLL/4
	SYSTEM.SCKCR3.WORD = 0x0400;		//動作クロックにPLLクロックを選択
	
	/* ここまででクロック設定完了：ICLK=96MHz, PCLKB=48MHz */
	

	/* 1ms割り込み用 */
	TMR0.TCCR.BYTE = 0x0D;	// PCLK/1024でカウント
	TMR0.TCR.BYTE = 0x48;	// コンペアマッチAでカウンタクリア、CMIEA許可
	TMR0.TCORA = 47;		// 1.002667ms Cycle
	
	IEN(TMR0,CMIA0) = 1;	// TMR0のCMIA0割り込み要求許可
	IPR(TMR0,CMIA0) = 3;	// TMR0のCMIA0割り込みレベル設定

	/* TickTimer用 */
	TMR1.TCCR.BYTE = 0x0C;	// PCLK/64でカウント、1カウント=1.3333us
	
	/* パルス出力 */
	MSTP(MTU) = 0;			//ストップ解除
	
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


	/* AD変換 */
	MSTP(S12AD) = 0;			//ストップ解除
	
	S12AD.ADCSR.BYTE = 0x0C;	// b0:EXTRG = 0
								// b1:TRGE = 0
								// b2,3:CKS = 11
								// b4:ADIE = 0
								// b6:ADCS = 0
								// b7:ADST = 0

	S12AD.ADCER.WORD = 0x0020;	// b5:ACE = 1
								// b15:ADRFMT = 0

	
	/* シリアル通信 */
	MSTP(SCI1) = 0;			//ストップ解除

	SCI1.SCR.BYTE = 0x00;	/* 内蔵クロック、送受信禁止 */
	SCI1.SMR.BYTE = 0x00;	/* PCLKクロック、STOP1bit、パリティ無し、8Bitデータ、調歩同期式 */
	SCI1.BRR = 77;			/* 19200bps */
	SCI1.SSR.BYTE = 0xC0;
	
	/* 送信許可 */
	SCI1.SCR.BYTE = 0x20;

	IEN(SCI1,RXI1) = 1;	/* SCI1のRXI1割り込み要求許可 */
	IPR(SCI1,RXI1) = 2;	/* SCI1のRXI1割り込みレベル設定 */
	IR(SCI1,RXI1) = 0;
	
	
	/* 簡易SPI通信 */
	MSTP(SCI5) = 0;			//ストップ解除
	
	SCI5.SCR.BYTE = 0x00;	/* 内蔵クロック、送受信禁止 */
	SCI5.SMR.BYTE = 0x80;	/* PCLKクロック、クロック同期式 */
	SCI5.SPMR.BYTE = 0x00;	/* SS端子未使用、マスターモード、クロック反転なし、クロック遅れなし */
	SCI5.BRR = 23;			/* 500kbps */
	SCI5.SSR.BYTE = 0xC0;
	
	/* 送受信許可 */
	SCI5.SCR.BYTE = 0x30;
	
	
	/* PBx,PCx切替設定 */
	PORT.PSRB.BYTE = 0x00u;
	
	/* ポート初期化 */
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

	MPC.PWPR.BIT.B0WI = 0;		//PFSWE書き込み許可
	MPC.PWPR.BIT.PFSWE = 1;		//PFS書き込み許可
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
	MPC.PWPR.BIT.PFSWE = 0;		//PFS書き込み禁止

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

