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
	TMR0.TCCR.BYTE = 0x0D;	/* PCLK/1024でカウント */
	TMR0.TCR.BYTE = 0x48;	/* コンペアマッチAでカウンタクリア、CMIEA許可 */
	TMR0.TCORA = 47;		/* 1.002667ms Cycle */
	
	IEN(TMR0,CMIA0) = 1;	/* TMR0のCMIA0割り込み要求許可 */
	IPR(TMR0,CMIA0) = 2;	/* TMR0のCMIA0割り込みレベル設定 */
	
	
}

static void waitClockSet(u8 count) {	/* 1count = 0.512ms(typ.) */
	u8 l_start = TMR0.TCNT;
	u8 l_delta = 0;
	
	while(l_delta < count) {
		l_delta = (u8)(TMR0.TCNT - l_start);
	}
}
