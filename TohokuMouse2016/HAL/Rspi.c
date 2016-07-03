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

	/* MTUユニットモジュールストップ解除 */
	MSTP(RSPI1) = 0;

	RSPI1.SPCR.BYTE = 0x09;		/* bit7:0 受信割り込み禁止 */
								/* bit6:0 RSPI機能無効 */
								/* bit5:0 送信割り込み禁止 */
								/* bit4:0 エラー割り込み禁止 */
								/* bit3:1 マスタモード */
								/* bit2:0 モードフォールトエラー検出禁止 */
								/* bit1:0 全2重同期式通信 */
								/* bit0:1 クロック同期式(3線式) */

	RSPI1.SPPCR.BYTE = 0x30;	/* bit7-6:0 Reserve */
								/* bit5:1 MOSI出力値はMOIFVに従う */
								/* bit4:1 MOSIアイドル固定値はHi */
								/* bit3:0 Reserve */
								/* bit2:0 RSPI出力端子はCMOS出力 */
								/* bit1:0 通常モード(ループバック無し) */
								/* bit0:0 通常モード(ループバック無し) */
								
	RSPI1.SPBR = 5;				/* 1Mbps */
	TimerWait1_333us(20);
	
	RSPI1.SPDCR.BYTE = 0x20;	/* bit7-6:0 Reserve */
								/* bit5:1 SPDRレジスタへはLONGアクセス */
								/* bit4:0 SPDRは受信バッファを読みだす */
								/* bit3-2:0 SSL全端子出力 */
								/* bit1-0:1 格納フレーム数1 */

	RSPI1.SPCR2.BYTE = 0x00;	/* bit7-4:0 Reserve */
								/* bit3:0 パリティ自己診断無効 */
								/* bit2:0 アイドル割り込み禁止 */
								/* bit1:0 偶数パリティ */
								/* bit0:0 パリティ無し */

	ClearSpsr();

	RSPI1.SPCMD0.WORD = 0x0F09;	/* bit15:0 RSPCK 1RSPCK */
								/* bit14:0 NegationDelay 1RSPCK */
								/* bit13:0 Next Access Delay 1RSPCK */
								/* bit12:0 MSB First */
								/* bit11-8:15 Data size=16bit */
								/* bit7:0 */
								/* bit6-4:0 */
								/* bit3-2:3 4分周 */
								/* bit1:0 */
								/* bit0:1 */

	RSPI1.SPCR.BYTE = 0xC9;		/* bit7:1 受信割り込み許可 */
								/* bit6:1 RSPI機能有効 */
								/* bit5:0 送信割り込み許可 */
								/* bit4:0 エラー割り込み禁止 */
								/* bit3:1 マスタモード */
								/* bit2:0 モードフォールトエラー検出禁止 */
								/* bit1:0 全2重同期式通信 */
								/* bit0:1 クロック同期式(3線式) */	
	
	Dummy = RSPI1.SPCR.BYTE;

}


u16 RspiTxRx(u16 txData, u16* rxData) {

	volatile u8 t;

	u16 err = 0;
	u32 data = 0;
	
	/* 送信バッファ状態確認 */
	if (RSPI1.SPSR.BIT.IDLNF == 0) {

		/* 送信データ設定 */
		RSPI1.SPDR.LONG = (u32)txData;
		
		
		/* データ転送完了待ち@1MHz */
		TimerWait1_333us(20);

		ClearSpsr();

		/* データ取得 */
		data = RSPI1.SPDR.LONG;
	}

	/* 受信データコピー */
	(*rxData) = (u16)data;
	
	return err;
}

void RspiSetCS(u16 level) {
	PORTC.PODR.BIT.B4 = level;
}

