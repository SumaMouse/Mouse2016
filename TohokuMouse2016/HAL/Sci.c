#include	"iodefine.h"
#include	"Common.h"
#include	"Sci.h"

#if 0

#pragma interrupt (Excep_SCI1_ERI1(vect=218))
#pragma interrupt (Excep_SCI1_RXI1(vect=219))


#define		TX_BUFFER_SIZE				(100)
#define		RX_BUFFER_SIZE				(100)


static	u8	TxBuffer[TX_BUFFER_SIZE];
static	u8	RxBuffer[RX_BUFFER_SIZE];

static	u16	TxBufferWriteIndex;
static	u16	TxBufferReadIndex;

static	u16	RxBufferWriteIndex;
static	u16	RxBufferReadIndex;

static u8 RcvCommand = 0;

void	SciInit(void) {

	u16 i;
	
	for (i=0;i<TX_BUFFER_SIZE;i++) {
		TxBuffer[i] = 0;
	}

	for (i=0;i<RX_BUFFER_SIZE;i++) {
		RxBuffer[i] = 0;
	}

	TxBufferWriteIndex = 0;
	TxBufferReadIndex = 0;
	
	RxBufferWriteIndex = 0;
	RxBufferReadIndex = 0;
}

void	SciSendPeriodic(void) {

	if ((TxBufferWriteIndex != TxBufferReadIndex) && (SCI1.SSR.BIT.TDRE == 1)) {
		u8 data;

		data = TxBuffer[TxBufferReadIndex];
		TxBufferReadIndex++;

		if (TxBufferReadIndex >= TX_BUFFER_SIZE) {
			TxBufferReadIndex = 0;
		}

		SCI1.TDR = data;
	}
}

void	SciSendString(s8 *str) {
	while (*str != '\0'){
		SciSendByte(*str);
		str++;
	}
}

void	SciSendHex(u8 beam, u32 value) {
	
	u16	i;
	u16	cc;

	if (Beam > 8) Beam = 8;

	for(i=8-beam;i<8;i++){
		cc = (Value >> (28-4*i)) & 0x0000000FL;
		if(cc < 10)	SciSendByte(cc + '0');
		else		SciSendByte(cc + 'A' - 10);
	}
}

void	SciSendDec(u8 beam, s32 value) {

	if (value < 0) {
		SciSendByte('-');
		value = value * (-1);
	}
	
	switch (beam) {
		default :
		case 7 : SciSendByte('0'+ value / 1000000 % 10);
		case 6 : SciSendByte('0'+ value /  100000 % 10);
		case 5 : SciSendByte('0'+ value /   10000 % 10);
		case 4 : SciSendByte('0'+ value /    1000 % 10);
		case 3 : SciSendByte('0'+ value /     100 % 10);
		case 2 : SciSendByte('0'+ value /      10 % 10);
		case 1 : SciSendByte('0'+ value           % 10);
	}
}

void	SciSendByte(u8 data)
{
	u16 wp;

	wp = TxBufferWriteIndex + 1;
	if (wp >= TX_BUFFER_SIZE) {
		wp = 0;
	}

	if (wp != TxBufferReadIndex) {
		TxBuffer[TxBufferWriteIndex] = data;
		TxBufferWriteIndex = wp;
	}

}

u8		SciReceiveByte(void)
{
	u16 rp;
	u8 ans;

	if (RxBufferReadIndex != RxBufferWriteIndex) {
		ans = RxBuffer[RxBufferReadIndex];
		rp = RxBufferReadIndex + 1;
		if (rp >= RX_BUFFER_SIZE) {
			rp = 0;
		}
		RxBufferReadIndex = rp;

	} else {
		ans = 0xff;
	}

	return ans;
}

u8		SciGetRxDataSize(void) {
	
	u8	rxDataSize = 0;
	
	if (RxBufferWriteIndex < RxBufferReadIndex) {
		rxDataSize = RX_BUFFER_SIZE - RxBufferReadIndex + RxBufferWriteIndex;
	} else {
		rxDataSize = RxBufferWriteIndex - RxBufferReadIndex;
	}
	
	return(rxDataSize);
}

void	SciSetRxEnable(void) {
	SCI1.SCR.BIT.RIE = 1;
	SCI1.SCR.BIT.RE = 1;
}

void	SciSetRxDisable(void) {
	SCI1.SCR.BIT.RE = 0;
	SCI1.SCR.BIT.RIE = 0;
}

void	Excep_SCI1_RXI1(void)
{
	u16 wp;
	u8 data;
	
	data = SCI1.RDR;
	
	wp = RxBufferWriteIndex + 1;
	if (wp >= RX_BUFFER_SIZE) {
		wp = 0;
	}
	
	if (wp != RxBufferReadIndex) {
		RxBuffer[RxBufferWriteIndex] = data;
		RxBufferWriteIndex = wp;
	}

	RcvCommand = data;
	SciSendByte(RcvCommand);
}


void	Excep_SCI1_ERI1(void)
{
	u8	ErrData;

	ErrData = SCI1.SSR.BYTE;
	SCI1.SSR.BYTE &= ~0x38;			/* ステータス(受信,エラー)クリア */

#if 0
	//Parity Error
	if ((ErrData & 0x08) != 0) 	PORT_LED5 = 0;
	else						PORT_LED5 = 1;
	
	//Framing Error
	if ((ErrData & 0x10) != 0) 	PORT_LED6 = 0;
	else						PORT_LED6 = 1;

	//Overrun Error
	if ((ErrData & 0x20) != 0) 	PORT_LED7 = 0;
	else						PORT_LED7 = 1;
#endif

}

#endif
