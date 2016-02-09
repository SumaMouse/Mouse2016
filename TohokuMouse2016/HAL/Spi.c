#include	"iodefine.h"
#include	"common.h"

#include	"Spi.h"
#include	"Timer.h"


static u8 ExchangeMsbLsb(u8 input);


void SpiInit(void) {

}

u8 SpiSimpleTxRx(u8 txData, u8* rxData) {
	
	u8 isNoError = 1;
	u8 data = 0;
	u8 ssr = 0;
	
	if (SCI5.SSR.BIT.TEND == 1) {
		
		SCI5.TDR = ExchangeMsbLsb(txData);
		
		while(SCI5.SSR.BIT.TEND == 0);

		ssr = SCI5.SSR.BYTE;
		SCI5.SSR.BYTE = 0xC0;
		if ((ssr & 0x38) == 0) {
			data = ExchangeMsbLsb(SCI5.RDR);
		} else {
			isNoError = 0;
		}

	}
	
	(*rxData) = data;
	
	return isNoError;
}

void SpiSimpleSetCS(u8 level) {
	PORTA.PODR.BIT.B6 = level;
}

static u8 ExchangeMsbLsb(u8 input) {
	
	u8 data = 0;
	
	if ((input & 0x01) != 0x00) data |= 0x80;
	if ((input & 0x02) != 0x00) data |= 0x40;
	if ((input & 0x04) != 0x00) data |= 0x20;
	if ((input & 0x08) != 0x00) data |= 0x10;
	if ((input & 0x10) != 0x00) data |= 0x08;
	if ((input & 0x20) != 0x00) data |= 0x04;
	if ((input & 0x40) != 0x00) data |= 0x02;
	if ((input & 0x80) != 0x00) data |= 0x01;
	
	return data;
}

