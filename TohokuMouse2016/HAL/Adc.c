#include	"iodefine.h"
#include	"common.h"

#include	"Adc.h"


void AdcInit(void) {
	
	MSTP(S12AD) = 0;			//ストップ解除
	
	S12AD.ADCSR.BYTE = 0x0C;	// b0:EXTRG = 0
								// b1:TRGE = 0
								// b2,3:CKS = 11
								// b4:ADIE = 0
								// b6:ADCS = 0
								// b7:ADST = 0

	S12AD.ADCER.WORD = 0x0020;	//b5:ACE = 1
								//b15:ADRFMT = 0
}

u16 AdcRead(u8 channel) {

	u16 value = 0;
	
	/* ADC Stop */
	S12AD.ADCSR.BIT.ADST = 0;
	S12AD.ADANS0.WORD = (u16)(1ul << channel);
	
	/* ADC Start */
	S12AD.ADCSR.BIT.ADST = 1;	
	
	while(S12AD.ADCSR.BIT.ADST);
	
	switch (channel) {
		case  0 : value = S12AD.ADDR0; break;
		case  1 : value = S12AD.ADDR1; break;
		case  2 : value = S12AD.ADDR2; break;
		case  6 : value = S12AD.ADDR6; break;
		case  9 : value = S12AD.ADDR9; break;
		case 10 : value = S12AD.ADDR10; break;
		case 11 : value = S12AD.ADDR11; break;
		case 12 : value = S12AD.ADDR12; break;

		default : break;
	}
	
	return value;
}
