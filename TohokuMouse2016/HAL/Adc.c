#include "iodefine.h"
#include "common.h"

#include "Adc.h"


void AdcInit(void) {
	
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
		case  0 : value = (u16)(S12AD.ADDR0); break;
		case  1 : value = (u16)(S12AD.ADDR1); break;
		case  2 : value = (u16)(S12AD.ADDR2); break;
		case  6 : value = (u16)(S12AD.ADDR6); break;
		case  9 : value = (u16)(S12AD.ADDR9); break;
		case 10 : value = (u16)(S12AD.ADDR10); break;
		case 11 : value = (u16)(S12AD.ADDR11); break;
		case 12 : value = (u16)(S12AD.ADDR12); break;

		default : break;
	}
	
	return value;
}
