#include	"iodefine.h"
#include	"common.h"

#include	"Sci.h"
#include	"Adc.h"
#include	"Gpio.h"


void TestInit(void) {
	
	
	
}

void TestLed(void) {
	u32 timer;

	GpioWrteLed0(TRUE);
	for(timer=0;timer<0xFFFFu;timer++);
	GpioWrteLed0(FALSE);
	for(timer=0;timer<0xFFFFu;timer++);
	
}

void TestSci(void) {
	
	u8 rcvDataSize = 0;
	u8 rcvData = 0;
	u8 t = 0;
	
	SciSetRxEnable();
	
	SciSendString("MicroMouse2016!!\n\r");
	SciSendHex(4,0xABCD);
	SciSendString("\t");
	SciSendDec(5,65535);
	
	while(1) {
		
		
		/* 受信したデータをそのまま送信 */
		
		rcvDataSize = SciGetRxDataSize();
		
		if(rcvDataSize != 0) {
			
			t ^= 1;
			GpioWrteLed0(t);
			
			rcvData = SciReceiveByte();
			SciSendByte(rcvData);
		}
		
	}
}

