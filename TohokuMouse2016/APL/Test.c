#include "iodefine.h"
#include "common.h"

#include "Sci.h"
#include "Adc.h"
#include "Gpio.h"
#include "Timer.h"


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

void TestAdc(void) {
	
	u16	l_adc[8] = {0};
	u8 t;
	
	SciSendString("ADC TEST!!\n\r");
	SciSendString("CH0\tCH1\tCH2\tCH6\tCH9\tCH10\tCH11\tCH12\n\r");
	
	while(1) {
		
		l_adc[0] = AdcRead(0);
		l_adc[1] = AdcRead(1);
		l_adc[2] = AdcRead(2);
		l_adc[3] = AdcRead(6);
		l_adc[4] = AdcRead(9);
		l_adc[5] = AdcRead(10);
		l_adc[6] = AdcRead(11);
		l_adc[7] = AdcRead(12);
		
		for (t=0;t<8;t++) {
			SciSendDec(4, l_adc[t]);
			SciSendString("\t");
		}
		SciSendString("\r");
		
		TimerWait1ms(1000);
	}
	
}
