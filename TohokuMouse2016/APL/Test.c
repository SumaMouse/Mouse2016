#include "iodefine.h"
#include "common.h"

#include "Sci.h"
#include "Adc.h"
#include "Gpio.h"
#include "Timer.h"
#include "Spi.h"
#include "AS5055.h"
#include "Battery.h"
#include "Gyro.h"

#include "Test.h"

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
		
		GpioWrteWallSensFlOn(1);
		TimerWait1ms(1);
		l_adc[0] = AdcRead(0);
		GpioWrteWallSensFlOn(0);
		
		
		GpioWrteWallSensLsOn(1);
		TimerWait1ms(1);
		l_adc[1] = AdcRead(1);
		GpioWrteWallSensLsOn(0);


		GpioWrteWallSensRsOn(1);
		TimerWait1ms(1);
		l_adc[2] = AdcRead(2);
		GpioWrteWallSensRsOn(0);
		
		
		GpioWrteWallSensFrOn(1);
		TimerWait1ms(1);
		l_adc[3] = AdcRead(6);
		GpioWrteWallSensFrOn(0);
		
		
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

void TestSpiSimple(void) {
	
	u8 txData = 0, rxData = 0, result = 0;
	
	SciSendString("SIMPLE SPI TEST!!\n\r");
	SciSendString("Result\tRxData\n\r");

	txData = 0x80u | 0x75u;		//WHO AM I
	
	while(1) {
		
		SpiSimpleSetCS(0);
		
		result = SpiSimpleTxRx(txData, &rxData);
		result = SpiSimpleTxRx(0xFFu, &rxData);

		SpiSimpleSetCS(1);
		

		SciSendHex(2, result);
		SciSendString("\t");
		SciSendHex(2, rxData);
		SciSendString("\t");

		SciSendString("\r");
		
		TimerWait1ms(1000);
	}
	
}


void TestRspi(void) {
	
	u16 txData = 0, rxData = 0, err = 0;
	u16 command = 0x3FFFu;
	u8 numOf1 = 0;
	u8 readChannel;
	
	SciSendString("RSPI TEST!!\n\r");
	SciSendString("Channel\tErr\tRxData\n\r");

	txData = ((command << 1) | 0x8000u);
	if ((CountBit1For16bitData(txData) % 2) != 0) {
		txData |= 1u;
	}
	
	while(1) {
		
		for (readChannel=0;readChannel<2;readChannel++) {
		
			RspiSetSS(readChannel, 0);
			err = RspiTxRx(txData, &rxData);
			RspiSetSS(readChannel, 1);

			TimerWait1_333us(1);

			RspiSetSS(readChannel, 0);
			err = RspiTxRx(0x0000u, &rxData);
			RspiSetSS(readChannel, 1);
			
			numOf1 = CountBit1For16bitData(rxData);
			if ((numOf1 % 2) == 0) {
				err = 0;
			} else {
				err = 1;
			}

			SciSendDec(2, readChannel);
			SciSendString("\t");
			SciSendHex(2, err);
			SciSendString("\t");
			SciSendHex(4, rxData);
			SciSendString("\t");
			SciSendDec(2, numOf1);
			SciSendString("\t");
			

			SciSendString("\n\r");
		}

		TimerWait1ms(1000);
		
	}
	
}


void TestMotor(void) {

	SciSendString("MOTOR TEST!!\n\r");

	SetRightMotorDutyReg(0);
	SetLeftMotorDutyReg(0);

	GpioWrteRightMotorCw(1);
	GpioWrteLeftMotorCw(1);

	GpioWrteMotorSleep(1);

	StartMotorTimers();

	TimerWait1ms(5000);
	
	while(1) {

		SetRightMotorDutyReg(100);
		SetLeftMotorDutyReg(100);
		TimerWait1ms(3000);

		SetRightMotorDutyReg(0);
		SetLeftMotorDutyReg(0);
		TimerWait1ms(3000);

		GpioWrteMotorSleep(0);
		
		while(1);

#if 0		
		SciSendString("MOTOR Right CW ON!\n\r");
		SetRightMotorDutyReg(100);
		TimerWait1ms(1000);

		SciSendString("MOTOR Right OFF!\n\r");
		SetRightMotorDutyReg(0);
		TimerWait1ms(1000);

		GpioWrteRightMotorCw(1);

		SciSendString("MOTOR Right CCW ON!\n\r");
		SetRightMotorDutyReg(100);
		TimerWait1ms(1000);

		SciSendString("MOTOR Right OFF!\n\r");
		SetRightMotorDutyReg(0);
		TimerWait1ms(1000);

		GpioWrteRightMotorCw(0);


		SciSendString("MOTOR Left CW ON!\n\r");
		SetLeftMotorDutyReg(100);
		TimerWait1ms(1000);
		
		SciSendString("MOTOR Left OFF!\n\r");
		SetLeftMotorDutyReg(0);
		TimerWait1ms(1000);

		GpioWrteLeftMotorCw(1);
		
		SciSendString("MOTOR Left CCW ON!\n\r");
		SetLeftMotorDutyReg(100);
		TimerWait1ms(1000);
		
		SciSendString("MOTOR Left OFF!\n\r");
		SetLeftMotorDutyReg(0);
		TimerWait1ms(1000);
		
		GpioWrteLeftMotorCw(0);
#endif


	}

}

void TestAS5055(void) {
	
	u16 errorStatus[2] = {0};
	u16 angular12bit[2] = {0};
	
	SciSendString("AS5055 TEST!!\n\r");
	SciSendString("Angluar\tDataErrorStatus\n\r");
	
	
	while(1) {
		
		AS5055ReadAngularData(0, &angular12bit[0]);
		AS5055ReadErrorStatus(0, &errorStatus[0]);
		
		SciSendHex(4,angular12bit[0]);
		SciSendString("\t");
		SciSendHex(4,errorStatus[0]);

		SciSendString("\t");

		AS5055ReadAngularData(1, &angular12bit[1]);
		AS5055ReadErrorStatus(1, &errorStatus[1]);
		
		SciSendHex(4,angular12bit[1]);
		SciSendString("\t");
		SciSendHex(4,errorStatus[1]);

		
		SciSendString("\n\r");
		
		TimerWait1ms(1000);
		
	}
	
	
}

void TestWallSensor(void) {
	
	u16 battery1v = 0;
	u16 battery1mv = 0;
	
	SciSendString(" WALL SENSOR TEST!!\n\r");
	SciSendString("  LS \t FL \t FR \t RS \t Battery \n\r");
	
	while(1) {
		
		SciSendString(" ");
		SciSendDec(4, (u16)GetWallSensor(SIDE_LEFT));
		SciSendString("\t");
		SciSendDec(4, (u16)GetWallSensor(FRONT_LEFT));
		SciSendString("\t");
		SciSendDec(4, (u16)GetWallSensor(FRONT_RIGHT));
		SciSendString("\t");
		SciSendDec(4, (u16)GetWallSensor(SIDE_RIGHT));
		SciSendString("\t ");

		battery1mv = GetBatteryVoltage1mv();
		battery1v = (battery1mv/1000);

		SciSendDec(1, battery1v);
		SciSendString(".");
		SciSendDec(3, (battery1mv%1000));
		SciSendString("V");

		SciSendString("\r");
		
		TimerWait1ms(500);
	}
	
}

void GyroSensorTest(void) {

	float gyro = 0.0f;
	float angle = 0.0f;
	float ref = 0.0f;

	SciSendString("Gyro Ref Check...\n\r");
	TimerWait1ms(2000);
	UpdateGyroRef();

	SciSendString("Gyro Sensor Test.\n\r");
	SciSendString("  REF  GYRO  DEG \n\r");

	while (1) {
		
		/* ジャイロセンサテスト */
		ref = GetGyroSensorRef();
		gyro = GetGyroSensorValue();
		angle = (GetGyroAngle()*180.0f)/PI;
		
		SciSendDec(5,(s32)ref);			SciSendString(",");
		SciSendDec(5,(s32)gyro*1000);	SciSendString(",");
		SciSendDec(5,(s32)angle);		SciSendString(" ");

		SciSendString("\r");
		
		TimerWait1ms(300);
		
	}

}

