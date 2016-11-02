#include "Common.h"

#include "Mcu.h"
#include "Sci.h"
#include "Gpio.h"
#include "Adc.h"
#include "Spi.h"
#include "Rspi.h"
#include "Test.h"

#include "AS5055.h"
#include "WallSensor.h"
#include "Battery.h"
#include "PushSwitch.h"
#include "Gyro.h"


volatile static u16 startTime = 0;
volatile static u16 execTime = 0;

volatile u16 execTimeMax = 0;

volatile static u32 timer1ms = 0;

static u8 isLowBattery = 0;

void AplMainInit(void) {
	
	

	McuInit();
	SciInit();
	AdcInit();
	SpiInit();
	RspiInit();
	
	TestInit();
	
	AS5055Setup();
	MPU6500Setup();
	WallSensorInit();
	
}



void AplMain(void) {

	GpioWrteLed0(TRUE);
	GpioWrteLed1(TRUE);
	GpioWrteLed2(TRUE);

	while(1) {
		
//		TestLed();
//		TestSci();
//		TestAdc();
//		TestSpiSimple();
		TestRspi();
//		TestMotor();

//		TestAS5055();
//		TestWallSensor();

//		GyroSensorTest();
		
	}
}


void Apl1msTask(void) {

	static u8 t = 0;
	
	startTime = Get1usTimer();
	
	ReadGyroSensor();
	ReadWallSensors();
	
	
	SciSendPeriodic();

	isLowBattery = ReadBatteryStatus();
	
	ReadPushSwitch();
	
	
	timer1ms++;
	if (timer1ms >= 1000) {

		GpioWrteLed0(t);
		GpioWrteLed2(t);
		t ^= 1;
		GpioWrteLed1(t);

		timer1ms = 0;
	}
	
	execTime = (Get1usTimer() - startTime) & 0xFFFF;
	
	if (execTimeMax <= execTime) {
		execTimeMax = execTime;
	}
}


