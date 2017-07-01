#include "Common.h"

#include "Mcu.h"
#include "Sci.h"
#include "Gpio.h"
#include "Adc.h"
#include "Spi.h"
#include "Rspi.h"
#include "Test.h"
#include "Timer.h"

#include "AS5055.h"
#include "WallSensor.h"
#include "Battery.h"
#include "PushSwitch.h"
#include "Gyro.h"
#include "RotaryEncoder.h"
#include "Motor.h"


static volatile u16 startTime = 0;
static volatile u16 execTime = 0;

volatile u16 execTimeMax = 0;

static volatile u32 timer1ms = 0;

static u8 isLowBattery = 0;

void AplMainInit(void) {
	
	

	McuInit();

	GpioWrteLed0(TRUE);


	SciInit();
	AdcInit();
	SpiInit();
	RspiInit();
	
	TestInit();
	
	AS5055Setup();
	MPU6500Setup();
	WallSensorInit();

	InitializeMotorControl();

}



void AplMain(void) {

	while(1) {
		
//		TestLed();
//		TestSci();
//		TestAdc();
//		TestSpiSimple();
//		TestRspi();
//		TestMotor();

//		TestAS5055();
//		TestWallSensor();

//		GyroSensorTest();
				
//		TestDrive(0);
//		TestDrive(1);
		TestDrive(255);

		
	}
}


void Apl1msTask(void) {

	static u8 t = 0;
	
	startTime = Get1usTimer();
	
	ReadGyroSensor();
	ReadRotaryEncoder();
	ReadWallSensors();
	MotorControlCyclicTask();
	
	SciSendPeriodic();

	isLowBattery = ReadBatteryStatus();
	
	ReadPushSwitch();
	
	if (isLowBattery == 1) {
		
		InitializeMotorControl();

		GpioWrteMotorSleep(0);
		
		LowBattery();
	}
	
	timer1ms++;
	if (timer1ms >= 1000) {

		t ^= 1;
		GpioWrteLed0(t);

		timer1ms = 0;
	}
	
	execTime = (Get1usTimer() - startTime) & 0xFFFF;
	
	if (execTimeMax <= execTime) {
		execTimeMax = execTime;
	}
}


