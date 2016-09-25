#include	"iodefine.h"
#include	"common.h"

#include	"Gpio.h"


void GpioWrteLed0(bool isOn) {
	u8 level = 0;

	if (isOn == TRUE) {
		level = 1;
	}
	PORT2.PODR.BIT.B7 = level;
}

void GpioWrteLed1(bool isOn) {
	u8 level = 0;

	if (isOn == TRUE) {
		level = 1;
	}
	PORT3.PODR.BIT.B1 = level;
}

void GpioWrteLed2(bool isOn) {
	u8 level = 0;

	if (isOn == TRUE) {
		level = 1;
	}
	PORTB.PODR.BIT.B0 = level;
}

void GpioWrteMotorSleep(bool isOn) {
	u8 level = 0;

	if (isOn == TRUE) {
		level = 1;
	}
	PORTC.PODR.BIT.B7 = level;
}

void GpioWrteRightMotorCw(bool isCw) {
	
	u8 level = 0;

	if (isCw == TRUE) {
		level = 1;
	}
	PORT1.PODR.BIT.B5 = level;
	
}

void GpioWrteLeftMotorCw(bool isCw) {
	
	u8 level = 0;

	if (isCw == TRUE) {
		level = 1;
	}
	PORTB.PODR.BIT.B1 = level;
	
}

