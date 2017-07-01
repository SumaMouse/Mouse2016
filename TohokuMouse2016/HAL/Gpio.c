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
	
	u8 level = 1;

	if (isCw == TRUE) {
		level = 0;
	}
	PORTB.PODR.BIT.B1 = level;
	
}

void GpioWrteWallSensFrOn(bool isOn) {
	
	u8 level = 0;

	if (isOn == TRUE) {
		level = 1;
	}
	PORTC.PODR.BIT.B6 = level;
}

void GpioWrteWallSensFlOn(bool isOn) {
	
	u8 level = 0;

	if (isOn == TRUE) {
		level = 1;
	}
	PORT1.PODR.BIT.B6 = level;
}

void GpioWrteWallSensRsOn(bool isOn) {
	
	u8 level = 0;

	if (isOn == TRUE) {
		level = 1;
	}
	PORTC.PODR.BIT.B5 = level;
}

void GpioWrteWallSensLsOn(bool isOn) {
	
	u8 level = 0;

	if (isOn == TRUE) {
		level = 1;
	}
	PORT1.PODR.BIT.B7 = level;
}


u8 GpioIsPushSwitchOn(void) {
	u8 isOn = 0;
	
	if (PORT3.PIDR.BIT.B5 == 0) {
		isOn = 1;
	}
	
	return isOn;
}



