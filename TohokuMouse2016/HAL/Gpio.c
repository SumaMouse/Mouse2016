#include	"iodefine.h"
#include	"common.h"

#include	"Gpio.h"


void GpioWrteLed0(bool isOn) {
	u8 level = 1;

	if (isOn == TRUE) {
		level = 0;
	}
	PORT2.PODR.BIT.B7 = level;
}

void GpioWrteLed1(bool isOn) {
	u8 level = 1;

	if (isOn == TRUE) {
		level = 0;
	}
	PORT3.PODR.BIT.B1 = level;
}


