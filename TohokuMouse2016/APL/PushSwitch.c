#include "iodefine.h"
#include "Common.h"

#include "Gpio.h"

#include "PushSwitch.h"


static u8 IsPushSwOn		= FALSE;
static u8 IsPushSwOnEvent	= FALSE;


void ReadPushSwitch(void) {
	
	static u8 OldSw[2] = {0};
	
	u8 isOn;
	
	isOn = GpioIsPushSwitchOn();
	
	if ((isOn == OldSw[0]) && (OldSw[0] == OldSw[1])) {
		IsPushSwOn = isOn;
	}	

	OldSw[1] = OldSw[0];
	OldSw[0] = isOn;
}

u8 IsSwitchOn(void) {
	return IsPushSwOn;
}

u8 MakeSwitchOnEvent(void) {
	
	static u8 OldSw = FALSE;
	
	/* Event */
	if ((IsPushSwOn == TRUE) && (OldSw == FALSE)) {
		IsPushSwOnEvent = TRUE;
	} else {
		IsPushSwOnEvent = FALSE;
	}
	OldSw = IsPushSwOn;
	
	return IsPushSwOnEvent;
}

