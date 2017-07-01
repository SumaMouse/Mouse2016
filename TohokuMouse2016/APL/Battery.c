#include	"iodefine.h"
#include	"Common.h"

#include	"Adc.h"

#include	"Battery.h"


#define	BATTERY_LOW_VOLTAGE_1MV		(3400)

static	u16 battery1mv;
static	u8	isFirst = 1;
static	u16	batteryBuffer[10] = {0};
static	u16	lowBatteryDetectTimer1ms = 0;
static	u16 battery12ad;


u8 ReadBatteryStatus(void) {
	
	u32 ave;
	u8 isLowVoltage = 0;
	
	battery12ad = AdcRead(ADC_CH_BATTERY);

	if (isFirst == 1) {
		isFirst = 0;
		batteryBuffer[0] = battery12ad;
		batteryBuffer[1] = battery12ad;
		batteryBuffer[2] = battery12ad;
		batteryBuffer[3] = battery12ad;
		batteryBuffer[4] = battery12ad;
		batteryBuffer[5] = battery12ad;
		batteryBuffer[6] = battery12ad;
		batteryBuffer[7] = battery12ad;
		batteryBuffer[8] = battery12ad;
		batteryBuffer[9] = battery12ad;
	}
	batteryBuffer[9] = batteryBuffer[8];
	batteryBuffer[8] = batteryBuffer[7];
	batteryBuffer[7] = batteryBuffer[6];
	batteryBuffer[6] = batteryBuffer[5];
	batteryBuffer[5] = batteryBuffer[4];
	batteryBuffer[4] = batteryBuffer[3];
	batteryBuffer[3] = batteryBuffer[2];
	batteryBuffer[2] = batteryBuffer[1];
	batteryBuffer[1] = batteryBuffer[0];
	batteryBuffer[0] = battery12ad;
	
	ave = (batteryBuffer[0] + batteryBuffer[1] + batteryBuffer[2] + batteryBuffer[3] + batteryBuffer[4] + 
		   batteryBuffer[5] + batteryBuffer[6] + batteryBuffer[7] + batteryBuffer[8] + batteryBuffer[9]) / 10ul;

	battery1mv = (u16)(((float)ave * ADC12_1LSB_1MV) * 2.0f);

	if (battery1mv < BATTERY_LOW_VOLTAGE_1MV) {
		lowBatteryDetectTimer1ms++;
	} else {
		lowBatteryDetectTimer1ms = 0;
	}
	
	if (lowBatteryDetectTimer1ms > 1000u) {
		isLowVoltage = 1;
		
		LowBattery();
	}
	
	return isLowVoltage;
}

u16 GetBatteryVoltage1mv(void) {
	return battery1mv;
}

u16 GetBattery12ad(void) {
	return battery12ad;
}

void LowBattery(void) {
	
	u32 count;
	
	GpioWrteLed0(FALSE);
	GpioWrteLed1(FALSE);
	GpioWrteLed2(FALSE);
	
	while(1) {
		
		GpioWrteLed0(TRUE);
		GpioWrteLed2(FALSE);
		for(count=0;count<0xFFFFFF;count++);

		GpioWrteLed0(FALSE);
		GpioWrteLed2(TRUE);
		for(count=0;count<0xFFFFFF;count++);
	}
	
}

