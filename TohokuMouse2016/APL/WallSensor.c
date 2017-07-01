#include "iodefine.h"
#include "common.h"

#include "Adc.h"
#include "Gpio.h"
#include "Timer.h"

#include "WallSensor.h"


#define	WALL_SENS_START_DATA		(1000)


static s16	WallSensRs;
static s16	WallSensFr;
static s16	WallSensFl;
static s16	WallSensLs;

void WallSensorInit(void) {
	
	WallSensRs = 0;
	WallSensFr = 0;
	WallSensFl = 0;
	WallSensLs = 0;
}

s16 GetWallSensor(u8 pos) {

	s16 value = 0;
	
	switch (pos) {
		case SIDE_RIGHT :	value = WallSensRs; break;
		case FRONT_RIGHT :	value = WallSensFr; break;
		case FRONT_LEFT :	value = WallSensFl; break;
		case SIDE_LEFT :	value = WallSensLs; break;
		default:break;
	}
	
	return value;
}



void ReadWallSensors(void) {
	
	s16 offSens;
	
	offSens = AdcRead(ADC_CH_WALL_SENS_FR);
	GpioWrteWallSensFrOn(1);
	TimerWait1_333us(10);
	WallSensFr = (s16)AdcRead(ADC_CH_WALL_SENS_FR) - offSens;
	GpioWrteWallSensFrOn(0);

	offSens = AdcRead(ADC_CH_WALL_SENS_LS);
	GpioWrteWallSensLsOn(1);
	TimerWait1_333us(10);
	WallSensLs = (s16)AdcRead(ADC_CH_WALL_SENS_LS) - offSens;
	GpioWrteWallSensLsOn(0);

	offSens = AdcRead(ADC_CH_WALL_SENS_RS);
	GpioWrteWallSensRsOn(1);
	TimerWait1_333us(10);
	WallSensRs = (s16)AdcRead(ADC_CH_WALL_SENS_RS) - offSens;
	GpioWrteWallSensRsOn(0);

	offSens = AdcRead(ADC_CH_WALL_SENS_FL);
	GpioWrteWallSensFlOn(1);
	TimerWait1_333us(10);
	WallSensFl = (s16)AdcRead(ADC_CH_WALL_SENS_FL) - offSens;
	GpioWrteWallSensFlOn(0);

}

void WaitSensorHandStart(void) {
	
	/* àÍíUOFFÇåüèoÇµÇƒÇ©ÇÁ */
	while (1) {
		if (WallSensFr < WALL_SENS_START_DATA) {
			break;
		}
		
		GpioWrteLed2(TRUE);
		TimerWait1ms(100);
		GpioWrteLed2(FALSE);
		TimerWait1ms(200);

	}

	while (1) {
		if (WallSensFr > WALL_SENS_START_DATA) {
			break;
		}

		GpioWrteLed2(TRUE);
		TimerWait1ms(10);
		GpioWrteLed2(FALSE);
		TimerWait1ms(100);
	}

	TimerWait1ms(1000);
}


