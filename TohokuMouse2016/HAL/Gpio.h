#ifndef	__GPIO_H__
#define	__GPIO_H__

extern void GpioWrteLed0(bool isOn);
extern void GpioWrteLed1(bool isOn);
extern void GpioWrteLed2(bool isOn);
extern void GpioWrteMotorSleep(bool isOn);
extern void GpioWrteWallSensFrOn(bool isOn);
extern void GpioWrteWallSensFlOn(bool isOn);
extern void GpioWrteWallSensRsOn(bool isOn);
extern void GpioWrteWallSensLsOn(bool isOn);

extern u8 GpioIsPushSwitchOn(void);
	
#endif

