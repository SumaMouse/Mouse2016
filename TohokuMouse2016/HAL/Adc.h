#ifndef	__ADC_H__
#define	__ADC_H__

#define	ADC12_1LSB_1MV		(0.68359375f)

#define	ADC_CH_WALL_SENS_FR		(6)
#define	ADC_CH_WALL_SENS_FL		(0)
#define	ADC_CH_WALL_SENS_RS		(2)
#define	ADC_CH_WALL_SENS_LS		(1)
#define	ADC_CH_BATTERY			(12)

extern	void AdcInit_v(void);
extern	u16 AdcRead(u8 channel);

#endif

