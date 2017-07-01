#ifndef	__BATTERY_H__
#define	__BATTERY_H__

extern u8 ReadBatteryStatus(void);
extern u16 GetBatteryVoltage1mv(void);
extern u16 GetBattery12ad(void);
extern void LowBattery(void);

#endif

