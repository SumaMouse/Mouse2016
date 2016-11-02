#ifndef	__AS5055_H__
#define	__AS5055_H__

extern void AS5055Setup(void);
extern void AS5055SoftwareReset(u8 ch);
extern void AS5055MasterReset(u8 ch);
extern void AS5055ClearErrorFlag(u8 ch);
extern void AS5055ReadAngularData(u8 ch, u16* angluar12bit);
extern void AS5055ReadErrorStatus(u8 ch, u16* reg);
extern void AS5055ReadAgc(u8 ch, u16* reg);
extern void AS5055ReadSystemConfig1(u8 ch, u16* reg);


#endif

