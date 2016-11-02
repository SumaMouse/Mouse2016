#ifndef	__TIMER_H__
#define	__TIMER_H__


extern void TimerWait1ms(u32 time);
extern void TimerWait1_333us(u8 count);
extern u16 Get1usTimer(void);
extern void StartMotorTimers(void);
extern void StopMotorTimers(void);
extern void SetRightMotorDutyReg(u16 value);
extern void SetLeftMotorDutyReg(u16 value);


#endif

