#include "iodefine.h"
#include "common.h"

#include "Timer.h"


#define		MOTOR_DRIVE_FREQ_TO_REGSTER		(1000)


void TimerWait1ms(u32 time) {
	
	u32 count;
	
	for (count=0;count<time;count++) {
		TimerWait1_333us(150);
		TimerWait1_333us(150);
		TimerWait1_333us(150);
		TimerWait1_333us(150);
		TimerWait1_333us(150);
	}
	
}

void TimerWait1_333us(u8 count) {
	
	u8 l_start = TMR1.TCNT;
	u8 l_delta = 0;
	
	while(l_delta < count) {
		l_delta = (u8)(TMR1.TCNT - l_start);
	}
}

void StartMotorTimers(void) {
	
	MTU.TSTR.BYTE |= 0xC0u;
	
}

void StopMotorTimers(void) {
	
	MTU.TSTR.BYTE &= 0x3Fu;
	
}

void SetRightMotorDutyReg(u16 value) {
	MTU3.TGRB = value;
}

void SetLeftMotorDutyReg(u16 value) {
	MTU4.TGRB = value;
}

