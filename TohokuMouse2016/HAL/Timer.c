#include "iodefine.h"
#include "common.h"

#include "Timer.h"


static void Wait1_333us(u8 count);


void TimerWait1ms(u32 time) {
	
	u32 count;
	
	for (count=0;count<time;count++) {
		Wait1_333us(150);
		Wait1_333us(150);
		Wait1_333us(150);
		Wait1_333us(150);
		Wait1_333us(150);
	}
	
}

static void Wait1_333us(u8 count) {
	
	u8 l_start = TMR1.TCNT;
	u8 l_delta = 0;
	
	while(l_delta < count) {
		l_delta = (u8)(TMR1.TCNT - l_start);
	}
}

