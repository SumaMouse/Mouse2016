#include "common.h"

u8 CountBit1For16bitData(u16 data) {
	
	u8 numOfBit1 = 0;
	u8 bitPos;
	
	for (bitPos=0;bitPos<16;bitPos++) {
		if ((data & (1u << bitPos)) != 0) {
			numOfBit1++;
		}
	}
	
	return numOfBit1;
}

