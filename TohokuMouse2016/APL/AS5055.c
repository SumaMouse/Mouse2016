#include "iodefine.h"
#include "common.h"

#include "Gpio.h"
#include "Timer.h"
#include "Rspi.h"
#include "Utility.h"

#include "AS5055.h"


#define	REG_READ_MASK			(0x8000u)
#define	REG_WRITE_MASK			(0x0000u)

#define REG_ADDR_POR_OFF		(0x3F22u)
#define REG_ADDR_SOFTWARE_RESET	(0x3C00u)
#define	REG_ADDR_MASTER_RESET	(0x33A5u)
#define	REG_ADDR_CLEAR_EF		(0x3380u)
#define	REG_ADDR_NOP			(0x0000u)
#define	REG_ADDR_AGC			(0x3FF8u)
#define	REG_ADDR_ANGULAR_DATA	(0x3FFFu)
#define	REG_ADDR_ERROR_STATUS	(0x335Au)
#define	REG_ADDR_SYSTEM_CONFIG	(0x3F20u)


static void WriteCommand(u8 ch, u16 regAddr, u16 data);
static u8 ReadCommand(u8 ch, u16 regAddr, u16* rxData);
static void txRxData(u8 ch, u16 txData, u16* rxData);


static u16 readRawValue = 0;

void AS5055Setup(void) {
	
	RspiSetSS(0, 1);
	RspiSetSS(1, 1);
	
	AS5055MasterReset(0);
	AS5055MasterReset(1);
	
}


void AS5055SoftwareReset(u8 ch) {
	WriteCommand(ch, REG_ADDR_SOFTWARE_RESET, 0x0002u);
}

void AS5055MasterReset(u8 ch) {
	WriteCommand(ch, REG_ADDR_MASTER_RESET, 0x0000u);
}

void AS5055ClearErrorFlag(u8 ch) {
	u16 result;
	u8 isOk;
	
	isOk = ReadCommand(ch, REG_ADDR_CLEAR_EF, &result);
	
//	if (result == 0) {
//		/* Clear OK */
//	}

}

void AS5055ReadAngularData(u8 ch, u16* angluar12bit) {

	u16 rxData = 0;
	u8 isOk;
	
	isOk = ReadCommand(ch, REG_ADDR_ANGULAR_DATA, &rxData);
	
	readRawValue = rxData;
	
	(*angluar12bit) = (rxData >> 2) & 0x0FFFu;
}

void AS5055ReadErrorStatus(u8 ch, u16* reg) {
	u16 rxData = 0;
	u8 isOk;
	
	isOk = ReadCommand(ch, REG_ADDR_ERROR_STATUS, &rxData);
	
	(*reg) = rxData;
	
}

void AS5055ReadSystemConfig1(u8 ch, u16* reg) {
	u16 rxData = 0;
	u8 isOk;
	
	isOk = ReadCommand(ch, REG_ADDR_SYSTEM_CONFIG, &rxData);
	
	(*reg) = rxData;
	
}

void AS5055ReadAgc(u8 ch, u16* reg) {
	u16 rxData = 0;
	u8 isOk;
	
	isOk = ReadCommand(ch, REG_ADDR_AGC, &rxData);
	
	(*reg) = rxData;
	
}

static void WriteCommand(u8 ch, u16 regAddr, u16 data) {
	
	
	u16 txData;
	u16 rxDummy;
	
	/* Command */
	txData = (REG_WRITE_MASK | (regAddr << 1));
	
	if ((CountBit1For16bitData(txData) % 2) != 0) {
		txData |= 1u;
	}
	
	txRxData(ch, txData, &rxDummy);
	

	/* Data */
	if (data != 0x0000u) {
		txData = (data << 2);
		
		if ((CountBit1For16bitData(txData) % 2) != 0) {
			txData |= 1u;
		}
		
		txRxData(ch, txData, &rxDummy);
	}
	
}

static u8 ReadCommand(u8 ch, u16 regAddr, u16* rxData) {
	
	u8	isOk = 0;
	u16 txData;
	u16 rxDataBuffer = 0;
	u16 rxDummy;
	
	/* Command */
	txData = (REG_READ_MASK | (regAddr << 1));
	
	if ((CountBit1For16bitData(txData) % 2) != 0) {
		txData |= 1u;
	}
	
	txRxData(ch, txData, &rxDummy);
	

	/* Data */
	txData = 0x0000u;
	
	txRxData(ch, txData, &rxDataBuffer);
	
	if ((CountBit1For16bitData(rxDataBuffer) % 2) != 0) {
		isOk = 1;
	}
	
	(*rxData) = rxDataBuffer;
	
	return isOk;
}

static void txRxData(u8 ch, u16 txData, u16* rxData) {
	
	RspiSetSS(ch, 0);

	TimerWait1_333us(10);

	(void)RspiTxRx(txData, rxData);

	RspiSetSS(ch, 1);
	
	TimerWait1_333us(10);

}

