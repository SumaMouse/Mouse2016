#include "iodefine.h"
#include "Common.h"

#include "Spi.h"
#include "Gpio.h"
#include "Timer.h"

#include "Gyro.h"


#define		COUNT_OF_GYRO_REF_CHECK		(10000)

#define		GYRO_SENSITIVITY_RAD	(0.001065f)		// rad/LSB @2000dpss


static float	gyroRef = 0.0f;
static float	gyroOutput = 0.0f;		// rad/sec
static float	gyroAngle = 0.0f;		// rad

static u8	gyroRefCheckRunning = 0;

static s16 ChangeU16ToS16(u16 val);
static s16 ReadSensorValue(u8 mode);

void MPU6500Setup(void) {

	u8 dummy;

	/* MPU6500 Initialize for SPI mode */
	SpiSimpleSetCS(0);
	TimerWait1_333us(1);
	(void)SpiSimpleTxRx(0x6B, &dummy);	//PWR_MGMT_1 write
	(void)SpiSimpleTxRx(0x80, &dummy);	//DEVICE_RESET
	TimerWait1_333us(1);
	SpiSimpleSetCS(1);
	
	TimerWait1ms(100);

	SpiSimpleSetCS(0);
	TimerWait1_333us(1);
	(void)SpiSimpleTxRx(0x68, &dummy);	//SIGNAL_PATH_RESET write
	(void)SpiSimpleTxRx(0x07, &dummy);	//GYRO_RST,ACCEL_RST,TEMP_RST
	TimerWait1_333us(1);
	SpiSimpleSetCS(1);

	TimerWait1ms(100);

	SpiSimpleSetCS(0);
	TimerWait1_333us(1);
	(void)SpiSimpleTxRx(0x6A, &dummy);	//USER_CTRL write
	(void)SpiSimpleTxRx(0x10, &dummy);	//I2C_IF_DIS
	TimerWait1_333us(1);
	SpiSimpleSetCS(1);
	
	TimerWait1ms(1);

	SpiSimpleSetCS(0);
	TimerWait1_333us(1);
	(void)SpiSimpleTxRx(0x1B, &dummy);	//GYRO_CONFIG write
	(void)SpiSimpleTxRx(0x18, &dummy);	//2000dps - 0x18, 1000dps - 0x10, 500dps - 0x08, 250dps - 0x00
	TimerWait1_333us(1);
	SpiSimpleSetCS(1);

	TimerWait1ms(1);

}
float GetGyroSensorRef(void) {
	return gyroRef;
}

float GetGyroSensorValue(void) {
	return gyroOutput;
}

float GetGyroAngle(void) {
	return gyroAngle;
}

void UpdateGyroRef(void) {
	
	u16 count;
	float buffer = 0.0f;
	
	gyroRefCheckRunning = 1;
	
	for (count=0;count<COUNT_OF_GYRO_REF_CHECK;count++) {
		buffer += (float)ReadSensorValue(GYRO_OUTPUT);
	}
	gyroRef = (buffer / COUNT_OF_GYRO_REF_CHECK);

	gyroAngle = 0.0f;
	
	gyroRefCheckRunning = 0;
}

void ReadGyroSensor(void) {
	
	if (gyroRefCheckRunning == 0) {
		gyroOutput = ((float)ReadSensorValue(GYRO_OUTPUT) - gyroRef) * GYRO_SENSITIVITY_RAD;
		gyroAngle += (gyroOutput * CONTROL_CYCLE_SEC);
	}
	
}

static s16 ChangeU16ToS16(u16 val) {
	
	s16 output;
	
	if ((val & 0x8000u) == 0) {
		output = val;
	} else {
		output = (s16)(((val ^ 0xFFFFu) + 1) * (-1));
	}
	
	return output;
}


static s16 ReadSensorValue(u8 mode) {
	
	u8 dummy;
	u8 command;
	u8 lsb,msb;
	u16 output;
	
	switch (mode) {
		default : 
		case GYRO_OUTPUT : command = 0x47; break;
		case GYRO_OFFSET : command = 0x17; break;
	}
	
	SpiSimpleSetCS(0);
	TimerWait1_333us(10);

	(void)SpiSimpleTxRx((command | 0x80), &dummy);
	(void)SpiSimpleTxRx(0xFF, &msb);

	TimerWait1_333us(10);
	SpiSimpleSetCS(1);

	TimerWait1_333us(10);
	
	SpiSimpleSetCS(0);
	TimerWait1_333us(10);

	(void)SpiSimpleTxRx(((command+1) | 0x80), &dummy);
	(void)SpiSimpleTxRx(0xFF, &lsb);
	
	TimerWait1_333us(10);
	SpiSimpleSetCS(1);

	output = ((u16)msb << 8) + (u16)lsb;
	
	return (ChangeU16ToS16(output));
}

