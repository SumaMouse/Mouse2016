#include "common.h"

#include "MousePrameter.h"
#include "AS5055.h"
#include "Timer.h"
#include "Battery.h"
#include "Gyro.h"
#include "WallSensor.h"
#include "Sci.h"
#include "Gpio.h"
#include "RotaryEncoder.h"

#include "motor.h"


#define		SPEED_DOWN_ACC		(-1.0f)


extern u16 dbgREncPos;
extern u16 dbgLEncPos;


typedef struct {
	float	targetAngle;	/* 目標角度[deg] */
	float	targetAngleAcc;	/* 目標角加速度[deg/s2] */
	float	entryLen;		/* ターン前の前進距離[mm] */
	float	escapeLen;		/* ターン後の前進距離[mm] */
	float	maxG;			/* 最大G */
	s8		cnstTimeAdjst;	/* 円弧区間の時間微調整[ms] */
} typeSlalomParam;


static const typeSlalomParam typSlaParam[12] = {
	{ -45.0f, -20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* R45°ターン  @0.5m/s	*/
	{  45.0f,  20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* L45°ターン  @0.5m/s	*/
	{ -90.0f, -20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* R90°ターン  @0.5m/s	*/
	{  90.0f,  20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* L90°ターン  @0.5m/s	*/
	{-135.0f, -20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* R135°ターン @0.5m/s	*/
	{ 135.0f,  20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* L135°ターン @0.5m/s	*/
	{-180.0f, -20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* R180°ターン @0.5m/s	*/
	{ 180.0f,  20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* L180°ターン @0.5m/s	*/
	{ -90.0f, -20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* R90°ターン(斜め→斜め) @0.5m/s	*/
	{  90.0f,  20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* L90°ターン(斜め→斜め) @0.5m/s	*/
	{ -90.0f, -20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* R90°ターン(低速版)	@0.2m/s	*/
	{  90.0f,  20000.0f,	 1.0f,	1.0f,	 0.5f,		0	},	/* L90°ターン(低速版)	@0.2m/s	*/
};


typedef struct {
	s16		data1;
	s16		data2;
	s16		data3;
	s16		data4;
	s16		data5;
} typeMotorLog;

#define		NUMBER_OF_MOTOR_LOG_DATA		(1000)
typeMotorLog motorLogData[NUMBER_OF_MOTOR_LOG_DATA];
u16	motorLogIndex = 0;

enum {
	ST_DRV_STOP = 0,
	ST_DRV_PAUSE,
	ST_DRV_RUN,
	ST_DRV_RUN_CONST,
	ST_DRV_RUN_END,
	ST_DRV_RUN_END2,
	ST_DRV_RUN_PRE_CONTENUE,
	ST_DRV_RUN_CONTENUE,
	ST_DRV_TURN,
	ST_DRV_TURN_CONST,
	ST_DRV_TURN_END,
	ST_DRV_TURN_END2,
	ST_DRV_SLA_ENTRY,
	ST_DRV_SLA_ANG_UP,
	ST_DRV_SLA_CONST,
	ST_DRV_SLA_ANG_DN,
	ST_DRV_SLA_ESCAPE
};
static	u8	driveState = ST_DRV_STOP;



typedef struct {
	float distance;				// mm
	float acceleration;			// m/s2
	float velocity;				// m/s
	float angleAcceleration;	// rad/s2
	float angleVelocity;		// rad/s
	float angle;				// rad
} motionType;

static motionType targetMotion;
static motionType currentMotion;

static float idealVelocity;
static float idealAngleVelocity;

typedef struct {
	float targetTorque;	// N・m
	float targetDuty;
	float revolution;	// rpm
	float speed;		// m/s
} motorType;

static motorType leftMotor;
static motorType rightMotor;

static u16 wallOnSideL = WALL_ON_SIDE_L;
static u16 wallOnSideR = WALL_ON_SIDE_R;



typedef struct {
	float p;
	float i;
	float d;
	float oldDifference;
} pidType;

static pidType straightPid;
static pidType angleVelPid;
static pidType anglePid;
static pidType wallPid;

static float angleVelPGain		= 0.0f;
static float angleVelIGain		= 0.0f;
static float angleVelDGain		= 0.0f;
static float angleVelIGainMax	= 0.0f;
static float angleVelFFGain		= 0.0;

#define	NUMBER_OF_DRIVE_COMMAND		(100)
typedef struct {
	u8		driveMode;		/* 停止、前進、後退、右その場ターン、左その場ターン */
	float	maxSpeed;		/* 最高スピード[m/s] */
	float	endSpeed;		/* 最終スピード[m/s] 0の場合は停止 */
	float	maxAcc;			/* 加速度[m/s2] */
	float	distance;		/* 距離[mm] */
	float	angleAcc;		/* 角加速度[deg/sec2] */
	float	angleSpeed;		/* 角速度[deg/sec] */
	float	angle;			/* 角度[deg] */
} typeDriveCommand;

static typeDriveCommand driveCommand[NUMBER_OF_DRIVE_COMMAND];
static u8 drvComSetIndex = 0;
static u8 drvComGetIndex = 0;
static u8 drvComCount = 0;			/* バッファにたまっている実行前のコマンド数 */
static u8 isDrvComBusy = FALSE;		/* TRUE:コマンド実行中、FALSE:コマンド実行完了(実行コマンド無し) */


	float errL = 0.0f;
	float errR = 0.0f;


static float angleAccBuf = 0.0f;

static float	runDistance = 0;
static float	speedDownDistance = 0;

static float	runAngle = 0;
static float	speedDownAngle = 0;


static	u8		slalomMode = 0;
static	float	slalomStartAngle = 0.0f;
static	u16		slalomAccAngTimer = 0;
static	u16		slalomEntoryTime = 0;
static	u16		slalomConstAngvelTime = 0;

static	u8		stopMotorDrive = 0;


static	float	monitorAngleVelPid = 0.0f;
static	float	monitorAnglePid = 0.0f;
float	monitorWallPid = 0.0f;
static	float	monitorWallSens = 0.0f;
static	s16		monitorWallSensL = 0;
static	s16		monitorWallSensR = 0;

static	u8		anglePidOn = 1;
static	u8		isWallControlEnable = 1;
static	u8		isSlantWallControlEnable = 0;

static	float	slaEntryLengthBuf = 0.0f;
static	float	slaEscpLengthBuf = 0.0f;

static	u8		isSlantRun = 0;

static	u8		motorDirectTestMode = 0;

static void ClearMotorData(void);
static void ControlMotor(void);
static void CalculateCurrentVelocity(void);
static void CalculateCurrentAngleVelocity(void);
static float CalcStraightPID(void);
static float CalcAngleVelPID(void);
static float CalcAnglePID(void);
static float CalcWallPID(void);
static u8 GetDrvCommand(typeDriveCommand *drvCom);
static void ExecuteDriveCommand(void);


void InitializeMotorControl(void) {
	
	u8 cnt;
	
	ClearMotorData();
	
	for (cnt=0;cnt<NUMBER_OF_DRIVE_COMMAND;cnt++) {
		driveCommand[cnt].driveMode	= 0;
		driveCommand[cnt].maxSpeed	= 0.0f;
		driveCommand[cnt].endSpeed	= 0.0f;
		driveCommand[cnt].maxAcc	= 0.0f;
		driveCommand[cnt].distance	= 0.0f;
		driveCommand[cnt].angleAcc	= 0.0f;
		driveCommand[cnt].angleSpeed	= 0.0f;
		driveCommand[cnt].angle		= 0.0f;
	}
	
	drvComSetIndex = 0;
	drvComGetIndex = 0;
	drvComCount = 0;			/* バッファにたまっている実行前のコマンド数 */
	isDrvComBusy = FALSE;		/* TRUE:コマンド実行中、FALSE:コマンド実行完了(実行コマンド無し) */

}

static void ClearMotorData(void) {

	targetMotion.distance			= 0.0f;
	targetMotion.acceleration		= 0.0f;
	targetMotion.velocity			= 0.0f;
	targetMotion.angleAcceleration	= 0.0f;
	targetMotion.angleVelocity		= 0.0f;
	targetMotion.angle				= GetGyroAngle();

	currentMotion.distance			= 0.0f;
	currentMotion.acceleration		= 0.0f;
	currentMotion.velocity			= 0.0f;
	currentMotion.angleAcceleration	= 0.0f;
	currentMotion.angleVelocity		= 0.0f;
	currentMotion.angle				= GetGyroAngle();
	
	idealVelocity = 0.0f;
	idealAngleVelocity = 0.0f;
	
	leftMotor.targetTorque	= 0.0f;
	leftMotor.targetDuty	= 0.0f;
	leftMotor.revolution	= 0.0f;
	leftMotor.speed			= 0.0f;

	rightMotor.targetTorque	= 0.0f;
	rightMotor.targetDuty	= 0.0f;
	rightMotor.revolution	= 0.0f;
	rightMotor.speed		= 0.0f;
	
	
	straightPid.p = 0.0f;
	straightPid.i = 0.0f;
	straightPid.d = 0.0f;
	straightPid.oldDifference = 0.0f;
	
	angleVelPid.p = 0.0f;
	angleVelPid.i = 0.0f;
	angleVelPid.d = 0.0f;
	angleVelPid.oldDifference = 0.0f;

	angleVelPGain = ANGLE_V_PID_P_GAIN;
	angleVelIGain = ANGLE_V_PID_I_GAIN;
	angleVelDGain = ANGLE_V_PID_D_GAIN;
	angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX;
	angleVelFFGain = ANGLE_V_FF_GAIN;

	anglePid.p = 0.0f;
	anglePid.i = 0.0f;
	anglePid.d = 0.0f;
	anglePid.oldDifference = 0.0f;

	wallPid.p = 0.0f;
	wallPid.i = 0.0f;
	wallPid.d = 0.0f;
	wallPid.oldDifference = 0.0f;
	
	anglePidOn = 0;
	isWallControlEnable = 0;
	isSlantWallControlEnable = 0;
	
}

void MotorControlCyclicTask(void) {
	
	if (motorDirectTestMode == 0) {
		ExecuteDriveCommand();
		ControlMotor();
	} else {
		InitializeMotorControl();
	}
}


extern	u8	debugLogStart;


static void ControlMotor(void) {

	float	targetAccelerationFF;
	float	targetAngleAccelerationFF;
	float	targetAccelerationFB;
	float	targetAngleAccelerationFB;

	float	targetAcceleration;
	float	targetAngleAcceleration;

	float	torqueOfStraight;
	float	torqueOfAngle;
	
	float	leftOutputDuty;
	float	rightOutputDuty;
	
	/* --- 前進系 --- */
	/* エンコーダから現在速度計算 */
	CalculateCurrentVelocity();

	/* 速度に対するPID */
	targetAccelerationFB = CalcStraightPID();

	targetAccelerationFF = targetMotion.acceleration * STRIGHT_VELOCITY_FF_GAIN;

	
	/* 目標加速度算出 */
	targetAcceleration = targetAccelerationFF + targetAccelerationFB;
	
	
	/* --- 回転系 --- */
	targetAngleAccelerationFF = targetMotion.angleAcceleration * angleVelFFGain;
	
	/* ジャイロから現在角速度計算 */
	CalculateCurrentAngleVelocity();
	
	/* 角速度に対するPID + 角度に対するPID */
	targetAngleAccelerationFB = CalcAngleVelPID() + CalcAnglePID() + CalcWallPID();
	
	/* 目標角加速度算出 */
	targetAngleAcceleration = targetAngleAccelerationFF + targetAngleAccelerationFB;
	
//	targetAngleAcceleration = 0;	/* Debug */
	
	/* --- 目標トルク計算 --- */
	torqueOfStraight	= ((WEIGHT_OF_MOUSE * targetAcceleration) / 2.0f) * (RADIUS_OF_WHEEL / RATIO_OF_GEAR);
	torqueOfAngle		= (MOMENT_OF_INERTIA * targetAngleAcceleration) / TREAD;
	
	leftMotor.targetTorque	= (torqueOfStraight - torqueOfAngle);
	rightMotor.targetTorque	= (torqueOfStraight + torqueOfAngle);


	{
		float batteryVoltage1mv;

		batteryVoltage1mv = GetBatteryVoltage1mv();
		
		/* Duty比に変換 */
		leftMotor.targetDuty	= (((REGISTANCE_OF_MOTOR_COIL * leftMotor.targetTorque) / CONSTANT_OF_MOTOR_CURRENT) + (CONSTANT_OF_MOTOR_VOLTAGE * leftMotor.revolution)) / batteryVoltage1mv;
		rightMotor.targetDuty	= (((REGISTANCE_OF_MOTOR_COIL * rightMotor.targetTorque) / CONSTANT_OF_MOTOR_CURRENT) + (CONSTANT_OF_MOTOR_VOLTAGE * rightMotor.revolution)) / batteryVoltage1mv;
	}
	
	

#if 0

	/* debug */
	leftMotor.targetDuty = 0.3f;
	rightMotor.targetDuty = 0.0f;

#endif
	

	/* Dutyの符号によってモーターの回転方向を変える */
	if (leftMotor.targetDuty >= 0.0f) {
		GpioWrteLeftMotorCw(0);
		leftOutputDuty = leftMotor.targetDuty;
	} else {
		GpioWrteLeftMotorCw(1);
		leftOutputDuty = leftMotor.targetDuty * (-1.0f);
	}

	if (rightMotor.targetDuty >= 0.0f) {
		GpioWrteRightMotorCw(1);
		rightOutputDuty = rightMotor.targetDuty;
	} else {
		GpioWrteRightMotorCw(0);
		rightOutputDuty = rightMotor.targetDuty * (-1.0f);
	}


	if (leftOutputDuty > LIMIT_OF_DUTY_HIGH)	leftOutputDuty = LIMIT_OF_DUTY_HIGH;
//	if (leftOutputDuty < LIMIT_OF_DUTY_LOW)		leftOutputDuty = LIMIT_OF_DUTY_LOW;
	if (rightOutputDuty > LIMIT_OF_DUTY_HIGH)	rightOutputDuty = LIMIT_OF_DUTY_HIGH;
//	if (rightOutputDuty < LIMIT_OF_DUTY_LOW)	rightOutputDuty = LIMIT_OF_DUTY_LOW;


	/* Duty比をレジスタに設定 */
	SetMotorDuty(leftOutputDuty, rightOutputDuty);
	
	
	/* Debug */
	if (isDrvComBusy == TRUE) {
//	if (debugLogStart == 1) {
		if (motorLogIndex < NUMBER_OF_MOTOR_LOG_DATA ) {
#if 0
			/* スラローム */
			motorLogData[motorLogIndex].data1 = (s16)(idealVelocity * 1000);
			motorLogData[motorLogIndex].data2 = (s16)(currentMotion.velocity * 1000);
			motorLogData[motorLogIndex].data3 = (s16)(idealAngleVelocity * 1000);
//			motorLogData[motorLogIndex].data4 = (s16)(currentMotion.angleVelocity * 1000);
//			motorLogData[motorLogIndex].data4 = (s16)(driveState);
			motorLogData[motorLogIndex].data4 = (s16)(targetMotion.distance);
			motorLogData[motorLogIndex].data5 = (s16)(currentMotion.distance);
#elif 0
			/* スラローム */
			motorLogData[motorLogIndex].data1 = (s16)(idealVelocity * 1000);
			motorLogData[motorLogIndex].data2 = (s16)(currentMotion.velocity * 1000);
			motorLogData[motorLogIndex].data3 = (s16)(idealAngleVelocity * 1000);
			motorLogData[motorLogIndex].data4 = (s16)(currentMotion.angleVelocity * 1000);
			motorLogData[motorLogIndex].data5 = (s16)(currentMotion.angle*1000);
#elif 0
			/* スラローム */
			motorLogData[motorLogIndex].data1 = (s16)(currentMotion.velocity * 1000);
			motorLogData[motorLogIndex].data2 = (s16)(currentMotion.angleVelocity * 1000);
			motorLogData[motorLogIndex].data3 = (s16)(currentMotion.angle*1000);
			motorLogData[motorLogIndex].data4 = (s16)(leftMotor.targetDuty*1000);
			motorLogData[motorLogIndex].data5 = (s16)(rightMotor.targetDuty*1000);
#elif 1
			/* 直進 */
			motorLogData[motorLogIndex].data1 = (s16)(idealVelocity * 1000);
			motorLogData[motorLogIndex].data2 = (s16)(currentMotion.velocity * 1000);
			motorLogData[motorLogIndex].data3 = (s16)(currentMotion.distance);
			motorLogData[motorLogIndex].data4 = (s16)(leftMotor.targetDuty*1000);
			motorLogData[motorLogIndex].data5 = (s16)(rightMotor.targetDuty*1000);
#elif 0
			/* 超信地旋回 */
			motorLogData[motorLogIndex].data1 = (s16)(idealAngleVelocity * 1000);
			motorLogData[motorLogIndex].data2 = (s16)(currentMotion.angleVelocity * 1000);
			motorLogData[motorLogIndex].data3 = (s16)(currentMotion.angle*1000);
			motorLogData[motorLogIndex].data4 = (s16)(leftMotor.targetDuty*1000);
			motorLogData[motorLogIndex].data5 = (s16)(rightMotor.targetDuty*1000);
#elif 0
			/* 直進 */
			motorLogData[motorLogIndex].data1 = (s16)(monitorWallSens);
			motorLogData[motorLogIndex].data2 = (s16)(monitorAngleVelPid);
			motorLogData[motorLogIndex].data3 = (s16)(monitorAnglePid);
			motorLogData[motorLogIndex].data4 = (s16)(monitorWallPid);
			motorLogData[motorLogIndex].data5 = (s16)((monitorAnglePid+monitorAngleVelPid+monitorWallPid));
#elif 0
			/* 直進(壁制御) */
			motorLogData[motorLogIndex].data1 = (s16)(monitorWallSensL);
			motorLogData[motorLogIndex].data2 = (s16)(monitorWallSensR);
			motorLogData[motorLogIndex].data3 = (s16)(wallOnSideL);
			motorLogData[motorLogIndex].data4 = (s16)(wallOnSideR);
			motorLogData[motorLogIndex].data5 = (s16)(monitorWallPid);
#elif 0
			/* その他 */
			motorLogData[motorLogIndex].data1 = (s16)(drvComSetIndex);
			motorLogData[motorLogIndex].data2 = (s16)(drvComGetIndex);
			motorLogData[motorLogIndex].data3 = (s16)(currentMotion.angle*1000);
			motorLogData[motorLogIndex].data4 = (s16)(targetMotion.distance);
			motorLogData[motorLogIndex].data5 = (s16)(currentMotion.distance);
#endif
			motorLogIndex++;
		}
	}
	
	
	
}


static void CalculateCurrentVelocity(void) {
	
	static float leftSpeedBuff[10] = {0.0f};
	static float rightSpeedBuff[10] = {0.0f};
	
	GetDistance(leftSpeedBuff, rightSpeedBuff);
	
	/* --- Left --- */
	leftMotor.speed =  (leftSpeedBuff[0] +
						leftSpeedBuff[1] +
						leftSpeedBuff[2] +
						leftSpeedBuff[3] +
						leftSpeedBuff[4] +
						leftSpeedBuff[5] +
						leftSpeedBuff[6] +
						leftSpeedBuff[7] +
						leftSpeedBuff[8] +
						leftSpeedBuff[9]) / 10.0f;
						
	leftSpeedBuff[9] = leftSpeedBuff[8];
	leftSpeedBuff[8] = leftSpeedBuff[7];
	leftSpeedBuff[7] = leftSpeedBuff[6];
	leftSpeedBuff[6] = leftSpeedBuff[5];
	leftSpeedBuff[5] = leftSpeedBuff[4];
	leftSpeedBuff[4] = leftSpeedBuff[3];
	leftSpeedBuff[3] = leftSpeedBuff[2];
	leftSpeedBuff[2] = leftSpeedBuff[1];
	leftSpeedBuff[1] = leftSpeedBuff[0];
	
	leftMotor.revolution = ((60.0f * RATIO_OF_GEAR * leftMotor.speed) / (2.0f * PI * RADIUS_OF_WHEEL)) * (-1.0f);
	
	
	/* --- Right --- */
	rightMotor.speed = (rightSpeedBuff[0] +
						rightSpeedBuff[1] +
						rightSpeedBuff[2] +
						rightSpeedBuff[3] +
						rightSpeedBuff[4] +
						rightSpeedBuff[5] +
						rightSpeedBuff[6] +
						rightSpeedBuff[7] +
						rightSpeedBuff[8] +
						rightSpeedBuff[9]) / 10.0f;
						
	rightSpeedBuff[9] = rightSpeedBuff[8];
	rightSpeedBuff[8] = rightSpeedBuff[7];
	rightSpeedBuff[7] = rightSpeedBuff[6];
	rightSpeedBuff[6] = rightSpeedBuff[5];
	rightSpeedBuff[5] = rightSpeedBuff[4];
	rightSpeedBuff[4] = rightSpeedBuff[3];
	rightSpeedBuff[3] = rightSpeedBuff[2];
	rightSpeedBuff[2] = rightSpeedBuff[1];
	rightSpeedBuff[1] = rightSpeedBuff[0];

	rightMotor.revolution = ((60.0f * RATIO_OF_GEAR * rightMotor.speed) / (2.0f * PI * RADIUS_OF_WHEEL)) * (-1.0f);
	
	/* --- 重心の速度算出 --- */
	currentMotion.velocity = (leftMotor.speed + rightMotor.speed) / 2.0f;

	/* --- 重心の距離算出 --- */
	currentMotion.distance += ((leftSpeedBuff[0] + rightSpeedBuff[0]) / 2.0f);
}


static void CalculateCurrentAngleVelocity(void) {
	
	currentMotion.angleVelocity	= GetGyroSensorValue();
	currentMotion.angle			= GetGyroAngle();
	
}

static float CalcStraightPID(void) {
	
	float output;
	float difference;
	float buf;
	
	buf = (targetMotion.acceleration * CONTROL_CYCLE_SEC);

	/* 目標速度更新 */
	if ((ABS(idealVelocity - targetMotion.velocity)) > ABS(buf)) {
		idealVelocity += buf;
	} else {
		idealVelocity = targetMotion.velocity;
	}
	
	difference = idealVelocity - currentMotion.velocity;
	
	straightPid.p	=   difference * STRIGHT_VELOCITY_PID_P_GAIN;
	straightPid.i	+= (difference * STRIGHT_VELOCITY_PID_I_GAIN);
	straightPid.d	=  (difference - straightPid.oldDifference) * STRIGHT_VELOCITY_PID_D_GAIN;

	if(straightPid.i >  STRIGHT_VELOCITY_PID_I_GAIN_MAX)	straightPid.i = STRIGHT_VELOCITY_PID_I_GAIN_MAX;
	if(straightPid.i < -STRIGHT_VELOCITY_PID_I_GAIN_MAX)	straightPid.i = -STRIGHT_VELOCITY_PID_I_GAIN_MAX;

	output = straightPid.p + straightPid.i + straightPid.d;

	straightPid.oldDifference = difference;
	
	return output;
}


static float CalcAngleVelPID(void) {
	
	float output;
	float difference;
	float buf;
	
	/* 目標角速度更新 */
	buf = (targetMotion.angleAcceleration * CONTROL_CYCLE_SEC);

	/* 目標速度更新 */
	if ((ABS(idealAngleVelocity - targetMotion.angleVelocity)) > ABS(buf)) {
		idealAngleVelocity += buf;
	} else {
		idealAngleVelocity = targetMotion.angleVelocity;
	}

	difference = idealAngleVelocity - currentMotion.angleVelocity;
	
	angleVelPid.p	=   difference * angleVelPGain;
	angleVelPid.i	+= (difference * angleVelIGain);
	angleVelPid.d	=  (difference - angleVelPid.oldDifference) * angleVelDGain;

	if(angleVelPid.i > angleVelIGainMax)			angleVelPid.i = angleVelIGainMax;
	if(angleVelPid.i < angleVelIGainMax*(-1.0f))	angleVelPid.i = angleVelIGainMax*(-1.0f);

	output = angleVelPid.p + angleVelPid.i + angleVelPid.d;

	angleVelPid.oldDifference = difference;
	
	monitorAngleVelPid = output;//DEBUG
	
	return output;
}

static float CalcAnglePID(void) {
	
	float output = 0.0f;
	float difference = 0.0f;

	if (anglePidOn == 1) {
		difference = targetMotion.angle - currentMotion.angle;
		
		anglePid.p	=   difference * ANGLE_PID_P_GAIN;
		anglePid.i	+= (difference * ANGLE_PID_I_GAIN);
		anglePid.d	=  (difference - anglePid.oldDifference) * ANGLE_PID_D_GAIN;

		if(anglePid.i > ANGLE_PID_I_GAIN_MAX)	anglePid.i = ANGLE_PID_I_GAIN_MAX;
		if(anglePid.i < -ANGLE_PID_I_GAIN_MAX)	anglePid.i = -ANGLE_PID_I_GAIN_MAX;

		output = anglePid.p + anglePid.i + anglePid.d;

		anglePid.oldDifference = difference;
	} else {
		anglePid.oldDifference = 0.0f;
	}
	
	
	monitorAnglePid = output;//DEBUG

	return output;
}

static float CalcWallPID(void) {
	
	static s16 oldSensSideL[10] = {0};
	static s16 oldSensSideR[10] = {0};
	
	
	float output = 0.0f;
	float difference;
	
	
	s16 sensSideL;
	s16 sensSideR;
	s16 sensFrontL;
	s16 sensFrontR;
	
	u8 cnt;
	
	if (isWallControlEnable == 1) {

		sensSideL	= GetWallSensor(SIDE_LEFT);
		sensSideR	= GetWallSensor(SIDE_RIGHT);
		sensFrontL	= GetWallSensor(FRONT_LEFT);
		sensFrontR	= GetWallSensor(FRONT_RIGHT);

		monitorWallSensL = sensSideL; //DEBUG
		monitorWallSensR = sensSideR; //DEBUG

		if (isSlantWallControlEnable == 0) {

			/* --- 基準値更新 --- */
			#if 0
			if (sensSideL >= oldSensSideL[9]) {
				if ((sensSideL - oldSensSideL[9]) > WALL_ON_ADJUST_VALUE)	wallOnSideL = WALL_CENTER_SIDE_L + 5;
				else														wallOnSideL = WALL_ON_SIDE_L;
			} else {
				if ((oldSensSideL[9] - sensSideL) > WALL_OFF_ADJUST_VALUE)	wallOnSideL = WALL_CENTER_SIDE_L + 20;
				else														wallOnSideL = WALL_ON_SIDE_L;
			}
			#endif
			
			oldSensSideL[9] = oldSensSideL[8];
			oldSensSideL[8] = oldSensSideL[7];
			oldSensSideL[7] = oldSensSideL[6];
			oldSensSideL[6] = oldSensSideL[5];
			oldSensSideL[5] = oldSensSideL[4];
			oldSensSideL[4] = oldSensSideL[3];
			oldSensSideL[3] = oldSensSideL[2];
			oldSensSideL[2] = oldSensSideL[1];
			oldSensSideL[1] = oldSensSideL[0];
			oldSensSideL[0] = sensSideL;
			
//			sensSideL = (s16)((((s32)oldSensSideL[0]) + ((s32)oldSensSideL[1]) + ((s32)oldSensSideL[2]) + ((s32)oldSensSideL[3]) + ((s32)oldSensSideL[4]) +
//							   ((s32)oldSensSideL[5]) + ((s32)oldSensSideL[6]) + ((s32)oldSensSideL[7]) + ((s32)oldSensSideL[8]) + ((s32)oldSensSideL[9])) / 10ul);

			#if 0
			if (sensSideR >= oldSensSideR[9]) {
				if ((sensSideR - oldSensSideR[9]) > WALL_ON_ADJUST_VALUE)	wallOnSideR = WALL_CENTER_SIDE_R + 5;
				else														wallOnSideR = WALL_ON_SIDE_R;
			} else {
				if ((oldSensSideR[9] - sensSideR) > WALL_OFF_ADJUST_VALUE)	wallOnSideR = WALL_CENTER_SIDE_R + 20;
				else														wallOnSideR = WALL_ON_SIDE_R;
			}
			#endif
			
			oldSensSideR[9] = oldSensSideR[8];
			oldSensSideR[8] = oldSensSideR[7];
			oldSensSideR[7] = oldSensSideR[6];
			oldSensSideR[6] = oldSensSideR[5];
			oldSensSideR[5] = oldSensSideR[4];
			oldSensSideR[4] = oldSensSideR[3];
			oldSensSideR[3] = oldSensSideR[2];
			oldSensSideR[2] = oldSensSideR[1];
			oldSensSideR[1] = oldSensSideR[0];
			oldSensSideR[0] = sensSideR;

//			sensSideR = (s16)((((s32)oldSensSideR[0]) + ((s32)oldSensSideR[1]) + ((s32)oldSensSideR[2]) + ((s32)oldSensSideR[3]) + ((s32)oldSensSideR[4]) +
//							   ((s32)oldSensSideR[5]) + ((s32)oldSensSideR[6]) + ((s32)oldSensSideR[7]) + ((s32)oldSensSideR[8]) + ((s32)oldSensSideR[9])) / 10ul);
			
			/* --- 制御量計算 --- */
			if (currentMotion.velocity <= WALL_ADJ_STOP_SPD) {
				difference = 0.0f;
				
			} else if ((sensFrontL > SIDE_CTL_OFF_FRONT) || (sensFrontR > SIDE_CTL_OFF_FRONT)) {
//				difference = ((float)sensFrontR - (float)sensFrontL) * 0.1f;
				difference = 0.0f;
				
			} else if ((sensSideL > wallOnSideL) && (sensSideR > wallOnSideR)) {
				
				errL = ((float)WALL_CENTER_SIDE_L - (float)sensSideL);
				if (errL > 0) errL *= 2.0f;
				
				errR = ((float)sensSideR - (float)WALL_CENTER_SIDE_R);
				
				difference = (errL + errR) / 2.0f;
				
			} else if (sensSideL > wallOnSideL) {
				errL = ((float)WALL_CENTER_SIDE_L - (float)sensSideL);

				if (errL > 0) errL *= 2.0f;
				difference = errL;
				errR = 0.0f;
				
			} else if ((sensSideR > wallOnSideR)) {
				errR = ((float)sensSideR - (float)WALL_CENTER_SIDE_R);
				difference = errR;
				errL = 0.0f;
				
			} else {
				difference = 0.0f;
				errL = 0.0f;
				errR = 0.0f;
			}
			
			monitorWallSens = difference;	//DEBUG

			wallPid.p  = difference * WALL_PID_P_GAIN;
			wallPid.i += (difference * WALL_PID_I_GAIN);
			wallPid.d  = (difference - wallPid.oldDifference) * WALL_PID_D_GAIN;

			if(wallPid.i > WALL_PID_GAIN_MAX)	wallPid.i = WALL_PID_GAIN_MAX;
			if(wallPid.i < -WALL_PID_GAIN_MAX)	wallPid.i = -WALL_PID_GAIN_MAX;


			wallPid.oldDifference = difference;
			output = wallPid.p + wallPid.i + wallPid.d;

		} else {
			/* 斜め走行用姿勢制御 */
			
#if 0
			if (sensFrontL > WALL_CTL_SLANT_L) {
				output = (sensFrontL - WALL_CTL_SLANT_L) * (-1.0f);

			} else if (sensFrontR > WALL_CTL_SLANT_R) {
				output = (sensFrontR - WALL_CTL_SLANT_R);

			} else {
				output = 0.0f;
			}
			
			output *= 110.0f;
#endif

		}

	} else {
		
		for (cnt=0;cnt<10;cnt++) {
			oldSensSideL[cnt] = 0;
			oldSensSideR[cnt] = 0;
		}

		wallPid.oldDifference = 0.0f;
		output = 0.0f;
	}
	
	monitorWallPid = output;
	
	return output;
}

void WallControl(u8 mode) {
	
	if (mode == CONTROL_MODE_STRAIGHT) {
		isWallControlEnable = 1;
		isSlantWallControlEnable = 0;
		anglePidOn = 1;

	} else if (mode == CONTROL_MODE_SLANT) {
		isWallControlEnable = 1;
		isSlantWallControlEnable = 1;
		anglePidOn = 1;

	} else {
		isWallControlEnable = 0;
		isSlantWallControlEnable = 0;
		anglePidOn = 1;
	}
	
	
	
	isWallControlEnable = 0;	//DEBUG
	
	
	
}

u8	IsMoveComSetOk(void) {
	
	u8	isEmpty = 0;
	
	if (drvComCount < (NUMBER_OF_DRIVE_COMMAND-1)) {
		isEmpty = 1;
	}
	
	return (isEmpty);
}


void SetMoveCommand(u8 drvMode, float maxSpeed, float endSpeed, float maxAcc, float distance, float angleAcc, float angleSpeed, float angle) {

	isDrvComBusy = TRUE;
	
	driveCommand[drvComSetIndex].driveMode	= drvMode;
	driveCommand[drvComSetIndex].maxSpeed	= maxSpeed;
	driveCommand[drvComSetIndex].endSpeed	= endSpeed;
	driveCommand[drvComSetIndex].maxAcc		= maxAcc;
	driveCommand[drvComSetIndex].distance	= distance;
	driveCommand[drvComSetIndex].angleAcc	= (angleAcc * PI) / 180.0f;
	driveCommand[drvComSetIndex].angleSpeed	= (angleSpeed * PI) / 180.0f;
	driveCommand[drvComSetIndex].angle		= (angle * PI) / 180.0f;

	if (drvComSetIndex < (NUMBER_OF_DRIVE_COMMAND-1)) {
		drvComSetIndex++;
	} else {
		drvComSetIndex = 0;
	}

	drvComCount++;

}

static u8 GetDrvCommand(typeDriveCommand *drvCom) {

	u8	isDrvComEn = FALSE;

	if (drvComGetIndex != drvComSetIndex) {

		drvCom->driveMode	= driveCommand[drvComGetIndex].driveMode;
		drvCom->maxSpeed	= driveCommand[drvComGetIndex].maxSpeed;
		drvCom->endSpeed	= driveCommand[drvComGetIndex].endSpeed;
		drvCom->maxAcc		= driveCommand[drvComGetIndex].maxAcc;
		drvCom->distance	= driveCommand[drvComGetIndex].distance;
		drvCom->angleAcc	= driveCommand[drvComGetIndex].angleAcc;
		drvCom->angleSpeed	= driveCommand[drvComGetIndex].angleSpeed;
		drvCom->angle		= driveCommand[drvComGetIndex].angle;

		if (drvComGetIndex < (NUMBER_OF_DRIVE_COMMAND-1)) {
			drvComGetIndex++;
		} else {
			drvComGetIndex = 0;
		}

		if (drvComCount > 0) {
			drvComCount--;
		}

		isDrvComEn = TRUE;
		
	}

	return(isDrvComEn);
}

u8 IsDrvComExecuteBusy(void) {
	return(isDrvComBusy);
}

u8 IsDrvComBufferFull(void) {
	
	u8	isBufferFull = TRUE;
	
	if (drvComCount < (NUMBER_OF_DRIVE_COMMAND-1)) {
		isBufferFull = FALSE;
	}
	
	return (isBufferFull);
}

static void ExecuteDriveCommand(void) {

	static	u16	holdOffTimer = MOTOR_HOLD_TIME;
	static	typeDriveCommand drvCommand;
	
	u8		isDrvComEn;
	
	if (holdOffTimer < 0xFFFEu) holdOffTimer++;
	
	/* 強制停止 */
	if (stopMotorDrive == 1) {
		
		StopMotorTimers();
		driveState = ST_DRV_STOP;
		holdOffTimer = MOTOR_HOLD_TIME;
		InitializeMotorControl();
		
	} else {
	
		switch (driveState) {
			default:
			case ST_DRV_STOP:
			
				isDrvComEn = GetDrvCommand(&drvCommand);
				
				if (isDrvComEn == TRUE) {
					
					ClearDistance();
					
					ClearMotorData();
					GpioWrteMotorSleep(1);
					StartMotorTimers();

					runDistance = 0.0f;
					speedDownDistance = 0.0f;
					runAngle = 0.0f;
					speedDownAngle = 0.0f;

					
					switch (drvCommand.driveMode) {
						default :
						case DRV_MODE_STOP : 
							break;
						case DRV_MODE_PAUSE : 
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = 0.0f;
							targetMotion.distance = 0.0f;
							targetMotion.angleAcceleration = 0.0f;
							targetMotion.angleVelocity = 0.0f;
							driveState = ST_DRV_PAUSE;
							break;
						case DRV_MODE_STRAIGHT : 
							targetMotion.acceleration = drvCommand.maxAcc;
							targetMotion.velocity = drvCommand.maxSpeed;
							targetMotion.distance = drvCommand.distance;
							targetMotion.angleAcceleration = 0.0f;
							targetMotion.angleVelocity = 0.0f;

							angleVelPGain = ANGLE_V_PID_P_GAIN;
							angleVelIGain = ANGLE_V_PID_I_GAIN;
							angleVelDGain = ANGLE_V_PID_D_GAIN;
							angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX;
							angleVelFFGain = ANGLE_V_FF_GAIN;
							WallControl(CONTROL_MODE_STRAIGHT);

							driveState = ST_DRV_RUN;
							break;
						case DRV_MODE_SLANT :
							targetMotion.acceleration = drvCommand.maxAcc;
							targetMotion.velocity = drvCommand.maxSpeed;
							targetMotion.distance = drvCommand.distance;
							targetMotion.angleAcceleration = 0.0f;
							targetMotion.angleVelocity = 0.0f;

							angleVelPGain = ANGLE_V_PID_P_GAIN;
							angleVelIGain = ANGLE_V_PID_I_GAIN;
							angleVelDGain = ANGLE_V_PID_D_GAIN;
							angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX;
							angleVelFFGain = ANGLE_V_FF_GAIN;
							WallControl(CONTROL_MODE_SLANT);
							
							driveState = ST_DRV_RUN;
							break;
						case DRV_MODE_TURN_L :
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity =  0.0f;
							targetMotion.distance =  0.0f;
							targetMotion.angleAcceleration = drvCommand.angleAcc;
							targetMotion.angleVelocity = drvCommand.angleSpeed;
							targetMotion.angle += drvCommand.angle;
							
							angleAccBuf = targetMotion.angleAcceleration;
							angleVelPGain = ANGLE_V_PID_P_GAIN;
							angleVelIGain = ANGLE_V_PID_I_GAIN;
							angleVelDGain = ANGLE_V_PID_D_GAIN;
							angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX;
							angleVelFFGain = ANGLE_V_FF_GAIN;
							WallControl(CONTROL_MODE_NONE);
							
							driveState = ST_DRV_TURN;
							break;
						case DRV_MODE_TURN_R :
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity =  0.0f;
							targetMotion.distance =  0.0f;
							targetMotion.angleAcceleration = drvCommand.angleAcc * (-1.0f);
							targetMotion.angleVelocity = drvCommand.angleSpeed * (-1.0f);
							targetMotion.angle -= drvCommand.angle;
							driveState = ST_DRV_TURN;

							angleAccBuf = targetMotion.angleAcceleration;
							angleVelPGain = ANGLE_V_PID_P_GAIN;
							angleVelIGain = ANGLE_V_PID_I_GAIN;
							angleVelDGain = ANGLE_V_PID_D_GAIN;
							angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX;
							angleVelFFGain = ANGLE_V_FF_GAIN;
							WallControl(CONTROL_MODE_NONE);

							break;
					}
					
				} else {

					isDrvComBusy = FALSE;
					if (holdOffTimer >= MOTOR_HOLD_TIME) {
						GpioWrteMotorSleep(0);
					}
					
				}
				
				break;

			case ST_DRV_RUN:		/* 加速開始 */

				runDistance = targetMotion.distance - currentMotion.distance;
				speedDownDistance = ((currentMotion.velocity * currentMotion.velocity) - (drvCommand.endSpeed * drvCommand.endSpeed)) / (2.0f * drvCommand.maxAcc);
				speedDownDistance *= 1000.0f;

				/* 減速開始地点まで進んだら減速開始 */
				if (runDistance < speedDownDistance) {
					targetMotion.acceleration = (drvCommand.maxAcc) * SPEED_DOWN_ACC;
					targetMotion.velocity = drvCommand.endSpeed;
					driveState = ST_DRV_RUN_END;

				/* 目標速度まで達したら加速OFF */
				} else if (idealVelocity >= targetMotion.velocity) {
					targetMotion.acceleration = 0.0f;
					driveState = ST_DRV_RUN_CONST;

				}
//				 else if (targetMotion.distance <= currentMotion.distance) {
//					driveState = ST_DRV_RUN_END2;
//					
//				}

				/* 前進しようとして前壁があったら緊急停止 */
				if ((GetWallSensor(FRONT_RIGHT) >= WALL_CENTER_FRONT_R/2) && (GetWallSensor(FRONT_LEFT) >= WALL_CENTER_FRONT_L/2)) {
//					StopMotor();
				}
				
				break;

			case ST_DRV_RUN_CONST:		/* 一定速度区間 */

				runDistance = targetMotion.distance - currentMotion.distance;
				speedDownDistance = ((currentMotion.velocity * currentMotion.velocity) - (drvCommand.endSpeed * drvCommand.endSpeed)) / (2.0f * drvCommand.maxAcc);
				speedDownDistance *= 1000.0f;

				/* 減速開始地点まで進んだら減速開始 */
				if (runDistance < speedDownDistance) {
					targetMotion.acceleration = (drvCommand.maxAcc) * SPEED_DOWN_ACC;
					targetMotion.velocity = drvCommand.endSpeed;
					driveState = ST_DRV_RUN_END;
				}

				/* 前進しようとして前壁があったら緊急停止 */
//				if ((GetWallSensor(FRONT_RIGHT) >= WALL_CENTER_FRONT_R/2) && (GetWallSensor(FRONT_LEFT) >= WALL_CENTER_FRONT_L/2)) {
//					StopMotor();
//				}
				break;
			case ST_DRV_RUN_END:	/* 減速 */
				/* 目標速度まで減速したら減速OFF */
				if (idealVelocity <= targetMotion.velocity) {
					targetMotion.acceleration = 0.0f;
					driveState = ST_DRV_RUN_END2;
				}
				break;
			case ST_DRV_RUN_END2:	/* 減速 */

				if (isSlantRun == 1) {
					WallControl(CONTROL_MODE_NONE);	//斜め走行時、停止直前は壁をみない
				}

				if (drvCommand.endSpeed <= STOP_SPD) {

//					WallControl(CONTROL_MODE_NONE);	//停止直前は壁をみない

					/* 指定距離進んだら停止 */
					if (targetMotion.distance <= currentMotion.distance) {
						
						if ((GetWallSensor(FRONT_LEFT) > WALL_ON_FRONT_L) && (GetWallSensor(FRONT_RIGHT) > WALL_ON_FRONT_R) &&
							(GetWallSensor(FRONT_LEFT) <= WALL_CENTER_FRONT_L) && (GetWallSensor(FRONT_RIGHT) <= WALL_CENTER_FRONT_R)) {
							/* 進み足りない場合は前進 */
						} else {
							targetMotion.velocity = 0.0f;
							idealVelocity = 0.0f;
							StopMotorTimers();
							holdOffTimer = 0;
							driveState = ST_DRV_STOP;
						}

					/* 前壁に近すぎる場合は停止 */
					} else if ((GetWallSensor(FRONT_LEFT) > WALL_CENTER_FRONT_L) && (GetWallSensor(FRONT_RIGHT) > WALL_CENTER_FRONT_R)) {
						targetMotion.velocity = 0.0f;
						idealVelocity = 0.0f;
						StopMotorTimers();
						holdOffTimer = 0;
						driveState = ST_DRV_STOP;
					}
				} else {
					if (targetMotion.distance <= currentMotion.distance) {
						driveState = ST_DRV_RUN_CONTENUE;
					}
				}
				break;
			case ST_DRV_RUN_CONTENUE :	/* 走行継続 */
				
				isDrvComEn = GetDrvCommand(&drvCommand);
				
				if (isDrvComEn == TRUE) {
					
					ClearDistance();

					switch (drvCommand.driveMode) {
						default :
						case DRV_MODE_STOP : 
							/* 走行停止(コマンド異常) */
							break;
						case DRV_MODE_STRAIGHT :
							targetMotion.acceleration = drvCommand.maxAcc;
							targetMotion.velocity = drvCommand.maxSpeed;
							targetMotion.distance = drvCommand.distance;
							targetMotion.angleAcceleration = 0.0f;
							targetMotion.angleVelocity = 0.0f;

							angleVelPGain = ANGLE_V_PID_P_GAIN;
							angleVelIGain = ANGLE_V_PID_I_GAIN;
							angleVelDGain = ANGLE_V_PID_D_GAIN;
							angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX;
							angleVelFFGain = ANGLE_V_FF_GAIN;
							WallControl(CONTROL_MODE_STRAIGHT);

							isSlantRun = 0;

							driveState = ST_DRV_RUN;
							break;
						case DRV_MODE_SLANT :
							targetMotion.acceleration = drvCommand.maxAcc;
							targetMotion.velocity = drvCommand.maxSpeed;
							targetMotion.distance = drvCommand.distance;
							targetMotion.angleAcceleration = 0.0f;
							targetMotion.angleVelocity = 0.0f;

							angleVelPGain = ANGLE_V_PID_P_GAIN;
							angleVelIGain = ANGLE_V_PID_I_GAIN;
							angleVelDGain = ANGLE_V_PID_D_GAIN;
							angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX;
							angleVelFFGain = ANGLE_V_FF_GAIN;
							WallControl(CONTROL_MODE_SLANT);
							
							isSlantRun = 1;
							
							driveState = ST_DRV_RUN;
							break;
						case DRV_MODE_R45SLA :
							slalomMode = 0;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;

							if (isSlantRun == 1) {
								slaEntryLengthBuf = typSlaParam[slalomMode].entryLen + 15.0f;
								slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen - 22.0f;
							} else {
								slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
								slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							}
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_L45SLA :
							slalomMode = 1;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;
							targetMotion.distance = typSlaParam[slalomMode].entryLen;

							if (isSlantRun == 1) {
								slaEntryLengthBuf = typSlaParam[slalomMode].entryLen + 15.0f;
								slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen - 15.0f;
							} else {
								slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
								slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							}
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_R90SLA :
							slalomMode = 2;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;

							slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
							slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_L90SLA :
							slalomMode = 3;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;

							slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
							slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_R135SLA :
							slalomMode = 4;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;
							targetMotion.distance = typSlaParam[slalomMode].entryLen;

							if (isSlantRun == 1) {
								slaEntryLengthBuf = typSlaParam[slalomMode].entryLen - 6.0f;
								slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen + 7.0f;
							} else {
								slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
								slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							}
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_L135SLA :
							slalomMode = 5;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;
							targetMotion.distance = typSlaParam[slalomMode].entryLen;

							if (isSlantRun == 1) {
								slaEntryLengthBuf = typSlaParam[slalomMode].entryLen - 14.0f;
								slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen + 10.0f;
							} else {
								slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
								slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							}
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_R180SLA :
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;
							slalomMode = 6;

							slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
							slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_L180SLA :
							slalomMode = 7;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;

							slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
							slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_RN90SLA :
							slalomMode = 8;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;

							slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
							slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_LN90SLA :
							slalomMode = 9;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;

							slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
							slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_RS90SLA :
							slalomMode = 10;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;

							slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
							slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
						case DRV_MODE_LS90SLA :
							slalomMode = 11;
							targetMotion.acceleration = 0.0f;
							targetMotion.velocity = drvCommand.maxSpeed;

							slaEntryLengthBuf = typSlaParam[slalomMode].entryLen;
							slaEscpLengthBuf = typSlaParam[slalomMode].escapeLen;
							targetMotion.distance = slaEntryLengthBuf;

							driveState = ST_DRV_SLA_ENTRY;
							break;
					}
				} else {
					isDrvComBusy = FALSE;
				}
				
				break;
			case ST_DRV_SLA_ENTRY :


				if (currentMotion.distance >= targetMotion.distance) {
					ClearDistance();

					WallControl(CONTROL_MODE_NONE);
					
					/* 目標角速度、最大G設定 */
					targetMotion.angleAcceleration = ((typSlaParam[slalomMode].targetAngleAcc * PI) / 180.0f);
					targetMotion.angleVelocity = (9.8f * typSlaParam[slalomMode].maxG) / drvCommand.maxSpeed;
					targetMotion.angle += ((typSlaParam[slalomMode].targetAngle * PI) / 180.0f);
					
					slalomStartAngle = currentMotion.angle;
					slalomAccAngTimer = 0;
					angleVelPGain = ANGLE_V_PID_P_GAIN_S;
					angleVelIGain = ANGLE_V_PID_I_GAIN_S;
					angleVelDGain = ANGLE_V_PID_D_GAIN_S;
					angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX_S;
					angleVelFFGain = ANGLE_V_FF_GAIN_S;

					driveState = ST_DRV_SLA_ANG_UP;
				}
				
				break;
			case ST_DRV_SLA_ANG_UP :
				
				if (((targetMotion.angleAcceleration >= 0.0f) && (targetMotion.angleVelocity <= idealAngleVelocity)) ||
					((targetMotion.angleAcceleration <  0.0f) && (targetMotion.angleVelocity >= idealAngleVelocity))) {
//				if (((targetMotion.angleAcceleration >= 0.0f) && (targetMotion.angleVelocity <= currentMotion.angleVelocity)) ||
//					((targetMotion.angleAcceleration <  0.0f) && (targetMotion.angleVelocity >= currentMotion.angleVelocity))) {
					
					targetMotion.angleAcceleration = 0.0f;
					
					slalomEntoryTime = slalomAccAngTimer;
					slalomAccAngTimer = 0;
					
					slalomConstAngvelTime = (u16)(((((typSlaParam[slalomMode].targetAngle * PI) / 180.0f) - ((currentMotion.angle - slalomStartAngle) * 2.0f)) / targetMotion.angleVelocity) * 1000u);
					
					/* 微調整反映 */
					slalomConstAngvelTime += (float)typSlaParam[slalomMode].cnstTimeAdjst;
					
					driveState = ST_DRV_SLA_CONST;

				} else {
					slalomAccAngTimer++;
				}
				
				break;
			case ST_DRV_SLA_CONST :
				
				if (slalomAccAngTimer >= slalomConstAngvelTime) {
					
					targetMotion.angleAcceleration = ((typSlaParam[slalomMode].targetAngleAcc * PI) / 180.0f) * (-1.0f);
					targetMotion.angleVelocity = 0.0f;
					
					slalomAccAngTimer = 0;
					
					driveState = ST_DRV_SLA_ANG_DN;
					
				} else {
					slalomAccAngTimer++;
				}
				
				break;
			case ST_DRV_SLA_ANG_DN :
				
				if (slalomAccAngTimer >= slalomEntoryTime) {
					
					ClearDistance();

					if (drvCommand.endSpeed <= STOP_SPD) {
						targetMotion.acceleration = (drvCommand.maxAcc) * (-1.2f);
						targetMotion.velocity = drvCommand.endSpeed;
					} else {
						targetMotion.angleAcceleration = 0.0f;
					}
					
					targetMotion.distance = slaEscpLengthBuf;
					angleVelPGain = ANGLE_V_PID_P_GAIN;
					angleVelIGain = ANGLE_V_PID_I_GAIN;
					angleVelDGain = ANGLE_V_PID_D_GAIN;
					angleVelIGainMax = ANGLE_V_PID_I_GAIN_MAX;
					angleVelFFGain = ANGLE_V_FF_GAIN;
				
					driveState = ST_DRV_SLA_ESCAPE;
					
				} else {
					slalomAccAngTimer++;
				}
				
				break;
			case ST_DRV_SLA_ESCAPE :
				
				if (drvCommand.endSpeed <= STOP_SPD) {
					if (currentMotion.velocity <= targetMotion.velocity) {
						targetMotion.acceleration = 0.0f;
					}

					if (targetMotion.distance <= currentMotion.distance) {
						targetMotion.velocity = 0.0f;
						idealVelocity = 0.0f;
						StopMotorTimers();
						holdOffTimer = 0;
						driveState = ST_DRV_STOP;
					}
				} else {
					if (targetMotion.distance <= currentMotion.distance) {
						driveState = ST_DRV_RUN_CONTENUE;
					}
				}
				break;
			case ST_DRV_TURN :
				runAngle = targetMotion.angle - currentMotion.angle;
				speedDownAngle = (currentMotion.angleVelocity * currentMotion.angleVelocity) / (2.0f * angleAccBuf);

				/* 減速開始地点まで進んだら減速開始 */
				if ( ((angleAccBuf >= 0.0f) && (runAngle < speedDownAngle)) ||
					 ((angleAccBuf <  0.0f) && (runAngle > speedDownAngle))
					 ) {
					targetMotion.angleAcceleration = (angleAccBuf) * (-1.0f);		/* 早めに減速させてみる */
					if (angleAccBuf >= 0.0f)	targetMotion.angleVelocity = 0.3f;
					else						targetMotion.angleVelocity = -0.3f;
					
					driveState = ST_DRV_TURN_END;

				/* 目標角速度に達したら定角速度状態へ */
				} else if ( ((angleAccBuf >= 0.0f) && (targetMotion.angleVelocity < currentMotion.angleVelocity)) ||
					 		((angleAccBuf <  0.0f) && (targetMotion.angleVelocity > currentMotion.angleVelocity))
					 		) {
					targetMotion.angleAcceleration = 0.0f;
					driveState = ST_DRV_TURN_CONST;
				}
				
				break;
			case ST_DRV_TURN_CONST :
				runAngle = targetMotion.angle - currentMotion.angle;
				speedDownAngle = (currentMotion.angleVelocity * currentMotion.angleVelocity) / (2.0f * angleAccBuf);
				
				/* 減速開始地点まで進んだら減速開始 */
				if ( ((angleAccBuf >= 0.0f) && (runAngle < speedDownAngle)) ||
					 ((angleAccBuf <  0.0f) && (runAngle > speedDownAngle))
					 ) {
					targetMotion.angleAcceleration = (angleAccBuf) * (-1.0f);		/* 早めに減速させてみる */
					if (angleAccBuf >= 0.0f)	targetMotion.angleVelocity = 0.3f;
					else						targetMotion.angleVelocity = -0.3f;
					driveState = ST_DRV_TURN_END;
				}
				
				break;
			case ST_DRV_TURN_END :
				if ( ((angleAccBuf >= 0.0f) && (targetMotion.angleVelocity > currentMotion.angleVelocity)) ||
					 ((angleAccBuf <  0.0f) && (targetMotion.angleVelocity < currentMotion.angleVelocity))
					) {
					targetMotion.angleAcceleration = 0.0f;
					driveState = ST_DRV_TURN_END2;
				}
				break;
			case ST_DRV_TURN_END2 :
				if ((angleAccBuf >= 0.0f) && (targetMotion.angle < currentMotion.angle) ||
					(angleAccBuf <  0.0f) && (targetMotion.angle > currentMotion.angle)
					) {
					targetMotion.angleVelocity = 0.0f;
					idealAngleVelocity = 0.0f;
					StopMotorTimers();
					holdOffTimer = 0;
					driveState = ST_DRV_STOP;
				}
				break;

			case ST_DRV_PAUSE :
				
				/* 強制停止までそのまま */
				
				break;
		}
	}

}

float	GetMovedDistance(void)
{
	return currentMotion.distance;
}

void	ClearDistance(void)
{
	ClearEncoderCount();
	currentMotion.distance = 0.0f;
}

void	StopMotor(void) {
	stopMotorDrive = 1;
}

void	ClearMotorFail(void) {
	stopMotorDrive = 0;
}

u8 GetMotorStatus(void) {
	return stopMotorDrive;
}

void StartMotorDirectTestMode(void) {
	motorDirectTestMode = 1;
}

void StopMotorDirectTestMode(void) {
	motorDirectTestMode = 0;
}

void OutputMotorLogData(void) {
	
	u16 index;
	
	SciSendString("Motor Data Output... \n\r");
	
	for (index=0;index<NUMBER_OF_MOTOR_LOG_DATA;index++) {
		SciSendDec(5,index);	SciSendString(",");
		SciSendDec(5,motorLogData[index].data1);	SciSendString(",");
		SciSendDec(5,motorLogData[index].data2);	SciSendString(",");
		SciSendDec(5,motorLogData[index].data3);	SciSendString(",");
		SciSendDec(5,motorLogData[index].data4);	SciSendString(",");
		SciSendDec(5,motorLogData[index].data5);	SciSendString(",\n\r");
		TimerWait1ms(50);
	}
	
	motorLogIndex = 0;
	
	SciSendString("Fin. \n\r");
	
}

void ClearLogData(void) {
	u16 index;
	
	for (index=0;index<NUMBER_OF_MOTOR_LOG_DATA;index++) {
		motorLogData[index].data1 = 0;
		motorLogData[index].data2 = 0;
		motorLogData[index].data3 = 0;
		motorLogData[index].data4 = 0;
		motorLogData[index].data5 = 0;
		motorLogIndex = 0;
	}
}
