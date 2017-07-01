#ifndef	__MOTOR_H__
#define	__MOTOR_H__

#define		STOP_SPD			(0.02f)
#define		WALL_ADJ_STOP_SPD	(0.1f)		//���̑��x�ȉ��̏ꍇ�͕ǐ�������Ȃ�

#define		LIMIT_OF_DUTY_HIGH				(0.4f)
#define		LIMIT_OF_DUTY_LOW				(0.04f)


/* �����̑��x����p�萔 */
#define		STRIGHT_VELOCITY_PID_P_GAIN		(50.0f)
#define		STRIGHT_VELOCITY_PID_I_GAIN		(1.0f)
#define		STRIGHT_VELOCITY_PID_D_GAIN		(1.0f)
#define		STRIGHT_VELOCITY_PID_I_GAIN_MAX	(50.0f)
#define		STRIGHT_VELOCITY_FF_GAIN		(2.0f);

/* �p���x����p�萔(���M�n����) */
#define		ANGLE_V_PID_P_GAIN				(80.0f)
#define		ANGLE_V_PID_I_GAIN				(1.0f)
#define		ANGLE_V_PID_D_GAIN				(3.0f)
#define		ANGLE_V_PID_I_GAIN_MAX			(50.0f)
#define		ANGLE_V_FF_GAIN					(2.0f)

/* �p���x����p�萔(�X�����[��) */
#define		ANGLE_V_PID_P_GAIN_S			(220.0f)
#define		ANGLE_V_PID_I_GAIN_S			(5.0f)
#define		ANGLE_V_PID_D_GAIN_S			(0.0f)
#define		ANGLE_V_PID_I_GAIN_MAX_S		(100.0f)
#define		ANGLE_V_FF_GAIN_S				(0.7f)

/* �p�x����p�萔 */
#define		ANGLE_PID_P_GAIN				(80.0f)
#define		ANGLE_PID_I_GAIN				(1.0f)
#define		ANGLE_PID_D_GAIN				(0.0f)
#define		ANGLE_PID_I_GAIN_MAX			(40.0f)

/* �ǐ���p�萔 */
#define		WALL_PID_P_GAIN					(2.2f)	//2.0f
#define		WALL_PID_I_GAIN					(0.05f)
#define		WALL_PID_D_GAIN					(0.0f)
#define		WALL_PID_GAIN_MAX				(2.0f)


#define		MOTOR_HOLD_TIME					(500)		/* ���[�^�[��~��̃u���[�L����[ms] */


#define		CONTROL_MODE_NONE			(0)
#define		CONTROL_MODE_STRAIGHT		(1)
#define		CONTROL_MODE_SLANT			(2)


enum {
	DRV_MODE_STOP = 0,
	DRV_MODE_STRAIGHT,
	DRV_MODE_SLANT,
	DRV_MODE_TURN_R,	//���M�n����
	DRV_MODE_TURN_L,	//���M�n����
	DRV_MODE_R45SLA,
	DRV_MODE_L45SLA,
	DRV_MODE_R90SLA,
	DRV_MODE_L90SLA,
	DRV_MODE_R135SLA,
	DRV_MODE_L135SLA,
	DRV_MODE_R180SLA,
	DRV_MODE_L180SLA,
	DRV_MODE_RN90SLA,	//�΂߂Ǝ΂߂�����90���^�[��
	DRV_MODE_LN90SLA,	//�΂߂Ǝ΂߂�����90���^�[��
	DRV_MODE_RS90SLA,	//�T�����s�p�X�����[���^�[��
	DRV_MODE_LS90SLA,	//�T�����s�p�X�����[���^�[��
	DRV_MODE_PAUSE
};


void InitializeMotorControl(void);
void MotorControlCyclicTask(void);
void SetMoveCommand(u8 drvMode, float maxSpeed, float endSpeed, float maxAcc, float distance, float angleAcc, float angleSpeed, float angle);
u8	IsDrvComExecuteBusy(void);
u8	IsDrvComBufferFull(void);
float GetMovedDistance(void);
void ClearDistance(void);
void OutputMotorLogData(void);
void StopMotor(void);
void ClearMotorFail(void);
u8 GetMotorStatus(void);
u8 IsMoveComSetOk(void);
void WallControl(u8 mode);
void ClearLogData(void);

void StartMotorDirectTestMode(void);
void StopMotorDirectTestMode(void);

#endif

