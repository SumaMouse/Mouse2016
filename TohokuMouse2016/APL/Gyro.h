#ifndef	__GYRO_H__
#define	__GYRO_H__

#define		GYRO_OUTPUT		(0)
#define		GYRO_OFFSET		(1)

extern void MPU6500Setup(void);
extern void UpdateGyroRef(void);
extern void ReadGyroSensor(void);
extern float GetGyroSensorRef(void);
extern float GetGyroSensorValue(void);
extern float GetGyroAngle(void);


#endif

