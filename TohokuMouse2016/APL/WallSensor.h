#ifndef	__WALL_SENSOR_H__
#define	__WALL_SENSOR_H__


/* この値以上で壁ありと判定 */
#define	WALL_ON_SIDE_L		(500)
#define	WALL_ON_SIDE_R		(500)
#define	WALL_ON_FRONT_L		(500)
#define	WALL_ON_FRONT_R		(500)

/* マウスを迷路中央に置いた時の値 */
#define	WALL_CENTER_SIDE_L	(100)
#define	WALL_CENTER_SIDE_R	(100)
#define	WALL_CENTER_FRONT_L	(420)
#define	WALL_CENTER_FRONT_R	(310)

/* 前センサのこの値以上の場合は横センサで制御しない */
#define	SIDE_CTL_OFF_FRONT	(100)

extern	void WallSensorInit(void);
extern	s16 GetWallSensor(u8 pos);
extern	void ReadWallSensors(void);
extern	void WaitSensorHandStart(void);

#endif

