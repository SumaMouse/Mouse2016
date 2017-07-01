#ifndef	__MOUSE_PARAMTER_H__
#define	__MOUSE_PARAMTER_H__

/* 車体のパラメータ */
#define		DIAMETER_OF_WHEEL		(0.0163f)					// m
#define		RADIUS_OF_WHEEL			(DIAMETER_OF_WHEEL/2.0f)	// m
#define		WEIGHT_OF_MOUSE			(24.0f)						// g
#define		MOMENT_OF_INERTIA		(0.0000192f)				// kg・m2
#define		TREAD					(0.037f)					// m
#define		RATIO_OF_GEAR			(5.6666f)
#define		ANGLE_PER_1PULSE		(0.087891f)			//deg

#define		CONSTANT_OF_MOTOR_CURRENT	(0.855617f)		// mNm/A
#define		CONSTANT_OF_MOTOR_VOLTAGE	(0.0896f)		// mV/rpm
#define		REGISTANCE_OF_MOTOR_COIL	(4500.0f)		// mΩ

#define		DISTANCE_OF_1PULSE		((DIAMETER_OF_WHEEL*PI)*(ANGLE_PER_1PULSE/360.0f))

#endif

