#ifndef	__ROTARY_ENCODER_H__
#define	__ROTARY_ENCODER_H__

#define	DISTANCE_OF_1PULSE		((DIAMETER_OF_WHEEL*PI)*(ANGLE_PER_1PULSE/360.0f))

extern void ReadRotaryEncoder(void);
extern void GetEncoderRawPosition(u16* leftPos, u16* rightPos);
extern void GetEncoderRawErr(u16* lErr, u16* rErr);
extern void GetEncoderCount(s32* leftCount, s32* rightCount);
extern void GetDistance(float* lDistance, float* rDistance);
extern void ClearEncoderCount(void);
extern void GetDistanceForTest(float* lDistance, float* rDistance);

#endif

