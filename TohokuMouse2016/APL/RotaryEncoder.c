#include "common.h"

#include "MousePrameter.h"
#include "AS5055.h"

static s32 leftEncoderCount = 0;
static s32 rightEncoderCount = 0;

static float leftDistance = 0.0f;
static float rightDistance = 0.0f;

static u16 leftPos = 0;
static u16 rightPos = 0;
static u16 leftErr = 0;
static u16 rightErr = 0;
static u16 oldLeftPos = 0;
static u16 oldRightPos = 0;

u16 dbgREncPos = 0;
u16 dbgLEncPos = 0;

float dbgLeftTotalDistance = 0.0f;
float dbgRightTotalDistance = 0.0f;

void ReadRotaryEncoder(void) {
	
	s32 diff;
	
	AS5055ReadAngularData(0, &rightPos);
	AS5055ReadErrorStatus(0, &rightErr);

	dbgREncPos = rightPos;

	AS5055ReadAngularData(1, &leftPos);
	AS5055ReadErrorStatus(1, &leftErr);

	dbgLEncPos = leftPos;

	diff = (s32)(oldRightPos - rightPos);
	if (diff >  2048) diff -= 4096;
	if (diff < -2048) diff += 4096;

	rightEncoderCount += diff;
	oldRightPos	= rightPos;

	rightDistance = (DISTANCE_OF_1PULSE * (float)diff) * 1000.0f;

	dbgRightTotalDistance += rightDistance;


	diff = (s32)(leftPos - oldLeftPos);
	if (diff >  2048) diff -= 4096;
	if (diff < -2048) diff += 4096;

	leftEncoderCount += diff;
	oldLeftPos	= leftPos;
	
	leftDistance = (DISTANCE_OF_1PULSE * (float)diff) * 1000.0f;

	dbgLeftTotalDistance += leftDistance;
}

void GetEncoderRawPosition(u16* lPos, u16* rPos) {
	*lPos	= leftPos;
	*rPos	= rightPos;
}

void GetEncoderRawErr(u16* lErr, u16* rErr) {
	*lErr = leftErr;
	*rErr = rightErr;
}

void GetEncoderCount(s32* leftCount, s32* rightCount) {
	*leftCount	= leftEncoderCount;
	*rightCount	= rightEncoderCount;
}

void GetDistance(float* lDistance, float* rDistance) {
	*lDistance = leftDistance;
	*rDistance = rightDistance;
}

void ClearEncoderCount(void) {
	leftEncoderCount = 0;
	rightEncoderCount = 0;
}

void GetDistanceForTest(float* lDistance, float* rDistance) {
	*lDistance = dbgLeftTotalDistance;
	*rDistance = dbgRightTotalDistance;
}


