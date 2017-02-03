#pragma once
#include "robotcommandbase.h"
class robotCommandFaceCompass :
	public robotCommandBase
{
public:
	robotCommandFaceCompass(int _targetDegree, int targetThreshold, int& _compassSource, ofSerial& _ofSerial):robotCommandBase("0\r\n", _ofSerial),mTargetCompassDegree(_targetDegree),mCurrentCompassDegree(_compassSource),mTargetThreshold(targetThreshold),mInApproach(false){}
	virtual ~robotCommandFaceCompass(void){};

	bool onStart();
	bool onUpdate();
	void onEnd();

private:

	int mTargetThreshold;

	int diffToTarget(){return abs(mCurrentCompassDegree - mTargetCompassDegree);}
	int mTargetCompassDegree;
	const int& mCurrentCompassDegree;

	bool mInApproach;
};

