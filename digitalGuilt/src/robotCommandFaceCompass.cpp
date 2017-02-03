#include "robotCommandFaceCompass.h"
#include "ofMain.h"
#include "robotCommandStop.h"
#include "robotCommandTurnOnSpotLeft.h"
#include "robotCommandApproachSpeed.h"


bool robotCommandFaceCompass::onStart()
{
	//TODO : have a brilliant way of doing a dot product to know which rotation is closest
	if (!diffToTarget() < mTargetThreshold)
	{
		robotCommandTurnOnSpotLeft turnCommand = robotCommandTurnOnSpotLeft(mDataLink);

		turnCommand.onStart();

		return false;
	}
	

	return true;
}

bool robotCommandFaceCompass::onUpdate()
{
	int remaining = diffToTarget();

	cout << "trying to reach " << mTargetCompassDegree
		<< " current is " << mCurrentCompassDegree
		<< " difference " << remaining << endl;

	if (remaining < mTargetThreshold)
	{
		return false;

	}
	else if (!mInApproach && remaining < mTargetThreshold*10)
	{
		cout << "Applying approach speed" << endl;
		robotCommandApproachSpeed(mDataLink).onStart();
		mInApproach = true;
	}

	return true;	//no done
}

void robotCommandFaceCompass::onEnd()
{
	robotCommandStop stopCommand = robotCommandStop(mDataLink);

	stopCommand.onStart();
}
