#include "robotCommandTimedForward.h"
#include "robotCommandStop.h"
#include "ofMain.h"


bool robotCommandTimedForward::onStart()
{
	mStartTime = ofGetElapsedTimef();

	robotCommandForward::onStart();

	return true;
}

bool robotCommandTimedForward::onUpdate()
{
	float time = ofGetElapsedTimef() - mStartTime;
	time *= 1000.0;         
	if(time >= mDelayMs)
	{
		return false;
	}

	return true;	//no done
}

void robotCommandTimedForward::onEnd()
{
	robotCommandStop stopCommand = robotCommandStop(mDataLink);

	stopCommand.onStart();
}
