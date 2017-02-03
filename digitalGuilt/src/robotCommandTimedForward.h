#pragma once
#include "robotcommandbase.h"
#include "robotCommandForward.h"

class robotCommandTimedForward :
	public robotCommandForward{
	
private:
	float mStartTime;
	float mDelayMs;
	
public:
	robotCommandTimedForward(ofSerial& _ofSerial, float _timeMs):robotCommandForward(_ofSerial),mDelayMs(_timeMs){}
	~robotCommandTimedForward(void){}

	bool onStart();
	bool onUpdate();
	void onEnd();
};

