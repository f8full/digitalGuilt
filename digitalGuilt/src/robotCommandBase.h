#pragma once
#include "ofMain.h"

//Used to manage a full command lifecycle destined to the robot
class robotCommandBase
{
public:
	

	virtual bool onStart(){ send(); return false; }	//By default tasks resolves themselves at start and returns false
	virtual bool onUpdate(){ return false;}	//false when done
	virtual void onEnd(){}
	virtual ~robotCommandBase(void){ delete mCommandSerialChars;};

protected:
	robotCommandBase(string _commandString, ofSerial& _ofSerial):mDataLink(_ofSerial){

		mCommandLength = _commandString.size();

		mCommandSerialChars = new unsigned char[mCommandLength];

		int i=0;

		for(string::iterator itt = _commandString.begin(); itt != _commandString.end(); ++itt)
		{
			mCommandSerialChars[i] = *itt;

			++i;
		}
	}

	void send(){ mDataLink.writeBytes(mCommandSerialChars,mCommandLength);}

	ofSerial& mDataLink;

private:
	unsigned char* mCommandSerialChars;
	int mCommandLength;
};

