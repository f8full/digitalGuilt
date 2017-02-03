#pragma once
#include "robotcommandbase.h"
class robotCommandForward :
	public robotCommandBase
{
public:
	robotCommandForward(ofSerial& _ofSerial):robotCommandBase("1\r\n", _ofSerial){}
	virtual ~robotCommandForward(void){}
};

