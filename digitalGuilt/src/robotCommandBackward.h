#pragma once
#include "robotcommandbase.h"
class robotCommandBackward :
	public robotCommandBase
{
public:
	robotCommandBackward(ofSerial& _ofSerial):robotCommandBase("5\r\n", _ofSerial){}
	virtual ~robotCommandBackward(void){}
};

