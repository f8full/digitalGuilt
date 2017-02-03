#pragma once
#include "robotcommandbase.h"
class robotCommandApproachSpeed :
	public robotCommandBase
{
public:
	robotCommandApproachSpeed(ofSerial& _ofSerial):robotCommandBase("4\r\n", _ofSerial){}
	~robotCommandApproachSpeed(void){};
};

