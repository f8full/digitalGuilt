#pragma once
#include "robotcommandbase.h"
class robotCommandStop :
	public robotCommandBase
{
public:
	robotCommandStop(ofSerial& _ofSerial):robotCommandBase("0\r\n", _ofSerial){}
	~robotCommandStop(void){};
};

