#pragma once
#include "robotcommandbase.h"
class robotCommandTurnOnSpotLeft :
	public robotCommandBase
{
public:
	robotCommandTurnOnSpotLeft(ofSerial& _ofSerial):robotCommandBase("2\r\n",_ofSerial){}
	~robotCommandTurnOnSpotLeft(void){};
};

