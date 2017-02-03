#pragma once
#include "robotcommandbase.h"
class robotCommandTurnOnSpotRight :
	public robotCommandBase
{
public:
	robotCommandTurnOnSpotRight(ofSerial& _ofSerial):robotCommandBase("3\r\n", _ofSerial){}
	~robotCommandTurnOnSpotRight(void){};
};

