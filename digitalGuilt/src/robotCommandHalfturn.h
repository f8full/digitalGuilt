#pragma once
#include "robotCommandFaceCompass.h"
class robotCommandHalfturn :
	public robotCommandFaceCompass
{
public:
	robotCommandHalfturn(int targetThreshold, int& _compassSource, ofSerial& _ofSerial)
		:robotCommandFaceCompass((_compassSource-180) + (360 * (_compassSource > 180 ? 0 : 1)), targetThreshold, _compassSource, _ofSerial){}
	
	~robotCommandHalfturn(void){};
};

