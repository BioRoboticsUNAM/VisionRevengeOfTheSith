#pragma once
#include <iostream>

class Head
{
public:
	Head();
	Head(float pan, float deltaPan, float tilt, float deltaTilt, float roll, float headZ);
	~Head();

	float pan;
	float deltaPan;
	float tilt;
	float deltaTilt;
	float roll;
	float headZ;
};