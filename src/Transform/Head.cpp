#include "Head.hpp"

Head::Head()
{
	this->pan = 0;
	this->deltaPan = 0;
	this->tilt = 0;
	this->deltaTilt = 0;
	this->roll = 0;
	this->headZ = 0;
}

Head::Head(float pan, float deltaPan, float tilt, float deltaTilt, float roll, float headZ)
{
	this->pan = pan;
	this->deltaPan = deltaPan;
	this->tilt = tilt;
	this->deltaTilt = deltaTilt;
	this->roll = roll;
	this->headZ = headZ;
}

Head::~Head()
{
}