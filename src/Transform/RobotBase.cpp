#include "RobotBase.hpp"

RobotBase::RobotBase()
{
	this->X =0;
	this->Y = 0;
	this->Theta = 0;
}

RobotBase::RobotBase(float x, float y, float theta)
{
	this->X = x;
	this->Y = y;
	this->Theta = theta;
}

RobotBase::~RobotBase()
{
}