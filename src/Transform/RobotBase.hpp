#pragma once
#include <iostream>

class RobotBase
{
public:
	RobotBase();
	RobotBase(float x, float y, float theta);
	~RobotBase();
	float X;
	float Y;
	float Theta;
};