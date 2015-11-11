#include <iostream>
#include <map>
#include <math.h>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "visualization_msgs/Marker.h"

class OccupGrid
{
public:
	OccupGrid();
	~OccupGrid();

	float minX;
	float minY;
	float cellSize;
	float maxX;
	float maxY;
	int cellsInX;
	int cellsInY;
	int totalCells;
	float gridWidth;
	float gridHeight;
	void LoadParamsFromFile(std::string path);
	std::vector<int> GetOccupiedGridIndices(cv::Mat& pointCloud, cv::Mat& outputImage, float robotX, float robotY);
	std_msgs::Float32MultiArray IndicesToFloatArrayMsg(std::vector<int>& indices);
	visualization_msgs::Marker OccupPointsToMarker(std::vector<int>& indices);

private:
	int* occupancyCount;
	bool* isOccupied;
};
