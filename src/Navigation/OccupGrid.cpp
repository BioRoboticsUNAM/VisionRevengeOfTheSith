#include "OccupGrid.hpp"

OccupGrid::OccupGrid()
{
	this->occupancyCount = 0;
	this->isOccupied = 0;
}

OccupGrid::~OccupGrid()
{
	if(this->occupancyCount != 0)
		delete this->occupancyCount;
	if(this->isOccupied != 0)
		delete this->isOccupied;
}

void OccupGrid::LoadParamsFromFile(std::string path)
{
	this->minX = -10;	//Min x-coordinate in meters
	this->minY = -10;
	this->maxX = 10;
	this->maxY = 20;
	this->cellSize = 0.05f;
	this->gridWidth = this->maxX - this->minX;
	this->gridHeight = this->maxY - this->minY;
	this->cellsInX = (int)(this->gridWidth/this->cellSize) + 1;
	this->cellsInY = (int)(this->gridHeight/this->cellSize) + 1;
	this->totalCells = this->cellsInX * this->cellsInY;
	this->occupancyCount = new int[this->totalCells];
	this->isOccupied = new bool[this->totalCells];
}

std::vector<int> OccupGrid::GetOccupiedGridIndices(cv::Mat& pointCloud, cv::Mat& outputImage, float robotX, float robotY)
{
	//It is assumed that values in "mat" are in coordinate w.r.t. world. 
	//And that output image is 3U8
	int matSize = pointCloud.cols * pointCloud.rows * pointCloud.channels(); //Size of all xyz points
	int pointSize = pointCloud.channels();  //Size of each XYZ point in bytes
	float* dataf = (float*)pointCloud.data;
	float x, y, z;
	std::map<int, bool> idxMap;
	int idx_i, idx_j, idx;
	float deltaX, deltaY;

	bool* tempOccupancy = new bool[this->totalCells];

	for(int i=0; i < this->totalCells; i++)
	{
		this->occupancyCount[i] = 0;
		this->isOccupied[i] = false;
		tempOccupancy[i] = false;
	}

	for(int i=0; i < matSize; i+= pointSize)
	{
		x = dataf[i];
		y = dataf[i+1];
		z = dataf[i+2];

		if(x == 0 && y == 0 && z == 0)
			continue;
		deltaX = x - robotX;
		deltaY = y - robotY;
		if((deltaX*deltaX + deltaY*deltaY) > 9.0f) //Far points are not taken into account
			continue;
		if(z < 0.05f)
		{
			outputImage.data[i] = 85;
			outputImage.data[i+1] = 170;
			outputImage.data[i+2] = 0;
		}
		else
		{
			idx_i = (int)((x - this->minX)/this->cellSize);
			idx_j = (int)((y - this->minY)/this->cellSize);
			idx = idx_j * this->cellsInX + idx_i;
			this->occupancyCount[idx]++;
			outputImage.data[i] = 85;
			outputImage.data[i+1] = 0;
			outputImage.data[i+2] = 170;
		}
	}

	std::vector<int> indices;
	for(int i=0; i < this->totalCells; i++)
		if(this->occupancyCount[i]>49)
		{
			this->isOccupied[i] = true;
			tempOccupancy[i] = true;
		}

	for(int j=1; j < this->cellsInY-1; j++)
		for(int i=1; i < this->cellsInX-1; i++)
		{
			int idx = j*this->cellsInX + i;
			bool up = tempOccupancy[idx - this->cellsInX];
			bool down = tempOccupancy[idx + this->cellsInX];
			bool left = tempOccupancy[idx - 1];
			bool right = tempOccupancy[idx + 1];
			bool upR = tempOccupancy[idx - this->cellsInX + 1];
			bool upL = tempOccupancy[idx - this->cellsInX - 1];
			bool downR = tempOccupancy[idx + this->cellsInX + 1];
			bool downL = tempOccupancy[idx + this->cellsInX - 1];
			//8-connectivity
			//If an cell has 8-connectivity, then is inside an occupied space and is not necessary for collision calculations
			if(up && down && left && right && upR && upL && downR && downL)
				this->isOccupied[idx] = false;
		}

	for(int i=0; i < this->totalCells; i++)
		if(this->isOccupied[i])
			indices.push_back(i);

	delete tempOccupancy;
	return indices;
}

std_msgs::Float32MultiArray OccupGrid::IndicesToFloatArrayMsg(std::vector<int>& indices)
{
	std_msgs::Float32MultiArray msgVcOccupied;

	msgVcOccupied.data.push_back(this->minX);
	msgVcOccupied.data.push_back(this->minY);
	msgVcOccupied.data.push_back(this->cellSize);
	msgVcOccupied.data.push_back(this->cellsInX);
	for(size_t i=0; i< indices.size(); i++)
		msgVcOccupied.data.push_back(indices[i]);

	return msgVcOccupied;
}

visualization_msgs::Marker OccupGrid::OccupPointsToMarker(std::vector<int>& indices)
{
	visualization_msgs::Marker mrkOccupied;
	geometry_msgs::Point p;
	p.z = 0;

	mrkOccupied.ns = "vc_occupied_space";
	mrkOccupied.header.frame_id = "map";
	mrkOccupied.type = visualization_msgs::Marker::CUBE_LIST;
	mrkOccupied.action = visualization_msgs::Marker::ADD;
	mrkOccupied.lifetime = ros::Duration();
	mrkOccupied.pose.orientation.x = 0;
	mrkOccupied.pose.orientation.y = 0;
	mrkOccupied.pose.orientation.z = 0;
	mrkOccupied.color.r = 1.0f;
	mrkOccupied.color.g = 0.0f;
	mrkOccupied.color.b = 1.0f;
	mrkOccupied.color.a = 1.0f;
	mrkOccupied.scale.x = this->cellSize;
	mrkOccupied.scale.y = this->cellSize;
	mrkOccupied.scale.z = 0.04;
	mrkOccupied.id = 0;

	for(size_t i=0; i< indices.size(); i++)
	{
		int idx = indices[i];
		p.x = (idx % this->cellsInX) * this->cellSize + this->minX;
		p.y = (idx / this->cellsInX) * this->cellSize + this->minY;
		mrkOccupied.points.push_back(p);
	}
	mrkOccupied.header.stamp = ros::Time::now();

	return mrkOccupied;
}
