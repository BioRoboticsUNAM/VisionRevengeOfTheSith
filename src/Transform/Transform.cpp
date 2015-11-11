#include "Transform.hpp"

void Transform::Kinect2Head(cv::Mat& mat, float pan, float tilt)
{
	int matSize = mat.cols * mat.rows * mat.channels(); //Size of all xyz points
	int pointSize = mat.channels();  //Size of each XYZ point in bytes
	float* dataf = (float*)mat.data;
	//Sign in tilt angle is negative because a positive rotation over Y axis is positive downwards, 
	//nevertheless, in robot Justina, a positive tilt goes upwards.
	//Kinect system is not dextrorotatory. A world-point (x,y,z) corresponds to (z, -x, y) in the kinect (openni) system.
	float x, y, z;
	float cosPan = cos(pan);
	float cosTilt = cos(-tilt);
	float sinPan = sin(pan);
	float sinTilt = sin(-tilt);
	float cosPanCosTilt = cosPan * cosTilt;
	float cosPanSinTilt = cosPan * sinTilt;
	float sinPanCosTilt = sinPan * cosTilt;
	float sinPanSinTilt = sinPan * sinTilt;
	for(int i=0; i < matSize; i+= pointSize)
	{
		x = dataf[i+2];
		y = -dataf[i];
		z = dataf[i+1];
		dataf[i] = cosPanCosTilt*x - sinPan*y + cosPanSinTilt*z;
		dataf[i+1] = sinPanCosTilt*x + cosPan*y + sinPanSinTilt*z;
		dataf[i+2] = -sinTilt*x + cosTilt*z;
	}
	mat.data = (uchar*)dataf;
}

void Transform::Head2Robot(cv::Mat& mat, float theta, float headZ)
{
	int matSize = mat.cols * mat.rows * mat.channels(); //Size of all xyz points
	int pointSize = mat.channels();  //Size of each XYZ point in bytes
	float* dataf = (float*)mat.data;

	float x, y, z;
	float cosTheta = cos(theta);
	float sinTheta = sin(theta);
	for(int i=0; i < matSize; i+= pointSize)
	{
		x = dataf[i];
		y = dataf[i + 1];
		z = dataf[i + 2];
		dataf[i] = cosTheta * x -  sinTheta * y;
		dataf[i+1] = sinTheta * x + cosTheta * y;
		dataf[i+2] = z + headZ;
	}
	mat.data = (uchar*)dataf;
}

void Transform::Kinect2Robot(cv::Mat& mat, float pan, float tilt, float roll, float headZ)
{
	return Kinect2World(mat, pan, tilt, roll, headZ, 0, 0, 0);
}

void Transform::Kinect2Robot(cv::Mat& mat, float pan, float tilt, float headZ)
{
	return Kinect2World(mat, pan, tilt, 0, headZ, 0, 0, 0);
}

void Transform::Robot2World(cv::Mat& mat, float robotX, float robotY, float robotTheta)
{
	int matSize = mat.cols * mat.rows * mat.channels(); //Size of all floating points
	int pointSize = mat.channels();  
	float* dataf = (float*)mat.data;
	
	float cosTheta = cos(robotTheta);
	float sinTheta = sin(robotTheta);

	float x,y,z;
	for(int i=0; i < matSize; i+= pointSize)
	{
		x = dataf[i];
		y = dataf[i + 1];
		z = dataf[i + 2];
		if(x != 0 || y!= 0 || z != 0)
		{
			dataf[i] =   robotX + x*cosTheta - y*sinTheta;
			dataf[i+1] = robotY + x*sinTheta + y*cosTheta;
		}
		else
		{
			dataf[i] = 0;
			dataf[i+1] = 0;
			dataf[i+2] = 0;
		}
	}
	mat.data = (uchar*)dataf;
}

void Transform::Kinect2World(cv::Mat& mat, float pan, float tilt, float roll, float headZ, float robotX, float robotY, float theta)
{
	int matSize = mat.cols * mat.rows * mat.channels(); //Size of all floating points
	int pointSize = mat.channels();  
	float* dataf = (float*)mat.data;
	//Sign in tilt angle is negative because a positive rotation over Y axis is positive downwards, 
	//nevertheless, in robot Justina, a positive tilt goes upwards.
	//Kinect system is not dextrorotatory. A world-point (x,y,z) corresponds to (z, -x, y) in the kinect (openni) system.
	float x, y, z;
	float cosPan = cos(pan + theta);
	float cosTilt = cos(-tilt);
	float sinPan = sin(pan + theta);
	float sinTilt = sin(-tilt);
	float cosRoll = cos(roll);
	float sinRoll = sin(roll);
	//First row
	float cPcTcR_m_sPsR = cosPan * cosTilt * cosRoll - sinPan*sinRoll;
	float _cPcTsR_m_sPcR = -cosPan * cosTilt * sinRoll - sinPan*cosRoll;
	float cosPanSinTilt = cosPan * sinTilt;
	//Second row
	float sPcTcR_p_cPsR = sinPan*cosTilt*cosRoll + cosPan*sinRoll;
	float _sPcTsR_p_cPcR = -sinPan*cosTilt*sinRoll + cosPan*cosRoll;
	float sinPanSinTilt = sinPan * sinTilt;
	//Third row
	float _sTcR = -sinTilt * cosRoll;
	float sTsR = sinTilt * sinRoll;
	for(int i=0; i < matSize; i+= pointSize)
	{
		x = dataf[i+2];
		y = -dataf[i];
		z = dataf[i+1];
		if(x != 0 || y!= 0 || z != 0)
		{
			dataf[i] = robotX + cPcTcR_m_sPsR*x + _cPcTsR_m_sPcR*y + cosPanSinTilt*z;
			dataf[i+1] = robotY + sPcTcR_p_cPsR*x + _sPcTsR_p_cPcR*y + sinPanSinTilt*z;
			dataf[i+2] = headZ + _sTcR*x + sTsR*y + cosTilt*z;
		}
		else
		{
			dataf[i] = 0;
			dataf[i+1] = 0;
			dataf[i+2] = 0;
		}
	}
	mat.data = (uchar*)dataf;
}

// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Transform::Cv2Pcl(cv::Mat& points, cv::Mat& colors)
// {
// 	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBA>);
// 	pc->width = points.cols;
// 	pc->height = points.rows;
// 	pc->points.resize(pc->width*pc->height);
// 	int matSize = points.cols * points.rows * points.channels(); //Size of all floating points
// 	int pointSize = points.channels();  
// 	float* dataf = (float*)points.data;
// 	float x, y, z;
// 	uchar r, g, b;
// 	for(int i=0; i < matSize; i+= pointSize)
// 	{
// 		x = dataf[i];
// 		y = dataf[i+1];
// 		z = dataf[i+2];
// 		b = colors.data[i];
// 		g = colors.data[i+1];
// 		r = colors.data[i+2];
// 		pc->points[i/pointSize].x = x;
// 		pc->points[i/pointSize].y = y;
// 		pc->points[i/pointSize].z = z;
// 		pc->points[i/pointSize].r = r;
// 		pc->points[i/pointSize].g = g;
// 		pc->points[i/pointSize].b = b;
// 		pc->points[i/pointSize].a = 255;
// 	}
// 	return pc;
// }
