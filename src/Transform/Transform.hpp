#pragma once
#include <iostream>
#include <math.h>
#include <cmath>
#include <opencv2/core/core.hpp>
//#include <pcl/visualization/cloud_viewer.h>

class Transform
{
public:
	static void Kinect2Head(cv::Mat& mat, float pan, float tilt);
	static void Head2Robot(cv::Mat& mat, float theta, float headZ);
	static void Kinect2Robot(cv::Mat& mat, float pan, float tilt, float roll, float headZ);
	static void Kinect2Robot(cv::Mat& mat, float pan, float tilt, float headZ);
	static void Robot2World(cv::Mat& mat, float robotX, float robotY, float robotTheta);
	static void Kinect2World(cv::Mat& mat, float pan, float tilt, float roll, float headZ, float robotX, float robotY, float theta);
	//static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cv2Pcl(cv::Mat& points, cv::Mat& colors);
};
