#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "PlanarHorizontalSegment.hpp"

class DetectedObject
{	
public:
	DetectedObject();
	DetectedObject(std::vector< cv::Point2i > indexes, cv::Mat xyzPoints, cv::Mat oriMask, PlanarHorizontalSegment planarSeg );
	~DetectedObject();	

	std::vector< cv::Point2i > indexes; 
	cv::Mat xyzPoints; 
	cv::Mat oriMask; 
	PlanarHorizontalSegment planarSeg; 

	std::vector< cv::Point3f > pointCloud; 
	std::vector< cv::Point2f > xyPoints2D; 
	
	cv::Point3f centroid;

	cv::Rect boundBox; 
	double height; 

	cv::RotatedRect shadowOriBoundBoxt2D; 
	std::vector< cv::Point2f > shadowCHull; 
	std::vector< cv::Point2f > shadowContour2D; 
};  
