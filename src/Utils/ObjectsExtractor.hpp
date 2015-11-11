#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../Transform/Transform.hpp"
#include "Plane3D.hpp"
#include "PlanarHorizontalSegment.hpp"
#include "DetectedObject.hpp"
#include <queue>

class ObjectsExtractor
{
public: 	
	ObjectsExtractor();
	~ObjectsExtractor();

	bool Debug;
	std::vector<DetectedObject> ExtractObjectsHorizantalPlanes(
		cv::Mat rgbImage, cv::Mat pointsCloud, cv::Mat& outObjectsMask, float pan, float tilt, float headZ ); 
	
	void GetNormalsCross(cv::Mat& xyzPoints, cv::Mat& normals); 

	std::vector<PlanarHorizontalSegment> GetPlanesRANSAC(
		cv::Mat pointCloud, cv::Mat mask, int minPointsForPlane, double distThresholdForPlane, int maxIterations ); 
	//std::vector<PlanarHorizontalSegment> GetPlanesRANSAC_2(
	//cv::Mat pointCloud, std::vector<cv::Point2i> indexes, int minPointsForPlane, double distThresholdForPlane, int m    axIterations, const cv::Mat& out_planesMask = cv::Mat()); 	

	std::vector<PlanarHorizontalSegment> GetPlanesRANSAC_2(
		cv::Mat pointCloud, std::vector<cv::Point2i> indexes, int minPointsForPlane, 
		double distThresholdForPlane, int maxIterations, cv::Mat& out_planesMask);

	//std::vector<PlanarHorizontalSegment> GetPlanesRANSAC_2(cv::Mat pointCloud, std::vector<cv::Point2i> indexes, int minPointsForPlane, double distThresholdForPlane, int maxIterations, cv::Mat& out_planesMask = cv::Mat()); 

	void ConnectedComponents3D(
		cv::Mat pointCloud, cv::Mat mask, double minDist, cv::Mat labelsMask, 
		std::vector<std::vector<cv::Point2i> >& objIndexes, std::vector<cv::Mat>& objectsMask); 
}; 
