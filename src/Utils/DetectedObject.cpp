#include "DetectedObject.hpp"

DetectedObject::DetectedObject()
{
}

DetectedObject::DetectedObject(std::vector< cv::Point2i > indexes, cv::Mat xyzPoints, cv::Mat oriMask, PlanarHorizontalSegment planarSeg)
{
	this->indexes = indexes; 
	this->xyzPoints = xyzPoints; 
	this->oriMask = oriMask; 
	this->planarSeg = planarSeg;
	this->height = 0;

	this->centroid = cv::Point3f(0.0,0.0,0.0); 
	for( int i=0; i<indexes.size(); i++){
			
		cv::Point3f pXYZ = xyzPoints.at<cv::Vec3f>( indexes[i] ); 
			
		this->pointCloud.push_back( pXYZ ); 
		this->xyPoints2D.push_back( cv::Point2f(pXYZ.x, pXYZ.y) );

		this->centroid = centroid + pXYZ;

		double distToPlane = this->planarSeg.DistanceToPlane( pXYZ ); 
		if( distToPlane > this->height )
			this->height = distToPlane; 
	}

	this->centroid = centroid * (1.0 / (float)indexes.size() );
	this->boundBox = cv::boundingRect( this->indexes ); 
		
	this->shadowOriBoundBoxt2D = cv::minAreaRect( xyPoints2D );
	cv::convexHull( xyPoints2D, this->shadowCHull ); 
	cv::approxPolyDP( this->shadowCHull, this->shadowContour2D, 0.001, true);
}

DetectedObject::~DetectedObject()
{
}
