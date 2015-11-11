#include "PlaneExtractor.hpp"

PlaneExtractor::PlaneExtractor()
{

}

std::vector<PlanarHorizontalSegment> PlaneExtractor::ExtractHorizontalPlanes(cv::Mat& pointCloud, cv::Mat ouputBGRImg)
{
	//Constanst for normal calculation
	int normalsBlurSize = 8; 
	float normalsZThreshold = 0.8f;

	// Plane segmentation RANSAC
	int minPointsForPlane = 15000; 
	double distThresholdForPlane =  0.01; 
	int maxIterations = 1000;

	std::vector<PlanarHorizontalSegment> horizontalPlanes;

	// Removing floor and far away points
	//"validPoints" is a mask
	cv::Mat validPoints, notValidPoints;
	cv::inRange( pointCloud , cv::Scalar( -2, -2, 0.10) , cv::Scalar( 2, 2, 1.5 ), validPoints); 

	//Set to zero all far and floor points
	//In to not taking them into accounto for ransac
	cv::bitwise_not( validPoints, notValidPoints ); 
	pointCloud.setTo( 0.0, notValidPoints); 

	cv::Mat normals;
	cv::Mat bluredPointCloud;
	cv::blur( pointCloud, bluredPointCloud, cv::Size(normalsBlurSize,normalsBlurSize));
	// Calculating Normals
	GetNormalsCross( bluredPointCloud , normals); 

	// Getting horizontal normals Mask only
	cv::Mat horNormalsMask; 
	cv::inRange( normals, cv::Scalar( -1, -1, normalsZThreshold ), cv::Scalar( 1, 1, 1 ), horNormalsMask); 				

	
	std::vector< cv::Point2i > horNormalsIndexes;
	if( !horNormalsMask.isContinuous() ){
		return horizontalPlanes; 
	}
	cv::Mat planesMask; 
	cv::findNonZero( horNormalsMask , horNormalsIndexes ); 
	horizontalPlanes = GetPlanesRANSAC( pointCloud, horNormalsIndexes, minPointsForPlane, distThresholdForPlane, maxIterations, planesMask); 

	for(int i=0; i< horizontalPlanes.size(); i++)
	{
		//cv::Scalar color = cv::Scalar( rand() % 256 , rand() % 256 , rand() % 256 ); 
		cv::Scalar color = cv::Scalar(196 , 127 , 127); 
		for(size_t j=0; j < horizontalPlanes[i].indices.size(); j++)
		{
			ouputBGRImg.at<cv::Vec3b>(horizontalPlanes[i].indices[j])[0] = color[0];
			ouputBGRImg.at<cv::Vec3b>(horizontalPlanes[i].indices[j])[1] = color[1];
			ouputBGRImg.at<cv::Vec3b>(horizontalPlanes[i].indices[j])[2] = color[2];
		}
	}
	return horizontalPlanes;
}

void PlaneExtractor::GetNormalsCross(cv::Mat& xyzPoints, cv::Mat& normals)
{
	
	normals = cv::Mat::zeros(xyzPoints.rows, xyzPoints.cols, CV_32FC3); 
	
	cv::Point3f topLeft; 
	cv::Point3f topRight; 
	cv::Point3f downLeft; 
	cv::Point3f downRight; 

	cv::Point3f normal_1; 
	cv::Point3f normal_2; 
	cv::Point3f normal; 
			
	cv::Point3f center;
	
	float norm = 0.0;

	for( int idxRow = 1 ; idxRow < xyzPoints.rows - 1 ; idxRow++ )
	{
		for( int idxCol = 1 ; idxCol < xyzPoints.cols - 1 ; idxCol++ )
		{			
			// Getting Vectors
			topLeft = xyzPoints.at<cv::Vec3f>(idxRow-1, idxCol-1);
			topRight = xyzPoints.at<cv::Vec3f>(idxRow-1, idxCol+1);
			downLeft = xyzPoints.at<cv::Vec3f>(idxRow+1, idxCol-1);
			downRight = xyzPoints.at<cv::Vec3f>(idxRow+1, idxCol+1);
			
			if( topLeft.x == 0.0 && topLeft.y == 0.0 && topLeft.z == 0.0 )
				continue; 
			if( topRight.x == 0.0 && topRight.y == 0.0 && topRight.z == 0.0 )
				continue; 
			if( downLeft.x == 0.0 && downLeft.y == 0.0 && downLeft.z == 0.0 )
				continue; 
			if( downRight.x == 0.0 && downRight.y == 0.0 && downRight.z == 0.0 )
				continue; 
			
			// Normals
			normal_1 = topRight - downLeft;
			normal_2 = topLeft - downRight; 

			// Normal by cross product (v x h)
			normal  = normal_2.cross(normal_1);

			// Proyecting all normals over XY plane
			if(  normal.z < 0 )
				normal = -normal;

			// Make normal unitary
			norm = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
						
			if(norm == 0.0f)
				continue; 

			// Asignign Value To normal
			normals.at<cv::Vec3f>(idxRow, idxCol) = ( 1.0f / norm ) * normal;
		}
	}
}

std::vector<PlanarHorizontalSegment> PlaneExtractor::GetPlanesRANSAC(cv::Mat pointCloud, std::vector<cv::Point2i> indexes, int minPointsForPlane, double distThresholdForPlane, int maxIterations, cv::Mat& out_planesMask)
{
	
	float distToPlane; 
	cv::Point3f xyzPoint; 
	double dist1;
	double dist2;
	double dist3;

	std::vector<PlanarHorizontalSegment> horizontalPlanes; 
	std::vector< std::vector< cv::Point2i > > indexesPlanes; 

	out_planesMask = cv::Mat::zeros( pointCloud.rows, pointCloud.cols, CV_8UC1); 

	int iterationsCnt = 0; 
	while( iterationsCnt++ < maxIterations && indexes.size() > minPointsForPlane )
	{
		// Getting Random points 
		cv::Point3f p1 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] );  
		cv::Point3f p2 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] ); 
		cv::Point3f p3 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] ); 

		// Checking that are valid points for plane, ej. not paralls
		if( !Plane3D::AreValidPointsForPlane( p1, p2, p3 ) )
			continue;
		
		// Checking not so close points
		dist1 = cv::norm(p1 - p2);
		dist2 = cv::norm(p1 - p3); 
		dist3 = cv::norm(p2 - p3); 
		if(dist1 < 0.2 || dist2 < 0.2 || dist3 < 0.2 )
			continue; 

		// Calculating candidate plane
		Plane3D candidatePlane( p1, p2, p3 ); 

		if( std::abs( candidatePlane.GetNormal().z ) < 0.99 ){			
			continue; 
		}

		// Checking distance to candidate plane of points
		std::vector< cv::Point3f > inliers; 
		inliers.reserve( indexes.size() ); 

		for(size_t i=0; i<indexes.size(); i++ ){
			
			xyzPoint = pointCloud.at<cv::Vec3f>(indexes[i]); 
			distToPlane = candidatePlane.DistanceToPoint( xyzPoint );
			
			if( distToPlane < distThresholdForPlane )
				inliers.push_back( xyzPoint ); 
		}

		// If there are few inliers discard
		if( inliers.size() < minPointsForPlane )
			continue; 

		// Getting a better plane using PCA analisys
		cv::PCA pca( cv::Mat(inliers).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW);

		// Creating new plane with a normal(eigenvector with lowest eigenvalue) and a point (mean)
		cv::Point3f pcaNormal(pca.eigenvectors.at<float>(2,0), pca.eigenvectors.at<float>(2,1), pca.eigenvectors.at<float>(2,2)); 
		cv::Point3f pcaPoint(pca.mean.at<float>(0,0), pca.mean.at<float>(0,1), pca.mean.at<float>(0,2)); 		
		
		Plane3D refinedPlane(pcaNormal, pcaPoint); 

		// Checking for new inliers
		
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point2i > newIndexes; 
		newIndexes.reserve( indexes.size() );
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point3f > newInliers; 
		newInliers.reserve( indexes.size() ); 
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point2f > pointsXY_forConvexHull; 
		pointsXY_forConvexHull.reserve( indexes.size() ); 
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point2i > indexPlane; 
		indexPlane.reserve( indexes.size() ); 

		for(size_t i=0; i<indexes.size(); i++ ){
			
			xyzPoint = pointCloud.at<cv::Vec3f>(indexes[i]); 
			distToPlane = refinedPlane.DistanceToPoint( xyzPoint );

			if( distToPlane < distThresholdForPlane ){
				newInliers.push_back( xyzPoint ); 
				pointsXY_forConvexHull.push_back( cv::Point2f( xyzPoint.x , xyzPoint.y ) );
				indexPlane.push_back( indexes[i] );

				out_planesMask.at<uchar>( indexes[i] ) = 255; 
			}
			else
				newIndexes.push_back(indexes[i]);
		}

		indexes = newIndexes; 
		indexesPlanes.push_back( indexPlane ); 

		// Getting Convex Hull ( Convex hull is valid if remove Z, because is an horizontal plane ) 
		std::vector< cv::Point2f > convexHull2D; 
		//cv::convexHull( pointsXY_forConvexHull , convexHull2D); 		
		// Creating Horizontal Planar Segment
		PlanarHorizontalSegment ps( newInliers, refinedPlane, pca, convexHull2D, indexPlane); 
		// Adding to vector to return
		horizontalPlanes.push_back( ps );
	}

	return horizontalPlanes; 
}