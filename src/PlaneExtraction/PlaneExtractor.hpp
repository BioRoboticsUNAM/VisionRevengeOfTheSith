#include <iostream>
#include <opencv2/core/core.hpp>
#include "../Utils/Plane3D.hpp"
#include "../Utils/PlanarHorizontalSegment.hpp"

class PlaneExtractor
{
public:
	PlaneExtractor();
	~PlaneExtractor();

	static std::vector<PlanarHorizontalSegment> ExtractHorizontalPlanes(cv::Mat& pointCloud, cv::Mat ouputBGRImg);
	static void GetNormalsCross(cv::Mat& xyzPoints, cv::Mat& normals);
	static std::vector<PlanarHorizontalSegment> GetPlanesRANSAC(cv::Mat pointCloud, std::vector<cv::Point2i> indexes, int minPointsForPlane, double distThresholdForPlane, int maxIterations, cv::Mat& out_planesMask);
};