#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "vision/vsn_findonplanes.h"
#include "Transform/Transform.hpp"
#include "Navigation/OccupGrid.hpp"
#include "Transform/Head.hpp"
#include "Transform/RobotBase.hpp"
#include "Utils/ObjectsExtractor.hpp"
#include "PlaneExtraction/PlaneExtractor.hpp"

std::string nodePrompt = "VISION_NODE.->";
//Head head(0, 0, 0, -0.05f, -0.063f, 1.49f);
Head head(0, 0, 0, -0.05f, -0.01f, 1.46f);
RobotBase robot(0,0,0);

cv::Mat originalXYZ;
cv::Mat originalBGR;

void callback_mp_odometryPos(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	if(msg->data.size() != 3) return;
	robot.X = msg->data[0];
	robot.Y = msg->data[1];
	robot.Theta = msg->data[2];
	//std::cout << nodePrompt << "Received odometry: " << robot.X << "  " << robot.Y << "  " << robot.Theta << std::endl;
}

void callback_hd_pos(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	if(msg->data.size() != 2) return;
	head.pan = msg->data[0];
	head.tilt = msg->data[1];
	//std::cout << nodePrompt << "Received headPos: " << head.pan << "  " << head.tilt << std::endl;
}

bool callback_findonplanes(vision::vsn_findonplanes::Request &req, vision::vsn_findonplanes::Response &res)
{
	ObjectsExtractor objExt = ObjectsExtractor();
	objExt.Debug = false;
	cv::Mat detectedObj;
	std::vector<DetectedObject> detObj = objExt.ExtractObjectsHorizantalPlanes(originalBGR, originalXYZ, detectedObj, head.pan, head.tilt, head.headZ);

	visualization_msgs::Marker mrk;
	for(int i=0; i < detObj.size(); i++)
	{
		mrk.pose.position.x = detObj[i].centroid.x;
		mrk.pose.position.y = detObj[i].centroid.y;
		mrk.pose.position.z = detObj[i].centroid.z;
		mrk.ns = "unknown" + i;
		cv::rectangle(originalBGR, detObj[i].boundBox, cv::Scalar(0,0,255), 2);
		res.recognized.markers.push_back(mrk);
	}

	cv::imshow("RECOG_OBJECTS", originalBGR);
	return detObj.size() > 0;
}

int main(int argc, char** argv)
{
	OccupGrid occupGrid;
	occupGrid.LoadParamsFromFile("asdfsad");  

	std::cout << nodePrompt << "INITIALIZING VISION THE REVENGE OF THE SIFT ..." << std::endl;
	std::cout << nodePrompt << "Trying to connect with roscore... " << std::endl;
	ros::init(argc, argv, "vision");
	ros::NodeHandle n;
	ros::Subscriber subOdometryPos = n.subscribe("mp_odometryPos", 1, callback_mp_odometryPos);
	ros::Subscriber subHdPos = n.subscribe("hd_pos", 1, callback_hd_pos);
	ros::Publisher pubVcOccupied = n.advertise<std_msgs::Float32MultiArray>("vc_occupied", 1);
	ros::Publisher pubVcRecoObject = n.advertise<std_msgs::String>("vc_reco_obstacles", 1);
	ros::Publisher pubVcOccupiedMarker = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
	ros::ServiceServer srvFindOnPlanes = n.advertiseService("vsn_findonplanes", callback_findonplanes);
	std::cout << nodePrompt << "Connected with roscore :D" << std::endl;

	std::cout << nodePrompt << "Triying to initialize kinect sensor... " << std::endl;
	cv::VideoCapture capture(CV_CAP_OPENNI);
	if(!capture.isOpened())
	{
		std::cout << nodePrompt << "Cannot open kinect :'(" << std::endl;
		return 1;
	}
	capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
	std::cout << nodePrompt << "Kinect sensor started :D" << std::endl;
	std::cout << nodePrompt << "SYSTEM READY (I think so)" << std::endl;
	
	//ros::Rate loop_rate(10);
	long initTime = cv::getTickCount();
	long finalTime = cv::getTickCount();

	std_msgs::Float32MultiArray msgVcOccupied;
	visualization_msgs::Marker mrkOccupied;
	
	char c = 0;
	char cmd = 0;
	while(ros::ok() && c != 27)
	{
		c = cv::waitKey(15);
		if(c >= '0' && c <= '9') cmd = c;
		//Point cloud acquisition
		cv::Mat depthMap;
		cv::Mat bgrImage;
		capture.grab();
		capture.retrieve(depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
		capture.retrieve(bgrImage, CV_CAP_OPENNI_BGR_IMAGE);
		originalXYZ = depthMap.clone();
		originalBGR = bgrImage.clone();
		cv::Mat bgrOccupied = bgrImage.clone();
		cv::Mat bgrPlanes = bgrImage.clone();

		//Plane extraction
		Transform::Kinect2Robot(depthMap, head.pan, head.tilt, head.roll, head.headZ);
		PlaneExtractor::ExtractHorizontalPlanes(depthMap, bgrPlanes);

		//Separating free and occupied space
		Transform::Robot2World(depthMap, robot.X, robot.Y, robot.Theta);
		//Transform::Kinect2World(depthMap, head.pan, head.tilt, head.roll, head.headZ, robot.X, robot.Y, robot.Theta);
		std::vector<int> occupiedIdx = occupGrid.GetOccupiedGridIndices(depthMap, bgrOccupied, robot.X, robot.Y);
		msgVcOccupied = occupGrid.IndicesToFloatArrayMsg(occupiedIdx);
		mrkOccupied = occupGrid.OccupPointsToMarker(occupiedIdx);
		pubVcOccupied.publish(msgVcOccupied);
		pubVcOccupiedMarker.publish(mrkOccupied);

		//Displaying results
		if(cmd == '1')
			cv::imshow("VISUAL-CORTEX", bgrImage);
		else if(cmd == '2')
		 	cv::imshow("VISUAL-CORTEX", bgrPlanes);
		else if(cmd == '3')
		 	cv::imshow("VISUAL-CORTEX", bgrOccupied);		
		else
			cv::imshow("VISUAL-CORTEX", bgrImage);

		finalTime = cv::getTickCount();
		double t = (finalTime - initTime)/cv::getTickFrequency();
		initTime = finalTime;
		std::cout << "Elapsed time: " << t << std::endl;
		ros::spinOnce();
		//loop_rate.sleep();
	}

	return 0;
}
