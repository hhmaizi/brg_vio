//
//	brg_vio
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	brg_vio.cpp
//	Entry point to the Bretl Group Visual-Inertial Odometry ROS Package.
//
//	Options:
//
//
//	Usage:
//		
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include <iostream>
#include <stdio.h>
// Include the ROS C++ APIs
#include "ros/ros.h"
// Everything we need to publish and subscribe to images
#include <image_transport/image_transport.h>	
// Headers allow us to display images using OpenCV's simple GUI abilities
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
/*---------------- End Classes -----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
/*--------------- End Namespaces ---------------*/

/*------------------ Pragmas -------------------*/
/*---------------- End Pragmas -----------------*/

/*------------- Function Prototypes ------------*/
/*----------- End Function Prototypes ----------*/
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Preamble ---------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Helpers ------------------------------------*/
/*-----------------------------------------------------------------------------*/
// This callback function will get called when a new image has arrived
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert ROS image message to an OpenCV image with BGR pixel encoding. 
	// Then show it in a display window.
	try
	{
		cv::imshow("view",cv_bridge::toCvShare(msg,"bgr8")->image);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",msg->encoding.c_str());
	}
}
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Helpers ----------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*----------------------------------- Main ------------------------------------*/
/*-----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
	// Annouce this program to the ROS master as a "node" called "vio_main"
	ros::init(argc, argv, "brg_vio");

	// Define ROS nodes
	ros::NodeHandle sub_camera;

	// Start the node resource managers (communication, time, etc)
	ros::start();

	// Create an OpenCV display window
	cv::namedWindow("view");

	// Create an Image Transport instance
	cv::startWindowThread();

	// Subscribe to the ROS cam0/image_raw topic
	image_transport::ImageTransport it(sub_camera);
	image_transport::Subscriber sub = it.subscribe("cam0/image_raw", 1, imageCallback);

	// Broadcast a simple log message
	ROS_INFO_STREAM("Hello, world!");

	//Process ROS callbacks until receiving a SIGINT (ctrl-c)
	ros::spin();

	// Close the OpenCV window
	cv::destroyWindow("view");

	// Stop the node's resources
	ros::shutdown();

	return 0;
}
/*-----------------------------------------------------------------------------*/
/*--------------------------------- End Main ----------------------------------*/
/*-----------------------------------------------------------------------------*/
