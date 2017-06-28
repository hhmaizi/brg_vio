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
#include <fstream>
#include <stdio.h>
// Include the ROS C++ APIs
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
// Includes detectFeature Class
#include "detectFeature.h"
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
// Initializes the detectFeature object detector for use in imageCallback and main
detectFeature detector;
// Set ground truth file
FILE *myfile_vicon;
// Set state estimate file
FILE *myfile;
/*---------------- End Classes -----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
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
		
		// Gets image from CV bridge and places it in "im1"
		Mat im1 = cv_bridge::toCvShare(msg,"bgr8")->image;

		// Initializes keypoints
		std::vector<cv::KeyPoint> keypoints_1;

		// Calls the detect function for the detector object and inputs the image and keypoints
		detector.detect(im1, keypoints_1);
		// detect() writes the detected keypoints for im1 into keypoints_1

		// Draws keypoints_1 onto im1
		drawKeypoints( im1, keypoints_1, im1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

		// Displays im1 with the detected keypoints
		cv::imshow("view", im1);
		cv::waitKey(30);

		// Save VIO state estimate to file
		double timestamp;
		timestamp = msg->header.stamp.toNSec()/1000000000.0;
		fprintf(myfile,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",timestamp,0.0,0.0,0.0,0.0,0.0,0.0,1.0);
		
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",msg->encoding.c_str());
	}

}

void fireCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	double timestamp;
	timestamp = msg->header.stamp.toNSec()/1000000000.0;
	fprintf(myfile_vicon,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",timestamp,msg->transform.translation.x,msg->transform.translation.y,msg->transform.translation.z,msg->transform.rotation.x,msg->transform.rotation.y,msg->transform.rotation.z,msg->transform.rotation.w);
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
    ros::NodeHandle vicon_receiver;

	// Open File to Save Results
	myfile = fopen("/home/dave/Desktop/vio_results.txt","w");
	myfile_vicon = fopen("/home/dave/Desktop/ground_truth.txt","w");

	// Start the node resource managers (communication, time, etc)
	ros::start();

	// Loads the parameters in featureDetector.yaml into the detectFeature object detector
	detector.loadParams();

	// Create an OpenCV display window
	cv::namedWindow("view");

	// Create an Image Transport instance
	cv::startWindowThread();

	// Subscribe to the ROS cam0/image_raw topic
	image_transport::ImageTransport it(sub_camera);
	image_transport::Subscriber sub = it.subscribe("cam0/image_raw", 1, imageCallback);

    // Save Vicon Data to file
    ros::Subscriber vic_sub = vicon_receiver.subscribe("/vicon/firefly_sbx/firefly_sbx",1000, fireCallback);

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
