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
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
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
void drawAgastFD(Mat& im1){

	try
	{
		// Detecting keypoints using Agast Feature Detector

		// Agast Parameters
		int Threshold = 10;
		bool nonmaxSuppression = true;
		int type = AgastFeatureDetector::OAST_9_16;

		// Initialize Algorithm

		Ptr<AgastFeatureDetector> detector = AgastFeatureDetector::create(Threshold, nonmaxSuppression, type);

		vector<KeyPoint> keypoints_1;

		detector->detect(im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN AgastFeatureDetector");
	}
}

void drawAKAZE(Mat& im1){

	try
	{
		// Detecting keypoints using AKAZE

		// AKAZE Parameters
		int descriptor_type = AKAZE::DESCRIPTOR_MLDB;
		int descriptor_size = 0;
		int descriptor_channels = 3;
		float threshold = 0.001f;
		int nOctaves = 4;
		int nOctaveLayers = 4;
		int diffusivity = KAZE::DIFF_PM_G2;

		// Initialize Algorithm

		Ptr<AKAZE> detector =AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, threshold, nOctaves, nOctaveLayers, diffusivity);

		vector<KeyPoint> keypoints_1;

		detector->detect(im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN AKAZE");
	}
}

void drawBRISK(Mat& im1){

	try
	{
		// Detecting keypoints using BRISK

		// BRISK Parameters
		int Thresh1 = 60;
		int Octaves = 4;
		float PatternScales = 1.0f;

		// Initialize Algorithm

		Ptr<BRISK> detector = BRISK::create(Thresh1, Octaves, PatternScales);

		vector<KeyPoint> keypoints_1;

		detector->detect(im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN BRISK");
	}
}

void drawFastFD(Mat& im1){

	try
	{
		// Detecting keypoints using Fast Feature Detector

		// Agast Parameters
		int Threshold = 10;
		bool nonmaxSuppression = true;
		int type = FastFeatureDetector::TYPE_9_16;

		// Initialize Algorithm

		Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(Threshold, nonmaxSuppression, type);

		vector<KeyPoint> keypoints_1;

		detector->detect(im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN FastFeatureDetector");
	}
}

void drawGFTT(Mat& im1){

	try
	{
		// Detecting keypoints using Good Features to Track Detector

		// GFTT Parameters
		int maxCorners = 1000;
		double qualityLevel = 0.01;
		double minDistance = 1;
		int blockSize = 3;
		bool useHarrisDetector = false;
		double k = 0.04;

		// Initialize Algorithm

		Ptr<GFTTDetector> detector = GFTTDetector::create(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k);

		vector<KeyPoint> keypoints_1;

		detector->detect(im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN GFTT");
	}
}

void drawKAZE(Mat& im1){

	try
	{
		// Detecting keypoints using KAZE

		// KAZE Parameters
		bool extended = false;
		bool upright = false;
		float threshold = 0.001f;
		int nOctaves = 4;
		int nOctaveLayers = 4;
		int diffusivity = KAZE::DIFF_PM_G2;

		// Initialize Algorithm

		Ptr<KAZE> detector = KAZE::create(extended, upright, threshold, nOctaves, nOctaveLayers, diffusivity);

		vector<KeyPoint> keypoints_1;

		detector->detect(im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN KAZE");
	}
}

void drawMSER(Mat& im1){

	try
	{
		// Detecting keypoints using Maximally stable extremal region extractor detector

		// MSER Parameters
		int delta = 5;
		int min_area = 60;
		int max_area = 14400;
		double max_variation = 0.25;
		double min_diversity = 0.2;
		int max_evolution = 200;
		double area_threshold = 1.01;
		double min_margin = 0.003;
		int edge_blur_size = 5;

		// Initialize Algorithm

		Ptr<MSER> detector = MSER::create(delta, min_area, max_area, max_variation, min_diversity, max_evolution, area_threshold, min_margin, edge_blur_size);

		vector<KeyPoint> keypoints_1;

		detector->detect(im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN MSER");
	}
}

void drawORB(Mat& im1){

	try
	{
		// Detecting keypoints using oriented BRIEF keypoint detector

		// ORB Parameters
		int nfeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeTHreshold = 31;
		int firstLevel = 0;
		int WTA_K = 2;
		int scoreType = ORB::HARRIS_SCORE;
		int patchSize = 31;
		int fastThreshold = 20;

		// Initialize Algorithm

		Ptr<ORB> detector = ORB::create(nfeatures, scaleFactor, nlevels, edgeTHreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

		vector<KeyPoint> keypoints_1;

		detector->detect(im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN ORB");
	}
}

void drawSBD(Mat& im1){

	try
	{
		// Detecting keypoints using Simple Blob Detector

		Ptr<SimpleBlobDetector> detector;

		vector<KeyPoint> keypoints_1;
	
		detector->detect( im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN Simple Blob Detector");
	}
}

void drawSIFT(Mat& im1){

	try
	{
		// Detecting keypoints using SIFT

		// SIFT Parameters
		int nfeatures = 0;
		int nOctaveLayers = 3;
		double contrastThreshold = 0.04;
		double edgeThreshold = 10;
		double sigma = 1.6;

		Ptr<SIFT> detector = SIFT::create( nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);

		vector<KeyPoint> keypoints_1;
	
		detector->detect( im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN SIFT");
	}
}

void drawSURF(Mat& im1){

	try
	{
		// Detecting keypoints using SURF

		// SURF Parameters
		int minHessian = 400;

		Ptr<SURF> detector = SURF::create( minHessian );

		vector<KeyPoint> keypoints_1;
	
		detector->detect( im1, keypoints_1);

		// Drawing Keypoints into im_kp1
		Mat im_kp1;

		ROS_INFO_STREAM("Detected Keypoints");

		drawKeypoints( im1, keypoints_1, im_kp1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		im1 = im_kp1;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ERROR IN SURF");
	}
}


// This callback function will get called when a new image has arrived
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert ROS image message to an OpenCV image with BGR pixel encoding. 
	// Then show it in a display window.
	try
	{
		
		// Gets image from CV bridge and places it in "im1"
		Mat im1 = cv_bridge::toCvShare(msg,"bgr8")->image;
		
		//Calls the featuring function and returns the images as a Mat
		
		//drawAgastFD(im1);
		//drawAKAZE(im1);
		//drawBRISK(im1);
		//drawFastFD(im1);
		//drawGFTT(im1);
		//drawKAZE(im1);
		//drawMSER(im1);
		//drawORB(im1);
		//drawSBD(im1); error in Simple Blob Detector
		//drawSIFT(im1);
		//drawSURF(im1);

		cv::imshow("view",im1);
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
