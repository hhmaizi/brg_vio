//
//	detectFeature
//	David Hanley, David Degenhardt, Alex Faustino
//
//	detectFeature.cpp
//	Feature Detector selector intended for use in Bretl Group Visual-Inertial 
//	Odometry ROS Package.
//
//	Usage:
//  	detectFeature() is the constructor for this class.
//  	loadParams() is used to read in the params from the featureDetector.yaml 
//	file.
//  	This should be called after ros::start.
//  	detect() takes a Mat image file and a keypoints vector, to which it 
//	writes the keypoints of the new image

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*------------------ Includes ------------------*/
#include "detectFeature.h"
/*---------------- End Includes ----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
/*--------------- End Namespaces ---------------*/
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Preamble ---------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*---------------------------------- Class ------------------------------------*/
/*-----------------------------------------------------------------------------*/
detectFeature::detectFeature()
{
// constructor
}

// Code for reading YAML file and initializing the detector object
void detectFeature::loadParams()
{
	//Reads the Selector variable and prints the value
	ros::param::get("/brg_vio/Selector", selector);
	ROS_INFO_STREAM("Selector: " << selector);
	// Switch/Case uses "selector" to select and then initialize the chosen feature detector
	switch (selector)
	{
	case 1:
		// Comments for Case 1 apply generally to the rest of the cases
		// Prints out the type of Feature Detector initialized
		ROS_INFO_STREAM("Feature Detector: AGAST");
		// Gets params from ROS parameter server under /brg_vio/{detectorType_paramVariable}
		ros::param::get("/brg_vio/agast_Threshold", agast_Threshold);
		ros::param::get("/brg_vio/agast_nonmaxSuppression", agast_nonmaxSuppression);
		ros::param::get("/brg_vio/agast_type", agast_type);
		// creates the detector using the params loaded in above
		detector1 = AgastFeatureDetector::create(agast_Threshold, agast_nonmaxSuppression, agast_type);
		break;
	case 2:
		ROS_INFO_STREAM("Feature Detector: AKAZE");
		ros::param::get("/brg_vio/akaze_descriptor_type", akaze_descriptor_type);
		ros::param::get("/brg_vio/akaze_descriptor_size", akaze_descriptor_size);
		ros::param::get("/brg_vio/akaze_descriptor_channels", akaze_descriptor_channels);
		ros::param::get("/brg_vio/akaze_threshold", akaze_threshold);
		ros::param::get("/brg_vio/akaze_nOctaves", akaze_nOctaves);
		ros::param::get("/brg_vio/akaze_nOctaveLayers", akaze_nOctaveLayers);
		ros::param::get("/brg_vio/akaze_diffusivity", akaze_diffusivity);
		detector2 = AKAZE::create(akaze_descriptor_type, akaze_descriptor_size, akaze_descriptor_channels,
								  akaze_threshold, akaze_nOctaves, akaze_nOctaveLayers, akaze_diffusivity);
		break;
	case 3:
		ROS_INFO_STREAM("Feature Detector: BRISK");
		ros::param::get("/brg_vio/brisk_Thresh1", brisk_Thresh1);
		ros::param::get("/brg_vio/brisk_Octaves", brisk_Octaves);
		ros::param::get("/brg_vio/brisk_PatternScales", brisk_PatternScales);
		detector3 = BRISK::create(brisk_Thresh1, brisk_Octaves, brisk_PatternScales);
		break;
	case 4:
		ROS_INFO_STREAM("Feature Detector: FastFeatureDetector");
		ros::param::get("/brg_vio/ffd_Threshold", ffd_Threshold);
		ros::param::get("/brg_vio/ffd_nonmaxSuppression", ffd_nonmaxSuppression);
		ros::param::get("/brg_vio/ffd_type", ffd_type);
		detector4 = FastFeatureDetector::create(ffd_Threshold, ffd_nonmaxSuppression, ffd_type);
		break;
	case 5:
		ROS_INFO_STREAM("Feature Detector: GoodFeaturesToTrack");
		ros::param::get("/brg_vio/gftt_maxCorners", gftt_maxCorners);
		ros::param::get("/brg_vio/gftt_qualityLevel", gftt_qualityLevel);
		ros::param::get("/brg_vio/gftt_minDistance", gftt_minDistance);
		ros::param::get("/brg_vio/gftt_blockSize", gftt_blockSize);
		ros::param::get("/brg_vio/gftt_useHarrisDetector", gftt_useHarrisDetector);
		ros::param::get("/brg_vio/gftt_k", gftt_k);
		detector5 = GFTTDetector::create(gftt_maxCorners, gftt_qualityLevel,
										 gftt_minDistance, gftt_blockSize, gftt_useHarrisDetector, gftt_k);
		break;
	case 6:
		ROS_INFO_STREAM("Feature Detector: KAZE");
		ros::param::get("/brg_vio/kaze_extended", kaze_extended);
		ros::param::get("/brg_vio/kaze_upright", kaze_upright);
		ros::param::get("/brg_vio/kaze_threshold", kaze_threshold);
		ros::param::get("/brg_vio/kaze_nOctaves", kaze_nOctaves);
		ros::param::get("/brg_vio/kaze_nOctaveLayers", kaze_nOctaveLayers);
		ros::param::get("/brg_vio/kaze_diffusivity", kaze_diffusivity);
		detector6 = KAZE::create(kaze_extended, kaze_upright, kaze_threshold,
								 kaze_nOctaves, kaze_nOctaveLayers, kaze_diffusivity);
		break;
	case 7:
		ROS_INFO_STREAM("Feature Detector: MSER");
		ros::param::get("/brg_vio/mser_delta", mser_delta);
		ros::param::get("/brg_vio/mser_min_area", mser_min_area);
		ros::param::get("/brg_vio/mser_max_area", mser_max_area);
		ros::param::get("/brg_vio/mser_max_variation", mser_max_variation);
		ros::param::get("/brg_vio/mser_min_diversity", mser_min_diversity);
		ros::param::get("/brg_vio/mser_max_evolution", mser_max_evolution);
		ros::param::get("/brg_vio/mser_area_threshold", mser_area_threshold);
		ros::param::get("/brg_vio/mser_min_margin", mser_min_margin);
		ros::param::get("/brg_vio/mser_edge_blur_size", mser_edge_blur_size);
		detector7 = MSER::create(mser_delta, mser_min_area, mser_max_area, mser_max_variation, mser_min_diversity,
								 mser_max_evolution, mser_area_threshold, mser_min_margin, mser_edge_blur_size);
		break;
	case 8:
		ROS_INFO_STREAM("Feature Detector: ORB");
		ros::param::get("/brg_vio/orb_nfeatures", orb_nfeatures);
		ros::param::get("/brg_vio/orb_scaleFactor", orb_scaleFactor);
		ros::param::get("/brg_vio/orb_nlevels", orb_nlevels);
		ros::param::get("/brg_vio/orb_edgeTHreshold", orb_edgeTHreshold);
		ros::param::get("/brg_vio/orb_firstLevel", orb_firstLevel);
		ros::param::get("/brg_vio/orb_WTA_K", orb_WTA_K);
		ros::param::get("/brg_vio/orb_scoreType", orb_scoreType);
		ros::param::get("/brg_vio/orb_patchSize", orb_patchSize);
		ros::param::get("/brg_vio/orb_fastThreshold", orb_fastThreshold);
		detector8 = ORB::create(orb_nfeatures, orb_scaleFactor, orb_nlevels, orb_edgeTHreshold,
								orb_firstLevel, orb_WTA_K, orb_scoreType, orb_patchSize, orb_fastThreshold);

		break;
	case 9:
		ROS_INFO_STREAM("Feature Detector: SIFT");
		ros::param::get("/brg_vio/sift_nfeatures", sift_nfeatures);
		ros::param::get("/brg_vio/sift_nOctaveLayers", sift_nOctaveLayers);
		ros::param::get("/brg_vio/sift_contrastThreshold", sift_contrastThreshold);
		ros::param::get("/brg_vio/sift_edgeThreshold", sift_edgeThreshold);
		ros::param::get("/brg_vio/sift_sigma", sift_sigma);
		detector9 = SIFT::create(sift_nfeatures, sift_nOctaveLayers, sift_contrastThreshold,
								 sift_edgeThreshold, sift_sigma);
		break;
	case 10:
		ROS_INFO_STREAM("Feature Detector: SURF");
		ros::param::get("/brg_vio/surf_minHessian", surf_minHessian);
		detector10 = SURF::create(surf_minHessian);
		break;
	}
}

//Function to detect keypoints after detector is constructed and parameters are loaded
void detectFeature::detect(Mat& im1, vector<KeyPoint>& keypoints_1)
{
	// detect() takes the image Mat pointer "im1" and KeyPoint pointer "keypoints_1" ->
	// and uses the detector initialized in loadParams to write the new keypoints into keypoints_1 
	switch (selector)
	{
	case 1:
		try
		{
			// Detecting keypoints using Agast Feature Detector
			detector1->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN AgastFeatureDetector");
		}
		break;

	case 2:
		try
		{
			// Detecting keypoints using AKAZE
			detector2->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN AKAZE");
		}
		break;

	case 3:
		try
		{
			// Detecting keypoints using BRISK
			detector3->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN BRISK");
		}
		break;

	case 4:
		try
		{
			// Detecting keypoints using FastFeatureDetector
			detector4->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN BRISK");
		}
		break;

	case 5:
		try {
			// Detecting keypoints using Good Features to Track Detector
			detector5->detect(im1, keypoints_1);
		} catch (cv_bridge::Exception &e) {
			ROS_ERROR("ERROR IN GFTT");
		} break;
		case 6:
		try
		{
			// Detecting keypoints using KAZE
			detector6->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN KAZE");
		}
		break;
	case 7:
		try
		{
			// Detecting keypoints using Maximally stable extremal region extractor detector
			detector7->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN MSER");
		}
		break;
	case 8:
		try
		{
			// Detecting keypoints using oriented BRIEF keypoint detector
			detector8->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN ORB");
		}
		break;
	case 9:
		try
		{
			// Detecting keypoints using SIFT
			detector9->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN SIFT");
		}
		break;
	case 10:
		try
		{
			// Detecting keypoints using SURF
			detector10->detect(im1, keypoints_1);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("ERROR IN SURF");
		}
		break;
	}
}
/*-----------------------------------------------------------------------------*/
/*-------------------------------- End Class ----------------------------------*/
/*-----------------------------------------------------------------------------*/
