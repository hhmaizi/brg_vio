#ifndef detectFeature_H
#define detectFeature_H


// Everything we need to publish and subscribe to images
#include <image_transport/image_transport.h>
// Includes the necessary OpenCV libraries
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class detectFeature
{
	public:
		// Constructor and 
		detectFeature();
		void loadParams();
		void detect(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints_1);
	protected:
	private:
		// Selector used to select between different types of detectors
		int selector;

		// Initializes the detector object for each type of feature detector ->
		// -> under the name "detector" + int corresponding to the selector code for that type of detector
		// Initialization of parameters for each type of detector
		// Agast parameters
		cv::Ptr<cv::AgastFeatureDetector> detector1;
		int agast_Threshold;
		bool agast_nonmaxSuppression;
		int agast_type;
		// Akaze parameters
		cv::Ptr<cv::AKAZE> detector2;
		int akaze_descriptor_type;
		int akaze_descriptor_size;
		int akaze_descriptor_channels;
		float akaze_threshold;
		int akaze_nOctaves;
		int akaze_nOctaveLayers;
		int akaze_diffusivity;
		// Brisk parameters
		cv::Ptr<cv::BRISK> detector3;
		int brisk_Thresh1;
		int brisk_Octaves;
		float brisk_PatternScales;
		// FastFeatureDetector parameters
		cv::Ptr<cv::FastFeatureDetector> detector4;
		int ffd_Threshold;
		bool ffd_nonmaxSuppression;
		int ffd_type;
		// GFTT parameters
		cv::Ptr<cv::GFTTDetector> detector5;
		int gftt_maxCorners;
		double gftt_qualityLevel;
		double gftt_minDistance;
		int gftt_blockSize;
		bool gftt_useHarrisDetector;
		double gftt_k;
		// KAZE parameters
		cv::Ptr<cv::KAZE> detector6;
		bool kaze_extended;
		bool kaze_upright;
		float kaze_threshold;
		int kaze_nOctaves;
		int kaze_nOctaveLayers;
		int kaze_diffusivity;
		// MSER parameters
		cv::Ptr<cv::MSER> detector7;
		int mser_delta;
		int mser_min_area;
		int mser_max_area;
		double mser_max_variation;
		double mser_min_diversity;
		int mser_max_evolution;
		double mser_area_threshold;
		double mser_min_margin ;
		int mser_edge_blur_size;
		// ORB parameters
		cv::Ptr<cv::ORB> detector8;
		int orb_nfeatures;
		float orb_scaleFactor;
		int orb_nlevels;
		int orb_edgeTHreshold;
		int orb_firstLevel;
		int orb_WTA_K;
		int orb_scoreType;
		int orb_patchSize;
		int orb_fastThreshold;
		// SIFT parameters
		cv::Ptr<cv::xfeatures2d::SIFT> detector9;
		int sift_nfeatures;
		int sift_nOctaveLayers;
		double sift_contrastThreshold;
		double sift_edgeThreshold;
		double sift_sigma;
		// SURF parameters
		cv::Ptr<cv::xfeatures2d::SURF> detector10;
		int surf_minHessian;
};

#endif // DETECTFEATURE_H
