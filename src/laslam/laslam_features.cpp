#include "laslam_features.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography

bool
compare_strongest_features(cv::KeyPoint first, cv::KeyPoint second)
{
  if (first.response > second.response)
	  return true;
  else
	  return false;
}

std::vector<cv::KeyPoint>
extract_matched_features(const IplImage* leftImg, const IplImage* rightImg)
{
	std::vector<cv::KeyPoint> rightKeypoints;
	std::vector<cv::KeyPoint> leftKeypoints;
	cv::Mat rightDescriptors;
	cv::Mat leftDescriptors;

	////////////////////////////
	// EXTRACT KEYPOINTS
	////////////////////////////
	// The detector can be any of (see OpenCV features2d.hpp):
	// cv::FeatureDetector * detector = new cv::DenseFeatureDetector();
	// cv::FeatureDetector * detector = new cv::FastFeatureDetector();
	// cv::FeatureDetector * detector = new cv::GFTTDetector();
	// cv::FeatureDetector * detector = new cv::MSER();
	// cv::FeatureDetector * detector = new cv::ORB();
	cv::FeatureDetector * detector = new cv::SIFT();
	// cv::FeatureDetector * detector = new cv::StarFeatureDetector();
	// cv::FeatureDetector * detector = new cv::SURF(600.0);
	// cv::FeatureDetector * detector = new cv::BRISK();
	detector->detect(rightImg, rightKeypoints);
	detector->detect(leftImg, leftKeypoints);
	delete detector;

	////////////////////////////
	// EXTRACT DESCRIPTORS
	////////////////////////////
	// The extractor can be any of (see OpenCV features2d.hpp):
	// cv::DescriptorExtractor * extractor = new cv::BriefDescriptorExtractor();
	// cv::DescriptorExtractor * extractor = new cv::ORB();
	cv::DescriptorExtractor * extractor = new cv::SIFT();
	// cv::DescriptorExtractor * extractor = new cv::SURF(600.0);
	// cv::DescriptorExtractor * extractor = new cv::BRISK();
	// cv::DescriptorExtractor * extractor = new cv::FREAK();
	extractor->compute(rightImg, rightKeypoints, rightDescriptors);
	extractor->compute(leftImg, leftKeypoints, leftDescriptors);
	delete extractor;

	////////////////////////////
	// NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
	////////////////////////////
	cv::Mat results;
	cv::Mat dists;
	int k=2; // find the 2 nearest neighbors
	if(rightDescriptors.type()==CV_8U)
	{
		// Binary descriptors detected (from ORB or Brief)

		// Create Flann LSH index
		cv::flann::Index flannIndex(leftDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
		results = cv::Mat(rightDescriptors.rows, k, CV_32SC1); // Results index
		dists = cv::Mat(rightDescriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1 ?!?!? NOTE OpenCV doc is not clear about that...

		// search (nearest neighbor)
		flannIndex.knnSearch(rightDescriptors, results, dists, k, cv::flann::SearchParams() );
	}
	else
	{
		// assume it is CV_32F
		// Create Flann KDTree index
		cv::flann::Index flannIndex(leftDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
		results = cv::Mat(rightDescriptors.rows, k, CV_32SC1); // Results index
		dists = cv::Mat(rightDescriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1

		// search (nearest neighbor)
		flannIndex.knnSearch(rightDescriptors, results, dists, k, cv::flann::SearchParams() );
	}


	////////////////////////////
	// PROCESS NEAREST NEIGHBOR RESULTS
	////////////////////////////
	// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
	float nndrRatio = 0.6;
	std::vector<cv::KeyPoint> mpts_1, mpts_2; // Used for homography
	std::vector<int> indexes_1, indexes_2; // Used for homography
	std::vector<uchar> outlier_mask;  // Used for homography
	for(int i=0; i<rightDescriptors.rows; ++i)
	{
		// Check if this descriptor matches with those of the right image
		// Apply NNDR
		if(dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
		{
			mpts_1.push_back(rightKeypoints.at(i));
			indexes_1.push_back(i);

			mpts_2.push_back(leftKeypoints.at(results.at<int>(i,0)));
			indexes_2.push_back(results.at<int>(i,0));
		}
	}

	return mpts_1;
}

std::vector<cv::KeyPoint>
extract_features(const IplImage* rightImg)
{
	std::vector<cv::KeyPoint> rightKeypoints;
	cv::Mat rightDescriptors;

	////////////////////////////
	// EXTRACT KEYPOINTS
	////////////////////////////
	// The detector can be any of (see OpenCV features2d.hpp):
	// cv::FeatureDetector * detector = new cv::DenseFeatureDetector();
	// cv::FeatureDetector * detector = new cv::FastFeatureDetector();
	// cv::FeatureDetector * detector = new cv::GFTTDetector();
	// cv::FeatureDetector * detector = new cv::MSER();
	cv::FeatureDetector * detector = new cv::ORB();
	// cv::FeatureDetector * detector = new cv::SIFT();
	// cv::FeatureDetector * detector = new cv::StarFeatureDetector();
	// cv::FeatureDetector * detector = new cv::SURF(600.0);
	// cv::FeatureDetector * detector = new cv::BRISK();
	detector->detect(rightImg, rightKeypoints);
	delete detector;

	////////////////////////////
	// EXTRACT DESCRIPTORS
	////////////////////////////
	// The extractor can be any of (see OpenCV features2d.hpp):
	// cv::DescriptorExtractor * extractor = new cv::BriefDescriptorExtractor();
	cv::DescriptorExtractor * extractor = new cv::ORB();
	// cv::DescriptorExtractor * extractor = new cv::SIFT();
	// cv::DescriptorExtractor * extractor = new cv::SURF(600.0);
	// cv::DescriptorExtractor * extractor = new cv::BRISK();
	// cv::DescriptorExtractor * extractor = new cv::FREAK();
	extractor->compute(rightImg, rightKeypoints, rightDescriptors);
	delete extractor;

	std::vector<cv::KeyPoint> mpts_1;
	for(int i=0; i<rightDescriptors.rows; ++i)
	{
		mpts_1.push_back(rightKeypoints.at(i));
	}

    	std::sort(mpts_1.begin(), mpts_1.end(), compare_strongest_features);

	return mpts_1;
}
