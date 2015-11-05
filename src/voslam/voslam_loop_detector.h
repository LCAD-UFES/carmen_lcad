#ifndef VOSLAM_LOOP_DETECTOR_H_
#define VOSLAM_LOOP_DETECTOR_H_

#include <matcher.h>
#include <vector>
#include "voslam_generalized_icp.h"
#include "voslam_keyframes.h"
#include "voslam_rigid_transformation.h"
#include <stdlib.h>

using namespace std;

class VoslamLoopDetector {

	struct LoopDetectorParameters
	{
		double distanceForLoopDetection;
		int inliersThreshold;
		int width, height;
	};

private:
	VoslamRigidTransformation* rigid_transformation_extimator;
	Matcher::parameters matcher_params;
	Matcher* matcher;
	std::vector<Matcher::p_match> matches;
	std::vector<int32_t> inliers;

	LoopDetectorParameters params;

	unsigned char* temporary_gray_image;

	double DistanceBetweenKeyframes(carmen_voslam_pointcloud_t last_keyframe, carmen_voslam_pointcloud_t to_check_keyframe);
	int DoMatch(carmen_voslam_pointcloud_t last_keyframe, carmen_voslam_pointcloud_t to_check_keyframe);
	int GetNumberOfInliersFromMatcher(int ransac_iterations);
	bool NormalizeFeaturePoints(vector<Matcher::p_match> &p_matched,Matrix &Tp,Matrix &Tc);
	vector<int32_t> GetRandomSample(int32_t N,int32_t num);
	void FundamentalMatrix (const vector<Matcher::p_match> &p_matched,const vector<int32_t> &active,Matrix &F);
	std::vector<int32_t> GetInliers (vector<Matcher::p_match> &p_matched,Matrix &F);

public:
	VoslamLoopDetector(int width, int height, double distance_to_loop_detection, int inliers_threshold);
	bool CheckLoopDetection(VoslamKeyframes *keyframes, VoslamGeneralizedICP* icp);
	virtual ~VoslamLoopDetector();
};

#endif /* VOSLAM_LOOP_DETECTOR_H_ */
