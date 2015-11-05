#ifndef VOSLAM_RIGID_TRANSFORMATION_H_
#define VOSLAM_RIGID_TRANSFORMATION_H_

#include "voslam_util.h"
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

class VoslamRigidTransformation {

private:
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> corrsRejectorSAC;
	boost::shared_ptr<pcl::Correspondences> correspondences;
	double minDepthValue;
	double maxDepthValue;

	void matchesWith3DValidData(const std::vector<carmen_vector_2D_t>&, const std::vector<carmen_vector_2D_t>&, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

public:
	VoslamRigidTransformation(const double = 0.2,const double = 30.0);
	/*!Sets the minDepthValue to the specified value (in meters)*/
	void setMinDepthValue(const double);

	/*!Sets the maxDepthValue to the specified value (in meters)*/
	void setMaxDepthValue(const double);

	/*!Estimates the 3D rigid transformation that best align a pair of 3D point correspondences of two given point clouds. The method takes two vectors of 2D points that determines the pairs of correspondences and two 3D point clouds. The algorithm then takes the 3D points corresponding to the 2D correspondences to estimate the 3D rigid transformation. The algorithm uses RANSAC to make the method robust to outliers.*/
	int estimateVisual3DRigidTransformation(const std::vector<carmen_vector_2D_t>& current_features, const std::vector<carmen_vector_2D_t>&  previous_features,
											const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudPtr1, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudPtr2,
											Eigen::Matrix<float, 4, 4>& rigidTransformation);

	virtual ~VoslamRigidTransformation();
};

#endif /* VOSLAM_RIGID_TRANSFORMATION_H_ */
