#ifndef SLAM6D_H_
#define SLAM6D_H_

#include <vertex_buffer_objects.h>
#include <carmen/slam6d_messages.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

namespace CVIS {

	class Slam6d : public VertexBufferObjects {

	public:

		int width_, height_;
		float focal_length_, max_depth_;
		carmen_slam6d_pointcloud_message *message;

		pcl::PointCloud<pcl::PointXYZ> rawPointCloud;
		pcl::PointCloud<pcl::PointXYZ> transformedPointCloud;
		Eigen::Matrix4f pcTransform;


		Slam6d(int width, int height, float focal_length, float max_depth);
		virtual ~Slam6d();
		virtual void PopulatePointCloud(void* message);
		void UploadPointSlam6dCloudDataToVBOWithCUDA(void (*external_callback)(carmen_slam6d_pointcloud_message* message, float* posV, float* posC, float focal_length));

	private:
		Eigen::Matrix4f ComposePointCloudTransformationMatrix(carmen_slam6d_pointcloud_message* message);
	};
}


#endif /* SLAM6D_H_ */
