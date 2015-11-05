/*
 * velodyne.h
 *
 *  Created on: Jun 25, 2012
 *      Author: lcad
 */

#ifndef VELODYNE_H_
#define VELODYNE_H_

#include <vertex_buffer_objects.h>
#include <vector>
#include <tf.h>

#include <carmen/velodyne_messages.h>
#include <carmen/fused_odometry_messages.h>

namespace CVIS {

	class Velodyne : public VertexBufferObjects {

	public:

		int number_of_32_laser_shots;
		carmen_velodyne_partial_scan_message *message;

		Velodyne(int n);
		virtual ~Velodyne();
		virtual void PopulatePointCloud(void* message, carmen_fused_odometry_message* car_poses);
		void InitializeVelodyneTransforms(carmen_vector_3D_t position, carmen_orientation_3D_t orientation);

	private:

		typedef struct {
			std::vector<carmen_velodyne_32_laser_shot> shots;
			bool visited;
		}Velodyne360Shot;

		tf::Transformer transformer;
		tf::StampedTransform g_world_to_velodyne_transform;

		Velodyne360Shot velodyne360Shots[361];

		void SetupRotationalAndVerticalTables();
		void InitializeVelodyne360Shots();
		bool VisitedAllVelodyne360Shots();

		void CopyLastVelodyneMessage(carmen_velodyne_partial_scan_message* message);
	};
}

#endif /* VELODYNE_H_ */
