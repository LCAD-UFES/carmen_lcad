/*
 * kinect.h
 *
 *  Created on: Jun 25, 2012
 *      Author: lcad
 */

#ifndef KINECT_H_
#define KINECT_H_

#include <vertex_buffer_objects.h>

namespace CVIS {

	class Kinect : public VertexBufferObjects {

	public:

		int width_, height_;
		float focal_length_;
		carmen_kinect_depth_message *message;
		double *rand_drop_vector;
		int* index_drop_vector;

		int cut_off;
		int blowup_count;
		int cloud_density;


		Kinect(int width, int height, float focal_length);
		virtual ~Kinect();
		virtual void PopulatePointCloud(void* message, double cloudR, double cloudG, double cloudB, double foreRange, double backRange);
		void EffectPointCloudColor(double cor1R, double cor1G, double cor1B, double cor2R, double cor2G, double cor2B, double cor3R, double cor3G, double cor3B, double changeSpeed);
		void EffectPointCloudDrop();
		void EffectPointCloudBlowUp();
		void EffectPointCloudNoise();

		void SetCloudDensity(int density) { this->cloud_density = density; }
	};
}

#endif /* KINECT_H_ */
