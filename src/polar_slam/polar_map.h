/*
 * polar_map.h
 *
 *  Created on: Dec 11, 2012
 *      Author: filipe mutz
 */

#ifndef __POLAR_MAP_H_
#define __POLAR_MAP_H_

#include "sphere.h"
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

class PolarMap
{
	int _num_spheres;
	vector<Sphere> spheres;

	double calculate_range_from_sphere_index(int sphere_index);

	public:

		PolarMap();
		PolarMap(int num_spheres, int num_angular_sections, int num_points_to_store);
		~PolarMap();

		void move(carmen_pose_3D_t origin, carmen_pose_3D_t destination);
		void add(carmen_pose_3D_t point, double radius, double h_angle, double v_angle);

		double ray_cast(double angle);

		void draw(IplImage* img);
};

#endif /* __POLAR_MAP_H_ */
