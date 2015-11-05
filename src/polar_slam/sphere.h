/*
 *
 * sphere.h
 *  Created on: Dec 11, 2012
 *      Author: filipe mutz
 */

#ifndef __SPHERE_H_
#define __SPHERE_H_

#include <vector>
#include <list>

#include <carmen/carmen.h>
#include "polar_point.h"

using namespace std;

class Sphere
{
	class SphereAngularSectionData
	{
		public:

			int num_points_to_store;

			list<carmen_pose_3D_t> points;
			list<carmen_polar_point_t> polar_points;

			SphereAngularSectionData();
			~SphereAngularSectionData();
	};

	class SphereAngularSection
	{
		SphereAngularSectionData data;

		public:

			SphereAngularSection(int num_points_to_store);
			~SphereAngularSection();

			void add(carmen_pose_3D_t point, double radius, double angle);
			vector<carmen_pose_3D_t> pop();
			vector<carmen_pose_3D_t> get();

			int has_point();
			double get_radius_of_closest_point();
	};

	int _num_angle_sections;
	vector<SphereAngularSection> sections;

	public:

		Sphere(int num_angular_sections, int num_points_to_store);
		~Sphere();

		void add(carmen_pose_3D_t point, double radius, double h_angle, double v_angle);
		vector<carmen_pose_3D_t> pop();
		vector<carmen_pose_3D_t> get();

		int has_point();
		int has_point(double angle);
		double get_radius_of_closest_point();
		double get_radius_of_closest_point(double angle);
};

#endif /* __SPHERE_H_ */
