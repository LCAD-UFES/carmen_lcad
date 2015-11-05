/*
 * sphere.cpp
 *
 *  Created on: Dec 11, 2012
 *      Author: filipe mutz
 */

#include "sphere.h"
#include <vector>
#include <list>
#include <cmath>

using namespace std;

Sphere::SphereAngularSectionData::SphereAngularSectionData()
{
}


Sphere::SphereAngularSectionData::~SphereAngularSectionData()
{
}


Sphere::SphereAngularSection::SphereAngularSection(int num_points_to_store)
{
	data.num_points_to_store = num_points_to_store;
}


Sphere::SphereAngularSection::~SphereAngularSection()
{
}


void
Sphere::SphereAngularSection::add(carmen_pose_3D_t point, double radius, double angle)
{
	list<carmen_polar_point_t>::iterator it_polar = data.polar_points.begin();
	list<carmen_pose_3D_t>::iterator it_points = data.points.begin();

	carmen_polar_point_t polar_point;
	polar_point.angle = angle;
	polar_point.radius = radius;

	// busca a primeira posicao na lista de pontos que
	// contem um elemento com raio menor que o ponto
	// que queremos inserir

	while (it_polar != data.polar_points.end())
	{
		if (polar_point.radius < it_polar->radius)
			break;

		it_polar++;
		it_points++;
	}

	// inserimos o ponto nessa posicao. ao fazer isso, a
	// lista esta sempre ordenada de acordo com o raio
	// do ponto

	data.points.insert(it_points, point);
	data.polar_points.insert(it_polar, polar_point);

	// finalmente, removemos os elementos do final da lista
	// (com maior raio) ate atingir o numero maximo de pontos
	// que devem ser armazenados

	while ((int) data.points.size() > data.num_points_to_store)
		data.points.pop_back();

	while ((int) data.polar_points.size() > data.num_points_to_store)
		data.polar_points.pop_back();
}


vector<carmen_pose_3D_t>
Sphere::SphereAngularSection::pop()
{
	vector<carmen_pose_3D_t> v;
	carmen_pose_3D_t point;

	while(data.points.size() > 0)
	{
		point = data.points.front();

		data.points.pop_front();
		data.polar_points.pop_front();

		v.push_back(point);
	}

	return v;
}


vector<carmen_pose_3D_t>
Sphere::SphereAngularSection::get()
{
	vector<carmen_pose_3D_t> v;
	list<carmen_pose_3D_t>::iterator it;

	for(it = data.points.begin(); it != data.points.end(); it++)
		v.push_back(*it);

	return v;
}


int
Sphere::SphereAngularSection::has_point()
{
	return data.points.size();
}


double
Sphere::SphereAngularSection::get_radius_of_closest_point()
{
	if (data.polar_points.size() == 0)
	{
		printf("Error: requesting points in an empty sphere section\n");
		exit(-1);
	}

	return data.polar_points.begin()->radius;
}


Sphere::Sphere(int num_angular_sections, int num_points_to_store)
{
	_num_angle_sections = num_angular_sections;

	for(int i = 0; i < _num_angle_sections; i++)
		sections.push_back(SphereAngularSection(num_points_to_store));
}


Sphere::~Sphere()
{

}

double
normalize_angle(double angle)
{
	if ((angle <= M_PI) && (angle >= -M_PI))
		return angle;
	else
		return carmen_normalize_theta(angle);
}


void
Sphere::add(carmen_pose_3D_t point, double radius, double h_angle, double v_angle)
{
	// this parameter will be
	// necessary in the future
	(void) v_angle;

	h_angle = normalize_angle(h_angle);

	double angle_granularity = (2 * M_PI) / _num_angle_sections;
	double sphere_section_index = (h_angle + M_PI) / angle_granularity;

	sections[sphere_section_index].add(point, radius, h_angle);
}


vector<carmen_pose_3D_t>
Sphere::pop()
{
	vector<carmen_pose_3D_t> points;

	for(int i = 0; i < _num_angle_sections; i++)
	{
		vector<carmen_pose_3D_t> section_points = sections[i].pop();

		for(int j = 0; j < (int) section_points.size(); j++)
			points.push_back(section_points[j]);
	}

	return points;
}


vector<carmen_pose_3D_t>
Sphere::get()
{
	vector<carmen_pose_3D_t> points;

	for(int i = 0; i < _num_angle_sections; i++)
	{
		vector<carmen_pose_3D_t> section_points = sections[i].get();

		for(int j = 0; j < (int) section_points.size(); j++)
			points.push_back(section_points[j]);
	}

	return points;
}

int
Sphere::has_point()
{
	// not implemented yet...
	return 0;
}


int
Sphere::has_point(double angle)
{
	double angle_variation = (2 * M_PI) / _num_angle_sections;
	int sphere_section_index = (int) ((angle + M_PI) / angle_variation);

	return sections[sphere_section_index].has_point();
}


double
Sphere::get_radius_of_closest_point()
{
	// not implemented yet...
	return 0;
}


double
Sphere::get_radius_of_closest_point(double angle)
{
	double angle_variation = (2 * M_PI) / _num_angle_sections;
	int sphere_section_index = (int) ((angle + M_PI) / angle_variation);

	return sections[sphere_section_index].get_radius_of_closest_point();
}

