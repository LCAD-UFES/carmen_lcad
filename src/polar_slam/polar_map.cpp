/*
 * polar_map.cpp
 *
 *  Created on: Dec 11, 2012
 *      Author: filipe mutz
 */

#include "polar_map.h"
#include "tf_util.h"
#include <cmath>


double
PolarMap::calculate_range_from_sphere_index(int sphere_index)
{
	return pow(2.0, sphere_index + 1.0);
}


PolarMap::PolarMap()
{
}


PolarMap::PolarMap(int num_spheres, int num_angular_sections, int num_points_to_store)
{
	_num_spheres = num_spheres;

	for(int i = 0; i < num_spheres; i++)
		spheres.push_back(Sphere(num_angular_sections, num_points_to_store));
}


PolarMap::~PolarMap()
{
}


void
PolarMap::move(carmen_pose_3D_t origin, carmen_pose_3D_t destination)
{
	vector<carmen_pose_3D_t> points;
	TfFrameTransformationWrapper tf_wrapper(origin, destination);

	// esse loop junta todos os pontos das esferas em um array.
	// nao podemos fazer a transformacao e reinsercao nesse loop pq
	// alguns pontos podem trocar de esferas e com isso serem
	// transformados de novo.
	for(int i = 0; i < _num_spheres; i++)
	{
		vector<carmen_pose_3D_t> sphere_points = spheres[i].pop();

		for(int j = 0; j < (int) sphere_points.size(); j++)
			points.push_back(sphere_points[j]);
	}

	// aplica a transformacao sobre cada um dos
	// pontos do array e o readiciona no mapa
	for(int i = 0; i < (int) points.size(); i++)
	{
		double angle, radius;

		/**
		 * TODO: Existe uma singularidade aqui que eu nao entendi:
		 * Quando eu faco a transformacao com o yaw do ponto, ele eh movido para
		 * uma posicao maluca. Quando eu coloco zero e atualizo no final, ele
		 * eh movido normalmente. Eh importante estudar e entender as transformacoes
		 * da TF para ver o motivo disso.
		 */

		carmen_pose_3D_t point = points[i];
		point.orientation.yaw = 0;

		carmen_pose_3D_t moved_point;
		moved_point = tf_wrapper.transform(point);

		/**
		 * TODO: Outra coisa que acontece eh que a transformada da TF pura faz
		 * o ponto "linearizar" as curvas. Toda vez que o robo passa por uma curva
		 * ela estava ficando reta. Por esse motivo, tive que fazer a rotacao abaixo
		 * na mao de forma que as curvas fiquem de fato curvas.
		 */

		transform_cartesian_coordinates_to_polar_coordinates(moved_point.position.x, moved_point.position.y, &radius, &angle);

		moved_point.orientation.yaw = carmen_normalize_theta(angle - destination.orientation.yaw + origin.orientation.yaw);
		angle = moved_point.orientation.yaw;

		transform_polar_coordinates_to_cartesian_coordinates(radius, angle, &(moved_point.position.x), &(moved_point.position.y));

		add(moved_point, radius, angle, 0.0);
	}
}


void
PolarMap::add(carmen_pose_3D_t point, double radius, double h_angle, double v_angle)
{
	for(int i = 0; i < _num_spheres; i++)
	{
		double sphere_radius = calculate_range_from_sphere_index(i);

		if (radius < sphere_radius)
		{
			spheres[i].add(point, radius, h_angle, v_angle);
			break;
		}
	}
}


double
PolarMap::ray_cast(double angle)
{
	for(unsigned int i = 0; i < spheres.size(); i++)
		if (spheres[i].has_point(angle))
			return spheres[i].get_radius_of_closest_point(angle);

	return -1;
}


void
PolarMap::draw(IplImage* img)
{
	double pixels_per_meter_x = ((double) img->width) / (2 * calculate_range_from_sphere_index(_num_spheres - 1));
	double pixels_per_meter_y = ((double) img->height) / (2 * calculate_range_from_sphere_index(_num_spheres - 1));

	memset(img->imageData, 255, img->imageSize);

	cvCircle(
		img,
		cvPoint(img->width / 2, img->height / 2),
		img->width / 2,
		cvScalar(0, 0, 255, 0),
		1, 1, 0
	);

	cvRectangle(img,
		cvPoint(img->width / 2 - (2.2 * pixels_per_meter_x), img->height / 2 - (0.8 * pixels_per_meter_y)),
		cvPoint(img->width / 2 + (2.2 * pixels_per_meter_x), img->height / 2 + (0.8 * pixels_per_meter_y)),
		cvScalar(0, 0, 255, 0),
		1, 1, 0
	);

	for(int i = 0; i < _num_spheres; i++)
	{
		vector<carmen_pose_3D_t> points = spheres[i].get();

		for(int j = 0; j < (int) points.size(); j++)
		{
			int pixel_x = (int) (points[j].position.x * pixels_per_meter_x);
			int pixel_y = (int) (points[j].position.y * pixels_per_meter_y);

			pixel_x += (img->width / 2);
			pixel_y += (img->height / 2);

			// coloco o pixel_y de cabeca para baixo para ficar
			// mais parecido com o referencial do carmen
			pixel_y = img->height - pixel_y;

			int p = 3 * (pixel_y * img->width + pixel_x);

			img->imageData[p + 0] = 0;
			img->imageData[p + 1] = 0;
			img->imageData[p + 2] = 0;
		}
	}
}

