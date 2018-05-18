/*
 * laser_ldmrs_utils.cpp
 *
 *  Created on: Nov 10, 2016
 *      Author: vinicius
 */

#include "laser_ldmrs_utils.h"
#include <stdio.h>
#include <stdlib.h>


void
get_layers(carmen_laser_ldmrs_message *msg,
		int &layer_1_start, int &layer_1_size, int &layer_2_start, int &layer_2_size,
		int &layer_3_start, int &layer_3_size, int &layer_4_start, int &layer_4_size)
{
	int i = 1;

	layer_1_start = 0;
	while (msg->arraypoints[i].horizontal_angle < msg->arraypoints[i - 1].horizontal_angle)
		i++;
	layer_1_size = i - layer_1_start;

	layer_2_start = i;
	i++;
	while (msg->arraypoints[i].horizontal_angle < msg->arraypoints[i - 1].horizontal_angle)
		i++;
	layer_2_size = i - layer_2_start;

	layer_3_start = i;
	i++;
	while (msg->arraypoints[i].horizontal_angle < msg->arraypoints[i - 1].horizontal_angle)
		i++;
	layer_3_size = i - layer_3_start;

	layer_4_start = i;
	i++;
	while (msg->arraypoints[i].horizontal_angle < msg->arraypoints[i - 1].horizontal_angle)
		i++;
	layer_4_size = i - layer_4_start;
}


carmen_velodyne_partial_scan_message
carmen_laser_ldmrs_convert_laser_scan_to_partial_velodyne_message(carmen_laser_ldmrs_message *msg, double laserscan_timestamp)
{
	carmen_velodyne_partial_scan_message velodyne_message;

	int layer_1_start, layer_1_size, layer_2_start, layer_2_size,
		layer_3_start, layer_3_size, layer_4_start, layer_4_size;

	get_layers(msg, layer_1_start, layer_1_size, layer_2_start, layer_2_size,
					layer_3_start, layer_3_size, layer_4_start, layer_4_size);

	double layer_1_start_angle = msg->arraypoints[layer_1_start].horizontal_angle;
	double layer_1_end_angle = msg->arraypoints[layer_1_start + layer_1_size - 1].horizontal_angle;
	double layer_2_start_angle = msg->arraypoints[layer_2_start].horizontal_angle;
	double layer_2_end_angle = msg->arraypoints[layer_2_start + layer_2_size - 1].horizontal_angle;
	double layer_3_start_angle = msg->arraypoints[layer_3_start].horizontal_angle;
	double layer_3_end_angle = msg->arraypoints[layer_3_start + layer_3_size - 1].horizontal_angle;
	double layer_4_start_angle = msg->arraypoints[layer_4_start].horizontal_angle;
	double layer_4_end_angle = msg->arraypoints[layer_4_start + layer_4_size - 1].horizontal_angle;

	double largest_angle12 = (layer_1_start_angle > layer_2_start_angle) ? layer_1_start_angle: layer_2_start_angle;
	double largest_angle34 = (layer_3_start_angle > layer_4_start_angle) ? layer_3_start_angle: layer_4_start_angle;
	double largest_angle = (largest_angle12 > largest_angle34) ? largest_angle12: largest_angle34;

	double smallest_angle12 = (layer_1_end_angle < layer_2_end_angle) ? layer_1_end_angle: layer_2_end_angle;
	double smallest_angle34 = (layer_3_end_angle < layer_4_end_angle) ? layer_3_end_angle: layer_4_end_angle;
	double smallest_angle = (smallest_angle12 < smallest_angle34) ? smallest_angle12: smallest_angle34;

	double half_a_degree = carmen_degrees_to_radians(0.5);
	int number_of_shots = (int) ((largest_angle - smallest_angle) / half_a_degree);
	if (number_of_shots <= 0)
	{
		velodyne_message.number_of_32_laser_shots = 0;
		return (velodyne_message);
	}

	velodyne_message.number_of_32_laser_shots = number_of_shots;
	velodyne_message.partial_scan = (carmen_velodyne_32_laser_shot *) calloc (velodyne_message.number_of_32_laser_shots, sizeof(carmen_velodyne_32_laser_shot));
	velodyne_message.timestamp = laserscan_timestamp;
	velodyne_message.host = carmen_get_host();

	for (int i = 0, i1 = 0, i2 = 0, i3 = 0, i4 = 0; i < velodyne_message.number_of_32_laser_shots; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			velodyne_message.partial_scan[i].distance[j] = 0;
			velodyne_message.partial_scan[i].intensity[j] = 255;
		}

		double angle = largest_angle - ((double) i) * half_a_degree - 0.000005;
		velodyne_message.partial_scan[i].angle = -carmen_radians_to_degrees(angle);

		if ((angle <= msg->arraypoints[layer_1_start + i1].horizontal_angle) &&
			(angle >= layer_1_end_angle))
		{
			double distance = msg->arraypoints[layer_1_start + i1].radial_distance;
			distance = (distance > 130.0)? 0.0: distance; // larger than 130 meters does no fit an unsigned short int distance field of Velodyne...
			velodyne_message.partial_scan[i].distance[0] = (unsigned short) (round(distance * 500.0));
			velodyne_message.partial_scan[i].intensity[0] = 255;
			i1++;
		}

		if ((angle <= msg->arraypoints[layer_2_start + i2].horizontal_angle) &&
			(angle >= layer_2_end_angle))
		{
			double distance = msg->arraypoints[layer_2_start + i2].radial_distance;
			distance = (distance > 130.0)? 0.0: distance; // larger than 130 meters does no fit an unsigned short int distance field of Velodyne...
			velodyne_message.partial_scan[i].distance[1] = (unsigned short) (round(distance * 500.0));
			velodyne_message.partial_scan[i].intensity[1] = 255;
			i2++;
		}

		if ((angle <= msg->arraypoints[layer_3_start + i3].horizontal_angle) &&
			(angle >= layer_3_end_angle))
		{
			double distance = msg->arraypoints[layer_3_start + i3].radial_distance;
			distance = (distance > 130.0)? 0.0: distance; // larger than 130 meters does no fit an unsigned short int distance field of Velodyne...
			velodyne_message.partial_scan[i].distance[2] = (unsigned short) (round(distance * 500.0));
			velodyne_message.partial_scan[i].intensity[2] = 255;
			i3++;
		}

		if ((angle <= msg->arraypoints[layer_4_start + i4].horizontal_angle) &&
			(angle >= layer_4_end_angle))
		{
			double distance = msg->arraypoints[layer_4_start + i4].radial_distance;
			distance = (distance > 130.0)? 0.0: distance; // larger than 130 meters does no fit an unsigned short int distance field of Velodyne...
			velodyne_message.partial_scan[i].distance[3] = (unsigned short) (round(distance * 500.0));
			velodyne_message.partial_scan[i].intensity[3] = 255;
			i4++;
		}
	}

	return (velodyne_message);
}


carmen_velodyne_partial_scan_message
carmen_laser_ldmrs_new_convert_laser_scan_to_partial_velodyne_message(carmen_laser_ldmrs_new_message *msg, double laserscan_timestamp)
{
	carmen_velodyne_partial_scan_message velodyne_message;

	double delta_angle = carmen_degrees_to_radians(0.25);
	if (((msg->start_angle - msg->end_angle) / delta_angle) <= 0)
	{
		velodyne_message.number_of_32_laser_shots = 0;
		return (velodyne_message);
	}

	velodyne_message.number_of_32_laser_shots = (int) ceil((msg->start_angle - msg->end_angle) / delta_angle);
	velodyne_message.partial_scan = (carmen_velodyne_32_laser_shot *) calloc(velodyne_message.number_of_32_laser_shots, sizeof(carmen_velodyne_32_laser_shot));
	velodyne_message.timestamp = laserscan_timestamp;
	velodyne_message.host = carmen_get_host();

	int index = 0;
	int layer = msg->arraypoints[0].layer;
	if (layer != 0)
	{
		for (int j = 0; j < layer; j++)
		{
			velodyne_message.partial_scan[index].distance[j] = 0;
			velodyne_message.partial_scan[index].intensity[j] = 255;
			velodyne_message.partial_scan[index].angle = -carmen_radians_to_degrees(msg->arraypoints[0].horizontal_angle);
		}
	}
	for (int i = 0; i < msg->scan_points; i++)
	{
		double distance = msg->arraypoints[i].radial_distance;
		distance = (distance > 130.0) ? 0.0: distance; // Sem esta linha aparecem echos de laser que sujam o mapa. Deve estar excedendo o limite do short int que eh usado para codificar a distancia no Velodyne.

		int flags = msg->arraypoints[i].flags;
		if ((flags & SLDMRS_POINT_FLAG_NOISE) || (flags & SLDMRS_POINT_FLAG_GROUND) || (flags & SLDMRS_POINT_FLAG_DIRT))
			distance = 0.0;

		layer = msg->arraypoints[i].layer;
		velodyne_message.partial_scan[index].distance[layer] = (unsigned short) (round(distance * 500.0));
		velodyne_message.partial_scan[index].intensity[layer] = 255;
		velodyne_message.partial_scan[index].angle = -carmen_radians_to_degrees(msg->arraypoints[i].horizontal_angle);
		if (((i + 1) < msg->scan_points) && (msg->arraypoints[i + 1].layer < msg->arraypoints[i].layer))
		{
			for (layer = msg->arraypoints[i].layer + 1; layer < 4; layer++)
			{
				velodyne_message.partial_scan[index].distance[layer] = 0;
				velodyne_message.partial_scan[index].intensity[layer] = 255;
				velodyne_message.partial_scan[index].angle = -carmen_radians_to_degrees(msg->arraypoints[i].horizontal_angle);
			}
			index++;
		}
	}
	velodyne_message.number_of_32_laser_shots = index;

	return (velodyne_message);
}


void
carmen_laser_ldmrs_copy_laser_scan_to_message(carmen_laser_ldmrs_message *message, vpLaserScan laserscan[4])
{
	message->scan_number = laserscan[0].getMeasurementId();
	message->scan_start_time = laserscan[0].getStartTimestamp();
	message->scan_end_time = laserscan[0].getEndTimestamp();
	message->angle_ticks_per_rotation = laserscan[0].getNumSteps();
	message->start_angle = laserscan[0].getStartAngle();
	message->end_angle = laserscan[0].getStopAngle();

	if (laserscan[0].getNumPoints() == 0)
		return;

	int num_of_points = laserscan[0].getNumPoints();

	std::vector<vpScanPoint> pointsInLayer1 = laserscan[0].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer2 = laserscan[1].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer3 = laserscan[2].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer4 = laserscan[3].getScanPoints();

	int sizeLayer1 = pointsInLayer1.size();
	int sizeLayer2 = pointsInLayer2.size();
	int sizeLayer3 = pointsInLayer3.size();
	int sizeLayer4 = pointsInLayer4.size();

	if (num_of_points > (sizeLayer1 + sizeLayer2 + sizeLayer3 + sizeLayer4))
	{
		num_of_points = sizeLayer1 + sizeLayer2 + sizeLayer3 + sizeLayer4;
	}

	if (message->scan_points != num_of_points)
	{
		message->scan_points = num_of_points;
		message->arraypoints = (carmen_laser_ldmrs_point *) realloc(message->arraypoints, message->scan_points * sizeof(carmen_laser_ldmrs_point));
		carmen_test_alloc(message->arraypoints);
	}

	int i;
	for (i = 0; i < sizeLayer1; i++)
	{
		message->arraypoints[i].horizontal_angle = pointsInLayer1[i].getHAngle();
		message->arraypoints[i].vertical_angle = pointsInLayer1[i].getVAngle();
		message->arraypoints[i].radial_distance = pointsInLayer1[i].getRadialDist();
		message->arraypoints[i].flags = pointsInLayer1[i].getFlags();
	}

	for (i = 0; i < sizeLayer2; i++)
	{
		message->arraypoints[i + sizeLayer1].horizontal_angle = pointsInLayer2[i].getHAngle();
		message->arraypoints[i + sizeLayer1].vertical_angle = pointsInLayer2[i].getVAngle();
		message->arraypoints[i + sizeLayer1].radial_distance = pointsInLayer2[i].getRadialDist();
		message->arraypoints[i + sizeLayer1].flags = pointsInLayer2[i].getFlags();
	}

	for (i = 0; i < sizeLayer3; i++)
	{
		message->arraypoints[i + sizeLayer1 + sizeLayer2].horizontal_angle = pointsInLayer3[i].getHAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].vertical_angle = pointsInLayer3[i].getVAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].radial_distance = pointsInLayer3[i].getRadialDist();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].flags = pointsInLayer3[i].getFlags();

	}

	for (i = 0; i < sizeLayer4; i++)
	{
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].horizontal_angle = pointsInLayer4[i].getHAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].vertical_angle = pointsInLayer4[i].getVAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].radial_distance = pointsInLayer4[i].getRadialDist();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].flags = pointsInLayer4[i].getFlags();
	}
}


void
carmen_laser_ldmrs_new_copy_laser_scan_to_message(carmen_laser_ldmrs_new_message *message, struct sickldmrs_scan *scan)
{
	message->scan_number = scan->scan_number;
	message->scanner_status = scan->scanner_status;
	message->sync_phase_offset = scan->sync_phase_offset;
	message->scan_start_time = (double) scan->scan_start_time.tv_nsec / 1000000000.0 + (double) scan->scan_start_time.tv_sec;
	message->scan_end_time = (double) scan->scan_end_time.tv_nsec / 1000000000.0 + (double) scan->scan_end_time.tv_sec;
	message->angle_ticks_per_rotation = scan->angle_ticks_per_rotation;
	message->start_angle = scan->start_angle;
	message->end_angle = scan->end_angle;
	message->flags = scan->flags;

	if (message->scan_points != scan->scan_points)
	{
		message->scan_points = scan->scan_points;
		message->arraypoints = (carmen_laser_ldmrs_new_point *) realloc(message->arraypoints, message->scan_points * sizeof(carmen_laser_ldmrs_new_point));
		carmen_test_alloc(message->arraypoints);
	}

	for (int i = 0; i < message->scan_points; i++)
	{
		message->arraypoints[i].horizontal_angle = scan->points[i].horizontal_angle;
		message->arraypoints[i].radial_distance = scan->points[i].radial_distance;
		message->arraypoints[i].flags = scan->points[i].flags;
		message->arraypoints[i].echo = scan->points[i].echo;
		message->arraypoints[i].layer = scan->points[i].layer;
		switch (scan->points[i].layer)
		{
			case 0:
				message->arraypoints[i].vertical_angle = -0.020944;
				break;
			case 1:
				message->arraypoints[i].vertical_angle = -0.00698132;
				break;
			case 2:
				message->arraypoints[i].vertical_angle = 0.00698132;
				break;
			case 3:
				message->arraypoints[i].vertical_angle = 0.020944;
				break;
		}
	}
}


void
carmen_laser_ldmrs_copy_message_to_laser_scan(vpLaserScan laserscan[4], carmen_laser_ldmrs_message *message)
{
	laserscan[0].setMeasurementId(message->scan_number);
	laserscan[0].setStartTimestamp(message->scan_start_time);
	laserscan[0].setEndTimestamp(message->scan_end_time);
	laserscan[0].setNumSteps(message->angle_ticks_per_rotation);
	laserscan[0].setStartAngle(message->start_angle);
	laserscan[0].setStopAngle(message->end_angle);

	laserscan[0].setNumPoints(message->scan_points);

	if (laserscan[0].getNumPoints() == 0)
		return;

	int num_of_points = laserscan[0].getNumPoints();

	int j = 0;
	double reference_v_angle = 0;
	for (int i = 0; i < num_of_points; i++)
	{
		vpScanPoint pt(message->arraypoints[i].radial_distance,
				message->arraypoints[i].horizontal_angle,
				message->arraypoints[i].vertical_angle,
				message->arraypoints[i].flags);

		if ((pt.getVAngle() != reference_v_angle))
		{
			reference_v_angle = pt.getVAngle();
			j++;
		}

		if (j == 1)
			laserscan[0].getScanPoints().push_back(pt);
		else if (j == 2)
			laserscan[1].getScanPoints().push_back(pt);
		else if (j == 3)
			laserscan[2].getScanPoints().push_back(pt);
		else if (j == 4)
			laserscan[3].getScanPoints().push_back(pt);
	}
}
