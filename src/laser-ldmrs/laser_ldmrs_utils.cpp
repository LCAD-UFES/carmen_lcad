/*
 * laser_ldmrs_utils.cpp
 *
 *  Created on: Nov 10, 2016
 *      Author: vinicius
 */

#include "laser_ldmrs_utils.h"


carmen_velodyne_partial_scan_message
carmen_laser_ldmrs_convert_laser_scan_to_partial_velodyne_message(vpLaserScan laserscan[4], double laserscan_timestamp)
{
	carmen_velodyne_partial_scan_message velodyne_message;
	double angular_resolution = 0.5;
	int i;
	int camera = 1;

	double start_angle_deg = carmen_radians_to_degrees(2*M_PI*laserscan[0].getStartAngle() /(double) laserscan[0].getNumSteps());
	double end_angle_deg = carmen_radians_to_degrees(2*M_PI*laserscan[0].getStopAngle() /(double) laserscan[0].getNumSteps());
	int number_of_shots = (start_angle_deg - end_angle_deg) / angular_resolution;


	velodyne_message.number_of_32_laser_shots = number_of_shots;
	velodyne_message.partial_scan = (carmen_velodyne_32_laser_shot *) calloc (velodyne_message.number_of_32_laser_shots, sizeof(carmen_velodyne_32_laser_shot));
	velodyne_message.timestamp = laserscan_timestamp;
	velodyne_message.host = carmen_get_host();


	// alocacao
	for (int i = 0; i < velodyne_message.number_of_32_laser_shots; i++)
	{
		velodyne_message.partial_scan[i].angle = start_angle_deg + angular_resolution * i;
	}

	std::vector<vpScanPoint> pointsInLayer1 = laserscan[0].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer2 = laserscan[1].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer3 = laserscan[2].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer4 = laserscan[3].getScanPoints();

	int sizeLayer1 = pointsInLayer1.size();
	int sizeLayer2 = pointsInLayer2.size();
	int sizeLayer3 = pointsInLayer3.size();
	int sizeLayer4 = pointsInLayer4.size();

	int index = 0;

	for(i = 0; i < sizeLayer1; i++)
	{
		index = (int) round(abs((carmen_radians_to_degrees(pointsInLayer1[i].getHAngle()) - start_angle_deg) / angular_resolution));
		velodyne_message.partial_scan[index].distance[0] = (unsigned short) (pointsInLayer1[i].getRadialDist() * 500.0);
		velodyne_message.partial_scan[index].intensity[0] = 255;
	}

	for(i = 0; i < sizeLayer2; i++)
	{
		index = (int) round(abs((carmen_radians_to_degrees(pointsInLayer2[i].getHAngle()) - start_angle_deg) / angular_resolution));
		velodyne_message.partial_scan[index].distance[1] = (unsigned short) (pointsInLayer2[i].getRadialDist() * 500.0);
		velodyne_message.partial_scan[index].intensity[1] = 255;
	}

	for(i = 0; i < sizeLayer3; i++)
	{
		index = (int) round(abs((carmen_radians_to_degrees(pointsInLayer3[i].getHAngle()) - start_angle_deg - 0.25) / angular_resolution));
		velodyne_message.partial_scan[index].distance[2] = (unsigned short) (pointsInLayer3[i].getRadialDist() * 500.0);
		velodyne_message.partial_scan[index].intensity[2] = 255;
	}

	for(i = 0; i < sizeLayer4; i++)
	{
		index = (int) round(abs((carmen_radians_to_degrees(pointsInLayer4[i].getHAngle()) - start_angle_deg - 0.25) / angular_resolution));
		velodyne_message.partial_scan[index].distance[3] = (unsigned short) (pointsInLayer4[i].getRadialDist() * 500.0);
		velodyne_message.partial_scan[index].intensity[3] = 255;
	}

	return velodyne_message;
}


void
carmen_laser_ldmrs_copy_laser_scan_to_message(vpLaserScan laserscan[4], carmen_laser_ldmrs_message *message)
{
	message->scan_number = laserscan[0].getMeasurementId();
	message->scan_start_time = laserscan[0].getStartTimestamp();
	message->scan_end_time = laserscan[0].getEndTimestamp();
	message->angle_ticks_per_rotation = laserscan[0].getNumSteps();
	message->start_angle = laserscan[0].getStartAngle();
	message->end_angle = laserscan[0].getStopAngle();

	if(laserscan[0].getNumPoints() == 0) {
		return;
	}

	int num_of_points = laserscan[0].getNumPoints();

	std::vector<vpScanPoint> pointsInLayer1 = laserscan[0].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer2 = laserscan[1].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer3 = laserscan[2].getScanPoints();
	std::vector<vpScanPoint> pointsInLayer4 = laserscan[3].getScanPoints();

	int sizeLayer1 = pointsInLayer1.size();
	int sizeLayer2 = pointsInLayer2.size();
	int sizeLayer3 = pointsInLayer3.size();
	int sizeLayer4 = pointsInLayer4.size();

	if(num_of_points > (sizeLayer1 + sizeLayer2 +sizeLayer3 + sizeLayer4)){
		num_of_points = sizeLayer1 + sizeLayer2 +sizeLayer3 + sizeLayer4;
	}

	if(message->scan_points != num_of_points)
	{
		message->scan_points = num_of_points;
		message->arraypoints = (carmen_laser_ldmrs_point *)realloc(message->arraypoints, message->scan_points * sizeof(carmen_laser_ldmrs_point));
		carmen_test_alloc(message->arraypoints);
	}

	int i;
	for(i = 0; i < sizeLayer1; i++)
	{
		message->arraypoints[i].horizontal_angle = pointsInLayer1[i].getHAngle();
		message->arraypoints[i].vertical_angle = pointsInLayer1[i].getVAngle();
		message->arraypoints[i].radial_distance = pointsInLayer1[i].getRadialDist();
		message->arraypoints[i].flags = pointsInLayer1[i].getFlags();
	}

	for(i = 0; i < sizeLayer2; i++)
	{
		message->arraypoints[i + sizeLayer1].horizontal_angle = pointsInLayer2[i].getHAngle();
		message->arraypoints[i + sizeLayer1].vertical_angle = pointsInLayer2[i].getVAngle();
		message->arraypoints[i + sizeLayer1].radial_distance = pointsInLayer2[i].getRadialDist();
		message->arraypoints[i + sizeLayer1].flags = pointsInLayer2[i].getFlags();
	}

	for(i = 0; i < sizeLayer3; i++)
	{
		message->arraypoints[i + sizeLayer1 + sizeLayer2].horizontal_angle = pointsInLayer3[i].getHAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].vertical_angle = pointsInLayer3[i].getVAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].radial_distance = pointsInLayer3[i].getRadialDist();
		message->arraypoints[i + sizeLayer1 + sizeLayer2].flags = pointsInLayer3[i].getFlags();

	}

	for(i = 0; i < sizeLayer4; i++)
	{
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].horizontal_angle = pointsInLayer4[i].getHAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].vertical_angle = pointsInLayer4[i].getVAngle();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].radial_distance = pointsInLayer4[i].getRadialDist();
		message->arraypoints[i + sizeLayer1 + sizeLayer2 + sizeLayer3].flags = pointsInLayer4[i].getFlags();
	}
}


void
carmen_laser_ldmrs_copy_message_to_laser_scan(carmen_laser_ldmrs_message *message, vpLaserScan laserscan[4])
{
	laserscan[0].setMeasurementId(message->scan_number);
	laserscan[0].setStartTimestamp(message->scan_start_time);
	laserscan[0].setEndTimestamp(message->scan_end_time);
	laserscan[0].setNumSteps(message->angle_ticks_per_rotation);
	laserscan[0].setStartAngle(message->start_angle);
	laserscan[0].setStopAngle(message->end_angle);

	laserscan[0].setNumPoints(message->scan_points);

	if(laserscan[0].getNumPoints() == 0) {
		return;
	}

	int num_of_points = laserscan[0].getNumPoints();

	int j = 0;
	double reference_v_angle = 0;
	for(int i = 0; i < num_of_points; i++)
	{
		vpScanPoint pt(message->arraypoints[i].radial_distance,
				message->arraypoints[i].horizontal_angle,
				message->arraypoints[i].vertical_angle,
				message->arraypoints[i].flags);


		if((pt.getVAngle() != reference_v_angle))
		{
			reference_v_angle = pt.getVAngle();
			j++;
		}

		if(j == 1)
			laserscan[0].getScanPoints().push_back(pt);
		else if(j == 2)
			laserscan[1].getScanPoints().push_back(pt);
		else if (j == 3)
			laserscan[2].getScanPoints().push_back(pt);
		else if (j == 4)
			laserscan[3].getScanPoints().push_back(pt);
	}
}



