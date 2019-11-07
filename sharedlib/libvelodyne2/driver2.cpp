/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: driver.cc 2221 2012-03-25 23:12:32Z jack.oquin $
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include "driver2.h"

#include <string>
#include <stdlib.h>
#include <cmath>



namespace velodyne_driver
{

/*
VelodyneDriver::VelodyneDriver(int laser_udp_port, int gps_upd_port, carmen_velodyne_partial_scan_message &scan)
{
	scan.partial_scan = (carmen_velodyne_32_laser_shot *) malloc (velodyne_driver::VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION * sizeof(carmen_velodyne_32_laser_shot));
	scan_buffer_.partial_scan = (carmen_velodyne_32_laser_shot *) malloc (velodyne_driver::VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION * sizeof(carmen_velodyne_32_laser_shot));

	laser_input_ = new velodyne_driver::InputSocket(laser_udp_port);
	gps_input_ = new velodyne_driver::InputSocket(gps_upd_port);
}*/

VelodyneDriver::VelodyneDriver(carmen_velodyne_variable_scan_message &variable_scan, int velodyne_num_lasers, int velodyne_max_laser_shots_per_revolution, int velodyne_udp_port, int velodyne_gps_udp_port)
{
	//scan.partial_scan = (carmen_velodyne_32_laser_shot *) malloc (velodyne_driver::VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION * sizeof(carmen_velodyne_32_laser_shot));
	scan_buffer_.partial_scan = (carmen_velodyne_32_laser_shot *) malloc (velodyne_max_laser_shots_per_revolution * sizeof(carmen_velodyne_32_laser_shot));

	if (velodyne_num_lasers == 16)
	{
		variable_scan.partial_scan = (carmen_velodyne_shot*)malloc(2 * velodyne_max_laser_shots_per_revolution * sizeof(carmen_velodyne_shot));
		for (int i=0; i<(2 * velodyne_max_laser_shots_per_revolution); i++)
		{
			variable_scan.partial_scan[i].distance = (unsigned short*)malloc(velodyne_num_lasers*sizeof(unsigned short));
			variable_scan.partial_scan[i].intensity = (unsigned char*)malloc(velodyne_num_lasers*sizeof(unsigned char));
		}
	}
	else if (velodyne_num_lasers == 32)
	{
		variable_scan.partial_scan = (carmen_velodyne_shot*)malloc(velodyne_max_laser_shots_per_revolution * sizeof(carmen_velodyne_shot));
		for (int i=0; i<velodyne_max_laser_shots_per_revolution; i++)
		{
			variable_scan.partial_scan[i].distance = (unsigned short*)malloc(velodyne_num_lasers*sizeof(unsigned short));
			variable_scan.partial_scan[i].intensity = (unsigned char*)malloc(velodyne_num_lasers*sizeof(unsigned char));
		}
	}

	laser_input_ = new velodyne_driver::InputSocket(velodyne_udp_port);
	gps_input_ = new velodyne_driver::InputSocket(velodyne_gps_udp_port);
}

void VelodyneDriver::copy_packet_to_scan_buffer(int i, int j, const velodyne_packet_t& packet, int velodyne_num_lasers)
{
	for (int k = 0; k < velodyne_driver::VELODYNE_NUM_LASERS; k++)
	{
		scan_buffer_.partial_scan[i].distance[k] = packet.shots[j].lasers[k].distance;
		scan_buffer_.partial_scan[i].intensity[k] = packet.shots[j].lasers[k].intensity;
	}
	scan_buffer_.partial_scan[i].angle = packet.shots[j].rotational_pos / 100.0;
}

//funcao para laser de 32 raios
/*void VelodyneDriver::copy_scan_buffer_to_scan(carmen_velodyne_partial_scan_message &scan, int l, int m)
{
	for (int k = 0; k < velodyne_driver::VELODYNE_NUM_LASERS; k++)
	{
		scan.partial_scan[l].distance[k] = scan_buffer_.partial_scan[m].distance[k];
		scan.partial_scan[l].intensity[k] = scan_buffer_.partial_scan[m].intensity[k];
	}
	scan.partial_scan[l].angle = scan_buffer_.partial_scan[m].angle;
}*/

//funcao para laser de 16 ou 32 raios (verifica pela variavel velodyne_num_lasers
void VelodyneDriver::copy_scan_buffer_to_scan(int l, int m, carmen_velodyne_variable_scan_message &variable_scan, int velodyne_num_lasers)
{
	if (velodyne_num_lasers == 16)
	{
		for (int k = 0; k < velodyne_driver::VELODYNE_NUM_LASERS; k++)
		{
			if (k < velodyne_num_lasers)
			{
				variable_scan.partial_scan[2*l].distance[k] = scan_buffer_.partial_scan[m].distance[k];
				variable_scan.partial_scan[2*l].intensity[k] = scan_buffer_.partial_scan[m].intensity[k];
				variable_scan.partial_scan[2*l].shot_size = velodyne_num_lasers;
			}
			else
			{
				variable_scan.partial_scan[2*l + 1].distance[k-velodyne_num_lasers] = scan_buffer_.partial_scan[m].distance[k];
				variable_scan.partial_scan[2*l + 1].intensity[k-velodyne_num_lasers] = scan_buffer_.partial_scan[m].intensity[k];
				variable_scan.partial_scan[2*l + 1].shot_size = velodyne_num_lasers;
			}
		}
		variable_scan.partial_scan[2*l].angle = scan_buffer_.partial_scan[m].angle;
	}
	else if (velodyne_num_lasers == 32)
	{
		for (int k = 0; k < velodyne_driver::VELODYNE_NUM_LASERS; k++)
		{
			variable_scan.partial_scan[l].distance[k] = scan_buffer_.partial_scan[m].distance[k];
			variable_scan.partial_scan[l].intensity[k] = scan_buffer_.partial_scan[m].intensity[k];
			variable_scan.partial_scan[l].shot_size = velodyne_num_lasers;
		}
		variable_scan.partial_scan[l].angle = scan_buffer_.partial_scan[m].angle;
	}

}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
//para laser de 32 raios
/*
bool VelodyneDriver::pollScan(carmen_velodyne_partial_scan_message &scan, int velodyne_number)
{
	static int i = -1;
	static int begin_previous_scan_i;
	static bool scan_buffer_complete = false;
	static bool first_time = true;
	int fraction_of_VELODYNE_NUM_SHOTS = 0;
	double last_package_timestamp;

	while (true)
	{
		// Fill in the scan_buffer
		velodyne_packet_t packet;
		int rc = laser_input_->getScanPacket(&packet, velodyne_number);
		last_package_timestamp = carmen_get_time();

		if (rc < 0)
			return false; // end of file reached?

		for (int j = 0; j < velodyne_driver::VELODYNE_NUM_SHOTS; j++)
		{
			// Compute previous_i. It is equal -1 only once.
			int previous_i = i;
			i+=1;
			if (i >= velodyne_driver::VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION)
				i = 0;

			copy_packet_to_scan_buffer(i, j, packet);

			if ((previous_i != -1) &&
				(scan_buffer_.partial_scan[i].angle >= 180.0) && (scan_buffer_.partial_scan[previous_i].angle < 180.0))
			{
				if (!first_time)
				{
					scan_buffer_complete = true;
				}
				else  // During the first scan just after Velodyne initialization, wait until complete a full scan
				{
					// This "else" happen only once
					begin_previous_scan_i = i;
					first_time = false;
				}
				fraction_of_VELODYNE_NUM_SHOTS = j;
			}
		}

		// When scan_buffer is complete, copy it to scan
		int l;
		if (scan_buffer_complete)
		{
			int m = begin_previous_scan_i;
			for (l = 0; l < velodyne_driver::VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION; l++)
			{
				copy_scan_buffer_to_scan(scan, l, m);

				int previous_m = m;
				m++;
				if (m >= velodyne_driver::VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION)
					m = 0;
				if ((scan_buffer_.partial_scan[m].angle >= 180.0) && (scan_buffer_.partial_scan[previous_m].angle < 180.0))
					break;
			}

			scan.number_of_32_laser_shots = l < velodyne_driver::VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION ?  l + 1 : l; // quando liga o velodyne ele pode estar muito lento
			scan.timestamp = last_package_timestamp -
					((double) (velodyne_driver::VELODYNE_NUM_SHOTS - 1 - fraction_of_VELODYNE_NUM_SHOTS) /
					 (double) (velodyne_driver::VELODYNE_PACKAGE_RATE * velodyne_driver::VELODYNE_NUM_SHOTS));

			begin_previous_scan_i = m;
			scan_buffer_complete = false;

			return true;
		}
	}

	return true;
}*/

/** poll the device
 *
 *  @returns true unless end of file reached
 */
//para laser de raios variados
bool VelodyneDriver::pollScan(carmen_velodyne_variable_scan_message &variable_scan, int velodyne_number, int velodyne_udp_port,
							  int velodyne_max_laser_shots_per_revolution, int velodyne_num_shots, double velodyne_package_rate,
							  int velodyne_num_lasers)
{
	static int i = -1;
	static int begin_previous_scan_i;
	static bool scan_buffer_complete = false;
	static bool first_time = true;
	int fraction_of_VELODYNE_NUM_SHOTS = 0;
	double last_package_timestamp;

	while (true)
	{
		// Fill in the scan_buffer
		velodyne_packet_t packet;
		int rc = laser_input_->getScanPacket(&packet, velodyne_udp_port);
		last_package_timestamp = carmen_get_time();

		if (rc < 0)
			return false; // end of file reached?

		for (int j = 0; j < velodyne_num_shots; j++)
		{
			// Compute previous_i. It is equal -1 only once.
			int previous_i = i;
			i+=1;
			if (i >= velodyne_max_laser_shots_per_revolution)
				i = 0;

			copy_packet_to_scan_buffer(i, j, packet, velodyne_num_lasers);

			if ((previous_i != -1) &&
				(scan_buffer_.partial_scan[i].angle >= 180.0) && (scan_buffer_.partial_scan[previous_i].angle < 180.0))
			{
				if (!first_time)
				{
					scan_buffer_complete = true;
				}
				else  // During the first scan just after Velodyne initialization, wait until complete a full scan
				{
					// This "else" happen only once
					begin_previous_scan_i = i;
					first_time = false;
				}
				fraction_of_VELODYNE_NUM_SHOTS = j;
			}
		}

		// When scan_buffer is complete, copy it to scan
		int l;
		if (scan_buffer_complete)
		{
			int m = begin_previous_scan_i;
			for (l = 0; l < velodyne_max_laser_shots_per_revolution; l++)
			{
				copy_scan_buffer_to_scan(l, m, variable_scan, velodyne_num_lasers);

				int previous_m = m;
				m++;
				if (m >= velodyne_max_laser_shots_per_revolution)
					m = 0;
				if ((scan_buffer_.partial_scan[m].angle >= 180.0) && (scan_buffer_.partial_scan[previous_m].angle < 180.0))
					break;
			}

			//faz o calculo dos angulos intermediarios entre dois disparos
			if (velodyne_num_lasers == 16)
			{
				for (int j = 1; j < l*2; j+=2)
				{
					if (variable_scan.partial_scan[j+1].angle - variable_scan.partial_scan[j-1].angle < 0) //este if só executa uma vez. Devido ao angulo de volta completa ser proximo a 360 e o próximo proximo a 0, a media nao corresponderá ao angulo intermediario, por isso, somo 360 ao angulo menor, calculo a media, e desconto 360 se o resultado for maior que 360
					{
						variable_scan.partial_scan[j].angle = (variable_scan.partial_scan[j-1].angle + variable_scan.partial_scan[j+1].angle + 360) / 2.0;
						if (variable_scan.partial_scan[j].angle >= 360.0)
							variable_scan.partial_scan[j].angle = variable_scan.partial_scan[j].angle - 360.0;
					}
					else
						variable_scan.partial_scan[j].angle = (variable_scan.partial_scan[j-1].angle + variable_scan.partial_scan[j+1].angle) / 2.0;
				}
			}

			/*scan.number_of_32_laser_shots = l < velodyne_driver::VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION ?  l + 1 : l; // quando liga o velodyne ele pode estar muito lento
			scan.timestamp = last_package_timestamp -
					((double) (velodyne_driver::VELODYNE_NUM_SHOTS - 1 - fraction_of_VELODYNE_NUM_SHOTS) /
					 (double) (velodyne_driver::VELODYNE_PACKAGE_RATE * velodyne_driver::VELODYNE_NUM_SHOTS));*/

			int numberOfShots = l < velodyne_max_laser_shots_per_revolution ?  l + 1 : l; // quando liga o velodyne ele pode estar muito lento

			if (velodyne_num_lasers == 32)
				variable_scan.number_of_shots = numberOfShots;
			else if (velodyne_num_lasers == 16)
				variable_scan.number_of_shots = (2 * numberOfShots) - 1;

			double timestamp = last_package_timestamp -
					((double) (velodyne_num_shots - 1 - fraction_of_VELODYNE_NUM_SHOTS) /
					 (double) (velodyne_package_rate * velodyne_num_shots));

			variable_scan.timestamp = timestamp;

			begin_previous_scan_i = m;
			scan_buffer_complete = false;

			return true;
		}
	}

	return true;
}

bool VelodyneDriver::pollGps(int velodyne_gps_udp_port)
{
	while (true)
	{
		// keep reading until full packet received
		int rc = gps_input_->getGpsPacket(&gps_.packet, velodyne_gps_udp_port);
		if (rc == 0) break;       // got a full packet?
		if (rc < 0) return false; // end of file reached?
	}

	gps_.timestamp = carmen_get_time();  //get the last shot timestamp

	return true;
}

void VelodyneDriver::printVelodyneGps()
{
	gps_input_->printGpsPacket(gps_.packet);
}

void VelodyneDriver::printVelodyneScan(carmen_velodyne_partial_scan_message scan)
{
	for(int i=0; i < scan.number_of_32_laser_shots; i++)
	{
		printf("angle (%d): %6.2f\n", i, scan.partial_scan[i].angle);

		//printf("data :");
		//for(int j=0; j < 32; j++)
		//	printf("%d (%d) ", scan_.laser_shots[i].distance[j], scan_.laser_shots[i].intensity[j]);
		//printf("\n");
	}
}

VelodyneDriver::~VelodyneDriver()
{
	free(scan_buffer_.partial_scan);

	if (laser_input_ != NULL)
		laser_input_->~InputSocket();
}

} // namespace velodyne_driver
