/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include <string.h>
#include <stdlib.h>
#include "rsdriver.h"

namespace rslidar_driver
{
  static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 18000;
  static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;
//  carmen_velodyne_variable_scan_message &variable_scan, int num_lasers, int velodyne_max_laser_shots_per_revolution, int velodyne_udp_port, int velodyne_gps_udp_port
rslidarDriver::rslidarDriver(carmen_velodyne_variable_scan_message &variable_scan, int num_lasers, int velodyne_max_laser_shots_per_revolution, rslidar_param &private_nh)
{
	shots_array = (carmen_velodyne_shot *) malloc (MAX_NUM_SHOTS * sizeof(carmen_velodyne_shot));
	if (num_lasers == 16)
	{
		for (int i = 0 ; i < MAX_NUM_SHOTS; i++)  // TODO move to initialization function
		{
			shots_array[i].shot_size = RS16_SCANS_PER_FIRING;
			shots_array[i].distance = (unsigned short*) malloc (RS16_SCANS_PER_FIRING * sizeof(unsigned short));
			shots_array[i].intensity = (unsigned char*) malloc (RS16_SCANS_PER_FIRING * sizeof(unsigned char));
		}

	}

	sleep(2.0);
  skip_num_ = 0;
  //dual or
  return_mode_ = 1;
  //resolution meters 0.5 or 1.0
  dis_resolution_mode_ = 0;
  // use private node handle to get parameters
  // get model name, validate string, determine packet rate
  config_.model = private_nh.model;
  double packet_rate;  // packet frequency (Hz)
  std::string model_full_name;
  // product model
  if (config_.model == "RS16")
  {
    //for 0.18 degree horizontal angle resolution
    //packet_rate = 840;
    //for 0.2 degree horizontal angle resolution
    packet_rate = 750;
    model_full_name = "RS-LiDAR-16";
  }
  else if (config_.model == "RS32")
  {
    //for 0.18 degree horizontal angle resolution
    //packet_rate = 1690;
    //for 0.2 degree horizontal angle resolution
    packet_rate = 1500;
    model_full_name = "RS-LiDAR-32";
  }
  else if (config_.model == "RSBPEARL")
  {
    packet_rate = 1500;
    model_full_name = "RSBPEARL";
  }
  else
  {
    printf("[driver] unknown LIDAR model: %s\n", config_.model.c_str());
    packet_rate = 2600.0;
  }
  std::string deviceName(std::string("Robosense ") + model_full_name);

  if (private_nh.rpm == 0)
	  config_.rpm =  600.0;
  else
	  config_.rpm = private_nh.rpm;
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)

  int npackets = (int)ceil(packet_rate / frequency);
  config_.npackets = npackets;
  private_nh.npackets = config_.npackets;

  printf("[driver] publishing %d packets per scan\n", config_.npackets);

//default is MSOP_DATA_PORT_NUMBER
  int msop_udp_port = private_nh.msop_port;

//default is DIFOP_DATA_PORT_NUMBER
  int difop_udp_port =  private_nh.difop_udp_port;

  //default is -0.01
  double cut_angle;

  cut_angle = private_nh.cut_angle;
  if (cut_angle < 0.0)
    printf("[driver] Cut at specific angle feature deactivated. \n");
  else if (cut_angle < 360)
    printf("[driver] Cut at specific angle feature activated.\n Cutting rslidar points always at %lf degree.\n", cut_angle);
  else
  {
    printf("[driver] cut_angle parameter is out of range. Allowed range is between 0.0 and 360 negative values to deactivate this feature.");
    cut_angle = -0.01;
  }

  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in rslidar packets
  config_.cut_angle = static_cast<int>(cut_angle * 100);

  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;

  // read data from live socket
  msop_input_ = new rslidar_driver::InputSocket(private_nh, msop_udp_port);
  difop_input_ = new rslidar_driver::InputSocket(private_nh, difop_udp_port);

  // raw packet output topic Montando o Publisher do ROS
//  std::string output_packets_topic;
//  private_nh.param("output_packets_topic", output_packets_topic, std::string("rslidar_packets"));
//  msop_output_ = node.advertise<rslidar_msgs::rslidarScan>(output_packets_topic, 10);

//  std::string output_difop_topic;  Montando o Publisher do ROS
//  private_nh.param("output_difop_topic", output_difop_topic, std::string("rslidar_packets_difop"));
//  difop_output_ = node.advertise<rslidar_msgs::rslidarPacket>(output_difop_topic, 10);

//  Não vou usar thread, apagar depois de tirar as dependencias
//  difop_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&rslidarDriver::difopPoll, this)));

}


/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void
rslidarDriver::unpack(int *shot_num, const rslidarPacket& pkt, carmen_velodyne_shot *variable_scan)
{

  float azimuth;  // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[42];

  for (int block = 0; block < BLOCKS_PER_PACKET; block++)  // 1 packet:12 data blocks
  {

	  azimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

    if (block < (BLOCKS_PER_PACKET - 1))  // 12
    {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block + 1].rotation_1 + raw->blocks[block + 1].rotation_2;
      azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
    }
    else
    {
      int azi1, azi2;
      azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
      azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
      azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
    }

    for (int firing = 0, k = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++)  // 2
    {

      for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
      {
        if (0 == return_mode_)
        {
          azimuth_corrected_f = azimuth + (azimuth_diff * (dsr * RS16_DSR_TOFFSET)) / RS16_FIRING_TOFFSET;
        }
        else
        {
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) /
                                           RS16_BLOCK_TDURATION);
        }
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;  // convert to integral value...

        union two_bytes tmp;
        tmp.bytes[1] = raw->blocks[block].data[k];
        tmp.bytes[0] = raw->blocks[block].data[k + 1];
        int distance = tmp.uint;

        // read intensity
        intensity = raw->blocks[block].data[k + 2];
//        if (Curvesis_new)
//          intensity = calibrateIntensity(intensity, dsr, distance);
//        else
//          intensity = calibrateIntensity_old(intensity, dsr, distance);

        float distance2 = distance;//pixelToDistance(distance, dsr);
        if (dis_resolution_mode_ == 0)  // distance resolution is 0.5cm
        {
          distance2 = distance2 * DISTANCE_RESOLUTION_NEW;
        }
        else
        {
          distance2 = distance2 * DISTANCE_RESOLUTION;
        }

        int arg_horiz = (azimuth_corrected + 36000) % 36000;
        int arg_horiz_orginal = arg_horiz;
        //angulo da mensagem tem que ser o azimuth para o primeiro firing e com a correcao para o segundo firing
        //talvez tem que dividir por 100 (aqui logo)
        variable_scan[*shot_num].angle = (azimuth/100.0f);
        variable_scan[*shot_num].shot_size = RS16_SCANS_PER_FIRING;
        variable_scan[*shot_num].distance[dsr] = (unsigned short) distance2;
        variable_scan[*shot_num].intensity[dsr] = (unsigned char) intensity;
      }
      (*shot_num)++;
    }
  }
}


/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
bool
rslidarDriver::unpack_socket_data_to_velodyne_shot(int *shot_num, const rslidarPacket& pkt, carmen_velodyne_shot *shots_array)
{
	int last_byte_of_shot;
	bool complete_turn = false;
	carmen_velodyne_shot* current_shot = &shots_array[*shot_num];

	for (int i = 44; i < 1248; i += 100) // i + 44 to skip the header and the block identifier
	{
		current_shot->angle = (double)((256 * pkt.data[i] + pkt.data[i + 1]) / 100.0);

		if (*shot_num > 0 && current_shot->angle < 5.0 && shots_array[*shot_num - 2].angle < 358.0)
			complete_turn = true;
		printf("Azimuth data %02X %02X %f\n", pkt.data[i], pkt.data[i + 1], current_shot->angle);

		last_byte_of_shot = i + 2 + 48; // (RS16_SCANS_PER_FIRING * 3) 3 is the number of bytes per channel (each ray measure) data (see manual)

		for (int ray = i + 2, k = 0; ray < last_byte_of_shot; ray += 3, k++)  // i + 2 to skip the
		{
			memcpy(&current_shot->distance[k], &pkt.data[ray], sizeof(unsigned short));
			//printf("  Distance data: %02X %02X %04X\n", pkt.data[ray], pkt.data[ray + 1], current_shot->distance[k]);
			current_shot->intensity[k] = (unsigned char) pkt.data[ray + 2];
		}

		*shot_num += 1;
		current_shot = &shots_array[*shot_num];

		//current_shot->angle = -1; // TODO remove if not used

		for (int ray = i + 52, k = 0; ray < 100; ray += 3, k++)  // 52 is the first byte of the second channel of the current block and 100 is the last byte of the current block
		{
			memcpy(&current_shot->distance[k], &pkt.data[ray], sizeof(unsigned short));
			//printf("  Distance data: %02X %02X %04X\n", pkt.data[ray], pkt.data[ray + 1], current_shot->distance[k]);
			current_shot->intensity[k] = (unsigned char) pkt.data[ray + 2];
		}
	}
	return (complete_turn);
}


/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool rslidarDriver::poll(carmen_velodyne_variable_scan_message &variable_scan, int num_lasers, int velodyne_max_laser_shots_per_revolution, int velodyne_udp_port, int velodyne_gps_udp_port, rslidar_param &private_nh)
{  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
	rslidarScan_t *scan = (rslidarScan_t *) calloc (1, sizeof(rslidarScan_t));
	scan->packets = (rslidarPacket*)malloc(velodyne_max_laser_shots_per_revolution * sizeof(rslidarPacket));
	//	memset(scan)

	// Since the rslidar delivers data at a very high rate, keep
	// reading and publishing scans as fast as possible.
	if (config_.cut_angle >= 0)  // Cut at specific angle feature enabled
	{
		rslidarPacket tmp_packet;
		int read_packets = 0;
		while (true)
		{
			double time1 = carmen_get_time();

			while (true)
			{
				int rc = msop_input_->getPacket(&tmp_packet, config_.time_offset);
				if (rc == 0)
					break;  // got a full packet?
				if (rc < 0)
					return false;  // end of file reached?
			}
			printf("Time: %lf\n", carmen_get_time() - time1);
			//Unpack to velodyne_shot struct
			int num_shot = 0;
			unpack(&num_shot, tmp_packet, shots_array);

			scan->packets[read_packets] = (tmp_packet);
			read_packets++;

			static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
			static int last_azimuth = ANGLE_HEAD;

			int azimuth = 256 * tmp_packet.data[44] + tmp_packet.data[45];
			// int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

			// Handle overflow 35999->0
			if (azimuth < last_azimuth)
			{
				last_azimuth -= 36000;
			}
			// Check if currently passing cut angle
			if (last_azimuth != ANGLE_HEAD && last_azimuth < config_.cut_angle && azimuth >= config_.cut_angle)
			{
				last_azimuth = azimuth;
				config_.npackets = read_packets;
				break;  // Cut angle passed, one full revolution collected
				printf("Last angle: %lf\n", last_azimuth);
				exit(0);
			}
			last_azimuth = azimuth;

		}
	}
	else  // standard behaviour
	{
		if (difop_input_->getUpdateFlag())
		{
			int packets_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND/BLOCKS_ONE_CHANNEL_PER_PKT);
			int mode = difop_input_->getReturnMode();
			if (config_.model == "RS16" && (mode == 1 || mode == 2))
			{
				packets_rate = ceil(packets_rate/2);
			}
			else if ((config_.model == "RS32" || config_.model == "RSBPEARL") && (mode == 0))
			{
				packets_rate = packets_rate*2;
			}
			config_.rpm = difop_input_->getRpm();
			config_.npackets = ceil(packets_rate*60/config_.rpm);

			difop_input_->clearUpdateFlag();

			printf("[driver] update npackets. rpm: %lf packets: %d", config_.rpm, config_.npackets);
		}
		//    if(config_.npackets != private_nh.npackets)
		//    	scan->packets = (rslidarPacket *)realloc(scan->packets, config_.npackets*sizeof(rslidarPacket));
		//    else
		//    	scan->packets = (rslidarPacket*)malloc(config_.npackets * sizeof(rslidarPacket));
		// use in standard behaviour only
		while (skip_num_)
		{
			while (true)
			{
				// keep reading until full packet received
				int rc = msop_input_->getPacket(&scan->packets[0], config_.time_offset);
				if (rc == 0)
					break;  // got a full packet?
				if (rc < 0)
					return false;  // end of file reached?
			}
			--skip_num_;
		}

		for (int i = 0; i < config_.npackets; ++i)
		{
			while (true)
			{
				// keep reading until full packet received
				int rc = msop_input_->getPacket(&scan->packets[i], config_.time_offset);
				if (rc == 0)
					break;  // got a full packet?
				if (rc < 0)
					return false;  // end of file reached?
			}
		}

	}
	int j = 0;
	for (int i = 0; i < config_.npackets; ++i)
		unpack(&j, scan->packets[i], &variable_scan);
	variable_scan.number_of_shots = velodyne_max_laser_shots_per_revolution;
	variable_scan.timestamp = carmen_get_time();

	//  printf("to funcionando \n");

	// publish message using time of last packet read
	//  ROS_DEBUG("[driver] Publishing a full rslidar scan.");
	//TODO converter para carmen message
	//  scan->header.stamp = scan->packets.back().stamp;
	//  scan->header.frame_id = config_.frame_id;
	//  msop_output_.publish(scan);
	//  free(scan);
	return true;
}


void
fill_carmen_velodyne_variable_scan_message(carmen_velodyne_variable_scan_message &variable_scan, carmen_velodyne_shot *shots_array, int num_shot)
{
//	variable_scan.partial_scan = (carmen_velodyne_shot *)malloc(sizeof(carmen_velodyne_shot)); //ja foi alocada no construtor! verificar se eh melhor realocar
	int end_of_scan = 0;
	for (int i = 0; i < num_shot - 1; i++)
	{
		if ((i+1) % 2 == 0)//interpolate angle //testar o rollover --fazer a media se der menor que zero soma 360
		{
			double previous_angle = shots_array[i-1].angle;
			double next_angle = shots_array[i+1].angle;

			if (next_angle < previous_angle) //Ajust for a rollover from 359.99 to 0 -> verficar se eh necessario
				next_angle = next_angle + 360.0;

			shots_array[i].angle = previous_angle + ((next_angle - previous_angle) / 2.0); //interpolate

			if (shots_array[i].angle > 360.0) //correct for any rollover over form 359.99 to 0
				shots_array[i].angle = shots_array[i].angle - 360.0;
		}

		if (shots_array[i].angle > 360.0)
			end_of_scan = i;
	}

	variable_scan.number_of_shots = num_shot;
	variable_scan.partial_scan = (carmen_velodyne_shot*)malloc(num_shot * sizeof(carmen_velodyne_shot));
//	variable_scan.partial_scan[i].intensity = (unsigned char*)malloc(num_lasers*sizeof(unsigned char));
//	variable_scan.partial_scan[i].distance = (unsigned short*)malloc(num_lasers*sizeof(unsigned short));

	for (int i=0; i<(velodyne_max_laser_shots_per_revolution); i++)
	{
		memcpy(&variable_scan.partial_scan[i], &shots_array[i], sizeof(shots_array[i]));
	}

}


bool
rslidarDriver::receive_socket_data_and_fill_message(carmen_velodyne_variable_scan_message &variable_scan, int num_lasers, int velodyne_max_laser_shots_per_revolution, int velodyne_udp_port, int velodyne_gps_udp_port, rslidar_param &private_nh)
{
//	carmen_velodyne_shot shots_array[MAX_NUM_SHOTS]; //criado no construtor, devemos precisar manter ela viva
	rslidarPacket tmp_packet;
	static int num_shot = 0;

	static int previous_index_buffer;

	while (true)
	{
		double time1 = carmen_get_time();

		while (true)
		{
			int rc = msop_input_->getPacket(&tmp_packet, config_.time_offset);
			if (rc == 0)
				break;  // got a full packet?
			if (rc < 0)
				return false;  // end of file reached?
		}
		//printf("Time: %lf\n", carmen_get_time() - time1);

		if (unpack_socket_data_to_velodyne_shot(&num_shot, tmp_packet, shots_array))
		{
			fill_carmen_velodyne_variable_scan_message(variable_scan, shots_array, num_shot);
			break;
		}
	}
	printf("Completou\n");
	return true;
}

//void rslidarDriver::difopPoll(void)
//{
//  // reading and publishing scans as fast as possible.
//  rslidarPacket_t* difop_packet_ptr;
//  while (ros::ok())
//  {
//    // keep reading
//    rslidar_msgs::rslidarPacket difop_packet_msg;
//    int rc = difop_input_->getPacket(&difop_packet_msg, config_.time_offset);
//    if (rc == 0)
//    {
////      ROS_DEBUG("[driver] Publishing a difop data.");
//      *difop_packet_ptr = difop_packet_msg;
//      difop_output_.publish(difop_packet_ptr);
//    }
//    if (rc < 0)
//      return;  // end of file reached?
//    ros::spinOnce();
//  }
//}

}  // namespace rslidar_driver
