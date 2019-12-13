/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the RSLIDAR 3D LIDARs
 */
#ifndef _RSDRIVER_H_
#define _RSDRIVER_H_

#include <string>
#include "input.h"

namespace rslidar_driver
{
typedef struct rslidarScan
{
	unsigned char status_timestamp[4];
	rslidarPacket *packets;
}  __attribute__((packed)) rslidarScan_t ;

// static const float  ROTATION_SOLUTION_ = 0.18f;  //水平角分辨率 10hz
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96

static const float ROTATION_RESOLUTION = 0.01f;   /**< degrees 旋转角分辨率*/
static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

static const float DISTANCE_MAX = 200.0f;            /**< meters */
static const float DISTANCE_MIN = 0.4f;              /**< meters */
static const float DISTANCE_RESOLUTION = 0.01f;      /**< meters */
static const float DISTANCE_RESOLUTION_NEW = 0.005f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);
/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;  //
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for RS16 support **/
static const int RS16_FIRINGS_PER_BLOCK = 2;
static const int RS16_SCANS_PER_FIRING = 16;
static const float RS16_BLOCK_TDURATION = 100.0f;  // [µs]
static const float RS16_DSR_TOFFSET = 3.0f;        // [µs]
static const float RS16_FIRING_TOFFSET = 50.0f;    // [µs]

/** Special Defines for RS32 support **/
static const int RS32_FIRINGS_PER_BLOCK = 1;
static const int RS32_SCANS_PER_FIRING = 32;
static const float RS32_BLOCK_TDURATION = 50.0f;  // [µs]
static const float RS32_DSR_TOFFSET = 3.0f;       // [µs]
static const float RL32_FIRING_TOFFSET = 50.0f;   // [µs]

static const int TEMPERATURE_MIN = 31;

#define RS_TO_RADS(x) ((x) * (M_PI) / 180)
/** \brief Raw rslidar data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
// block
typedef struct raw_block
{
  uint16_t header;  ///< UPPER_BANK or LOWER_BANK
  uint8_t rotation_1;
  uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];  // 96
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1248;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

static bool Curvesis_new = true;
/** \brief Raw Rsldar packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;


class rslidarDriver
{
public:
  /**
 * @brief rslidarDriver
 * @param node          raw packet output topic
 * @param private_nh
 */
	rslidarDriver(carmen_velodyne_variable_scan_message &variable_scan, int velodyne_num_lasers, int velodyne_max_laser_shots_per_revolution, int velodyne_udp_port, int velodyne_gps_udp_port, rslidar_param &private_nh);

  ~rslidarDriver()
  {
  }
  void unpack(int *shot_pos, const rslidarPacket& pkt, carmen_velodyne_variable_scan_message *variable_scan);
  bool poll(carmen_velodyne_variable_scan_message &variable_scan, int num_lasers, int velodyne_max_laser_shots_per_revolution, int velodyne_udp_port, int velodyne_gps_udp_port, rslidar_param &private_nh);
  void difopPoll(void);

private:
  /// Callback for dynamic reconfigure
//  void callback(rslidar_driver::rslidarNodeConfig& config, uint32_t level);
  /// Callback for skip num for time synchronization
//  void skipNumCallback(const std_msgs::Int32::ConstPtr& skip_num);

  /// Pointer to dynamic reconfigure service srv_
//  boost::shared_ptr<dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each  time stamp
    int cut_angle;
  } config_;

  InputSocket *msop_input_;
  InputSocket *difop_input_;

//  ros::Publisher msop_output_;
//  ros::Publisher difop_output_;
//  ros::Publisher output_sync_;
  // Converter convtor_;
  /** diagnostics updater */
//  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
//  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
//  boost::shared_ptr<boost::thread> difop_thread_;

  // add for time synchronization
  uint32_t skip_num_;
//  ros::Subscriber skip_num_sub_;
  int dis_resolution_mode_;
  int return_mode_;

};

}  // namespace rslidar_driver

#endif
