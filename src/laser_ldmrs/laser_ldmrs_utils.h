/*
 * laser_ldmrs_utils.h
 *
 *  Created on: Nov 10, 2016
 *      Author: vinicius
 */

#ifndef SRC_LASER_LDMRS_LASER_LDMRS_UTILS_H_
#define SRC_LASER_LDMRS_LASER_LDMRS_UTILS_H_

#include <carmen/carmen.h>
#include <sickldmrs.h>
#include <vpLaserScan.h>

void
carmen_laser_ldmrs_copy_message_to_laser_scan(vpLaserScan laserscan[4], carmen_laser_ldmrs_message *message);

void
carmen_laser_ldmrs_copy_laser_scan_to_message(carmen_laser_ldmrs_message *message, vpLaserScan laserscan[4]);

void
carmen_laser_ldmrs_new_copy_laser_scan_to_message(carmen_laser_ldmrs_new_message *message, struct sickldmrs_scan *scan);

carmen_velodyne_partial_scan_message
carmen_laser_ldmrs_convert_laser_scan_to_partial_velodyne_message(carmen_laser_ldmrs_message *msg, double laserscan_timestamp);

carmen_velodyne_partial_scan_message
carmen_laser_ldmrs_new_convert_laser_scan_to_partial_velodyne_message(carmen_laser_ldmrs_new_message *msg, double laserscan_timestamp);

#endif /* SRC_LASER_LDMRS_LASER_LDMRS_UTILS_H_ */
