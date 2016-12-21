/*
 * Copyright (c) 2015 CNRS/LAAS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#ifndef _SICKLDMRS_H
#define _SICKLDMRS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * scan level flags
 */
#define SLDMRS_SCAN_FLAG_GROUND_DETECTION  0x0001
#define SLDMRS_SCAN_FLAG_DIRT_DETECTION    0x0002
#define SLDMRS_SCAN_FLAG_RAIN_DETECTION    0x0004
#define SLDMRS_SCAN_FLAG_TRANS_DETECTION   0x0020
#define SLDMRS_SCAN_FLAG_HORIZONTAL_OFFSET 0x0040
#define SLDMRS_SCAN_FLAG_MIRROR_SIDE       0x0400

/**
 * point level flags
 */
#define SLDMRS_POINT_FLAG_TRANSPARENT 0x01
#define SLDMRS_POINT_FLAG_NOISE       0x02
#define SLDMRS_POINT_FLAG_GROUND      0x04
#define SLDMRS_POINT_FLAG_DIRT        0x08

/**
 * LD-MRS Parameters for GET/SET_PARAM
 * section 5.4 pp 18-24
 */
#define SLDMRS_PARAM_IP_ADDRESS              0x1000 /* u32 */
#define SLDMRS_PARAM_TCP_PORT                0x1001 /* u16 */
#define SLDMRS_PARAM_SUBNET_MASK             0x1002 /* u32 */
#define SLDMRS_PARAM_GATEWAY                 0x1003 /* u32 */
#define SLDMRS_PARAM_CAN_BASE_ID	     0x1010 /* u16 */
#define SLDMRS_PARAM_CAN_BAUD_RATE	     0x1011 /* u16 */
#define SLDMRS_PARAM_DATA_OUTPUT_FLAGS       0x1012 /* u16 */
#define SLDMRS_PARAM_MAX_OBJECTS_VIA_CAN     0x1013 /* u16 */
#define SLDMRS_PARAM_CONTOUR_POINT_DENSITY   0x1014 /* u16 */
#define SLDMRS_PARAM_CAN_OBJECT_PRIORITY     0x1015 /* u16 */
#define SLDMRS_PARAM_CAN_OBJECT_OPTION	     0x1016 /* u16 */
#define SLDMRS_PARAM_MIN_OBJECT_AGE	     0x1017 /* u16 */
#define SLDMRS_PARAM_MAX_PREDICTION_AGE	     0x1018 /* u16 */
#define SLDMRS_PARAM_START_ANGLE             0x1100 /* u16 */
#define SLDMRS_PARAM_END_ANGLE               0x1101 /* u16 */
#define SLDMRS_PARAM_SCAN_FREQUENCY          0x1102 /* u16 */
#define SLDMRS_PARAM_SYNC_ANGLE_OFFEST       0x1103 /* u16 */
#define SLDMRS_PARAM_ANGULAR_RESOLUTION_TYPE 0x1104 /* u16 */
#define SLDMRS_PARAM_ANGLE_TICK_PER_ROTATION 0x1105 /* u16 */
#define SLDMRS_PARAM_RANGE_REDUCTION	     0x1108 /* u16 */
#define SLDMRS_PARAM_UPSIDE_DOWN_MODE	     0x1109 /* u16 */
#define SLDMRS_PARAM_IGNORE_NEAR_RANGE	     0x110a /* u16 */
#define SLDMRS_PARAM_SENSITIVITY_CONTROL     0x110b /* u16 */
#define SLDMRS_PARAM_SENSOR_MOUNTING_X	     0x1200 /* i16 */
#define SLDMRS_PARAM_SENSOR_MOUNTING_Y	     0x1201 /* i16 */
#define SLDMRS_PARAM_SENSOR_MOUNTING_Z	     0x1202 /* i16 */
#define SLDMRS_PARAM_SENSOR_MOUNTING_YAW     0x1202 /* i16 */
#define SLDMRS_PARAM_SENSOR_MOUNTING_PITCH   0x1204 /* i16 */
#define SLDMRS_PARAM_SENSOR_MOUNTING_ROLL    0x1205 /* i16 */
#define SLDMRS_PARAM_VEHICLE_MOTION_DATA     0x1210 /* u16 */
#define SLDMRS_PARAM_ENABLE_SENSOR_INFO	     0x2208 /* u16 */
#define SLDMRS_PARAM_TIMEMETER		     0x3500 /* u32 */
#define SLDMRS_PARAM_NUMSECTORS		     0x4000 /* u16 */
#define SLDMRS_PARAM_START_ANGLE_SECTOR_1    0x4001 /* i16 */
#define SLDMRS_PARAM_START_ANGLE_SECTOR_2    0x4002 /* i16 */
#define SLDMRS_PARAM_START_ANGLE_SECTOR_3    0x4003 /* i16 */
#define SLDMRS_PARAM_START_ANGLE_SECTOR_4    0x4004 /* i16 */
#define SLDMRS_PARAM_START_ANGLE_SECTOR_5    0x4005 /* i16 */
#define SLDMRS_PARAM_START_ANGLE_SECTOR_6    0x4006 /* i16 */
#define SLDMRS_PARAM_START_ANGLE_SECTOR_7    0x4007 /* i16 */
#define SLDMRS_PARAM_START_ANGLE_SECTOR_8    0x4008 /* i16 */
#define SLDMRS_PARAM_ANGULAR_RES_SECTOR_1    0x4009 /* i16 */
#define SLDMRS_PARAM_ANGULAR_RES_SECTOR_2    0x400a /* i16 */
#define SLDMRS_PARAM_ANGULAR_RES_SECTOR_3    0x400b /* i16 */
#define SLDMRS_PARAM_ANGULAR_RES_SECTOR_4    0x400c /* i16 */
#define SLDMRS_PARAM_ANGULAR_RES_SECTOR_5    0x400d /* i16 */
#define SLDMRS_PARAM_ANGULAR_RES_SECTOR_6    0x400e /* i16 */
#define SLDMRS_PARAM_ANGULAR_RES_SECTOR_7    0x400f /* i16 */
#define SLDMRS_PARAM_ANGULAR_RES_SECTOR_8    0x4010 /* i16 */

/**
 * Human readable measure point
 */
struct sickldmrs_point {
	uint8_t layer;
	uint8_t echo;
	uint8_t flags;
	float horizontal_angle;
	float radial_distance;
	float pulse_width;
};

/**
 * Scan data frames in human readable form
 */
struct sickldmrs_scan {
	uint16_t scan_number;
	uint16_t scanner_status;
	uint16_t sync_phase_offset;
	struct timespec scan_start_time;
	struct timespec scan_end_time;
	uint16_t angle_ticks_per_rotation;
	float start_angle;
	float end_angle;
	uint16_t scan_points;
	float mount_yaw;
	float mount_pitch;
	float mount_roll;
	float mount_x;
	float mount_y;
	float mount_z;
	uint16_t flags;
	struct sickldmrs_point points[];
};


/**
 * Errors and warnings - data type = SLDMRS_ERROR_WARNING
 */
struct sickldmrs_error {
	uint16_t error_register_1;
	uint16_t error_register_2;
	uint16_t warning_register_1;
	uint16_t warning_register_2;
	uint16_t reserved[4];
} __attribute__((__packed__));

#define SLDMRS_ERR_REG_1_INCOMPLETE_SCAN 0x0004
#define SLDMRS_ERR_REG_1_SCAN_OVERFLOW 0x0008
#define SLDMRS_ERR_REG_1_OVER_TEMP 0x0010
#define SLDMRS_ERR_REG_1_UNDER_TEMP 0x0020
#define SLDMRS_ERR_REG_1_CONTACT_SUPPORT 0x2013

#define SLDMRS_ERR_REG_2_BAD_CONFIG 0x0010
#define SLDMRS_ERR_REG_2_BAD_PARAMS 0x0020
#define SLDMRS_ERR_REG_2_TIMEOUT 0x0040
#define SLDMRS_ERR_REG_2_CONTACT_SUPPORT 0x008f

#define SLDMRS_WARN_REG_1_INTERNAL_COMM_ERR 0x0001
#define SLDMRS_WARN_REG_1_INTERNAL_WARN 0x0066
#define SLDMRS_WARN_REG_1_LOW_TEMP 0x0008
#define SLDMRS_WARN_REG_1_HIGH_TEMP 0x0010
#define SLDMRS_WARN_REG_1_SYNC_ERR 0x0080

#define SLDMRS_WARN_REG_2_ETH_BLOCKED 0x0002
#define SLDMRS_WARN_REG_2_CONTACT_SUPORT 0x0008
#define SLDMRS_WARN_REG_2_ETH_RECV_ERR 0x0010
#define SLDMRS_WARN_REG_2_BAD_CLD 0x0020
#define SLDMRS_WARN_REG_2_MEM_FAIL 0x0040

/**
 * main device structure
 */
struct sickldmrs_device {
	struct sickldmrs_private *priv;
	int debug;
	char firmware_version[8];
	char fpga_version[8];
	char serial_number[16];
	char fpga_date[16];
	char dsp_date[16];
	uint16_t status;
	double temperature;
	double start_angle;
	double end_angle;
	double scan_frequency;
	struct sickldmrs_error errors;
	struct sickldmrs_scan *scan;
};

/**
 * Prototypes
 */
extern struct sickldmrs_device *sickldmrs_init(const char *,
    const char *, bool);
extern void sickldmrs_end(struct sickldmrs_device *);
extern int sickldmrs_lock(struct sickldmrs_device *);
extern int sickldmrs_unlock(struct sickldmrs_device *);
extern int sickldmrs_wait_reply(struct sickldmrs_device *, int16_t, int);

extern int sickldmrs_receive_frame(struct sickldmrs_device *, int);

extern int sickldmrs_reset(struct sickldmrs_device *, int);
extern int sickldmrs_get_status(struct sickldmrs_device *, int);
extern int sickldmrs_save_config(struct sickldmrs_device *, int);

extern int sickldmrs_config_network(struct sickldmrs_device *,
    const char *, const char *, const char *, uint16_t, int);
extern int sickldmrs_config_scan(struct sickldmrs_device *,
    double, double, int);
extern int sickldmrs_set_parameter(struct sickldmrs_device *,
    uint16_t, long, int);
extern int sickldmrs_get_parameter(struct sickldmrs_device *, uint16_t, int);
extern int sickldmrs_start_measure(struct sickldmrs_device *, bool, int);
extern int sickldmrs_settime(struct sickldmrs_device *,
    const struct timespec *, int);
extern int sickldmrs_config_output(struct sickldmrs_device *, uint16_t, int);

extern void sickldmrs_print_scan(struct sickldmrs_scan *);
extern void sickldmrs_print_scan2(struct sickldmrs_scan *);

#ifdef __cplusplus
}
#endif

#endif
