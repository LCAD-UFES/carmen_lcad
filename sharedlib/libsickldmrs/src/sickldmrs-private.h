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
#ifndef _SICKLDMRS_PRIVATE_H
#define _SICKLDMRS_PRIVATE_H


/*
 * Data Header
 */
#define SLDMRS_MAGIC_WORD 0xaffec0c2

struct sickldmrs_header {
	uint32_t magic_word;
	uint32_t prev_msg_size;
	uint32_t msg_size;
	uint8_t reserved;
	uint8_t device_id;
	uint16_t data_type;
	uint64_t ntp_time;
}  __attribute__((__packed__));

/* Data types */
#define SLDMRS_COMMAND        0x2010
#define SLDMRS_REPLY	      0x2020
#define SLDMRS_ERROR_WARNING  0x2030
#define SLDMRS_SCAN_DATA      0x2202
#define SLDMRS_IBEO_SCAN      0x2204
#define SLDMRS_OBJECT_DATA    0x2221
#define SLDMRS_MOVEMENT_DATA  0x2805
#define SLDMRS_EGOMOTION_DATA 0x2850
#define SLDMRS_SENSORINFO     0x7100

struct sickldmrs_ts {
	uint32_t sec;
	uint32_t nsec;
};

/*
 * Scan data frames data type = SLDMRS_SCAN_DATA
 */
struct sickldmrs_scan_header {
	uint16_t scan_number;
	uint16_t scanner_status;
	uint16_t sync_phase_offset;
	uint64_t scan_start_time;
	uint64_t scan_end_time;
	uint16_t angle_ticks_per_rotation;
	uint16_t start_angle;
	uint16_t end_angle;
	uint16_t scan_points;
	int16_t mount_yaw;
	int16_t mount_pitch;
	int16_t mount_roll;
	int16_t mount_x;
	int16_t mount_y;
	int16_t mount_z;
	uint16_t flags;
} __attribute__((__packed__));

/*
 * Measure point - wire format
 */
struct sickldmrs_point_wire {
	uint8_t layer_echo;
	uint8_t flags;
	int16_t horizontal_angle;
	uint16_t radial_distance;
	uint16_t pulse_width;
	uint16_t reserved;
} __attribute__((__packed__));

/*
 * LD-MRS object data
 */
struct sickldmrs_point2d {
	int16_t x;
	int16_t y;
} __attribute__((__packed__));

struct sickldmrs_size2d {
	int16_t dx;
	int16_t dy;
} __attribute__((__packed__));

struct sickldmrs_object_data_header {
	uint64_t start_time;
	uint16_t num_objects;
} __attribute__((__packed__));

struct sickldmrs_object_header {
	uint16_t id;
	uint16_t age;
	uint16_t prediction_age;
	uint16_t rel_timestamp;
	struct sickldmrs_point2d ref_point;
	struct sickldmrs_size2d ref_point_sigma;
	struct sickldmrs_point2d closest_point;
	struct sickldmrs_point2d bbox_center;
	struct sickldmrs_size2d bbox_size;
	struct sickldmrs_point2d box_center;
	struct sickldmrs_size2d box_size;
	uint16_t box_orientation;
	struct sickldmrs_point2d abs_vel;
	struct sickldmrs_size2d abs_vel_sigma;
	struct sickldmrs_point2d rel_vel;
	uint16_t reserved[3];
	uint16_t num_contour_points;
} __attribute__((__packed__));

/*
 * Commands - data type = SLDMRS_COMMAND
 */
#define SLDMRS_CMD_RESET                  0x0000
#define SLDMRS_CMD_GET_STATUS             0x0001
#define SLDMRS_CMD_SAVE_CONFIG            0x0004
#define SLDMRS_CMD_SET_PARAM              0x0010
#define SLDMRS_CMD_GET_PARAM              0x0011
#define SLDMRS_CMD_RESET_FACTORY_DEFAULTS 0x001a
#define SLDMRS_CMD_START_MEASURE          0x0020
#define SLDMRS_CMD_STOP_MEASURE           0x0021
#define SLDMRS_CMD_SET_NTP_SEC            0x0030
#define SLDMRS_CMD_SET_NTP_FRAC_SEC        0x0031

/* generic command packet - without parameters */
struct sickldmrs_cmd {
	uint16_t command_id;
	uint16_t reserved;
} __attribute__((__packed__));

/* generic reply packet - without data */
struct sickldmrs_reply {
	uint16_t reply_id;
} __attribute__((__packed__));

/* GET_STATUS reply packet */
struct sickldmrs_status {
	uint16_t reply_id;
	uint16_t firmware_version;
	uint16_t fpga_version;
	uint16_t scanner_status;
	uint16_t reserved[2];
	uint16_t temperature;
	uint16_t serial_number_0;
	uint16_t serial_number_1;
	uint16_t serial_number_2;
	uint16_t fpga_time_stamp[3];
	uint16_t dsp_time_stamp[3];
} __attribute__((__packed__));

/* SET_PARAM command packet */
struct sickldmrs_set_param {
	uint16_t command_id;
	uint16_t reserved;
	uint16_t param_index;
	uint32_t value;
} __attribute__((__packed__));

/* GET_PARAM command packet */
struct sickldmrs_get_param {
	uint16_t command_id;
	uint16_t reserved;
	uint16_t param_index;
} __attribute__((__packed__));

/* GET_PARAM reply packet */
struct sickldmrs_param {
	uint16_t reply_id;
	uint16_t param_index;
	uint32_t param_value;
} __attribute__((__packed__));

/* SET_NTP_SEC and  SET_NTP_FRAC_SEC command packet */
struct sickldmrs_set_ts {
uint16_t command_id;
	uint16_t reserved;
	uint32_t timestamp;
} __attribute__((__packed__));

struct sickldmrs_private {
	int fd;			/**< socket file descriptor */
	pthread_t tid;		/**< thread id of the read task */
	pthread_mutex_t lock;	/**< lock for the dev structure  */
	pthread_cond_t cond;	/**< condition variable for waiting replies  */
	uint16_t pending;	/**< pending answer message ID */
	int done;		/**< flag to end the read task */
};

#endif
