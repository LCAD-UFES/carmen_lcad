/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include <carmen/carmen_stdio.h>
#include <carmen/readlog.h>
#include <ctype.h>
#include <opencv2/highgui/highgui.hpp>

#define HEX_TO_BYTE(hi, lo) (hi << 4 | lo)
#define HEX_TO_SHORT(fourth, third, second, first) ( fourth << 12 | (third << 8 | (second << 4 | first)))

#define HEX_TO_RGB_BYTE(hi, lo) (hi << 4 | lo)
#define GETINDEX(a) isalpha(a) ? a - 'a' + 10 : a - '0'

void CLF_READ_STRING(char *dst, char **string)
{
	int l;

	/* advance past spaces */
	while(*string[0] == ' ')
		*string += 1;

	l = first_wordlength(*string);
	strncpy(dst, *string, l);
	dst[l] = '\0';
	*string += l;
}

off_t carmen_logfile_uncompressed_length(carmen_FILE *infile)
{
	//  unsigned char buffer[10000];
	//  long int log_bytes = 0;
	//  int nread;
	struct stat stat_buf;

	//  if(!infile->compressed) {
	//    /* compute total length of logfile */
	//   carmen_fseek(infile, 0L, SEEK_SET);
	//    log_bytes = 0;
	//    do {
	//      nread = carmen_fread(buffer, 1, 10000, infile);
	//      log_bytes += nread;
	//    } while(nread > 0);
	//    carmen_fseek(infile, 0L, SEEK_SET);
	//    return log_bytes;
	//  }
	//  else {
	//    /* report compressed size for compressed files */
	fstat(fileno(infile->fp), &stat_buf);
	return stat_buf.st_size;
	//  }
}

/**
 * Builds the index structure used for parsing a carmen log file.
 **/
carmen_logfile_index_p carmen_logfile_index_messages(carmen_FILE *infile)
{
#define READ_LOG_BUUFER_SIZE	100000
	carmen_logfile_index_p index;
	int i, found_linebreak = 1, nread, max_messages;
	off_t file_length = 0, file_position = 0, total_bytes, read_count = 0;

	unsigned char buffer[READ_LOG_BUUFER_SIZE];

	/* allocate and initialize an index */
	index = (carmen_logfile_index_p)calloc(1, sizeof(carmen_logfile_index_t));
	carmen_test_alloc(index);

	/* compute the total length of the uncompressed logfile. */
	fprintf(stderr, "\n\rIndexing messages (0%%)    ");
	file_length = carmen_logfile_uncompressed_length(infile);

	/* mark the start of all messages */
	index->num_messages = 0;
	max_messages = READ_LOG_BUUFER_SIZE;
	index->offset = (off_t*)calloc(max_messages, sizeof(off_t));
	carmen_test_alloc(index->offset);

	carmen_fseek(infile, 0L, SEEK_SET);

	total_bytes = 0;
	do {
		nread = carmen_fread(buffer, 1, READ_LOG_BUUFER_SIZE, infile);
		read_count++;
		if(read_count % 1000 == 0) {
			if(!infile->compressed)
				file_position = total_bytes + nread;
			else
				file_position = lseek(fileno(infile->fp), 0, SEEK_CUR);
			fprintf(stderr, "\rIndexing messages (%.0f%%)      ",
					((float)file_position) / file_length * 100.0);
		}

		if(nread > 0) {
			for(i = 0; i < nread; i++) {
				if(found_linebreak && buffer[i] != '\r') {
					found_linebreak = 0;
					if(index->num_messages == max_messages) {
						max_messages += READ_LOG_BUUFER_SIZE;
						index->offset = (off_t*)realloc(index->offset, max_messages *
								sizeof(off_t));
						carmen_test_alloc(index->offset);
					}
					index->offset[index->num_messages] = total_bytes + i;
					index->num_messages++;
				}
				if(buffer[i] == '\n')
					found_linebreak = 1;
			}
			total_bytes += nread;
		}
	} while(nread > 0);

	// set file size as last offset
	// offset array now contains one element more than messages
	// required by carmen_logfile_read_line to read the last line
	if(index->num_messages == max_messages) {
		max_messages += 1;
		index->offset = (off_t*)realloc(index->offset, max_messages * sizeof(off_t));
		carmen_test_alloc(index->offset);
	}
	index->offset[index->num_messages] = total_bytes;

	fprintf(stderr, "\rIndexing messages (100%%) - %d messages found.      \n",
			index->num_messages);
	carmen_fseek(infile, 0L, SEEK_SET);
	index->current_position = 0;
	return index;
}

void carmen_logfile_free_index(carmen_logfile_index_p* pindex) {
	if (pindex == NULL)
		return;

	if ( (*pindex) == NULL)
		return;

	if ( (*pindex)->offset != NULL) {
		free( (*pindex)->offset);
	}
	free(*pindex);
	(*pindex) = NULL;
}

int carmen_logfile_eof(carmen_logfile_index_p index)
{
	if(index->current_position > index->num_messages - 1)
		return 1;
	else
		return 0;
}

float carmen_logfile_percent_read(carmen_logfile_index_p index)
{
	return index->current_position / (float)index->num_messages;
}

int carmen_logfile_read_line(carmen_logfile_index_p index, carmen_FILE *infile,
		int message_num, int max_line_length, char *line)
{
	size_t nread;

	/* are we moving sequentially through the logfile?  If not, fseek */
	if(message_num != index->current_position) {
		index->current_position = message_num;
		carmen_fseek(infile, index->offset[index->current_position], SEEK_SET);
	}

	/* check maximum line length */
	if(index->offset[index->current_position + 1] -
			index->offset[index->current_position] >= max_line_length)
		carmen_die("Error: exceed maximum line length.\n");

	/* read the line of the logfile */
	nread = carmen_fread(line, 1, index->offset[index->current_position + 1] -
			index->offset[index->current_position], infile);
	line[nread] = '\0';
	index->current_position++;

	return nread;
}

int carmen_logfile_read_next_line(carmen_logfile_index_p index, carmen_FILE *infile,
		int max_line_length, char *line)
{
	return carmen_logfile_read_line(index, infile,
			index->current_position,
			max_line_length, line);
}


int first_wordlength(char *str)
{
	char* c_enter = strchr(str, '\n'); // check also for newline
	char* c = strchr(str, ' ');

	if (c_enter == NULL && c == NULL) // it is the last word in the string
		return strlen(str);

	if (c_enter != NULL && c == NULL) // there is no space but a newline
		return c_enter - str;

	if (c_enter == NULL && c != NULL) // there is a space but no newline
		return c - str;

	if (c_enter < c )    // use whatever comes first
		return c_enter - str;
	else
		return c - str;
}

void copy_host_string(char **host, char **string)
{
	int l;
	while(*string[0] == ' ')
		*string += 1;                           /* advance past spaces */
	l = first_wordlength(*string);
	if(*host != NULL)
		free(*host);
	*host = (char *)calloc(1, l+1);     /* allocate one extra char for the \0 */
	carmen_test_alloc(*host);
	strncpy(*host, *string, l);
	(*host)[l] = '\0';
	*string += l;
}

char *carmen_string_to_base_ackerman_odometry_message(char *string,
		carmen_base_ackerman_odometry_message
		*odometry)
{
	char *current_pos = string;

	if (strncmp(current_pos, "ODOM_ACK ", 9) == 0)
		current_pos = carmen_next_word(current_pos);

	odometry->x = CLF_READ_DOUBLE(&current_pos);
	odometry->y = CLF_READ_DOUBLE(&current_pos);
	odometry->theta = CLF_READ_DOUBLE(&current_pos);
	odometry->v = CLF_READ_DOUBLE(&current_pos);
	odometry->phi = CLF_READ_DOUBLE(&current_pos);
	odometry->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&odometry->host, &current_pos);
	return current_pos;
}

char *carmen_string_to_visual_odometry_message(char *string,
		carmen_visual_odometry_pose6d_message *odometry)
{
	char *current_pos = string;

	if (strncmp(current_pos, "VISUAL_ODOMETRY ", 16) == 0)
		current_pos = carmen_next_word(current_pos);

	odometry->pose_6d.x = CLF_READ_DOUBLE(&current_pos);
	odometry->pose_6d.y = CLF_READ_DOUBLE(&current_pos);
	odometry->pose_6d.z = CLF_READ_DOUBLE(&current_pos);
	odometry->pose_6d.roll = CLF_READ_DOUBLE(&current_pos);
	odometry->pose_6d.pitch = CLF_READ_DOUBLE(&current_pos);
	odometry->pose_6d.yaw = CLF_READ_DOUBLE(&current_pos);
	odometry->v = CLF_READ_DOUBLE(&current_pos);
	odometry->phi = CLF_READ_DOUBLE(&current_pos);
	odometry->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&odometry->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_simulator_ackerman_truepos_message(char *string,
		carmen_simulator_ackerman_truepos_message *truepos)
{
	char *current_pos = string;

	if (strncmp(current_pos, "TRUEPOS_ACK ", 12) == 0)
		current_pos = carmen_next_word(current_pos);

	truepos->truepose.x = CLF_READ_DOUBLE(&current_pos);
	truepos->truepose.y = CLF_READ_DOUBLE(&current_pos);
	truepos->truepose.theta = CLF_READ_DOUBLE(&current_pos);
	truepos->odometrypose.x = CLF_READ_DOUBLE(&current_pos);
	truepos->odometrypose.y = CLF_READ_DOUBLE(&current_pos);
	truepos->odometrypose.theta = CLF_READ_DOUBLE(&current_pos);
	truepos->v = CLF_READ_DOUBLE(&current_pos);
	truepos->phi = CLF_READ_DOUBLE(&current_pos);
	truepos->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&truepos->host, &current_pos);
	return current_pos;
}

double carmen_laser_guess_fov(int num_beams)
{
	if (num_beams == 181)
		return M_PI;                  /* 180 degrees */
	else if (num_beams == 180)
		return M_PI / 180.0 * 179.0;  /* 179 degrees (last beam ignored)*/
	else if (num_beams == 361)
		return M_PI;                  /* 180 degrees */
	else if (num_beams == 360)
		return M_PI / 180.0 * 179.5 ; /* 179.5 degrees (last beam ignored)*/
	else if (num_beams == 401)
		return M_PI / 180.0 * 100.0 ; /* 100.0 degrees */
	else if (num_beams == 400)
		return M_PI / 100.0 * 99.75 ; /* 99.75 degrees (last beam ignored)*/
	else
		return M_PI;                  /* assume 180 degrees */
}

double carmen_laser_guess_angle_increment(int num_beams)
{
	if (num_beams == 181 || num_beams == 180)
		return M_PI / 180.0; /* 1 degree = M_PI/180 */
	else if (num_beams == 361 || num_beams == 360)
		return M_PI / 360.0;  /* 0.5 degrees = M_PI/360 */
	else if (num_beams == 401 || num_beams == 400)
		return M_PI / 720.0;  /* 0.25 degrees = M_PI/720 */
	else
		return carmen_laser_guess_fov(num_beams) /
		((double) (num_beams-1));
}

char *carmen_string_to_laser_laser_message_orig(char *string,
		carmen_laser_laser_message
		*laser)
{
	char *current_pos = string;
	int i, num_readings;

	if (strncmp(current_pos, "LASER", 5) == 0)
		current_pos = carmen_next_word(current_pos);


	num_readings = CLF_READ_INT(&current_pos);
	if(laser->num_readings != num_readings) {
		laser->num_readings = num_readings;
		laser->range = (double *)realloc(laser->range, laser->num_readings *
				sizeof(double));
		carmen_test_alloc(laser->range);
	}
	for(i = 0; i < laser->num_readings; i++)
		laser->range[i] = CLF_READ_DOUBLE(&current_pos);
	laser->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&laser->host, &current_pos);

	/* fill in remission with nothing */
	laser->num_remissions = 0;
	laser->remission = NULL;

	/* guess at fields */
	laser->config.laser_type = SICK_LMS;
	laser->config.fov = carmen_laser_guess_fov(laser->num_readings);
	laser->config.start_angle = -M_PI / 2.0;
	laser->config.angular_resolution =
		carmen_laser_guess_angle_increment(laser->num_readings);
	laser->config.maximum_range = 80.0;
	laser->config.accuracy = 0.01;
	laser->config.remission_mode = (carmen_laser_remission_type_t) 0;

	return current_pos;
}

char *carmen_string_to_laser_laser_message(char *string,
		carmen_laser_laser_message *laser)
{
	char *current_pos = string - 10;
	int i, num_readings, num_remissions;
	if (strncmp(current_pos, "RAWLASER", 8) == 0) {
		current_pos += 8;
		laser->id = CLF_READ_INT(&current_pos);
	} else {
		laser->id = -1;
	}

	laser->config.laser_type = (carmen_laser_laser_type_t) CLF_READ_INT(&current_pos);
	laser->config.start_angle = CLF_READ_DOUBLE(&current_pos);
	laser->config.fov = CLF_READ_DOUBLE(&current_pos);
	laser->config.angular_resolution = CLF_READ_DOUBLE(&current_pos);
	laser->config.maximum_range = CLF_READ_DOUBLE(&current_pos);
	laser->config.accuracy = CLF_READ_DOUBLE(&current_pos);
	laser->config.remission_mode = (carmen_laser_remission_type_t) CLF_READ_INT(&current_pos);

	num_readings = CLF_READ_INT(&current_pos);
	if(laser->num_readings != num_readings) {
		laser->num_readings = num_readings;
		laser->range = (double *)realloc(laser->range, laser->num_readings *
				sizeof(double));
		carmen_test_alloc(laser->range);
	}
	for(i = 0; i < laser->num_readings; i++)
		laser->range[i] = CLF_READ_DOUBLE(&current_pos);

	num_remissions = CLF_READ_INT(&current_pos);
	if(laser->num_remissions != num_remissions) {
		laser->num_remissions = num_remissions;
		laser->remission = (double *)realloc(laser->remission,
				laser->num_remissions * sizeof(double));
		carmen_test_alloc(laser->remission);
	}
	for(i = 0; i < laser->num_remissions; i++)
		laser->remission[i] = CLF_READ_DOUBLE(&current_pos);

	laser->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&laser->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_laser_ldmrs_message(char *string,
		carmen_laser_ldmrs_message *laser)
{
	char *current_pos = string;
	int i, num_readings;

	if (strncmp(current_pos, "LASER_LDMRS", 11) == 0)
		current_pos += 11;

	laser->scan_number = CLF_READ_INT(&current_pos);
	laser->scan_start_time = CLF_READ_DOUBLE(&current_pos);
	laser->scan_end_time = CLF_READ_DOUBLE(&current_pos);
	laser->angle_ticks_per_rotation = CLF_READ_INT(&current_pos);
	laser->start_angle = CLF_READ_DOUBLE(&current_pos);
	laser->end_angle = CLF_READ_DOUBLE(&current_pos);
	num_readings = CLF_READ_INT(&current_pos);

	if(laser->scan_points != num_readings)
	{
		laser->scan_points = num_readings;
		laser->arraypoints = (carmen_laser_ldmrs_point *)realloc(laser->arraypoints, laser->scan_points * sizeof(carmen_laser_ldmrs_point));
		carmen_test_alloc(laser->arraypoints);
	}

	for(i = 0; i < laser->scan_points; i++)
	{
	    laser->arraypoints[i].horizontal_angle = CLF_READ_DOUBLE(&current_pos);
	    laser->arraypoints[i].vertical_angle = CLF_READ_DOUBLE(&current_pos);
	    laser->arraypoints[i].radial_distance = CLF_READ_DOUBLE(&current_pos);
	    laser->arraypoints[i].flags = CLF_READ_INT(&current_pos);
	}

	laser->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&laser->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_laser_ldmrs_new_message(char *string,
		carmen_laser_ldmrs_new_message *laser)
{
	char *current_pos = string;
	int i, num_readings;

	if (strncmp(current_pos, "LASER_LDMRS_NEW", 15) == 0)
		current_pos += 15;

	laser->scan_number = CLF_READ_INT(&current_pos);
	laser->scanner_status = CLF_READ_INT(&current_pos);
	laser->sync_phase_offset = CLF_READ_INT(&current_pos);
	laser->scan_start_time = CLF_READ_DOUBLE(&current_pos);
	laser->scan_end_time = CLF_READ_DOUBLE(&current_pos);
	laser->angle_ticks_per_rotation = CLF_READ_INT(&current_pos);
	laser->start_angle = CLF_READ_DOUBLE(&current_pos);
	laser->end_angle = CLF_READ_DOUBLE(&current_pos);
	num_readings = CLF_READ_INT(&current_pos);
	laser->flags = CLF_READ_INT(&current_pos);

	if (laser->scan_points != num_readings)
	{
		laser->scan_points = num_readings;
		laser->arraypoints = (carmen_laser_ldmrs_new_point *)realloc(laser->arraypoints, laser->scan_points * sizeof(carmen_laser_ldmrs_new_point));
		carmen_test_alloc(laser->arraypoints);
	}

	for (i = 0; i < laser->scan_points; i++)
	{
	    laser->arraypoints[i].horizontal_angle = CLF_READ_DOUBLE(&current_pos);
	    laser->arraypoints[i].vertical_angle = CLF_READ_DOUBLE(&current_pos);
	    laser->arraypoints[i].radial_distance = CLF_READ_DOUBLE(&current_pos);
	    laser->arraypoints[i].layer = CLF_READ_INT(&current_pos);
	    laser->arraypoints[i].echo = CLF_READ_INT(&current_pos);
	    laser->arraypoints[i].flags = CLF_READ_INT(&current_pos);
	}

	laser->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&laser->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_laser_ldmrs_objects_message(char *string,
		carmen_laser_ldmrs_objects_message *laserObjects)
{
	char *current_pos = string;
	int i, num_readings;

	if (strncmp(current_pos, "LASER_LDMRS_OBJECTS", 19) == 0)
			current_pos += 19;

	num_readings = CLF_READ_INT(&current_pos);

	if(laserObjects->num_objects != num_readings)
	{
		laserObjects->num_objects = num_readings;
		laserObjects->objects_list = (carmen_laser_ldmrs_object *)realloc(laserObjects->objects_list, laserObjects->num_objects * sizeof(carmen_laser_ldmrs_object));
		carmen_test_alloc(laserObjects->objects_list);
	}

	for(i = 0; i < laserObjects->num_objects; i++)
	{
		laserObjects->objects_list[i].id = CLF_READ_INT(&current_pos);
		laserObjects->objects_list[i].x = CLF_READ_DOUBLE(&current_pos);
		laserObjects->objects_list[i].y = CLF_READ_DOUBLE(&current_pos);
		laserObjects->objects_list[i].lenght = CLF_READ_DOUBLE(&current_pos);
		laserObjects->objects_list[i].width = CLF_READ_DOUBLE(&current_pos);
		laserObjects->objects_list[i].velocity = CLF_READ_DOUBLE(&current_pos);
		laserObjects->objects_list[i].orientation = CLF_READ_DOUBLE(&current_pos);
		laserObjects->objects_list[i].classId = CLF_READ_INT(&current_pos);
	}

	laserObjects->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&laserObjects->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_laser_ldmrs_objects_data_message(char *string,
		carmen_laser_ldmrs_objects_data_message *laserObjectsData)
{
	char *current_pos = string;
	int i, num_readings;

	if (strncmp(current_pos, "LASER_LDMRS_OBJECTS_DATA", 24) == 0)
			current_pos += 24;

	num_readings = CLF_READ_INT(&current_pos);

	if(laserObjectsData->num_objects != num_readings)
	{
		laserObjectsData->num_objects = num_readings;
		laserObjectsData->objects_data_list = (carmen_laser_ldmrs_object_data *)realloc(laserObjectsData->objects_data_list, laserObjectsData->num_objects * sizeof(carmen_laser_ldmrs_object_data));
		carmen_test_alloc(laserObjectsData->objects_data_list);
	}

	for(i = 0; i < laserObjectsData->num_objects; i++)
	{
		laserObjectsData->objects_data_list[i].object_id = CLF_READ_INT(&current_pos);
		laserObjectsData->objects_data_list[i].object_age = CLF_READ_INT(&current_pos);
		laserObjectsData->objects_data_list[i].object_prediction_age = CLF_READ_INT(&current_pos);
		laserObjectsData->objects_data_list[i].reference_point_x = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].reference_point_y = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].reference_point_sigma_x = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].reference_point_sigma_y = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].closest_point_x = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].closest_point_y = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].bounding_box_center_x = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].bounding_box_center_y = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].bounding_box_length = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].bounding_box_width = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].object_box_center_x = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].object_box_center_y = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].object_box_lenght = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].object_box_width = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].object_box_orientation = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].abs_velocity_x = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].abs_velocity_y = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].abs_velocity_sigma_x = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].abs_velocity_sigma_y = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].relative_velocity_x = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].relative_velocity_y = CLF_READ_DOUBLE(&current_pos);
		laserObjectsData->objects_data_list[i].class_id = CLF_READ_INT(&current_pos);
	}

	laserObjectsData->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&laserObjectsData->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_robot_ackerman_laser_message(char *string,
		carmen_robot_ackerman_laser_message *laser)
{
	char *current_pos = string;
	int i, num_readings, num_remissions;

	if (strncmp(current_pos, "ROBOTLASER_ACK", 14) == 0) {
		current_pos += 14;
		laser->id = CLF_READ_INT(&current_pos);
	} else {
		laser->id = -1;
	}

	laser->config.laser_type = (carmen_laser_laser_type_t) CLF_READ_INT(&current_pos);
	laser->config.start_angle = CLF_READ_DOUBLE(&current_pos);
	laser->config.fov = CLF_READ_DOUBLE(&current_pos);
	laser->config.angular_resolution = CLF_READ_DOUBLE(&current_pos);
	laser->config.maximum_range = CLF_READ_DOUBLE(&current_pos);
	laser->config.accuracy = CLF_READ_DOUBLE(&current_pos);
	laser->config.remission_mode = (carmen_laser_remission_type_t) CLF_READ_INT(&current_pos);

	num_readings = CLF_READ_INT(&current_pos);
	if(laser->num_readings != num_readings) {
		laser->num_readings = num_readings;
		laser->range = (double *)realloc(laser->range, laser->num_readings *
				sizeof(double));
		carmen_test_alloc(laser->range);


		laser->tooclose = (char *)realloc(laser->tooclose, laser->num_readings *
				sizeof(char));
		carmen_test_alloc(laser->tooclose);


	}
	for(i = 0; i < laser->num_readings; i++) {
		laser->range[i] = CLF_READ_DOUBLE(&current_pos);
		laser->tooclose[i] = 0;
	}

	num_remissions = CLF_READ_INT(&current_pos);
	if(laser->num_remissions != num_remissions) {
		laser->num_remissions = num_remissions;
		laser->remission = (double *)realloc(laser->remission,
				laser->num_remissions * sizeof(double));
		carmen_test_alloc(laser->remission);
	}
	for(i = 0; i < laser->num_remissions; i++)
		laser->remission[i] = CLF_READ_DOUBLE(&current_pos);

	laser->laser_pose.x = CLF_READ_DOUBLE(&current_pos);
	laser->laser_pose.y = CLF_READ_DOUBLE(&current_pos);
	laser->laser_pose.theta = CLF_READ_DOUBLE(&current_pos);
	laser->robot_pose.x = CLF_READ_DOUBLE(&current_pos);
	laser->robot_pose.y = CLF_READ_DOUBLE(&current_pos);
	laser->robot_pose.theta = CLF_READ_DOUBLE(&current_pos);

	laser->v = CLF_READ_DOUBLE(&current_pos);
	laser->phi = CLF_READ_DOUBLE(&current_pos);
	laser->forward_safety_dist = CLF_READ_DOUBLE(&current_pos);
	laser->side_safety_dist = CLF_READ_DOUBLE(&current_pos);
	laser->turn_axis = CLF_READ_DOUBLE(&current_pos);

	laser->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&laser->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_gps_gpgga_message(char *string,
		carmen_gps_gpgga_message *gps_msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "NMEAGGA ", 8) == 0)
		current_pos = carmen_next_word(current_pos);

	gps_msg->nr               = CLF_READ_INT(&current_pos);
	gps_msg->utc              = CLF_READ_DOUBLE(&current_pos);
	gps_msg->latitude_dm      = CLF_READ_DOUBLE(&current_pos);
	gps_msg->latitude         = carmen_global_convert_degmin_to_double(gps_msg->latitude_dm);
	current_pos = carmen_next_word(current_pos);
	gps_msg->lat_orient       = CLF_READ_CHAR(&current_pos);
	gps_msg->longitude_dm     = CLF_READ_DOUBLE(&current_pos);
	gps_msg->longitude        = carmen_global_convert_degmin_to_double(gps_msg->longitude_dm);
	current_pos = carmen_next_word(current_pos);
	gps_msg->long_orient      = CLF_READ_CHAR(&current_pos);
	gps_msg->gps_quality      = CLF_READ_INT(&current_pos);
	gps_msg->num_satellites   = CLF_READ_INT(&current_pos);
	gps_msg->hdop             = CLF_READ_DOUBLE(&current_pos);
	gps_msg->sea_level        = CLF_READ_DOUBLE(&current_pos);
	gps_msg->altitude         = CLF_READ_DOUBLE(&current_pos);
	gps_msg->geo_sea_level    = CLF_READ_DOUBLE(&current_pos);
	gps_msg->geo_sep          = CLF_READ_DOUBLE(&current_pos);
	gps_msg->data_age         = CLF_READ_INT(&current_pos);
	gps_msg->timestamp        = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&gps_msg->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_gps_gphdt_message(char *string,
		carmen_gps_gphdt_message *gps_msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "NMEAHDT ", 8) == 0)
		current_pos = carmen_next_word(current_pos);

	gps_msg->nr               = CLF_READ_INT(&current_pos);
	gps_msg->heading          = CLF_READ_DOUBLE(&current_pos);
	gps_msg->valid            = CLF_READ_INT(&current_pos);
	gps_msg->timestamp        = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&gps_msg->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_gps_gprmc_message(char *string,
		carmen_gps_gprmc_message *gps_msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "NMEARMC ", 8) == 0)
		current_pos = carmen_next_word(current_pos);

	gps_msg->nr               = CLF_READ_INT(&current_pos);
	gps_msg->validity         = CLF_READ_INT(&current_pos);
	gps_msg->utc              = CLF_READ_DOUBLE(&current_pos);
	gps_msg->latitude_dm      = CLF_READ_DOUBLE(&current_pos);
	gps_msg->latitude         = carmen_global_convert_degmin_to_double(gps_msg->latitude_dm);
	current_pos = carmen_next_word(current_pos);
	gps_msg->lat_orient       = CLF_READ_CHAR(&current_pos);
	gps_msg->longitude_dm     = CLF_READ_DOUBLE(&current_pos);
	gps_msg->longitude        = carmen_global_convert_degmin_to_double(gps_msg->longitude_dm);
	current_pos = carmen_next_word(current_pos);
	gps_msg->long_orient      = CLF_READ_CHAR(&current_pos);
	gps_msg->speed            = CLF_READ_DOUBLE(&current_pos);
	gps_msg->true_course      = CLF_READ_DOUBLE(&current_pos);
	gps_msg->variation        = CLF_READ_DOUBLE(&current_pos);
	current_pos = carmen_next_word(current_pos);
	gps_msg->var_dir          = CLF_READ_CHAR(&current_pos);
	gps_msg->date             = CLF_READ_INT(&current_pos);
	gps_msg->timestamp        = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&gps_msg->host, &current_pos);

	return current_pos;
}

char* carmen_string_to_ultrasonic_message(char* string, carmen_ultrasonic_sonar_sensor_message* ultrasonic_msg)
{
	int i;
	char* current_pos = string;

	if (strncmp(current_pos, "ULTRASONIC_SONAR_SENSOR ", 24) == 0)
		current_pos = carmen_next_word(current_pos);

	ultrasonic_msg->number_of_sonars = CLF_READ_INT(&current_pos);
	ultrasonic_msg->sonar_beans = CLF_READ_INT(&current_pos);
	ultrasonic_msg->fov = CLF_READ_DOUBLE(&current_pos);
	ultrasonic_msg->angle_step = CLF_READ_DOUBLE(&current_pos);
	ultrasonic_msg->start_angle = CLF_READ_DOUBLE(&current_pos);
	ultrasonic_msg->max_range = CLF_READ_DOUBLE(&current_pos);

	for (i = 0; i < 4; i++)
		ultrasonic_msg->sensor[i] = CLF_READ_DOUBLE(&current_pos);

	ultrasonic_msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&ultrasonic_msg->host, &current_pos);

	return current_pos;
}

char* carmen_string_to_pantilt_scanmark_message(char* string, carmen_pantilt_scanmark_message* scanmark) {

	char *current_pos = string;

	if (strncmp(current_pos, "SCANMARK ", 9) == 0) {
		current_pos = carmen_next_word(current_pos);
	}

	scanmark->type = CLF_READ_INT(&current_pos);
	scanmark->laserid = CLF_READ_INT(&current_pos);

	scanmark->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&scanmark->host, &current_pos);

	return current_pos;
}

char* carmen_string_to_pantilt_laserpos_message(char* string, carmen_pantilt_laserpos_message* laserpos) {

	char *current_pos = string;

	if (strncmp(current_pos, "POSITIONLASER ", 14) == 0) {
		current_pos = carmen_next_word(current_pos);
	}

	laserpos->id = CLF_READ_INT(&current_pos);

	laserpos->x = CLF_READ_DOUBLE(&current_pos);
	laserpos->y = CLF_READ_DOUBLE(&current_pos);
	laserpos->z = CLF_READ_DOUBLE(&current_pos);

	laserpos->phi = CLF_READ_DOUBLE(&current_pos);
	laserpos->theta = CLF_READ_DOUBLE(&current_pos);
	laserpos->psi = CLF_READ_DOUBLE(&current_pos);

	laserpos->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&laserpos->host, &current_pos);

	return current_pos;
}


char* carmen_string_to_imu_message(char* string, carmen_imu_message* msg)
{
	char* current_pos = string;
	if (strncmp(current_pos, "IMU ", 4) == 0)
		current_pos = carmen_next_word(current_pos);

	msg->accX = CLF_READ_DOUBLE(&current_pos);
	msg->accY = CLF_READ_DOUBLE(&current_pos);
	msg->accZ = CLF_READ_DOUBLE(&current_pos);

	msg->q0 = CLF_READ_DOUBLE(&current_pos);
	msg->q1 = CLF_READ_DOUBLE(&current_pos);
	msg->q2 = CLF_READ_DOUBLE(&current_pos);
	msg->q3 = CLF_READ_DOUBLE(&current_pos);

	msg->magX = CLF_READ_DOUBLE(&current_pos);
	msg->magY = CLF_READ_DOUBLE(&current_pos);
	msg->magZ = CLF_READ_DOUBLE(&current_pos);

	msg->gyroX = CLF_READ_DOUBLE(&current_pos);
	msg->gyroY = CLF_READ_DOUBLE(&current_pos);
	msg->gyroZ = CLF_READ_DOUBLE(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char *carmen_string_to_robot_ackerman_velocity_message(char *string, carmen_robot_ackerman_velocity_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "ROBOTVELOCITY_ACK ", 18) == 0)
		current_pos = carmen_next_word(current_pos);

	msg->v = CLF_READ_DOUBLE(&current_pos);
	msg->phi = CLF_READ_DOUBLE(&current_pos);
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char *carmen_string_to_robot_ackerman_vector_move_message(char *string, carmen_robot_ackerman_vector_move_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "VECTORMOVE_ACK", 14) == 0)
		current_pos = carmen_next_word(current_pos);

	msg->distance = CLF_READ_DOUBLE(&current_pos);
	msg->theta = CLF_READ_DOUBLE(&current_pos);
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}


char *carmen_string_to_robot_ackerman_follow_trajectory_message(char *string, carmen_robot_ackerman_follow_trajectory_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "FOLLOWTRAJECTORY_ACK", 20) == 0)
		current_pos = carmen_next_word(current_pos);

	msg->robot_position.x = CLF_READ_DOUBLE(&current_pos);
	msg->robot_position.y = CLF_READ_DOUBLE(&current_pos);
	msg->robot_position.theta = CLF_READ_DOUBLE(&current_pos);
	msg->robot_position.v = CLF_READ_DOUBLE(&current_pos);
	msg->robot_position.phi = CLF_READ_DOUBLE(&current_pos);

	int length = CLF_READ_INT(&current_pos);

	if(msg->trajectory_length != length) {
		msg->trajectory_length = length;
		msg->trajectory = (carmen_ackerman_traj_point_t *)realloc(msg->trajectory, length *
				sizeof(carmen_ackerman_traj_point_t));
		carmen_test_alloc(msg->trajectory);
	}



	int i;
	for (i=0; i<msg->trajectory_length; i++)
	{
		msg->trajectory[i].x = CLF_READ_DOUBLE(&current_pos);
		msg->trajectory[i].y = CLF_READ_DOUBLE(&current_pos);
		msg->trajectory[i].theta = CLF_READ_DOUBLE(&current_pos);
		msg->trajectory[i].v = CLF_READ_DOUBLE(&current_pos);
		msg->trajectory[i].phi = CLF_READ_DOUBLE(&current_pos);
	}

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}

char *carmen_string_to_base_ackerman_velocity_message(char *string, carmen_base_ackerman_velocity_message *msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "BASEVELOCITY_ACK", 16) == 0)
		current_pos = carmen_next_word(current_pos);

	msg->v = CLF_READ_DOUBLE(&current_pos);
	msg->phi = CLF_READ_DOUBLE(&current_pos);
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);
	return current_pos;
}

char *carmen_string_to_kinect_depth_message(char *string,
		carmen_kinect_depth_message *kinect)
{
	char *current_pos = string;
	int i, size, r;
	unsigned char first, second, third, fourth;

	if (strncmp(current_pos, "RAW_DEPTH_KINECT", 16) == 0) {
		current_pos += 16;
		kinect->id = CLF_READ_INT(&current_pos);
	} else {
		kinect->id = -1;
	}

	kinect->width = CLF_READ_INT(&current_pos);
	kinect->height = CLF_READ_INT(&current_pos);

	size = CLF_READ_INT(&current_pos);

	if(kinect->size != size) {
		kinect->size = size;
		kinect->depth = (float*) realloc(kinect->depth, kinect->size * sizeof(float));
		carmen_test_alloc(kinect->depth);
	}

	current_pos++;

	for(i = 0; i< kinect->size; i++)
	{
		r = *current_pos; current_pos++;
		first = GETINDEX(r);
		r = *current_pos; current_pos++;
		second = GETINDEX(r);
		r = *current_pos; current_pos++;
		third = GETINDEX(r);
		r = *current_pos; current_pos++;
		fourth = GETINDEX(r);

		kinect->depth[i] = convert_kinect_depth_raw_to_meters(HEX_TO_SHORT(fourth, third, second, first));
	}

	kinect->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&kinect->host, &current_pos);

	return current_pos;
}

char *carmen_string_to_kinect_video_message(char *string,
		carmen_kinect_video_message *kinect)
{
	char *current_pos = string;
	int i, size, r;
	unsigned char hi, lo;

	if (strncmp(current_pos, "RAW_VIDEO_KINECT", 16) == 0) {
		current_pos += 16;
		kinect->id = CLF_READ_INT(&current_pos);
	} else {
		kinect->id = -1;
	}

	kinect->width = CLF_READ_INT(&current_pos);
	kinect->height = CLF_READ_INT(&current_pos);

	size = CLF_READ_INT(&current_pos);

	if(kinect->size != size) {
		kinect->size = size;
		kinect->video = (unsigned char*) realloc(kinect->video, kinect->size * sizeof(unsigned char));
		carmen_test_alloc(kinect->video);
	}

	current_pos++;

	for(i = 0; i < kinect->size; i++)
	{
		r = *current_pos; current_pos++;
		hi = GETINDEX(r);

		r = *current_pos; current_pos++;
		lo = GETINDEX(r);

		kinect->video[i]= HEX_TO_RGB_BYTE(hi, lo);
	}

	kinect->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&kinect->host, &current_pos);

	return current_pos;
}

char* carmen_string_to_xsens_euler_message(char* string, carmen_xsens_global_euler_message* msg)
{
	char* current_pos = string;

	if (strncmp(current_pos, "XSENS_EULER ", 12) == 0){
		current_pos = carmen_next_word(current_pos);
	}

	msg->m_acc.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_acc.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_acc.z = CLF_READ_DOUBLE(&current_pos);

	msg->euler_data.m_pitch = CLF_READ_DOUBLE(&current_pos);
	msg->euler_data.m_roll = CLF_READ_DOUBLE(&current_pos);
	msg->euler_data.m_yaw = CLF_READ_DOUBLE(&current_pos);

	msg->m_mag.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_mag.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_mag.z = CLF_READ_DOUBLE(&current_pos);

	msg->m_gyr.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_gyr.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_gyr.z = CLF_READ_DOUBLE(&current_pos);

	msg->m_temp = CLF_READ_DOUBLE(&current_pos);
	msg->m_count = CLF_READ_DOUBLE(&current_pos);
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);

	copy_host_string(&msg->host, &current_pos);
	//msg->timestamp = CLF_READ_DOUBLE(&current_pos);

	return current_pos;
}

char* carmen_string_to_xsens_quat_message(char* string, carmen_xsens_global_quat_message* msg)
{
	char* current_pos = string;
	if (strncmp(current_pos, "XSENS_QUAT ", 11) == 0)
		current_pos = carmen_next_word(current_pos);

	msg->m_acc.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_acc.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_acc.z = CLF_READ_DOUBLE(&current_pos);

	msg->quat_data.m_data[0] = CLF_READ_DOUBLE(&current_pos);
	msg->quat_data.m_data[1] = CLF_READ_DOUBLE(&current_pos);
	msg->quat_data.m_data[2] = CLF_READ_DOUBLE(&current_pos);
	msg->quat_data.m_data[3] = CLF_READ_DOUBLE(&current_pos);

	msg->m_mag.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_mag.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_mag.z = CLF_READ_DOUBLE(&current_pos);

	msg->m_gyr.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_gyr.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_gyr.z = CLF_READ_DOUBLE(&current_pos);

	msg->m_temp = CLF_READ_DOUBLE(&current_pos);
	msg->m_count = CLF_READ_DOUBLE(&current_pos);
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);

	copy_host_string(&msg->host, &current_pos);
	//msg->timestamp = CLF_READ_DOUBLE(&current_pos);

	return current_pos;
}

char* carmen_string_to_xsens_matrix_message(char* string, carmen_xsens_global_matrix_message* msg)
{
	char* current_pos = string;
	if (strncmp(current_pos, "XSENS_MATRIX ", 13) == 0)
		current_pos = carmen_next_word(current_pos);

	msg->m_acc.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_acc.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_acc.z = CLF_READ_DOUBLE(&current_pos);

	msg->matrix_data.m_data[0][0] = CLF_READ_DOUBLE(&current_pos);
	msg->matrix_data.m_data[0][1] = CLF_READ_DOUBLE(&current_pos);
	msg->matrix_data.m_data[0][2] = CLF_READ_DOUBLE(&current_pos);

	msg->matrix_data.m_data[1][0] = CLF_READ_DOUBLE(&current_pos);
	msg->matrix_data.m_data[1][1] = CLF_READ_DOUBLE(&current_pos);
	msg->matrix_data.m_data[1][2] = CLF_READ_DOUBLE(&current_pos);

	msg->matrix_data.m_data[2][0] = CLF_READ_DOUBLE(&current_pos);
	msg->matrix_data.m_data[2][1] = CLF_READ_DOUBLE(&current_pos);
	msg->matrix_data.m_data[2][2] = CLF_READ_DOUBLE(&current_pos);

	msg->m_mag.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_mag.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_mag.z = CLF_READ_DOUBLE(&current_pos);

	msg->m_gyr.x = CLF_READ_DOUBLE(&current_pos);
	msg->m_gyr.y = CLF_READ_DOUBLE(&current_pos);
	msg->m_gyr.z = CLF_READ_DOUBLE(&current_pos);

	msg->m_temp = CLF_READ_DOUBLE(&current_pos);
	msg->m_count = CLF_READ_DOUBLE(&current_pos);
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);

	copy_host_string(&msg->host, &current_pos);
	//msg->timestamp = CLF_READ_DOUBLE(&current_pos);

	return current_pos;
}

char* carmen_string_to_xsens_mtig_message(char* string, carmen_xsens_mtig_message* msg)
{
	char* current_pos = string;
	if (strncmp(current_pos, "XSENS_MTIG ", 11) == 0)
		current_pos = carmen_next_word(current_pos);

	msg->quat.q0 = CLF_READ_DOUBLE(&current_pos);
	msg->quat.q1 = CLF_READ_DOUBLE(&current_pos);
	msg->quat.q2 = CLF_READ_DOUBLE(&current_pos);
	msg->quat.q3 = CLF_READ_DOUBLE(&current_pos);

	msg->acc.x = CLF_READ_DOUBLE(&current_pos);
	msg->acc.y = CLF_READ_DOUBLE(&current_pos);
	msg->acc.z = CLF_READ_DOUBLE(&current_pos);

	msg->gyr.x = CLF_READ_DOUBLE(&current_pos);
	msg->gyr.y = CLF_READ_DOUBLE(&current_pos);
	msg->gyr.z = CLF_READ_DOUBLE(&current_pos);

	msg->mag.x = CLF_READ_DOUBLE(&current_pos);
	msg->mag.y = CLF_READ_DOUBLE(&current_pos);
	msg->mag.z = CLF_READ_DOUBLE(&current_pos);

	msg->velocity.x = CLF_READ_DOUBLE(&current_pos);
	msg->velocity.y = CLF_READ_DOUBLE(&current_pos);
	msg->velocity.z = CLF_READ_DOUBLE(&current_pos);

	msg->latitude = CLF_READ_DOUBLE(&current_pos);
	msg->longitude = CLF_READ_DOUBLE(&current_pos);
	msg->height = CLF_READ_DOUBLE(&current_pos);

	msg->gps_fix = CLF_READ_INT(&current_pos);
	msg->xkf_valid = CLF_READ_INT(&current_pos);
	msg->sensor_ID = CLF_READ_INT(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}

char* carmen_string_to_bumblebee_basic_stereoimage_message_old(char* string, carmen_bumblebee_basic_stereoimage_message* msg)
{
	char *current_pos = string - 23;
	int i;
	int camera;

	if (strncmp(current_pos, "BUMBLEBEE_BASIC_STEREOIMAGE", 27) == 0) {
		current_pos += 27;
		camera = CLF_READ_INT(&current_pos);
	} else {
		camera = -1;
		if(camera) {}
		return NULL;
	}

	msg->width = CLF_READ_INT(&current_pos);
	msg->height = CLF_READ_INT(&current_pos);

	msg->image_size = CLF_READ_INT(&current_pos);
	msg->isRectified = CLF_READ_INT(&current_pos);

	msg->raw_left = (unsigned char*) malloc (msg->image_size * sizeof(unsigned char));
	msg->raw_right = (unsigned char*) malloc (msg->image_size * sizeof(unsigned char));

	for(i = 0; i < msg->image_size; i++)
		msg->raw_right[i] = CLF_READ_DOUBLE(&current_pos);

	for(i = 0; i < msg->image_size; i++)
		msg->raw_left[i] = CLF_READ_DOUBLE(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}

char* carmen_string_to_velodyne_partial_scan_message(char* string, carmen_velodyne_partial_scan_message* msg)
{
	char *current_pos = string;
	int i, j, r;
	unsigned char hi, lo, first, second, third, fourth;

	if (strncmp(current_pos, "VELODYNE_PARTIAL_SCAN", 21) == 0)
		current_pos += 21;

    // store the number of 32 laser shots allocated to avoid unecessary reallocs
    static int num_laser_shots_allocated = 0;

	msg->number_of_32_laser_shots = CLF_READ_INT(&current_pos);

	if (msg->partial_scan == NULL)
	{
		msg->partial_scan = (carmen_velodyne_32_laser_shot*) malloc (msg->number_of_32_laser_shots * sizeof(carmen_velodyne_32_laser_shot));
		carmen_test_alloc(msg->partial_scan);
		num_laser_shots_allocated = msg->number_of_32_laser_shots;
	}
	else if (num_laser_shots_allocated != msg->number_of_32_laser_shots)
	{
		msg->partial_scan = (carmen_velodyne_32_laser_shot*) realloc ((void *) msg->partial_scan, msg->number_of_32_laser_shots * sizeof(carmen_velodyne_32_laser_shot));
		carmen_test_alloc(msg->partial_scan);
		num_laser_shots_allocated = msg->number_of_32_laser_shots;
	}

	for(i = 0; i < msg->number_of_32_laser_shots; i++)
	{
		msg->partial_scan[i].angle = CLF_READ_INT(&current_pos) / 100.0;
		current_pos++;

		for(j=0; j < 32; j++)
		{
			r = *current_pos; current_pos++;
			first = GETINDEX(r);
			r = *current_pos; current_pos++;
			second = GETINDEX(r);
			r = *current_pos; current_pos++;
			third = GETINDEX(r);
			r = *current_pos; current_pos++;
			fourth = GETINDEX(r);

			msg->partial_scan[i].distance[j] = HEX_TO_SHORT(fourth, third, second, first);

			r = *current_pos; current_pos++;
			lo = GETINDEX(r);
			r = *current_pos; current_pos++;
			hi = GETINDEX(r);

			msg->partial_scan[i].intensity[j] = HEX_TO_RGB_BYTE(hi, lo);
		}
	}

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}

char* carmen_string_and_file_to_velodyne_partial_scan_message(char* string, carmen_velodyne_partial_scan_message* msg)
{
	int i;
	char *current_pos = string;

	if (strncmp(current_pos, "VELODYNE_PARTIAL_SCAN_IN_FILE", 29) == 0)
		current_pos += 29;

	static char path[1024];

	CLF_READ_STRING(path, &current_pos);
	msg->number_of_32_laser_shots = CLF_READ_INT(&current_pos);

    // store the number of 32 laser shots allocated to avoid unecessary reallocs
    static int num_laser_shots_allocated = 0;

	if(msg->partial_scan == NULL)
	{
		msg->partial_scan = (carmen_velodyne_32_laser_shot*) malloc (msg->number_of_32_laser_shots * sizeof(carmen_velodyne_32_laser_shot));
		num_laser_shots_allocated = msg->number_of_32_laser_shots;
	}
	else if (num_laser_shots_allocated != msg->number_of_32_laser_shots)
	{
		msg->partial_scan = (carmen_velodyne_32_laser_shot*) realloc (msg->partial_scan, msg->number_of_32_laser_shots * sizeof(carmen_velodyne_32_laser_shot));
		num_laser_shots_allocated = msg->number_of_32_laser_shots;
	}

	FILE *image_file = fopen(path, "rb");

	if (image_file)
	{
		for(i = 0; i < msg->number_of_32_laser_shots; i++)
		{
			fread(&(msg->partial_scan[i].angle), sizeof(double), 1, image_file);
			fread(msg->partial_scan[i].distance, sizeof(short), 32, image_file);
			fread(msg->partial_scan[i].intensity, sizeof(char), 32, image_file);
		}

		fclose(image_file);
	}
	else
		msg->number_of_32_laser_shots = 0;

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char* carmen_string_to_variable_velodyne_scan_message(char* string, carmen_velodyne_variable_scan_message* msg)
{
	char *current_pos = string;
	int i, j, r;
	int shot_size;
	int number_of_shots;
	unsigned char hi, lo, first, second, third, fourth;

	if (strncmp(current_pos, "VARIABLE_VELODYNE_SCAN", 22) == 0)
		current_pos += 22;

	number_of_shots = CLF_READ_INT(&current_pos);
	shot_size = CLF_READ_INT(&current_pos);

	if(msg->partial_scan == NULL)
	{
		msg->partial_scan = (carmen_velodyne_shot *) calloc (number_of_shots, sizeof(carmen_velodyne_shot));
		carmen_test_alloc(msg->partial_scan);
		msg->number_of_shots = number_of_shots;
	}

	if (msg->number_of_shots != number_of_shots)
	{
		for(i = 0; i < msg->number_of_shots; i++)
		{
			free(msg->partial_scan[i].distance);
			free(msg->partial_scan[i].intensity);
		}

		msg->partial_scan = (carmen_velodyne_shot *) realloc (msg->partial_scan, number_of_shots * sizeof(carmen_velodyne_shot));
		memset(msg->partial_scan, 0, number_of_shots * sizeof(carmen_velodyne_shot));
		msg->number_of_shots = number_of_shots;
	}

	for (i = 0; i < msg->number_of_shots; i++)
	{
		msg->partial_scan[i].angle = CLF_READ_INT(&current_pos) / 100.0;
		msg->partial_scan[i].shot_size = shot_size;

		current_pos++;

		if (msg->partial_scan[i].distance == NULL)
		{
			msg->partial_scan[i].distance = (unsigned short *) calloc (shot_size, sizeof(unsigned short));
			msg->partial_scan[i].intensity = (unsigned char *) calloc (shot_size, sizeof(unsigned char));
		}

		for(j = 0; j < shot_size; j++)
		{
			r = *current_pos; current_pos++;
			first = GETINDEX(r);
			r = *current_pos; current_pos++;
			second = GETINDEX(r);
			r = *current_pos; current_pos++;
			third = GETINDEX(r);
			r = *current_pos; current_pos++;
			fourth = GETINDEX(r);

			msg->partial_scan[i].distance[j] = HEX_TO_SHORT(fourth, third, second, first);

			r = *current_pos; current_pos++;
			lo = GETINDEX(r);
			r = *current_pos; current_pos++;
			hi = GETINDEX(r);

			msg->partial_scan[i].intensity[j] = HEX_TO_RGB_BYTE(hi, lo);
		}
	}

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char* carmen_string_to_velodyne_gps_message(char* string, carmen_velodyne_gps_message* msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "VELODYNE_GPS", 12) == 0)
		current_pos += 12;

	msg->gyro1 = CLF_READ_DOUBLE(&current_pos);
	msg->gyro2 = CLF_READ_DOUBLE(&current_pos);
	msg->gyro3 = CLF_READ_DOUBLE(&current_pos);

	msg->temp1 = CLF_READ_DOUBLE(&current_pos);
	msg->temp2 = CLF_READ_DOUBLE(&current_pos);
	msg->temp3 = CLF_READ_DOUBLE(&current_pos);

	msg->accel1_x = CLF_READ_DOUBLE(&current_pos);
	msg->accel2_x = CLF_READ_DOUBLE(&current_pos);
	msg->accel3_x = CLF_READ_DOUBLE(&current_pos);

	msg->accel1_y = CLF_READ_DOUBLE(&current_pos);
	msg->accel2_y = CLF_READ_DOUBLE(&current_pos);
	msg->accel3_y = CLF_READ_DOUBLE(&current_pos);
	msg->utc_time = CLF_READ_INT(&current_pos);
	current_pos = carmen_next_word(current_pos);
	msg->status = CLF_READ_CHAR(&current_pos);
	msg->latitude = CLF_READ_DOUBLE(&current_pos);
	current_pos = carmen_next_word(current_pos);
	msg->latitude_hemisphere = CLF_READ_CHAR(&current_pos);
	msg->longitude = CLF_READ_DOUBLE(&current_pos);
	current_pos = carmen_next_word(current_pos);
	msg->longitude_hemisphere = CLF_READ_CHAR(&current_pos);
	msg->speed_over_ground = CLF_READ_DOUBLE(&current_pos);
	msg->course_over_ground = CLF_READ_DOUBLE(&current_pos);
	msg->utc_date = CLF_READ_INT(&current_pos);
	msg->magnetic_variation_course = CLF_READ_DOUBLE(&current_pos);
	msg->magnetic_variation_direction = CLF_READ_DOUBLE(&current_pos);
	current_pos = carmen_next_word(current_pos);
	msg->mode_indication = CLF_READ_CHAR(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char* carmen_string_to_ford_escape_estatus_message(char* string, carmen_ford_escape_status_message* msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "FORD_ESCAPE_STATUS", 18) == 0)
		current_pos += 18;

	msg->g_XGV_throttle = CLF_READ_DOUBLE(&current_pos);
	msg->g_XGV_steering = CLF_READ_DOUBLE(&current_pos);
	msg->g_XGV_brakes = CLF_READ_DOUBLE(&current_pos);

	msg->g_XGV_component_status = CLF_READ_UNSIGNED_INT(&current_pos);

	msg->g_XGV_main_propulsion = CLF_READ_INT(&current_pos);
	msg->g_XGV_main_fuel_supply = CLF_READ_INT(&current_pos);
	msg->g_XGV_parking_brake = CLF_READ_INT(&current_pos);
	msg->g_XGV_gear = CLF_READ_INT(&current_pos);

	msg->g_XGV_turn_signal = CLF_READ_INT(&current_pos);
	msg->g_XGV_horn_status = CLF_READ_INT(&current_pos);
	msg->g_XGV_headlights_status = CLF_READ_INT(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char* carmen_string_to_globalpos_message(char* string, carmen_localize_ackerman_globalpos_message* msg)
{
	char *current_pos = string;

	if (strncmp(current_pos, "GLOBALPOS_ACK", 13) == 0)
		current_pos += 13;

	msg->globalpos.x = CLF_READ_DOUBLE(&current_pos);
	msg->globalpos.y = CLF_READ_DOUBLE(&current_pos);
	msg->globalpos.theta = CLF_READ_DOUBLE(&current_pos);
	msg->globalpos_std.x = CLF_READ_DOUBLE(&current_pos);
	msg->globalpos_std.y = CLF_READ_DOUBLE(&current_pos);
	msg->globalpos_std.theta = CLF_READ_DOUBLE(&current_pos);
	msg->odometrypos.x = CLF_READ_DOUBLE(&current_pos);
	msg->odometrypos.y = CLF_READ_DOUBLE(&current_pos);
	msg->odometrypos.theta = CLF_READ_DOUBLE(&current_pos);
	msg->pose.orientation.pitch = CLF_READ_DOUBLE(&current_pos);
	msg->pose.orientation.roll = CLF_READ_DOUBLE(&current_pos);
	msg->pose.orientation.yaw = CLF_READ_DOUBLE(&current_pos);
	msg->pose.position.x = CLF_READ_DOUBLE(&current_pos);
	msg->pose.position.y = CLF_READ_DOUBLE(&current_pos);
	msg->pose.position.z = CLF_READ_DOUBLE(&current_pos);
	msg->velocity.x = CLF_READ_DOUBLE(&current_pos);
	msg->velocity.y = CLF_READ_DOUBLE(&current_pos);
	msg->velocity.z = CLF_READ_DOUBLE(&current_pos);
	msg->v = CLF_READ_DOUBLE(&current_pos);
	msg->phi = CLF_READ_DOUBLE(&current_pos);
	msg->globalpos_xy_cov = CLF_READ_DOUBLE(&current_pos);
	msg->converged = CLF_READ_INT(&current_pos);
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char* carmen_string_to_bumblebee_basic_stereoimage_message(char* string, carmen_bumblebee_basic_stereoimage_message* msg)
{
	char r;
	unsigned char hi, lo;
	char *current_pos = string - 29;
	int camera, i;

	if (strncmp(current_pos, "BUMBLEBEE_BASIC_STEREOIMAGE", 27) == 0) {
		current_pos += 27;
		camera = CLF_READ_INT(&current_pos);
	} else {
		camera = -1;
		if(camera) {}
		return NULL;
	}

	msg->width = CLF_READ_INT(&current_pos);
	msg->height = CLF_READ_INT(&current_pos);

	msg->image_size = CLF_READ_INT(&current_pos);
	msg->isRectified = CLF_READ_INT(&current_pos);

	if(msg->raw_left == NULL)
		msg->raw_left = (unsigned char*) malloc (msg->image_size * sizeof(unsigned char));

	if(msg->raw_right == NULL)
		msg->raw_right = (unsigned char*) malloc (msg->image_size * sizeof(unsigned char));


	current_pos++;

	for(i = 0; i < msg->image_size; i++)
	{
		r = *current_pos; current_pos++;
		hi = GETINDEX(r);

		r = *current_pos; current_pos++;
		lo = GETINDEX(r);

		msg->raw_right[i] = HEX_TO_RGB_BYTE(hi, lo);
	}

	current_pos++;

	for(i = 0; i < msg->image_size; i++)
	{
		r = *current_pos; current_pos++;
		hi = GETINDEX(r);

		r = *current_pos; current_pos++;
		lo = GETINDEX(r);

		msg->raw_left[i] = HEX_TO_RGB_BYTE(hi, lo);
	}

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char* carmen_string_and_file_to_bumblebee_basic_stereoimage_message(char* string, carmen_bumblebee_basic_stereoimage_message* msg)
{
	int tam = strlen("BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILEX ");
	char *current_pos = string - tam;
	int camera;

	if (strncmp(current_pos, "BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE", tam - 2 /*ignore the cam number and the space*/) == 0)
	{
		current_pos += tam;
		camera = CLF_READ_INT(&current_pos);
	}
	else
	{
		camera = -1;
		if(camera) {}
		return NULL;
	}

	static char path[1024];

	CLF_READ_STRING(path, &current_pos);

    msg->width = CLF_READ_INT(&current_pos);
    msg->height = CLF_READ_INT(&current_pos);
    msg->image_size = CLF_READ_INT(&current_pos);
    msg->isRectified = CLF_READ_INT(&current_pos);

	if(msg->raw_left == NULL)
		msg->raw_left = (unsigned char*) malloc (msg->image_size * sizeof(unsigned char));

	if(msg->raw_right == NULL)
		msg->raw_right = (unsigned char*) malloc (msg->image_size * sizeof(unsigned char));

	if (0)//strcmp("png", path + strlen(path) - 3) == 0) // ZED Camera
	{
		cv::Mat img = cv::imread(path);
		cv::Mat left = img(cv::Rect(0, 0, img.cols / 2, img.rows)).clone();
		cv::Mat right = img(cv::Rect(img.cols / 2, 0, img.cols / 2, img.rows)).clone();

		// DEBUG:
		//cv::imshow("img", img);
		//cv::imshow("left", left);
		//cv::imshow("right", right);
		//cv::waitKey(10);

		memcpy(msg->raw_left, left.data, sizeof(unsigned char) * msg->image_size);
		memcpy(msg->raw_right, right.data, sizeof(unsigned char) * msg->image_size);
	}
	else
	{
		FILE *image_file = fopen(path, "rb");

		if (image_file)
		{
			fread(msg->raw_left, msg->image_size, sizeof(unsigned char), image_file);
			fread(msg->raw_right, msg->image_size, sizeof(unsigned char), image_file);

			fclose(image_file);
		}
		else
			msg->image_size = 0;
	}

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char* carmen_string_to_web_cam_message(char *string, carmen_web_cam_message *msg)
{
	int i;
	char *current_pos = string;

	if (strncmp(current_pos, "WEB_CAM_IMAGE", 13) == 0)
		current_pos += 13;

	msg->width = CLF_READ_INT(&current_pos);
	msg->height = CLF_READ_INT(&current_pos);
	msg->image_size = CLF_READ_INT(&current_pos);

	if (msg->img_data == NULL)
		msg->img_data = (char*) calloc (msg->image_size, sizeof(char));

	for(i = 0; i < (msg->image_size); i++)
		msg->img_data[i] = (char) CLF_READ_INT(&current_pos);

	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}


char *
carmen_string_to_base_ackerman_motion_message(char *string, carmen_base_ackerman_motion_command_message *msg)
{
	int i;
	char *current_pos = string;
	int current_motion_command_vector_size;

	if (strncmp(current_pos, "BASEMOTION_ACK", 14) == 0)
		current_pos += 14;

	current_motion_command_vector_size = CLF_READ_INT(&current_pos);

	if (msg->motion_command == NULL)
	{
		msg->num_motion_commands = current_motion_command_vector_size;
		msg->motion_command = (carmen_ackerman_motion_command_t *) calloc(msg->num_motion_commands, sizeof(carmen_ackerman_motion_command_t));
	}
	else if (msg->num_motion_commands != current_motion_command_vector_size)
	{
		msg->num_motion_commands = current_motion_command_vector_size;
		msg->motion_command = (carmen_ackerman_motion_command_t *) realloc(msg->motion_command, msg->num_motion_commands * sizeof(carmen_ackerman_motion_command_t));
	}

	for (i = 0; i < msg->num_motion_commands; i++)
	{
		msg->motion_command[i].v = CLF_READ_DOUBLE(&current_pos);
		msg->motion_command[i].phi = CLF_READ_DOUBLE(&current_pos);
		msg->motion_command[i].time = CLF_READ_DOUBLE(&current_pos);
	}
	msg->timestamp = CLF_READ_DOUBLE(&current_pos);
	copy_host_string(&msg->host, &current_pos);

	return current_pos;
}
