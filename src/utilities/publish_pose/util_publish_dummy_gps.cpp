#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/carmen_gps.h>
#include <carmen/gps_nmea_interface.h>
#include "../../gps/gps.h"

carmen_gps_gpgga_message *carmen_gpgga_ptr = NULL;
carmen_gps_gphdt_message *carmen_gphdt_ptr = NULL;
int gps_number = 1;
int autonomous = 0;


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("util_publish_dummy_gps: disconnected\n");
		exit(0);
	}
}


void
ipc_initialize_messages( void )
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_GPS_GPGGA_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			      CARMEN_GPS_GPGGA_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_GPGGA_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPHDT_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			      CARMEN_GPS_GPHDT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_GPHDT_MESSAGE_NAME);

	int gps_number = 1; // This says it is a Trimble GPS
	static carmen_gps_gpgga_message gpgga;
	static carmen_gps_gphdt_message gphdt;
	static carmen_gps_gprmc_message gprmc;

	carmen_erase_structure(&gpgga, sizeof(carmen_gps_gpgga_message));
	carmen_erase_structure(&gphdt, sizeof(carmen_gps_gphdt_message));
	carmen_erase_structure(&gprmc, sizeof(carmen_gps_gprmc_message));

	gpgga.host = carmen_get_host();
	gphdt.host = carmen_get_host();
	gprmc.host = carmen_get_host();

	carmen_gpgga_ptr = &gpgga;
	carmen_gpgga_ptr->nr = gps_number;
	carmen_gphdt_ptr = &gphdt;
	carmen_gphdt_ptr->nr = gps_number;
}



int
publish_carmen_gps_gpgga_message(int gps_number)
{
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_gpgga_ptr != NULL)
	{
		carmen_gpgga_ptr->nr = gps_number;

		err = IPC_publishData (CARMEN_GPS_GPGGA_MESSAGE_NAME, carmen_gpgga_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPGGA_MESSAGE_NAME);
		printf("mensagem gga publicada!\n");

		return (1);
	}
	else
		return (0);
}


void
publish_carmen_gps_gphdt_message(int gps_number)
{
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_gphdt_ptr != NULL)
	{
		carmen_gphdt_ptr->nr = gps_number;
		err = IPC_publishData (CARMEN_GPS_GPHDT_MESSAGE_NAME, carmen_gphdt_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPHDT_MESSAGE_NAME);
		printf("mensagem hdt publicada!\n");
	}
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



char *carmen_string_to_publish_gps_gpgga_message(char *string,
		carmen_gps_gpgga_message *gps_msg)
{
	char *current_pos = string;

	current_pos = carmen_next_word(current_pos);

	gps_msg->nr               = CLF_READ_INT(&current_pos); fprintf(stderr, " gps number:     %d\n", gps_msg->nr);
	gps_msg->utc              = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " utc:            %lf\n", gps_msg->utc);
	gps_msg->latitude_dm      = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " latitude (DM):  %.7lf\n", gps_msg->latitude_dm);
	gps_msg->latitude         = carmen_global_convert_degmin_to_double(gps_msg->latitude_dm); fprintf(stderr, " latitude:       %.9lf\n", gps_msg->latitude);
	current_pos = carmen_next_word(current_pos);
	gps_msg->lat_orient       = CLF_READ_CHAR(&current_pos); fprintf(stderr, " lat_orient:     %c\n", gps_msg->lat_orient);
	gps_msg->longitude_dm     = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " longitude (DM): %.7lf\n", gps_msg->longitude_dm);
	gps_msg->longitude        = carmen_global_convert_degmin_to_double(gps_msg->longitude_dm); fprintf(stderr, " longitude:      %.9lf\n", gps_msg->longitude);
	current_pos = carmen_next_word(current_pos);
	gps_msg->long_orient      = CLF_READ_CHAR(&current_pos); fprintf(stderr, " long_orient:    %c\n", gps_msg->long_orient);
	gps_msg->gps_quality      = CLF_READ_INT(&current_pos); fprintf(stderr, " gps_quality:    %d\n", gps_msg->gps_quality);
	gps_msg->num_satellites   = CLF_READ_INT(&current_pos); fprintf(stderr, " num_satellites: %d\n", gps_msg->num_satellites);
	gps_msg->hdop             = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " hdop:           %lf\n", gps_msg->hdop);
	gps_msg->sea_level        = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " sea_level:      %lf\n", gps_msg->sea_level);
	gps_msg->altitude         = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " altitude:       %lf\n", gps_msg->altitude);
	gps_msg->geo_sea_level    = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " geo_sea_level:  %lf\n", gps_msg->geo_sea_level);
	gps_msg->geo_sep          = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " geo_sep:        %lf\n", gps_msg->geo_sep);
	gps_msg->data_age         = CLF_READ_INT(&current_pos); fprintf(stderr, " data_age:       %d\n", gps_msg->data_age);
	gps_msg->timestamp        = CLF_READ_DOUBLE(&current_pos); fprintf(stderr, " timestamp:       %lf\n", gps_msg->timestamp);
	fflush(stderr);
	gps_msg->host = (char*) malloc(512*sizeof(char));
	copy_host_string(&gps_msg->host, &current_pos);

	gps_number = gps_msg->nr;

	if(autonomous == 1)
		gps_msg->timestamp = carmen_get_time();

	publish_carmen_gps_gpgga_message(gps_number);

	return current_pos;
}

char *carmen_string_to_publish_gps_gphdt_message(char *string,
		carmen_gps_gphdt_message *gps_msg)
{
	char *current_pos = string;

	current_pos = carmen_next_word(current_pos);

	gps_msg->nr               = CLF_READ_INT(&current_pos);
	gps_msg->heading          = CLF_READ_DOUBLE(&current_pos);
	gps_msg->valid            = CLF_READ_INT(&current_pos);
	gps_msg->timestamp        = CLF_READ_DOUBLE(&current_pos);
	gps_msg->host = (char*) malloc(512*sizeof(char));
	copy_host_string(&gps_msg->host, &current_pos);

	if(gps_number == 1)
	{
		if(autonomous == 1)
			gps_msg->timestamp = carmen_get_time();
		publish_carmen_gps_gphdt_message(gps_number);
	}

	return current_pos;
}


int
main(int argc, char **argv)
{
	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);
	ipc_initialize_messages();
	int size = 8000;
	char line[size];
	int num_messages_per_second;

	fprintf(stderr, "INFO: ************************\n");
	fprintf(stderr, "INFO: ********* DUMMY GPS*****\n");
	fprintf(stderr, "INFO: ************************\n");

	FILE *dummy_gps_file = fopen(argv[1], "r");
	num_messages_per_second = atoi(argv[2]);
	if(argc == 4 && strcmp("-autonomous", argv[3]) == 0)
		autonomous = 1;

	double sleep_time = (1.0 / (double) num_messages_per_second) * 10e5;
	int cont = 0;


//	while (1)
//	{
//		FILE *dummy_gps_file = fopen(argv[1], "r");
		while (fgets(line, size, dummy_gps_file) != NULL)
		{
			if(strncmp(line, "NMEAGGA", 7) == 0)
			{
				printf("%s\n", line);
				carmen_string_to_publish_gps_gpgga_message(line, carmen_gpgga_ptr);
				cont++;

			}
			else if(strncmp(line, "NMEAHDT", 7) == 0)
			{
				carmen_string_to_publish_gps_gphdt_message(line, carmen_gphdt_ptr);
				cont++;
			}

			if(cont != 0 && cont % 2 == 0)
			{
				usleep((int) sleep_time);
				printf("Published GPS!\n");
			}

		}
//		fclose(dummy_gps_file);
//		usleep((int) sleep_time);
//		printf("Published GPS!\n");
//	}
	fclose(dummy_gps_file);

	return 0;
}
