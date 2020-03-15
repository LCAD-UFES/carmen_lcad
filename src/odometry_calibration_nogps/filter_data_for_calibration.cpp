
#include <pso.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <carmen/carmen.h>
#include <carmen/Gdc_Coord_3d.h>
#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_To_Utm_Converter.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/gps_xyz_interface.h>


using namespace std;


#define MAX_LINE_LENGTH (5*4000000)


carmen_robot_ackerman_velocity_message read_odometry(FILE *f)
{
	carmen_robot_ackerman_velocity_message m;
	fscanf(f, "%lf %lf %lf", &m.v, &m.phi, &m.timestamp);
	return m;
}


carmen_gps_xyz_message read_gps(FILE *f, int gps_to_use)
{
	static char dummy[128];

	double lt_dm, lt, lg_dm, lg, sea_level;
	char lt_orientation, lg_orientation;
	int quality, gps_id;

	carmen_gps_xyz_message m;
	memset(&m, 0, sizeof(m));

	fscanf(f, "%d", &gps_id);

	if (gps_to_use == gps_id)
	{
		fscanf(f, "%s", dummy);

		fscanf(f, "%lf", &lt_dm);
		fscanf(f, " %c ", &lt_orientation); // read a char ignoring space
		fscanf(f, "%lf", &lg_dm);
		fscanf(f, " %c ", &lg_orientation); // read a char ignoring space
		fscanf(f, "%d", &quality);

		fscanf(f, "%s", dummy);
		fscanf(f, "%s", dummy);

		fscanf(f, "%lf", &sea_level);

		fscanf(f, "%s", dummy);
		fscanf(f, "%s", dummy);
		fscanf(f, "%s", dummy);
		fscanf(f, "%s", dummy);

		fscanf(f, "%lf", &m.timestamp);

		lt = carmen_global_convert_degmin_to_double(lt_dm);
		lg = carmen_global_convert_degmin_to_double(lg_dm);

		// verify the latitude and longitude orientations
		if ('S' == lt_orientation) lt = -lt;
		if ('W' == lg_orientation) lg = -lg;

		// convert to x and y coordinates
		Gdc_Coord_3d gdc = Gdc_Coord_3d(lt, lg, sea_level);

		// Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
		Utm_Coord_3d utm;
		Gdc_To_Utm_Converter::Init();
		Gdc_To_Utm_Converter::Convert(gdc , utm);

		m.x = utm.y;
		m.y = -utm.x;
	}

	return m;
}


void
process_gps(carmen_gps_xyz_message &m, vector<carmen_robot_ackerman_velocity_message> &odoms)
{
	if (odoms.size() == 0 || m.timestamp == 0)
		return;

	int near = 0;
	
	for (size_t i = 0; i < odoms.size(); i++)
		if (fabs(m.timestamp - odoms[i].timestamp) < fabs(m.timestamp - odoms[near].timestamp))
			near = i;

	printf("%lf %lf %lf %lf %lf %lf\n", odoms[near].v, odoms[near].phi, odoms[near].timestamp, m.x, m.y, m.timestamp);

}


void
read_and_print_filtered_data(char *filename, int gps_to_use)
{
	static char tag[256];
	static char line[MAX_LINE_LENGTH];

	FILE *f = fopen(filename, "r");

	vector<carmen_robot_ackerman_velocity_message> odoms;

	if (f != NULL)
	{
		while(!feof(f))
		{
			fscanf(f, "\n%s", tag);

			if (!strcmp(tag, "NMEAGGA"))
			{
				carmen_gps_xyz_message m = read_gps(f, gps_to_use);
				process_gps(m, odoms);
			}

			else if (!strcmp(tag, "ROBOTVELOCITY_ACK"))
			{
				carmen_robot_ackerman_velocity_message m = read_odometry(f);
				odoms.push_back(m);
			}
			else
				fscanf(f, "%[^\n]\n", line);
		}
	}
	else
		exit(printf("File '%s' not found!\n", filename));

	fclose(f);
}


int 
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <log_path> <gps_to_use: default: 1>\n", argv[0]));

	int gps_to_use = 1;

	if (argc == 3)
		gps_to_use = atoi(argv[2]);

	read_and_print_filtered_data(argv[1], gps_to_use);

	return 0;
}

