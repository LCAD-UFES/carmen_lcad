
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


carmen_gps_xyz_message read_gps(FILE *f)
{
	static char dummy[256];

    double lt_dm, lt, lg_dm, lg, sea_level;
    char lt_orientation, lg_orientation;
    int quality, gps_id;

    carmen_gps_xyz_message m;
    memset(&m, 0, sizeof(m));

    fscanf(f, "%s", dummy);
    fscanf(f, "%d", &gps_id);

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
	m.gps_quality = quality;

    return m;
}


int
copy_fields(FILE *f, FILE *g, int n, char check)
{
	static char data[256];

	for (int i = 0; i < n; i++)
	{
		fscanf(f, "\n%s", data);

		if (i == 0 && data[0] != check)
			return 0;

		fprintf(g, "%s ", data);
	}

	return 1;
}


void
write_gps(carmen_gps_xyz_message m, int number, FILE *g)
{
	fprintf(g, "GPS_XYZ %d %lf %lf %d %lf ", number, m.x, m.y, m.gps_quality, m.timestamp);
}


int
main(int argc, char **argv)
{
	if (argc < 3)
	{
		printf("\nError: Use %s <sync_file> <output_file>\n\n", argv[0]);
		exit(0);
	}

	FILE *f = fopen(argv[1], "r");
	FILE *g = fopen(argv[2], "w");

	int success = 1;

	while (!feof(f))
	{
		success = copy_fields(f, g, 4, 'V'); // velodyne

		if (!success)
			break;

		success = copy_fields(f, g, 7, 'B'); // bumblebee
		write_gps(read_gps(f), 1, g);
		copy_fields(f, g, 5, 'N'); // gps NMEAHDT
		copy_fields(f, g, 4, 'R'); // robot ackerman
		copy_fields(f, g, 17, 'X'); // xsens

		fprintf(g, "\n");
	}

	fclose(f);
	fclose(g);

	return 0;
}
