#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/fused_odometry_interface.h>
#include <kml/base/file.h>
#include <kml/engine.h>
#include <kml/dom.h>
#include <kml/engine/kml_file.h>
#include "rddf_util.h"


void
carmen_rddf_convert(char *carmen_rddf_kml, char *carmen_rddf_txt)
{
	int annotation = 0;
	carmen_fused_odometry_message message;
	placemark_vector_t placemark_vector;

	FILE *fptr = fopen(carmen_rddf_txt, "w");
	carmen_rddf_play_open_kml(carmen_rddf_kml, &placemark_vector);

	for (unsigned int i = 0; i < placemark_vector.size(); i++)
	{
		if (carmen_rddf_play_copy_kml(placemark_vector[i], &message, &annotation))
		{
			fprintf(fptr, "%lf %lf %lf %lf %lf %lf\n",
				message.pose.position.x, message.pose.position.y,
				message.pose.orientation.yaw, message.velocity.x, message.phi,
				message.timestamp);
		}
	}

	fclose(fptr);
}


int
main(int argc, char **argv)
{
	setlocale(LC_ALL, "C");

	if (argc < 2)
		exit(printf("Error: Use %s <rddf-kml> <rddf-txt>\n", argv[0]));

	char *carmen_rddf_kml = argv[1];
	char *carmen_rddf_txt = argv[2];

	carmen_rddf_convert(carmen_rddf_kml, carmen_rddf_txt);

	return (0);
}


