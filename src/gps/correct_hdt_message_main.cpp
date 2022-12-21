#include <stdio.h>
#include <stdlib.h>
#include <carmen/carmen.h>


int
main(int argc, char **argv)
{
	if (argc != 4)
		exit(printf("Error! Wrong number of parameters.\n Usage %s log_in log_out added_heading_angle\n", argv[0]));

	FILE *log_in = fopen(argv[1], "r");
	FILE *log_out = fopen(argv[2], "w");

	printf("correcting %s -> %s: adding %s\n", argv[1], argv[2], argv[3]);
	double correction = atof(argv[3]);

	// NMEAHDT 1 1.461469 1 1671656057.524407 gps_nmea_trimble@lume02 1.532716
	char line[1000000];
	char message[2028];
	int id;
	double heading;
	int valid;
	double timestamp;
	char host[2048];
	double t;
	while (fgets(line, 1000000 - 1, log_in))
	{
		sscanf(line, "%s ", message);
		if (strcmp(message, "NMEAHDT") == 0)
		{
			sscanf(line, "%s %d %lf %d %lf %s %lf", message, &id, &heading, &valid, &timestamp, host, &t);
//			printf(" %s", line);
//			printf("*%s %d %lf %d %lf %s %lf\n", message, id, carmen_normalize_theta(heading + correction), valid, timestamp, host, t);
			fprintf(log_out, "%s %d %lf %d %lf %s %lf\n", message, id, carmen_normalize_theta(heading + correction), valid, timestamp, host, t);
		}
		else
			fprintf(log_out, "%s", line);
	}

	carmen_normalize_theta(1.0);
	return (0);
}

