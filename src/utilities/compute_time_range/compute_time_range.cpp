#include <stdlib.h>
#include <vector>
#include <carmen/carmen.h>
#include <carmen/util_io.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/Gdc_Coord_3d.h>
#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_To_Utm_Converter.h>

#define MAX_LINE_LENGTH (5 * 4000000)

typedef struct
{
	carmen_pose_3D_t pose;
	double time;
}StampedPose;

std::vector<StampedPose> gps_queue;
std::vector<double> gps_queue_stds;


int 
read_gps(FILE *f, int gps_to_use, carmen_gps_xyz_message &m)
{
	static char dummy[128];

	double lt_dm, lt, lg_dm, lg, sea_level;
	char lt_orientation, lg_orientation;
	int quality, gps_id;

	memset(&m, 0, sizeof(m));

	fscanf(f, "%d", &gps_id);
	m.nr = gps_id;

	if (gps_to_use == gps_id)
	{
		fscanf(f, "%s", dummy);

		fscanf(f, "%lf", &lt_dm);
		fscanf(f, " %c ", &lt_orientation); // read a char ignoring space
		fscanf(f, "%lf", &lg_dm);
		fscanf(f, " %c ", &lg_orientation); // read a char ignoring space
		fscanf(f, "%d", &quality);
		m.gps_quality = quality;

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

        return 1;
	}

	return 0;
}



void
gps_xyz_message_handler(carmen_gps_xyz_message *message, int gps_to_use, double gps_latency, double hdt_yaw, double hdt_timestamp)
{
	if (message->nr != gps_to_use)
		return;

	StampedPose pose;
	pose.pose.position.x = message->x;
	pose.pose.position.y = message->y;
	pose.pose.position.z = 0.0;
	pose.pose.orientation.roll = 0.0;
	pose.pose.orientation.pitch = 0.0;
	if (gps_queue.size() > 0)
	{
		if (fabs(hdt_timestamp - message->timestamp) < 1.0)
			pose.pose.orientation.yaw = hdt_yaw;
		else
			pose.pose.orientation.yaw = carmen_normalize_theta(atan2(message->y - gps_queue[gps_queue.size() - 1].pose.position.y,
						  	  	  	  	  	  	  	  	  	  	  	 message->x - gps_queue[gps_queue.size() - 1].pose.position.x));
	}
	else
		pose.pose.orientation.yaw = 0.0;
	pose.time = message->timestamp - gps_latency;

	double gps_std;
	switch (message->gps_quality)
	{
		case 1:
			gps_std = 8.0;
			break;
		case 2:
			gps_std = 4.0;
			break;
		case 4:
			gps_std = 1.0;
			break;
		case 5:
        case 6:
			gps_std = 2.0;
			break;
		default:
			gps_std = DBL_MAX;
	}

    gps_queue.push_back(pose);
    gps_queue_stds.push_back(gps_std);
}


void
gps_hdt_handler(FILE *f, double &hdt_yaw, double &hdt_timestamp)
{
	int gps_id;
	int valid;

	fscanf(f, "%d %lf %d %lf", &gps_id, &hdt_yaw, &valid, &hdt_timestamp);

	if (!valid)
		hdt_timestamp = 0.0;
}


int
read_gps_from_log(const char *log_filename, int gps_to_use)
{
    static char tag[256];
    int num_gps_messages = 0;
	double gps_latency = 0.0, hdt_yaw = 0.0, hdt_timestamp = 0.0;

    FILE *f = safe_fopen(log_filename, "r");

    while(!feof(f))
	{
		fscanf(f, "\n%s", tag);

		if (!strcmp(tag, "NMEAGGA"))
		{
            carmen_gps_xyz_message m;
            if (read_gps(f, gps_to_use, m))
            {
                gps_xyz_message_handler(&m, gps_to_use, gps_latency, hdt_yaw, hdt_timestamp);
			    num_gps_messages++;
            }
		}
        else if (!strcmp(tag, "NMEAHDT"))
		{
			gps_hdt_handler(f, hdt_yaw, hdt_timestamp);
		}
    }
    fclose(f);

    return num_gps_messages;
}


StampedPose
read_log_and_compare_poses_from_gps(double x, double y, char *outfile)
{
    double dist, initial_time = .0, choosen_dist = DBL_MAX;
    StampedPose choosen = gps_queue[0];
    initial_time = gps_queue[0].time;

    for (StampedPose item : gps_queue)
    {
        dist = ((item.pose.position.x - x) * (item.pose.position.x - x) + (item.pose.position.y - y) * (item.pose.position.y - y));
        if (dist < choosen_dist)
        {
            choosen_dist = dist;
            choosen = item;
        }
    }

    printf("%lf\t%lf\n", choosen.pose.position.x, choosen.pose.position.y);
    printf("dist: %lf\n", choosen_dist);
    printf("time: %lf\n\n", choosen.time - initial_time);

    FILE *fp = fopen(outfile, "w");
    fprintf(fp, "%lf\n", choosen.pose.position.x);
    fprintf(fp, "%lf\n", choosen.pose.position.y);
    fprintf(fp, "%lf\n", choosen_dist);
    fprintf(fp, "%lf\n", choosen.time - initial_time);
    fclose(fp);

    return choosen;
}


void
check_args(int argc, char **argv)
{
    if (argc < 5)
    {
        printf("usage: %s <x> <y> <log_path> <outfile> <OPTIONAL gps_to_use>", argv[0]);
        exit(1);
    }
}

int 
main(int argc, char **argv)
{
	check_args(argc, argv);

    StampedPose choosen;

    if (argc == 6)
        read_gps_from_log(argv[3], atoi(argv[5]));
    else
        read_gps_from_log(argv[3], 1);

    choosen = read_log_and_compare_poses_from_gps(atof(argv[1]), atof(argv[2]), argv[4]);

    return 0;
}