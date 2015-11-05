#include <carmen/carmen.h>

#define DEPTH_WIDTH_PADDING 8

static float fov_in_degrees = 0.0;
static double timestamp_depth = 0.0;
static double laser_ranges[4096];
static carmen_laser_laser_message message_laser;

int is_new_msg(double timestamp_new, double timestamp_old){
	return (timestamp_new - timestamp_old) > 0.0 ? 1 : 0;
}

void
convert_depthmap_to_beams(double* beams, float fov_horiz_in_degrees, int number_of_angular_steps, float max_range,
		float* depthmap, int depthmap_width, int depthmap_height, int depthmap_size)
{
	int j, i = number_of_angular_steps-1;
	float alpha, alpha_step, z;
	float focal_length_in_pixels = ((float)depthmap_width ) / (2.0 * tan(carmen_degrees_to_radians(fov_horiz_in_degrees/ 2.0)));
	int x, y = depthmap_height/2;

	alpha_step = fov_horiz_in_degrees / (float) number_of_angular_steps;
	for (alpha = -fov_horiz_in_degrees/2.0; alpha < fov_horiz_in_degrees/2.0; alpha += alpha_step, i--)
	{
		float alpha_in_radians = carmen_degrees_to_radians(alpha);
		float tan_alpha_in_radians = tan(alpha_in_radians);
		float cos_alpha_in_radians = cos(alpha_in_radians);

		x = (int)(((float)depthmap_width)/2.0) + (int)floor(focal_length_in_pixels * tan_alpha_in_radians);
		j = y * depthmap_width + x;

		if (j < depthmap_size)
			z = depthmap[j];
		else
		{
			carmen_warn("Laser beams range greater than depthmap size.");
			continue;
		}

		if (z < 0.5)
			z = max_range;

		if (fabs(cos_alpha_in_radians) > 1e-5)
			beams[i] = z / cos_alpha_in_radians;
		else
			beams[i] = z;

	}
	if(i>0)
		carmen_warn("Laser beams does not fulfill entirely FOV: total range %d, not filled range %d.\n", number_of_angular_steps, i);
}

static void
ipc_kinect_depth_handler(carmen_kinect_depth_message *message)
{
	if (is_new_msg(message->timestamp, timestamp_depth))
	{
		message_laser.timestamp = carmen_get_time();

		convert_depthmap_to_beams(message_laser.range, fov_in_degrees, message_laser.num_readings, message_laser.config.maximum_range,
				message->depth, message->width-DEPTH_WIDTH_PADDING, message->height, message->size);

		carmen_laser_publish_laser_message(message_laser.id, &message_laser);

		timestamp_depth = message->timestamp;
	}
}

static void
shutdown_camera_view(int x __attribute__ ((unused)))
{
	carmen_ipc_disconnect();
	printf("Kinect Laser Disconnected from robot.\n");
	exit(1);
}

void
init_message_laser_params(int laser_id, double accuracy, double fov, double resolution, double maxrange)
{
	fov_in_degrees = fov;
	message_laser.id = laser_id;
	message_laser.num_remissions=0;
	message_laser.config.remission_mode = REMISSION_NONE;
	message_laser.config.laser_type = LASER_EMULATED_USING_KINECT;
	message_laser.config.fov = carmen_degrees_to_radians(fov);
    message_laser.config.start_angle = -0.5 * message_laser.config.fov;
    message_laser.config.angular_resolution = carmen_degrees_to_radians(resolution);
	message_laser.num_readings=1 + carmen_round(message_laser.config.fov / message_laser.config.angular_resolution);
    message_laser.config.maximum_range = maxrange;
	message_laser.config.accuracy = accuracy;
	message_laser.host = carmen_get_host();
	message_laser.range = laser_ranges;
}

int
to_kinect_id(int laser_id)
{
	return laser_id-1;
}

int
to_laser_id(int kinect_id)
{
	return kinect_id+1;
}

int
main(int argc, char **argv)
{
	int num_kinect_devices = 1;
	int *flipped=NULL;
	double *resolution=NULL;
	double *fov=NULL;
	double *maxrange=NULL;
	char var_name[256];
	char var_res[256];
	char var_fov[256];
	char var_flipped[256];
	char var_maxrange[256];

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_param_t laser_num_devs[] = {{"kinect", "num_kinect_devices", CARMEN_PARAM_INT, &num_kinect_devices, 0, NULL}};

	carmen_param_install_params(argc, argv, laser_num_devs,	sizeof(laser_num_devs) / sizeof(laser_num_devs[0]));

	if (num_kinect_devices<1)
		carmen_die("You have to specify at least one kinect device to run laser.\nPlease check you ini file for the parameter num_kinect_devices.\n");

	resolution     = calloc( num_kinect_devices, sizeof(double) );
	carmen_test_alloc(resolution);

	fov            = calloc( num_kinect_devices, sizeof(double) );
	carmen_test_alloc(fov);

	flipped        = calloc( num_kinect_devices, sizeof(int) );
	carmen_test_alloc(flipped);

	maxrange       = calloc( num_kinect_devices, sizeof(double) );
	carmen_test_alloc(maxrange);

	for(int kinect_id=0; kinect_id<num_kinect_devices; kinect_id++)
	{
	    sprintf(var_name,"laser%d", to_laser_id(kinect_id));
	    strcpy(var_fov,var_name);
	    strcat(var_fov, "_fov");
	    strcpy(var_res,var_name);
	    strcat(var_res, "_resolution");
	    strcpy(var_maxrange,var_name);
	    strcat(var_maxrange, "_maxrange");
	    strcpy(var_flipped,var_name);
	    strcat(var_flipped, "_flipped");

		carmen_param_t laser_params[] = {
		{"laser", var_fov, CARMEN_PARAM_DOUBLE,              &(fov[kinect_id]),            0, NULL},
		{"laser", var_res, CARMEN_PARAM_DOUBLE,              &(resolution[kinect_id]),     0, NULL},
		{"laser", var_maxrange, CARMEN_PARAM_DOUBLE,         &(maxrange[kinect_id]),       0, NULL},
		{"laser", var_flipped, CARMEN_PARAM_INT,             &(flipped[kinect_id]),        0, NULL}};

		carmen_param_install_params(argc, argv, laser_params, sizeof(laser_params) / sizeof(laser_params[0]));

		init_message_laser_params(to_laser_id(kinect_id), 0.02, fov[kinect_id], resolution[kinect_id], maxrange[kinect_id]);

		carmen_laser_define_laser_message(to_laser_id(kinect_id));

		carmen_kinect_subscribe_depth_message(kinect_id, NULL,
				(carmen_handler_t) ipc_kinect_depth_handler,
				CARMEN_SUBSCRIBE_ALL );
	}

	signal(SIGINT, shutdown_camera_view);

	IPC_dispatch();

	free(fov);
	free(flipped);
	free(maxrange);
	free(resolution);

	return 0;
}
