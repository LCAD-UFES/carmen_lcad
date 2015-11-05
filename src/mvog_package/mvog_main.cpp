#include "mvog_main.h"
#include <btBulletDynamicsCommon.h>

double _map_resolution;
double _init_map_size_x;
double _init_map_size_y;
int _model_negative_space;
int _use_bumblebee_sensor;
int _use_kinect_sensor;
double _bumblebee_sensor_hfov;
double _kinect_sensor_hfov;

int stereo_height;
int stereo_width;
float* _depth;
carmen_vector_3D_t* _image_3d;
stereo_util stereo_instance;
carmen_6d_point _point6d;

static carmen_laser_laser_message laser_message;

carmen_laser_laser_message* laser_messages;
int laserStart;
int laserCount;

carmen_mvog_odometry_and_laser_t* laser_and_odometry;
int laser_and_odometry_size;
int laser_and_odometry_start;
long int laser_and_odometry_count;

MVOGServer* mvog_server;

#define LASER_COUNT 10

void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("mvog_model: disconnected.\n");

    exit(0);
  }
}

void insert_laser_and_odometry_into_list(carmen_laser_laser_message* laser_value, carmen_fused_odometry_message* fused_odometry_value)
{
		if (laser_and_odometry[laser_and_odometry_start].laser_message.num_readings > 0)
		{
			free(laser_and_odometry[laser_and_odometry_start].laser_message.range);
		}

		laser_and_odometry[laser_and_odometry_start].laser_message = (*laser_value);
		laser_and_odometry[laser_and_odometry_start].laser_message.range = (double*) malloc(laser_value->num_readings * sizeof(double));

		for (int i = 0; i < laser_value->num_readings; i++)
			laser_and_odometry[laser_and_odometry_start].laser_message.range[i] = laser_value->range[i];

		laser_and_odometry[laser_and_odometry_start].fused_odometry = (*fused_odometry_value);
}

void scan_kinect_depth_handler(carmen_kinect_depth_message* scan)
{
	btQuaternion orientation(0, 0, 0);
	btVector3 position(0, 0, 1.55);

	btTransform six_degree_pose(orientation, position);

	timeval start_insert, stop_insert;
	gettimeofday(&start_insert, NULL);

	mvog_server->gui_->getDrawer3D()->setRobot(0, 0, 1.55, 0, 0, 0);
	mvog_server->mapper_->addKinectData(scan, six_degree_pose, _kinect_sensor_hfov, 3);

	gettimeofday(&stop_insert, NULL);

	double dur_insert = ( (stop_insert.tv_sec  - start_insert.tv_sec ) * 1000000 +
												(stop_insert.tv_usec - start_insert.tv_usec) ) /1000.0;

	printf("dur_insert: %f ms\n", dur_insert);
}

void scan_bumblebee_stereo_handler(carmen_simple_stereo_disparity_message* scan)
{
	btQuaternion orientation(_point6d.roll, _point6d.pitch, _point6d.yaw);
	btVector3 position(_point6d.x, _point6d.z, _point6d.y);

	btTransform six_degree_pose(orientation, position);

	timeval start_insert, stop_insert;
	gettimeofday(&start_insert, NULL);

	reproject_to_3D(scan->disparity, _image_3d, carmen_degrees_to_radians(8.5), stereo_instance);
	getDepthMap(_image_3d, _depth, stereo_instance);

	mvog_server->gui_->getDrawer3D()->setRobot(_point6d.x, _point6d.z, _point6d.y, _point6d.roll, _point6d.pitch, _point6d.yaw);
	mvog_server->mapper_->addBumblebeeData(_depth, stereo_width, stereo_height,  six_degree_pose, _bumblebee_sensor_hfov, 1);

	gettimeofday(&stop_insert, NULL);

	double dur_insert = ( (stop_insert.tv_sec  - start_insert.tv_sec ) * 1000000 +
												(stop_insert.tv_usec - start_insert.tv_usec) ) /1000.0;

	printf("dur_insert: %f ms\n", dur_insert);
}

static void carmen_fused_odometry_message_handler(carmen_fused_odometry_message* odometry_message)
{
	double min_timestamp = 1000;
	double cur_timestamp;
	int index_min_timestamp = 0;

	if(laserCount >= LASER_COUNT)
	{
		for(int i=0; i < LASER_COUNT; i++)
		{
			cur_timestamp = fabs(odometry_message->timestamp - laser_messages[i].timestamp);
			if(cur_timestamp < min_timestamp)
			{
				min_timestamp = cur_timestamp;
				index_min_timestamp = i;
			}
		}

		insert_laser_and_odometry_into_list(&laser_messages[index_min_timestamp], odometry_message);

		btVector3 position(laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.position.x,
				 	 	 	 	 	 	 	 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.position.y,
											 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.position.z + 1.80);

		btQuaternion orientation(laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.orientation.roll,
				 	 	 	 	 	 	 	 	 	 	 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.orientation.pitch,
				 	 	 	 	 	 	 	 	 	 	 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.orientation.yaw + carmen_degrees_to_radians(90.0));
		btTransform six_degree_pose(orientation, position);

		mvog_server->gui_->getDrawer3D()->setRobot(laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.position.x,
																							 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.position.y,
																							 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.position.z + 1.80,
																							 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.orientation.roll,
																							 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.orientation.pitch,
																							 laser_and_odometry[laser_and_odometry_start].fused_odometry.pose.orientation.yaw + carmen_degrees_to_radians(90.0));


		if(laser_and_odometry_count >= laser_and_odometry_size - 1)
		{
			int pos = abs((laser_and_odometry_start + 1) % (laser_and_odometry_size));

			btVector3 position_remove(laser_and_odometry[pos].fused_odometry.pose.position.x,
					 	 	 	 	 	 	 	 	 	 	 	laser_and_odometry[pos].fused_odometry.pose.position.y,
																laser_and_odometry[pos].fused_odometry.pose.position.z + 1.80);

			btQuaternion orientation_remove(laser_and_odometry[pos].fused_odometry.pose.orientation.roll,
																			laser_and_odometry[pos].fused_odometry.pose.orientation.pitch,
																			laser_and_odometry[pos].fused_odometry.pose.orientation.yaw + carmen_degrees_to_radians(90.0));

			btTransform six_degree_pose_remove(orientation_remove, position_remove);

			mvog_server->mapper_->removeLaserData(&laser_and_odometry[pos].laser_message, six_degree_pose_remove);
		}

		mvog_server->mapper_->addLaserData(&laser_and_odometry[laser_and_odometry_start].laser_message, six_degree_pose);
//		mvog_server->gui_->getDrawer3D()->setRobot(odometry_message->x, odometry_message->y, odometry_message->z + 1.80, odometry_message->roll, odometry_message->pitch, odometry_message->yaw + carmen_degrees_to_radians(90.0));
//		mvog_server->mapper_->addLaserData(&laser_messages[index_min_timestamp], six_degree_pose);

		laser_and_odometry_start++;
		laser_and_odometry_count++;
		if (laser_and_odometry_start >= laser_and_odometry_size)
			laser_and_odometry_start = 0;
	}
}

/*static void carmen_fused_odometry_message_handler(carmen_fused_odometry_message* odometry_message)
{
	double min_timestamp = 1000;
	double cur_timestamp;
	int index_min_timestamp;

	if(laserCount >= LASER_COUNT)
	{
		for(int i=0; i < LASER_COUNT; i++)
		{
			cur_timestamp = fabs(odometry_message->timestamp - laser_messages[i].timestamp);
			if(cur_timestamp < min_timestamp)
			{
				min_timestamp = cur_timestamp;
				index_min_timestamp = i;
			}
		}

		btVector3 position(odometry_message->x,
											 odometry_message->y,
											 odometry_message->z + 1.80);

		btQuaternion orientation(odometry_message->roll,
														 odometry_message->pitch,
														 odometry_message->yaw + carmen_degrees_to_radians(90.0));

		btTransform six_degree_pose(orientation, position);

		mvog_server->gui_->getDrawer3D()->setRobot(odometry_message->x, odometry_message->y, odometry_message->z + 1.80, odometry_message->roll, odometry_message->pitch, odometry_message->yaw + carmen_degrees_to_radians(90.0));
		mvog_server->mapper_->addLaserData(&laser_messages[index_min_timestamp], six_degree_pose);
	}
}*/

static void carmen_laser_laser_message_handler(void)
{
	if(laser_messages[laserStart].num_readings > 0)
	{
		free(laser_messages[laserStart].range);
	}

	laser_messages[laserStart] = laser_message;

	laser_messages[laserStart].range = (double*) malloc(laser_message.num_readings*sizeof(double));

	for(int i=0; i < laser_message.num_readings; i++)
		laser_messages[laserStart].range[i] = laser_message.range[i];

	laserStart++;
	laserCount++;

	if(laserStart >= LASER_COUNT)
	{
		laserStart = 0;
	}
}

int read_parameters(int argc, char **argv)
{
		int num_items;

		carmen_param_t param_list[] = {
			{(char*)"mvog", (char*)"map_resolution", CARMEN_PARAM_DOUBLE, &_map_resolution, 0, NULL},
			{(char*)"mvog", (char*)"init_map_size_x", CARMEN_PARAM_DOUBLE, &_init_map_size_x, 0, NULL},
			{(char*)"mvog", (char*)"init_map_size_y", CARMEN_PARAM_DOUBLE, &_init_map_size_y, 0, NULL},
			{(char*)"mvog", (char*)"model_negative_space", CARMEN_PARAM_ONOFF, &_model_negative_space, 0, NULL},
			{(char*)"mvog", (char*)"use_bumblebee_sensor", CARMEN_PARAM_ONOFF, &_use_bumblebee_sensor, 0, NULL},
			{(char*)"mvog", (char*)"bumblebee_sensor_hfov", CARMEN_PARAM_DOUBLE, &_bumblebee_sensor_hfov, 0, NULL},
			{(char*)"mvog", (char*)"use_kinect_sensor", CARMEN_PARAM_ONOFF, &_use_kinect_sensor, 0, NULL},
			{(char*)"mvog", (char*)"kinect_sensor_hfov", CARMEN_PARAM_DOUBLE, &_kinect_sensor_hfov, 0, NULL},
			{(char*)"stereo", (char*)"width", CARMEN_PARAM_INT, &stereo_width, 0, NULL},
			{(char*)"stereo", (char*)"height", CARMEN_PARAM_INT, &stereo_height, 0, NULL}
			};

		num_items = sizeof(param_list)/sizeof(param_list[0]);
		carmen_param_install_params(argc, argv, param_list, num_items);

		return 0;
}

static gint
updateIPC(gpointer *data __attribute__ ((unused)))
{
  carmen_ipc_sleep(0.01);
  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}

void init_internal_params()
{
	laser_messages = (carmen_laser_laser_message*) malloc(LASER_COUNT * sizeof(carmen_laser_laser_message));
	laserStart = 0;
	laserCount = 0;

	for(int i=0; i < LASER_COUNT ;i++)
		laser_messages[i].num_readings = 0;
}

void alloc_laser_and_odometry_list()
{
		laser_and_odometry_start = 0;
		laser_and_odometry_size = 200;
		laser_and_odometry_count = 0;

		laser_and_odometry = (carmen_mvog_odometry_and_laser_t*) malloc(laser_and_odometry_size * sizeof(carmen_mvog_odometry_and_laser_t));

		int i;
		for (i = 0; i < laser_and_odometry_size; i++)
		{
			laser_and_odometry[i].laser_message.num_readings = 0;
			laser_and_odometry[i].fused_odometry.pose.position.x = 0.0;
			laser_and_odometry[i].fused_odometry.pose.position.y = 0.0;
			laser_and_odometry[i].fused_odometry.pose.position.z = 0.0;
			laser_and_odometry[i].fused_odometry.pose.orientation.pitch = 0.0;
			laser_and_odometry[i].fused_odometry.pose.orientation.roll = 0.0;
			laser_and_odometry[i].fused_odometry.pose.orientation.yaw = 0.0;
		}
}

int
main(int argc, char **argv)
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  read_parameters(argc,argv);

  if (argc != 2)
      carmen_die("%s: Wrong number of parameters. %s requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number>\n", argv[0], argv[0], argc-1, argv[0]);

  int camera = atoi(argv[1]);

  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);

  stereo_instance = get_stereo_instance(camera, stereo_width, stereo_height);
  _depth = (float*) malloc ((stereo_width*stereo_height)*sizeof(float));
  _image_3d = (carmen_vector_3D_t* ) malloc ((stereo_width*stereo_height)*sizeof(carmen_vector_3D_t));
  alloc_laser_and_odometry_list();

  init_internal_params();

  carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t)carmen_fused_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

  if(_use_bumblebee_sensor)
    carmen_stereo_subscribe(1, NULL, (carmen_handler_t) scan_bumblebee_stereo_handler, CARMEN_SUBSCRIBE_LATEST);
  else if(_use_kinect_sensor)
  {
  	carmen_kinect_subscribe_depth_message(0, NULL, (carmen_handler_t)scan_kinect_depth_handler, CARMEN_SUBSCRIBE_LATEST);
  }
  else
  {
  	carmen_laser_subscribe_frontlaser_message(&laser_message, (carmen_handler_t)carmen_laser_laser_message_handler, CARMEN_SUBSCRIBE_LATEST);
  }

  mvog_server = new MVOGServer();
  mvog_server->MVOGInitialize();

  mvog_server->spinOnce();

  //carmen_ipc_dispatch();

  return (0);
}

MVOGServer::MVOGServer ()
{
	this->mvog_map_resolution = _map_resolution;
	this->mvog_init_map_size_x = _init_map_size_x;
	this->mvog_init_map_size_y = _init_map_size_y;
	this->mvog_model_negative_space = _model_negative_space;
	this->mvog_use_bumblebee_sensor = _use_bumblebee_sensor;
	this->mvog_use_kinect_sensor = _use_kinect_sensor;
	this->mvog_bumblebee_sensor_hfov = _bumblebee_sensor_hfov;
	this->mvog_kinect_sensor_hfov = _kinect_sensor_hfov;
}

void MVOGServer::MVOGInitialize()
{
	  mapper_ = new MVOG::Mapper(mvog_map_resolution, mvog_init_map_size_x, mvog_init_map_size_y);
	  mapper_->setModelNegativeSpace(mvog_model_negative_space);
	  // **** create gui
	  gui_ = new MVOG::GTKGui();
	  gui_->setMap(mapper_->getMap());
	  gui_->setUpGTK();
	  gui_->start();
}

MVOGServer::~MVOGServer ()
{
  printf("Final Size: %f\n", mapper_->getMap()->getMemorySize());
}

void MVOGServer::spinOnce()
{
  gui_->spinOnce();
}
