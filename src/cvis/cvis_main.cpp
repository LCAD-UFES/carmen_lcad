#include "cvis_main.h"
#include <btBulletDynamicsCommon.h>

CVISServer* cvis_server;

static int car_pose_count = 0;
static carmen_fused_odometry_message car_poses[100];
static carmen_pose_3D_t velodyne_pose;
carmen_kinect_depth_message* message_efeito3;
//extern "C" void create_point_cloud(carmen_slam6d_pointcloud_message* message, float* pos_v, float* pos_c, float focal_length);

void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
	(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects())->DeleteVertexBufferObjects();
    carmen_ipc_disconnect();
    printf("cvis: disconnected.\n");

    exit(0);
  }
}

int read_parameters(int argc, char **argv)
{
		int num_items;

		carmen_param_t param_list[] = {
				{(char*)"velodyne", (char*)"x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
				{(char*)"velodyne", (char*)"y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
				{(char*)"velodyne", (char*)"z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
				{(char*)"velodyne", (char*)"roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
				{(char*)"velodyne", (char*)"pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
				{(char*)"velodyne", (char*)"yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL}
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

void
carmen_velodyne_handler(carmen_velodyne_partial_scan_message* message)
{
	CVIS::Velodyne* velodyne = (CVIS::Velodyne*)(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects());

	velodyne->PopulatePointCloud(message, car_poses);
	velodyne->UploadPointCloudDataToVBO();
}

void
carmen_fused_odometry_message_handler(carmen_fused_odometry_message* message)
{
	car_poses[car_pose_count].pose = message->pose;
	car_poses[car_pose_count].timestamp = message->timestamp;

	if(car_pose_count == 100)
		car_pose_count = 0;

	car_pose_count++;
}

double fade = 1;
double has_frame = 0;

void
carmen_kinect_handler(carmen_kinect_depth_message* message)
{
	double timestamp = 0;

	CVIS::Kinect* kinect = (CVIS::Kinect* )(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects());
	kinect->SetCloudDensity(cvis_server->gui_->options_.valueCloudDensity);
	cvis_server->gui_->getDrawer3D()->SetPointSize(cvis_server->gui_->options_.valuePointSize);

	if(cvis_server->gui_->options_.autoZoom)
	{
		cvis_server->gui_->getDrawer3D()->zoom(1.0 - cvis_server->gui_->options_.valueVelZoom * CVIS::ZOOM_SPEED);
		cvis_server->gui_->updateControls();
	}


	if(cvis_server->gui_->options_.efeito10enabled)
	{
		kinect->PopulatePointCloud(message, cvis_server->gui_->options_.valueCloudR,
				cvis_server->gui_->options_.valueCloudG, cvis_server->gui_->options_.valueCloudB,
				cvis_server->gui_->options_.valueForegroundRange, cvis_server->gui_->options_.valueBackgroundRange);
	}

	has_frame = 1;
//	else if(cvis_server->gui_->options_.efeito4enabled)
//	{
//		kinect->blowup_count = 0;
//		kinect->PopulatePointCloudBlowUp(message);
//	}


	//kinect->UploadPointCloudDataToVBO();
}

static void
color_handler(void *clientData __attribute__ ((unused)), unsigned long currentTime __attribute__ ((unused)), unsigned long scheduledTime __attribute__ ((unused)))
{
	CVIS::Kinect* kinect = (CVIS::Kinect* )(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects());

	if(cvis_server->gui_->options_.efeito1enabled)
		kinect->EffectPointCloudColor(cvis_server->gui_->options_.valueCor1R,
				cvis_server->gui_->options_.valueCor1G,
				cvis_server->gui_->options_.valueCor1B,
				cvis_server->gui_->options_.valueCor2R,
				cvis_server->gui_->options_.valueCor2G,
				cvis_server->gui_->options_.valueCor2B,
				cvis_server->gui_->options_.valueCor3R,
				cvis_server->gui_->options_.valueCor3G,
				cvis_server->gui_->options_.valueCor3B,
				cvis_server->gui_->options_.changeColorSpeed);

	if(has_frame)
	{
		has_frame = 0;
		kinect->UploadPointCloudDataToVBO();
	}
}

static void
drop_handler(void *clientData __attribute__ ((unused)), unsigned long currentTime __attribute__ ((unused)), unsigned long scheduledTime __attribute__ ((unused)))
{
	CVIS::Kinect* kinect = (CVIS::Kinect* )(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects());

	if(cvis_server->gui_->options_.efeito3enabled)
	{
		cvis_server->gui_->options_.efeito10enabled = 0;
		kinect->EffectPointCloudDrop();

		kinect->UploadPointCloudDataToVBO();
	}
}

static void
blow_handler(void *clientData __attribute__ ((unused)), unsigned long currentTime __attribute__ ((unused)), unsigned long scheduledTime __attribute__ ((unused)))
{
	CVIS::Kinect* kinect = (CVIS::Kinect* )(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects());

	if(cvis_server->gui_->options_.efeito4enabled)
	{
		cvis_server->gui_->options_.efeito10enabled = 0;
		kinect->EffectPointCloudBlowUp();

		kinect->UploadPointCloudDataToVBO();
	}
}

static void
noise_handler(void *clientData __attribute__ ((unused)), unsigned long currentTime __attribute__ ((unused)), unsigned long scheduledTime __attribute__ ((unused)))
{
	CVIS::Kinect* kinect = (CVIS::Kinect* )(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects());

	if(cvis_server->gui_->options_.efeito5enabled)
	{
		kinect->EffectPointCloudNoise();

		kinect->UploadPointCloudDataToVBO();
	}
}

void
carmen_joystick_handler(carmen_joystick_status_message* message)
{
	cvis_server->gui_->getDrawer3D()->setDrawAxesEnabled(message->buttons[5]);


	cvis_server->gui_->joystickMove(message);
}

double last_slam6d_timestamp = 0.0;

/*void
carmen_slam6d_handler(carmen_slam6d_pointcloud_message* message)
{
	if(message->is_keyframe)
	{
		//CVIS::Slam6d* slam6d = (CVIS::Slam6d* )(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects());
		//slam6d->message = message;

		//slam6d->PopulatePointCloud(message);
		//slam6d->UploadPointCloudDataToVBO();
	}

	return;
}*/

int
main(int argc, char **argv)
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  //read_parameters(argc,argv);

  carmen_graphics_update_ipc_callbacks((GdkInputFunction)updateIPC);

  cvis_server = new CVISServer();

  cvis_server->CVISInitializeGUI();

  //if(cvis_server->gui_->drawKinectCloud_)
  carmen_kinect_subscribe_depth_message(0, NULL, (carmen_handler_t) carmen_kinect_handler, CARMEN_SUBSCRIBE_LATEST);
  carmen_joystick_subscribe_status_message(NULL, (carmen_handler_t) carmen_joystick_handler, CARMEN_SUBSCRIBE_LATEST);

  carmen_ipc_addPeriodicTimer(1.0 / 24.0, color_handler, NULL);
  carmen_ipc_addPeriodicTimer(1.0 / 24.0, drop_handler, NULL);
  carmen_ipc_addPeriodicTimer(1.0 / 24.0, blow_handler, NULL);
  carmen_ipc_addPeriodicTimer(1.0 / 12.0, noise_handler, NULL);

//  if(cvis_server->gui_->drawSlam6DCloud_)
//	carmen_slam6d_subscribe_pointcloud_message(NULL, (carmen_handler_t) carmen_slam6d_handler, CARMEN_SUBSCRIBE_LATEST);

//  if(cvis_server->gui_->drawVelodyneCloud_)
//  {
//	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t)carmen_fused_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
//	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) carmen_velodyne_handler, CARMEN_SUBSCRIBE_LATEST);
//
//	CVIS::Velodyne* velodyne = (CVIS::Velodyne*)(cvis_server->gui_->getDrawer3D()->getVertexBufferObjects());
//	velodyne->InitializeVelodyneTransforms(velodyne_pose.position, velodyne_pose.orientation);
//  }

  cvis_server->gui_->start();
  cvis_server->spinOnce();
  return (0);
}

CVISServer::CVISServer ()
{
	gui_ = NULL;
}

void CVISServer::CVISInitializeGUI()
{
	gui_ = new CVIS::GTKGui();
	gui_->setUpGTK();
	gui_->setUpOpenGL(true, false, false);
}

CVISServer::~CVISServer ()
{
}

void CVISServer::spinOnce()
{
  gui_->spinOnce();
}
