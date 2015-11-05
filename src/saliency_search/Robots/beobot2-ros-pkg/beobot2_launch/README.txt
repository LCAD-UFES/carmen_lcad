
==== Main Launch file====

stereo_view.launch
		stereo_cameras.launch
  	image_sync.launch
  	stereo_image_proc.launch
		stereo_view_base.launch


stereo_calibration.launch
		stereo_cameras.launch
  	image_sync.launch
  	stereo_image_proc.launch
		stereo_calibration_base.launch   : Change check board size, default is 6x4 0.035m size grid

==== Base Launch file====

//////////Camera////////
stereo_cameras.launch
	camera3.launch   				: Left Camera
	camera4.launch					: Right Camera
image_sync.launch					: Create sync stereo camera data
stereo_image_proc.launch	: Create usable stereo camera info


//////////Navigation////////
beobot2_core.launch				: Beobot2.0 Navigation Core
	beopilot.launch					: Motor Controller
	beobot_teleop.launch		: Odometry Data
	hokyuo.launch						: Laser Range Finder

navigation.launch					: ROS Navigation Stack
	map_server.launch				: ROS Map server
	amcl.launch							: Setup all beobot2 geometry parameter
	move_base								: ROS move base stack

gmapping.launch						: ROS gmapping SLAM
