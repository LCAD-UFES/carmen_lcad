#include "helper.h"
#include <carmen/carmen.h>
#include <carmen/rrt_node.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/collision_detection.h>
#include <list>

//#define MAPPING_MODE

#define MPS_TO_MPH 2.23694 // m/s -> mph
#define MPH_TO_MPS (1.0 / MPS_TO_MPH)
#define CAR_POSE_DISPLACEMENT (-1.15)

#define	BUS			'B' // Width: 2,4 m to 2,6 m; Length: 10 m to 14 m;
#define	CAR			'C' // Width: 1,8 m to 2,1; Length: 3,9 m to 5,3 m
#define	BIKE		'b' // Width: 1,20 m; Length: 2,20 m
#define	PEDESTRIAN	'P'

typedef struct
{
	int c;
	int id;
	double x;
	double y;
	double theta;
	double v;
	double width;
	double length;
} box_model_t;

carmen_base_ackerman_motion_command_message carmen_motion_commands;
rrt_path_message carmen_rrt_path_message;


void
load_map(string map_file_, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s,
		vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy)
{
	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
}


auto
get_simulator_message(size_t length, char* data, uWS::WebSocket<uWS::SERVER>& ws, bool &ok)
{
	// "42" at the start of the message means there's a websocket message event.
	// The 4 signifies a websocket message
	// The 2 signifies a websocket event
	//auto sdata = string(data).substr(0, length);
	//cout << sdata << endl;
	string fake_message = "[\"caco\",{\"x\":909.48,\"y\":1128.67,\"yaw\":0,\"speed\":0,\"s\":124.8336,\"d\":6.164833,\"previous_path_x\":[],\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":0,\"sensor_fusion\":[[0,775.99,1421.6,0,0,6721.839,-277.6729],[1,775.8,1425.2,0,0,6719.219,-280.1494],[2,775.8,1429,0,0,6716.599,-282.9019],[3,775.8,1432.9,0,0,6713.911,-285.7268],[4,775.8,1436.3,0,0,6711.566,-288.1896],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6711.778,-268.0964],[7,762.1,1425.2,0,0,6709.296,-270.7039],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6657.743,-277.6157],[11,762.1,1441.7,0,0,6653.453,-280.8947]]}]";

	ok = true;
	if (!(length && length > 2 && data[0] == '4' && data[1] == '2'))
		ok = false;

	string s = hasData(data);
	if (s == "")
		ok = false;

	if (!ok)
		s = fake_message;

	auto message = json::parse(s);
	string event = message[0].get<string>();
	if (event != "telemetry")
	{
		// Manual driving
		std::string msg = "42[\"manual\",{}]";
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		ok = false;
	}
	return (message);
}


void
get_car_state(double &car_x, double &car_y,	double &car_s, double &car_d, double &car_yaw_deg, double &car_speed_mph, auto simulator_message)
{
	car_x = simulator_message[1]["x"];
	car_y = simulator_message[1]["y"];
	car_s = simulator_message[1]["s"];
	car_d = simulator_message[1]["d"];
	car_yaw_deg = simulator_message[1]["yaw"];
	car_speed_mph = simulator_message[1]["speed"];
//	double steering_angle = simulator_message[1]["steering_angle"];
//	printf("steering angle %lf\n", steering_angle);
}


void
send_trajectory_to_simulator(const vector<double> trajectory_x, const vector<double> trajectory_y, uWS::WebSocket<uWS::SERVER> &ws)
{
	json msgJson;
	msgJson["next_x"] = trajectory_x;
	msgJson["next_y"] = trajectory_y;
	auto msg = "42[\"control\"," + msgJson.dump() + "]";
	//this_thread::sleep_for(chrono::milliseconds(1000));
	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


void
publish_globalpos(double x, double y, double theta, double v, double phi, double timestamp)
{
	carmen_localize_ackerman_globalpos_message globalpos;

	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();
	globalpos.globalpos.x = x;
	globalpos.globalpos.y = y;
	globalpos.globalpos.theta = theta;
	globalpos.globalpos_std.x = 0.2;
	globalpos.globalpos_std.y = 0.2;
	globalpos.globalpos_std.theta = 0.2;
	globalpos.odometrypos = globalpos.globalpos;
	globalpos.globalpos_xy_cov = 0.0;
	globalpos.v = v;
	globalpos.phi = phi;
	globalpos.converged = 1;

	globalpos.pose.orientation.pitch = globalpos.pose.orientation.roll = 0.0;
	globalpos.pose.orientation.yaw = globalpos.globalpos.theta;

	globalpos.velocity.y = globalpos.velocity.z = 0.0;
	globalpos.velocity.x = v;

	globalpos.pose.position.x = globalpos.globalpos.x;
	globalpos.pose.position.y = globalpos.globalpos.y;
	globalpos.pose.position.z = 0;

	carmen_localize_ackerman_publish_globalpos_message(&globalpos);
}


void
publish_odometry(double x, double y, double theta, double v, double phi, double timestamp)
{
	carmen_base_ackerman_odometry_message msg;

	msg.x = x;
	msg.y = y;
	msg.theta = theta;
	msg.v = v;
	msg.phi = phi;
	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &msg);
	carmen_test_ipc(err, "Could not publish base_odometry_message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
}


void
publish_velodyne_partial_scan(double timestamp)
{
	carmen_velodyne_partial_scan_message velodyne_partial_scan;

	velodyne_partial_scan.number_of_32_laser_shots = 720;
	carmen_velodyne_32_laser_shot partial_scan[720];
	for (int i = 0; i < 720; i++)
	{
		partial_scan[i].angle = ((double) i / 2.0) - 180.0;
		for (int j = 0; j < 32; j++)
		{
			partial_scan[i].distance[j] = 21 * 500;
			partial_scan[i].intensity[j] = 128;
		}
	}
	velodyne_partial_scan.partial_scan = partial_scan;
	velodyne_partial_scan.timestamp = timestamp;
	velodyne_partial_scan.host = carmen_get_host();

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, &velodyne_partial_scan);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT);
}


void
publish_xsens(double timestamp)
{
	carmen_xsens_global_quat_message message = {-0.076347, 0.022741, 9.856972,
												 0.150156, -0.005627, -0.002592,
												 0.988643, -0.278449, -0.101014,
												 0.281111, 0.022601, 0.002073, 0.006112,
												 60.500000, 0,
												 timestamp,
												 carmen_get_host()};
	publish_mti_quat_message(message);
}


bool
get_yaw_and_phi(double timestamp, double &yaw, double &phi)
{
	if (carmen_rrt_path_message.timestamp == 0.0)
		return (false);

	double smallest_delta_t = fabs(timestamp - carmen_rrt_path_message.timestamp);
	int nearest = 0;
	int i = 0;
	double t = carmen_rrt_path_message.path[i].time;
	for (i = 1; (i < carmen_rrt_path_message.size) && (i < 40); i++)
	{
		double delta_t = fabs(timestamp - (carmen_rrt_path_message.timestamp + t));
		if (delta_t < smallest_delta_t)
		{
			nearest = i;
			smallest_delta_t = delta_t;
		}
		t += carmen_rrt_path_message.path[i].time;
	}

	printf("**** i = %d, smallest_delta_t %lf\n", nearest, smallest_delta_t);

	yaw = carmen_rrt_path_message.path[nearest].p1.theta;
	phi = carmen_rrt_path_message.path[nearest].p1.phi;

	return (true);
}


void
fill_in_moving_objects_message_element(int k, carmen_moving_objects_point_clouds_message *message, box_model_t *box)
{
	message->point_clouds[k].r = 0.0;
	message->point_clouds[k].g = 0.0;
	message->point_clouds[k].b = 1.0;
	message->point_clouds[k].linear_velocity = box->v;
	message->point_clouds[k].orientation = box->theta;
	message->point_clouds[k].object_pose.x = box->x;
	message->point_clouds[k].object_pose.y = box->y;
	message->point_clouds[k].object_pose.z = 0.0;
	message->point_clouds[k].height = box->width;
	message->point_clouds[k].length = box->length;
	message->point_clouds[k].width = box->width;
	message->point_clouds[k].geometric_model = box->c;
	message->point_clouds[k].point_size = 0;
	message->point_clouds[k].points = NULL;
	message->point_clouds[k].num_associated = box->id;
	object_model_features_t &model_features = message->point_clouds[k].model_features;
	model_features.model_id = box->c;

	switch (box->c)
	{
		case BUS:
			model_features.model_name = (char *) ("Bus");
			model_features.red = 1.0;
			model_features.green = 0.0;
			model_features.blue = 0.0;
			break;
		case CAR:
			model_features.model_name = (char *) ("Car");
			model_features.red = 0.5;
			model_features.green = 1.0;
			model_features.blue = 0.0;
			break;
		case BIKE:
			model_features.model_name = (char *) ("Bike");
			model_features.red = 0.5;
			model_features.green = 0.5;
			model_features.blue = 0.5;
			break;
		case PEDESTRIAN:
			model_features.model_name = (char *) ("Pedestrian");
			model_features.red = 0.0;
			model_features.green = 1.0;
			model_features.blue = 1.0;
			break;
	}
	model_features.geometry.length = box->length;
	model_features.geometry.width = box->width;
	model_features.geometry.height = box->width;
}


carmen_moving_objects_point_clouds_message *
fill_in_moving_objects_message(auto sensor_fusion)
{
	carmen_moving_objects_point_clouds_message *message = (carmen_moving_objects_point_clouds_message *) malloc(sizeof(carmen_moving_objects_point_clouds_message));
	message->host = carmen_get_host();
	message->timestamp = carmen_get_time();

	int num_moving_objects = sensor_fusion.size();

	message->point_clouds = (t_point_cloud_struct *) malloc(sizeof(t_point_cloud_struct) * num_moving_objects);
	message->num_point_clouds = num_moving_objects;

	for (int i = 0; i < num_moving_objects; i++)
	{
		box_model_t box;
		box.c = 'C'; 		// car
		box.length = 4.0;
		box.width = 2.0;
		box.id = sensor_fusion[i][0];
		box.x = sensor_fusion[i][1];
		box.y = sensor_fusion[i][2];
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
//		vehicle.s = sensor_fusion[i][5];
//		vehicle.d = sensor_fusion[i][6];
		box.v = sqrt(vx * vx + vy * vy);
		box.theta = atan2(vy, vx);

		fill_in_moving_objects_message_element(i, message, &box);
	}

	return (message);
}


void
free_moving_objects(carmen_moving_objects_point_clouds_message *moving_objects)
{
	if (moving_objects != NULL)
	{
		free(moving_objects->point_clouds);
		free(moving_objects);
	}
}


void
publish_moving_objects(auto simulator_message)
{
	auto sensor_fusion = simulator_message[1]["sensor_fusion"];
	carmen_moving_objects_point_clouds_message *moving_objects = fill_in_moving_objects_message(sensor_fusion);
	carmen_moving_objects_point_clouds_publish_message(moving_objects);
	free_moving_objects(moving_objects);
}


void
publish_carmen_messages(double car_x, double car_y, double car_yaw, double v, auto simulator_message)
{
#define L 2.64

	static double previous_yaw;
	static double previous_timestamp = 0.0;
	static double phi;
	static double previous_car_x;
	static double previous_car_y;
	double yaw;

	if (previous_timestamp == 0.0)
	{
		previous_timestamp = carmen_get_time();
		previous_yaw = car_yaw;
		phi = 0.0;

		previous_car_x = car_x;
		previous_car_y = car_y;

		carmen_point_t mean = {car_x, car_y, car_yaw};
		carmen_point_t std = {1.0, 1.0, 1.0};
		carmen_localize_ackerman_initialize_gaussian_command(mean, std);

		return;
	}

	double timestamp = carmen_get_time();

	if (!get_yaw_and_phi(timestamp, yaw, phi))
	{
		yaw = atan2(car_y - previous_car_y, car_x - previous_car_x);
		double d_yaw = carmen_normalize_theta(yaw - previous_yaw) / (timestamp - previous_timestamp);
		phi = atan((L * d_yaw) / v);
//		phi += 0.2 * (atan((L * d_yaw) / v) - phi);
	}

	carmen_ackerman_traj_point_t car_pose = {car_x, car_y, car_yaw, v, phi};
	carmen_point_t new_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&car_pose, CAR_POSE_DISPLACEMENT); // o simulador informa a pose do carro como sendo o meio do carro
	car_x = new_car_pose.x;
	car_y = new_car_pose.y;

	publish_globalpos(car_x, car_y, yaw, v, phi, timestamp);
	publish_odometry(car_x, car_y, yaw, v, phi, timestamp);
	publish_velodyne_partial_scan(timestamp);
	publish_xsens(timestamp);
#ifndef MAPPING_MODE
	publish_moving_objects(simulator_message);
#endif
	previous_car_x = car_x;
	previous_car_y = car_y;
	previous_yaw = yaw;
	previous_timestamp = timestamp;
}


void
print_trajectories(vector<double> trajectory_x, vector<double> trajectory_y, vector<double> trajectory_x2, vector<double> trajectory_y2)
{
	FILE *graph = fopen("graph.txt", "a");
	for (int i = 0; (i < trajectory_x.size()) && (i < 10); i++)
		fprintf(graph, "previous %lf %lf, current %lf %lf, plan %lf %lf\n", trajectory_x[i], trajectory_y[i], trajectory_x2[i], trajectory_y2[i],
				carmen_rrt_path_message.path[i].p1.x, carmen_rrt_path_message.path[i].p1.y);

	fprintf(graph, "******\n");
	fclose(graph);
}


void
plot_trajectories(vector<double> trajectory_x1, vector<double> trajectory_y1, vector<double> trajectory_x2, vector<double> trajectory_y2,
		char *title1, char *title2)
{
	#define PAST_SIZE 300
	static list<double> cvel;
	static list<double> dvel;
	static list<double> timestamp;
	static bool first_time = true;
	static double first_timestamp;
	static FILE *gnuplot_pipe;
	list<double>::iterator itc;
	list<double>::iterator itd;
	list<double>::iterator itt;

	double t = carmen_get_time();
	if (first_time)
	{
		first_timestamp = t;
		first_time = false;
		gnuplot_pipe = popen("gnuplot", "w"); //("gnuplot -persist", "w") to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [-3:3]\n set yrange [-3:3]\n");
	}

	FILE *gnuplot_data_file = fopen("trajectories.txt", "w");

	for (int i = 0; (i < trajectory_x1.size()) && (i < trajectory_x2.size()); i++)
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf\n", trajectory_x1[i] - trajectory_x1[0], trajectory_y1[i] - trajectory_y1[0],
				trajectory_x2[i] - trajectory_x1[0], trajectory_y2[i] - trajectory_y1[0]);

	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "plot "
			"'./trajectories.txt' using 1:2 with lines title '%s', './trajectories.txt' using 3:4 with lines title '%s',"
			"'./trajectories.txt' using 1:2 title '%s', './trajectories.txt' using 3:4 title '%s'\n", title1, title2, title1, title2);

	fflush(gnuplot_pipe);
}


double
path_to_previous_path_point_distance(double x1, double y1, double x2, double y2)
{
	return (sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
}


int
find_nearest_point_in_previous_path(vector<double> previous_car_x, vector<double> previous_car_y)
{
	int nearest = 0;
	double smallest_distance = path_to_previous_path_point_distance(carmen_rrt_path_message.path[0].p1.x, carmen_rrt_path_message.path[0].p1.y,
			previous_car_x[0], previous_car_y[0]);
	for (int i = 1; i < carmen_rrt_path_message.size; i++)
	{
		double distance = path_to_previous_path_point_distance(carmen_rrt_path_message.path[i].p1.x, carmen_rrt_path_message.path[i].p1.y,
				previous_car_x[0], previous_car_y[0]);
		if (distance < smallest_distance)
		{
			smallest_distance = distance;
			nearest = i;
		}
	}

	return (nearest);
}


void
build_carmen_trajectory(vector<double> previous_path_x, vector<double> previous_path_y, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
	int best_i = 0;
	carmen_point_t new_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&(carmen_rrt_path_message.path[0].p1), -CAR_POSE_DISPLACEMENT); // o simulador informa a pose do carro como sendo o meio do carro
	double smallest_distance = sqrt((new_car_pose.x - previous_path_x[0]) * (new_car_pose.x - previous_path_x[0]) +
						   (new_car_pose.y - previous_path_y[0]) * (new_car_pose.y - previous_path_y[0]));

	for (int i = 1; (i < carmen_rrt_path_message.size)  && (i < 20); i++)
	{
		new_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&(carmen_rrt_path_message.path[i].p1), -CAR_POSE_DISPLACEMENT); // o simulador informa a pose do carro como sendo o meio do carro
		double distance = sqrt((new_car_pose.x - previous_path_x[0]) * (new_car_pose.x - previous_path_x[0]) +
							   (new_car_pose.y - previous_path_y[0]) * (new_car_pose.y - previous_path_y[0]));

		if (distance < smallest_distance)
		{
			best_i = i;
			smallest_distance = distance;
		}
	}

	printf("best_i  = %d, smallest_distance %lf\n", best_i, smallest_distance);
	printf("best_i  = %d, smallest_distance %lf\n", best_i, smallest_distance);

	for (int i = best_i; i < carmen_rrt_path_message.size; i++)
	{
		carmen_point_t new_car_pose = carmen_collision_detection_displace_car_pose_according_to_car_orientation(&(carmen_rrt_path_message.path[i].p1), -CAR_POSE_DISPLACEMENT); // o simulador informa a pose do carro como sendo o meio do carro
		next_x_vals.push_back(new_car_pose.x);
		next_y_vals.push_back(new_car_pose.y);
	}
}


void
simulator_message_handler(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s,
		vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy,
		int &lane, int &lane_change_wp,
		uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
{
	bool ok;

	auto simulator_message = get_simulator_message(length, data, ws, ok);
	if (!ok)
		return;

	double car_x, car_y, car_s, car_d, car_yaw_deg, car_speed_mph;
	get_car_state(car_x, car_y,	car_s, car_d, car_yaw_deg, car_speed_mph, simulator_message);

	// Reference velocity of 49.5 mph(below the 50mph speed limit)
//	double ref_vel = 49.5 * (60.0 / 80.0);
	double ref_vel = 8.333 * MPS_TO_MPH;

	vector<vector<double>> sensor_fusion = simulator_message[1]["sensor_fusion"];
	vector<double> previous_path_x = simulator_message[1]["previous_path_x"];
	vector<double> previous_path_y = simulator_message[1]["previous_path_y"];

	int previous_path_size = previous_path_x.size();
	int next_wp = -1;
	double ref_x, ref_y, ref_yaw;
	static double start_time = 0.0;
	if (start_time == 0.0)
		start_time = carmen_get_time();

	if (previous_path_size < 2)
	{
		// NOTE: This is the first time the path planner is going to run
		ref_x = car_x;
		ref_y = car_y;
		ref_yaw = deg2rad(car_yaw_deg);

		// Get the next way point using the ego car's (x, y, yaw)
		next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
	}
	else
	{
		ref_x = previous_path_x[previous_path_size - 1];
		ref_y = previous_path_y[previous_path_size - 1];

		double ref_x_prev = previous_path_x[previous_path_size - 2];
		double ref_y_prev = previous_path_y[previous_path_size - 2];
		ref_yaw = atan2((ref_y - ref_y_prev), (ref_x - ref_x_prev));

		car_speed_mph = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / .02) * MPS_TO_MPH;

		car_s = simulator_message[1]["end_path_s"];

		next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

		publish_carmen_messages(previous_path_x[0], previous_path_y[0], deg2rad(car_yaw_deg), car_speed_mph * MPH_TO_MPS, simulator_message);
	}

	// Call function to check if lane change is possible and required
	bool change_lanes = LaneChangePossible(sensor_fusion, previous_path_size, lane, car_s, ref_vel);
	change_lanes = false;

	// Based on the answer returned from the previous step change lanes
	ChangeLane(sensor_fusion, map_waypoints_x, previous_path_size, change_lanes, lane, car_s, next_wp, lane_change_wp);

	vector<double> waypoints_x, waypoints_y;
	// Generate the way points for creating the spline below
	GenerateWayPoints(waypoints_x, waypoints_y, previous_path_size, ref_x, car_x, ref_y, car_y, ref_yaw, car_yaw_deg, car_s, lane, previous_path_x, previous_path_y, map_waypoints_x,
			map_waypoints_y, map_waypoints_s);

	carmen_ipc_sleep(0.01);

	// Vector of co-ordinates for the car to follow
	vector<double> next_x_vals, next_y_vals;
	// Call function to create a trajectory for the car to follow
	static int size = 0;
	static double previous_t = 0.0;
#ifndef MAPPING_MODE
	if (carmen_get_time() - start_time < 5.0)
#else
	if (1)
#endif
	{
		CreateSpline(waypoints_x, waypoints_y, next_x_vals, next_y_vals, previous_path_x, previous_path_y, ref_x, car_x, ref_y, car_y, ref_yaw, car_yaw_deg, car_speed_mph, ref_vel);
		size = next_x_vals.size();
		printf("previous_path_size %d, size %d\n", previous_path_size, size);
		send_trajectory_to_simulator(next_x_vals, next_y_vals, ws);
	}
	else
	{
//		if (previous_path_size >= 2)
//			printf("*previous_path_size %d, pt %lf, t %lf\n", previous_path_size, previous_t, carmen_get_time());
//		else
//			printf("***** previous_path_size %d, pt %lf, t %lf\n", previous_path_size, previous_t, carmen_get_time());

		double t = carmen_get_time();
		if ((t - previous_t) >= 0.1)
		{
			previous_t = t;
			build_carmen_trajectory(previous_path_x, previous_path_y, next_x_vals, next_y_vals);
//			print_trajectories(previous_path_x, previous_path_y, next_x_vals, next_y_vals);
			send_trajectory_to_simulator(next_x_vals, next_y_vals, ws);
		}
		else
			send_trajectory_to_simulator(previous_path_x, previous_path_y, ws);
	}
}


void
define_ipc_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);

	err = IPC_defineMsg(CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_PARTIAL_SCAN_MESSAGE_NAME);

    err = IPC_defineMsg(CARMEN_XSENS_GLOBAL_QUAT_NAME, IPC_VARIABLE_LENGTH, CARMEN_XSENS_GLOBAL_QUAT_FMT);
    carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_GLOBAL_QUAT_NAME);

    carmen_moving_objects_point_clouds_define_messages();
}


static void
base_ackerman_motion_command_message_handler(carmen_base_ackerman_motion_command_message *motion_command_message)
{
	carmen_motion_commands = *motion_command_message;
}


void
rrt_path_message_handler(rrt_path_message *msg)
{
	carmen_rrt_path_message = *msg;
}


void
subscribe_to_ipc_messages()
{
	carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) base_ackerman_motion_command_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_subscribe_message((char *) RRT_PATH_NAME, (char *) RRT_PATH_FMT, NULL, sizeof(rrt_path_message),
			(carmen_handler_t) rrt_path_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	// Problem this code solves and a solution: https://github.com/tommytracey/Udacity-CarND-Term3/blob/master/p11-path-planning/README.md
	// Video about the problem: https://www.youtube.com/watch?v=7sI3VHFPP0w&feature=youtu.be
	// This code is based on: https://github.com/spgitmonish/PathPlanning

	uWS::Hub h;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	define_ipc_messages();
	subscribe_to_ipc_messages();
	memset(&carmen_motion_commands, 0, sizeof(carmen_base_ackerman_motion_command_message));
	memset(&carmen_rrt_path_message, 0, sizeof(rrt_path_message));

	string map_file_ = "../data/highway_map.csv";
	vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy;
	load_map(map_file_, map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

	// Start in lane 1(Middle lane)
	int lane = 1;
	int lane_change_wp = 0;

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&lane_change_wp]
				 (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
	{
		simulator_message_handler(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy,
				lane, lane_change_wp,
				ws, data, length, opCode);
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req,
			char *data, size_t, size_t)
	{
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
	{
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length)
	{
		static int repetitions = 0;

		repetitions++;
		std::cout << "***** Trying to disconnected *****" << std::endl;
		if (repetitions > 4)
			exit(1);

		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	// Listen to port 4567 for communication
	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	// Run server
	h.run();
}
