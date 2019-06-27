#include "neural_object_detector.hpp"

using namespace std;

carmen_pose_3D_t velodyne_pose;
carmen_velodyne_partial_scan_message *velodyne_msg;
carmen_point_t globalpos;

bool go_msg_absent;


struct fake_pedestrian
{
	bool active;
	double orientation;  // rad
	double velocity;    // m/s
	double start_time; // s
	double stop_time; // s
	double x;        // m - relative to world pos
	double y;       // m - relative to world pos
	pedestrian p;
};

vector<fake_pedestrian> simulated_pedestrians;
vector<pedestrian> pedestrian_tracks;

double
no_out_mean(vector<double> values)// calculates the mean value removing outlayers(values > 1.5*std_dev)
{
	if (values.size() == 0)
		return 0.0;
	double mean = 0;
	for (int i=0; i<values.size(); i++)
		mean += values[i];
	mean /= values.size();
	double std = 0;
	for (int i=0; i<values.size(); i++)
		std += abs(mean-values[i]);
	std /= values.size();
	double sum = 0;
	int elements = 0;
	for (int i=0; i<values.size(); i++)
	{
		if (abs(mean-values[i]) < 1.5*std) //change outlayer treshold by changing this
		{
			sum += values[i];
			elements++;
		}
	}
	if (elements == 0)
		return 0.0;
	return sum/elements;
}

double slope_angle(const vector<double>& x, const vector<double>& y)
{
    double n = x.size();

    double avgX = accumulate(x.begin(), x.end(), 0.0) / n;
    double avgY = accumulate(y.begin(), y.end(), 0.0) / n;

    double numerator = 0.0;
    double denominator = 0.0;

    for(int i=0; i<n; ++i){
        numerator += (x[i] - avgX) * (y[i] - avgY);
        denominator += (x[i] - avgX) * (x[i] - avgX);
    }

    if(denominator == 0){
        return 1.0;
    }

    return atan2(numerator,denominator);
}

void update_world_position(pedestrian* p, double new_x, double new_y, double new_timestamp)
{	
	p->circular_idx = (p->circular_idx+1)%P_BUFF_SIZE;
	p->timestamp[p->circular_idx] = new_timestamp;
	p->x_world[p->circular_idx] = new_x;
	p->y_world[p->circular_idx] = new_y;

	p->velocity = 0.0;
	p->orientation = 0.0;

	vector<double> velx_vect;
	vector<double> vely_vect;
	vector<double> valid_x;
	vector<double> valid_y;
	vector<double> ori;

	int i = 0;
	for(i = 0; i<P_BUFF_SIZE-1; i++)
	{
		int idx = (p->circular_idx+P_BUFF_SIZE-i) % P_BUFF_SIZE;
		int prev_idx = (p->circular_idx+P_BUFF_SIZE-i-1) % P_BUFF_SIZE;
		if (p->x_world[prev_idx] == -999.0 && p->y_world[prev_idx] == -999.0)
			break;
		double delta_x = p->x_world[idx]-p->x_world[prev_idx];
		double delta_y = p->y_world[idx]-p->y_world[prev_idx];
		double delta_t = p->timestamp[idx]-p->timestamp[prev_idx];

		//printf("DELTAS: %f ; %f ; %f --- Vel = %f\n",delta_x, delta_y, delta_t,new_vel);
		velx_vect.push_back(delta_x/delta_t);
		vely_vect.push_back(delta_y/delta_t);
		valid_x.push_back(p->x_world[idx]);
		valid_y.push_back(p->y_world[idx]);
		ori.push_back(atan2(delta_y,delta_x));
	}
	//Calculate Orientation
	double slope = carmen_normalize_theta(slope_angle(valid_x,valid_y));
	double mean_ori = carmen_normalize_theta(no_out_mean(ori));
	if (abs(carmen_normalize_theta(mean_ori-slope)) < abs(carmen_normalize_theta(mean_ori-slope-M_PI)))
		p->orientation = slope;
	else
		p->orientation = carmen_normalize_theta(slope-M_PI);
	//Calculate Velocity
	double velx = no_out_mean(velx_vect);
	double vely = no_out_mean(vely_vect);
	p->velocity = sqrt(velx*velx+vely*vely);
}

void
update_pedestrian_bbox(pedestrian* p,short* bbox_vector)
{
	p->x = bbox_vector[0];
	p->y = bbox_vector[1];
	p->w = bbox_vector[2];
	p->h = bbox_vector[3];

	p->active = true;
}

double
get_pedestrian_x(pedestrian p){
	return p.x_world[p.circular_idx];
}

double
get_pedestrian_y(pedestrian p){
	return p.y_world[p.circular_idx];
}

pedestrian create_pedestrian(int t_id)
{
	pedestrian p;

	p.circular_idx = P_BUFF_SIZE-1;
	p.track_id = t_id;

	for(int i = 0; i<P_BUFF_SIZE; i++)
	{
		p.timestamp[i] = 0;
		p.x_world[i] = -999.0;
		p.y_world[i] = -999.0;
	}
	p.velocity = 0.0;
	p.orientation = 0.0;
	p.active = true;
	p.last_timestamp = 0;
	return p;
}

void
update_pedestrians(short* pedestrian_python, double timestamp)
{
	for (int i=0; i<pedestrian_tracks.size(); i++)
	{
		pedestrian_tracks[i].active=false;
	}

	for (int i=1; i<=pedestrian_python[0]*5; i+=5)
	{
		int p_id = (pedestrian_python+i)[4];
		int j=0;
		for(; j<pedestrian_tracks.size(); j++)
		{
			if(pedestrian_tracks[j].track_id == p_id)
			{
				update_pedestrian_bbox(&pedestrian_tracks[j],pedestrian_python+i);
				pedestrian_tracks[j].last_timestamp = timestamp;
				break;
			}
		}
		if (j == pedestrian_tracks.size())
		{
			pedestrian new_p = create_pedestrian(p_id);
			update_pedestrian_bbox(&new_p,pedestrian_python+i);
			new_p.last_timestamp = timestamp;
			pedestrian_tracks.push_back(new_p);
		}
	}
}

void
clean_pedestrians(double timestamp, double max_time)
{
	for (vector<pedestrian>::iterator it=pedestrian_tracks.begin(); it != pedestrian_tracks.end();)
	{
		if(timestamp - it->last_timestamp > max_time)
		{
			it = pedestrian_tracks.erase(it);
		}
		else{
			it++;
		}
	}
}

void
update_simulated_pedestrians (double timestamp)
{
	if (go_msg_absent)
		return;

	for (int i=0; i<simulated_pedestrians.size(); i++)
	{
		if (!simulated_pedestrians[i].active && timestamp > simulated_pedestrians[i].start_time && timestamp < simulated_pedestrians[i].stop_time)
		{
			simulated_pedestrians[i].active = true;
			pedestrian new_ped = create_pedestrian(-(i+1));
			new_ped.active = true;
			new_ped.last_timestamp = timestamp;
			simulated_pedestrians[i].p = new_ped;
			update_world_position(&simulated_pedestrians[i].p, simulated_pedestrians[i].x,
						simulated_pedestrians[i].y, timestamp);
			//printf("[%03d] - Created\n",simulated_pedestrians[i].p.track_id);
		} 		
		else if (simulated_pedestrians[i].active)
		{
			if (timestamp > simulated_pedestrians[i].stop_time)
			{
				simulated_pedestrians[i].active = false;
				//printf("[%03d] - Deleted\n",simulated_pedestrians[i].p.track_id);
				continue;
			}
			simulated_pedestrians[i].p.active = true;
			double dt = timestamp - simulated_pedestrians[i].p.last_timestamp;
			simulated_pedestrians[i].p.last_timestamp = timestamp;
			double ds = simulated_pedestrians[i].velocity*dt;
			simulated_pedestrians[i].x += cos(simulated_pedestrians[i].orientation)*ds;
			simulated_pedestrians[i].y += sin(simulated_pedestrians[i].orientation)*ds;
			update_world_position(&simulated_pedestrians[i].p, simulated_pedestrians[i].x,
						simulated_pedestrians[i].y, timestamp);
			//printf("[%03d] - Updated\n",simulated_pedestrians[i].p.track_id);
			//Force Velocity
			simulated_pedestrians[i].p.velocity=simulated_pedestrians[i].velocity;
			simulated_pedestrians[i].p.orientation=simulated_pedestrians[i].orientation;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_moving_objects_message(double timestamp, carmen_moving_objects_point_clouds_message *msg)
{
	msg->timestamp = timestamp;
	msg->host = carmen_get_host();

    carmen_moving_objects_point_clouds_publish_message(msg);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


vector<image_cartesian>
compute_detected_objects_poses(vector<vector<image_cartesian>> filtered_points)
{
	vector<image_cartesian> objects_positions;
	unsigned int i, j;

	for(i = 0; i < filtered_points.size(); i++)
	{
		image_cartesian point;

		if (filtered_points[i].size() == 0)
		{
			point.cartesian_x = -999.0;    // This error code is set, probably the object is out of the LiDAR's range
			point.cartesian_y = -999.0;
			point.cartesian_z = -999.0;
			//printf("Empty Bbox\n");
		}
		else
		{
			point.cartesian_x = 0.0;
			point.cartesian_y = 0.0;
			point.cartesian_z = 0.0;

			for(j = 0; j < filtered_points[i].size(); j++)
			{
				point.cartesian_x += filtered_points[i][j].cartesian_x;
				point.cartesian_y += filtered_points[i][j].cartesian_y;
				point.cartesian_z += filtered_points[i][j].cartesian_z;
			}
			point.cartesian_x = point.cartesian_x / j;
			point.cartesian_y = point.cartesian_y / j;
			point.cartesian_z = point.cartesian_z / j;
		}
		objects_positions.push_back(point);
	}
	return (objects_positions);
}


void
carmen_translte_2d(double *x, double *y, double offset_x, double offset_y)
{
	*x += offset_x;
	*y += offset_y;
}


void
carmen_spherical_to_cartesian(double *x, double *y, double *z, double horizontal_angle, double vertical_vangle, double range)
{
	*x = range * cos(vertical_vangle) * cos(horizontal_angle);
	*y = range * cos(vertical_vangle) * sin(horizontal_angle);
	*z = range * sin(vertical_vangle);
}

int
compute_num_measured_objects(vector<pedestrian> objects_poses)
{
	int num_objects = 0;

	for (int i = 0; i < objects_poses.size(); i++)
	{
		if ((get_pedestrian_x(objects_poses[i]) > 0.0 || get_pedestrian_y(objects_poses[i]) > 0.0)&&objects_poses[i].active)
			num_objects++;
	}
	return (num_objects);
}


carmen_moving_objects_point_clouds_message
build_detected_objects_message(vector<pedestrian> predictions, vector<vector<image_cartesian>> points_lists)
{
	carmen_moving_objects_point_clouds_message msg;

	vector<pedestrian> tmp_predictions = predictions;

	for (int i=0; i<simulated_pedestrians.size(); i++)
	{
		if (simulated_pedestrians[i].active)
		{
			tmp_predictions.push_back(simulated_pedestrians[i].p);
			image_cartesian ic;
			vector<image_cartesian> vic;
			vic.push_back(ic);
			points_lists.push_back(vic);
		}
	}
	int num_objects = compute_num_measured_objects(tmp_predictions);

	//printf ("Predictions %d Poses %d, Points %d\n", (int) predictions.size(), (int) objects_poses.size(), (int) points_lists.size());

	msg.num_point_clouds = num_objects;
	msg.point_clouds = (t_point_cloud_struct *) malloc (num_objects * sizeof(t_point_cloud_struct));

	for (int i = 0, l = 0; i < tmp_predictions.size(); i++)
	{                                                                                                               // The error code of -999.0 is set on compute_detected_objects_poses,
		if ((get_pedestrian_x(tmp_predictions[i]) != -999.0 || get_pedestrian_y(tmp_predictions[i]) != -999.0)&&tmp_predictions[i].active)                       // probably the object is out of the LiDAR's range
		{
			msg.point_clouds[l].r = 1.0;
			msg.point_clouds[l].g = 1.0;
			msg.point_clouds[l].b = 0.0;

			msg.point_clouds[l].linear_velocity = tmp_predictions[i].velocity;
			msg.point_clouds[l].orientation = tmp_predictions[i].orientation;

			msg.point_clouds[l].length = 0.2;
			msg.point_clouds[l].height = 0.2;
			msg.point_clouds[l].width  = 0.2;

			msg.point_clouds[l].object_pose.x = get_pedestrian_x(tmp_predictions[i]);
			msg.point_clouds[l].object_pose.y = get_pedestrian_y(tmp_predictions[i]);
			msg.point_clouds[l].object_pose.z = 0.0;


			msg.point_clouds[l].geometric_model = 0;
			msg.point_clouds[l].model_features.geometry.height = 0.2;
			msg.point_clouds[l].model_features.geometry.length = 0.2;
			msg.point_clouds[l].model_features.geometry.width = 0.2;
			msg.point_clouds[l].model_features.red = 1.0;
			msg.point_clouds[l].model_features.green = 1.0;
			msg.point_clouds[l].model_features.blue = 0.8;
			msg.point_clouds[l].model_features.model_name = (char *) "pedestrian";

			msg.point_clouds[l].num_associated = tmp_predictions[i].track_id;

			msg.point_clouds[l].point_size = points_lists[i].size();

			msg.point_clouds[l].points = (carmen_vector_3D_t *) malloc (msg.point_clouds[l].point_size * sizeof(carmen_vector_3D_t));

			for (int j = 0; j < points_lists[i].size(); j++)
			{
				carmen_vector_3D_t p;

				p.x = points_lists[i][j].cartesian_x;
				p.y = points_lists[i][j].cartesian_y;
				p.z = points_lists[i][j].cartesian_z;

				msg.point_clouds[l].points[j] = p;
			}
			l++;
		}
	}
	return (msg);
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	globalpos.theta = globalpos_message->globalpos.theta;
	globalpos.x = globalpos_message->globalpos.x;
	globalpos.y = globalpos_message->globalpos.y;

	vector<vector<image_cartesian>> filtered_points;
	update_simulated_pedestrians(globalpos_message->timestamp);
	carmen_moving_objects_point_clouds_message msg = build_detected_objects_message(pedestrian_tracks, filtered_points);
	publish_moving_objects_message(globalpos_message->timestamp, &msg);
}


static void
navigator_ackerman_go_message_handler()
{
	go_msg_absent = false;
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Neural Object Detector: Disconnected.\n");
        exit(0);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_subscribe_message((char *) CARMEN_NAVIGATOR_ACKERMAN_GO_NAME, (char *) CARMEN_DEFAULT_MESSAGE_FMT, NULL, sizeof(carmen_navigator_ackerman_go_message),
    		(carmen_handler_t)navigator_ackerman_go_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
initialize_simulated_pedestrians()
{
	go_msg_absent = false;

	fake_pedestrian new_p;

//	new_p.start_time = 1515703040.0;
//	new_p.stop_time = 1515703060.0;
//	new_p.x = 7757327.00;
//	new_p.y = -364110.00;
//	new_p.active = false;
//	new_p.velocity = 2.0;
//	new_p.orientation = .0;
//	simulated_pedestrians.push_back(new_p);

//	new_p2.start_time = 1515703020.0;
//	new_p2.stop_time = 1515703060.0;
//	new_p2.x = 7757327.00;
//	new_p2.y = -364110.00;
//	new_p2.active = false;
//	new_p2.velocity = 2.0;
//	new_p2.orientation = 1.5;
//	simulated_pedestrians.push_back(new_p2);

	new_p.start_time = 0.0;                     // Stopped near border
	new_p.stop_time = 9915703060.0;
	new_p.x = 7757888.0;
	new_p.y = -363586.0;
	new_p.active = false;
	new_p.velocity = 0.001;
	new_p.orientation = -0.766;
	simulated_pedestrians.push_back(new_p);

//	new_p.start_time = 0.0;                     // Sideways
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757900.40;
//	new_p.y = -363598.40;
//	new_p.active = false;
//	new_p.velocity = 0.5;
//	new_p.orientation = -0.766;
//	simulated_pedestrians.push_back(new_p);
//
//	new_p.start_time = 0.0;                      // Bottom Left
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757900.11;
//	new_p.y = -363608.86;
//	new_p.active = false;
//	new_p.velocity = 1.0;
//	new_p.orientation = 0.873	;
//	simulated_pedestrians.push_back(new_p);
//
//	new_p.start_time = 0.0;                       // Bottom Right
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757918.28;
//	new_p.y = -363601.10;
//	new_p.active = false;
//	new_p.velocity = 0.7;
//	new_p.orientation = 2.988;
//	simulated_pedestrians.push_back(new_p);
//
//	new_p.start_time = 0.0;                       // Up Right
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757907.18;
//	new_p.y = -363590.69;
//	new_p.active = false;
//	new_p.velocity = 0.4;
//	new_p.orientation = -1.380;
//	simulated_pedestrians.push_back(new_p);
//
//	new_p.start_time = 0.0;                       // Up Left
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757899.01;
//	new_p.y = -363597.10;
//	new_p.active = false;
//	new_p.velocity = 0.7;
//	new_p.orientation = -0.288;
//	simulated_pedestrians.push_back(new_p);

//	new_p.start_time = 0.0;                       // Tangent to the crosswalk
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757890.34;
//	new_p.y = -363607.10;
//	new_p.active = false;
//	new_p.velocity = 2.0;
//	new_p.orientation = 0.166;
//	simulated_pedestrians.push_back(new_p);

//	new_p.start_time = 0.0;                       // Crosswalk direction
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757900.92;
//	new_p.y = -363607.70;
//	new_p.active = false;
//	new_p.velocity = 1.0;
//	new_p.orientation = 0.87;
//	simulated_pedestrians.push_back(new_p);

//	new_p.start_time = 0.0;                       // Car's direction
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757914.89;
//	new_p.y = -363608.30;
//	new_p.active = false;
//	new_p.velocity = 1.0;
//	new_p.orientation = 2.46;
//	simulated_pedestrians.push_back(new_p);

//	new_p.start_time = 0.0;
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757173.00;
//	new_p.y = -364063.20;
//	new_p.active = false;
//	new_p.velocity = 2.01;
//	new_p.orientation = -2.55;
//	simulated_pedestrians.push_back(new_p);

//	new_p.start_time = 0.0;
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757158.40;
//	new_p.y = -364110.60;
//	new_p.active = false;
//	new_p.velocity = 1.01;
//	new_p.orientation = 1.573;
//	simulated_pedestrians.push_back(new_p);
//
//	new_p.start_time = 0.0;
//	new_p.stop_time = 9915703060.0;
//	new_p.x = 7757145.46;
//	new_p.y = -364076.0;
//	new_p.active = false;
//	new_p.velocity = 1.01;
//	new_p.orientation = -0.04;
//	simulated_pedestrians.push_back(new_p);
//
// new_p.start_time = 0.0;
// new_p.stop_time = 9915703060.0;
// new_p.x = 7757158.70;
// new_p.y = -364070.55;
// new_p.active = false;
// new_p.velocity = 0.01;
// new_p.orientation = 2.46;
// simulated_pedestrians.push_back(new_p);
}


int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);

    subscribe_messages();

    signal(SIGINT, shutdown_module);

    initialize_simulated_pedestrians();

	setlocale(LC_ALL, "C");

    carmen_ipc_dispatch();

    return 0;
}
