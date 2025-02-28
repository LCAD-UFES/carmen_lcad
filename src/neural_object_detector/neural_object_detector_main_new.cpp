#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/camera_boxes_to_world.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <string>
#include <cstdlib>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <dbscan.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <numpy/arrayobject.h>
#include "Darknet.hpp"

using namespace std;
using namespace cv;

int camera;
int camera_side;

carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t camera_pose;

Detector *darknet;
vector<string> obj_names_vector;
vector<Scalar> obj_colours_vector;

carmen_velodyne_partial_scan_message *velodyne_msg;
carmen_point_t globalpos;

#define P_BUFF_SIZE 10

struct pedestrian
{
	int track_id;
	double velocity;
	double orientation;
	unsigned int x, y, w, h;
	double last_timestamp;
	bool active;
	double timestamp[P_BUFF_SIZE];
	double x_world[P_BUFF_SIZE];
	double y_world[P_BUFF_SIZE];
	unsigned int circular_idx;// should be changed only for update_world_position function
};

struct fake_pedestrian
{
	bool active;
	double orientation; // rad
	double velocity;  // m/s
	double start_time; // s
	double stop_time; // s
	double x;   // m - relative to world pos
	double y;   // m - relative to world pos
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

double slope_angle(vector<double> x, vector<double> y)
{
	if (x.size() < 2)
		return 0.0;

    double n = x.size();

    double sum_x = 0.0;
    for (int i = 0; i < n; i++)
        sum_x += x[i];
    double sum_y = 0.0;
    for (int i = 0; i < n; i++)
        sum_y += y[i];
    double avgX = sum_x / n;
    double avgY = sum_y / n;

    double numerator = 0.0;
    double denominator = 0.0;

    for (int i=0; i<n; ++i)
    {
        numerator += (x[i] - avgX) * (y[i] - avgY);
        denominator += (x[i] - avgX) * (x[i] - avgX);
    }

    if (denominator == 0)
        return 0.0;

    if (x[0] > x[x.size()-1])
    	return (numerator / denominator)+3.14; // 3.14 eh PI? Por que nao usar M_PI?
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

	double slope = carmen_normalize_theta(slope_angle(valid_x,valid_y));
	double mean_ori = carmen_normalize_theta(no_out_mean(ori));
	if (abs(carmen_normalize_theta(mean_ori-slope)) < abs(carmen_normalize_theta(mean_ori-slope-M_PI)))
		p->orientation = slope;
	else
		p->orientation = carmen_normalize_theta(slope-M_PI);
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
		}
	}
}


void
objects_names_from_file(string const class_names_file)
{
    ifstream file(class_names_file);

    if (!file.is_open())
    	return;

    for (string line; getline(file, line);)
    	obj_names_vector.push_back(line);

    cout << "Object names loaded!\n\n";
}


void
set_object_vector_color()
{
	for (unsigned int i = 0; i < obj_names_vector.size(); i++)
	{
		if (i == 2 ||                                         // car
			i == 3 ||                                         // motorbike
			i == 5 ||                                         // bus
			i == 6 ||                                         // train
			i == 7)                                           // truck
			obj_colours_vector.push_back(Scalar(0, 0, 255));
		else if (i == 0 || i == 1)                            // person or bicycle
			obj_colours_vector.push_back(Scalar(255, 0, 0));
		else
			obj_colours_vector.push_back(Scalar(230, 230, 230));
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

vector<vector<image_cartesian>>
get_points_inside_bounding_boxes(vector<pedestrian> &predictions, vector<image_cartesian> &velodyne_points_vector)
{
	vector<vector<image_cartesian>> laser_list_inside_each_bounding_box; //each_bounding_box_laser_list

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		vector<image_cartesian> lasers_points_inside_bounding_box;
		laser_list_inside_each_bounding_box.push_back(lasers_points_inside_bounding_box);
	}

	for (unsigned int j = 0; j < velodyne_points_vector.size(); j++)
	{
		double min_dist = 0.0;
		int min_dist_index = -1;
		for (unsigned int i = 0; i < predictions.size(); i++)
		{
			if (predictions[i].active && 	(velodyne_points_vector[j].image_x >  predictions[i].x &&
											velodyne_points_vector[j].image_x < (predictions[i].x + predictions[i].w) &&
											velodyne_points_vector[j].image_y >  predictions[i].y &&
											velodyne_points_vector[j].image_y < (predictions[i].y + predictions[i].h)))
			{
				double delta_x =  get_pedestrian_x(predictions[i]);
				double delta_y =  get_pedestrian_y(predictions[i]);
				double actual_dist = sqrt(delta_x*delta_x+delta_y*delta_y);
				if ( actual_dist < min_dist || min_dist_index==-1)
				{
					min_dist = actual_dist;
					min_dist_index = i;
				}
				//laser_list_inside_each_bounding_box[i].push_back(velodyne_points_vector[j]);
			}
		}
		if (min_dist_index > -1)
		{
			laser_list_inside_each_bounding_box[min_dist_index].push_back(velodyne_points_vector[j]);
		}
	}
	return laser_list_inside_each_bounding_box;
}


vector<image_cartesian>
get_biggest_cluster(vector<vector<image_cartesian>> &clusters)
{
	unsigned int max_size = 0, max_index = 0;

	for (unsigned int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].size() > max_size)
		{
			max_size = clusters[i].size();
			max_index = i;
		}
	}
	return (clusters[max_index]);
}


inline double
distance2(image_cartesian a, image_cartesian b)
{
	double dx = a.cartesian_x - b.cartesian_x;
	double dy = a.cartesian_y - b.cartesian_y;

	return (dx * dx + dy * dy);
}


vector<int>
query(double d2, int i, const vector<image_cartesian> &points, std::vector<bool> clustered)
{
	vector<int> neighbors;
	const image_cartesian &point = points[i];

	for (size_t j = 0; j < points.size(); j++)
	{
		if ((distance2(point, points[j]) < d2) && !clustered[j])
			neighbors.push_back(j);
	}

	return neighbors;
}


vector<vector<image_cartesian>>
dbscan_compute_clusters(double d2, size_t density, const vector<image_cartesian> &points)
{
	vector<vector<image_cartesian>> clusters;
	vector<bool> clustered(points.size(), false);

	for (size_t i = 0; i < points.size(); ++i)
	{
		// Ignore already clustered points.
		if (clustered[i])
			continue;

		// Ignore points without enough neighbors.
		vector<int> neighbors = query(d2, i, points, clustered);
		if (neighbors.size() < density)
			continue;

		// Create a new cluster with the i-th point as its first element.
		vector<image_cartesian> c;
		clusters.push_back(c);
		vector<image_cartesian> &cluster = clusters.back();
		cluster.push_back(points[i]);
		clustered[i] = true;

		// Add the point's neighbors (and possibly their neighbors) to the cluster.
		for (size_t j = 0; j < neighbors.size(); ++j)
		{
			int k = neighbors[j];
			if (clustered[k])
				continue;

			cluster.push_back(points[k]);
			clustered[k] = true;

			vector<int> farther = query(d2, k, points, clustered);
			if (farther.size() >= density)
				neighbors.insert(neighbors.end(), farther.begin(), farther.end());
		}
	}
	return (clusters);
}


vector<vector<image_cartesian>>
filter_object_points_using_dbscan(vector<vector<image_cartesian>> &points_lists)
{
	vector<vector<image_cartesian>> filtered_points;

	for (unsigned int i = 0; i < points_lists.size(); i++)
	{
		vector<vector<image_cartesian>> clusters = dbscan_compute_clusters(0.5, 3, points_lists[i]);        // Compute clusters using dbscan

		if (clusters.size() == 0)                                          // If dbscan returned zero clusters
		{
			vector<image_cartesian> empty_cluster;
			filtered_points.push_back(empty_cluster);                      // An empty cluster is added to the clusters vector
		}
		else if (clusters.size() == 1)
		{
			filtered_points.push_back(clusters[0]);
		}
		else                                                               // dbscan returned more than one cluster
		{
			filtered_points.push_back(get_biggest_cluster(clusters));      // get the biggest, the biggest cluster will always better represent the object
		}
	}
	return (filtered_points);
}


void
show_LIDAR_points(Mat &image, vector<image_cartesian> all_points)
{
	for (unsigned int i = 0; i < all_points.size(); i++)
		circle(image, Point(all_points[i].image_x, all_points[i].image_y), 1, cvScalar(0, 0, 255), 1, 8, 0);
}


void
show_LIDAR(Mat &image, vector<vector<image_cartesian>> points_lists, int r, int g, int b)
{
	for (unsigned int i = 0; i < points_lists.size(); i++)
	{
		for (unsigned int j = 0; j < points_lists[i].size(); j++)
			circle(image, Point(points_lists[i][j].image_x, points_lists[i][j].image_y), 1, cvScalar(b, g, r), 1, 8, 0);
	}
}


void
show_all_points(Mat &image, unsigned int image_width, unsigned int image_height, unsigned int crop_x, unsigned int crop_y, unsigned int crop_width, unsigned int crop_height)
{
	vector<carmen_velodyne_points_in_cam_t> all_points = carmen_velodyne_camera_calibration_lasers_points_in_camera(
					velodyne_msg, camera_parameters, velodyne_pose, camera_pose, image_width, image_height);

	int max_x = crop_x + crop_width, max_y = crop_y + crop_height;

	for (unsigned int i = 0; i < all_points.size(); i++)
		if (all_points[i].ipx >= crop_x && all_points[i].ipx <= max_x && all_points[i].ipy >= crop_y && all_points[i].ipy <= max_y)
			circle(image, Point(all_points[i].ipx - crop_x, all_points[i].ipy - crop_y), 1, cvScalar(0, 0, 255), 1, 8, 0);
}

void
show_detections(Mat image, vector<pedestrian> predictions, vector<image_cartesian> points, vector<vector<image_cartesian>> points_inside_bbox,
		vector<vector<image_cartesian>> filtered_points, double fps, unsigned int image_width, unsigned int image_height, unsigned int crop_x, unsigned int crop_y, unsigned int crop_width, unsigned int crop_height)
{
	char object_info[25];
    char frame_rate[25];

    cvtColor(image, image, COLOR_RGB2BGR);

    sprintf(frame_rate, "FPS = %.2f", fps);

    putText(image, frame_rate, Point(10, 25), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
    	if (predictions[i].active)
    	{
			sprintf(object_info, "%d Person", predictions[i].track_id);

			rectangle(image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
						  obj_colours_vector[0.0], 2);

			putText(image, object_info, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);
    	}
	}

//	show_all_points(image, image_width, image_height, crop_x, crop_y, crop_width, crop_height);
	vector<vector<image_cartesian>> lidar_points;
	lidar_points.push_back(points);
    show_LIDAR(image, lidar_points, 255, 0, 0);
	show_LIDAR(image, points_inside_bbox,    0, 0, 255);				// Blue points are all points inside the bbox
    show_LIDAR(image, filtered_points, 0, 255, 0); 						// Green points are filtered points

//    resize(image, image, Size(600, 300));
    imshow("Neural Object Detector", image);
    //imwrite("Image.jpg", image);
    waitKey(1);
}


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

int board_x = 0.572;      // TODO ler do carmen ini
int board_y = 0.0;
int board_z = 2.154;

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
//			carmen_translte_2d(&tmp_predictions[i].x_world[tmp_predictions[i].circular_idx], &tmp_predictions[i].y_world[tmp_predictions[i].circular_idx], board_x, board_y);
//			carmen_rotate_2d  (&tmp_predictions[i].x_world[tmp_predictions[i].circular_idx], &tmp_predictions[i].y_world[tmp_predictions[i].circular_idx], carmen_normalize_theta(globalpos.theta));
//			carmen_translte_2d(&tmp_predictions[i].x_world[tmp_predictions[i].circular_idx], &tmp_predictions[i].y_world[tmp_predictions[i].circular_idx], globalpos.x, globalpos.y);

			msg.point_clouds[l].r = 1.0;
			msg.point_clouds[l].g = 1.0;
			msg.point_clouds[l].b = 0.0;

			msg.point_clouds[l].linear_velocity = tmp_predictions[i].velocity;
			printf("%f\n",tmp_predictions[i].velocity);
			msg.point_clouds[l].orientation = tmp_predictions[i].orientation;

			msg.point_clouds[l].length = 4.5;
			msg.point_clouds[l].height = 1.8;
			msg.point_clouds[l].width  = 1.6;

			msg.point_clouds[l].object_pose.x = get_pedestrian_x(tmp_predictions[i]);
			msg.point_clouds[l].object_pose.y = get_pedestrian_y(tmp_predictions[i]);
			msg.point_clouds[l].object_pose.z = 0.0;


			msg.point_clouds[l].geometric_model = 0;
			msg.point_clouds[l].model_features.geometry.height = 1.8;
			msg.point_clouds[l].model_features.geometry.length = 3.0;
			msg.point_clouds[l].model_features.geometry.width = 1.0;
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

				carmen_translte_2d(&p.x, &p.y, board_x, board_y);
				carmen_rotate_2d  (&p.x, &p.y, globalpos.theta);
				carmen_translte_2d(&p.x, &p.y, globalpos.x, globalpos.y);

				msg.point_clouds[l].points[j] = p;
			}
			l++;
		}
	}
	return (msg);
}


vector<bbox_t>
filter_predictions_of_interest(vector<bbox_t> &predictions)
{
	vector<bbox_t> filtered_predictions;

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		if (predictions[i].obj_id == 0)// ||  // person
//			i == 1 ||  // bicycle
//			i == 2 ||  // car
//			i == 3 ||  // motorbike
//			i == 5 ||  // bus
//			i == 6 ||  // train
//			i == 7 ||  // truck
//			i == 9)    // traffic light
		{
			filtered_predictions.push_back(predictions[i]);
		}
	}
	return (filtered_predictions);
}


void
compute_annotation_specifications(vector<vector<image_cartesian>> traffic_light_clusters)
{
	double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
	int i, j;

	for (i = 0; i < traffic_light_clusters.size(); i++)
	{
		for (j = 0; j < traffic_light_clusters[i].size(); j++)
		{
			//printf("%lf %lf %lf\n", traffic_light_clusters[i][j].cartesian_x, traffic_light_clusters[i][j].cartesian_y, traffic_light_clusters[i][j].cartesian_z);

			mean_x += traffic_light_clusters[i][j].cartesian_x;
			mean_y += traffic_light_clusters[i][j].cartesian_y;
			mean_z += traffic_light_clusters[i][j].cartesian_z;
		}
		printf("TL %lf %lf %lf\n", mean_x/j, mean_y/j, mean_z/j);

		mean_x = 0.0;
		mean_y = 0.0;
		mean_z = 0.0;
	}
}


void
carmen_translte_3d(double *x, double *y, double *z, double offset_x, double offset_y, double offset_z)
{
	*x += offset_x;
	*y += offset_y;
	*z += offset_z;
}


void
generate_traffic_light_annotations(vector<bbox_t> predictions, vector<vector<image_cartesian>> points_inside_bbox)
{
	static vector<image_cartesian> traffic_light_points;
	static int count = 0;
	int traffic_light_found = 1;

	//printf("--- %lf %lf\n", globalpos.x, globalpos.y);

	for (int i = 0; i < predictions.size(); i++)
	{
		if (predictions[i].obj_id == 9)
		{
			//printf("%s\n", obj_names_vector[predictions[i].obj_id].c_str());
			for (int j = 0; j < points_inside_bbox[i].size(); j++)
			{
				//printf("%lf %lf\n", points_inside_bbox[i][j].cartesian_x, points_inside_bbox[i][j].cartesian_y);

				//carmen_translte_3d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, &points_inside_bbox[i][j].cartesian_z, board_x, board_y, board_z);
				carmen_translte_2d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, board_x, board_y);
				carmen_rotate_2d  (&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, globalpos.theta);
				carmen_translte_2d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, globalpos.x, globalpos.y);

				traffic_light_points.push_back(points_inside_bbox[i][j]);
				//printf("%lf %lf\n", points_inside_bbox[i][j].cartesian_x, points_inside_bbox[i][j].cartesian_y);
			}
			count = 0;
			traffic_light_found = 0;
		}
	}
	count += traffic_light_found;

	if (count >= 20)                // If stays without see a traffic light for more than 20 frames
	{                               // Compute traffic light positions and generate annotations
		vector<vector<image_cartesian>> traffic_light_clusters = dbscan_compute_clusters(0.5, 3, traffic_light_points);
		printf("--- %d\n", (int)traffic_light_clusters.size());
		compute_annotation_specifications(traffic_light_clusters);
		traffic_light_points.clear();
		count = 0;
	}
	//printf("Cont %d\n", count);
}

//////// Python
PyObject *python_pedestrian_tracker_function;
npy_intp image_dimensions[3];

void
init_python(int image_width, int image_height)
{
	Py_Initialize();
	import_array();

	PyObject *python_module_name = PyString_FromString((char *) "pedestrian_tracker");

	PyObject *python_module = PyImport_Import(python_module_name);

	if (python_module == NULL)
	{
		Py_Finalize();
		exit (printf("Error: The python_module could not be loaded.\nMay be PYTHON_PATH is not set.\n"));
	}
	Py_DECREF(python_module_name);

	PyObject *python_set_image_settings_function = PyObject_GetAttrString(python_module, (char *) "set_image_settings");

	if (python_set_image_settings_function == NULL || !PyCallable_Check(python_set_image_settings_function))
	{
		Py_Finalize();
		exit (printf("Error: Could not load the python_set_image_settings_function.\n"));
	}

	PyObject *python_arguments = Py_BuildValue("(ii)", image_width, image_height);               // Generic function, create objects from C values by a format string

	PyObject_CallObject(python_set_image_settings_function, python_arguments);

	Py_DECREF(python_arguments);
	Py_DECREF(python_set_image_settings_function);

	python_pedestrian_tracker_function = PyObject_GetAttrString(python_module, (char *) "run_pedestrian_tracker");

	if (python_pedestrian_tracker_function == NULL || !PyCallable_Check(python_pedestrian_tracker_function))
	{
		Py_DECREF(python_module);
		Py_Finalize();
		exit (printf("Error: Could not load the python_semantic_segmentation_function.\n"));
	}

	image_dimensions[0] = image_height;           //create shape for numpy array
	image_dimensions[1] = image_width;
	image_dimensions[2] = 3;
}


float*
convert_predtions_array(vector<bbox_t> predictions)
{
	unsigned int size = predictions.size();

	float *array = (float*) malloc(size * 4 * sizeof(float));

	for (int i = 0; i < size; i++)
	{
		array[i*4]     = predictions[i].x;
		array[i*4 + 1] = predictions[i].y;
		array[i*4 + 2] = predictions[i].w;
		array[i*4 + 3] = predictions[i].h;
	}

	return (array);
}


void
call_python_function(unsigned char *image, vector<bbox_t> predictions, double timestamp)
{
	npy_intp predictions_dimensions[2] = {(int)predictions.size(), 4};


	PyObject* numpy_image_array = PyArray_SimpleNewFromData(3, image_dimensions, NPY_UBYTE, image);        //convert testVector to a numpy array

	float *array = convert_predtions_array(predictions);

	PyObject* numpy_predictions_array = PyArray_SimpleNewFromData(2, predictions_dimensions, NPY_FLOAT, array);

	PyArrayObject* identifications = (PyArrayObject*)PyObject_CallFunctionObjArgs(python_pedestrian_tracker_function, numpy_image_array, numpy_predictions_array, NULL);

	short *predict = (short*)PyArray_DATA(identifications);
	if (predict == NULL)
	{
		Py_Finalize();
		exit (printf("Error: The predctions erro.\n"));
	}
	
	update_pedestrians(predict,timestamp);

	if (PyErr_Occurred())
        PyErr_Print();

	free(array);
	Py_DECREF(numpy_image_array);
	Py_DECREF(numpy_predictions_array);
	Py_DECREF(identifications);
}
///////////////



unsigned char *
crop_raw_image(int image_width, int image_height, unsigned char *raw_image, int displacement_x, int displacement_y, int crop_width, int crop_height)
{
	if (displacement_x == 0 && displacement_y == 0 && crop_width == image_width && image_height == crop_height)
		return (raw_image);

	unsigned char *cropped_image = (unsigned char *) malloc (crop_width * crop_height * 3 * sizeof(unsigned char));  // Only works for 3 channels image

	displacement_x = (displacement_x - 2) * 3;
	displacement_y = (displacement_y - 2) * image_width * 3;
	crop_width     = displacement_x + ((crop_width + 1) * 3);
	crop_height    = displacement_y + ((crop_height + 1) * image_width * 3);
	image_height   = image_height * image_width * 3;
	image_width   *= 3;

	for (int line = 0, index = 0; line < image_height; line += image_width)
	{
		for (int column = 0; column < image_width; column += 3)
		{
			if (column > displacement_x && column < crop_width && line > displacement_y && line < crop_height)
			{
				cropped_image[index]     = raw_image[line + column];
				cropped_image[index + 1] = raw_image[line + column + 1];
				cropped_image[index + 2] = raw_image[line + column + 2];

				index += 3;
			}
		}
	}

	return (cropped_image);
}


void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	unsigned char *img;
	double fps;
	static double start_time = 0.0;
	static bool first_time = true;

	vector<image_cartesian> points;
	vector<vector<image_cartesian>> points_inside_bbox;
	vector<vector<image_cartesian>> filtered_points;
	vector<image_cartesian> positions;

//	int crop_x = image_msg->width * 0.25;
//	int crop_y = image_msg->height * 0.25;
//	int crop_w = image_msg->width * 0.50;
//	int crop_h = image_msg->height * 0.5;

	int crop_x = 0;
	int crop_y = 0;
	int crop_w = image_msg->width;// 1280;
	int crop_h = image_msg->height;//400; // 500;

	if (camera_side == 0)
		img = image_msg->raw_left;
	else
		img = image_msg->raw_right;

	Mat open_cv_image = Mat(image_msg->height, image_msg->width, CV_8UC3, img, 0);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	//printf("%d %d\n", image_msg->width, image_msg->height);
	//printf("%d %d %d %d\n", crop_x, crop_y, crop_w, crop_h);

	Rect myROI(crop_x, crop_y, crop_w, crop_h);     // TODO put this in the .ini file
	open_cv_image = open_cv_image(myROI);

	//cvtColor(open_cv_image, open_cv_image, COLOR_RGB2BGR);

	//char image_name[256];
	//sprintf(image_name, "%s%lf.png", "image", image_msg->timestamp);
	//imwrite(image_name , open_cv_image);
	//imshow("NOD", open_cv_image);
	//waitKey(1);

	vector<bbox_t> predictions = darknet->detect(open_cv_image, 0.2);  // Arguments (image, threshold)
	predictions = filter_predictions_of_interest(predictions);

	unsigned char *croped_image = crop_raw_image(image_msg->width, image_msg->height, img, crop_x, crop_y, crop_w, crop_h);

	//////// Python
	call_python_function(croped_image, predictions, image_msg->timestamp);
	///////////////

	if ((pedestrian_tracks.size() > 0 || simulated_pedestrians.size() > 0) && velodyne_msg != NULL)
	{
		if (pedestrian_tracks.size() > 0)
		{
			points = velodyne_camera_calibration_fuse_camera_lidar(velodyne_msg, camera_parameters, velodyne_pose, camera_pose,
							image_msg->width, image_msg->height, crop_x, crop_y, crop_w, crop_h);
	//		vector<image_cartesian> points = carmen_velodyne_camera_calibration_test(velodyne_msg, camera_parameters, velodyne_pose, camera_pose, image_msg->width, image_msg->height);

			points_inside_bbox = get_points_inside_bounding_boxes(pedestrian_tracks, points); // TODO remover bbox que nao tenha nenhum ponto

			//generate_traffic_light_annotations(predictions, points_inside_bbox);

			filtered_points = filter_object_points_using_dbscan(points_inside_bbox);

			positions = compute_detected_objects_poses(filtered_points);

			for (int i=0; i<positions.size();i++)
			{
				if (!(positions[i].cartesian_x == -999.0 && positions[i].cartesian_y == -999.0))
				{
					carmen_translte_2d(&positions[i].cartesian_x, &positions[i].cartesian_y, board_x, board_y);
					carmen_rotate_2d  (&positions[i].cartesian_x, &positions[i].cartesian_y, carmen_normalize_theta(globalpos.theta));
					carmen_translte_2d(&positions[i].cartesian_x, &positions[i].cartesian_y, globalpos.x, globalpos.y);

					update_world_position(&pedestrian_tracks[i],positions[i].cartesian_x,positions[i].cartesian_y,image_msg->timestamp);
					printf("[%03d] Velocity: %2.2f  - Orientation(absolute | car): %.3f | %.3f \n",
							pedestrian_tracks[i].track_id, pedestrian_tracks[i].velocity,pedestrian_tracks[i].orientation,abs(pedestrian_tracks[i].orientation - globalpos.theta));
				}
			}

			clean_pedestrians(image_msg->timestamp, 1.0);
		}
		update_simulated_pedestrians(image_msg->timestamp);
		carmen_moving_objects_point_clouds_message msg = build_detected_objects_message(pedestrian_tracks, filtered_points);
		publish_moving_objects_message(image_msg->timestamp, &msg);
		fps = 1.0 / (carmen_get_time() - start_time);
		start_time = carmen_get_time();

		//printf("%d %d %d\n", (int) predictions.size(), (int) points_inside_bbox.size(), (int) filtered_points.size());

		//show_detections(open_cv_image, pedestrian_tracks, points, points_inside_bbox, filtered_points, fps, image_msg->width, image_msg->height, crop_x, crop_y, crop_w, crop_h);
	}
	show_detections(open_cv_image, pedestrian_tracks, points, points_inside_bbox, filtered_points, fps, image_msg->width, image_msg->height, crop_x, crop_y, crop_w, crop_h);
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	velodyne_msg = velodyne_message;

    carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_msg);
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	globalpos.theta = globalpos_message->globalpos.theta;
	globalpos.x = globalpos_message->globalpos.x;
	globalpos.y = globalpos_message->globalpos.y;
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
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
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters(int argc, char **argv)
{
	if ((argc != 3))
		carmen_die("%s: Wrong number of parameters. neural_object_detector requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>", argv[0], argc - 1, argv[0]);

	camera = atoi(argv[1]);             // Define the camera to be used
    camera_side = atoi(argv[2]);        // 0 For left image 1 for right image

    int num_items;

    char bumblebee_string[256];
    char camera_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Geather the cameri ID
    sprintf(camera_string, "%s%d", "camera", camera);

    carmen_param_t param_list[] =
    {
		{bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
		{bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
		{bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
		{bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
		{bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },

		{(char *) "velodyne", (char *) "x",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
		{(char *) "velodyne", (char *) "y",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
		{(char *) "velodyne", (char *) "z",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
		{(char *) "velodyne", (char *) "roll",  CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
		{(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
		{(char *) "velodyne", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

		{camera_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
		{camera_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
		{camera_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
		{camera_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
		{camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
		{camera_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL }
    };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
}


void
initialize_yolo_DRKNET()
{
	string darknet_home = getenv("DARKNET_HOME");  // Get environment variable pointing path of DARKNET_HOME

	if (darknet_home.empty())
		printf("Cannot find darknet path. Check if you have correctly set DARKNET_HOME environment variable.\n");

	string cfg_filename = darknet_home + "/cfg/neural_object_detector_yolo.cfg";
	string weight_filename = darknet_home + "/yolo.weights";
	string class_names_file = darknet_home + "/data/coco.names";

	darknet = new Detector(cfg_filename, weight_filename, 0);   // (cfg_filename, weight_filename, GPU ID)

	carmen_test_alloc(darknet);

	objects_names_from_file(class_names_file);

	set_object_vector_color();

	velodyne_msg = NULL;

//	init_python(704, 480);
	init_python(1280, 960);

//	init_python(640, 480);

	//namedWindow( "Display window", WINDOW_AUTOSIZE);

	//CREATE SIMULATED PEDESTRIANS
	fake_pedestrian new_p;
//	new_p.start_time = 1515703040.0;
//	new_p.stop_time = 1515703060.0;
//	new_p.x = 7757327.00;
//	new_p.y = -364110.00;
//	new_p.active = false;
//	new_p.velocity = 2.0;
//	new_p.orientation = .0;
//	simulated_pedestrians.push_back(new_p);
//	fake_pedestrian new_p2;
//	new_p2.start_time = 1515703020.0;
//	new_p2.stop_time = 1515703060.0;
//	new_p2.x = 7757327.00;
//	new_p2.y = -364110.00;
//	new_p2.active = false;
//	new_p2.velocity = 2.0;
//	new_p2.orientation = 1.5;
//	simulated_pedestrians.push_back(new_p2);

	new_p.start_time = 0.0;
	new_p.stop_time = 9915703060.0;
	new_p.x = 7757918.28;
	new_p.y = -363601.10;
	new_p.active = false;
	new_p.velocity = 1.0;
	new_p.orientation = 2.988+M_PI;
	simulated_pedestrians.push_back(new_p);
}


int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);

    read_parameters(argc, argv);

    subscribe_messages();

    signal(SIGINT, shutdown_module);

    initialize_yolo_DRKNET();

	setlocale(LC_ALL, "C");

    carmen_ipc_dispatch();

    return 0;
}
