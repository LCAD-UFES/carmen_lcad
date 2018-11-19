
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <deque>
#include <string>
#include <random>
#include <iostream>
#include <Eigen/Geometry>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sys/stat.h>


using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;


double distance_between_rear_wheels = 1.535;
double distance_between_front_and_rear_axles = 2.625;
double distance_between_front_car_and_front_wheels = 0.85;
double distance_between_rear_car_and_rear_wheels = 0.96;
double car_length = (distance_between_front_and_rear_axles +
		distance_between_rear_car_and_rear_wheels +
		distance_between_front_car_and_front_wheels);
double car_width = distance_between_rear_wheels;
double center_to_rear_axis = car_length / 2. - distance_between_rear_car_and_rear_wheels;


double
normalize_theta(double theta)
{
	double multiplier;

	if (theta >= -M_PI && theta < M_PI)
		return theta;

	multiplier = floor(theta / (2*M_PI));
	theta = theta - multiplier*2*M_PI;
	if (theta >= M_PI)
		theta -= 2*M_PI;
	if (theta < -M_PI)
		theta += 2*M_PI;

	return theta;
}


double
radians_to_degrees(double theta)
{
	return (theta * 180.0 / M_PI);
}


double
degrees_to_radians(double theta)
{
	return (theta * M_PI / 180.0);
}


Matrix<double, 4, 4>
pose6d_to_matrix(double x, double y, double z, double roll, double pitch, double yaw)
{
	Matrix<double, 3, 3> Rx, Ry, Rz, R;
	Matrix<double, 4, 4> T;

	double rx, ry, rz;

	rx = roll;
	ry = pitch;
	rz = yaw;

    Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
    Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
    R  = Rz * Ry * Rx;

    T << R(0, 0), R(0, 1), R(0, 2), x,
    	R(1, 0), R(1, 1), R(1, 2), y,
		R(2, 0), R(2, 1), R(2, 2), z,
		0, 0, 0, 1;

    return T;
}


class GridMapTile
{
public:
	// make these attributes private
	// they are used in the view function
	double _hm, _wm, _xo, _yo;
	double _m_by_pixel;
	double _pixels_by_m;
	int _h, _w;
	Mat *_map;
	//double **_map_tiles;

	static const int TYPE_SEMANTIC = 0;
	static const int TYPE_VISUAL = 1;
	static const int _N_CLASSES = 15;

	vector<double> _unknown;
	int _n_fields_by_cell;
	int _map_type;
	string _tiles_dir;

	bool contains(double x, double y)
	{
		if (x < _xo || x >= (_xo + _wm) || y < _yo || y >= (_yo + _hm))
			return false;

		return true;
	}

	static const char* type2str(int map_type)
	{
		if (map_type == TYPE_SEMANTIC)
			return "semantic";
		else if (map_type == TYPE_VISUAL)
			return "visual";
		else
			exit(printf("Map type '%d' not found.\n", map_type));
	}

	void save()
	{
		char name[256];
		sprintf(name, "%s/%s_%lf_%lf.config",
				_tiles_dir.c_str(),
				GridMapTile::type2str(_map_type),
				_xo, _yo);

		FILE *fptr = fopen(name, "w");

		if (fptr == NULL)
			exit(printf("Error: Unable to create the file '%s'", name));

		fprintf(fptr, "xo: %lf\n", _xo);
		fprintf(fptr, "yo: %lf\n", _yo);
		fprintf(fptr, "hm: %lf\n", _hm);
		fprintf(fptr, "wm: %lf\n", _wm);
		fprintf(fptr, "m_by_pixel: %lf\n", _m_by_pixel);
		fprintf(fptr, "map_type: %d\n", _map_type);

		fclose(fptr);

		sprintf(name, "%s/%s_%lf_%lf.png",
				_tiles_dir.c_str(),
				GridMapTile::type2str(_map_type),
				_xo, _yo);

		imwrite(name, *_map);
	}

	void
	_initialize_map()
	{
		// assumes the origin is set.
		char name[256];

		sprintf(name, "%s/%s_%lf_%lf.png",
				_tiles_dir.c_str(),
				GridMapTile::type2str(_map_type),
				_xo, _yo);

		_map = new Mat(_h, _w, CV_8UC3);

		struct stat buffer;

		if (stat(name, &buffer) == 0)
			imread(name).copyTo(*_map);
		else
			memset(_map->data, 128, _h * _w * 3 * sizeof(unsigned char));
	}

	void
	_initialize_derivated_values()
	{
		_pixels_by_m = 1 / _m_by_pixel;
		_h = (int) (_pixels_by_m * _hm);
		_w = (int) (_pixels_by_m * _wm);

		if (_map_type == GridMapTile::TYPE_SEMANTIC)
		{
			// The first '_N_CLASSES' fields contains the count for
			// each class, and the last field is a boolean informing
			// if the cell was observed. All classes are initialized with
			// a count of 1 to prevent probabilities equal to zero when
			// computing particle weights.
			_n_fields_by_cell = _N_CLASSES + 1;
			_unknown = vector<double>(_n_fields_by_cell, 1.);
			_unknown[_unknown.size() - 1] = 0;
		}
		else if (_map_type == GridMapTile::TYPE_VISUAL)
		{
			// r, g, b
			_n_fields_by_cell = 3;
			_unknown = vector<double>(_n_fields_by_cell, -1);
		}
		else
			exit(printf("Map type '%d' not found.\n", _map_type));
	}

	GridMapTile(double point_y, double point_x,
			double height_meters, double width_meters,
			double resolution, int map_type, string tiles_dir)
	{
		// map tile origin
		_xo = floor(point_x / width_meters) * width_meters;
		_yo = floor(point_y / height_meters) * height_meters;
		printf("Origin: %lf %lf\n", _xo, _yo);
		_hm = height_meters;
		_wm = width_meters;
		_m_by_pixel = resolution;
		_map_type = map_type;
		_tiles_dir = tiles_dir;

		_initialize_derivated_values();
		_initialize_map();
	}

	~GridMapTile()
	{
		save();
		delete(_map);
	}

	void
	add_point(PointXYZRGB &p)
	{
		int px, py, pos;

		px = (p.x - _xo) * _pixels_by_m;
		py = (p.y - _yo) * _pixels_by_m;

		double _r = 0;
		unsigned char *data_vector = (unsigned char *) _map->data;

		if (px >= 0 && px < _w && py >= 0 && py < _h)
		{
			pos = 3 * (py * _w + px);

			data_vector[pos + 0] = (unsigned char) (data_vector[pos + 0] * _r + p.b * (1. - _r));
			data_vector[pos + 1] = (unsigned char) (data_vector[pos + 1] * _r + p.g * (1. - _r));
			data_vector[pos + 2] = (unsigned char) (data_vector[pos + 2] * _r + p.r * (1. - _r));
		}
	}

	vector<double>
	read_cell(PointXYZRGB &p)
	{
		int px, py, pos;

		px = (p.x - _xo) * _pixels_by_m;
		py = (p.y - _yo) * _pixels_by_m;

		double _r = 0;
		unsigned char *data_vector = (unsigned char *) _map->data;

		vector<double> v;

		if (px >= 0 && px < _w && py >= 0 && py < _h)
		{
			pos = 3 * (py * _w + px);

			v.push_back(data_vector[pos + 0]);
			v.push_back(data_vector[pos + 1]);
			v.push_back(data_vector[pos + 2]);
		}
		else
		{
			v.push_back(-1);
			v.push_back(-1);
			v.push_back(-1);
		}

		return v;
	}

	Mat
	to_image() { return _map->clone(); }
};


class GridMap
{
public:
	static const int _N_TILES = 3;

	string _tiles_dir;
	double _height_meters;
	double _width_meters;
	double _resolution;
	int _middle_tile;
	int _map_type;

	GridMapTile *_tiles[_N_TILES][_N_TILES];

	GridMap(string tiles_dir, double height_meters, double width_meters, double resolution,
			int map_type)
	{
		_tiles_dir = tiles_dir;
		_height_meters = height_meters;
		_width_meters = width_meters;
		_resolution = resolution;
		_map_type = map_type;

		for (int i = 0; i < _N_TILES; i++)
			for (int j = 0; j < _N_TILES; j++)
				_tiles[i][j] = NULL;

		_middle_tile = (int) (_N_TILES / 2.);
	}

	GridMapTile*
	_reload_tile(double x, double y)
	{
		return new GridMapTile(y, x, _height_meters,
				_width_meters, _resolution, _map_type, _tiles_dir);
	}

	void
	_reload_tiles(double robot_x, double robot_y)
	{
		printf("Reloading tiles!\n");

		int i, j;

		for (i = 0; i < _N_TILES; i++)
			for (j = 0; j < _N_TILES; j++)
				if (_tiles[i][j] != NULL)
					delete(_tiles[i][j]);

		// TODO: make the following code general.
		_tiles[0][0] = _reload_tile(robot_x - 75., robot_y - 75.);
		_tiles[0][1] = _reload_tile(robot_x, robot_y - 75.);
		_tiles[0][2] = _reload_tile(robot_x + 75., robot_y - 75.);
		_tiles[1][0] = _reload_tile(robot_x - 75., robot_y);
		_tiles[1][1] = _reload_tile(robot_x, robot_y);
		_tiles[1][2] = _reload_tile(robot_x + 75., robot_y);
		_tiles[2][0] = _reload_tile(robot_x - 75., robot_y + 75.);
		_tiles[2][1] = _reload_tile(robot_x, robot_y + 75.);
		_tiles[2][2] = _reload_tile(robot_x + 75., robot_y + 75.);
	}

	void
	reload(double robot_x, double robot_y)
	{
		if (_tiles[_middle_tile][_middle_tile] == NULL)
			_reload_tiles(robot_x, robot_y);

		if (!_tiles[_middle_tile][_middle_tile]->contains(robot_x, robot_y))
			_reload_tiles(robot_x, robot_y);
	}

	void
	add_point(PointXYZRGB &p)
	{
		int i, j;

		for (i = 0; i < _N_TILES; i++)
		{
			for (j = 0; j < _N_TILES; j++)
			{
				if (_tiles[i][j]->contains(p.x, p.y))
				{
					_tiles[i][j]->add_point(p);
					return;
				}
			}
		}
	}

	vector<double>
	read_cell(PointXYZRGB &p)
	{
		int i, j;

		for (i = 0; i < _N_TILES; i++)
		{
			for (j = 0; j < _N_TILES; j++)
			{
				if (_tiles[i][j]->contains(p.x, p.y))
					return _tiles[i][j]->read_cell(p);
			}
		}
	}

	Mat
	to_image()
	{
		Mat middle_tile = _tiles[1][1]->to_image();
		int h = middle_tile.rows;
		int w = middle_tile.cols;

		Mat viewer(h * 3, w * 3, CV_8UC3);

		_tiles[0][0]->to_image().copyTo(viewer(Rect(0, 0, w, h)));
		_tiles[0][1]->to_image().copyTo(viewer(Rect(w, 0, w, h)));
		_tiles[0][2]->to_image().copyTo(viewer(Rect(2*w, 0, w, h)));
		_tiles[1][0]->to_image().copyTo(viewer(Rect(0, h, w, h)));
		_tiles[1][1]->to_image().copyTo(viewer(Rect(w, h, w, h)));
		_tiles[1][2]->to_image().copyTo(viewer(Rect(2*w, h, w, h)));
		_tiles[2][0]->to_image().copyTo(viewer(Rect(0, 2*h, w, h)));
		_tiles[2][1]->to_image().copyTo(viewer(Rect(w, 2*h, w, h)));
		_tiles[2][2]->to_image().copyTo(viewer(Rect(2*w, 2*h, w, h)));

		return viewer;
	}
};


class Pose2d
{
public:
	double x, y, th;

	Pose2d(double px = 0, double py = 0, double pth = 0)
	{
		x = px;
		y = py;
		th = pth;
	}

	Pose2d operator=(Pose2d p)
	{
		x = p.x;
		y = p.y;
		th = p.th;
		return *this;
	}

	static double theta_from_matrix(Matrix<double, 4, 4> &m)
	{
		// extract rotation matrix
		static Matrix<double, 3, 3> R;
		R << m(0, 0), m(0, 1), m(0, 2),
				m(1, 0), m(1, 1), m(1, 2),
				m(2, 0), m(2, 1), m(2, 2);

		// Important:
		// Extracting the yaw component from the rotation matrix is not
		// the right wayt of computing theta. Note that a rotation of yaw=pitch=roll=0
		// is equivalent to a rotation of yaw=pitch=roll=3.14 (in this order), but
		// yaw=0. is the opposite of yaw=3.14.
		// Matrix<double, 3, 1> ypr = R.eulerAngles(2, 1, 0);

		// Here I'm using the ad-hoc approach of rotating an unitary vector
		// and computing theta using the x and y coordinates. TODO: find a principled approach.
		static Matrix<double, 3, 1> unit;
		unit << 1, 0, 0;
		unit = R * unit;

		return atan2(unit(1, 0), unit(0, 0));
	}

	static Pose2d from_matrix(Matrix<double, 4, 4> &m)
	{
		Pose2d p;

		p.x = m(0, 3) / m(3, 3);
		p.y = m(1, 3) / m(3, 3);
		p.th = Pose2d::theta_from_matrix(m);

		return p;
	}

	static Matrix<double, 4, 4> to_matrix(Pose2d &p)
	{
		return pose6d_to_matrix(p.x, p.y, 0, 0, 0, p.th);
	}
};


class ParticleFilter
{
public:
	int _n;
	Pose2d *_p;
	double *_w;

	// p and w before resampling
	Pose2d *_p_bef;
	double *_w_bef;

	double _x_std, _y_std, _th_std, _v_std, _phi_std;
	double _pred_x_std, _pred_y_std, _pred_th_std;
	double _gps_var_x, _gps_var_y, _gps_var_th;
	double _color_var_r, _color_var_g, _color_var_b;

	std::default_random_engine _random_generator;
	std::normal_distribution<double> _std_normal_distribution;

	Pose2d best;

	double _gauss()
	{
		return _std_normal_distribution(_random_generator);
	}

	// public:
	ParticleFilter(int n_particles, double x_std, double y_std, double th_std,
			double v_std, double phi_std, double pred_x_std, double pred_y_std, double pred_th_std,
			double gps_var_x, double gps_var_y, double gps_var_th,
			double color_var_r, double color_var_g, double color_var_b)
	{
		_n = n_particles;

		_p = (Pose2d *) calloc (_n, sizeof(Pose2d));
		_w = (double *) calloc (_n, sizeof(double));

		_p_bef = (Pose2d *) calloc (_n, sizeof(Pose2d));
		_w_bef = (double *) calloc (_n, sizeof(double));

		_x_std = x_std;
		_y_std = y_std;
		_th_std = th_std;
		_v_std = v_std;
		_phi_std = phi_std;

		_pred_x_std = pred_x_std;
		_pred_y_std = pred_y_std;
		_pred_th_std = pred_th_std;

		_gps_var_x = gps_var_x;
		_gps_var_y = gps_var_y;
		_gps_var_th = gps_var_th;

		_color_var_r = color_var_r / pow(255., 2.);
		_color_var_g = color_var_g / pow(255., 2.);
		_color_var_b = color_var_b / pow(255., 2.);
	}

	~ParticleFilter()
	{
		free(_p);
		free(_w);
		free(_p_bef);
		free(_w_bef);
	}

	void seed(int val)
	{
		srand(val);
		_random_generator.seed(val);
	}

	void reset(double x, double y, double th)
	{
		for (int i = 0; i < _n; i++)
		{
			_p[i].x = x + _gauss() * _x_std;
			_p[i].y = y + _gauss() * _y_std;
			_p[i].th = th + _gauss() * _th_std;

			_w[i] = _w_bef[i] = 1. / (double) _n;
			_p_bef[i] = _p[i];
		}

		best = _p[0];
	}

	void predict(double v, double phi, double dt)
	{
		double noisy_v, noisy_phi;

		for (int i = 0; i < _n; i++)
		{
			noisy_v = v + _gauss() * _v_std;
			noisy_phi = phi + _gauss() * _phi_std;

			_p[i].x += noisy_v * dt * cos(_p[i].th) + _gauss() * _pred_x_std;
			_p[i].y += noisy_v * dt * sin(_p[i].th) + _gauss() * _pred_y_std;
			_p[i].th += noisy_v * dt * tan(noisy_phi) / distance_between_front_and_rear_axles;
			_p[i].th = normalize_theta(_p[i].th +  _gauss() * _pred_th_std);

			_p_bef[i] = _p[i];
			_w_bef[i] = _w[i];
		}

		best = _p[0];
	}

	double
	sensor_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map)
	{
		double n_votes = 0;
		double unnorm_log_prob = 0.;
		PointXYZRGB point;

		for (int i = 0; i < transformed_cloud->size(); i++)
		{
			point = transformed_cloud->at(i);
			vector<double> v = map.read_cell(point);

			//printf("point %d: %lf %lf %lf %d %d %d\n",
			//		i, point.x, point.y, point.z,
			//		point.r, point.g, point.b);
            //
			//printf("v: %lf %lf %lf\n", v[0], v[1], v[2]);
			//printf("diff: %lf %lf %lf\n",
			//		(point.r - v[2]) / 255.,
			//		(point.g - v[1]) / 255.,
			//		(point.b - v[0]) / 255.);

			// cell observed at least once.
			// TODO: what to do when the cell was never observed?
			if (v[0] != -1)
			{
				//unnorm_log_prob += (1. / _color_var_r) * pow((point.r - v[2]) / 255., 2) +
				//		(1. / _color_var_g) * pow((point.g - v[1]) / 255., 2) +
				//		(1. / _color_var_b) * pow((point.b - v[0]) / 255., 2);

				unnorm_log_prob += fabs((point.r - v[2]) / 255.) +
							fabs((point.g - v[1]) / 255.) +
							fabs((point.b - v[0]) / 255.);

				n_votes += 1;
			}

			//printf("unnorm_log_prob: %lf\n\n", unnorm_log_prob);
		}

		// return (1. / unnorm_log_prob);

		double c = 10.;  // constant to magnify the particle weights.
		unnorm_log_prob /= (3. * n_votes);
		//printf("n votes: %lf\n", n_votes);
		//printf("unnorm_log_prob: %lf\n", c * unnorm_log_prob);

		return exp(-c * unnorm_log_prob);
	}

	void correct(Pose2d &gps, PointCloud<PointXYZRGB>::Ptr cloud,
			GridMap &map, PointCloud<PointXYZRGB>::Ptr transformed_cloud)
	{
		int i;
		double gps_unnorm_log_prob;
		double sum_probs = 0.;

		int max_id = 0;

		for (i = 0; i < _n; i++)
		{
			if (0)
			{
				gps_unnorm_log_prob = (1. / _gps_var_x) * pow(_p[i].x - gps.x, 2) +
									 (1. / _gps_var_y) * pow(_p[i].y - gps.y, 2) +
									 (1. / _gps_var_th) * pow(_p[i].th - gps.th, 2);

				_w[i] = exp(-gps_unnorm_log_prob);
			}
			else
			{
				transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(_p[i]));

				_w[i] = sensor_weight(transformed_cloud, map);
				//printf("Unormalized _w[%d]: %lf\n", i, _w[i]);
			}

			_p_bef[i] = _p[i];
			sum_probs += _w[i];

			if (_w[i] > _w[max_id])
				max_id = i;
		}

		best = _p[max_id];

		//transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(_p[max_id]));
		//for(int i = 0; i < transformed_cloud->size(); i++)
		//{
		//	transformed_cloud->at(i).r = 255;
		//	transformed_cloud->at(i).g = 0;
		//	transformed_cloud->at(i).b = 0;
		//	map.add_point(transformed_cloud->at(i));
		//}

		//printf("Sum probs: %lf\n", sum_probs);

		// normalize the probabilities
		for (i = 0; i < _n; i++)
		{
			_w[i] /= sum_probs;
			_w_bef[i] = _w[i];

			//printf("Normalized _w[%d]: %lf\n", i, _w[i]);
		}

		// resample
		_p[0] = best; // elitism
		for (i = 1; i < _n; i++)
		{
			double unif = (double) rand() / (double) RAND_MAX;
			int pos = rand() % _n;
			double sum = _w[pos];

			while (sum < unif)
			{
				pos++;

				if (pos == _n)
					pos = 0;

				sum += _w[pos];
			}

			_p[i] = _p_bef[pos];
		}

		// make all particles equally likely after resampling
		for (i = 0; i < _n; i++)
			_w[i] = 1. / (double) _n;
	}

	Pose2d mean()
	{
		Pose2d p;

		for (int i = 0; i < _n; i++)
		{
			p.x += (_p[i].x * _w[i]);
			p.y += (_p[i].y * _w[i]);
			p.th += (_p[i].th * _w[i]);
		}

		p.th = normalize_theta(p.th);

		return p;
	}

	Pose2d mode()
	{
		return best;
	}
};


vector<double>
load_timestamps()
{
	vector<double> times;
	char *time_name = "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/oxts/timestamps.txt";

	int dummy;
	double t;
	FILE *f = fopen(time_name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", time_name));

	while(fscanf(f, "%d-%d-%d %d:%d:%lf", &dummy, &dummy, &dummy, &dummy, &dummy, &t) == 6)
		times.push_back(t);

	fclose(f);
	return times;
}


vector<double>
read_vector(char *name)
{
	double d;
	vector<double> data;

	FILE *f = fopen(name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", name));

	while (!feof(f))
	{
		fscanf(f, "%lf\n", &d);
		data.push_back(d);
	}

	fclose(f);
	return data;
}


// debug
void
print_vector(vector<double> &v)
{
	for (int i = 0; i < v.size(); i++)
		printf("%.2lf ", v[i]);

	printf("\n");
}


vector<vector<double>>
load_oxts(vector<double> &times)
{
	vector<vector<double>> data;

	char *dir = "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/oxts/data";
	char name[1024];

	for (int i = 0; i < times.size(); i++)
	{
		sprintf(name, "%s/%010d.txt", dir, i);
		data.push_back(read_vector(name));
	}

	return data;
}


vector<Matrix<double, 4, 4>>
oxts2Mercartor(vector<vector<double>> &data)
{
	double scale = cos(data[0][0] * M_PI / 180.);
	double er = 6378137;

	// pose[i] contains the transformation which takes a
	// 3D point in the i'th frame and projects it into the oxts
	// coordinates of the first frame.
	vector<Matrix<double, 4, 4>> poses;
	Matrix<double, 4, 4> p0;

	for (int i = 0; i < data.size(); i++)
	{
		double x, y, z;
		x = scale * data[i][1] * M_PI * er / 180;
		y = scale * er * log(tan((90 + data[i][0]) * M_PI / 360));
		z = data[i][2];

		double rx = data[i][3]; // roll
		double ry = data[i][4]; // pitch
		double rz = data[i][5]; // heading

		Matrix<double, 3, 3> Rx, Ry, Rz, R;
		Matrix<double, 4, 4> p;

		Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
		Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
		Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
		R  = Rz * Ry * Rx;

		p << R(0, 0),  R(0, 1),  R(0, 2), x,
				R(1, 0),  R(1, 1),  R(1, 2), y,
				R(2, 0),  R(2, 1),  R(2, 2), z,
				0, 0, 0, 1;

		if (i == 0)
			p0 = p;

		p = p0.inverse() * p;
		poses.push_back(p);
	}

	return poses;
}


void
print_poses(vector<Matrix<double, 4, 4>> &poses)
{
	Matrix<double, 4, 4> p;

	for (int i = 0; i < poses.size(); i++)
	{
		p = poses[i];

		printf("%.2lf %.2lf %.2lf\n",
				p(0, 3) / p(3, 3),
				p(1, 3) / p(3, 3),
				p(2, 3) / p(3, 3));
	}
}


void
read_pointcloud_kitti(int i, PointCloud<PointXYZRGB>::Ptr cloud)
{
	// pointers
	static int num;
	static float *data;
	static int first = 1;

	float *px = data+0;
	float *py = data+1;
	float *pz = data+2;
	float *pr = data+3;

	char name[1024];
	sprintf(name, "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/velodyne_points/data/%010d.bin", i);

	num = 1000000;

	if (first)
	{
		data = (float*) malloc(num * sizeof(float));
		first = 0;
	}

	printf("loading pointcloud '%s'\n", name);

	// load point cloud
	FILE *stream;
	stream = fopen(name, "rb");
	num = fread(data, sizeof(float), num, stream) / 4;
	fclose(stream);

	for (int i = 0; i < num; i++)
	{
		px += 4; py += 4; pz += 4; pr += 4;

		pcl::PointXYZRGB point;

		point.x = *px;
		point.y = *py;
		point.z = *pz;
		point.r = *pr;
		point.g = *pr;
		point.b = *pr;

		cloud->push_back(point);
	}
}


void
read_pointcloud_carmen(int i, PointCloud<PointXYZRGB>::Ptr cloud)
{
	char name[1024];
	sprintf(name, "/dados/data_20180112-2/data/velodyne/%010d.ply", i);
	pcl::io::loadPLYFile(name, *cloud);
}


Matrix<double, 3, 4>
kitti_velodyne_to_cam()
{
	Matrix<double, 4, 4> R00;
	Matrix<double, 3, 4> P02;
	Matrix<double, 4, 4> Rt;

	R00 << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0.,
			-9.869795e-03, 9.999421e-01, -4.278459e-03, 0.,
			7.402527e-03, 4.351614e-03, 9.999631e-01, 0.,
			0., 0., 0., 1.;

	Rt << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
			1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
			9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
			0., 0., 0., 1.;

	P02 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
			0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
			0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03;

	return P02 * R00 * Rt;

}


Matrix<double, 3, 4>
carmen_velodyne_to_cam3(int image_width = 1280, int image_height = 960)
{
	Matrix<double, 3, 4> projection;
	Matrix<double, 4, 4> velodyne2board;
	Matrix<double, 4, 4> cam2board;

	velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	cam2board = pose6d_to_matrix(0.245, -0.04, 0.210, -0.017453, 0.026037, -0.023562);

	// This is a rotation to change the ref. frame from x: forward, y: left, z: up
	// to x: right, y: down, z: forward.
	Matrix<double, 4, 4> R;
	R = pose6d_to_matrix(0., 0., 0., 0., M_PI/2., -M_PI/2);
	//R = pose6d_to_matrix(0., 0., 0., 0., 0., 0.); // TODO

	double fx_factor = 0.764749;
	double fy_factor = 1.01966;
	double cu_factor = 0.505423;
	double cv_factor = 0.493814;
	double pixel_size = 0.00000375;

    double fx_meters = fx_factor * image_width * pixel_size;
    double fy_meters = fy_factor * image_height * pixel_size;

    double cu = cu_factor * image_width;
    double cv = cv_factor * image_height;

    // see http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
    // Note: putting cu and cv in the 3rd column instead of the 4th is a trick
    // because to compute the actual pixel coordinates we divide the first two
    // dimensions of the point in homogenous coordinates by the third one (which is Z).
	projection << fx_meters / pixel_size, 0, cu, 0,
				  0, fy_meters / pixel_size, cv, 0,
				  0, 0, 1, 0.;

	return projection * R * cam2board.inverse() * velodyne2board;
}


void
load_fused_pointcloud_and_camera(int i, PointCloud<PointXYZRGB>::Ptr cloud)
{
	cloud->clear();

	PointCloud<PointXYZRGB>::Ptr raw_cloud(new PointCloud<PointXYZRGB>);
	//read_pointcloud_kitti(i, raw_cloud);
	read_pointcloud_carmen(i, raw_cloud);

	char name[1024];
	//sprintf(name, "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/image_02/data/%010d.png", i);
	//sprintf(name, "/dados/imgs_kitti/%010d_trainval.png", i);
	//sprintf(name, "/dados/data_20180112-2/data/trainfine/%010d.png", i);
	sprintf(name, "/dados/data_20180112-2/data/bb3/%010d.png", i);

	//Mat img(375, 1242, CV_8UC3);
	//Mat raw_img = imread(name);
	//cv::resize(raw_img, img, img.size());
	Mat img = imread(name);

	printf("loading image '%s'\n", name);

	Mat view = img.clone();

	int p, x, y;
	Matrix<double, 4, 1> P;
	Matrix<double, 3, 1> pixel;
	PointXYZRGB point;

	//Matrix<double, 3, 4> vel2cam = kitti_velodyne_to_cam();
	Matrix<double, 3, 4> vel2cam = carmen_velodyne_to_cam3(img.cols, img.rows);

	for (int i = 0; i < raw_cloud->size(); i++)
	{
		point = raw_cloud->at(i);

		P << point.x, point.y, point.z, 1.;
		pixel = vel2cam * P;

		x = pixel(0, 0) / pixel(2, 0);
		y = pixel(1, 0) / pixel(2, 0);
		y = img.rows - y - 1;
		x = img.cols - x - 1;

		//cout << pose << endl << endl;
		//cout << P << endl << endl;
		//cout << Pt << endl << endl;
		//cout << "P3d:" << endl;
		//cout << P << endl;
		//cout << "pixel:" << endl;
		//cout << pixel << endl;
		//cout << "x-y: " << x << " " << y << endl;
		//cout << endl;

		//if (0)
		if (point.x > 7 && x >= 0 && x < img.cols && y >= 0 && y < img.rows)
		{
			pcl::PointXYZRGB point2;

			point2.x = P(0, 0) / P(3, 0);
			point2.y = P(1, 0) / P(3, 0);
			point2.z = P(2, 0) / P(3, 0);

			// colors
			p = 3 * (y * img.cols + x);
			point2.r = img.data[p + 2];
			point2.g = img.data[p + 1];
			point2.b = img.data[p + 0];

			circle(view, Point(x, y), 2, Scalar(0,0,255), -1);
			cloud->push_back(point2);
		}

		else if (0)
		//else if (1) //point.z < 0.)
		{
			point.r *= 3;
			point.g *= 3;
			point.b *= 3;
			cloud->push_back(point);
		}
	}

	imshow("img", view);
}


void
add_point_cloud_to_map(PointCloud<PointXYZRGB>::Ptr cloud, GridMap &map)
{
	for (int i = 0; i < cloud->size(); i++)
		map.add_point(cloud->at(i));
}


vector<pair<double, double>>
estimate_v(vector<Matrix<double, 4, 4>> &poses, vector<double> &ts)
{
	vector<pair<double, double>> vs;

	double lx = 0, ly = 0, lt = 0;

	for (int i = 0; i < poses.size(); i++)
	{
		double x = poses[i](0, 3) / poses[i](3, 3);
		double y = poses[i](1, 3) / poses[i](3, 3);

		double ds = sqrt(pow(x - lx, 2) + pow(y - ly, 2));
		double dt = ts[i] - lt;

		lx = x;
		ly = y;
		lt = ts[i];

		if (i > 0)
			vs.push_back(pair<double, double>(ds / dt, 0));

		if (i == 1)
			vs.push_back(pair<double, double>(ds / dt, 0));
	}

	return vs;
}


void
draw_rectangle(Mat &img,
		double x, double y, double theta,
		double height, double width, Scalar color,
		double x_origin, double y_origin, double pixels_by_meter)
{
/*
	vector<Point2f> vertices;
	vector<Point> vertices_px;

	vertices.push_back(Point2f(-width / 2., -height / 2.));
	vertices.push_back(Point2f(-width / 2., height / 2.));
	vertices.push_back(Point2f(width / 2., height / 2.));
	vertices.push_back(Point2f(width / 2., 0.));
	vertices.push_back(Point2f(0., 0.));
	vertices.push_back(Point2f(width / 2., 0));
	vertices.push_back(Point2f(width / 2., -height / 2.));

	double v_radius, v_angle;

	// transform vertices
	for (int i = 0; i < vertices.size(); i++)
	{
		v_radius = sqrt(pow(vertices[i].x, 2.) + pow(vertices[i].y, 2.));
		v_angle = atan2(vertices[i].y, vertices[i].x);

		vertices[i].x = v_radius * cos(v_angle + theta) + x;
		vertices[i].y = v_radius * sin(v_angle + theta) + y;

		Point p;
		p.x = (int) ((vertices[i].x - x_origin) * pixels_by_meter);
		p.y = (int) ((vertices[i].y - y_origin) * pixels_by_meter);

		vertices_px.push_back(p);
	}

	for (int i = 0; i < vertices_px.size(); i++)
	{
		if (i == vertices_px.size() - 1)
			line(img, vertices_px[i], vertices_px[0], color, 1);
		else
			line(img, vertices_px[i], vertices_px[i + 1], color, 1);
	}
*/
}


void
draw_poses(GridMap &map, Mat &map_img, vector<Matrix<double, 4, 4>> &poses)
{
/*
	Point pixel;
	int radius = (0.25 * map._pixels_by_m);
	Pose2d p;
	double shift_x, shift_y;

	for (int i = 0; i < poses.size(); i += 1)
	{
		p = Pose2d::from_matrix(poses[i]);

		shift_x = center_to_rear_axis * cos(p.th);
		shift_y = center_to_rear_axis * sin(p.th);

		pixel.x = (p.x - map._xo) * map._pixels_by_m;
		pixel.y = (p.y - map._yo) * map._pixels_by_m;
		circle(map_img, pixel, radius, Scalar(0, 255, 0), -1);

		draw_rectangle(map_img,
				p.x + shift_x,
				p.y + shift_y,
				p.th,
				car_width, car_length, Scalar(0, 255, 0),
				map._xo,
				map._yo,
				map._pixels_by_m);
	}
*/
}


void
draw_particle(Mat &map_img, Pose2d &p, GridMap &map, int radius, Scalar color)
{
/*
	Point pixel;

	pixel.x = (p.x - map._xo) * map._pixels_by_m;
	pixel.y = (p.y - map._yo) * map._pixels_by_m;
	circle(map_img, pixel, radius, color, 1);

	Point p2;
	p2.x = ((2.0 * cos(p.th) + p.x) - map._xo) * map._pixels_by_m;
	p2.y = ((2.0 * sin(p.th) + p.y) - map._yo) * map._pixels_by_m;

	line(map_img, pixel, p2, color, 1);
*/
}


void
view(ParticleFilter &pf, GridMap &map, vector<Matrix<double, 4, 4>> &poses, Pose2d current_pose,
	PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr transformed_cloud)
{
	/*
	int i, j;
	Pose2d p;
	Point pixel;
	double shift_x, shift_y;

	Mat bkp;
	Pose2d mean = pf.mean();
	Pose2d mode = pf.mode();

	if (cloud != NULL)
	{
		bkp = map.to_image();
		transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(mode));
		for(int i = 0; i < transformed_cloud->size(); i++)
		{
			transformed_cloud->at(i).r = 255;
			transformed_cloud->at(i).g = 0;
			transformed_cloud->at(i).b = 0;
			map.add_point(transformed_cloud->at(i));
		}
	}

	Mat map_img = map.to_image();

	if (cloud != NULL)
		bkp.copyTo(*map._map);

	int radius = (0.25 * map._pixels_by_m);
	*/

	/*
	for (i = 0; i < poses.size(); i += 1)
	{
		p = Pose2d::from_matrix(poses[i]);

		shift_x = center_to_rear_axis * cos(p.th);
		shift_y = center_to_rear_axis * sin(p.th);

		pixel.x = (p.x - map._xo) * map._pixels_by_m;
		pixel.y = (p.y - map._yo) * map._pixels_by_m;
		circle(map_img, pixel, radius, Scalar(0, 255, 0), -1);

		// GPS error area
		circle(map_img, pixel, sqrt(pf._gps_var_x) * map._pixels_by_m, Scalar(0, 255, 0), 1);

		draw_rectangle(map_img,
				p.x + shift_x,
				p.y + shift_y,
				p.th,
				car_width, car_length, Scalar(0, 255, 0),
				map._xo,
				map._yo,
				map._pixels_by_m);
	}
	*/

	/*
	for (i = 0; i < pf._n; i++)
		draw_particle(map_img, pf._p[i], map, radius, Scalar(0, 0, 255));

	draw_particle(map_img, mean, map, radius, Scalar(0, 255, 255));
	draw_particle(map_img, mode, map, radius, Scalar(255, 0, 0));
	draw_particle(map_img, current_pose, map, radius, Scalar(0, 0, 0));

	//Mat resized_map(1000, 1000, CV_8UC3);
	//resize(map_img, resized_map, resized_map.size());
	imshow("map", map_img);
	waitKey(-1);
	*/
}


Matrix<double, 4, 4>
carmen_vel2car()
{
	Matrix<double, 4, 4> velodyne2board;
	Matrix<double, 4, 4> board2car;

	velodyne2board = pose6d_to_matrix(0.145, 0., 0.48, 0.0, -0.0227, -0.01);
	board2car = pose6d_to_matrix(0.572, 0, 1.394, 0.0, 0.0122173048, 0.0);

	return board2car * velodyne2board;
}


void
create_map(GridMap &map, vector<Matrix<double, 4, 4>> &poses, PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr transformed_cloud)
{
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(.5, .5, .5);
	viewer.removeAllPointClouds();
	viewer.addCoordinateSystem(10);

	Matrix<double, 4, 4> vel2car = carmen_vel2car();
	Matrix<double, 4, 4> car2world;

	deque<string> cloud_names;
	int step = 1;

	for (int i = 0; i < poses.size(); i += 1)
	{
		load_fused_pointcloud_and_camera(i, cloud);

		car2world = poses[i] * vel2car;
		transformPointCloud(*cloud, *transformed_cloud, car2world);
		//transformed_cloud = cloud;

		map.reload(poses[i](0, 3), poses[i](1, 3));
		printf("car pose: %lf %lf\n", poses[i](0, 3), poses[i](1, 3));

		add_point_cloud_to_map(transformed_cloud, map);

		char *cloud_name = (char *) calloc (32, sizeof(char));
		sprintf(cloud_name, "cloud%d", i);
		//viewer.removeAllPointClouds();
		viewer.addPointCloud(transformed_cloud, cloud_name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name);
		cloud_names.push_back(cloud_name);

		//if (cloud_names.size() >= 10)
		//{
		//	viewer.removePointCloud(cloud_names[0]);
		//	cloud_names.pop_front();
		//}

		Mat map_img = map.to_image().clone();
		draw_poses(map, map_img, poses);
		imshow("map", map_img);

		char c = ' ';
		while (1)
		{
			viewer.spinOnce();
			c = waitKey(5);

			if (c == 's')
				step = !step;
			if (!step || (step && c == 'n'))
				break;
			if (c == 'r')
			{
				printf("Reinitializing\n");
				i = 0;
			}
		}
	}

	waitKey(-1);
}


void
run_particle_filter(ParticleFilter &pf, GridMap &map, vector<Matrix<double, 4, 4>> &poses,
		vector<pair<double, double>> &odom, vector<double> &times, PointCloud<PointXYZRGB>::Ptr cloud,
		PointCloud<PointXYZRGB>::Ptr transformed_cloud)
{
	while (1)
	{
		pf.seed(time(NULL));
		pf.reset(0., 0., 0.);

		printf("Initial particles\n");
		view(pf, map, poses, Pose2d::from_matrix(poses[0]), NULL, NULL);

		for (int i = 1; i < times.size(); i++)
		{
			printf("Step %d of %ld\n", i+1, times.size());
			Pose2d gps = Pose2d::from_matrix(poses[i]); // TODO: add noise

			printf("Prediction\n");
			pf.predict(odom[i].first, odom[i].second, times[i] - times[i-1]);
			view(pf, map, poses, gps, NULL, NULL);

			//if (i % 4 == 0 && i > 0)
			//if (i > 16)
			if (1)
			{
				printf("Correction\n");
				load_fused_pointcloud_and_camera(i, cloud);
				pf.correct(gps, cloud, map, transformed_cloud);

				Pose2d mean = pf.mean();
				Pose2d mode = pf.mode();

				printf("True pose: %.2lf %.2lf %.2lf\n", gps.x, gps.y, radians_to_degrees(gps.th));
				printf("PF Mean: %.2lf %.2lf %.2lf Error: %lf\n", mean.x, mean.y, radians_to_degrees(mean.th),
						sqrt(pow(gps.x - mean.x, 2) + pow(gps.y - mean.y, 2)));
				printf("PF Mode: %.2lf %.2lf %.2lf Error: %lf\n", mode.x, mode.y, radians_to_degrees(mode.th),
						sqrt(pow(gps.x - mode.x, 2) + pow(gps.y - mode.y, 2)));

				view(pf, map, poses, gps, cloud, transformed_cloud);
			}
		}
	}
}


void
load_carmen_data(vector<double> &times,
		vector<Matrix<double, 4, 4>> &poses,
		vector<pair<double, double>> &odom)
{
	FILE *f = fopen("/dados/data_20180112-2/data/poses.txt", "r");

	char dummy[128];
	double x, y, th, t, v, phi;
	double x0, y0;
	int first = 1;

	double ds, lt, dt, odom_x, odom_y, odom_th;

	odom_x = odom_y = odom_th = 0.;

	while (!feof(f))
	{
		fscanf(f, "\n%s %lf %lf %lf %lf %s %lf %lf %s\n",
				dummy, &x, &y, &th, &t, dummy, &v, &phi, dummy);

		if (first)
		{
			x0 = x;
			y0 = y;
			first = 0;
		}
		else
		{
			dt = t - lt;
			ds = v * 1.006842 * dt;
			odom_x += ds * cos(odom_th);
			odom_y += ds * sin(odom_th);
			odom_th += ds * tan(phi * 0.861957 - 0.002372) / distance_between_front_and_rear_axles;
			odom_th = normalize_theta(odom_th);
		}
		lt = t;

		Pose2d pose(odom_x, odom_y, odom_th);
		// Pose2d pose(x - x0, y - y0, normalize_theta(th));

		poses.push_back(Pose2d::to_matrix(pose));
		times.push_back(t);
		odom.push_back(pair<double, double>(v, phi));
	}

	Matrix<double, 4, 4> p0_inv = Matrix<double, 4, 4>(poses[0]).inverse();

	for (int i = 0; i < poses.size(); i++)
	{
		poses[i] = p0_inv * poses[i];
		Pose2d p = Pose2d::from_matrix(poses[i]);
		p.y = -p.y;
		p.th = normalize_theta(-p.th);
		poses[i] = Pose2d::to_matrix(p);
	}

	fclose(f);
}


int
main()
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);

	ParticleFilter pf(50, 0.5, 0.5, degrees_to_radians(10),
			0.2, degrees_to_radians(1),
			0.1, 0.1, degrees_to_radians(2),
			25.0, 25.0, degrees_to_radians(100),
			100., 100., 100.);

	//GridMap map(-20, -2, 100, 200, 0.2);
	system("rm -rf /dados/maps/maps_20180112-2/*");
	GridMap map("/dados/maps/maps_20180112-2/", 75., 75., 0.4, GridMapTile::TYPE_SEMANTIC);

	// KITTI
	/*
	vector<double> times = load_timestamps();
	vector<vector<double>> data = load_oxts(times);
	vector<Matrix<double, 4, 4>> poses = oxts2Mercartor(data);
	vector<pair<double, double>> odom = estimate_v(poses, times); // assumes for simplicity that phi is zero.
	*/

	// Carmen
	vector<double> times;
	vector<Matrix<double, 4, 4>> poses;
	vector<pair<double, double>> odom;
	load_carmen_data(times, poses, odom);

	create_map(map, poses, cloud, transformed_cloud);
	run_particle_filter(pf, map, poses, odom, times, cloud, transformed_cloud);

	printf("Done\n");
	return 0;
}


