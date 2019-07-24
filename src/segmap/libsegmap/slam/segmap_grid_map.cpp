
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>
#include <carmen/util_time.h>
#include <carmen/util_math.h>

#include <boost/filesystem.hpp>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>
#include <carmen/segmap_particle_filter_viewer.h>

using namespace pcl;
using namespace std;
using namespace cv;


void
GridMapTile::_initialize_map()
{
	// assumes the origin is set.
	char name[256];

	sprintf(name, "%s/%s_%lf_%lf.map",
	        _tiles_dir.c_str(),
	        GridMapTile::type2str(_map_type),
	        _xo, _yo);

	_map = (double *) calloc (_h * _w * _n_fields_by_cell, sizeof(double));
	_observed_cells.clear();

	struct stat buffer;

	if (stat(name, &buffer) == 0)
	{
		FILE *fptr = fopen(name, "rb");
		fread(_map, sizeof(double), _h * _w * _n_fields_by_cell, fptr);
		fclose(fptr);

		for (int i = 0; i < _h; i++)
		{
			for (int j = 0; j < _w; j++)
			{
				int p = _n_fields_by_cell * (i * _w + j);
				double *cell = &(_map[p]);

				if (
					((_map_type == GridMapTile::TYPE_SEMANTIC) && (cell[_unknown.size() - 1] > 0.0)) ||
					((_map_type == GridMapTile::TYPE_VISUAL) && (cell[3] > 1.0)) ||
					((_map_type == GridMapTile::TYPE_OCCUPANCY) && (cell[2] > 2.0)) ||
					((_map_type == GridMapTile::TYPE_REFLECTIVITY) && (cell[2] > 1.0)) 
					)
					_observed_cells.insert(p);
			}
		}
	}
	else
	{
		for (int i = 0; i < _h; i++)
			for (int j = 0; j < _w; j++)
				for (int k = 0; k < _n_fields_by_cell; k++)
					_map[_n_fields_by_cell * (i * _w + j) + k] = _unknown[k];
	}
}


void
GridMapTile::_initialize_derivated_values()
{
	_pixels_by_m = 1 / _m_by_pixel;
	_h = (int) (_pixels_by_m * _hm);
	_w = (int) (_pixels_by_m * _wm);

	if (_map_type == GridMapTile::TYPE_SEMANTIC)
	{
		// The first '_N_CLASSES' fields contains the count for
		// each class, and the last two fields are the total
		// number of rays that hit the cell, and a boolean informing
		// if the cell was observed. All classes are initialized with
		// a count of 1 to prevent probabilities equal to zero when
		// computing particle weights.
		_n_fields_by_cell = _color_map.n_classes + 2;
		_unknown = vector<double>(_n_fields_by_cell, 1);
		_unknown[_unknown.size() - 2] = _color_map.n_classes;
		_unknown[_unknown.size() - 1] = 0;
	}
	else if (_map_type == GridMapTile::TYPE_VISUAL)
	{
		// r, g, b
		_n_fields_by_cell = 4;
		_unknown = vector<double>(_n_fields_by_cell, 128.);
		_unknown[3] = 1.;
	}
	else if (_map_type == GridMapTile::TYPE_REFLECTIVITY)
	{
		_n_fields_by_cell = 2;
		_unknown = vector<double>(_n_fields_by_cell, 128.);
		_unknown[1] = 1.;
	}
	else if (_map_type == GridMapTile::TYPE_OCCUPANCY)
	{
		_n_fields_by_cell = 2;
		_unknown = vector<double>(_n_fields_by_cell, 0.0);
		// initialize cells with two readings with one occupied measurement.
		_unknown[0] = 1;
		_unknown[1] = 2;
	}
	else
		exit(printf("Map type '%d' not found.\n", (int) _map_type));
}


GridMapTile::GridMapTile(double point_y, double point_x,
                         double height_meters, double width_meters,
                         double resolution, GridMapTile::MapType map_type, string tiles_dir,
                         int save_maps)
{
	// map tile origin
	_xo = floor(point_x / width_meters) * width_meters;
	_yo = floor(point_y / height_meters) * height_meters;
	_hm = height_meters;
	_wm = width_meters;
	_m_by_pixel = resolution;
	_map_type = map_type;
	_tiles_dir = tiles_dir;
	_save_maps = save_maps;

	//printf("Creating tile with origin: %lf %lf\n", _xo, _yo);
	_initialize_derivated_values();
	_initialize_map();
}


GridMapTile::~GridMapTile()
{
	if (_save_maps)
		save();

	free(_map);
}


const char*
GridMapTile::type2str(MapType map_type)
{
	if (map_type == TYPE_SEMANTIC)
		return "semantic";
	else if (map_type == TYPE_VISUAL)
		return "visual";
	else if (map_type == TYPE_OCCUPANCY)
		return "occupancy";
	else if (map_type == TYPE_REFLECTIVITY)
		return "reflectivity";
	else
		exit(printf("Map type '%d' not found.\n", map_type));
}


void
GridMapTile::save()
{
	char name[256];
	sprintf(name, "%s/%s_%lf_%lf.config",
	        _tiles_dir.c_str(),
	        GridMapTile::type2str(_map_type),
	        _xo, _yo);

	FILE *fptr = fopen(name, "w");

	if (fptr == NULL)
		exit(printf("Error: Unable to create the file '%s'\n", name));

	fprintf(fptr, "xo: %lf\n", _xo);
	fprintf(fptr, "yo: %lf\n", _yo);
	fprintf(fptr, "hm: %lf\n", _hm);
	fprintf(fptr, "wm: %lf\n", _wm);
	fprintf(fptr, "m_by_pixel: %lf\n", _m_by_pixel);
	fprintf(fptr, "map_type: %d\n", (int) _map_type);

	fclose(fptr);

	sprintf(name, "%s/%s_%lf_%lf.map",
	        _tiles_dir.c_str(),
	        GridMapTile::type2str(_map_type),
	        _xo, _yo);

	fptr = fopen(name, "wb");
	fwrite(_map, sizeof(double), _h * _w * _n_fields_by_cell, fptr);
	fclose(fptr);

	// only for debugging
	sprintf(name, "%s/%s_%lf_%lf.png",
	        _tiles_dir.c_str(),
	        GridMapTile::type2str(_map_type),
	        _xo, _yo);

	imwrite(name, to_image());
}


void
GridMapTile::add_point(PointXYZRGB &p)
{
	int px, py, pos;

	px = (p.x - _xo) * _pixels_by_m;
	py = (p.y - _yo) * _pixels_by_m;

	if (px >= 0 && px < _w && py >= 0 && py < _h)
	{
		pos = _n_fields_by_cell * (py * _w + px);

		if (_map_type == TYPE_VISUAL)
		{
			double weight;
			weight = 1. / (double) _map[pos + 3];

			_map[pos + 0] = _map[pos + 0] * (1. - weight) + p.b * weight;
			_map[pos + 1] = _map[pos + 1] * (1. - weight) + p.g * weight;
			_map[pos + 2] = _map[pos + 2] * (1. - weight) + p.r * weight;
			_map[pos + 3] += 1;
		}
		else if (_map_type == TYPE_REFLECTIVITY)
		{
			double weight;
			weight = 1. / (double) _map[pos + 1];

			_map[pos + 0] = _map[pos + 0] * (1. - weight) + p.r * weight;
			_map[pos + 1] += 1;
		}
		else if (_map_type == TYPE_SEMANTIC)
		{
			// invalid class 
			if (p.r == 19)
				return;
		
			// increment the class count
			_map[pos + p.r]++;
			// increment the number of rays that hit the cell
			_map[pos + (_n_fields_by_cell - 2)]++;
			// set the cell as observed.
			_map[pos + (_n_fields_by_cell - 1)] = 1;
		}
		else if (_map_type == TYPE_OCCUPANCY)
		{
			if (p.r > 0.5)
			{
				// when we observe an obstacle we count it 5 times to 
				// make it hard to be erased.
				_map[pos] += 5;
				_map[pos + 1] += 5;
			}
			else
				_map[pos + 1]++;
		}
		else
			exit(printf("Error: map_type '%d' not defined.\n", (int) _map_type));

		// we have to add the position to the set in the end because of the return in the SEMANTIC update.
		_observed_cells.insert(pos);
	}
}


bool
GridMapTile::contains(double x, double y)
{
	if (x < _xo || x >= (_xo + _wm) || y < _yo || y >= (_yo + _hm))
		return false;

	return true;
}


vector<double>
GridMapTile::read_cell(PointXYZRGB &p)
{
	return read_cell(p.x, p.y);
}


vector<double>
GridMapTile::read_cell(double x_world, double y_world)
{
	int px, py, pos;

	px = (x_world - _xo) * _pixels_by_m;
	py = (y_world - _yo) * _pixels_by_m;

	static vector<double> v(_unknown);

	if (px >= 0 && px < _w && py >= 0 && py < _h)
	{
		pos = _n_fields_by_cell * (py * _w + px);

		for (int k = 0; k < _n_fields_by_cell; k++)
			//v.push_back(_map[pos + k]);
			v[k] = _map[pos + k];

		return v;
	}
	else
	{
		//printf("Warning: reading a cell outside the current map. Returning unknown vector.\n");
		return _unknown;
	}
}


double*
GridMapTile::read_cell_ref(double x_world, double y_world)
{
	int px, py, pos;

	px = (x_world - _xo) * _pixels_by_m;
	py = (y_world - _yo) * _pixels_by_m;

	if (px >= 0 && px < _w && py >= 0 && py < _h)
	{
		pos = _n_fields_by_cell * (py * _w + px);
		return (_map + pos);
	}
	else
		return 0;
}


Scalar
GridMapTile::cell2color(double *cell_vals)
{
	Scalar color;
	int cell_was_observed = 0;

	if (_map_type == TYPE_VISUAL)
	{
		if (cell_vals[_n_fields_by_cell - 1] > 1.0)
		{
			color[0] = (unsigned char) cell_vals[0];
			color[1] = (unsigned char) cell_vals[1];
			color[2] = (unsigned char) cell_vals[2];
			cell_was_observed = 1;
		}
	}
	else if (_map_type == TYPE_REFLECTIVITY)
	{
		if (cell_vals[1] > 1.0)
		{
			color[0] = color[1] = color[2] = cell_vals[0];
			cell_was_observed = 1;
		}
	}
	else if (_map_type == TYPE_SEMANTIC)
	{
		if (cell_vals[_n_fields_by_cell - 1])
		{
			Scalar rgb = _color_map.color(argmax(cell_vals, _n_fields_by_cell - 2));
			color = Scalar(rgb[2], rgb[1], rgb[0]);
			cell_was_observed = 1;
		}
	}
	else if (_map_type == TYPE_OCCUPANCY)
	{
		if (cell_vals[_n_fields_by_cell - 1] > 2.0)
		{
			color[0] = color[1] = color[2] = (unsigned char) (255 * (1.0 - cell_vals[0] / cell_vals[1]));
			cell_was_observed = 1;
		}
	}
	else
		exit(printf("Error: map_type '%d' not defined.\n", (int) _map_type));

	if (!cell_was_observed)
	{
		color[0] = 255;
		color[1] = 0;
		color[2] = 0;
	}

	return color;
}


Mat
GridMapTile::to_image()
{
	int p;
	double *cell_vals;
	Scalar color;

	Mat m(_h, _w, CV_8UC3);

	for (int i = 0; i < _h; i++)
	{
		for (int j = 0; j < _w; j++)
		{
			cell_vals = &(_map[_n_fields_by_cell * (i * _w + j)]);
			color = cell2color(cell_vals);
			p = 3 * (i * m.cols + j);

			m.data[p] = color[0];
			m.data[p + 1] = color[1];
			m.data[p + 2] = color[2];
		}
	}

	return m;
}


GridMap::GridMap(string tiles_dir, double tile_height_meters,
								 double tile_width_meters, double resolution,
								 GridMapTile::MapType map_type, int save_maps)
{
	_tiles_dir = tiles_dir;
	_tile_height_meters = tile_height_meters;
	_tile_width_meters = tile_width_meters;
	m_by_pixels = resolution;
	pixels_by_m = 1. / resolution;
	_map_type = map_type;
	_save_maps = save_maps;

	if (save_maps && !boost::filesystem::exists(tiles_dir))
		boost::filesystem::create_directory(tiles_dir);

	for (int i = 0; i < _N_TILES; i++)
		for (int j = 0; j < _N_TILES; j++)
			_tiles[i][j] = NULL;

	_middle_tile = (int) (_N_TILES / 2.);

	height_meters = _tile_height_meters * _N_TILES;
	width_meters = _tile_width_meters * _N_TILES;
	xo = yo = 0;

	_map_initialized = 0;
}


GridMap::~GridMap()
{
	if (_save_maps)
		save();

	_free_tiles();
}


void
GridMap::_free_tiles()
{
	for (int i = 0; i < _N_TILES; i++)
		for (int j = 0; j < _N_TILES; j++)
			if (_tiles[i][j] != NULL)
				delete(_tiles[i][j]);
}


GridMapTile*
GridMap::_reload_tile(double x, double y)
{
	return new GridMapTile(y, x, _tile_height_meters,
	                       _tile_width_meters, m_by_pixels,
												 _map_type, _tiles_dir, _save_maps);
}


void
GridMap::_reload_tiles(double robot_x, double robot_y)
{
	_free_tiles();

	// TODO: make the following code general.
	_tiles[0][0] = _reload_tile(robot_x - _tile_width_meters, robot_y - _tile_height_meters);
	_tiles[0][1] = _reload_tile(robot_x, robot_y - _tile_height_meters);
	_tiles[0][2] = _reload_tile(robot_x + _tile_width_meters, robot_y - _tile_height_meters);
	_tiles[1][0] = _reload_tile(robot_x - _tile_width_meters, robot_y);
	_tiles[1][1] = _reload_tile(robot_x, robot_y);
	_tiles[1][2] = _reload_tile(robot_x + _tile_width_meters, robot_y);
	_tiles[2][0] = _reload_tile(robot_x - _tile_width_meters, robot_y + _tile_height_meters);
	_tiles[2][1] = _reload_tile(robot_x, robot_y + _tile_height_meters);
	_tiles[2][2] = _reload_tile(robot_x + _tile_width_meters, robot_y + _tile_height_meters);

	xo = _tiles[0][0]->_xo;
	yo = _tiles[0][0]->_yo;
}


void
GridMap::reload(double robot_x, double robot_y)
{
	if (_tiles[_middle_tile][_middle_tile] == NULL)
		_reload_tiles(robot_x, robot_y);

	if (!_tiles[_middle_tile][_middle_tile]->contains(robot_x, robot_y))
		_reload_tiles(robot_x, robot_y);
	
	_map_initialized = 1;
}


void
GridMap::_check_if_map_was_initialized()
{
	if (!_map_initialized)
		exit(printf("Error: initialize the map using the method reload before using it.\n"));
}


void
GridMap::add_point(PointXYZRGB &p)
{
	_check_if_map_was_initialized();

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


double
compute_expected_delta_ray(double h, double r1, double theta)
{
	double a = r1 * sin(acos(h / r1));
	double alpha = asin(a / r1);
	double expected_delta_ray = ((sin(alpha + theta) * h) / sin(M_PI / 2.0 + alpha + theta)) - a;
	return (expected_delta_ray);
}

 
double
compute_obstacle_evidence(SensorPreproc::CompletePointData &prev, SensorPreproc::CompletePointData &curr)
{
	// velodyne.z + sensor_board.z + (wheel_radius / 2.0)
	// TODO: read it from param file.
	const double sensor_height = 2.152193;

	double measured_dray_floor = sqrt(pow(curr.car.x, 2) + pow(curr.car.y, 2)) - sqrt(pow(prev.car.x, 2) + pow(prev.car.y, 2));
	double diff_vert_angle = normalize_theta(curr.v_angle - prev.v_angle);
	double expected_dray_floor = compute_expected_delta_ray(sensor_height, prev.range, diff_vert_angle);
	double obstacle_evidence = (expected_dray_floor - measured_dray_floor) / expected_dray_floor;

	// if the obstacle evidence is higher than 0.4, the point is 
	// classified as obstacle (return 1), else it is classified as free (return 0).
	if (abs(obstacle_evidence) > 0.4) // valor ad-hoc
		return 1;
	else
		return 0;

	// Testa se tem um obstaculo com um buraco embaixo
	// ??? Alberto's powerful black magic...
	/* 
	double sigma = 0.45;
	obstacle_evidence = (obstacle_evidence > 1.0)? 1.0: obstacle_evidence;
	double p0 = exp(-1.0 / sigma);
	double p_obstacle = (exp(-pow(1.0 - obstacle_evidence, 2) / sigma) - p0) / (1.0 - p0);
	
	if (p_obstacle > 1.0) p_obstacle = 1.0;
	else if (p_obstacle < 0.0) p_obstacle = 0.0;
	
	return p_obstacle;
	*/
}


void
GridMap::add_occupancy_shot(std::vector<SensorPreproc::CompletePointData> &points, int do_raycast,
							int use_world_ref)
{
	_check_if_map_was_initialized();

	assert(points.size() == 32);

	int first_valid = -1;
	int last_valid = -1;
	int nearest_obstacle = -1;

	double obstacle_prob;
	std::vector<double> obstacle_probs;
	PointXYZRGB point;

	if (points[0].valid)
	{
		first_valid = 0;
		last_valid = 0;
	}

	for (int i = 1; i < points.size(); i++)
	{
		if (points[i].valid)
		{
			if (first_valid == -1)
				first_valid = i;
			
			if (i > last_valid)
				last_valid = i;
		}

		if (points[i - 1].valid && points[i].valid)
		{
			obstacle_prob = compute_obstacle_evidence(points[i - 1], points[i]);
			obstacle_probs.push_back(obstacle_prob);

			if (obstacle_prob > 0.5 && nearest_obstacle == -1)
				nearest_obstacle = i;

			if (use_world_ref)
				point = PointXYZRGB(points[i].world);
			else
				point = PointXYZRGB(points[i].car);

			point.r = point.g = point.b = obstacle_prob;
			add_point(point);
		}
	}

	// raycast until the nearest cell that hit an obstacle
	if (do_raycast && (first_valid >= 0) && (first_valid != last_valid))
	{
		int update_until_range_max = 0;

		// if all points were classified as free.
		if (nearest_obstacle == -1)
		{
			nearest_obstacle = last_valid;
			update_until_range_max = 1;
		}

		double dx, dy;

		if (use_world_ref)
		{
			dx = points[nearest_obstacle].world.x - points[first_valid].world.x;
			dy = points[nearest_obstacle].world.y - points[first_valid].world.y;
		}
		else
		{
			dx = points[nearest_obstacle].car.x - points[first_valid].car.x;
			dy = points[nearest_obstacle].car.y - points[first_valid].car.y;
		}

		// the -2 is to prevent raycasting over the cell that contains an obstacle
		double dx_cells = dx * pixels_by_m - 2;
		double dy_cells = dy * pixels_by_m - 2;
		double n_cells_in_line = sqrt(pow(dx_cells, 2) + pow(dy_cells, 2));

		//printf("dx: %lf dy: %lf dxc: %lf dxy: %lf nc: %lf\n", dx, dy, dx_cells, dy_cells, n_cells_in_line);

		dx /= n_cells_in_line;
		dy /= n_cells_in_line;

		//printf("after dx: %lf dy: %lf\n", dx, dy);

		// if all points were classified as free, we 
		// in increase the number of visited cells until MAX_RANGE.
		if (update_until_range_max)
		{
			double d = sqrt(pow(points[nearest_obstacle].sensor.x, 2) + 
							pow(points[nearest_obstacle].sensor.y, 2));
							
			n_cells_in_line *= (MAX_RANGE / d);
		}

		// the -2 is to prevent raycasting over the cell that contains an obstacle.
		for (int i = 0; i < (n_cells_in_line - 2); i++)
		{
			if (use_world_ref)
			{
				point.x = points[first_valid].world.x + dx * i;
				point.y = points[first_valid].world.y + dy * i;
			}
			else
			{
				point.x = points[first_valid].car.x + dx * i;
				point.y = points[first_valid].car.y + dy * i;
			}

			// set the point as free
			point.r = point.g = point.b = 0;
			add_point(point);
		}
	}
}


vector<double>
GridMap::read_cell(PointXYZRGB &p)
{
	_check_if_map_was_initialized();
	return read_cell(p.x, p.y);
}


vector<double>
GridMap::read_cell(double x_world, double y_world)
{
	_check_if_map_was_initialized();

	int i, j;

	for (i = 0; i < _N_TILES; i++)
	{
		for (j = 0; j < _N_TILES; j++)
		{
			if (_tiles[i][j]->contains(x_world, y_world))
				return _tiles[i][j]->read_cell(x_world, y_world);
		}
	}

	//printf("Warning: trying to read a cell that is not in the current map tiles! Returning unknown.\n");
	return _tiles[0][0]->_unknown;
}


double *
GridMap::read_cell_ref(double x_world, double y_world)
{
	_check_if_map_was_initialized();

	int i, j;

	for (i = 0; i < _N_TILES; i++)
	{
		for (j = 0; j < _N_TILES; j++)
		{
			if (_tiles[i][j]->contains(x_world, y_world))
				return _tiles[i][j]->read_cell_ref(x_world, y_world);
		}
	}

	return 0;
}


Mat
GridMap::to_image()
{
	_check_if_map_was_initialized();

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

	//line(viewer, Point(w, 0), Point(w, 3*h), Scalar(0,0,0), 1);
	//line(viewer, Point(2*w, 0), Point(2*w, 3*h), Scalar(0,0,0), 1);
	//line(viewer, Point(0, h), Point(3*w, h), Scalar(0,0,0), 1);
	//line(viewer, Point(0, 2*h), Point(3*w, 2*h), Scalar(0,0,0), 1);

	return viewer;
}


void
GridMap::save()
{
	for (int i = 0; i < _N_TILES; i++)
	{
		for (int j = 0; j < _N_TILES; j++)
		{
			if (_tiles[i][j] != NULL)
				_tiles[i][j]->save();
		}
	}
}


void
update_maps(DataSample *sample, SensorPreproc &preproc, GridMap *visual_map, GridMap *reflectivity_map, GridMap *semantic_map, GridMap *occupancy_map)
{
	PointXYZRGB point;
	std::vector<SensorPreproc::CompletePointData> points;
	preproc.reinitialize(sample);

	for (int i = 0; i < preproc.size(); i++)
	{
		points = preproc.next_points_with_full_information();

		if (occupancy_map)
			occupancy_map->add_occupancy_shot(points);

		for (int j = 0; j < points.size(); j++)
		{
			if (points[j].valid)
			{
				if (visual_map && points[j].visible_by_cam)
				{
					point = points[j].world;
					point.r = points[j].colour[2];
					point.g = points[j].colour[1];
					point.b = points[j].colour[0];
					visual_map->add_point(point);
				}

				if (semantic_map && points[j].visible_by_cam)
				{
					point = points[j].world;
					point.r = point.g = point.b = points[j].semantic_class;
					semantic_map->add_point(point);
				}

				if (reflectivity_map)
				{
					point = points[j].world;
					point.r = point.g = point.b = points[j].calibrated_intensity;
					reflectivity_map->add_point(point);
				}
			}
		}
	}
}


void 
create_instantaneous_map(DataSample *sample, SensorPreproc &preproc, GridMap *inst_map, int do_raycast)
{
	PointXYZRGB point;
	std::vector<SensorPreproc::CompletePointData> points;
	preproc.reinitialize(sample);

	for (int i = 0; i < preproc.size(); i++)
	{
		points = preproc.next_points_with_full_information();

		if (inst_map->_map_type == GridMapTile::TYPE_OCCUPANCY)
			inst_map->add_occupancy_shot(points, do_raycast, 0);

		for (int j = 0; j < points.size(); j++)
		{
			if (points[j].valid)
			{
				if (inst_map->_map_type == GridMapTile::TYPE_VISUAL && points[j].visible_by_cam)
				{
					point = points[j].car;
					point.r = points[j].colour[2];
					point.g = points[j].colour[1];
					point.b = points[j].colour[0];
					inst_map->add_point(point);
				}

				if (inst_map->_map_type == GridMapTile::TYPE_SEMANTIC && points[j].visible_by_cam)
				{
					point = points[j].car;
					point.r = point.g = point.b = points[j].semantic_class;
					inst_map->add_point(point);
				}

				if (inst_map->_map_type == GridMapTile::TYPE_REFLECTIVITY)
				{
					point = points[j].car;
					point.r = point.g = point.b = points[j].calibrated_intensity;
					inst_map->add_point(point);
				}
			}
		}
	}
}