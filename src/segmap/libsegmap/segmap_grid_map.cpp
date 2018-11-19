
#include "segmap_grid_map.h"
#include <sys/stat.h>


void
GridMapTile::_initialize_map()
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
GridMapTile::_initialize_derivated_values()
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


GridMapTile::GridMapTile(double point_y, double point_x,
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


GridMapTile::~GridMapTile()
{
	save();
	delete(_map);
}


const char*
GridMapTile::type2str(int map_type)
{
	if (map_type == TYPE_SEMANTIC)
		return "semantic";
	else if (map_type == TYPE_VISUAL)
		return "visual";
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
GridMapTile::add_point(PointXYZRGB &p)
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
	int px, py, pos;

	px = (p.x - _xo) * _pixels_by_m;
	py = (p.y - _yo) * _pixels_by_m;

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
GridMapTile::to_image()
{
	return _map->clone();
}


GridMap::GridMap(string tiles_dir, double height_meters, double width_meters, double resolution,
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
GridMap::_reload_tile(double x, double y)
{
	return new GridMapTile(y, x, _height_meters,
			_width_meters, _resolution, _map_type, _tiles_dir);
}


void
GridMap::_reload_tiles(double robot_x, double robot_y)
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
GridMap::reload(double robot_x, double robot_y)
{
	if (_tiles[_middle_tile][_middle_tile] == NULL)
		_reload_tiles(robot_x, robot_y);

	if (!_tiles[_middle_tile][_middle_tile]->contains(robot_x, robot_y))
		_reload_tiles(robot_x, robot_y);
}


void
GridMap::add_point(PointXYZRGB &p)
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
GridMap::read_cell(PointXYZRGB &p)
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

	printf("Warning: trying to read a cell that is not in the current map tiles!\n");
	return vector<double>();
}


Mat
GridMap::to_image()
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

	line(viewer, Point(w, 0), Point(w, 3*h), Scalar(0,0,0), 1);
	line(viewer, Point(2*w, 0), Point(2*w, 3*h), Scalar(0,0,0), 1);
	line(viewer, Point(0, h), Point(3*w, h), Scalar(0,0,0), 1);
	line(viewer, Point(0, 2*h), Point(3*w, 2*h), Scalar(0,0,0), 1);

	return viewer;
}


