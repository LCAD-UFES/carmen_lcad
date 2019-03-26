
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <opencv/cv.h>
#include "segmap_grid_map.h"
#include "segmap_util.h"
#include <sys/stat.h>


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

	struct stat buffer;

	if (stat(name, &buffer) == 0)
	{
		FILE *fptr = fopen(name, "rb");
		fread(_map, sizeof(double), _h * _w * _n_fields_by_cell, fptr);
		fclose(fptr);
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
	else
		exit(printf("Map type '%d' not found.\n", _map_type));
}


GridMapTile::GridMapTile(double point_y, double point_x,
                         double height_meters, double width_meters,
                         double resolution, int map_type, string tiles_dir,
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
		exit(printf("Error: Unable to create the file '%s'\n", name));

	fprintf(fptr, "xo: %lf\n", _xo);
	fprintf(fptr, "yo: %lf\n", _yo);
	fprintf(fptr, "hm: %lf\n", _hm);
	fprintf(fptr, "wm: %lf\n", _wm);
	fprintf(fptr, "m_by_pixel: %lf\n", _m_by_pixel);
	fprintf(fptr, "map_type: %d\n", _map_type);

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
		else if (_map_type == TYPE_SEMANTIC)
		{
			// increment the class count
			_map[pos + p.r]++;
			// increment the number of rays that hit the cell
			_map[pos + (_n_fields_by_cell - 2)]++;
			// set the cell as observed.
			_map[pos + (_n_fields_by_cell - 1)] = 1;
		}
		else
			exit(printf("Error: map_type '%d' not defined.\n", _map_type));
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


Scalar
GridMapTile::cell2color(double *cell_vals)
{
	Scalar color;

	if (_map_type == TYPE_VISUAL)
	{
		color[0] = (unsigned char) cell_vals[0];
		color[1] = (unsigned char) cell_vals[1];
		color[2] = (unsigned char) cell_vals[2];
	}
	else if (_map_type == TYPE_SEMANTIC)
	{
		if (cell_vals[_n_fields_by_cell - 1])
		{
			Scalar rgb = _color_map.color(argmax(cell_vals, _n_fields_by_cell - 2));
			Scalar bgr(rgb[2], rgb[1], rgb[0]);
			return bgr;
		}
		else
		{
			color[0] = 128;
			color[1] = 128;
			color[2] = 128;
		}
	}
	else
		exit(printf("Error: map_type '%d' not defined.\n", _map_type));

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


GridMap::GridMap(string tiles_dir, double tile_height_meters, double tile_width_meters, double resolution, int map_type, int save_maps)
{
	_tiles_dir = tiles_dir;
	_tile_height_meters = tile_height_meters;
	_tile_width_meters = tile_width_meters;
	m_by_pixels = resolution;
	pixels_by_m = 1. / resolution;
	_map_type = map_type;
	_save_maps = save_maps;

	for (int i = 0; i < _N_TILES; i++)
		for (int j = 0; j < _N_TILES; j++)
			_tiles[i][j] = NULL;

	_middle_tile = (int) (_N_TILES / 2.);

	height_meters = _tile_height_meters * _N_TILES;
	width_meters = _tile_width_meters * _N_TILES;
	xo = yo = 0;
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
	                       _tile_width_meters, m_by_pixels, _map_type, _tiles_dir, _save_maps);
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

	//printf("Warning: trying to read a cell that is not in the current map tiles! Returning unknown.\n");
	return _tiles[0][0]->_unknown;
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

