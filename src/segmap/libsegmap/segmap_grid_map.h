
#ifndef _SEGMAP_GRID_MAP_H_
#define _SEGMAP_GRID_MAP_H_


#include <string>
#include <vector>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include "segmap_util.h"


using namespace pcl;
using namespace cv;
using namespace std;
using namespace Eigen;


class GridMapTile
{
public:
	// make these attributes private
	// they are used in the view function
	double _hm, _wm, _xo, _yo;
	double _m_by_pixel;
	double _pixels_by_m;
	int _h, _w;
	double *_map;
	CityScapesColorMap _color_map;

	static const int TYPE_SEMANTIC = 0;
	static const int TYPE_VISUAL = 1;

	vector<double> _unknown;
	int _n_fields_by_cell;
	int _map_type;
	string _tiles_dir;
	int _save_maps;

	void _initialize_map();
	void _initialize_derivated_values();
	Scalar cell2color(double *cell_vals);

	GridMapTile(double point_y, double point_x,
			double height_meters, double width_meters,
			double resolution, int map_type, string tiles_dir,
			int save_maps=0);

	~GridMapTile();

	static const char* type2str(int map_type);

	void save();
	void add_point(PointXYZRGB &p);
	bool contains(double x, double y);
	vector<double> read_cell(PointXYZRGB &p);
	Mat to_image();
};


class GridMap
{
public:
	static const int _N_TILES = 3;

	string _tiles_dir;
	double _tile_height_meters;
	double _tile_width_meters;
	int _middle_tile;
	int _map_type;
	int _save_maps;

	double m_by_pixels;
	double pixels_by_m;
	double height_meters;
	double width_meters;
	int xo, yo;

	GridMapTile *_tiles[_N_TILES][_N_TILES];

	GridMap(string tiles_dir, double tile_height_meters, double tile_width_meters, double resolution, int map_type, int save_maps=0);
	~GridMap();
	GridMapTile* _reload_tile(double x, double y);
	void _reload_tiles(double robot_x, double robot_y);
	void reload(double robot_x, double robot_y);
	void add_point(PointXYZRGB &p);
	vector<double> read_cell(PointXYZRGB &p);
	Mat to_image();
	void save();

	void _free_tiles();
};


#endif
