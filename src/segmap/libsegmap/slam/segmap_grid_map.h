
#ifndef _SEGMAP_GRID_MAP_H_
#define _SEGMAP_GRID_MAP_H_


#include <string>
#include <vector>

#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <carmen/segmap_dataset.h>
#include <carmen/segmap_colormaps.h>
#include <carmen/segmap_preproc.h>


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

	std::vector<double> _unknown;
	int _n_fields_by_cell;
	int _map_type;
	std::string _tiles_dir;
	int _save_maps;

	void _initialize_map();
	void _initialize_derivated_values();
	cv::Scalar cell2color(double *cell_vals);

	GridMapTile(double point_y, double point_x,
			double height_meters, double width_meters,
			double resolution, int map_type, std::string tiles_dir,
			int save_maps=0);

	~GridMapTile();

	static const char* type2str(int map_type);

	void save();
	void add_point(pcl::PointXYZRGB &p);
	bool contains(double x, double y);
	std::vector<double> read_cell(pcl::PointXYZRGB &p);
	cv::Mat to_image();
};


class GridMap
{
public:
	static const int _N_TILES = 3;

	std::string _tiles_dir;
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

	GridMap(std::string tiles_dir, double tile_height_meters, double tile_width_meters, double resolution, int map_type, int save_maps=0);
	~GridMap();
	GridMapTile* _reload_tile(double x, double y);
	void _reload_tiles(double robot_x, double robot_y);
	void reload(double robot_x, double robot_y);
	void add_point(pcl::PointXYZRGB &p);
	std::vector<double> read_cell(pcl::PointXYZRGB &p);
	cv::Mat to_image();
	void save();

	void _free_tiles();
};

// utility function for updating the map with a point cloud.
void update_map(DataSample *sample, GridMap *map, SensorPreproc &preproc);

void
create_map(GridMap &map, NewCarmenDataset *dataset, int step,
					 SensorPreproc &preproc, double skip_velocity_threshold,
					 int view_flag, int img_width);

#endif
