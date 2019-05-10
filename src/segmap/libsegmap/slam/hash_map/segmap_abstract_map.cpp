
#include "segmap_abstract_map.h"


GridMapHeader::GridMapHeader(int height, int width, double resolution, double y_origin, double x_origin)
{
	_width_m = width;
	_height_m = height;
	_m_by_pixel = resolution;
	_pixels_by_m = 1.0 / resolution;
	_width = _width_m * _pixels_by_m;
	_height = _height_m * _pixels_by_m;
	_y_origin = y_origin;
	_x_origin = x_origin;
}


GridMapHeader::~GridMapHeader()
{
}


int
GridMapHeader::index_from_coordinate(double coordinate, double origin, double pixels_by_m)
{
	return (coordinate - origin) * pixels_by_m;
}


int
GridMapHeader::contains(const pcl::PointXYZRGB &point)
{
	int cx = index_from_coordinate(point.x, _x_origin, _pixels_by_m);
	int cy = index_from_coordinate(point.y, _y_origin, _pixels_by_m);

	return (cx >= 0) && (cy >= 0) && (cx < _width) && (cy < _height);
}


void
AbstractMap::add(const std::vector<pcl::PointXYZRGB> &points)
{
	for (int i = 0; i < points.size(); i++)
		add(points[i]);
}


void
AbstractMap::add(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
	for (int i = 0; i < point_cloud->size(); i++)
		add(point_cloud->at(i));
}
