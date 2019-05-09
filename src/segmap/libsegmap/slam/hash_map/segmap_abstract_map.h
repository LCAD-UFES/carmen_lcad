
#ifndef __SEGMAP_MAP_INTERFACE_H__
#define __SEGMAP_MAP_INTERFACE_H__

#include <vector>
#include <opencv/cv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class GridMapHeader
{
public:
	GridMapHeader(int height, int width, double resolution, double y_origin, double x_origin);
	~GridMapHeader();

	int index_from_coordinate(double coordinate, double origin, double pixels_by_m);
	int contains(const pcl::PointXYZRGB &point);

	double _pixels_by_m;
	double _m_by_pixel;
	double _x_origin;
	double _y_origin;
	int _width_m;
	int _height_m;
	int _width;
	int _height;
};


class AbstractMap
{
public:
	AbstractMap(const GridMapHeader &header) { _header = new GridMapHeader(header); }
	virtual ~AbstractMap() { delete(_header); }

	void add(const std::vector<pcl::PointXYZRGB> &points);
	void add(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

	virtual void add(const pcl::PointXYZRGB &point) = 0;
	virtual void save(const char *path) = 0;
	virtual void load(const char *path) = 0;

	GridMapHeader *_header;
};


#endif
