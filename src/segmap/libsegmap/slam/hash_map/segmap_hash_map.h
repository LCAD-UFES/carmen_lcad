
#ifndef __LIBSEGMAP_SLAM_SEGMAP_HASH_MAP_H__
#define __LIBSEGMAP_SLAM_SEGMAP_HASH_MAP_H__

#include <map>
#include <vector>
#include <opencv/cv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_colormaps.h>
#include <carmen/util_io.h>

/*

class SemanticCell : public CellInterface
{
public:
	Categorical statistics;

	virtual void update(const pcl::PointXYZRGB &point)
	{
		statistics.update(point.r);
	}

	virtual double likelihood(const pcl::PointXYZRGB &point)
	{
		return statistics.likelihood(point.r);
	}

	cv::Scalar get_color()
	{
		CityScapesColorMap colormap;
		return colormap.color(statistics.most_likely());
	}
};

// Important: this class assumes that the dimensions are independent.
class ColorCell : public CellInterface
{
public:
	Gaussian r_statistics, g_statistics, b_statistics;

	virtual void update(const pcl::PointXYZRGB &point)
	{
		r_statistics.update(point.r);
		g_statistics.update(point.g);
		b_statistics.update(point.b);
	}

	virtual double likelihood(const pcl::PointXYZRGB &point)
	{
		double l = 1.0;

		l *= r_statistics.likelihood(point.r);
		l *= g_statistics.likelihood(point.g);
		l *= b_statistics.likelihood(point.b);

		return l;
	}

	cv::Scalar get_color()
	{
		unsigned char r = (unsigned char) r_statistics.mean;
		unsigned char g = (unsigned char) g_statistics.mean;
		unsigned char b = (unsigned char) b_statistics.mean;
		return cv::Scalar(b, g, r);
	}
};
*/

/**
 * Convention that classes that represent cell values must follow:
 * - In the same header file in which the class is declared, the following functions shall be provided:
 * 			- A function for updating the cell value.
 * 			- A function for generating a color based on the cell value.
 * 			- A function for computing the likelihood of a point.
 * 			- A function for updating the cell value with another cell value for soft measurements.
 * 			- A function for computing the kl-divergence between two cell values for soft measurements.
  */



template<class CellType>
class HashGridMap
{
public:
	CellType cell;

	std::map<int, std::map<int, CellType>> _cells;

	double _pixels_by_m;
	double _m_by_pixel; // m by pixels
	double _x_origin;
	double _y_origin;
	int _width_m;
	int _height_m;
	int _width;
	int _height;

	HashGridMap(int height, int width, double resolution,
							double y_origin, double x_origin);
	~HashGridMap();

	static int _get_index_from_value(double val, double offset, double n_elems_by_val);

	void add(const pcl::PointXYZRGB &point);
	void add(const std::vector<pcl::PointXYZRGB> &points);
	void add(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
	void save(const char *path);
	void load(const char *path);
};


template<class CellType>
HashGridMap<CellType>::HashGridMap(int height, int width, double resolution,
																	 double y_origin, double x_origin)
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


template<class CellType>
HashGridMap<CellType>::~HashGridMap()
{
}


template<class CellType> int
HashGridMap<CellType>::_get_index_from_value(double val, double offset, double n_elems_by_val)
{
	return (val - offset) * n_elems_by_val;
}


template<class CellType> void
HashGridMap<CellType>::add(const pcl::PointXYZRGB &point)
{
	int xc = _get_index_from_value(point.x, _x_origin, _pixels_by_m);
	int yc = _get_index_from_value(point.y, _y_origin, _pixels_by_m);

	// check if point is inside the map
	if (xc < 0 || xc >= _width || yc < 0 || yc >= _height)
		return;

	// To understand the need of "typename" see https://stackoverflow.com/questions/610245/where-and-why-do-i-have-to-put-the-template-and-typename-keywords/613132#613132
	typename std::map<int, std::map<int, CellType>>::iterator col_it;
	typename std::map<int, CellType>::iterator cell_it;

	col_it = _cells.find(yc);

	if (col_it == _cells.end())
		// The insert method returns a pair with the first element being an iterator for the newly
		// inserted item. See http://www.cplusplus.com/reference/map/map/insert/ .
		col_it = _cells.insert(std::pair<int, std::map<int, CellType>>(yc, std::map<int, CellType>())).first;

	cell_it = col_it->second.find(xc);

	if (cell_it == col_it->second.end())
		cell_it = col_it->second.insert(std::pair<int, CellType>(xc, CellType())).first;

	cell_it->second.add(point);
}


template<class CellType> void
HashGridMap<CellType>::add(const std::vector<pcl::PointXYZRGB> &points)
{
	for (int i = 0; i < points.size(); i++)
		add(points[i]);
}


template<class CellType> void
HashGridMap<CellType>::add(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
	for (int i = 0; i < point_cloud->size(); i++)
		add(point_cloud->at(i));
}


template<class CellType> void
HashGridMap<CellType>::save(const char *path)
{
	FILE *fptr = safe_fopen(path, "wb");

	int id;
	long n;

	n = _cells.size();
	fwrite(&n, sizeof(n), 1, fptr);

	typename std::map<int, std::map<int, CellType>>::iterator col_it;
	typename std::map<int, CellType>::iterator cell_it;

	for (col_it = _cells.begin(); col_it != _cells.end(); col_it++)
	{
		id = col_it->first;
		fwrite(&id, sizeof(id), 1, fptr);

		n = col_it->second.size();
		fwrite(&n, sizeof(n), 1, fptr);

		for (cell_it = col_it->second.begin(); cell_it != col_it->second.end(); cell_it++)
		{
			id = cell_it->first;
			fwrite(&id, sizeof(id), 1, fptr);

			cell_it->second.write(fptr);
		}
	}

	fclose(fptr);
}


template<class CellType> void
HashGridMap<CellType>::load(const char *path)
{
	FILE *fptr = fopen(path, "rb");

	if (fptr == NULL)
	{
		fprintf(stderr, "Warning: Unable to load map '%s'\n", path);
		return;
	}

	int id;
	long n, k;

	fread(&n, sizeof(n), 1, fptr);

	typename std::map<int, std::map<int, CellType>>::iterator col_it;
	typename std::map<int, CellType>::iterator cell_it;

	for (int i = 0; i < n; i++)
	{
		fread(&id, sizeof(id), 1, fptr);

		col_it = _cells.insert(std::pair<int, std::map<int, CellType>>(id, std::map<int, CellType>())).first;

		fread(&k, sizeof(k), 1, fptr);

		for (int j = 0; j < k; j++)
		{
			fread(&id, sizeof(id), 1, fptr);
			cell_it = col_it->second.insert(std::pair<int, CellType>(id, CellType())).first;
			cell_it->second.read(fptr);
		}
	}

	fclose(fptr);
}


#endif
