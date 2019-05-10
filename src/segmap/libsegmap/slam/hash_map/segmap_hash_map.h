
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
#include "segmap_abstract_map.h"

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
class HashGridMap : public AbstractMap
{
public:
	HashGridMap(const GridMapHeader &header);
	~HashGridMap();

	// inherit the overloads for the add method from the superclass
	// see https://stackoverflow.com/questions/5636289/overloaded-method-not-seen-in-subclass
	using AbstractMap::add;

	virtual void add(const pcl::PointXYZRGB &point);
	virtual void save(const char *path);
	virtual void load(const char *path);
	void iterate(void function_to_iterate(int row, int col, CellType &cell, void *data), void *data);

	std::map<int, std::map<int, CellType>> _cells;
};


template<class CellType>
HashGridMap<CellType>::HashGridMap(const GridMapHeader &header) : AbstractMap(header)
{
}


template<class CellType>
HashGridMap<CellType>::~HashGridMap()
{
}


template<class CellType> void
HashGridMap<CellType>::add(const pcl::PointXYZRGB &point)
{
	// point is outside the map
	if (!_header->contains(point))
		return;

	int xc = _header->index_from_coordinate(point.x, _header->_x_origin, _header->_pixels_by_m);
	int yc = _header->index_from_coordinate(point.y, _header->_y_origin, _header->_pixels_by_m);

	// To understand why we have to use "typename" here see:
	// https://stackoverflow.com/questions/610245/where-and-why-do-i-have-to-put-the-template-and-typename-keywords/613132#613132
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


template<class CellType> void
HashGridMap<CellType>::iterate(void function_to_iterate(int row, int col, CellType &cell, void *data), void *data)
{
	int row, col;

	typename std::map<int, std::map<int, CellType>>::iterator col_it;
	typename std::map<int, CellType>::iterator cell_it;

	for (col_it = _cells.begin(); col_it != _cells.end(); col_it++)
	{
		row = col_it->first;

		for (cell_it = col_it->second.begin(); cell_it != col_it->second.end(); cell_it++)
		{
			col = cell_it->first;
			function_to_iterate(row, col, cell_it->second, data);
		}
	}
}


#endif
