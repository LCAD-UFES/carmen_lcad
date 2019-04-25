
#ifndef __LIBSEGMAP_SLAM_SEGMAP_HASH_MAP_H__
#define __LIBSEGMAP_SLAM_SEGMAP_HASH_MAP_H__

#include <map>
#include <vector>
#include <opencv/cv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_colormaps.h>


template<class T>
class DistributionInterface
{
public:
	virtual ~DistributionInterface() { }
	virtual void update(const T &sample) = 0;
	virtual double likelihood(const T &sample) = 0;

	// soft update
	// prior
	// kl-divergence
};

// virtual void color(unsigned char *r, unsigned char *g, unsigned char *b) = 0;

class Bernoulli : public DistributionInterface<bool>
{
public:
	double n_true, n_total;

	Bernoulli()
	{
		n_true = n_total = 0.0;
	}

	virtual void update(const bool &sample)
	{
		if (sample)
			n_true++;

		n_total++;
	}

	virtual double likelihood(const bool &sample)
	{
		double true_prob = n_true / n_total;

		if (sample)
			return true_prob;
		else
			return 1.0 - true_prob;
	}
};


class Categorical : public DistributionInterface<int>
{
public:
	std::map<int, int> categories_count;
	int total;

	Categorical()
	{
		total = 0;
	}

	virtual void update(const int &sample)
	{
		std::map<int, int>::iterator it = categories_count.find(sample);

		if (it != categories_count.end())
			it->second++;
		else
			categories_count.insert(std::pair<int, int>(sample, 1));

		total++;
	}

	virtual double likelihood(const int &sample)
	{
		std::map<int, int>::iterator it = categories_count.find(sample);

		if (it != categories_count.end())
			return (double) it->second / (double) total;
		else
			return 0.0;
	}

	int most_likely()
	{
		int n_max, id_max;
		std::map<int, int>::iterator it;

		id_max = n_max = -1;

		for (it = categories_count.begin(); it != categories_count.end(); it++)
		{
			if (it->second > n_max)
			{
				n_max = it->second;
				id_max = it->first;
			}
		}

		return id_max;
	}
};


class Gaussian : public DistributionInterface<double>
{
public:
	double sum_squared, mean, std;
	long int n;

	Gaussian()
	{
		sum_squared = mean = std = 0.0;
		n = 0;
	}

	virtual void update(const double &sample)
	{
		//SUM(i=1..n){values[i]^2} - period*(average^2)
		sum_squared += pow(sample, 2);
		mean = (sample + n * mean) / ((double) (n + 1));
		std = sqrt(sum_squared / (n + 1) - pow(mean, 2));
		n++;
	}

	virtual double likelihood(const double &sample)
	{
		if (std > 0)
		{
			double multiplier = 1. / (std * sqrt(2.0 * M_PI));
			double exponent = -0.5 * pow((sample - mean) / std, 2);
			return multiplier * exp(exponent);
		}
		else
		{
			// if std = 0.0, then we have no uncertainty about the mean.
			if (sample == mean)
				return 1.0;
			else
				return 0.0;
		}
	}
};


class CellInterface
{
public:
	virtual ~CellInterface() {}
	virtual void update(const pcl::PointXYZRGB &point) = 0;
	virtual double likelihood(const pcl::PointXYZRGB &point) = 0;
	virtual cv::Scalar get_color() = 0;
};


class OccupancyCell : public CellInterface
{
public:
	Bernoulli statistics;

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
		unsigned char color = (unsigned char) (statistics.likelihood(1) * 255.0);
		return cv::Scalar(color, color, color);
	}
};


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


class ReflectivityCell : public CellInterface
{
public:
	Gaussian statistics;

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
		unsigned char color = (unsigned char) statistics.mean;
		return cv::Scalar(color, color, color);
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


/**
 * Convention that classes that represent cell values must follow:
 * - In the same header file in which the class is declared, the following functions shall be provided:
 * 			- A function for updating the cell value.
 * 			- A function for generating a color based on the cell value.
 * 			- A function for computing the likelihood of a point.
 * 			- A function for updating the cell value with another cell value for soft measurements.
 * 			- A function for computing the kl-divergence between two cell values for soft measurements.
  */

class KeyGeneratorInterface
{
public:
	virtual ~KeyGeneratorInterface() {}
	virtual unsigned long coordinates_to_key(int cx, int cy, int cz);
	virtual void key_to_coordinates(unsigned long key, int *cx, int *cy, int *cz);
};

class KeyGen2D
{
public:
	int offset;

	KeyGen2D() { offset = 32; }

	virtual ~KeyGen2D() {}

	virtual unsigned long coordinates_to_key(int cx, int cy, int cz __attribute__((unused)))
	{
		unsigned long key = 0;

		key |= (unsigned long) cx;
		key <<= offset;
		key |= (unsigned long) cy;

		return key;
	}

	virtual void key_to_coordinates(unsigned long key, int *cx, int *cy, int *cz)
	{
		*cx = *cy = *cz = 0;
		*cy = key & 0x00000000FFFFFFFF;
		key >>= offset;
		*cx = key & 0x00000000FFFFFFFF;
	}
};


class KeyGen3D
{
public:
	int offset;

	KeyGen3D()
	{
		offset = 20;
	}

	virtual ~KeyGen3D() {}

	virtual unsigned long coordinates_to_key(int cx, int cy, int cz)
	{
		unsigned long key = 0;

		key |= (unsigned long) cx;
		key <<= offset;
		key |= (unsigned long) cy;
		key <<= offset;
		key |= (unsigned long) cz;

		return key;
	}

	virtual void key_to_coordinates(unsigned long key, int *cx, int *cy, int *cz)
	{
		*cx = *cy = *cz = 0;
		*cz = key & 0x00000000000FFFFF;
		key >>= offset;
		*cy = key & 0x00000000000FFFFF;
		key >>= offset;
		*cx = key & 0x00000000000FFFFF;
	}
};


template<class T>
class HashGridMap
{
public:
	HashGridMap() { }
	~HashGridMap() { }

	void add(const pcl::PointXYZRGB &point)
	{
		cell.update(point);
	}

	void add(const std::vector<pcl::PointXYZRGB> &points)
	{
		for (int i = 0; i < points.size(); i++)
			add(points[i]);
	}

	void add(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
	{
		for (int i = 0; i < point_cloud->size(); i++)
			add(point_cloud->at(i));
	}

	//std::vector<double> get(pcl::PointXYZRGB point);

	//double resolution();
	//double x_origin();
	//double y_origin();
	//double width();
	//double height();

	//int n_cells();

	// returns information regarding the ith cell.
	//T* at(int i);
	//T* operator[](int i);

	T cell;

protected:

	std::map<int, std::vector<double>> _cells;

	//double _resolution;
	//double _x_origin;
	//double _y_origin;
	//double _width;
	//double _height;
};


#endif
