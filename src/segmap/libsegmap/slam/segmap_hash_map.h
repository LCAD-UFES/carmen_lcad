
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
class Statistics
{
public:
	virtual ~Statistics() { }
	virtual void update(const T &sample) = 0;
	virtual double likelihood(const T &sample) = 0;

	// soft update
	// prior
	// kl-divergence
};

// virtual void color(unsigned char *r, unsigned char *g, unsigned char *b) = 0;

class Bernoulli : public Statistics<bool>
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


class Categorical : public Statistics<int>
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


void update_distribution(Bernoulli *distribution, pcl::PointXYZRGB &point)
{
	distribution->update(point.r);
}


void update_distribution(Categorical *distribution, pcl::PointXYZRGB &point)
{
	distribution->update(point.r);
}


cv::Scalar get_color(Bernoulli *distribution)
{
	unsigned char color = (unsigned char) (distribution->likelihood(1) * 255.0);
	return cv::Scalar(color, color, color);
}


cv::Scalar get_color(Categorical *distribution)
{
	CityScapesColorMap colormap;
	return colormap.color(distribution->most_likely());
}

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
	virtual ~KeyGeneratorInterface();
	virtual int coordinates_to_key(double x, double y, double z);
	virtual void key_to_coordinates(int key, double *x, double *y, double *z);
};


template<class T>
class HashGridMap
{
public:
	HashGridMap() { }
	~HashGridMap() { }

	void add(pcl::PointXYZRGB &point)
	{
		update_distribution(&test_distr, point);
	}

	//void add(std::vector<pcl::PointXYZRGB> &point);
	//void add(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

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

	T test_distr;

protected:

	std::map<int, std::vector<double>> _cells;

	//double _resolution;
	//double _x_origin;
	//double _y_origin;
	//double _width;
	//double _height;
};


#endif
