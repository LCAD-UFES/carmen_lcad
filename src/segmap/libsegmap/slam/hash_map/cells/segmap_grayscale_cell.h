
#ifndef __SEGMAP_REFLECTIVITY_CELL_H__
#define __SEGMAP_REFLECTIVITY_CELL_H__

#include <cstdio>
#include <opencv/cv.h>
#include <pcl/point_types.h>
#include <carmen/segmap_gaussian.h>
#include <carmen/segmap_cell_interface.h>


class GrayscaleCell : public CellInterface
{
public:
	Gaussian statistics;

	virtual void add(const pcl::PointXYZRGB &point);
	virtual double log_likelihood(const pcl::PointXYZRGB &point);
	virtual cv::Scalar get_color();
	virtual void write(FILE *fptr);
	virtual void read(FILE *fptr);
};



#endif
