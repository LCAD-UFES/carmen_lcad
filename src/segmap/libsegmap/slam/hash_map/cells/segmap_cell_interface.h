
#ifndef __SEGMAP_CELL_INTERFACE_H__
#define __SEGMAP_CELL_INTERFACE_H__

#include <cstdio>
#include <opencv/cv.h>
#include <pcl/point_types.h>


class CellInterface
{
public:
	CellInterface() {}
	virtual ~CellInterface() {}
	virtual void add(const pcl::PointXYZRGB &point) = 0;
	virtual double log_likelihood(const pcl::PointXYZRGB &point) = 0;
	virtual cv::Scalar get_color() = 0;
	virtual void write(FILE *fptr) = 0;
	virtual void read(FILE *fptr) = 0;
};


#endif
