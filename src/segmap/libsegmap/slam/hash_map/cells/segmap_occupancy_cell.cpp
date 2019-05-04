

#include <carmen/segmap_cell_interface.h>
#include <carmen/segmap_occupancy_cell.h>
#include <cstdio>


void
OccupancyCell::add(const pcl::PointXYZRGB &point)
{
	statistics.update(point.r);
}


double
OccupancyCell::log_likelihood(const pcl::PointXYZRGB &point)
{
	return statistics.log_likelihood(point.r);
}


cv::Scalar
OccupancyCell::get_color()
{
	unsigned char color = (unsigned char) (statistics.likelihood(1) * 255.0);
	return cv::Scalar(color, color, color);
}


void
OccupancyCell::write(FILE *fptr)
{
	fwrite(&statistics.n_true, sizeof(double), 1, fptr);
	fwrite(&statistics.n_total, sizeof(double), 1, fptr);
}


void
OccupancyCell::read(FILE *fptr)
{
	fread(&statistics.n_true, sizeof(double), 1, fptr);
	fread(&statistics.n_total, sizeof(double), 1, fptr);
}
