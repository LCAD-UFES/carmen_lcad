
#include <carmen/segmap_cell_interface.h>
#include "segmap_grayscale_cell.h"


void
GrayscaleCell::add(const pcl::PointXYZRGB &point)
{
	statistics.update(point.r);
}


double
GrayscaleCell::log_likelihood(const pcl::PointXYZRGB &point)
{
	return statistics.log_likelihood(point.r);
}


cv::Scalar
GrayscaleCell::get_color()
{
	unsigned char color = (unsigned char) statistics.mean;
	return cv::Scalar(color, color, color);
}


void
GrayscaleCell::write(FILE *fptr)
{
	fwrite(&statistics.mean, sizeof(statistics.mean), 1, fptr);
	fwrite(&statistics.std, sizeof(statistics.std), 1, fptr);
	fwrite(&statistics.n, sizeof(statistics.n), 1, fptr);
	fwrite(&statistics.sum_squared, sizeof(statistics.sum_squared), 1, fptr);
}


void
GrayscaleCell::read(FILE *fptr)
{
	fread(&statistics.mean, sizeof(statistics.mean), 1, fptr);
	fread(&statistics.std, sizeof(statistics.std), 1, fptr);
	fread(&statistics.n, sizeof(statistics.n), 1, fptr);
	fread(&statistics.sum_squared, sizeof(statistics.sum_squared), 1, fptr);
}
