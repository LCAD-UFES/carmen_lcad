
#include <carmen/segmap_cell_interface.h>
#include "segmap_height_cell.h"


void
HeightCell::add(const pcl::PointXYZRGB &point)
{
	statistics.update(point.z);
}


double
HeightCell::log_likelihood(const pcl::PointXYZRGB &point)
{
	return statistics.log_likelihood(point.z);
}


cv::Scalar
HeightCell::get_color()
{
	double color_d = ((statistics.mean + 1) / 3.0) * 255.0;

	if (color_d < 0.0) color_d = 0.0;
	else if (color_d > 255) color_d = 255.0;

	unsigned char color = (unsigned char) color_d;

	return cv::Scalar(color, color, color);
}


void
HeightCell::write(FILE *fptr)
{
	fwrite(&statistics.mean, sizeof(statistics.mean), 1, fptr);
	fwrite(&statistics.std, sizeof(statistics.std), 1, fptr);
	fwrite(&statistics.n, sizeof(statistics.n), 1, fptr);
	fwrite(&statistics.sum_squared, sizeof(statistics.sum_squared), 1, fptr);
}


void
HeightCell::read(FILE *fptr)
{
	fread(&statistics.mean, sizeof(statistics.mean), 1, fptr);
	fread(&statistics.std, sizeof(statistics.std), 1, fptr);
	fread(&statistics.n, sizeof(statistics.n), 1, fptr);
	fread(&statistics.sum_squared, sizeof(statistics.sum_squared), 1, fptr);
}
