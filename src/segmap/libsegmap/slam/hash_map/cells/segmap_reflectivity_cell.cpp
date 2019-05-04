
#include <carmen/segmap_cell_interface.h>
#include <carmen/segmap_reflectivity_cell.h>


void
ReflectivityCell::add(const pcl::PointXYZRGB &point)
{
	statistics.update(point.r);
}


double
ReflectivityCell::log_likelihood(const pcl::PointXYZRGB &point)
{
	return statistics.log_likelihood(point.r);
}


cv::Scalar
ReflectivityCell::get_color()
{
	unsigned char color = (unsigned char) statistics.mean;
	return cv::Scalar(color, color, color);
}


void
ReflectivityCell::write(FILE *fptr)
{
	fwrite(&statistics.mean, sizeof(statistics.mean), 1, fptr);
	fwrite(&statistics.std, sizeof(statistics.std), 1, fptr);
	fwrite(&statistics.n, sizeof(statistics.n), 1, fptr);
	fwrite(&statistics.sum_squared, sizeof(statistics.sum_squared), 1, fptr);
}


void
ReflectivityCell::read(FILE *fptr)
{
	fread(&statistics.mean, sizeof(statistics.mean), 1, fptr);
	fread(&statistics.std, sizeof(statistics.std), 1, fptr);
	fread(&statistics.n, sizeof(statistics.n), 1, fptr);
	fread(&statistics.sum_squared, sizeof(statistics.sum_squared), 1, fptr);
}
