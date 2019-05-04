#include "segmap_color_cell.h"


pcl::PointXYZRGB
copy_point_and_set_color(const pcl::PointXYZRGB &point, unsigned char color)
{
	pcl::PointXYZRGB ret_point(point);
	ret_point.r = ret_point.g = ret_point.b = color;
	return ret_point;
}


void
ColorCell::add(const pcl::PointXYZRGB &point)
{
	red_cell.add(copy_point_and_set_color(point, point.r));
	green_cell.add(copy_point_and_set_color(point, point.g));
	blue_cell.add(copy_point_and_set_color(point, point.b));
}


double
ColorCell::log_likelihood(const pcl::PointXYZRGB &point)
{
	return red_cell.log_likelihood(copy_point_and_set_color(point, point.r))
			+ green_cell.log_likelihood(copy_point_and_set_color(point, point.g))
			+ blue_cell.log_likelihood(copy_point_and_set_color(point, point.b));
}


cv::Scalar
ColorCell::get_color()
{
	return cv::Scalar(blue_cell.get_color()[0],
										green_cell.get_color()[0],
										red_cell.get_color()[0]);
}


void
ColorCell::write(FILE *fptr)
{
	red_cell.write(fptr);
	green_cell.write(fptr);
	blue_cell.write(fptr);
}


void
ColorCell::read(FILE *fptr)
{
	red_cell.read(fptr);
	green_cell.read(fptr);
	blue_cell.read(fptr);
}
