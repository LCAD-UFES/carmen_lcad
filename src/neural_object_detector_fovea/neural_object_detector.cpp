#include "neural_object_detector.hpp"


carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset)
{
	point.x += offset.x;
	point.y += offset.y;
	point.z += offset.z;
	return (point);
}


carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta)
{
	carmen_vector_3D_t p;
	p.x = point.x * cos(theta) - point.y * sin(theta);
	p.y = point.x * sin(theta) + point.y * cos(theta);
	p.z = point.z;
	return (p);
}
