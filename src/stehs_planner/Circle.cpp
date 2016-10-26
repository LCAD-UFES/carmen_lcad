#include <cmath>
#include "Circle.hpp"


// basic constructor
Circle::Circle() : x(0.0), y(0.0), radius(1.0) {}

// explicit constructor
Circle::Circle(double x_, double y_, double r_) : x(x_), y(y_), radius(r_) {}

// copy constructor
Circle::Circle(const Circle &c) : x(c.x), y(c.y), radius(c.radius) {}

// Evaluate the overlap of 1 - overlap_factor
bool Circle::Overlap(const Circle& c, double overlap_factor)
{

	double distance;
	double greater, smaller;

	if (radius < c.radius)
	{
		greater = radius;
		smaller = c.radius;

	}
	else
	{
		greater = c.radius;
		smaller = radius;
	}

	// get the squared distance between the 2 circles
	distance = std::sqrt(std::pow(x - c.x, 2) + std::pow(y - c.y, 2));

	return (distance - greater) < smaller * overlap_factor;

}
