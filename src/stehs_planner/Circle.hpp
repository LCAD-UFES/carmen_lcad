#ifndef STEHS_CIRCLE_H
#define STEHS_CIRCLE_H

class Circle
{

public:

	// public variables
	double x;
	double y;
	double radius;

	// basic constructor
	Circle();

	// explicit constructor
	Circle(double x_, double y_, double r_);

	// copy constructor
	Circle(const Circle &c);

	// comparison operator overloading
	bool Overlap(const Circle& c);

};

#endif
