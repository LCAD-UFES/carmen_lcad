#include "Pose2D.hpp"
#include <cmath>

using namespace smoother;

// basic constructor
Pose2D::Pose2D() : position(0.0, 0.0), orientation(0.0) {}

// copy constructor
Pose2D::Pose2D(const Pose2D &pose) :position(pose.position), orientation(pose.orientation) {}

// explicit constructor
Pose2D::Pose2D(const Vector2D<double> &p, double o) : position(p), orientation(o) {}

// explicit constructor
Pose2D::Pose2D(double x, double y, double o) : position(x, y), orientation(o) {}

// assignement operator overloading
void Pose2D::operator=(const Pose2D &pose) {

    position = pose.position;
    orientation = pose.orientation;

}

// equals to operator overloading
bool Pose2D::operator==(const Pose2D &pose) {

    return (0.0001 > position.Distance2(pose.position) && 0.1 > std::fabs(pose.orientation - orientation));

}

// differet operator overloading
bool Pose2D::operator!=(const Pose2D &pose) {

    return (0.0001 < std::fabs(position.Distance2(position)) || 0.1 < std::fabs(pose.orientation - orientation));

}
