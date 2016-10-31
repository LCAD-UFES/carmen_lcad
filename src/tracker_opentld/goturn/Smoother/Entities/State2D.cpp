#include "State2D.hpp"

#include <cmath>

using namespace smoother;

// simple constructor
State2D::State2D() :
        Pose2D::Pose2D(), v(0.0), phi(0.0), t(0.0) {}

// simple constructor, the input is a pose
State2D::State2D(
        const Pose2D &p,
        double vel,
        double wheel_angle,
        double time
    ) : Pose2D::Pose2D(p), v(vel), phi(wheel_angle), t(time) {}

// copy constructor
State2D::State2D(const State2D &s):
        Pose2D::Pose2D(s),
        v(s.v),
        phi(s.phi),
        t(s.t) {}


// distance between two poses
double State2D::Distance(const State2D &p)
{
    return position.Distance(p.position);
}

// distance squared between two poses
// Euclidean distance
double State2D::Distance2(const State2D &p)
{
    return position.Distance2(p.position);
}

// get the minimum difference between two angles
double State2D::GetOrientationDiff(const State2D &p)
{
    return mrpt::math::angDistance<double>(orientation, p.orientation);
}

// get difference between two orientations (yaw), overloading
double State2D::GetOrientationDiff(double t)
{
    return mrpt::math::angDistance<double>(orientation, t);
}

// the assignment operator
void State2D::operator=(const State2D &s)
{
    position = s.position;
    orientation = s.orientation;
    v = s.v;
    phi = s.phi;
    t = s.t;

}

// alternative assignement operator
void State2D::operator=(const Pose2D &p) {

    position = p.position;
    orientation = p.orientation;

}

// == operator overloading
bool State2D::operator==(const State2D &s)
{
    return (0.0001 > std::fabs(s.position.Distance2(position)) && 0.001 > std::fabs(s.orientation - orientation));
}

// != operator overloading
bool State2D::operator!=(const State2D &s)
{
    return (0.0001 < std::fabs(s.position.Distance2(position)) || 0.001 < std::fabs(s.orientation - orientation));
}

