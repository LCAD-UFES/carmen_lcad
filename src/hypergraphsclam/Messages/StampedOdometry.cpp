#include <StampedOdometry.hpp>
#include <iostream>

using namespace hyper;


// {1.0016352367, 1.1777266776, 0.0010183523},
// 1.000527 0.000000 bias phi: 1.113861 0.000061

// static variables
double StampedOdometry::vmb = 1.0f;
double StampedOdometry::phiab = 0.0f;
double StampedOdometry::phimb = 1.0f;

StampedOdometry::StampedOdometry(unsigned msg_id) :
    StampedMessage(msg_id), raw_v(0.0f), raw_phi(0.0f), v(0.0f), phi(0.0f) {}

// basic destructor
StampedOdometry::~StampedOdometry() {}


// parse odometry from string stream
bool StampedOdometry::FromCarmenLog(std::stringstream &ss)
{
    // read the velocity value
    ss >> raw_v;

    // read the phi value
    ss >> raw_phi;

    // read the timestamp value
    ss >> StampedOdometry::timestamp;

    // filtering
    if (0.001 > std::fabs(v))
    {
        v = 0.0;
    }

    v = raw_v * vmb;
    phi = raw_phi * phimb + phiab;

    return true;
}


// get the message type
StampedMessageType StampedOdometry::GetType() { return StampedOdometryMessage; }