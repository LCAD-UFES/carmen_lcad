#include <StampedOdometry.hpp>
#include <iostream>

using namespace hyper;


// {1.0016352367, 1.1777266776, 0.0010183523},
// 1.000527 0.000000 bias phi: 1.113861 0.000061

StampedOdometry::StampedOdometry(unsigned msg_id) :
    StampedMessage(msg_id), v(0.0),
    phi(0.0),
    vmb(1.000527),
    phiab(0.000061),
    phimb(1.113861) {}

// basic destructor
StampedOdometry::~StampedOdometry() {}

// parse odometry from string stream
bool StampedOdometry::FromCarmenLog(std::stringstream &ss) {

    // read the velocity value
    ss >> raw_v;

    // read the phi value
    ss >> raw_phi;

    // read the timestamp value
    ss >> StampedOdometry::timestamp;

    // filtering
    if (0.0011 > std::fabs(v)) {

        // reset
        v = 0.0;

    }

    v = raw_v * vmb;
    phi = raw_phi * phimb + phiab;

    return true;

}