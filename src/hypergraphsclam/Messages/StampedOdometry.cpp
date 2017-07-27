#include <StampedOdometry.hpp>

using namespace hyper;

// basic constructor
StampedOdometry::StampedOdometry(unsigned msg_id) : StampedMessage(msg_id), v(0.0), phi(0.0) {}

// basic destructor
StampedOdometry::~StampedOdometry() {}

// parse odometry from string stream
bool StampedOdometry::FromCarmenLog(std::stringstream &ss) {

    // read the velocity value
    ss >> v;

    // read the phi value
    ss >> phi;

    // read the timestamp value
    ss >> StampedOdometry::timestamp;

    // filtering
    if (0.0011 > std::fabs(v)) {

        // reset
        v = 0.0;

    }

    v *= 1.004232;
    phi = phi * 0.917138 - 0.002914;

    return true;

}
