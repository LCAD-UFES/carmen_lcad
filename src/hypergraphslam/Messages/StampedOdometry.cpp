#include <StampedOdometry.hpp>

using namespace hyper;

// basic constructor
StampedOdometry::StampedOdometry(unsigned msg_id) : StampedMessage(msg_id), v(0.0), phi(0.0) {}

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

    v = raw_v * 0.98;
    phi = raw_phi * 0.9906333 - 0.003390;

    return true;

}
