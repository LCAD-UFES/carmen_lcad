#include <StampedXSENS.hpp>
#include <cmath>

using namespace hyper;

// the basic constructor
StampedXSENS::StampedXSENS(unsigned msg_id) : StampedMessage(msg_id), yaw(00.0) {}


// the basic destructor
StampedXSENS::~StampedXSENS() {}


// PRIVATE METHODS


// get the yaw angle from a quaternion
double StampedXSENS::GetYawFromQuaternion(double w, double x, double y, double z)
{
    // get the test around the poles
    double t = w * y - x * z;

    if (0.499 < t)
    {
        // singularity at the north pole
        return - 2.0 * std::atan2(x, w);
    }
    else if (-0.499 > t)
    {
        // singularity at the south pole
        return 2.0 * std::atan2(x, w);
    }

    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// PUBLIC METHODS

// parse the pose from string stream
bool StampedXSENS::FromCarmenLog(std::stringstream &ss)
{
    // the quaternion values
    double w, x, y, z;

    // discards the first three values
    SkipValues(ss, 3);

    // get the four quaternions values
    ss >> w >> x >> y >> z;

    // convert the quaternion to yaw
    yaw = GetYawFromQuaternion(w, x, y, z);

    // discards the next eight values
    SkipValues(ss, 8);

    // get the message timestamp
    ss >> StampedMessage::timestamp;

    return true;
}


// get the message type
StampedMessageType StampedXSENS::GetType()
{
    return StampedXsensMessage;
}