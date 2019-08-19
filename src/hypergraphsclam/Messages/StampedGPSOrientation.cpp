#include <StampedGPSOrientation.hpp>

using namespace hyper;

// basic constructor
StampedGPSOrientation::StampedGPSOrientation(unsigned msg_id) : StampedMessage(msg_id), yaw(0.0) {}

// basic destructor
StampedGPSOrientation::~StampedGPSOrientation() {}

// parse the pose from string stream
bool StampedGPSOrientation::FromCarmenLog(std::stringstream &ss)
{
    // discards the first value
    SkipValues(ss, 1);

    // read the yaw value
    ss >> yaw;

    // discard the next value
    SkipValues(ss, 1);

    // get the timestamp
    ss >> StampedMessage::timestamp;

    StampedMessage::timestamp += StampedGPSOrientation::delay;

    return true;
}


// get the message type
StampedMessageType StampedGPSOrientation::GetType()
{
    return StampedGPSOrientationMessage;
}


double StampedGPSOrientation::delay = 0.0;