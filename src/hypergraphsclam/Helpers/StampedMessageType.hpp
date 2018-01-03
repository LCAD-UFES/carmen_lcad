#ifndef HYPERGRAPHSCLAM_MESSAGE_TYPE_HPP
#define HYPERGRAPHSCLAM_MESSAGE_TYPE_HPP

namespace hyper
{
    enum StampedMessageType
    {
        StampedOdometryMessage,
        StampedGPSMessage,
        StampedGPSOrientationMessage,
        StampedXsensMessage,
        StampedVelodyneMessage,
        StampedSICKMessage,
        StampedBumblebeeMessage
    };
}

#endif