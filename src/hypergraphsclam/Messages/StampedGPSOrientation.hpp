#ifndef HYPERGRAPHSLAM_STAMPED_GPS_ORIENTATION_HPP
#define HYPERGRAPHSLAM_STAMPED_GPS_ORIENTATION_HPP

#include <StampedMessage.hpp>

namespace hyper {

    class StampedGPSOrientation : virtual public StampedMessage
    {
        public:

            // public members
            double yaw;

            // basic constructor
            StampedGPSOrientation(unsigned msg_id);

            // the gps message delay
            static double delay;

            // basic destructor
            ~StampedGPSOrientation();

            // parse the pose from string stream
            virtual bool FromCarmenLog(std::stringstream &ss);

            // get the message type
            virtual StampedMessageType GetType();
    };

    // syntactic sugar
    typedef StampedGPSOrientation* StampedGPSOrientationPtr;
    typedef StampedGPSOrientation& StampedGPSOrientationeRef;

}

#endif
