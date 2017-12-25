#ifndef HYPERGRAPHSLAM_STAMPED_XSENS_HPP
#define HYPERGRAPHSLAM_STAMPED_XSENS_HPP

#include <StampedMessage.hpp>

namespace hyper {

    class StampedXSENS : virtual public StampedMessage
    {

        private:

            // get the yaw angle from a quaternion
            double GetYawFromQuaternion(double w, double x, double y, double z);

        public:

            // the yaw orientation
            double yaw;

            // the basic constructor
            StampedXSENS(unsigned msg_id);

            // the basic destructor
            virtual ~StampedXSENS();

            // parse the pose from string stream
            virtual bool FromCarmenLog(std::stringstream &ss);

            // get the message type
            virtual StampedMessageType GetType();
    };

    // syntactic sugar
    typedef StampedXSENS* StampedXSENSPtr;
    typedef StampedXSENS& StampedXSENSRef;

    // helper
    typedef std::vector<StampedXSENSPtr> StampedXSENSPtrVector;

}

#endif
