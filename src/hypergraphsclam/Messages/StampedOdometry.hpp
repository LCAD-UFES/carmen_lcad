#ifndef HYPERGRAPHSLAM_STAMPED_ODOMETRY_HPP
#define HYPERGRAPHSLAM_STAMPED_ODOMETRY_HPP

#include <StampedMessage.hpp>

namespace hyper {

    class StampedOdometry : virtual public StampedMessage
    {
        public:

            double raw_v, raw_phi;
            double v, phi;

            // basic constructor
            StampedOdometry(unsigned msg_id);

            // basic destructor
            ~StampedOdometry();

            // parse odometry from string stream
            virtual bool FromCarmenLog(std::stringstream &ss);

            // get the message type
            virtual StampedMessageType GetType();

            // static public variables
            static double vmb, phiab, phimb;
    };

    // syntactic sugar
    typedef StampedOdometry* StampedOdometryPtr;
    typedef StampedOdometry& StampedOdometryRef;

    // define the standard vector type
    typedef std::vector<StampedOdometryPtr> StampedOdometryPtrVector;

}

#endif
