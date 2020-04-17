#ifndef HYPERGRAPHSLAM_STAMPED_MESSAGE_HPP
#define HYPERGRAPHSLAM_STAMPED_MESSAGE_HPP

#include <limits>
#include <sstream>
#include <vector>

#include <g2o/types/slam2d/se2.h>

#include <StampedMessageType.hpp>

namespace hyper {

    class StampedMessage
    {
        protected:

            // skip n values inside the string stream
            // considering the space character ' ' as the limiter one
            void SkipValues(std::stringstream &ss, unsigned n)
            {
                if (0 < n)
                {
                    // we need one extra skiping procedure
                    ++n;

                    while (0 < n)
                    {
                        // ignore the current value
                        ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');

                        // consume one iteration
                        --n;
                    }
                }
            }

        public:

            // the usual timestamp
            double timestamp;

            // a simple id to help the g2o processing
            unsigned id;

            // the SE2 pose estimate
            g2o::SE2 est;

            // the odometry measure
            g2o::SE2 odom_measurement;

            // the raw estimate
            g2o::SE2 raw_est;

            // the raw measure
            g2o::SE2 raw_measurement;

            // the basic constructor
            StampedMessage(unsigned msg_id) :
                timestamp(-1.0),
                id(msg_id),
                est(0.0, 0.0, 0.0),
                odom_measurement(0.0, 0.0, 0.0),
                raw_est(0.0, 0.0, 0.0),
                raw_measurement(0.0, 0.0, 0.0) {}

            // the basic destructor
            virtual ~StampedMessage() {}

            // parse the pose from string stream
            virtual bool FromCarmenLog(std::stringstream &ss) =0;

            // a static method to compare pointers of stamped messages
            static bool compare(StampedMessage *a, StampedMessage *b)
            {
				return nullptr != a && nullptr != b && a->timestamp < b->timestamp;
            }

            // get the message type
            virtual StampedMessageType GetType() =0;

    };

    // syntactic sugar
    typedef StampedMessage* StampedMessagePtr;
    typedef StampedMessage& StampedMessageRef;

    // define the standard vector type
    typedef std::vector<StampedMessagePtr> StampedMessagePtrVector;
}

#endif
