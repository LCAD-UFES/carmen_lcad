#ifndef HYPERGRAPHSLAM_STAMPED_ODOMETRY_HPP
#define HYPERGRAPHSLAM_STAMPED_ODOMETRY_HPP

#include <StampedMessage.hpp>

namespace hyper {

class StampedOdometry : virtual public StampedMessage {

	public:

		double v;
		double phi;

		// basic constructor
		StampedOdometry(unsigned msg_id);

		// basic destructor
		~StampedOdometry();

		// parse odometry from string stream
		virtual bool FromCarmenLog(std::stringstream &ss);

};

// syntactic sugar
typedef StampedOdometry* StampedOdometryPtr;
typedef StampedOdometry& StampedOdometryRef;

// define the standard vector type
typedef std::vector<StampedOdometryPtr> StampedOdometryPtrVector;

}

#endif
