#ifndef HYPERGRAPHSLAM_STAMPED_GPS_POSE_HPP
#define HYPERGRAPHSLAM_STAMPED_GPS_POSE_HPP

#include <vector>
#include <sstream>

#include <StampedMessage.hpp>

namespace hyper {

	class StampedGPSPose : virtual public StampedMessage
	{
		public:

			// the gps relative displacement wrt the car
			static g2o::SE2 inv_gps_pose;

			// the external identifier
			static std::string gps_id;
			
			// the gps measure
			g2o::SE2 gps_measurement;

			// the standard deviation value
			double gps_std;

			// basic constructor
			StampedGPSPose(unsigned msg_id);

			// basic destructor
			~StampedGPSPose();

			// parse the pose from string stream
			virtual bool FromCarmenLog(std::stringstream &ss);

			// get the message type
			virtual StampedMessageType GetType();

			// set the local displacement
			void SetSensorPose(g2o::SE2 gpose);
	};

	// syntactic sugar
	typedef StampedGPSPose* StampedGPSPosePtr;
	typedef StampedGPSPose& StampedGPSPoseRef;

	// the standard vector typedef
	typedef std::vector<StampedGPSPosePtr> StampedGPSPosePtrVector;
}

#endif
