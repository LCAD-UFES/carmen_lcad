#ifndef HYPERGRAPHSLAM_STAMPED_GPS_POSE_HPP
#define HYPERGRAPHSLAM_STAMPED_GPS_POSE_HPP

#include <sstream>
#include <utility>
#include <vector>
#include <unordered_map>

#include <StampedMessage.hpp>

namespace hyper {

	class StampedGPSPose : virtual public StampedMessage
	{
		public:

			// the gps relative displacement wrt the car
			static std::unordered_map<std::string, std::pair<g2o::SE2, double>> gps_pose_delays;

			// the gps measure
			g2o::SE2 gps_measurement;

			// the standard deviation value
			double gps_std;

			// the gps id from carmen log
			std::string gps_id;

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
