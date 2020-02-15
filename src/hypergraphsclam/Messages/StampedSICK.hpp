#ifndef HYPERGRAPHSLAM_STAMPED_SICK_HPP
#define HYPERGRAPHSLAM_STAMPED_SICK_HPP

#include <StampedLidar.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hyper {

    class StampedSICK : virtual public StampedLidar
    {
        protected:

            // the default sick file path
            static const std::string base_sick_path;

            // the mirror mask
            static const uint16_t MIRROR_MASK;

        public:

            // the basic constructor
            StampedSICK(unsigned msg_id);

            // the basic destructor
            virtual ~StampedSICK();

            virtual void LoadPointCloud(PointCloudHSV &cloud);

            // parse the pose from string stream
            virtual bool FromCarmenLog(std::stringstream &ss);

            // get the message type
            virtual StampedMessageType GetType();

    };

    // syntactic sugar
    typedef StampedSICK* StampedSICKPtr;
    typedef StampedSICK& StampedSICKRef;

}

#endif
