#ifndef HYPERGRAPHSLAM_STAMPED_BUMBLEBEE_HPP
#define HYPERGRAPHSLAM_STAMPED_BUMBLEBEE_HPP

#include <StampedMessage.hpp>

namespace hyper {

    class StampedBumblebee : virtual public StampedMessage
    {
        protected:

            // the base path
            std::string raw_image;

            // the png image paths
            std::string left_image, right_image;

            // resolution
            unsigned width, height, size;

            // is it a rectified image?
            bool is_rectified;

        public:

            // the current speed, for filtering purpose
            double speed;

            // the sequential visual odometry measure
            g2o::SE2 seq_measurement;

            // the seq id
            unsigned seq_id;

            // the sequantial estimate
            g2o::SE2 visual_estimate;

            // the basic constructor
            StampedBumblebee(unsigned msg_id);

            // the basic destructor
            virtual ~StampedBumblebee();

            // parse the pose from string stream
            virtual bool FromCarmenLog(std::stringstream &ss);

            // parse the raw image and save it to the output folder
            bool ParseBumblebeeImage(std::vector<uint8_t> &limg, std::vector<uint8_t> &rimg);

            // load the image from the log
            bool LoadBumblebeeImage(std::vector<uint8_t> &limg, std::vector<uint8_t> &rimg);

            // the raw image path
            std::string GetRawImagePath();

            // get the image file path
            std::string GetLeftImagePath();

            // the image file path
            std::string GetRightImagePath();

            // get the image width
            unsigned GetWidth();

            // get the image width
            unsigned GetHeight();

            // get the image size
            unsigned GetImageSize();

            // get the message type
            virtual StampedMessageType GetType();

    };

    // syntactic sugar
    typedef StampedBumblebee* StampedBumblebeePtr;
    typedef StampedBumblebee& StampedBumblebeeRef;

    // define the standard vector type
    typedef std::vector<StampedBumblebeePtr> StampedBumblebeePtrVector;

}

#endif