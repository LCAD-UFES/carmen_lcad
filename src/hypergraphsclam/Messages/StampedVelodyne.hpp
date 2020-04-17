#ifndef HYPERGRAPHSLAM_STAMPED_VELODYNE_HPP
#define HYPERGRAPHSLAM_STAMPED_VELODYNE_HPP

#include <StampedLidar.hpp>

namespace hyper {

    class StampedVelodyne : virtual public StampedLidar
    {
        protected:

            // the default velodyne file path
            static const std::string base_velodyne_path;

            // the vertical correction values
            static const double vertical_correction[32];

            // the double size
            static const unsigned double_size;

            // the char size
            static const unsigned char_size;

            // the short size
            static const unsigned short_size;

            // the char and short size
            static const unsigned short_char_size;

            // the struct size
            static const unsigned velodyne_struct_size;

            // read the point cloud from file
            void ReadVelodyneCloudFromFile(PointCloudHSV &cloud);
            
            // read the point cloud from carmen log
            PointCloudHSV::Ptr ReadVelodyneCloudFromLog(std::stringstream &ss);

            bool from_file;

        public:

            // how many vertical scans
            unsigned vertical_scans;

            // the basic constructor
            StampedVelodyne(unsigned msg_id);

            // the basic destructor
            virtual ~StampedVelodyne();
            
            virtual void LoadPointCloud(PointCloudHSV &cloud);

            // parse the pose from string stream
            virtual bool FromCarmenLog(std::stringstream &ss);

            // get the message type
            virtual StampedMessageType GetType();

    };

    typedef StampedVelodyne* StampedVelodynePtr;
    typedef StampedVelodyne& StampedVelodyneRef;

}

#endif
