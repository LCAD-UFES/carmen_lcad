#include <StampedBumblebee.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

#include <png++/png.hpp>

using namespace hyper;

// the basic constructor
StampedBumblebee::StampedBumblebee(unsigned msg_id) :
    StampedMessage(msg_id), raw_image(""),
    left_image("/dados/tmp/images/I1_"),
    right_image("/dados/tmp/images/I2_"),
    width(0),
    height(0),
    size(0),
    is_rectified(false),
    speed(0.0),
    seq_measurement(0.0, 0.0, 0.0),
    seq_id(std::numeric_limits<unsigned>::max()),
    visual_estimate(0.0, 0.0, 0.0) {}

// the basic destructor
StampedBumblebee::~StampedBumblebee() {}

// parse the raw image and save it to the output folder
bool StampedBumblebee::LoadBumblebeeImage(std::vector<uint8_t> &limg, std::vector<uint8_t> &rimg)
{
    bool result = false;

    if (0 < size)
    {
        // open the images
        std::ifstream images(raw_image, std::ifstream::in | std::ios::binary);

        if (!images.is_open())
        {
            throw std::runtime_error("Could not open the input file\n");
        }

        // resize the local buffers
        limg.resize(width * height);
        rimg.resize(width * height);

        // the input buffers
        std::vector<unsigned char> left(size), right(size);

        // copy all data to buffers
        images.read((char *) &left[0], size);
        images.read((char *) &right[0], size);

        // close the file stream
        images.close();

        for (unsigned i = 0; i < width * height; ++i)
        {
            unsigned char *lpxl = &left[i * 3];
            unsigned char *rpxl = &right[i * 3];

            limg[i] = uint8_t(0.21 * float(lpxl[0]) + 0.72 * float(lpxl[1]) + 0.07 * float(lpxl[2]));
            rimg[i] = uint8_t(0.21 * float(rpxl[0]) + 0.72 * float(rpxl[1]) + 0.07 * float(rpxl[2]));
        }

        // save the png output image
        result = true;
    }

    return result;
}

// parse the raw image and save it to the output folder
bool StampedBumblebee::ParseBumblebeeImage(std::vector<uint8_t> &limg, std::vector<uint8_t> &rimg)
{
    bool result = false;

    if (0 < size)
    {
        // alloc the image in memmory
        std::vector<unsigned char> left(size), right(size);

        // open the images
        std::ifstream images(raw_image, std::ifstream::in | std::ios::binary);

        if (!images.is_open())
        {
            throw std::runtime_error("Could not open the input file\n");
        }

        // copy all data to buffers
        images.read((char *) &left[0], size);
        images.read((char *) &right[0], size);

        // close the file stream
        images.close();

        // create the png images
        png::image<png::gray_pixel> lpng(width, height);
        png::image<png::gray_pixel> rpng(width, height);

        // resize the vectors
        limg.resize(width * height);
        rimg.resize(width * height);

        unsigned k = 0;
        unsigned l = 0;

        for (unsigned j = 0; j < height; ++j)
        {
            for (unsigned i = 0; i < width; ++i)
            {
                unsigned char *lpxl = &left[k];
                unsigned char *rpxl = &right[k];

                limg[l] = uint8_t(0.21 * float(lpxl[0]) + 0.72 * float(lpxl[1]) + 0.07 * float(lpxl[2]));
                rimg[l] = uint8_t(0.21 * float(rpxl[0]) + 0.72 * float(rpxl[1]) + 0.07 * float(rpxl[2]));

                lpng[j][i] = unsigned(limg[l]);
                rpng[j][i] = unsigned(rimg[l]);

                k += 3;
                l += 1;
            }
        }

        // create the names
        std::stringstream ss;
        ss << std::setw(8) << std::setfill('0') << StampedMessage::id;
        ss << ".png";

        left_image += ss.str();
        right_image += ss.str();

        lpng.write(left_image);
        rpng.write(right_image);

        // save the png output image
        result = true;
    }

    return result;
}

// parse the pose from string stream
bool StampedBumblebee::FromCarmenLog(std::stringstream &ss)
{
    // read the path
    ss >> raw_image;

    // read the image size
    ss >> width >> height >> size >> is_rectified;

    // read the timestamp
    ss >> StampedMessage::timestamp;

    // verify if file exists
    return 0 < width && 0 < height && 0 < size && width * height * 3 == size;
}

// the raw image path
std::string StampedBumblebee::GetRawImagePath()
{
    return raw_image;
}

// get the image file path
std::string StampedBumblebee::GetLeftImagePath()
{
    return left_image;
}

// the image file path
std::string StampedBumblebee::GetRightImagePath()
{
    return right_image;
}

// get the image width
unsigned StampedBumblebee::GetWidth()
{
    return width;
}

// get the image width
unsigned StampedBumblebee::GetHeight()
{
    return height;
}

// get the image size
unsigned StampedBumblebee::GetImageSize()
{
    return width * height;
}

// get the message type
StampedMessageType StampedBumblebee::GetType()
{
    return StampedBumblebeeMessage;
}