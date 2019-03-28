
#ifndef __SEGMAP_SENSORS_H__
#define __SEGMAP_SENSORS_H__

#include <opencv/cv.hpp>
#include <carmen/synchronized_data_package.h>


class SemanticSegmentationLoader
{
public:
    SemanticSegmentationLoader(const char *log_path, const char *data_path="/dados/data");
    ~SemanticSegmentationLoader();

    cv::Mat load(DataSample *sample);

protected:

    std::string _log_data_dir;
};


#endif
