#ifndef _POSETRACKER_H
#define _POSETRACKER_H

#include <carmen/tf.h>
#include <aruco/aruco.h>
#include <opencv2/highgui.hpp>

class PoseTracker
{
public:    
    double offset;

private:
    int type; // 1: by dict, 2: by board
    int cam_initialized;
    double marker_size;
    char *filepath;
    aruco::MarkerMap mmap;
    aruco::MarkerDetector Detector;
    aruco::CameraParameters camera;
    aruco::MarkerMapPoseTracker MMTracker;

    cv::Mat _tvec;
    cv::Mat _rvec;

public:
    PoseTracker(double _marker_size_in_meters, const char *_board_filename_or_dictionary, double _offset = 0.0)
    {
        marker_size = _marker_size_in_meters;
        filepath = (char*) malloc(strlen(_board_filename_or_dictionary)*sizeof(char));
        strcpy(filepath, _board_filename_or_dictionary);
        offset = _offset;

        if (aruco::Dictionary::isPredefinedDictinaryString(filepath))
        {
            Detector.setDictionary(filepath);

            type = 1;
        }
        else
        {
            mmap.readFromFile(filepath);
            mmap = mmap.convertToMeters(marker_size);
            Detector.setDictionary(mmap.getDictionary());

            type = 2;
        }

        _tvec = cv::Mat::zeros(3, 1, CV_32F);
        _rvec = cv::Mat::zeros(3, 1, CV_32F);

        cam_initialized = 0;
    }

    void setCamera(aruco::CameraParameters _camera)
    {
        camera = _camera;
        if (type == 2)
            MMTracker.setParams(camera, mmap, marker_size);
        
        cam_initialized = 1;
    }

    void setOffset(double _offset)
    {
        _offset = offset;
    }

    int detect_posetracker(cv::Mat &image, bool show_output = true)
    {
        int n_markers_detected = 0;

        std::vector<aruco::Marker> markers = Detector.detect(image);
        std::vector<int> markers_from_set = mmap.getIndices(markers);
        n_markers_detected = markers_from_set.size();
        for (int i = 0; (i < n_markers_detected) && show_output; i++)
                markers[i].draw(image, cv::Scalar(0, 0, 255), 1.5);

        if (MMTracker.estimatePose(markers))
        {
            _rvec = MMTracker.getRvec();
            _tvec = MMTracker.getTvec();

            // +X is Right on the screen, +Y is Up, +Z is INTO the screen
            aruco::CvDrawingUtils::draw3dAxis(image, camera, _rvec, _tvec, marker_size);
        }
        return n_markers_detected;
    }

    int detect_markers(cv::Mat &image, bool show_output = true)
    {
        int n_markers_detected = 0;

        std::vector<aruco::Marker> markers = Detector.detect(image, camera, marker_size);
        n_markers_detected = markers.size();
        for (int i = 0; i < n_markers_detected; i++)
        {
            _tvec += markers[i].Tvec / n_markers_detected;
            _rvec += markers[i].Rvec / n_markers_detected;
        }
        if (n_markers_detected > 0)
        {
            cv::Mat _std_tvec = cv::Mat::zeros(3, 1, CV_32F), _std_rvec = cv::Mat::zeros(3, 1, CV_32F), pow;

            for (int i = 0; i < n_markers_detected; i++)
            {
                cv::pow((markers[i].Tvec - _tvec), 2, pow);
                _std_tvec += pow;
                cv::pow((markers[i].Rvec - _rvec), 2, pow);
                _std_rvec += pow;
            }
            cv::sqrt(_std_tvec / n_markers_detected, _std_tvec);
            cv::sqrt(_std_rvec / n_markers_detected, _std_rvec);

            // printf("std L2 Tvec: %lf\n", cv::norm(_std_tvec));
            // printf("std L2 Rvec: %lf\n", cv::norm(_std_rvec));
        }

        if (camera.isValid() && (marker_size > 0))
        {
            for (int i = 0; i < n_markers_detected; i++)
            {
                aruco::CvDrawingUtils::draw3dAxis(image, markers[i], camera);
                markers[i].draw(image, cv::Scalar(0, 0, 255), 1.5);
            }
        }
        
        return n_markers_detected;
    }

    int detect(cv::Mat &image, bool show_output = true)
    {
        if (!cam_initialized)
        {
            std::cout << "please set the camera!" << std::endl;
            return 0;
        }
        if (type == 1)
            return detect_markers(image, show_output);
        else
            return detect_posetracker(image, show_output);
    }

    cv::Mat getRvec()
    {
        return _rvec;
    }

    cv::Mat getTvec()
    {
        return _tvec;
    }

    void reset()
    {
        MMTracker.reset();
    }
};

#endif
