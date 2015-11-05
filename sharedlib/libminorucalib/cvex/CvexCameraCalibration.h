#ifndef _CVEX_CAMERA_CALIBRATION_H_
#define _CVEX_CAMERA_CALIBRATION_H_

#include "mcv.h"

class CvexCameraCalibration
{
protected:
    int image_count;
    CvSize image_size;   // image resolution
    CvSize pattern_size; // num. of col and row in a pattern

    double CHESS_LENGTH; // length(mm)
    int PATTERN_SIZE;
    int MAX_CHESS_COUNT;

    int corner_count;
    CvPoint2D32f *corners;

    int isFound;

    int *p_count;

public:
    CvPoint2D32f **subpix_corners;
    CvMat *intrinsic;
    CvMat *rodrigues;
    CvMat *rotation;
    CvMat *translation;
    CvMat *distortion;

    void init(
        CvSize img_size,
        CvSize _pattern_size,
        double _CHESS_LENGTH, 
        int _max_chess_count=10000);
    int getImageCount();

    CvexCameraCalibration();
    CvexCameraCalibration(
        CvSize img_size,
        CvSize _pattern_size,
        double _CHESS_LENGTH,
        int _max_chess_count=10000);
    ~CvexCameraCalibration();

    bool findChessboardCorners(
        IplImage* src_img, bool isStore=true);

    void drawChessboardCorners(IplImage* src, IplImage* dest);
    void solveExtrinsic(int pattern_number);

    void solveIntrinsic();
};

#endif

