#ifndef _CVEX_STEREO_CAMERA_CALIBRATION_H_
#define _CVEX_STEREO_CAMERA_CALIBRATION_H_

#include <stdio.h>
#include "CvexCameraCalibration.h"

enum {
    CVEX_STEREO_CALIB_LEFT=0,
    CVEX_STEREO_CALIB_RIGHT
};

class CvexStereoCameraCalibration : protected CvexCameraCalibration
{
private:
    CvexCameraCalibration leftCamera;
    CvexCameraCalibration rightCamera;

    CvMat *relate_rot;
    CvMat *relate_trans;

    CvMat *E;
    CvMat *F;

    CvMat *rectification_H_left;
    CvMat *rectification_H_right;

    CvMat* remap_left_x;
    CvMat* remap_left_y;
    CvMat* remap_right_x;
    CvMat* remap_right_y;

    void getKRK(CvMat* srcK,CvMat* R, CvMat* destK, CvMat* dest);
public:
    CvexStereoCameraCalibration(
        CvSize img_size,
        CvSize _pattern_size,
        double _CHESS_LENGTH,
        int _max_chess_count=1000);
    ~CvexStereoCameraCalibration();

    bool findChess(IplImage* left_img,IplImage* right_img);
    void drawChessboardCorners(
        IplImage* src, IplImage* dest,
        int left_or_right=CVEX_STEREO_CALIB_LEFT);

    void showRectificationHomography();
    void saveRectificationHomography(FILE * file);
    void showExtrinsicParameters();
    void saveExtrinsicParameters(FILE * file);
    void showIntrinsicParameters();
    void saveIntrinsicParameters(FILE * file);

    void solveStereoParameter();
    void getRectificationMatrix(CvMat* h_left, CvMat* h_right, CvMat* Q);
    void rectifyImageRemap(IplImage* src, IplImage* dest, int left_right);

    int getImageCount();
};

#endif

