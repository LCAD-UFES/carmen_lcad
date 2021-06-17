#ifndef _MCV_H_
#define _MCV_H_

#include <stdio.h>
#include <stdarg.h>
#include <opencv/ml.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#define CVEX_KEY_ARROW_LEFT 2424832
#define CVEX_KEY_ARROW_RIGHT 2555904
#define CVEX_KEY_ARROW_UP 2490368
#define CVEX_KEY_ARROW_DOWN 2621440

#define CVEX_WHITE CV_RGB(255,255,255)
#define CVEX_GRAY10 CV_RGB(10,10,10)
#define CVEX_GRAY20 CV_RGB(20,20,20)
#define CVEX_GRAY30 CV_RGB(10,30,30)
#define CVEX_GRAY40 CV_RGB(40,40,40)
#define CVEX_GRAY50 CV_RGB(50,50,50)
#define CVEX_GRAY60 CV_RGB(60,60,60)
#define CVEX_GRAY70 CV_RGB(70,70,70)
#define CVEX_GRAY80 CV_RGB(80,80,80)
#define CVEX_GRAY90 CV_RGB(90,90,90)
#define CVEX_GRAY100 CV_RGB(100,100,100)
#define CVEX_GRAY110 CV_RGB(101,110,110)
#define CVEX_GRAY120 CV_RGB(120,120,120)
#define CVEX_GRAY130 CV_RGB(130,130,140)
#define CVEX_GRAY140 CV_RGB(140,140,140)
#define CVEX_GRAY150 CV_RGB(150,150,150)
#define CVEX_GRAY160 CV_RGB(160,160,160)
#define CVEX_GRAY170 CV_RGB(170,170,170)
#define CVEX_GRAY180 CV_RGB(180,180,180)
#define CVEX_GRAY190 CV_RGB(190,190,190)
#define CVEX_GRAY200 CV_RGB(200,200,200)
#define CVEX_GRAY210 CV_RGB(210,210,210)
#define CVEX_GRAY220 CV_RGB(220,220,220)
#define CVEX_GRAY230 CV_RGB(230,230,230)
#define CVEX_GRAY240 CV_RGB(240,240,240)
#define CVEX_GRAY250 CV_RGB(250,250,250)
#define CVEX_BLACK CV_RGB(0,0,0)

#define CVEX_RED CV_RGB(255,0,0)
#define CVEX_GREEN CV_RGB(0,255,0)
#define CVEX_BLUE CV_RGB(0,0,255)
#define CVEX_ORANGE CV_RGB(255,100,0)
#define CVEX_YELLOW CV_RGB(230,230,0)
#define CVEX_MAGENDA CV_RGB(230,0,230)
#define CVEX_CYAN CV_RGB(0,230,230)

//size define
#define CVEX_VGA cvSize(640,480)
#define CVEX_VGA_COLOR cvSize(640,480),8,3
#define CVEX_QVGA cvSize(320,240)
#define CVEX_QVGA_COLOR cvSize(320,240),8,3
#define CVEX_XGA cvSize(1024,768)
#define CVEX_XGA_COLOR cvSize(1024,768),8,3

#define CVEX_256X256 cvSize(256,256)
#define CVEX_512X512 cvSize(512,512)

void cvexShowMatrix(CvMat* src);

void cvexSaveMatrix(CvMat* src, FILE* file);

void cvexWarpShift(IplImage* src, IplImage* dest, int shiftx, int shifty);

IplImage* cvexLoadImage(const char *format, ...);

void cvexSaveImage(IplImage* save, const char *format, ...);

enum {
    CVEX_CONNECT_HORIZON=0,
    CVEX_CONNECT_VERTICAL
};

IplImage* cvexCloneGray(IplImage* src);

IplImage* cvexConnect(IplImage* src1, IplImage* src2,int mode);

IplImage* cvexConnectMulti(
    IplImage** src, int arrayWidth, int arrayHeight,unsigned char* _mask);

void cvexPutText(
    IplImage* render, char* text, CvPoint orign,
    double amp=1.0, double shear=0.0,
    int fontType=CV_FONT_HERSHEY_SIMPLEX, int thickcness=1);

#endif
