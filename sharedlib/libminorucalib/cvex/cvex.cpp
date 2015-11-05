#include "mcv.h"

void cvexShowMatrix(
    CvMat* src)
{
    for (int j=0;j<src->rows;j++) {
        for (int i=0;i<src->cols;i++) {
            printf("%05.5f  ",cvmGet(src,j,i));
        }
        printf("\n");
    }
    printf("\n");
}

void cvexSaveMatrix(
    CvMat* src,
    FILE * file)
{
    for (int j=0;j<src->rows;j++) {
        for (int i=0;i<src->cols;i++) {
            fprintf(file, "%05.5f  ",cvmGet(src,j,i));
        }
        fprintf(file,"\n");
    }
    fprintf(file,"\n");
}


void cvexWarpShift(
    IplImage* src,
    IplImage* dest,
    int shiftx,
    int shifty)
{
    IplImage* _dest;
    if (src==dest) {
        _dest=cvCreateImage(cvGetSize(src),src->depth,src->nChannels);
    }
    else {
        _dest = dest;
    }
    cvZero(_dest);

    if (shiftx>=0 &&shifty>=0) {
        cvSetImageROI( src, cvRect( 0, 0, src->width-shiftx, src->height-shifty ) );
        cvSetImageROI( _dest, cvRect( shiftx, shifty, src->width, src->height ) );
    }
    else if(shiftx>=0 &&shifty<0) {
        cvSetImageROI( src, cvRect( 0, -shifty, src->width-shiftx, src->height ) );
        cvSetImageROI( _dest, cvRect( shiftx, 0, src->width, src->height+shifty ) );
    }
    else if(shiftx<0 &&shifty<0) {
        cvSetImageROI( src, cvRect( -shiftx, -shifty, src->width, src->height ) );
        cvSetImageROI( _dest, cvRect( 0, 0, src->width+shiftx, src->height+shifty ) );
    }
    else if(shiftx<0 &&shifty>=0) {
        cvSetImageROI( src, cvRect(-shiftx, 0, src->width, src->height-shifty ) );
        cvSetImageROI( _dest, cvRect( 0, shifty, src->width+shiftx, src->height ) );
    }

    cvCopy(src, _dest);

    cvResetImageROI( src );
    cvResetImageROI( _dest );

    if (src==dest) {
        cvCopy(_dest,dest);
        cvReleaseImage(&_dest);
    }
}

IplImage* cvexLoadImage(
    const char *format, ...)
{
    char buff[255]; 

    va_list ap;
    va_start(ap, format);
    vsprintf(buff, format, ap);
    va_end(ap);

    return cvLoadImage(buff,1);
}

void cvexSaveImage(
    IplImage* save,
    const char *format, ...)
{
    char buff[255]; 

    va_list ap;
    va_start(ap, format);
    vsprintf(buff, format, ap);
    va_end(ap);
    cvSaveImage(buff,save);
}

IplImage* cvexCloneGray(
    IplImage* src)
{
    IplImage* ret = cvCreateImage(cvGetSize(src),8,1);
    cvCvtColor(src,ret,CV_BGR2GRAY);
    return ret;
}


IplImage* cvexConnect(
    IplImage* src1,
    IplImage* src2,
    int mode)
{
    IplImage* ret;

    switch(mode) {
        case CVEX_CONNECT_HORIZON:
        default:
            ret =
                cvCreateImage(
                    cvSize(
                        src1->width+src2->width,
                        MAX(src1->height,src2->height)),8,3);
            cvSetImageROI( ret, cvRect( 0, 0, src1->width, src1->height ) );
            cvCopy(src1,ret);

            cvSetImageROI( ret, cvRect( src1->width, 0, src2->width, src2->height ) );
            cvCopy(src2,ret);
            cvResetImageROI( ret );
            break;
        case CVEX_CONNECT_VERTICAL:
            ret =
                cvCreateImage(
                    cvSize(
                        MAX(src1->width,src2->width),
                        src1->height+src2->height),8,3);
            cvSetImageROI( ret, cvRect( 0, 0, src1->width, src1->height ) );
            cvCopy(src1,ret);

            cvSetImageROI( ret, cvRect( 0, src1->height, src2->width, src2->height ) );
            cvCopy(src2,ret);
            cvResetImageROI( ret );
            break;
    }
    return ret;
}

IplImage* cvexConnectMulti(
    IplImage** src,
    int arrayWidth,
    int arrayHeight,
    unsigned char* _mask)
{
    IplImage* ret;
    uchar* mask = new uchar[arrayWidth*arrayHeight];
    if (_mask!=NULL) {
        memcpy(mask,_mask,arrayWidth*arrayHeight);
    }
    else {
        memset(mask,255,arrayWidth*arrayHeight);
    }

    ret =
        cvCreateImage(cvSize(src[0]->width*arrayWidth,src[0]->height*arrayHeight),8,3);
    for (int j=0;j<arrayHeight;j++) {
        for(int i=0;i<arrayWidth;i++) {
            if (mask[arrayWidth*j+i] != 0) {
                cvSetImageROI(
                    ret, cvRect( src[0]->width*i, src[0]->height*j,
                    src[0]->width, src[0]->height ) );
                cvCopy(src[arrayWidth*j+i],ret);
                cvResetImageROI( ret );
            }
            else {
                cvSetImageROI(
                    ret, cvRect( src[0]->width*i, src[0]->height*j,
                    src[0]->width, src[0]->height ) );
                cvZero(ret);
                cvResetImageROI( ret );
            }
        }
    }
    delete[] mask;
    return ret;
}

void cvexPutText(
    IplImage* render,
    char* text,
    CvPoint orign,
    CvScalar color,
    double amp,
    double shear,
    int fontType,
    int thickness)
{
    CvFont font;
    cvInitFont(&font,fontType,amp,amp,shear,thickness);
    cvPutText(render,text,orign,&font, color);
}

