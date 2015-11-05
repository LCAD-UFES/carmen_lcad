#include "image_utils.h"

double image_utils_get_slope(CvPoint A, CvPoint B, int image_height)
{
  int A_y = image_height - A.y;
  int B_y = image_height - B.y;
  int dX = abs(A.x - B.x);
  int dY = abs(A_y - B_y);
  double theta_radians = atan2(dY, dX);

  return theta_radians;
}

// Return the image points that are at the line segment
CvPoint *image_utils_get_line_points(IplImage *img, CvPoint A, CvPoint B, int *n_points)
{
  int i;

  CvLineIterator iterator;
  *n_points = cvInitLineIterator(img, A, B, &iterator, 8, 0);
  CvPoint* line_points = (CvPoint*)malloc((*n_points) * sizeof(CvPoint));

  for(i = 0; i < *n_points; i++)
  {
    void *p_0 = &img->imageData[0];//get the memory address of the image data
    void *p_c = &iterator.ptr[0];//get the memory address of the current point at the line segment

    // computes the memory offset (in bytes) between image data and current point at the line segment
    int offset = (char *)p_c - (char *)p_0;

    // since 'offset = y * widthStep + x * nChannels', compute the current point at the line segment
    line_points[i].x = (offset % img->widthStep) / img->nChannels;
    line_points[i].y = offset / img->widthStep;

    CV_NEXT_LINE_POINT(iterator);
  }

  return line_points;
}

void image_utils_equalize(IplImage *src, IplImage *dst)
{
  IplImage *channels[3];
  int c;

  cvCopy(src, dst, NULL);

  for (c = 0; c < 3; c++)
  {
    channels[c] = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
    cvSetImageCOI(src, c + 1);
    cvCopy(src, channels[c], NULL);
    cvSetImageCOI(src, 0);
    cvEqualizeHist(channels[c], channels[c]);
  }

  cvMerge(channels[0], channels[1], channels[2], NULL, dst);

  for (c = 0; c < 3; c++)
  {
    cvReleaseImage(&channels[c]);
  }
}

int min(int x, int y)
{
  if (x < y)
    return x;
	
  return y;
}

CvRect image_utils_get_rect(IplImage *image, CvPoint A, CvPoint B)
{
  int x0 = min(A.x, B.x);
  int y0 = min(image->height - A.y, image->height - B.y);
  int width = abs(A.x - B.x);
  int height = abs(A.y - B.y);

  return cvRect(x0, y0, width, height);
}
