/*! @file ObjRec/test-gbr.C test for geon based recognition */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-gbr.C $
// $Id: test-gbr.C 13716 2010-07-28 22:07:03Z itti $
//


#include "Image/OpenCVUtil.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ColorOps.H"
#include "Image/Transforms.H"
#include "Image/MathOps.H"
#include "Image/Kernels.H"
#include "Image/fancynorm.H"
#include "Image/Layout.H"
#include "Transport/FrameInfo.H"
#include "Raster/Raster.H"
#include "Raster/GenericFrame.H"
#include "GUI/DebugWin.H"
#include "Neuro/EnvVisualCortex.H"
#include "Neuro/getSaliency.H"
#include "Media/FrameSeries.H"
#include "Util/Timer.H"
#include "RCBot/Motion/MotionEnergy.H"
#include "Neuro/BeoHeadBrain.H"
#include "Learn/Bayes.H"

#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define OPENCV_HOUGH 1    // opencv hough transform
//#define SALIENCY_HOUGH 1    // opencv hough transform
#define ORI_QUE 1           //use orientation que
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Image<byte> watershed(Image<float> &img)
{

  Image<float> ret(img.getDims(), ZEROS);
  Image<byte> Data2(img.getDims(), ZEROS);
  int Ydim = img.getHeight();
  int Xdim = img.getWidth();

  /* Calculate gradient magnitude image */
  if (false)
  {
    for (int y = 1; y < (img.getHeight() - 1); y++)
      for (int x = 1; x < (img.getWidth() - 1); x++)
      {
        float Dx = (img.getVal(x+1, y + 1) + 2 * img.getVal(x+1, y) + img.getVal(x+1, y-1)
            - img.getVal(x-1, y+1) - 2 * img.getVal(x-1, y) - img.getVal(x-1, y-1)) / 8;
        float Dy = (img.getVal(x+1, y + 1) + 2 * img.getVal(x, y+1) + img.getVal(x-1, y+1)
            - img.getVal(x+1, y-1) - 2 * img.getVal(x, y-1) - img.getVal(x-1, y-1)) / 8;

        ret.setVal(x,y,  (float) sqrt((double) (Dx * Dx + Dy * Dy)));
      }

    //Fix borders
    for (int y = 1; y < (Ydim - 1); y++)
      for (int x = 1; x < (Xdim - 1); x++)
        img.setVal(x,y,  ret.getVal(x,y));

    for (int x = 0; x < Xdim; x++)
    {
      img.setVal(x, 0, img.getVal(x,1));
      img.setVal(x, Ydim-1, img.getVal(x, Ydim - 2));
    }
    for (int y = 0; y < Ydim; y++)
    {
      img.setVal(0,y, img.getVal(1,y));
      img.setVal(Xdim-1, y,  img.getVal(Xdim-2, y));
    }
  }


  float min, max;
  getMinMax(img, min, max);

  int mask = 2;
  for(int y=0; y<Ydim; y++)
    for (int x=0; x<Xdim; x++)
    {
      if (img.getVal(x,y) == min)
      {
        Data2.setVal(x,y,mask);
      }
    }


  return Data2;


}

float w (float *p, int k)
{
        int i;
        float x=0.0;

        for (i=1; i<=k; i++) x += p[i];
        return x;
}

float u (float *p, int k)
{
        int i;
        float x=0.0;

        for (i=1; i<=k; i++) x += (float)i*p[i];
        return x;
}

float nu (float *p, int k, float ut, float vt)
{
        float x, y;

        y = w(p,k);
        x = ut*y - u(p,k);
        x = x*x;
        y = y*(1.0-y);
        if (y>0) x = x/y;
         else x = 0.0;
        return x/vt;
}

Image<float> segmentProb(const Image<float> &img)
{
  Image<float> retImg = img;
  int hist[260];
  float p[260];

  inplaceNormalize(retImg, 0.0F, 255.0F);

  for(int i=0; i<260; i++)
  {
    hist[i] = 0; //set histogram to 0
    p[i] = 0; //set prob to 0
  }

  for (int y=0; y<retImg.getHeight(); y++)
    for(int x=0; x<retImg.getWidth(); x++)
    {
      int val = (int)retImg.getVal(x,y);
      hist[val+1]++;
    }

  //nomalize into a distribution
  for (int i=1; i<=256; i++)
    p[i] = (float)hist[i]/(float)retImg.getSize();

  //find the global mean
  float ut = 0.0;
  for(int i=1; i<=256; i++)
    ut += (float)i*p[i];

  //find the global variance
  float vt = 0.0;
  for(int i=1; i<=256; i++)
    vt += (i-ut)*(i-ut)*p[i];

  int j=-1, k=-1;
  for(int i=1; i<=256; i++)
  {
    if ((j<0) && (p[i] > 0.0)) j = i; //first index
    if (p[i] > 0.0) k = i; //last index
  }

  float z = -1.0;
  int m = -1;
  for (int i=j; i<=k; i++)
  {
    float y = nu(p,i,ut,vt); //compute nu

    if (y>=z)
    {
      z = y;
      m = i;
    }
  }

  int t = m;

  if (t < 0)
    LINFO("ERROR");

  //threshold
  for (int y=0; y<retImg.getHeight(); y++)
    for(int x=0; x<retImg.getWidth(); x++)
    {
      int val = (int)retImg.getVal(x,y);
      if (val < t)
        retImg.setVal(x,y,0);
      else
        retImg.setVal(x,y,255.0);
    }


  return retImg;
}

Image<float> integralImage(const Image<float> &img)
{

  Image<float> integImg(img.getDims(), ZEROS);

  int xDim = integImg.getWidth();
  int yDim = integImg.getHeight();


  float s[xDim];
  for (int i=0; i<xDim; i++) s[i] = 0;

  for(int y=0; y<yDim; y++)
    for(int x=0; x<xDim; x++)
    {
      float ii = x > 0 ? integImg.getVal(x-1, y) : 0;
      s[x] += img.getVal(x,y);
      integImg.setVal(x,y, ii+s[x]);
    }

  return integImg;


}

Image<float> getHaarFeature(Image<float> &integImg, int i)
{

    Image<float> fImg(integImg.getDims(), ZEROS);

    /*int w = 2, h = 4;

    for(int y=0; y<fImg.getHeight()-2*w; y++)
      for(int x=0; x<fImg.getWidth()-h; x++)
      {
        float left  = integImg.getVal(x+w,y+h) + integImg.getVal(x,y) -
          (integImg.getVal(x+w,y) + integImg.getVal(x,y+h));
        float right = integImg.getVal(x+(2*w),y+h) + integImg.getVal(x+w,y) -
          (integImg.getVal(x+(2*w),y) + integImg.getVal(x+w,y+h));

        float top  = integImg.getVal(x,y) + integImg.getVal(x+h,y+w) -
          (integImg.getVal(x+h,y) + integImg.getVal(x,y+w));
        float bottom = integImg.getVal(x,y+w) + integImg.getVal(x+h,y+(2*w)) -
          (integImg.getVal(x+h,y+w) + integImg.getVal(x,y+(2*w)));


        fImg.setVal(x,y, fabs(left-right) + fabs(top-bottom));
      }*/

    int c = 6+i, s = 8+i;

    int x = 320/2, y=240/2;

    Rectangle rect(Point2D<int>(x,y),Dims(s,s));
    drawRect(fImg, rect, float(255.0));
    //for(int y=0; y<fImg.getHeight()-s; y++)
    //  for(int x=0; x<fImg.getWidth()-s; x++)
      {
        int d = (s-c)/2;
        float center  = integImg.getVal(x+d,y+d) + integImg.getVal(x+d+c,y+d+c) -
          (integImg.getVal(x+d,y+d+c) + integImg.getVal(x+d+c,y+d));
        float surround  = integImg.getVal(x,y) + integImg.getVal(x+s,y+s) -
          (integImg.getVal(x+s,y) + integImg.getVal(x,y+s));

        center /= c*2;
        surround /= s*2;
        //fImg.setVal(x,y, center-surround);
        //printf("%i %f\n", i, center-surround);
      }


    return fImg;

}

Image<float> centerSurround(Image<float> &integImg)
{

    Image<float> objImg(integImg.getDims(), ZEROS);

    for(int y=0; y<objImg.getHeight()-20; y++)
      for(int x=0; x<objImg.getWidth()-20; x++)
      {
        int  c, s;
        float center, surround;
        c = 3+(10/2);
        s = 6+10;

        int d = (s-c)/2;
        center  = integImg.getVal(x+d,y+d) + integImg.getVal(x+d+c,y+d+c) -
          (integImg.getVal(x+d,y+d+c) + integImg.getVal(x+d+c,y+d));
        surround  = integImg.getVal(x,y) + integImg.getVal(x+s,y+s) -
          (integImg.getVal(x+s,y) + integImg.getVal(x,y+s)) - center;

        center /= (c*c);
        surround /= ((s*s) - (c*c));

        float val = fabs(center-surround);
        objImg.setVal(x,y, val);
      }

    return objImg;

}

// various tracking parameters (in seconds)
const double MHI_DURATION = 1;
const double MAX_TIME_DELTA = 0.5;
const double MIN_TIME_DELTA = 0.05;
// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 4;

// ring image buffer
IplImage **buf = 0;
int last = 0;

// temporary images
IplImage *mhi = 0; // MHI
IplImage *orient = 0; // orientation
IplImage *mask = 0; // valid orientation mask
IplImage *segmask = 0; // motion segmentation map
CvMemStorage* storage = 0; // temporary storage

// parameters:
//  img - input video frame
//  dst - resultant motion picture
//  args - optional parameters
Rectangle  update_mhi( IplImage* img, IplImage* dst, int diff_threshold )
{
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
    CvSize size = cvSize(img->width,img->height); // get current frame size
    int i, idx1 = last, idx2;
    IplImage* silh;
    CvSeq* seq;
    CvRect comp_rect;
    CvRect maxComp_rect = cvRect(0,0,0,0);
    double count;
    //double angle;
    CvPoint center;

    Rectangle rect;

    // allocate images at the beginning or
    // reallocate them if the frame size is changed
    if( !mhi || mhi->width != size.width || mhi->height != size.height ) {
        if( buf == 0 ) {
            buf = (IplImage**)malloc(N*sizeof(buf[0]));
            memset( buf, 0, N*sizeof(buf[0]));
        }

        for( i = 0; i < N; i++ ) {
            cvReleaseImage( &buf[i] );
            buf[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
            cvZero( buf[i] );
        }
        cvReleaseImage( &mhi );
        cvReleaseImage( &orient );
        cvReleaseImage( &segmask );
        cvReleaseImage( &mask );

        mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        cvZero( mhi ); // clear MHI at the beginning
        orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );
    }

    cvCvtColor( img, buf[last], CV_BGR2GRAY ); // convert frame to grayscale

    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;

    silh = buf[idx2];
    cvAbsDiff( buf[idx1], buf[idx2], silh ); // get difference between frames

    cvThreshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
    cvUpdateMotionHistory( silh, mhi, timestamp, MHI_DURATION ); // update MHI

    // convert MHI to blue 8u image
    cvCvtScale( mhi, mask, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );
    cvZero( dst );
    cvCvtPlaneToPix( mask, 0, 0, 0, dst );

    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );

    if( !storage )
        storage = cvCreateMemStorage(0);
    else
        cvClearMemStorage(storage);

    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    seq = cvSegmentMotion( mhi, segmask, storage, timestamp, MAX_TIME_DELTA );

    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)

    float maxMotCount = 0;

    for(int i=0; i< seq->total; i++)
    {

      comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;

      // select component ROI
      cvSetImageROI( silh, comp_rect );
      cvSetImageROI( mhi, comp_rect );
      cvSetImageROI( orient, comp_rect );
      cvSetImageROI( mask, comp_rect );


      count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

      cvResetImageROI( mhi );
      cvResetImageROI( orient );
      cvResetImageROI( mask );
      cvResetImageROI( silh );

      float motCount = count + (comp_rect.width*comp_rect.height);
      if (motCount > maxMotCount)
                        {
              maxComp_rect = comp_rect;
                                maxMotCount = motCount;
                        }
                }

                if (maxMotCount > 0)
                {

                        // draw a clock with arrow indicating the direction
                        center = cvPoint( (maxComp_rect.x + maxComp_rect.width/2),
                                        (maxComp_rect.y + maxComp_rect.height/2) );

                        cvCircle( dst, center, cvRound(35), CV_RGB(255,0,0), 3, CV_AA, 0 );
    //  cvLine( dst, center, cvPoint( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
    //        cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );

                        rect = Rectangle(Point2D<int>(maxComp_rect.x, maxComp_rect.y), Dims(maxComp_rect.width, maxComp_rect.height));

    }

    return rect;
}

void findMinMax(const std::vector<double> &vec, double &min, double &max)
{
  max = vec[0];
  min = max;
  for (uint n = 1 ; n < vec.size() ; n++)
  {
    if (vec[n] > max) max = vec[n];
    if (vec[n] < min) min = vec[n];
  }
}

Image<PixRGB<byte> > showHist(const std::vector<double> &hist, int loc=0)
{
  int w = 256, h = 256;
  if (hist.size() > (uint)w) w = hist.size();

  if (hist.size() == 0) return Image<PixRGB<byte> >();

  int dw = w / hist.size();
  Image<byte> res(w, h, ZEROS);

  // draw lines for 10% marks:
  for (int j = 0; j < 10; j++)
    drawLine(res, Point2D<int>(0, int(j * 0.1F * h)),
             Point2D<int>(w-1, int(j * 0.1F * h)), byte(64));
  drawLine(res, Point2D<int>(0, h-1), Point2D<int>(w-1, h-1), byte(64));

  double minii, maxii;
  findMinMax(hist, minii, maxii);

   // uniform histogram
  if (maxii == minii) minii = maxii - 1.0F;

  double range = maxii - minii;

  for (uint i = 0; i < hist.size(); i++)
    {
      int t = abs(h - int((hist[i] - minii) / range * double(h)));

      // if we have at least 1 pixel worth to draw
      if (t < h-1)
        {
          for (int j = 0; j < dw; j++)
            drawLine(res,
                     Point2D<int>(dw * i + j, t),
                     Point2D<int>(dw * i + j, h - 1),
                     byte(255));
          //drawRect(res, Rectangle::tlbrI(t,dw*i,h-1,dw*i+dw-1), byte(255));
        }
    }
  return res;
}

void smoothHist(std::vector<double> &hist)
{
  const uint siz = hist.size();
  float vect[siz];

  for (uint n = 0 ; n < siz ; n++)
  {
    float val0 = hist[ (n-1+siz) % siz ];
    float val1 = hist[ (n  +siz) % siz ];
    float val2 = hist[ (n+1+siz) % siz ];

    vect[n] = 0.25F * (val0 + 2.0F*val1 + val2);
  }

  for (uint n = 0 ; n < siz ; n++) hist[n] = vect[n];
}

void normalizeHist(std::vector<double> &hist, double high, double low)
{

  double oldmin, oldmax;
  findMinMax(hist, oldmin, oldmax);

  float scale = float(oldmax) - float(oldmin);
  //if (fabs(scale) < 1.0e-10F) scale = 1.0; // image is uniform
  const float nscale = (float(high) - float(low)) / scale;

  for(uint i=0; i<hist.size(); i++)
  {
    hist[i] = low + (float(hist[i]) - float(oldmin)) * nscale ;
  }
}

void normalizeHist(std::vector<double> &hist)
{
  double sum = 0;
  for(uint i=0; i<hist.size(); i++)
    if (!isnan(hist[i]) && !isinf(hist[i]) && exp(hist[i]) > 1.0e-5f) //if its a normal number
      sum += exp(hist[i]);

  for(uint i=0; i<hist.size(); i++)
    hist[i] = exp(hist[i])/sum;

}


void putLine(Image<PixRGB<byte> > &img, int x, int y, float a, int len)
{
  int x1 = int(cos(a)*(float)len);
  int y1 = int(sin(a)*(float)len);

  Point2D<int> p1(x, y);
  Point2D<int> p2(x+x1, y-y1);
  drawLine(img, p1, p2, PixRGB<byte>(0,255,0),2);

}

Image<PixRGB<byte> > generateInput(float &angle)
{
  Image<PixRGB<byte> > retImg(320,240,ZEROS);

  angle = M_PI*randomDouble();
  float angle2 = M_PI*randomDouble();

  putLine(retImg, retImg.getWidth()/2, retImg.getHeight()/2, angle, 50);

  putLine(retImg, retImg.getWidth()/2, retImg.getHeight()/2, angle+angle2, 50);


  angle = angle2;
  return retImg;


}


int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test ObjRec");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(*mgr));
  mgr->addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*mgr));
  mgr->addSubComponent(ofs);

  nub::ref<EnvVisualCortex> evc(new EnvVisualCortex(*mgr));
  mgr->addSubComponent(evc);

  //nub::ref<GetSaliency> smap(new GetSaliency(*mgr));
  //mgr->addSubComponent(smap);


  //mgr->setOptionValString(&OPT_EvcLevelSpec, "0,2,6,8,0");
  //mgr->setOptionValString(&OPT_EvcLevelSpec, "0,2,6,8,2");

  //evc->setIweight(0);
  //evc->setFweight(0);
  //evc->setMweight(0);
  //evc->setCweight(0);
  //evc->setOweight(0);

  mgr->exportOptions(MC_RECURSE);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false)
    return 1;
  mgr->start();

  // do post-command-line configs:

  //start streaming
  //ifs->startStream();

  //int i=0;


        initRandomNumbers();

  Bayes bayes(2, 360);
 // bayes.load("Edge.net");

  unsigned int frame=0;
        Point2D<int> target;
  while(1)
  {
    Image< PixRGB<byte> > inputImg;
    const FrameState is = ifs->updateNext();
    if (is == FRAME_COMPLETE)
      break;

    //grab the images
    GenericFrame input = ifs->readFrame();
    if (!input.initialized())
      break;
    inputImg = input.asRgb();

    inputImg = rescale(inputImg, 320, 240);
 //   inputImg = generateInput(trueAngle);


    //evc->input(inputImg);
    usleep(50000);
    ofs->writeRGB(inputImg, "Input", FrameInfo("Input", SRC_POS));
    Layout<PixRGB<byte> > outDisp;


    Image<byte> lum = luminance(inputImg);
    Image<float> mag, ori;
    gradientSobel(lum, mag, ori, 3);

    outDisp = vcat(outDisp, hcat(toRGB(Image<byte>(mag)), toRGB(Image<byte>(lum))));

    ofs->writeRgbLayout(outDisp, "Edges", FrameInfo("Edges", SRC_POS));

    frame++;
  }

  // stop all our ModelComponents
  mgr->stop();

  return 0;

}
