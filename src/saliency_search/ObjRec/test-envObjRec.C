/*! @file ObjRec/test-envObjRec.C test various obj rec alg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-envObjRec.C $
// $Id: test-envObjRec.C 13716 2010-07-28 22:07:03Z itti $
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
#include "Image/fancynorm.H"
#include "Transport/FrameInfo.H"
#include "Raster/Raster.H"
#include "Raster/GenericFrame.H"
#include "GUI/DebugWin.H"
#include "Neuro/EnvVisualCortex.H"
#include "Media/FrameSeries.H"
#include "Util/Timer.H"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define OPENCV_HOUGH 1    // opencv hough transform
//#define SALIENCY_HOUGH 1    // opencv hough transform
#define ORI_QUE 1           //use orientation que
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ModelManager *mgr;

bool debug = 0;
Timer timer;
int smap_level = 0;

#define halfPi ((float)(CV_PI*0.5))
#define Pi     ((float)CV_PI)
#define a0  0 /*-4.172325e-7f*/   /*(-(float)0x7)/((float)0x1000000); */
#define a1 1.000025f        /*((float)0x1922253)/((float)0x1000000)*2/Pi; */
#define a2 -2.652905e-4f    /*(-(float)0x2ae6)/((float)0x1000000)*4/(Pi*Pi); */
#define a3 -0.165624f       /*(-(float)0xa45511)/((float)0x1000000)*8/(Pi*Pi*Pi); */
#define a4 -1.964532e-3f    /*(-(float)0x30fd3)/((float)0x1000000)*16/(Pi*Pi*Pi*Pi); */
#define a5 1.02575e-2f      /*((float)0x191cac)/((float)0x1000000)*32/(Pi*Pi*Pi*Pi*Pi); */
#define a6 -9.580378e-4f    /*(-(float)0x3af27)/((float)0x1000000)*64/(Pi*Pi*Pi*Pi*Pi*Pi); */

#define _sin(x) ((((((a6*(x) + a5)*(x) + a4)*(x) + a3)*(x) + a2)*(x) + a1)*(x) + a0)
#define _cos(x) _sin(halfPi - (x))
#define NUMANGLE 180
float tabSin[NUMANGLE];
float tabCos[NUMANGLE];


void display(Image<PixRGB<byte> > &leftImg,
    const Image<PixRGB<byte> > &leftSmap,
    const Image<PixRGB<byte> > &hough,
    const Point2D<int> &leftWinner,
    const byte maxVal,
    const Point2D<int> &targetLoc,
    nub::ref<OutputFrameSeries> ofs);


int biasProc(const char *tagName,
    env_size_t clev,
    env_size_t slev,
    struct env_image *cmap,
    const struct env_image *center,
    const struct env_image *surround)
{
  //intg32* src = cmap->pixels;

  ENV_SHOWIMG(cmap, 512, 512);

  //uint foveaWidth = (FSIZE >> smap_level)/2;
  //uint foveaHeight = (FSIZE >> smap_level)/2;

  //struct env_image *outImg = new struct env_image;
 // env_img_init(outImg, cmap->dims);
  //intg32* outSrc = outImg->pixels;

  return 0;
}

void putLine(Image<PixRGB<byte> > &img, float r, float m)
{
  double a = cos(m), b = sin(m);
  double x0 = a*r, y0 = b*r;
  Point2D<int> p1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
  Point2D<int> p2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));

  drawLine(img, p1, p2, PixRGB<byte>(255,0,0));

}

Image<float> houghTrans(const Image<float> &edge, const Image<float> &ori,
    const Image<float> &smap)
{

  int w = edge.getWidth();
  int h = edge.getHeight();
  float theta = CV_PI/180;
  int rho = 1;
  int numangle = int(CV_PI/theta);
  int numrho = int(((w+h)*2 +1) / rho);

  Image<float> acc(numrho+2,numangle+2,ZEROS); //the accumulator


  Image<float>::const_iterator magPtr = edge.begin();
  Image<float>::iterator accPtr = acc.beginw();

#ifdef ORI_QUE
  Image<float>::const_iterator oriPtr = ori.begin();
#endif

  for(int y=0; y<h; y++)
    for(int x=0; x<w; x++)
    {
      float val = magPtr[y*w+x];
      if (val > 100)
      {
        int minOri = 0, maxOri = numangle;
#ifdef ORI_QUE
        int oriQue = int((oriPtr[y*w+x]*180/M_PI));
        if (oriQue < 0) oriQue += 180;

        minOri = (oriQue - 25); if (minOri < 0) minOri = 0;
        maxOri = (oriQue + 25); if (maxOri > numangle) maxOri = numangle-1;
#endif

        for(int m=minOri; m<maxOri; m++)
        {
          int r = int(x*tabCos[m] + y*tabSin[m]);
          r += (numrho -1) /2;
          accPtr[(m+1) * (numrho+2) + r+1] += val;
        }
      }
    }

  return acc;

}

Image<PixRGB<byte> > showHough(const Image<float> &acc, const Image<float> &mag)
{

  Image<float> tmpAcc = acc; //(acc.getDims(), ZEROS);
  Image<PixRGB<byte> > retImg = mag;
 // int thresh = 1;
  int numrho = acc.getWidth()-2;
 // int numangle = acc.getHeight()-2;

  Point2D<int> maxPos;
  float maxVal;

  /*//Image<float>::const_iterator accPtr = acc.begin();
  //find local maximums
  for(int r=0; r<numrho; r++)
    for(int m=0; m<numangle; m++)
    {
      int base = (m+1)* (numrho+2) + r+1;
      if (acc[base] > thresh &&
          acc[base] > acc[base-1] && acc[base] >= acc[base+1] &&
          acc[base] > acc[base - numrho - 2] && acc[base] >= acc[base + numrho + 2])
      {
        tmpAcc[base] = acc[base];
        //float rho = (r - (numrho - 1)*0.5f);
        //float theta = (m * CV_PI/180);
        //printf("Base %f\n", acc[base]);
        //putLine(retImg, rho, theta);

      }
    }
  */

  for(int i=0; i<10; i++)
  {
    findMax(tmpAcc, maxPos, maxVal);
    int r = maxPos.i;
    int m = maxPos.j;
    float rho = (r - (numrho - 1)*0.5f);
    float theta = (m * CV_PI/180);
    putLine(retImg, rho, theta);

    drawDisk(tmpAcc, Point2D<int>(r,m), 5 , float(0)); //IOR
   // SHOWIMG(tmpAcc);
   // SHOWIMG(retImg);
  }


  return retImg;
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
  else
    LINFO("THreshold is %i", t);

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

float entropy (float *h, int a, float p)
{
        if (h[a] > 0.0 && p>0.0)
          return -(h[a]/p * (float)log((double)(h[a])/p));
        return 0.0;
}


Image<float> segment(const Image<float> &img)
{
  Image<float> retImg = img;
  float hist[300];
  float pt[300];
  float F[300];

  inplaceNormalize(retImg, 0.0F, 255.0F);

  for(int i=0; i<256; i++)
  {
    hist[i] = 0; //set histogram to 0
    pt[i] = 0; //set prob to 0
  }

  for (int y=0; y<retImg.getHeight(); y++)
    for(int x=0; x<retImg.getWidth(); x++)
    {
      int val = (int)retImg.getVal(x,y);
      hist[val]++;
    }

  //nomalize into a distribution
  for (int i=0; i<256; i++)
    hist[i] = (float)hist[i]/(float)retImg.getSize();

  //compute factors
  pt[0] = hist[0];
  for(int i=1; i<256; i++)
    pt[i] = pt[i-1] + hist[i];

  int t = -1;

  for(int i=0; i<256; i++)
  {
    float Hb = 0, Hw = 0;
    for(int j=0; j<256; j++)
      if (j<=i)
        Hb += entropy(hist, j, pt[i]);
      else
        Hw += entropy(hist, j, 1.0-pt[i]);
    F[i] = Hb+Hw;
    if (i>0 && F[i] > F[t]) t = i;
  }
  if (t < 0)
    LINFO("ERROR");
  else
    LINFO("THreshold is %i", t);

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



int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  mgr = new ModelManager("Test ObjRec");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(*mgr));
  mgr->addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*mgr));
  mgr->addSubComponent(ofs);

  nub::ref<EnvVisualCortex> evc(new EnvVisualCortex(*mgr));
  mgr->addSubComponent(evc);

  mgr->exportOptions(MC_RECURSE);
  //mgr->setOptionValString(&OPT_EvcMaxnormType, "None");
  //mgr->setOptionValString(&OPT_EvcLevelSpec, "0,2,5,6,0");
  //mgr->setOptionValString(&OPT_EvcLevelSpec, "0,4,1,4,0");
  //mgr->setOptionValString(&OPT_EvcLevelSpec, "0,2,3,4,2");
  //mgr->setOptionValString(&OPT_EvcLevelSpec, "0,1,8,8,0");


  //evc->setIweight(0);
  evc->setFweight(0);
  evc->setMweight(0);
  evc->setCweight(0);
  evc->setOweight(0);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "image", 0, 0) == false)
    return 1;

  mgr->start();

  // do post-command-line configs:

  //start streaming
  ifs->startStream();

  //build the sin/cos tables
  float ang;
  int i;
  for(ang =0,i=0; i < 180; ang += CV_PI/180, i++)
  {
    tabSin[i] = (float)(sin(ang));
    tabCos[i] = (float)(cos(ang));
  }

  while(1)
  {
    Image< PixRGB<byte> > inputImg;
    const FrameState is = ifs->updateNext();
    LINFO("Frame %i\n", ifs->frame());
    if (is == FRAME_COMPLETE)
      break;

    //grab the images
    GenericFrame input = ifs->readFrame();
    if (!input.initialized())
      break;
    inputImg = input.asRgb();

    int fw = 100;
    Image<PixRGB<byte> > fimg = crop(inputImg,
        Point2D<int>((inputImg.getWidth()/2)-(fw/2), (inputImg.getHeight()/2)-(fw/2)), Dims(fw,fw));
  //  SHOWIMG(rescale(inputImg, 512, 512));

    //evc->setSubmapPostNormProc(&biasProc);
    //evc->setSubmapPreProc(&biasProc);
    //evc->setSubmapPostProc(NULL);

    fimg = rescale(fimg, inputImg.getDims());
    evc->input(fimg);

    Image<float> vcxMap = evc->getVCXmap();
    Image<float> lum = luminance(fimg);


    vcxMap = rescale(vcxMap, inputImg.getDims());
    lum = rescale(lum, inputImg.getDims());

    Image<float> mag, ori;
    gradientSobel(lum, mag, ori, 3);
    //inplaceNormalize(vcxMap, 0.0F, 255.0F);
    //inplaceNormalize(mag, 0.0F, 255.0F);
    mag = (vcxMap);


    Image<float> seg = segment(mag);

    gradientSobel(lum, mag, ori, 3);
    lum =mag;

    Image<PixRGB<byte> > salHoughDisp;
    Image<PixRGB<byte> > opencvHoughDisp;

    //calculate the hough
    Image<float> acc = houghTrans(mag, ori, vcxMap);

    //show the lines
    salHoughDisp = showHough(acc, mag);



#ifdef SALIENCY_HOUGH
    Image<float> mag, ori;
    gradientSobel(vcxMap, mag, ori, 3);
    mag *= vcxMap; //weight the orientations by the saliency
    inplaceNormalize(mag, 0.0F, 255.0F);

    //calculate the hough
    Image<float> acc = houghTrans(mag, ori, vcxMap);

    //show the lines
    salHoughDisp = showHough(acc, mag);
#endif

#ifdef OPENCV_HOUGH

    //Opencv hough
    inplaceNormalize(lum, 0.0F, 255.0F);
    CvMemStorage* storage = cvCreateMemStorage(0);
    IplImage *dst = cvCreateImage( cvGetSize(img2ipl(lum)), 8, 1 );
    cvCanny( img2ipl((Image<byte>)lum), dst, 150, 200, 3 );

    Image<float> edge = ipl2float(dst);
    opencvHoughDisp = edge;

    /*CvSeq *lines = cvHoughLines2(dst, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 50 , 0, 0);

    for(int i = 0; i < MIN(lines->total,10); i++ )
    {
    ccb
      float rho = line[0];
      float theta = line[1];
      CvPoint pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));

      drawLine(opencvHoughDisp, Point2D<int>(pt1.x,pt1.y), Point2D<int>(pt2.x, pt2.y), PixRGB<byte>(255,0,0));
    }*/


    CvSeq *lines = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 10, 5, 1 );
    for( i = 0; i < lines->total; i++ )
    {
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
        drawLine(opencvHoughDisp, Point2D<int>(line[0].x,line[0].y), Point2D<int>(line[1].x, line[1].y), PixRGB<byte>(255,0,0));
    }

#endif


    display(inputImg, vcxMap, lum, Point2D<int>(-1, -1), 2, Point2D<int>(-1, -1), ofs);


  }



  // stop all our ModelComponents
  mgr->stop();

  return 0;

}


void display(Image<PixRGB<byte> > &leftImg,
    const Image<PixRGB<byte> > &leftSmap,
    const Image<PixRGB<byte> > &hough,
    const Point2D<int> &leftWinner,
    const byte maxVal,
    const Point2D<int> &targetLoc,
    nub::ref<OutputFrameSeries> ofs)
{
  static int avgn = 0;
  static uint64 avgtime = 0;
  static double fps = 0;
  char msg[255];

  Image<PixRGB<byte> > outDisp(leftImg.getWidth()*2,leftImg.getHeight()*2+20, ZEROS);


  //Left Image
  inplacePaste(outDisp, leftImg, Point2D<int>(0,0));
  inplacePaste(outDisp, leftSmap, Point2D<int>(0,leftImg.getHeight()));
  inplacePaste(outDisp, hough, Point2D<int>(leftImg.getWidth(),leftImg.getHeight()));


  //calculate fps
  avgn++;
  avgtime += timer.getReset();
  if (avgn == 20)
  {
    fps = 1000.0F / double(avgtime) * double(avgn);
    avgtime = 0;
    avgn = 0;
  }

  sprintf(msg, "%.1ffps ", fps);

  Image<PixRGB<byte> > infoImg(leftImg.getWidth()*2, 20, NO_INIT);
  writeText(infoImg, Point2D<int>(0,0), msg,
        PixRGB<byte>(255), PixRGB<byte>(127));
  inplacePaste(outDisp, infoImg, Point2D<int>(0,leftImg.getHeight()*2));

  ofs->writeRGB(outDisp, "output", FrameInfo("output", SRC_POS));
}


