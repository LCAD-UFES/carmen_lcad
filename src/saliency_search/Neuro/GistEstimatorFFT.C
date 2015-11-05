/*!@file Neuro/GistEstimatorFFT.C Extract gist of image using the Fourier Transform */

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
// Primary maintainer for this file: Christian Siagian <siagian at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorFFT.C $
// $Id: GistEstimatorFFT.C 13065 2010-03-28 00:01:00Z itti $
//

// ######################################################################
/*! Extract gist of image using the Fourier transform                  */

#ifndef NEURO_GISTESTIMATORFFT_C_DEFINED
#define NEURO_GISTESTIMATORFFT_C_DEFINED

#include "Neuro/GistEstimatorFFT.H"

#include "Neuro/gistParams.H"
#include "Channels/IntensityChannel.H"
#include "Image/Kernels.H"     // for gaborFilter()
#include "Image/CutPaste.H"    // for inplacePaste()
#include "Image/MathOps.H"     // for mean(), stdev()
#include "Image/DrawOps.H"     // drawGrid()
#include "Raster/Raster.H"

#include "Simulation/SimEventQueue.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEvents.H"

#define NUM_G_LEV              5  // number of gabor levels
#define NUM_G_DIR              8  // number of gabor orientations

#define WIN_NUM                4  // 4: 4 by 4 windows

// ######################################################################
GistEstimatorFFT::GistEstimatorFFT(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName)
  :
  GistEstimatorAdapter(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventRetinaImage)
{
  gaborMaskImg = NULL;

  // (4x4 grid) x (5 levels x 8 direction feature size) = 16x40 = 640
  itsGistVector.resize
    (1, WIN_NUM * WIN_NUM * NUM_G_LEV * NUM_G_DIR, NO_INIT);
}

// ######################################################################
GistEstimatorFFT::~GistEstimatorFFT()
{ }

// ######################################################################
Image<double> GistEstimatorFFT::getGist()
{
  return itsGistVector;
}

// ######################################################################
void GistEstimatorFFT::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  Image<float> img = normalize(e->frame().grayFloat());
  computeGistFeatureVector(img);
  LINFO("Gist features are computed");

  // post an event so that anyone interested in gist can grab it:
  rutz::shared_ptr<SimEventGistOutput>
    ew(new SimEventGistOutput(this, itsGistVector));
  q.post(ew);
}

// ######################################################################
void GistEstimatorFFT::computeGistFeatureVector(Image<float> img)
{
  // get the current (normalized) input image
  // check whether it is not the same size as the previous one
  int w = img.getWidth()/WIN_NUM, h = img.getHeight()/WIN_NUM;
  Image<float> wImg(w,h, ZEROS);

  itsFftImage.resize(img.getDims());

  // reset the gabor masks initially or whenever dimension changed
  if((gaborMaskImg == NULL) ||
     (w != gaborMaskImg[0][0].getWidth() ) ||
     (h != gaborMaskImg[0][0].getHeight()))
    {
      //xw = new XWinManaged
      //  (Dims(2*img.getWidth(), img.getHeight()), 0, 0, "xw");
      //xw->drawImage(img,0,0); Raster::waitForKey();
      LDEBUG("w: %d, h: %d",w,h);
      setupFFTW(w,h);
    }
  //Raster::waitForKey();

  // get the features on each window
  Image<float> cImg(img.getWidth(), img.getHeight(), ZEROS);
  Image<float> dImg = img;
  float mn, mx; getMinMax(dImg,mn,mx);
  drawGrid(dImg, img.getWidth()/4,img.getHeight()/4,1,1, mx);

  Image<double>::iterator aptr = itsGistVector.beginw();
  for(uint ii = 0; ii < WIN_NUM; ii++)
    for(uint jj = 0; jj < WIN_NUM; jj++)
      {
        Point2D<int> p((ii*img.getWidth())/WIN_NUM,
                       (jj*img.getHeight())/WIN_NUM);
        Rectangle r(p, Dims(w,h));
        wImg = crop(img, r);

        //compute feature vector
        //Fourier Transform of the image
        fftCompute(wImg, itsOutFftwBuffer);

        // go through each gabor masks
        for(int i = 0; i < NUM_G_LEV; i++)
          for(int j = 0; j < NUM_G_DIR; j++)
            {
              // weighted sum of fft for a feature
              double sum = 0.0;
              for(int k = 0; k < h; k++)
                for(int l = 0; l < w/2+1; l++)
                  {
                    sum += log(1.0 + gaborMask[i][j][k][l]) *
                      log(1.0 + itsOutFftwBuffer[k][l]);
                  }
              *aptr++ = sum/(h*w/2+1);
            }

        // display Fourier Transform Image
        Image<float> temp = getFftImage(itsOutFftwBuffer, w, h);
        LDEBUG("w,h:%d:%d: i,j:%d:%d [%d %d]",
               ii*w, jj*h, temp.getWidth(), temp.getHeight(),
               itsFftImage.getWidth(), itsFftImage.getHeight());

        inplacePaste(itsFftImage, temp, Point2D<int>(ii*w, jj*h));
      }
}

// ######################################################################
// setup input and output array for FFT
void GistEstimatorFFT::setupFFTW(int w, int h)
{
  itsFftw = new FFTWWrapper(w,h);
  delete [] itsFftwBuffer;
  itsFftwBuffer = new double[w*h];
  itsFftw->init(itsFftwBuffer);

  // setup output array
  itsOutFftwBuffer = new double*[h];
  for(int i = 0; i < h; i++)
    itsOutFftwBuffer[i] = new double[w/2+1];

  // setup the log-gabor masks
  setupGaborMask(w,h);
}

// ######################################################################
void GistEstimatorFFT::setupGaborMask(int w, int h)
{
  int dim      = (w<h)?w:h;
  float stdev  = double(floor((dim - 3.0)/2.0/sqrt(10.0)));
  float period = double(stdev * 2.0);
  LDEBUG("w:h: %d:%d; dim -> %d== %f, %f",w,h,dim, stdev, period);
  itsStddev = stdev;

  // setup storage for gabor masks
  gaborMask = new double***[NUM_G_LEV];
  for(int i = 0; i < NUM_G_LEV; i++)
    gaborMask[i] = new double**[NUM_G_DIR];

  gaborMaskImg = new Image<float>*[NUM_G_LEV];
  for(int i = 0; i < NUM_G_LEV; i++)
    gaborMaskImg[i] = new Image<float>[NUM_G_DIR];

  // traverse through the different frequency and orientation
  float scale = 1.0;
  for(int  i = 0; i < NUM_G_LEV; i++)
  {
    for(int j = 0; j < NUM_G_DIR; j++)
    {
      // allocate space for the solution
      gaborMask[i][j] = new double*[h];
      for(int k = 0; k < h; k++)
        gaborMask[i][j][k] = new double[w/2+1];

      // setup the gabor kernel
      //Image<float> gaborF = gaborFilter<float>(stdev/scale, period/scale, 0.0,
      //                      (j*180.0)/NUM_G_DIR);
      //Image<float> temp (w, h, ZEROS); inplacePaste(temp, gaborF, Point2D<int>(0,0));
      //LINFO("gabor: [%d,%d]: scale: %f",gaborF.getWidth(), gaborF.getHeight(), scale);
      //xw->drawImage(temp,w,0); Raster::waitForKey();
      //fftCompute(temp, gaborMask[i][j]);
      //gaborMaskImg[i][j] = getFftImage(gaborMask[i][j], w, h);
      //xw->drawImage(gaborMaskImg[i][j],w,0);

      // perform the fourier transform and save the results
      computeFFTgabor(stdev/scale, period/scale, (j*180.0)/NUM_G_DIR, w, h,
                      gaborMask[i][j]);

      gaborMaskImg[i][j] = getFftImage(gaborMask[i][j], w, h);
      //xw->drawImage(gaborMaskImg[i][j],w,0);

      //Raster::waitForKey();
    }
    scale *= 2;
  }
}

// ######################################################################
void GistEstimatorFFT::computeFFTgabor
( const float stddev, const float period,
  const float theta, uint w, uint h, double **gm)
{
  float rtDeg = M_PI / 180.0F * theta;
  LDEBUG("std: %f, period: %f 1/p:[%f], theta: %f:%f:: cos: %f, sin: %f",
        stddev, period, 1.0/period, theta,rtDeg, cos(rtDeg), sin(rtDeg));

  double aSq = 1/(2.0*M_PI*stddev*stddev);
  double Kf = itsStddev*8.0;
  float X  = M_PI/aSq/4800.0;  //float X2 = .1*stddev/itsStddev;
  LDEBUG("Kf: %f X: %f, a^2: %f", Kf, X, aSq);

  // fourier transform cycles
  // so we perform the calculation four times

  double co = cos(rtDeg); double si = sin(rtDeg);
  double u0 = Kf*co/period; double v0 = Kf*si/period;
  LDEBUG("1: [u0,v0]:[%f,%f]", u0,v0);
  for(uint i = 0; i < w/2+1; i++)
    {
      for(uint j = 0; j < h/2; j++)
        {
          float ur =      (float(i) - u0)*co + (float(j) - v0)*si;
          float vr = -1.0*(float(i) - u0)*si + (float(j) - v0)*co;
          gm[j][i] = exp(-X*(ur*ur + vr*vr))/aSq;
          // gm[j][i] = exp((-M_PI/(aSq))*(ur*ur + vr*vr));
       }
    }

  co = cos(rtDeg);  si = sin(rtDeg);
  u0 = Kf*co/period; v0 = Kf*si/period + float(h);
  LDEBUG("2: [u0,v0]:[%f,%f]", u0,v0);
  for(uint i = 0; i < w/2+1; i++)
    {
      for(uint j = h/2; j < h; j++)
        {
          float ur =      (float(i) - u0)*co + (float(j) - v0)*si;
          float vr = -1.0*(float(i) - u0)*si + (float(j) - v0)*co;
          gm[j][i] = exp(-X*(ur*ur + vr*vr))/aSq;
      }
    }

  co = cos(-rtDeg);  si = sin(-rtDeg);
  u0 = -Kf*co/period; v0 = Kf*si/period;
  LDEBUG("3: [u0,v0]:[%f,%f]", u0,v0);
  for(uint i = 0; i < w/2+1; i++)
    {
      for(uint j = 0; j < h/2; j++)
        {
          float ur =      (float(i) - u0)*co + (float(j) - v0)*si;
          float vr = -1.0*(float(i) - u0)*si + (float(j) - v0)*co;
          gm[j][i] += exp(-X*(ur*ur + vr*vr))/aSq;
        }
    }

  co = cos(-rtDeg);  si = sin(-rtDeg);
  u0 = -Kf*co/period; v0 = Kf*si/period + float(h);
  LDEBUG("4: [u0,v0]:[%f,%f]", u0,v0);
  for(uint i = 0; i < w/2+1; i++)
    {
      for(uint j = h/2; j < h; j++)
        {
          float ur =      (float(i) - u0)*co + (float(j) - v0)*si;
          float vr = -1.0*(float(i) - u0)*si + (float(j) - v0)*co;
          gm[j][i] += exp(-X*(ur*ur + vr*vr))/aSq;
       }
    }
}

// ######################################################################
// convert image to array to input to FFT
// and compute the FFT
void GistEstimatorFFT::fftCompute(Image<float> img, double **fftOut)
{
  int w = img.getWidth(), h = img.getHeight();

  double *itsFftwInputData = itsFftwBuffer;
  for(int j = 0; j < h; j++)
    for(int i = 0; i < w; i++)
      *itsFftwInputData++ = (double)img.getVal(i,j);

  itsFftw->compute(fftOut);
}

// ######################################################################
Image<float> GistEstimatorFFT::getFftImage(double **res, int w, int h)
{
  // scaling the large dynamic range of the image Fourier Transform
  // using log: ln(1.0 + |F(i,j)|)
  // origin is centered to the middle of the image

  // gamma correction
  float gc = 1.5;

  // redraw the images
  Image<float> ftImg(w,h,ZEROS);
  for(int i = 0; i < w/2; i++)
    for(int j = 0; j < h/2; j++)
      ftImg.setVal(i+w/2, h/2-j,
                   pow(log(1.0 + res[j][i+1]), gc));

  for(int i = 0; i < w/2; i++)
    for(int j = h/2; j < h; j++)
      ftImg.setVal(i+w/2, 3*h/2-1-j,
                   pow(log(1.0 + res[j][i+1]), gc));

  for(int i = 0; i < w/2; i++)
    for(int j = 0; j < h; j++)
      ftImg.setVal(i, j, ftImg.getVal(w-1-i, h-1-j));


//   float mn, mx; getMinMax(ftImg,mn,mx);
//   for(int i = 0; i < ftImg.getWidth(); i++)
//     {
//       for(int j = 0; j < ftImg.getHeight(); j++)
//         {
//           if(ftImg.getVal(i,j) >= mx)
//             {
//               LINFO("coor: %d, %d",i,j);
//               ftImg.setVal(i,j,0.0);
//             }
//           }
//     }
//   LINFO("min-i-max: %f, %f",mn,mx);


  return ftImg;
}

// ######################################################################
// normalize image by subtracting it with mean and dividing by stdev
Image<float> GistEstimatorFFT::normalize(Image<float> img)
{
  Image<float> tImg = img;
  double tMean  = float(mean(img));
  double tStdev = float(stdev(img));
  tImg -= tMean;
  tImg /= tStdev;

  return tImg;
}

// ######################################################################
Image<float> GistEstimatorFFT::getFftImage()
{
  return itsFftImage;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_GISTESTIMATORFFT_C_DEFINED
