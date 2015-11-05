/*!@file Gist/CenterSurroundHistogramSegmenter.C segment out object
  depicted by the salient point using varying center surround
  histogram differencing */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/CenterSurroundHistogramSegmenter.C $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Gist/CenterSurroundHistogramSegmenter.H"

#include "Image/Kernels.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"

#include "Util/Timer.H"

#include "GUI/XWinManaged.H"

#include <cstdio>

#define SLACK_SIZE                    5

#define DEFAULT_WIDTH               600
#define DEFAULT_HEIGHT              480 

#define GRID_SIZE                    10

#define NUM_CHANNELS                  3 // current number of channels   
#define NUM_L_BINS                   25 // # bins for L component of CIELab
#define NUM_A_BINS                   25 // # bins for a component of CIELab
#define NUM_B_BINS                   25 // # bins for b component of CIELab
#define NUM_HISTOGRAM_DIMS           75 // current total number of bins

// ######################################################################
CenterSurroundHistogramSegmenter::CenterSurroundHistogramSegmenter()
{
  itsWin.reset();

  // These color features are inspired by Martin, PAMI 2004 pb paper
  float bg_smooth_sigma    = 0.1;       // bg histogram smoothing sigma
  float cg_smooth_sigma    = 0.05;      // cg histogram smoothing sigma

  // Gaussian smoothing kernel for each color channel
  itsLKernel = 
    gaussian<float>(1.0F, NUM_L_BINS*bg_smooth_sigma, 
                    int(3.0F*NUM_L_BINS*bg_smooth_sigma + 0.5F));
  itsAKernel = 
    gaussian<float>(1.0F, NUM_A_BINS*cg_smooth_sigma, 
                    int(3.0F*NUM_A_BINS*cg_smooth_sigma + 0.5F));
  itsBKernel = 
    gaussian<float>(1.0F, NUM_B_BINS*cg_smooth_sigma, 
                    int(3.0F*NUM_B_BINS*cg_smooth_sigma + 0.5F));

  // normalize the kernels
  float suml = sum(itsLKernel); itsLKernel /= suml;
  float suma = sum(itsAKernel); itsAKernel /= suma;
  float sumb = sum(itsBKernel); itsBKernel /= sumb;

  // Center Surround templates to estimate salient region shape
  computeCStemplates();
}

// ######################################################################
CenterSurroundHistogramSegmenter::~CenterSurroundHistogramSegmenter()
{ }

// ######################################################################
void CenterSurroundHistogramSegmenter::setImage
(Image<PixRGB<byte> > image)
{ 
  itsImage = image;
  itsCSrectangle = Rectangle();

  Timer tim(1000000); tim.reset();

  // create the CIE lab histogram entry images
  // this is the feature histogram in Martin's pb PAMI 2004
  setImageFeatureHistogramValues();
  LINFO("fHist   time: %f",tim.get()/1000.0F); tim.reset();

  // create the grid histogram and integral images
  computeGridHistogram();
  computeHistogramIntegralImage();
  LINFO("cHistII time: %f",tim.get()/1000.0F);

  // reset the center surround belief 
  int gwidth  = itsImage.getWidth() /GRID_SIZE;
  int gheight = itsImage.getHeight()/GRID_SIZE;
  itsGridCenterBelief   = Image<float>(gwidth, gheight, ZEROS);
  itsGridSurroundBelief = Image<float>(gwidth, gheight, ZEROS);
}

// ######################################################################
void CenterSurroundHistogramSegmenter::setImageFeatureHistogramValues()
{
  Timer tim(1000000); tim.reset();

  // convert to CIElab color space
  Image<float> lImg; 
  Image<float> aImg;
  Image<float> bImg;
  getNormalizedLAB(itsImage, lImg, aImg, bImg);
  LINFO("nLAB: %f", tim.get()/1000.0F); tim.reset();

  // convert to bin numbers
  itsImageHistogramEntries.clear();
  itsImageHistogramEntries.push_back(quantize_values(lImg, NUM_L_BINS));
  itsImageHistogramEntries.push_back(quantize_values(aImg, NUM_A_BINS));
  itsImageHistogramEntries.push_back(quantize_values(bImg, NUM_B_BINS));
  LINFO("qLAB: %f", tim.get()/1000.0F);
}

// ######################################################################
Image<int> CenterSurroundHistogramSegmenter::quantize_values
(Image<float> image, int num_bins)
{
  // FIXXX: May want to move to MathOps later
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();
  Image<int> result(w,h,NO_INIT);

  Image<float>::iterator iptr = image.beginw();  
  Image<int>::iterator aptr = result.beginw(), stop = result.endw();  
  float nbins = num_bins;
  while(aptr != stop)
    {
      float in = *iptr++;

      // bins will be 0 to num_bins-1
      int bin = int(in*nbins);
      if(bin == num_bins) bin = num_bins - 1;
      *aptr++ = bin;
    }

  return result;
}

// ######################################################################
void CenterSurroundHistogramSegmenter::computeGridHistogram()
{
  int gwidth  = itsImage.getWidth() /GRID_SIZE;
  int gheight = itsImage.getHeight()/GRID_SIZE;

  itsGridHistogram = 
    Image<rutz::shared_ptr<Histogram> >(gwidth, gheight, ZEROS);

  // compute histogram for each grid location
  for(int y = 0; y < gheight; y++)
    {
      int yy = y*GRID_SIZE;
      for(int x = 0; x < gwidth; x++)
        {
          int xx = x*GRID_SIZE;
          rutz::shared_ptr<Histogram> h = 
            getHistogramDistribution(Point2D<int>(xx,yy), GRID_SIZE);
          itsGridHistogram.setVal(x,y, h);
        }
    }
}

// ######################################################################
void CenterSurroundHistogramSegmenter::computeHistogramIntegralImage()
{
  int gwidth  = itsImage.getWidth() /GRID_SIZE;
  int gheight = itsImage.getHeight()/GRID_SIZE;

  // create integral image from the grid histogram
  itsIntegralHistogram = 
    Image<rutz::shared_ptr<Histogram> >(gwidth, gheight, ZEROS);

  std::vector<Histogram> s(gwidth);
  for (int i = 0; i < gwidth; i++) 
    {
      Histogram h(NUM_HISTOGRAM_DIMS);
      for(int j = 0; j < NUM_HISTOGRAM_DIMS; j++) 
        h.setValue(j, 0.0F);
      s[i] = h;
    }

  for(int y = 0; y < gheight; y++)
    {
      rutz::shared_ptr<Histogram> hist = itsGridHistogram.getVal(0,y);      
      s[0] = s[0] + *hist;

      itsIntegralHistogram.setVal
        (0,y, rutz::shared_ptr<Histogram>(new Histogram(s[0])));
      
      for(int x = 1; x < gwidth; x++)
        {
          rutz::shared_ptr<Histogram> h1 = itsIntegralHistogram.getVal(x-1,y);
          rutz::shared_ptr<Histogram> h2 = itsGridHistogram.getVal(x,y); 

          s[x] = s[x] + *h2;
          Histogram th2 = *h1 + s[x]; 

          itsIntegralHistogram.setVal
            (x,y, rutz::shared_ptr<Histogram>(new Histogram(th2)));
        }
    }
}

// ######################################################################
rutz::shared_ptr<Histogram> 
CenterSurroundHistogramSegmenter::getHistogramDistribution
(Point2D<int> pt, int grid_size)
{
  rutz::shared_ptr<Histogram> histogram;

  Histogram lHist(NUM_L_BINS);
  Histogram aHist(NUM_A_BINS);
  Histogram bHist(NUM_L_BINS);

  int rad = grid_size;
  int numPoints = 0;
  for(int i = 0; i < rad; i++) 
    for(int j = 0; j < rad; j++) 
      {
        Point2D<int> pt2 = pt + Point2D<int>(i,j);
        if(itsImage.coordsOk(pt2))
          {
            // L component
            int l_val = itsImageHistogramEntries[0].getVal(pt2);
            lHist.addValue(l_val, 1.0F);

            // a component
            int a_val = itsImageHistogramEntries[1].getVal(pt2);
            aHist.addValue(a_val, 1.0F);

            // b component
            int b_val = itsImageHistogramEntries[2].getVal(pt2);
            bHist.addValue(b_val, 1.0F);

            numPoints++;
          }
      }

  // maybe later want to add the weight for each channel
  //(Point2D<int> pt, std::vector<float> &dist, float weight)

  // combine the histograms
  std::vector<Histogram> histograms;
  histograms.push_back(lHist);
  histograms.push_back(aHist);
  histograms.push_back(bHist);
  histogram.reset(new Histogram(histograms));

  return histogram;
}

// ######################################################################
Image<float> CenterSurroundHistogramSegmenter::getSalientRegion
(Point2D<int> pt)
{ 
  Image<float> mask(itsImage.getDims(), ZEROS); 

  Point2D<int> gpt(pt.i/GRID_SIZE, pt.j/GRID_SIZE);
  Point2D<int> cpt = gpt * GRID_SIZE;
  LINFO("pt: %3d %3d --> %3d %3d gpt: %3d %3d", 
        pt.i, pt.j, cpt.i, cpt.j, gpt.i, gpt.j);

  // compute the center surround region
  ptsSalientRegion(pt);
  csTemplateSalientRegion(pt);  

  // grow from the center surround region map
  itsCSrectangle = growCSregion(pt);
  drawCurrentCSbelief(pt);

  // display computed mask
  drawFilledRect(mask, itsCSrectangle*GRID_SIZE, 1.0F);
  return mask;
}

// ######################################################################
void CenterSurroundHistogramSegmenter::ptsSalientRegion(Point2D<int> pt)
{
  Timer tim(1000000); tim.reset();

  // // display window
  // uint width  = itsImage.getWidth();
  // uint height = itsImage.getHeight();
  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(width, height), 0, 0, "CSHse"));
  // else itsWin->setDims(Dims(width, height));

  Point2D<int> gpt(pt.i/GRID_SIZE, pt.j/GRID_SIZE);

  // try various center surround combination
  for(uint i = 0; i < itsCSpoints.size(); i++)
    {
      Timer tim(1000000); tim.reset();
      std::vector<Point2D<int> > cPoints = itsCSpoints[i].first;
      std::vector<Point2D<int> > sPoints = itsCSpoints[i].second;

      // create center histogram from the center points
      // make sure the points are in bounds
      Histogram hC; bool cinit = false; 
      std::vector<Point2D<int> > cAPoints;
      for(uint j = 0; j < cPoints.size(); j++)
        {
          Point2D<int> pt = gpt + cPoints[j];
          if(!itsGridHistogram.coordsOk(pt)) continue;
             
          cAPoints.push_back(pt);
          if(!cinit)
            { hC = *itsGridHistogram.getVal(pt); cinit = true; }
          else
            hC = hC + *itsGridHistogram.getVal(pt);  
        }

      // create surround histogram from the surround points
      // make sure the points are in bounds
      Histogram hS; bool sinit = false; 
      std::vector<Point2D<int> > sAPoints;
      for(uint j = 0; j < sPoints.size(); j++)
        {
          Point2D<int> pt = gpt + sPoints[j];
          if(!itsGridHistogram.coordsOk(pt)) continue;
             
          sAPoints.push_back(pt);
          if(!sinit)
            { hS = *itsGridHistogram.getVal(pt); sinit = true; }
          else
            hS = hS + *itsGridHistogram.getVal(pt);  
        }
  
      // smooth and normalize the resulting histogram
      int npointC = cAPoints.size()*GRID_SIZE*GRID_SIZE;
      int npointS = sAPoints.size()*GRID_SIZE*GRID_SIZE;
      Histogram shC = smoothAndNormalize(hC, npointC);
      Histogram shS = smoothAndNormalize(hS, npointS);

      // print the difference
      float diff = shS.getChiSqDiff(shC);

      // update the center belief estimation
      // we store the max as the best estimation of belief
      for(uint cc = 0; cc < cAPoints.size(); cc++)
        {
          Point2D<int> pt = cAPoints[cc];
          float prevC = itsGridCenterBelief.getVal(pt);
          if(prevC < diff) itsGridCenterBelief.setVal(pt, diff);
        }        

      // update the surround belief estimation
      for(uint ss = 0; ss < sAPoints.size(); ss++)
        {
          Point2D<int> pt = sAPoints[ss];
          float prevS = itsGridSurroundBelief.getVal(pt);
          if(prevS < diff) itsGridSurroundBelief.setVal(pt, diff);
        }        

      // // display the window      
      // Image<PixRGB<byte> > disp(width, height, ZEROS); 

      // float mVal = 127;
      // float bVal = 255 - mVal;
      
      // Image<byte> dImaR, dImaG, dImaB;
      // getComponents(itsImage, dImaR, dImaG, dImaB);
      // inplaceNormalize(dImaR, byte(0), byte(mVal));
      // inplaceNormalize(dImaG, byte(0), byte(mVal));
      // inplaceNormalize(dImaB, byte(0), byte(mVal));
      // Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);      
      // //inplacePaste (disp, dIma, Point2D<int>(0,0));
      
      // Image<PixRGB<byte> > dImaCS(width,height, ZEROS);
      // for(uint j = 0; j < cAPoints.size(); j++)
      //   drawFilledRect(dImaCS, Rectangle(cAPoints[j],Dims(1,1))*GRID_SIZE, 
      //                  PixRGB<byte>(byte(bVal),0,0));

      // for(uint j = 0; j < sAPoints.size(); j++)
      //   drawFilledRect(dImaCS, Rectangle(sAPoints[j],Dims(1,1))*GRID_SIZE, 
      //                  PixRGB<byte>(0, byte(bVal),0));
      // Image<PixRGB<byte> > tdIma(dIma+dImaCS);
      // inplacePaste (disp, tdIma, Point2D<int>(0,0));

      // drawCross(disp, pt, PixRGB<byte>(255,0,0), 10, 1);

      // itsWin->drawImage(disp,0,0);
      // Raster::waitForKey();
    }

  LINFO("time: %f", tim.get()/1000.0F);
}

// ######################################################################
void CenterSurroundHistogramSegmenter::csTemplateSalientRegion
(Point2D<int> pt)
{
  Point2D<int> gpt(pt.i/GRID_SIZE, pt.j/GRID_SIZE);

  Rectangle intRect = itsIntegralHistogram.getBounds();

  Timer tim(1000000); tim.reset();

  // try the various center surround combination
  for(uint i = 0; i < itsCStemplates.size(); i++)
    {
      Rectangle cR = itsCStemplates[i].first;
      Rectangle sR = itsCStemplates[i].second;

      // only use the rectangle part that overlaps the image
      Rectangle grC = intRect.getOverlap(cR+gpt);
      Rectangle grS = intRect.getOverlap(sR+gpt);
  
      // get the center and surround histograms
      rutz::shared_ptr<Histogram> hC  = getGridHistogramDistribution(grC);
      rutz::shared_ptr<Histogram> hCS = getGridHistogramDistribution(grS);
      Histogram hS = (*hCS) - (*hC);

      // smooth and normalize
      int npointC = grC.area()*GRID_SIZE*GRID_SIZE;
      int npointS = grS.area()*GRID_SIZE*GRID_SIZE - npointC;
      Histogram shC = smoothAndNormalize(*hC, npointC);
      Histogram shS = smoothAndNormalize( hS, npointS);

      // get the difference
      float diff = shS.getChiSqDiff(shC);      

      // update the center surround belief estimation
      // we store the max as the best estimation of belief
      for(int ii = grS.left(); ii <= grS.rightI(); ii++)
        for(int jj = grS.top(); jj <= grS.bottomI(); jj++)
          {
            Point2D<int> pt(ii,jj);

            // if point is in center 
            if(grC.contains(pt))
              {
                float prevC = itsGridCenterBelief.getVal(ii,jj);
                if(prevC < diff) itsGridCenterBelief.setVal(ii,jj, diff);
              }
            // or surround
            else
              {
                float prevS = itsGridSurroundBelief.getVal(ii,jj);
                if(prevS < diff) itsGridSurroundBelief.setVal(ii,jj, diff);
              }
          }
    }

  LINFO("time: %f", tim.get()/1000.0F);
}

// ######################################################################
void CenterSurroundHistogramSegmenter::drawCurrentCSbelief
(Point2D<int> pt, Rectangle grC, Rectangle grS)
{
  uint width  = itsImage.getWidth();
  uint height = itsImage.getHeight();
  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(2*width, height), 0, 0, "CSHse"));
  else itsWin->setDims(Dims(2*width, height));

  uint gwidth  = width/GRID_SIZE;
  uint gheight = height/GRID_SIZE;

  // display the window      
  Image<PixRGB<byte> > disp(2*width, height, ZEROS);
  inplacePaste(disp, itsImage, Point2D<int>(0,0));
  if(pt.isValid())
    {
      drawCross(disp, pt, PixRGB<byte>(255,0,0), 10, 1);
    }
  if(grC.isValid())
    {
      drawRect(disp, grC*GRID_SIZE, PixRGB<byte>(255,0,0), 1);
      drawRect(disp, grS*GRID_SIZE, PixRGB<byte>(0,255,0), 1);
    }

  float mVal = 32;
  float bVal = 255 - mVal;
  
  Image<byte> dImaR, dImaG, dImaB;
  getComponents(itsImage, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);      
  
  // Image<float> dImaCf = itsGridCenterBelief;
  // inplaceNormalize(dImaCf, 0.0F, bVal);
  // Image<byte> dImaCb(dImaCf);
  // Image<PixRGB<byte> > dImaC = makeRGB(dImaCb,dImaCb,dImaCb);
  
  // Image<float> dImaSf = itsGridSurroundBelief;
  // inplaceNormalize(dImaSf, 0.0F, bVal);
  // Image<byte> dImaSb(dImaSf);
  // Image<PixRGB<byte> > dImaS = makeRGB(dImaSb,dImaSb,dImaSb);

  // Image<PixRGB<byte> > tdImaC(dIma+zoomXY(dImaC,GRID_SIZE));
  // Image<PixRGB<byte> > tdImaS(dIma+zoomXY(dImaS,GRID_SIZE));
  // inplacePaste (disp, tdImaC, Point2D<int>(width,0));
  // inplacePaste (disp, tdImaS, Point2D<int>(2*width,0));
 
  Image<float> dImaCSf = 
    clampedDiff((itsGridCenterBelief - itsGridSurroundBelief), 
              Image<float>(gwidth,gheight,ZEROS));
  inplaceNormalize(dImaCSf, 0.0F, bVal);
  Image<byte> dImaCSb(dImaCSf);
  Image<PixRGB<byte> > dImaCS = makeRGB(dImaCSb,dImaCSb,dImaCSb);
  Image<PixRGB<byte> > tdImaCS(dIma+zoomXY(dImaCS,GRID_SIZE));
  inplacePaste (disp, tdImaCS, Point2D<int>(width,0));

  Point2D<int> noff (width,0);
  drawCross(disp, pt+noff, PixRGB<byte>(255,0,0), 10, 1);
 
  if(itsCSrectangle.isValid())
    {
      drawRect(disp, itsCSrectangle*GRID_SIZE,        PixRGB<byte>(255,0,0), 1);
      drawRect(disp, (itsCSrectangle*GRID_SIZE)+noff, PixRGB<byte>(255,0,0), 1);
    }

  itsWin->drawImage(disp,0,0);
  Raster::waitForKey();
}

// ######################################################################
void CenterSurroundHistogramSegmenter::computeCStemplates()
{
  itsCStemplates.clear();
  itsCSpoints.clear();

  int width  = DEFAULT_WIDTH;
  int height = DEFAULT_HEIGHT;
  int mindim = width < height ? width : height;

  // compute for each scale: .1, .2, .3 of minimum of dimension
  // max was .7 NOTE: .4 is probably better
  std::vector<float> scales;
  for(float i = 0.1F; i <= 0.3F; i+=0.1F) scales.push_back(i);

  // compute ratio: .5, .75, 1.0, 1.5, 2.0
  std::vector<float> ratios;
  //for(float i = 0.5F; i <= 2.0F; i+=0.25F) ratios.push_back(i);
  ratios.push_back(0.5F);
  ratios.push_back(0.75F);
  ratios.push_back(1.0F);
  ratios.push_back(1.5F);
  ratios.push_back(2.0F);

  // cos and sin of 45 degree 
  // for the diagonal center surround shapes
  float cang = cos(M_PI/4.0F);
  float sang = sin(M_PI/4.0F);

  // for each scale and ratios
  for(uint i = 0; i < scales.size(); i++)
    {
      int h2 = int(scales[i]*mindim)/GRID_SIZE/2;
      LDEBUG("h2[%d]: %f --> %d ", i, scales[i]*mindim, h2);

      int prevw2 = -1;
      for(uint j = 0; j < ratios.size(); j++)
        {
          // get width, height, dimensions 
          // of the center and surround rectangles
          int w2 = int(h2*ratios[j]);
          if(w2 == prevw2) continue;  prevw2 = w2;
          int w   = 2*w2 + 1;        int h   = 2*h2 + 1;
          int sw2 = 2*w2;            int sh2 = 2*h2;
          int sw  = 2*sw2 + 1;       int sh  = 2*sh2 + 1;
          LDEBUG("w2[%d]: %f --> %d",j, h2*ratios[j], w2);
          
          // compute and store the offsets as well
          int ow = w2/2; if(ow == 0) ow = 1;
          int oh = h2/2; if(oh == 0) oh = 1;
          
          // here the off sets are spaced a quarter
          // the length of each dimension
          std::vector<int> offw; 
          offw.push_back(0); offw.push_back( ow); offw.push_back(-ow);
          if(w2 != 1){ offw.push_back( w2); offw.push_back(-w2); } 

          std::vector<int> offh; 
          offh.push_back(0); offh.push_back( oh); offh.push_back(-oh);
          if(h2 != 1){ offh.push_back( h2); offh.push_back(-h2); } 

          Dims cd(w,h); Dims sd(sw,sh);
          Point2D<int> offc( -w2,  -h2);
          Point2D<int> offs(-sw2, -sh2);
          LDEBUG("Dims: c:%d %d s:%d %d: off: %d %d", 
                 w, h, sw, sh, -w2, -h2);

          for(uint ii = 0; ii < offw.size(); ii++)
            for(uint jj = 0; jj < offh.size(); jj++)
              {
                Point2D<int> pt(offw[ii], offh[jj]);

                Rectangle rc(pt+offc, cd);
                Rectangle rs(pt+offs, sd);
                itsCStemplates.push_back
                  (std::pair<Rectangle,Rectangle>(rc,rs));
              }

          // now we rotate the rectangle 45 degrees 
          // and add it to the csPoints
          int sdiag = int(sqrt(sw*sw + sh*sh)+ 0.5F) + 1;
          Point2D<int> dpt(sdiag,sdiag);

          // here we rotate the image representation of the template
          Image<float> tcimg(2*sdiag,2*sdiag,ZEROS);
          drawFilledRect(tcimg, Rectangle(dpt+offc, cd), 1.0F);
          Image<float> rtcimg = rotate(tcimg, sdiag, sdiag, M_PI/4.0F);

          Image<float> tsimg(2*sdiag,2*sdiag,ZEROS);
          drawFilledRect(tsimg, Rectangle(dpt+offs, sd), 1.0F);
          Image<float> rtsimg = rotate(tsimg, sdiag, sdiag, M_PI/4.0F);

          // only select rotated point values 
          // that are above 0.5F
          std::vector<Point2D<int> > tempC;
          for(int jj = 0; jj < rtcimg.getHeight(); jj++)
            for(int ii = 0; ii < rtcimg.getWidth(); ii++)
              {
                float val = rtcimg.getVal(ii,jj);
                if(val > 0.5F) 
                  tempC.push_back(Point2D<int>(ii,jj)-dpt);
              }

          std::vector<Point2D<int> > tempS;
          for(int jj = 0; jj < rtsimg.getHeight(); jj++)
            for(int ii = 0; ii < rtsimg.getWidth(); ii++)
              {
                float cval = rtcimg.getVal(ii,jj);
                float sval = rtsimg.getVal(ii,jj);
                if(cval < 0.5F && sval > 0.5F) 
                  tempS.push_back(Point2D<int>(ii,jj)-dpt);
              }
          
          // rotate the offset points as well
          for(uint ii = 0; ii < offw.size(); ii++)
            for(uint jj = 0; jj < offh.size(); jj++)
              {
                float rx = round( offw[ii]*cang - offh[jj]*sang);
                float ry = round( offw[ii]*sang + offh[jj]*cang);
                Point2D<int> pt(int(rx+0.0F), int(ry+0.0F));

                LDEBUG("rotated (%3d %3d) --> %5.3f %5.3f --> pt: %3d %3d", 
                       offw[ii], offh[jj], rx, ry, pt.i, pt.j);

                std::vector<Point2D<int> > rtempC;
                for(uint cc = 0; cc < tempC.size(); cc++)
                  rtempC.push_back(tempC[cc]+pt);

                std::vector<Point2D<int> > rtempS;
                for(uint ss = 0; ss < tempS.size(); ss++)
                  rtempS.push_back(tempS[ss]+pt);

                itsCSpoints.push_back
                  (std::pair<std::vector<Point2D<int> >,
                   std::vector<Point2D<int> > >(rtempC,rtempS));
              }

          // Image<float> disp(4*sdiag*GRID_SIZE, 2*sdiag*GRID_SIZE, ZEROS);
          // if(itsWin.is_invalid())
          //   itsWin.reset(new XWinManaged(disp.getDims(), 0, 0, "CSHse"));
          // else itsWin->setDims(disp.getDims());
          // inplacePaste(disp, zoomXY(tcimg,GRID_SIZE), Point2D<int>(0,0));
          // inplacePaste(disp, zoomXY(rtcimg,GRID_SIZE), 
          //              Point2D<int>(2*sdiag*GRID_SIZE,0));
          // itsWin->drawImage(disp,0,0);
          // Raster::waitForKey();          

          // inplacePaste(disp, zoomXY(tsimg,GRID_SIZE), Point2D<int>(0,0));
          // inplacePaste(disp, zoomXY(rtsimg,GRID_SIZE), 
          //              Point2D<int>(2*sdiag*GRID_SIZE,0));
          // itsWin->drawImage(disp,0,0);
          // Raster::waitForKey(); 
        }
    }

  LDEBUG("done with templates: %" ZU , itsCStemplates.size());
  LDEBUG("done with points:    %" ZU , itsCSpoints.size());
}

// ######################################################################
rutz::shared_ptr<Histogram> 
CenterSurroundHistogramSegmenter::getGridHistogramDistribution(Rectangle r)
{
  int tt = r.top(), bb = r.bottomI(), ll = r.left(), rr = r.rightI();

  // integral image sum: l4 + l1 - (l2 + l3)
  rutz::shared_ptr<Histogram> l1, l2, l3, l4; 
  if(ll != 0 && tt != 0) l1 = itsIntegralHistogram.getVal(ll-1, tt-1); 
  if(tt != 0)            l2 = itsIntegralHistogram.getVal(rr  , tt-1);
  if(ll != 0)            l3 = itsIntegralHistogram.getVal(ll-1, bb  );
  l4 = itsIntegralHistogram.getVal(rr,bb);

  Histogram temp;
  if(ll == 0 && tt == 0) temp = *l4;
  else if(tt == 0)       temp = *l4 - *l3; 
  else if(ll == 0)       temp = *l4 - *l2; 
  else                   temp = *l4 + *l1 - (*l2 + *l3);

  return rutz::shared_ptr<Histogram>(new Histogram(temp)); 
}

// ######################################################################
Histogram CenterSurroundHistogramSegmenter::smoothAndNormalize
(Histogram h, int numPoints)
{
  // maybe later want to add the weight for each channel
  //(Point2D<int> pt, std::vector<float> &dist, float weight)

  uint sindex = 0; uint eindex = NUM_L_BINS - 1;
  Histogram lHist(h, sindex, eindex);
  sindex = eindex + 1; eindex += NUM_A_BINS;
  Histogram aHist(h, sindex, eindex);
  sindex = eindex + 1; eindex += NUM_B_BINS;
  Histogram bHist(h, sindex, eindex);
  
  // smooth the histogram individually
  lHist.smooth(itsLKernel);
  aHist.smooth(itsAKernel);
  bHist.smooth(itsBKernel);

  // combine the histograms
  std::vector<Histogram> histograms;
  histograms.push_back(lHist);
  histograms.push_back(aHist);
  histograms.push_back(bHist);
  //Histogram histogram.reset(new Histogram(histograms));
  Histogram rh(histograms);
  
  // normalize with the number of points
  rh.divide(numPoints);

  return rh;
}

// ######################################################################
Rectangle CenterSurroundHistogramSegmenter::growCSregion(Point2D<int> pt)
{
  Timer tim(1000000); tim.reset();

  Point2D<int> gpt(pt.i/GRID_SIZE, pt.j/GRID_SIZE);
  int gwidth  = itsGridCenterBelief.getWidth();
  int gheight = itsGridCenterBelief.getHeight();

  int rad_dist = 5;

  // clamp zero the difference of center and surround belief
  Image<float> cs = 
    clampedDiff((itsGridCenterBelief - itsGridSurroundBelief), 
                Image<float>(gwidth,gheight,ZEROS));

  Image<bool> visited(gwidth, gheight, ZEROS);
  visited.setVal(gpt, true);

  // grow from the center
  std::vector<Point2D<int> > csRegion;
  std::vector<Point2D<int> > regQueue; regQueue.push_back(gpt);
  uint cindex = 0;
  
  // start growing
  int tt = gpt.j, ll = gpt.i, bb = gpt.j, rr = gpt.i;
  float max_dist = 0.0F;
  while(cindex < regQueue.size())
    {
      Point2D<int> cgpt = regQueue[cindex];
      csRegion.push_back(cgpt);

      // go through all its neighbors
      for(int di = -1; di <= 1; di++)
        for(int dj = -1; dj <= 1; dj++)
          {
            if(di == 0 && dj == 0) continue;
            Point2D<int> ncgpt(cgpt.i+di, cgpt.j+dj);

            // skip if out of bounds and already visited
            if(!cs.coordsOk(ncgpt)) continue;
            if(visited.getVal(ncgpt)) continue;

            // include points that are
            // within max_dist distance or value above 0.0F
            float dist = gpt.distance(ncgpt);
            float val = cs.getVal(ncgpt);
            if(dist < rad_dist || val > 0.0F) 
              { 
                regQueue.push_back(ncgpt);
                if(max_dist < dist) max_dist = dist;
                if(tt > ncgpt.j) tt = ncgpt.j;
                if(bb < ncgpt.j) bb = ncgpt.j;
                if(ll > ncgpt.i) ll = ncgpt.i;
                if(rr < ncgpt.i) rr = ncgpt.i;

              }
            visited.setVal(ncgpt, true);
          }
      cindex++;
    }

  // add a bit of slack
  int slack = SLACK_SIZE;
  tt -= slack; bb += slack; ll -= slack; rr += slack;
  if(tt < 0) tt = 0; if(bb >= gheight) bb = gheight - 1; 
  if(ll < 0) ll = 0; if(rr >= gwidth)  rr = gwidth  - 1; 
  Rectangle res = Rectangle::tlbrI(tt,ll,bb,rr);
  res = res.getOverlap(visited.getBounds());

  LINFO("grow region: %f", tim.get()/1000.0F);

  return res;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
