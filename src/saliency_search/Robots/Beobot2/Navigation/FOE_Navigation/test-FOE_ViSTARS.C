/*!@file Robots/Beobot2/Navigation/FOE_Navigation/test-FOE_ViSTARS.C
  find the FOE using Browning 2009 */
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
// $HeadURL: $
// $Id: $

//

#define NUM_DIRS        8   
#define NUM_CS          2
 
// OpenCV must be first to avoid conflicting defs of int64, uint64
#include "Image/OpenCVUtil.H"

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "Image/CutPaste.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/FilterOps.H"
#include "Image/ShapeOps.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/MatrixOps.H"



#include "Util/Timer.H"

#include <stdio.h>

#include "Robots/Beobot2/Navigation/FOE_Navigation/FoeDetector.H"
#include "Robots/Beobot2/Navigation/FOE_Navigation/MotionOps.H"

// ######################################################################
struct ViSTARSmodelState
{
  ViSTARSmodelState() { };

  std::vector<std::vector<Image<double> > > I;
  std::vector<std::vector<Image<double> > > a;
  std::vector<std::vector<Image<double> > > b;
  std::vector<std::vector<Image<double> > > x;
  std::vector<std::vector<Image<double> > > y;
  std::vector<std::vector<Image<double> > > z;

  std::vector<std::vector<std::vector<Image<double> > > > c;
  std::vector<std::vector<std::vector<Image<double> > > > e;
  std::vector<std::vector<Image<double> > > f;

  std::vector<Image<double> > q;  
  std::vector<Image<double> > Q;  // MT outputs
  std::vector<Image<double> > m;  

  std::vector<double> r;

  // Heading firing rate
  std::vector<double> R; 
};

struct ViSTARSmodel
{
  ViSTARSmodel() { };

  // SortObj(const rutz::shared_ptr<VisualObject> _obj,
  //         const Point2D<int> _objOffset,
  //         const uint _fNum,
  //         const uint _sNum) :
  //   obj(_obj),
  //   objOffset(_objOffset),
  //   fNum(_fNum),
  //   sNum(_sNum)
  // {  }

  uint p_NumSpeeds;

  // Level 1
  double p_A1; 
  double p_B1;
  double p_C1;
  double p_D1;
  double p_phi1;
  double p_E1;
  Image<double> p_Fxy; // Fxy is a 2D symmetric Gaussian with sigma = 1
  double p_G1;

  // Level 2
  double p_A2;
  double p_B2;
  double p_C2;
  double p_D2;
  double p_K2;

  // Level 3
  double p_A3;
  double p_B3;
  double p_C3;
  double p_K3;
  double p_A4;
  double p_B4;
  double p_C4;
  double p_K4;

  // Level 4
  double p_A5;
  double p_B5;
  double p_C5;

  // Level 5 & 6
  double p_A6;   
  double p_B6;
  double p_C6;
  double p_D6;
  double p_theta6;  
  std::vector<double> p_vdD;

  double p_A7;
  double p_B7;
  double p_C7;
  double p_D7;
  double p_E7;
  double p_G7;
  double p_theta7;
  rutz::shared_ptr<std::vector<std::vector<Image<double> > > > p_w; 
  double p_M; 
  double p_N;
  rutz::shared_ptr<std::vector<Image<double> > > p_L; 

  std::vector<Point2D<int> > Rlocs;

  rutz::shared_ptr<ViSTARSmodelState> state;
};

// return the Focus of Expansion
Point2D<int> getFOE(ViSTARSmodel &model, Image<double> stim);
void updateFrame(ViSTARSmodel &model, Image<double> stim);

Point2D<int> bgm_2007(ViSTARSmodel &model);

std::vector< Image<double> > 
sumFxyI(Image<double> F, std::vector<Image<double> > I);

Image<double> combine(Image<double> y, double f);

std::vector<Image<double> > 
sumLxy(rutz::shared_ptr<std::vector<Image<double> > > L, 
       std::vector<Image<double> > I);

std::vector<Image<double> > 
sumRz(std::vector<double> R, 
      rutz::shared_ptr<std::vector<std::vector<Image<double> > > > w);

std::vector<Image<double> >
sumvD(std::vector<double>v, std::vector<Image<double> > I);

int dof(int x);

Image<double> filter2(Image<double> f, Image<double> d);

void report(ViSTARSmodel &model);


ViSTARSmodel setupParams(uint comp, Dims size); 

//
//rutz::shared_ptr<std::vector<std::vector<Image<double> > > > 
std::pair<rutz::shared_ptr<std::vector<std::vector<Image<double> > > >,
          std::vector<Point2D<int> > >
generate_w (Dims sz, std::vector<double> vert_pos, uint horz_spac, uint scale);

//
rutz::shared_ptr<std::vector<Image<double> > > 
generate_L(double l, double sigx, double sigy, double k);

void initializeModel(ViSTARSmodel &model, Image<double> stim);

Image<double> shrink(Image<double> y, double f);

double min(double a, double b)
{
  if(a<b) return a;
  else    return b;
}

double max(double a, double b)
{
  if(a>b) return a;
  else    return b;
}

double tstep = .1;

// ######################################################################
int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // instantiate a model manager:
  ModelManager manager("Test Optic Flow");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  // FIXXX: probably wants to do this later for the ViSTARS code
  //nub::ref<FoeDetector> fd(new FoeDetector(manager));
  //manager.addSubComponent(fd);

  manager.exportOptions(MC_RECURSE);

  if (manager.parseCommandLine((const int)argc, (const char**)argv,
                               "", 0, 0) == false)
    return(1);

  // get some options
  // if(manager.numExtraArgs() >  0)
  //   {
  //     outFilename = manager.getExtraArg(0);
  //     LINFO("save to: %s", outFilename.c_str());
  //   }

  // let's do it!
  manager.start();

  Timer timer(1000000);
  timer.reset();  // reset the timer
  int frame = 0;
  bool keepGoing = true;
  const FrameState is = ifs->updateNext();

  ViSTARSmodel model = setupParams(1, ifs->peekDims()); 
  Image< PixRGB<byte> > image;
  Image<double> stim;
  if (is == FRAME_COMPLETE) keepGoing = false;
  else 
    {
      image = ifs->readRGB();
      stim = luminance(image)/255.0;

      // model initialization
      initializeModel(model, stim);
    }

  while (keepGoing)
    {
      if (ofs->becameVoid())
        {
          LINFO("quitting because output stream was closed or became void");
          break;
        }

      Point2D<int> pt = getFOE(model, stim);      
      Image<PixRGB<byte> > disp = image;
      drawFilledRect
        (disp,Rectangle::tlbrI(pt.j, pt.i, pt.j+4, pt.i+4), 
         PixRGB<byte>(255,0,0));

      ofs->writeRGB(disp, "Optic Flow");
      const FrameState os = ofs->updateNext();
      //Raster::waitForKey();

      //Raster::WriteRGB(currImage, sformat("image%d.ppm",frame));

      if (os == FRAME_FINAL) break;

      const FrameState is = ifs->updateNext();
      if (is == FRAME_COMPLETE) break; // done receiving frames
      image = ifs->readRGB();
      stim  = luminance(image)/255.0;
      frame++;
    }

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;
}

// ######################################################################
ViSTARSmodel setupParams(uint comp, Dims size)
{
  ViSTARSmodel model;
  model.p_NumSpeeds = 3;

  // Level 1
  model.p_A1   = 0.001;
  model.p_B1   = 1.0;
  model.p_C1   = 2.0;
  model.p_D1   = 0.25;
  model.p_phi1 = 0.1;
  model.p_E1   = 10.225;    // 10, K3 1 , K4 1 gets good ogl results not yosem

  // Fxy is a 2D symmetric Gaussian with sigma = 1

  // x = meshgrid(-3:3);
  Image<double> xImg(7,7,ZEROS);
  for(int i = 0; i < 7; i++)
    {
      int x = i - 3;
      for(int j = 0; j < 7; j++) xImg.setVal(i,j, x);
    }

  // xx = sqrt(x.^2 + (x').^2);
  Image<double> txImg = transpose(xImg);
  Image<double> xxImg = toPower(xImg*xImg + txImg*txImg, .5);

  // model.p.Fxy = (model.p.E1/(2*pi))*exp(-xx.^2)';  
  Image<double> xxxImg = transpose(exp(xxImg*xxImg*-1.0)*(model.p_E1/(2*M_PI)));  
  model.p_Fxy = xxxImg;

      // LINFO("xxxImage");
      // for(int y = 0; y < xxxImg.getHeight(); y++)
      //   {
      //     for(int x = 0; x < xxxImg.getWidth(); x++)
      //       printf("%10.4f ", xxxImg.getVal(x,y));
      //     printf("\n");
      //   }


  model.p_G1 = 0.031623;

  // Level 2
  model.p_A2 = 10.0;
  model.p_B2 = 1.0;
  model.p_C2 = 2.0;
  model.p_D2 = 0.01;
  model.p_K2 = 20.0;

  // Level 3
  model.p_A3 = 1.0;
  model.p_B3 = 1.0;
  model.p_C3 = 1.0;
  model.p_K3 = 2.0;
  model.p_A4 = 10.0;
  model.p_B4 = 1.0;
  model.p_C4 = 1.0;
  model.p_K4 = 2.0;
  
  // Level 4
  model.p_A5 = 0.1;
  model.p_B5 = 1.0;
  model.p_C5 = 0.01;

  // Level 5 & 6
  model.p_A6 = 0.5;   
  model.p_B6 = 1.0;
  model.p_C6 = 0.5;
  model.p_D6 = 0.5;
  model.p_theta6 = 0.2;

  
  if(comp == 0)
    {
      // [0 0 0 0 0]
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(0.0);
    }
  else if(comp == 1) 
    {
      // [0 0.5 1 1 10]; 
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(0.5);
      model.p_vdD.push_back(1.0);
      model.p_vdD.push_back(1.0);
      model.p_vdD.push_back(10.0);
    }
  else if(comp == 2)
    {
      // [0 0 0 0 5];
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(0.0);
      model.p_vdD.push_back(5.0);
    }
  else if(comp == 3)
    {
      // [0.25 0.25 1 0.25 10];
      model.p_vdD.push_back(0.25);
      model.p_vdD.push_back(0.25);
      model.p_vdD.push_back(1.0);
      model.p_vdD.push_back(0.25);
      model.p_vdD.push_back(10.0);
    }
  else LFATAL("no such competitive environment");

  model.p_A7 = 0.5;
  model.p_B7 = 1.0;
  model.p_C7 = 4.0;
  model.p_D7 = 0.25;
  model.p_E7 = 0.25;
  model.p_G7 = 0.1;
  model.p_theta7 = 0.2;
  
  Dims sz(size.w()/(pow(2, model.p_NumSpeeds-1)),
          size.h()/(pow(2, model.p_NumSpeeds-1)) );

  // ORIGINAL
  //vert_pos.push_back(.5);    //1/2
  //vert_pos.push_back(.625);    //5/8
  //uint horz_spac = 3;

  // BROWNING
  std::vector<double> vert_pos;
  vert_pos.push_back(.25);    //1/4
  vert_pos.push_back(5.0/16.0);    //5/16
  vert_pos.push_back(.375);  //3/8 
  uint horz_spac = 5;

  // std::vector<double> vert_pos;
  // vert_pos.push_back(.5);    //1/2
  // vert_pos.push_back(9.0/16.0);    //9/16
  // vert_pos.push_back(.625);  //5/8 
  // uint horz_spac = 5;

  std::pair<rutz::shared_ptr<std::vector<std::vector<Image<double> > > >,
    std::vector<Point2D<int> > > temp =
  generate_w(sz, vert_pos, horz_spac, uint(pow(2, model.p_NumSpeeds-1)));
  model.p_w   = temp.first; 
  model.Rlocs = temp.second;
  model.p_M = vert_pos.size() * (sz.w()/horz_spac); 

  rutz::shared_ptr<std::vector<std::vector<Image<double> > > > p_w 
    = model.p_w;
  double min = 0;
  for(uint l = 0; l < model.p_w->size(); l++)
    {
      double total = 0;
      for(uint dir = 0; dir < 8; dir++)
        total += sum((*p_w)[l][dir]);
      if(total < min || l == 0)  min = total; 
    }
  model.p_N = min;

  model.p_L = generate_L(2,2,3,0.25);

  return model;
}

// ######################################################################
// SUB FUNCTION: generate_L, generates the long-range filter.
//function [L Nl err] = generate_L(l, sigx, sigy, k)
 rutz::shared_ptr<std::vector<Image<double> > > 
generate_L(double l, double sigx, double sigy, double k)
 {
   // FIX: hard coded information!!!
    int wid = 25;
    int mid = 13;
    double th = 0.005;

    // L = zeros(wid,wid, 8);
    rutz::shared_ptr<std::vector<Image<double> > > 
      L(new std::vector<Image<double> >());

    // rot45 = [cos(45*(pi/180)) -sin(45*(pi/180)); 
    //          sin(45*(pi/180))  cos(45*(pi/180))];
    Image<double> rot45(Dims(2,2),NO_INIT);
    rot45.setVal(0,0,  cos(45*(M_PI/180)));
    rot45.setVal(1,0, -sin(45*(M_PI/180)));
    rot45.setVal(0,1,  sin(45*(M_PI/180)));
    rot45.setVal(1,1,  cos(45*(M_PI/180)));

    Image<double> temp1(wid, wid, ZEROS);
    Image<double> temp8(wid, wid, ZEROS);
    for(int i = 0; i < wid; i++)
      {
        for(int j = 0; j < wid; j++)
          {
            int x = i-mid+1; int y = j-mid+1;

            // L(i,j,1) = (l/(2*pi*sigx*sigy))*exp(-k*((x/sigx)^2 + (y/sigy)^2));
            double val =
              (l/(2*M_PI*sigx*sigy))*exp(-k*(pow(x/sigx, 2.0) + pow(y/sigy, 2.0)));
            temp1.setVal(j,i, val); // MATLAB: (row, column) coordinate system

            // nxy = round(sqrt(2)*rot45*[x;y])+mid;
            Image<double> t(1,2,NO_INIT);
            t.setVal(0,0, x); t.setVal(0,1, y);
            Image<double> nxy = matrixMult(rot45,t)*sqrt(2.0);
            double v1 = nxy.getVal(0,0); double v2 = nxy.getVal(0,1);
            nxy.setVal(0,0, round(v1)+ mid);
            nxy.setVal(0,1, round(v2)+ mid);

            //L(max(min(nxy(1)-x ,wid),1),max(min(nxy(2) ,wid),1),8) = L(i,j,1);
            uint i8 = uint(max(min(double(nxy.getVal(0)-x), double(wid)),1))-1; 
            uint j8 = uint(max(min(double(nxy.getVal(1)  ), double(wid)),1))-1;
            temp8.setVal(j8,i8, val);
          }
      }

    // L(:,:,1) = (L(:,:,1)>th).*L(:,:,1);
    for(int i = 0; i < wid; i++)
      for(int j = 0; j < wid; j++)
        {
          double val  = temp1.getVal(i,j);
          temp1.setVal(i,j, (val>th)*val);
        }

    // L(:,:,8) = (L(:,:,8)>th).*L(:,:,8);
    for(int i = 0; i < wid; i++)
      for(int j = 0; j < wid; j++)
        {
          double val  = temp8.getVal(i,j);
          temp8.setVal(i,j, (val>th)*val);
        }

    // err = sum(sum(L(:,:,1)))-sum(sum(L(:,:,8)));

    // L(:,:,5) = L(:,:,1); L(:,:,3) = rot90(L(:,:,1)); L(:,:,7) = L(:,:,3);
    Image<double> temp5 = temp1;
    Image<double> temp3(wid,wid,NO_INIT);
    int w2 = wid/2;
    for(int i = 0; i < wid; i++)
      for(int j = 0; j < wid; j++)
        temp3.setVal(w2 + (j - w2), w2 - (i - w2), temp1.getVal(i,j));
    Image<double> temp7 = temp3;

    // L(:,:,4) = L(:,:,8); L(:,:,2) = fliplr(L(:,:,8)); L(:,:,6) = L(:,:,2);
    Image<double> temp4 = temp8;
    Image<double> temp2(wid,wid,NO_INIT);
    for(int i = 0; i < wid; i++)
      for(int j = 0; j < wid; j++)
        temp2.setVal(int(wid-1) - i, j, temp8.getVal(i,j));
    Image<double> temp6 = temp2;

    L->push_back(temp1);
    L->push_back(temp2);
    L->push_back(temp3);
    L->push_back(temp4);
    L->push_back(temp5);
    L->push_back(temp6);
    L->push_back(temp7);
    L->push_back(temp8);

    return L;
 }
   
// ######################################################################
// SUB FUNCTION: generate_w, generates the template weight matrices
//function [w M N] = generate_w(sz, vert_pos, horz_spac)
std::pair<rutz::shared_ptr<std::vector<std::vector<Image<double> > > >,
          std::vector<Point2D<int> > >
generate_w
(Dims sz, std::vector<double> vert_pos, uint horz_spac, uint scale)
{
  int smax = sz.w(); if(smax < sz.h()) smax = sz.h();
  int sizx = round(2*smax);

  Image<double> U(sizx*2+1, sizx*2+1, NO_INIT);
  Image<double> V(sizx*2+1, sizx*2+1, NO_INIT); 

  // FIX MESHGRID
  // [U, V] = meshgrid(-sizx:sizx, -sizx:sizx); 
  for(int i = 0; i < sizx*2+1; i++)
    {
      int x = i - sizx;
      for(int j = 0; j < sizx*2+1; j++)
        {
          U.setVal(i,j, x);
          V.setVal(j,i, x);
        }
    }

   // d = (4*atan2(V,U)/pi+1); d= d+(d<1)*8;
   Image<double> d(sizx*2+1, sizx*2+1, NO_INIT);
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val = 
             4*atan2(V.getVal(i,j),U.getVal(i,j))/M_PI + 1.0;
           double val2 = val + (val<1)*8;
           d.setVal(i,j, val2);
         }
     }

   // convert to 8 directions
   std::vector<Image<double> > tw;
   Image<double> res(sizx*2+1, sizx*2+1, NO_INIT);
   // tw(:,:,1,1) = double(d<2).*(2-d) + double(d > 8).*(d-8); 
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val  = d.getVal(i,j);
           double val2 = double(val<2)*(2-val) + double(val>8)*(val-8);
           res.setVal(i,j, val2);
         }
     }
   tw.push_back(res);

   // tw(:,:,1,2) = double(d<3&d>=2).*(3-d) + double(d>1&d<2).*(d-1);
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val  = d.getVal(i,j);
           double val2 = double((val<3)&(val>=2))*(3-val) + 
             double((val>1)&(val<2))*(val-1);
           res.setVal(i,j, val2);
         }
     }
   tw.push_back(res);

   // tw(:,:,1,3) = double(d<4&d>=3).*(4-d) + double(d>2&d<3).*(d-2);
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val  = d.getVal(i,j);
           double val2 = double((val<4)&(val>=3))*(4-val) + 
             double((val>2)&(val<3))*(val-2);
           res.setVal(i,j, val2);
         }
     }
   tw.push_back(res);

   // tw(:,:,1,4) = double(d<5&d>=4).*(5-d) + double(d>3&d<4).*(d-3);
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val  = d.getVal(i,j);
           double val2 = double((val<5)&(val>=4))*(5-val) + 
             double((val>3)&(val<4))*(val-3);
           res.setVal(i,j, val2);
         }
     }
   tw.push_back(res);

   // tw(:,:,1,5) = double(d<6&d>=5).*(6-d) + double(d>4&d<5).*(d-4);
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val  = d.getVal(i,j);
           double val2 = double((val<6)&(val>=5))*(6-val) + 
             double((val>4)&(val<5))*(val-4);
           res.setVal(i,j, val2);
         }
     }
   tw.push_back(res);

   // tw(:,:,1,6) = double(d<7&d>=6).*(7-d) + double(d>5&d<6).*(d-5);
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val  = d.getVal(i,j);
           double val2 = double((val<7)&(val>=6))*(7-val) + 
             double((val>5)&(val<6))*(val-5);
           res.setVal(i,j, val2);
         }
     }
   tw.push_back(res);

   // tw(:,:,1,7) = double(d<8&d>=7).*(8-d) + double(d>6&d<7).*(d-6);
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val  = d.getVal(i,j);
           double val2 = double((val<8)&(val>=7))*(8-val) + 
             double((val>6)&(val<7))*(val-6);
           res.setVal(i,j, val2);
         }
     }
   tw.push_back(res);

   // tw(:,:,1,8) = double(d>=8).*(9-d) + double(d>7&d<8).*(d-7);
   for(int i = 0; i < sizx*2+1; i++)
     {
       for(int j = 0; j < sizx*2+1; j++)
         {
           double val  = d.getVal(i,j);
           double val2 = double(val>=8)*(9-val) + 
             double((val>7)&(val<8))*(val-7);
           res.setVal(i,j, val2);
         }
     }
   tw.push_back(res);

   // for d=1:8, tw(sizx+1, sizx+1,1,d) = 1; end
   for(uint i = 0; i < 8; i++) tw[i].setVal(sizx,sizx, 1.0);

   // counter = 0;
   // w=zeros(sz(1), sz(2), 1, 8);
   rutz::shared_ptr<std::vector<std::vector<Image<double> > > > 
     w(new std::vector<std::vector<Image<double> > >());

   std::vector<Point2D<int> > rlocs;

   uint maxVert = vert_pos.size();  
   uint maxHorz = sz.w()/horz_spac;
   // for vert=[round(vert_pos*sz(1))]
   //     for horz=round(horz_spac/2):horz_spac:sz(2)
   //         counter = counter + 1;
   for(uint i = 0; i < maxVert; i++)
     {
       int  vert = round(vert_pos[i]*sz.h());
       for(uint j = 0; j < maxHorz; j++)
         {
           int horz = (horz_spac/2) + j*horz_spac;

           std::vector<Image<double> > temp;

           // w(:,:, counter,:) =  tw(sizx-vert   : sizx+((size(w,1)-1)-vert), 
           //                         sizx-horz+2 : sizx+((size(w,2)-1)-horz+2),
           //                         1, :);
           int t = sizx-vert - 1;
           int b = sizx+((sz.h()-1) -vert) - 1;
           int l = sizx-horz+0;
           int r = sizx+((sz.w()-1) - horz+0);
           for(uint k = 0; k < 8; k++)
             {
               temp.push_back(crop(tw[k],Rectangle::tlbrI(t,l,b,r)));
             }
           //LINFO("[sizx: %d vert:%d horz:%d] t:%d b:%d l:%d r:%d", 
           //  sizx, vert, horz, t,b,l,r);
           w->push_back(temp);

           rlocs.push_back(Point2D<int>(horz*scale, vert*scale));
           //LINFO("scale: %d [%3d %3d]", scale, horz*scale, vert*scale);
         }
     }

   std::pair<rutz::shared_ptr<std::vector<std::vector<Image<double> > > >,
    std::vector<Point2D<int> > > temp(w, rlocs);   
  return temp;
}

// ######################################################################
void initializeModel(ViSTARSmodel &model, Image<double> stim)
{
  rutz::shared_ptr<ViSTARSmodelState> 
    state( new ViSTARSmodelState());

  uint ws = 0, hs = 0;
  for(uint i = 0; i < model.p_NumSpeeds; i++)
    {
      // shrink the image properly
      // s = ['s' num2str(ss)];
      // if ss == 1, model.I.(s)(:,:,1) = stim(:,:,1);
      // else model.I.(s)(:,:,1) = shrink(stim(:,:,1), 2^(ss-1));
      // end
      // model.I.(s)(:,:,2) =  1-model.I.(s)(:,:,1);
      std::vector<Image<double> > tI;
      if(i == 0) 
        {
          // the stimuli and its negative
          tI.push_back(stim);
          tI.push_back((stim*-1.0) + 1.0);
        }
      else   
        {
          Image<double> temp = shrink(stim, pow(2, i));
          tI.push_back(temp);
          tI.push_back((temp*-1.0) + 1.0);
        }

      // uint sj = 0, ej = 20, si = 0, ei = 20;
      // LINFO("s[%d] cs[%d]", i, 0);
      // for(uint y = sj; y < ej; y++)
      //   {
      //     for(uint x = si; x < ei; x++)
      //       printf("%10.4f ", tI[0].getVal(x,y));
      //     printf("\n");
      //   }
      // LINFO("s[%d] cs[%d]", i, 1);
      // for(uint y = sj; y < ej; y++)
      //   {
      //     for(uint x = si; x < ei; x++)
      //       printf("%10.4f ", tI[1].getVal(x,y));
      //     printf("\n");
      //   }
      // Raster::waitForKey();
      

      state->I.push_back(tI);
      
      uint width  = tI[0].getWidth();
      uint height = tI[0].getHeight();

      // model.a.(s) = zeros(size(model.I.(s),1), size(model.I.(s),2), 2);
      std::vector<Image<double> > tA;
      Image<double> tA1(width, height, ZEROS);
      Image<double> tA2(width, height, ZEROS);
      tA.push_back(tA1);
      tA.push_back(tA2);
      state->a.push_back(tA);

      // model.b.(s) = model.a.(s);
      std::vector<Image<double> > tB;
      Image<double> tB1(width, height, ZEROS);
      Image<double> tB2(width, height, ZEROS);
      tB.push_back(tB1);
      tB.push_back(tB2);
      state->b.push_back(tB);

      // model.x.(s) = model.a.(s); 
      std::vector<Image<double> > tX;
      Image<double> tX1(width, height, ZEROS);
      Image<double> tX2(width, height, ZEROS);
      tX.push_back(tX1);
      tX.push_back(tX2);
      state->x.push_back(tX);

      // model.y.(s) = model.a.(s);
      std::vector<Image<double> > tY;
      Image<double> tY1(width, height, ZEROS);
      Image<double> tY2(width, height, ZEROS);
      tY.push_back(tY1);
      tY.push_back(tY2);
      state->y.push_back(tY);

      // model.z.(s) = ones(size(model.I.(s),1), size(model.I.(s),2), 2);
      std::vector<Image<double> > tZ;
      Image<double> tZ1(width, height, ZEROS);
      Image<double> tZ2(width, height, ZEROS);
      tZ.push_back(tZ1+1.0);
      tZ.push_back(tZ2+1.0);
      state->z.push_back(tZ);

      // model.c.(s) = zeros(size(model.I.(s),1), size(model.I.(s),2), 2, 8);
      std::vector<std::vector<Image<double> > > tC;
      for(uint j = 0; j < NUM_DIRS; j++)
        {
          std::vector<Image<double> > tC1;

          Image<double> tC11(width, height, ZEROS);
          Image<double> tC12(width, height, ZEROS);
          tC1.push_back(tC11);
          tC1.push_back(tC12);

          tC.push_back(tC1);
        }
      state->c.push_back(tC);
      
      // model.e.(s) = model.c.(s);
      std::vector<std::vector<Image<double> > > tE;
      for(uint j = 0; j < NUM_DIRS; j++)
        {
          std::vector<Image<double> > tE1;

          Image<double> tE11(width, height, ZEROS);
          Image<double> tE12(width, height, ZEROS);
          tE1.push_back(tE11);
          tE1.push_back(tE12);

          tE.push_back(tE1);
        }
      state->e.push_back(tE);

      // model.f.(s) = zeros(size(model.I.(s),1), size(model.I.(s),2), 1, 8);
      std::vector<Image<double> > tF;
      for(uint j = 0; j < NUM_DIRS; j++)
        {
          Image<double> tF1(width, height, ZEROS);
          tF.push_back(tF1);
        }
      state->f.push_back(tF);

      if(i == model.p_NumSpeeds-1)
        {
          ws = width;
          hs = height;
        }
    }

  // model.q = zeros(size(model.I.(s),1), size(model.I.(s),2), 1, 8);
  std::vector<Image<double> > tq;
  for(uint i = 0; i < NUM_DIRS; i++)
    {
      Image<double> tq1(ws, hs, ZEROS);
      tq.push_back(tq1);
    }
  state->q = tq;

  // model.Q = model.q;
  std::vector<Image<double> > tQ;
  for(uint i = 0; i < NUM_DIRS; i++)
    {
      Image<double> tQ1(ws, hs, ZEROS);
      tQ.push_back(tQ1);
    }
  state->Q = tQ;

  // model.m = model.q;
  std::vector<Image<double> > tm;
  for(uint i = 0; i < NUM_DIRS; i++)
    {
      Image<double> tm1(ws, hs, ZEROS);
      tm.push_back(tm1);
    }
  state->m = tm;

  // model.r = zeros(model.p.M, 1);
  state->r = std::vector<double>(model.p_M);
  for(uint i = 0; i < model.p_M; i++)
    state->r[i] = 0.0;

  // model.R = model.r;
  state->R = std::vector<double>(model.p_M);
  for(uint i = 0; i < model.p_M; i++)
    state->R[i] = 0.0;

  model.state = state;
}

// ######################################################################
// SUB FUNCTION: shrink is used to sum over groups of cells to shrink the array.
// function x = shrink(y, f)
Image<double> shrink(Image<double> y, double f)
{
  //if f > 1, ff = 1/f; 
  // else ff = f; f = 1/ff; end
  double ff;
  if(f> 1) { ff = 1/f; }
  else     { ff = f; f = 1/ff; } 

  uint w = y.getWidth();
  uint h = y.getHeight();
  uint width  = floor(w*ff);
  uint height = floor(h*ff);
  Image<double> x(width, height, NO_INIT);

  //LINFO("f: %f, ff:%f width: %d height: %d", f,ff,width,height);

  // SKIP this dimension: for t = 1:size(y,3) --> it's 1

  //for i=1:floor(size(y,1)*ff)
  //  for j=1:floor(size(y,2)*ff)
  for(uint i = 0; i < width; i++)
    for(uint j = 0; j < height; j++)
      {       
        // x(i,j,t,:) = sum(sum(y(i*f-f+1:i*f, j*f-f+1:j*f, t, :)))/f^2;
        double val = 0.0;
        uint sk = uint((i+1)*f-f);
        uint ek = uint((i+1)*f);

        uint sl = uint((j+1)*f-f);
        uint el = uint((j+1)*f);
        
        for(uint k = sk; k < ek; k++)
          for(uint l = sl; l < el; l++)
              val += y.getVal(k,l);
        //LINFO("val  %f", val);
        val /= (f*f);
        //LINFO("%f val/f2  %f", f*f, val/(f*f));
        x.setVal(i,j,val);       

        //LINFO("sk:%d ek:%d sl:%d el:%d : %f", sk, ek, sl, el,val);
        //Raster::waitForKey();
      }

  return x;
}

// ######################################################################
Point2D<int> getFOE(ViSTARSmodel &model, Image<double> stim)
{  
  // update the frame
  updateFrame(model, stim);

  Point2D<int> foe;
  uint nstep = uint(1.0/tstep);
  for(uint i = 0; i < nstep; i++)
    {
      // run model
      foe = bgm_2007(model);

      // logging, reporting, scripts etc
      report(model); 
    }

  FILE *fp;      
  LINFO("FOE%d %d", foe.i, foe.j);
  if((fp = fopen("BMG_walking.txt","at")) == NULL) LFATAL("not found");
  fputs(sformat("%d %d \n", foe.i, foe.j).c_str(), fp);
  fclose (fp);  

  return foe;
}

// ######################################################################
void updateFrame(ViSTARSmodel &model, Image<double> stim)
{
  //     for ss=1:model.p.NumSpeeds
  std::vector<std::vector<Image<double> > > tI(model.p_NumSpeeds);
  for(uint i = 0; i < model.p_NumSpeeds; i++)
    {
      // shrink the image properly
      // s = ['s' num2str(ss)];
      // if ss == 1, model.I.(s)(:,:,1) = stim(:,:,1);
      // else model.I.(s)(:,:,1) = shrink(stim(:,:,1), 2^(ss-1));
      // end
      // model.I.(s)(:,:,2) =  1-model.I.(s)(:,:,1);
      std::vector<Image<double> > tIcs;
      if(i == 0) 
        {
          // the stimuli and its negative
          tIcs.push_back(stim);
          tIcs.push_back((stim*-1.0) + 1.0);
        }
      else   
        {
          Image<double> temp = shrink(stim, pow(2, i));
          tIcs.push_back(temp);
          tIcs.push_back((temp*-1.0) + 1.0);
        }
      tI[i] = tIcs;
    }
  model.state->I = tI;
}

uint mainIndex = 0;
// ######################################################################
Point2D<int> bgm_2007(ViSTARSmodel &model)
{
  // Since s corresponds to the scale of the input it cannot be easily
  // incorporated into a higher dimensional matrix, therefore a structure is
  // used, indexes i and j are the first 2 dimensions of the matrix, on-off
  // index (p) is the 3rd dimension of the matrix, direction index(d) is
  // included as the 4th dimension of the matrix

  // get the parameters from the input and reduce notation
  //p = in.p; o.p = p;
  
  // run the simulation
  //for ss=1:p.NumSpeeds
  //   s = ['s' num2str(ss)];

  std::vector<std::vector<Image<double> > > oI(model.p_NumSpeeds);
  std::vector<std::vector<Image<double> > > oa(model.p_NumSpeeds);
  std::vector<std::vector<Image<double> > > ob(model.p_NumSpeeds);
  std::vector<std::vector<Image<double> > > ox(model.p_NumSpeeds);
  std::vector<std::vector<Image<double> > > oy(model.p_NumSpeeds);
  std::vector<std::vector<Image<double> > > oz(model.p_NumSpeeds);
  std::vector<std::vector<std::vector<Image<double> > > > oc(model.p_NumSpeeds);
  std::vector<std::vector<std::vector<Image<double> > > > oe(model.p_NumSpeeds);
  std::vector<std::vector<Image<double> > > of(model.p_NumSpeeds);
  std::vector<Image<double> > om(NUM_DIRS);
      
  for(uint s = 0; s < model.p_NumSpeeds; s++)
    {
      // The input needs to be scaled and the on-off channels need to be
      // segregated such that in.I.s holds the scaled inputs
     
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // Level 1, On-Center-Off-Surround Processing for Boundaries
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //I = in.I.(s); a = in.a.(s); b = in.b.(s); o.I.(s) = I;
      std::vector<Image<double> > I   = model.state->I[s];
      std::vector<Image<double> > a   = model.state->a[s];
      std::vector<Image<double> > b   = model.state->b[s];
      std::vector<Image<double> > oIs = model.state->I[s];
      std::vector<Image<double> > oas(NUM_CS);
      std::vector<Image<double> > obs(NUM_CS);

      std::vector<Image<double> > sumfxy = sumFxyI(model.p_Fxy,I);
      for(uint cs = 0; cs < NUM_CS; cs++)
        {
          // o.a.(s) = a + tstep*(-p.A1*a + (p.B1 - a).*I*p.C1 - 
          //                      (p.D1 + a).*sumFxyI(p.Fxy, I)   ); % 1.1
          oas[cs] = a[cs] +  ((a[cs]*-model.p_A1) + 
                              ((a[cs]*-1.0) + model.p_B1)*I[cs]*model.p_C1 -
                              (a[cs] + model.p_D1)*sumfxy[cs]
                              )*tstep;

          // tr = max(o.a.(s)-in.p.phi1, 0).^2
          uint w = oas[cs].getWidth();
          uint h = oas[cs].getHeight();
          Image<double> temp = oas[cs] - model.p_phi1;
          Image<double> temp2(w, h, ZEROS);
          Image<double> temp3 = takeMax(temp,temp2);
          Image<double> tr = temp3*temp3;

 // LINFO("I s[%d] cs[%d]", s, cs);
 // uint sj = 0, ej = 20, si = 0, ei = 20;
 // for(uint y = sj; y < ej; y++)
 //   {
 //     for(uint x = si; x < ei; x++)
 //       printf("%10.4f ", I[cs].getVal(x,y));
 //     printf("\n");
 //   }
 //  Raster::waitForKey();
 // LINFO("a s[%d] cs[%d]", s, cs);
 // //uint sj = 0, ej = 20, si = 0, ei = 20;
 // for(uint y = sj; y < ej; y++)
 //   {
 //     for(uint x = si; x < ei; x++)
 //       printf("%10.4f ", a[cs].getVal(x,y));
 //     printf("\n");
 //   }
 // Raster::waitForKey();


 //  LINFO("p_A1: %f", model.p_A1);
 //  LINFO("p_B1: %f", model.p_B1);
 //  LINFO("p_C1: %f", model.p_C1);
 //  LINFO("p_D1: %f", model.p_D1);
                    


 //  Image<double> sfxy = sumfxy[cs];
 // LINFO("sfxy");
 // //uint sj = 0, ej = 20, si = 0, ei = 20;
 // for(uint y = sj; y < ej; y++)
 //   {
 //     for(uint x = si; x < ei; x++)
 //       printf("%10.4f ", sfxy.getVal(x,y));
 //     printf("\n");
 //   }
 // Raster::waitForKey();


 // LINFO("oas s[%d] cs[%d]", s, cs);
 // //uint sj = 0, ej = 20, si = 0, ei = 20;
 // for(uint y = sj; y < ej; y++)
 //   {
 //     for(uint x = si; x < ei; x++)
 //       printf("%10.4f ", oas[cs].getVal(x,y));
 //     printf("\n");
 //   }
 // Raster::waitForKey();
       
 // LINFO("I %20.10f ", I[cs].getVal(0,19));
 // LINFO("a %20.10f ", a[cs].getVal(0,19));
 // LINFO("s %20.10f ", sfxy.getVal(0,19));
 // LINFO("o %20.10f ", oas[cs].getVal(0,19));

 // LINFO("%20.10f %20.10f %20.10f %20.10f ", 
 //       model.p_A1, model.p_B1, model.p_C1, model.p_D1);

 // Image<double> aaa = a[cs]*-model.p_A1;
 // Image<double> bbb = ((a[cs]*-1.0) + model.p_B1)*I[cs]*model.p_C1;
 // Image<double> ccc = (a[cs] + model.p_D1)*sumfxy[cs];

 // LINFO("aaa %20.10f ", aaa.getVal(0,19));
 // LINFO("bbb %20.10f ", bbb.getVal(0,19));
 // LINFO("ccc %20.10f ", ccc.getVal(0,19));

          // LINFO("temp s[%d] cs[%d]", s, cs);
          // for(uint y = sj; y < ej; y++)
          //   {
          //     for(uint x = si; x < ei; x++)
          //       printf("%10.4f ", temp.getVal(x,y));
          //     printf("\n");
          //   }

          // Raster::waitForKey();


          // LINFO("temp3 s[%d] cs[%d]", s, cs);
          // for(uint y = sj; y < ej; y++)
          //   {
          //     for(uint x = si; x < ei; x++)
          //       printf("%10.4f ", temp3.getVal(x,y));
          //     printf("\n");
          //   }
          // Raster::waitForKey();
         
          // LINFO("tr s[%d] cs[%d]", s, cs);
          // for(uint y = sj; y < ej; y++)
          //   {
          //     for(uint x = si; x < ei; x++)
          //       printf("%10.4f ", tr.getVal(x,y));
          //     printf("\n");
          //   }
          // Raster::waitForKey();


          //o.b.(s) = tr ./ (p.G1^2 + tr);                           % 1.3
          obs[cs] = tr / (tr + pow(model.p_G1,2.0));

  // LINFO("obs s[%d] cs[%d]", s, cs);
  // //uint sj = 0, ej = 20, si = 0, ei = 20;
  // for(uint y = sj; y < ej; y++)
  //   {
  //     for(uint x = si; x < ei; x++)
  //       printf("%10.4f ", obs[cs].getVal(x,y));
  //     printf("\n");
  //   }
  // Raster::waitForKey();
 







        }
      //clear I a tr;


      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // Level 2, Transient Cells
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //x = in.x.(s); z = in.z.(s); y = in.y.(s);
      std::vector<Image<double> > x  = model.state->x[s];
      std::vector<Image<double> > y  = model.state->y[s];
      std::vector<Image<double> > z  = model.state->z[s];

      std::vector<Image<double> > oxs(NUM_CS);
      std::vector<Image<double> > oys(NUM_CS);
      std::vector<Image<double> > ozs(NUM_CS);

      for(uint cs = 0; cs < NUM_CS; cs++)
        {
          //uint sj = 0, ej = 20, si = 0, ei = 20;
 // LINFO("(((((((((((((())))))))))))))))x s[%d] cs[%d]", s, cs);
 // for(uint j = sj; j < ej; j++)
 //   {
 //     for(uint i = si; i < ei; i++)
 //       printf("%10.4f ", x[cs].getVal(i,j));
 //     printf("\n");
 //   }
 // Raster::waitForKey();

 // LINFO("b s[%d] cs[%d]", s, cs);
 // for(uint j = sj; j < ej; j++)
 //   {
 //     for(uint i = si; i < ei; i++)
 //       printf("%10.4f ", b[cs].getVal(i,j));
 //     printf("\n");
 //   }
 // Raster::waitForKey();











          //o.x.(s) = x + tstep*(p.A2*(-p.B2*x + (p.C2-x).*b));       % 2.1
          oxs[cs] =  x[cs] + (((x[cs]*-model.p_B2) + 
                               ((x[cs]*-1.0) + model.p_C2)*b[cs])*model.p_A2)*tstep;

          // double first = x[cs].getVal(0,0);
          // double secon = b[cs].getVal(0,0);

          // double val = x[cs].getVal(0,0) + (((x[cs].getVal(0,0)*-model.p_B2) + 
          //                                   ((x[cs].getVal(0,0)*-1.0) + model.p_C2)*b[cs].getVal(0,0))*model.p_A2)*tstep;
          // LINFO("x: %f b: %f p: A2: %f B2: %f C2: %f val: %f ", first, secon, model.p_A2, model.p_B2, model.p_C2, val);


 // Raster::waitForKey();





          
          //o.z.(s) = z + tstep*(p.D2*(1 - z - p.K2*x.*z));           % 2.2
          ozs[cs] =  z[cs] + (((z[cs]*-1.0) - (x[cs]*z[cs]*model.p_K2) + 1.0)*model.p_D2)*tstep;          

          //o.y.(s) = max(o.x.(s).*o.z.(s),0);                        % 2.3
          uint w = oxs[cs].getWidth();
          uint h = oxs[cs].getHeight();
          Image<double> temp(w, h, ZEROS);
          oys[cs] = takeMax(oxs[cs]*ozs[cs], temp);


 // LINFO("oxs s[%d] cs[%d]", s, cs);
 // for(uint j = sj; j < ej; j++)
 //   {
 //     for(uint i = si; i < ei; i++)
 //       printf("%10.4f ", oxs[cs].getVal(i,j));
 //     printf("\n");
 //   }

 // LINFO("oys s[%d] cs[%d]", s, cs);
 // for(uint j = sj; j < ej; j++)
 //   {
 //     for(uint i = si; i < ei; i++)
 //       printf("%10.4f ", oys[cs].getVal(i,j));
 //     printf("\n");
 //   }

 // LINFO("ozs s[%d] cs[%d]", s, cs);
 // for(uint j = sj; j < ej; j++)
 //   {
 //     for(uint i = si; i < ei; i++)
 //       printf("%10.4f ", ozs[cs].getVal(i,j));
 //     printf("\n");
 //   }


 // LINFO("oxs %20.10f ", oxs[cs].getVal(0,76));
 // LINFO("oys %20.10f ", oxs[cs].getVal(0,76));
 // LINFO("ozs %20.10f ", oxs[cs].getVal(0,76));
 // LINFO("b   %20.10f ",   b[cs].getVal(0,76));


        }

      //clear b x z;
      
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // Level 3, Directional Transient Cells 
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //c = in.c.(s); e = in.e.(s); 
      std::vector<std::vector<Image<double> > > c  = model.state->c[s];
      std::vector<std::vector<Image<double> > > e  = model.state->e[s];

      std::vector<std::vector<Image<double> > >ocs(NUM_DIRS);
      std::vector<std::vector<Image<double> > >oes(NUM_DIRS);
      
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // It is possible to do this algorithmically as shown in this
      // commented code but it's faster to hard code each direction as
      // implemented below.
      //      for d=1:8
      //      if d < 5, D =d+4; else D = d-4; end
      //      theta = (d-1)*(2*pi/8); xd =round(cos(theta)); yd=round(sin(theta));
      //
      //      XY = [zeros((yd==-1), size(I,2)); 
      //            zeros(size(I,1)-(yd~=0), xd==-1) c((yd==1)+1:end-(yd==-1),(xd==1)+1:end-(xd==-1),t,D) zeros(size(I,1)-(yd~=0), xd==1); 
      //            zeros((yd==1), size(I,2))];
      //
      //      c(:,:,t+1, d) = max(c(:,:,t,d) + tstep*(Ac*(-Bc*c(:,:,t,d)+Cc*I(:,:,it) - Kc*XY )),0);
      //      e(:,:,t+1, d) = max(e(:,:,t,d) + tstep*(Ae*(-Be*e(:,:,t,d)+Ce*I(:,:,it) - Ke*XY )),0);
      //   end
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      for(uint cs = 0; cs < NUM_CS; cs++)
        {

          // LINFO("c   %20.10f ", c[0][cs].getVal(0,19));
          // LINFO("e   %20.10f ", e[0][cs].getVal(0,19));

          //nm = size(c);
          uint cWidth  = c[0][cs].getWidth();
          uint cHeight = c[0][cs].getHeight();

          // Rightward direction
          //XY = max( [c(:,2:end,:,5) zeros(nm(1), 1,2)],0);
          Image<double> XY0(cWidth,cHeight,ZEROS);
          for(uint i = 0; i < cWidth-1; i++)
            for(uint j = 0; j < cHeight; j++)
              {
                double v = c[4][cs].getVal(i+1,j);
                double val = 0.0;
                if(v > 0.0) val = v;
                XY0.setVal(i,j, val);
              }

          //o.c.(s)(:,:,:,1) = 
          //  c(:,:,:,1) + 
          //  tstep*(p.A3*(-p.B3*c(:,:,:,1) + p.C3*y - p.K3*XY));  % 3.1
          ocs[0].push_back
            (c[0][cs] + 
             (((c[0][cs]*-model.p_B3) + 
               y[cs]*model.p_C3 - XY0*model.p_K3)*model.p_A3)*tstep);

          //o.e.(s)(:,:,:,1) = 
          //  e(:,:,:,1) + 
          //  tstep*(p.A4*(-p.B4*e(:,:,:,1) + p.C4*y - p.K4*XY));  % 3.2
          oes[0].push_back
            (e[0][cs] + 
             (((e[0][cs]*-model.p_B4) + 
               y[cs]*model.p_C4 - XY0*model.p_K4)*model.p_A4)*tstep);          

          // UP-Rightward direction
          //XY = max( [c(2:end,2:end,:,6) zeros(nm(1)-1, 1,2); 
          //           zeros(1, nm(2),2)],0);
          Image<double> XY1(cWidth,cHeight,ZEROS);
          for(uint i = 0; i < cWidth-1; i++)
            for(uint j = 0; j < cHeight-1; j++)
              {
                double v = c[5][cs].getVal(i+1,j+1);
                double val = 0.0;
                if(v > 0.0) val = v;
                XY1.setVal(i,j, val);
              }

          //o.c.(s)(:,:,:,2) = 
          //  c(:,:,:,2) + 
          //  tstep*(p.A3*(-p.B3*c(:,:,:,2) + p.C3*y - p.K3*XY));  % 3.1 
          ocs[1].push_back
            (c[1][cs] + 
             (((c[1][cs]*-model.p_B3) + 
               y[cs]*model.p_C3 - XY1*model.p_K3)*model.p_A3)*tstep);

          //o.e.(s)(:,:,:,2) = 
          //  e(:,:,:,2) + 
          //  tstep*(p.A4*(-p.B4*e(:,:,:,2) + p.C4*y - p.K4*XY));  % 3.2
          oes[1].push_back
            (e[1][cs] + 
             (((e[1][cs]*-model.p_B4) + 
               y[cs]*model.p_C4 - XY1*model.p_K4)*model.p_A4)*tstep);          
                    
          // Upward direction
          //XY = max( [c(2:end,:,:,7); zeros(1, nm(2),2) ],0);
          Image<double> XY2(cWidth,cHeight,ZEROS);
          for(uint i = 0; i < cWidth; i++)
            for(uint j = 0; j < cHeight-1; j++)
              {
                double v = c[6][cs].getVal(i,j+1);
                double val = 0.0;
                if(v > 0.0) val = v;
                XY2.setVal(i,j, val);
              }

          //o.c.(s)(:,:,:,3) = 
          //  c(:,:,:,3) + 
          //  tstep*(p.A3*(-p.B3*c(:,:,:,3) + p.C3*y - p.K3*XY));  % 3.1 
          ocs[2].push_back
            (c[2][cs] + 
             (((c[2][cs]*-model.p_B3) + 
               y[cs]*model.p_C3 - XY2*model.p_K3)*model.p_A3)*tstep);

          //o.e.(s)(:,:,:,3) = 
          //  e(:,:,:,3) + 
          //  tstep*(p.A4*(-p.B4*e(:,:,:,3) + p.C4*y - p.K4*XY));  % 3.2
          oes[2].push_back
            (e[2][cs] + 
             (((e[2][cs]*-model.p_B4) + 
               y[cs]*model.p_C4 - XY2*model.p_K4)*model.p_A4)*tstep);          
                    
          // Up-Leftward direction
          //XY = max( [zeros(nm(1)-1, 1,2) c(2:end,1:end-1,:,8); 
          //           zeros(1, nm(2),2) ],0);
          Image<double> XY3(cWidth,cHeight,ZEROS);
          for(uint i = 1; i < cWidth; i++)
            for(uint j = 0; j < cHeight-1; j++)
              {
                double v = c[7][cs].getVal(i-1,j+1);
                double val = 0.0;
                if(v > 0.0) val = v;
                XY3.setVal(i,j, val);
              }

          //o.c.(s)(:,:,:,4) = 
          //  c(:,:,:,4) + 
          //  tstep*(p.A3*(-p.B3*c(:,:,:,4) + p.C3*y - p.K3*XY));  % 3.1 
          ocs[3].push_back
            (c[3][cs] + 
             (((c[3][cs]*-model.p_B3) + 
               y[cs]*model.p_C3 - XY3*model.p_K3)*model.p_A3)*tstep);

          //o.e.(s)(:,:,:,4) = 
          //  e(:,:,:,4) + 
          //  tstep*(p.A4*(-p.B4*e(:,:,:,4) + p.C4*y - p.K4*XY));  % 3.2
          oes[3].push_back
            (e[3][cs] + 
             (((e[3][cs]*-model.p_B4) + 
               y[cs]*model.p_C4 - XY3*model.p_K4)*model.p_A4)*tstep);
          
          // Leftward direction
          //XY =  max( [zeros(nm(1), 1,2) c(:,1:end-1,:,1) ],0);
          Image<double> XY4(cWidth,cHeight,ZEROS);
          for(uint i = 1; i < cWidth; i++)
            for(uint j = 0; j < cHeight; j++)
              {
                double v = c[0][cs].getVal(i-1,j);
                double val = 0.0;
                if(v > 0.0) val = v;
                XY4.setVal(i,j, val);
              }

          //o.c.(s)(:,:,:,5) = 
          //  c(:,:,:,5) + 
          //  tstep*(p.A3*(-p.B3*c(:,:,:,5) + p.C3*y - p.K3*XY));  % 3.1 
          ocs[4].push_back
            (c[4][cs] + 
             (((c[4][cs]*-model.p_B3) + 
               y[cs]*model.p_C3 - XY4*model.p_K3)*model.p_A3)*tstep);

          //o.e.(s)(:,:,:,5) = 
          //  e(:,:,:,5) + 
          //  tstep*(p.A4*(-p.B4*e(:,:,:,5) + p.C4*y - p.K4*XY));  % 3.2
          oes[4].push_back
            (e[4][cs] + 
             (((e[4][cs]*-model.p_B4) + 
               y[cs]*model.p_C4 - XY4*model.p_K4)*model.p_A4)*tstep);          

          // Down-Leftward direction
          //XY = max( [zeros(1, nm(2),2); zeros(nm(1)-1, 1,2) 
          //          c(1:end-1,1:end-1,:,2) ],0);
          Image<double> XY5(cWidth,cHeight,ZEROS);
          for(uint i = 1; i < cWidth; i++)
            for(uint j = 1; j < cHeight; j++)
              {
                double v = c[1][cs].getVal(i-1,j-1);
                double val = 0.0;
                if(v > 0.0) val = v;
                XY5.setVal(i,j, val);
              }

          //o.c.(s)(:,:,:,6) = 
          //  c(:,:,:,6) + 
          //  tstep*(p.A3*(-p.B3*c(:,:,:,6) + p.C3*y - p.K3*XY));  % 3.1 
          ocs[5].push_back
            (c[5][cs] + 
             (((c[5][cs]*-model.p_B3) + 
               y[cs]*model.p_C3 - XY5*model.p_K3)*model.p_A3)*tstep);

          //o.e.(s)(:,:,:,6) = 
          //  e(:,:,:,6) + 
          //  tstep*(p.A4*(-p.B4*e(:,:,:,6) + p.C4*y - p.K4*XY));  % 3.2
          oes[5].push_back
            (e[5][cs] + 
             (((e[5][cs]*-model.p_B4) + 
               y[cs]*model.p_C4 - XY5*model.p_K4)*model.p_A4)*tstep);          
          
          // Downward direction
          //XY = max( [zeros(1, nm(2),2); c(1:end-1,:,:,3) ],0);
          Image<double> XY6(cWidth,cHeight,ZEROS);
          for(uint i = 0; i < cWidth; i++)
            for(uint j = 1; j < cHeight; j++)
              {
                double v = c[2][cs].getVal(i,j-1);
                double val = 0.0;
                if(v > 0.0) val = v;
                XY6.setVal(i,j, val);
              }

          //o.c.(s)(:,:,:,7) = 
          //  c(:,:,:,7) + 
          //  tstep*(p.A3*(-p.B3*c(:,:,:,7) + p.C3*y - p.K3*XY));  % 3.1 
          ocs[6].push_back
            (c[6][cs] + 
             (((c[6][cs]*-model.p_B3) + 
               y[cs]*model.p_C3 - XY6*model.p_K3)*model.p_A3)*tstep);

          //o.e.(s)(:,:,:,7) = 
          //  e(:,:,:,7) + 
          //  tstep*(p.A4*(-p.B4*e(:,:,:,7) + p.C4*y - p.K4*XY));  % 3.2
          oes[6].push_back
            (e[6][cs] + 
             (((e[6][cs]*-model.p_B4) + 
               y[cs]*model.p_C4 - XY6*model.p_K4)*model.p_A4)*tstep);          
          
          // Downward-Rightward direction
          //XY = max( [zeros(1, nm(2),2); c(1:end-1,2:end,:,4) 
          //          zeros(nm(1)-1, 1,2) ],0); 
          Image<double> XY7(cWidth,cHeight,ZEROS);
          for(uint i = 0; i < cWidth-1; i++)
            for(uint j = 1; j < cHeight; j++)
              {
                double v = c[3][cs].getVal(i+1,j-1);
                double val = 0.0;
                if(v > 0.0) val = v;
                XY7.setVal(i,j, val);
              }

          //o.c.(s)(:,:,:,8) = 
          //  c(:,:,:,8) + 
          //  tstep*(p.A3*(-p.B3*c(:,:,:,8) + p.C3*y - p.K3*XY));  % 3.1 
          ocs[7].push_back
            (c[7][cs] + 
             (((c[7][cs]*-model.p_B3) + 
               y[cs]*model.p_C3 - XY7*model.p_K3)*model.p_A3)*tstep);

          //o.e.(s)(:,:,:,8) = 
          //  e(:,:,:,8) + 
          //  tstep*(p.A4*(-p.B4*e(:,:,:,8) + p.C4*y - p.K4*XY));  % 3.2
          oes[7].push_back
            (e[7][cs] + 
             (((e[7][cs]*-model.p_B4) + 
               y[cs]*model.p_C4 - XY7*model.p_K4)*model.p_A4)*tstep);          
   
          // NOT CODED:
          //%o.c.(s) = max(o.c.(s), 0);
          //%o.e.(s) = max(o.e.(s), 0);
        }
      //clear XY y c;

      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // Level 4, Feature Enhancement via Directional Competition
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      //f = in.f.(s); 
      std::vector<Image<double> > f  = model.state->f[s];      
      std::vector<Image<double> > ofs(NUM_DIRS);

      uint w = f[0].getWidth();
      uint h = f[0].getHeight();
      std::vector<Image<double> > sump(NUM_DIRS);
      Image<double> sumDp(w,h, ZEROS);
      //sump = sum(m ax(e,0),3); sumDp = sum(sump ,4);
      for(uint dir = 0; dir < NUM_DIRS; dir++)
        {
          Image<double> temp(w,h,ZEROS);
          for(uint cs = 0; cs < NUM_CS; cs++)
            {
              Image<double> temp2(w,h, ZEROS);
              Image<double> temp3 = takeMax(temp2, e[dir][cs]);
              temp += temp3;
            }
          sump[dir] = temp;
          sumDp += temp;
          //LINFO("acc s [%d]  %20.10f %20.10f", 
          //      dir, sumDp.getVal(0,76), temp.getVal(0,76));
        }

      //for d = 1:8, sumDnedSump(:,:,:,d) = sumDp - sump(:,:,:,d); end
      std::vector<Image<double> > sumDnedSump(NUM_DIRS);
      for(uint dir = 0; dir < NUM_DIRS; dir++)
        {
          sumDnedSump[dir] = sumDp - sump[dir];
          // LINFO("s [%d]  %20.10f ", dir, sumDp.getVal(0,76));
          // LINFO("s [%d]  %20.10f ", dir, sump[dir].getVal(0,76));
          // LINFO("sd[%d]  %20.10f ", dir, sumDnedSump[dir].getVal(0,76));

        }
      
      //o.f.(s) = f + 
      //  tstep*(-p.A5*f + (p.B5-f).*sump - (p.C5+f).*sumDnedSump); % 4.1
      for(uint dir = 0; dir < NUM_DIRS; dir++)
        {
          ofs[dir] = f[dir] + ((f[dir]*-model.p_A5) + 
                               ((f[dir]*-1.0)+model.p_B5)*sump[dir] - 
                               (f[dir]+model.p_C5)*sumDnedSump[dir])*tstep;
        }


      //uint sj = 0, ej = 20, si = 0, ei = 20;  
      // LINFO("f");
      // for(uint y = sj; y < ej; y++)
      //   {
      //     for(uint x = si; x < ei; x++)
      //       printf("%10.5f ", f[0].getVal(x,y));
      //     printf("\n");
      //   }

      // LINFO("sump");
      // for(uint y = sj; y < ej; y++)
      //   {
      //     for(uint x = si; x < ei; x++)
      //       printf("%10.5f ", sump[0].getVal(x,y));
      //     printf("\n");
      //   }

      // LINFO("sumDp");
      // for(uint y = sj; y < ej; y++)
      //   {
      //     for(uint x = si; x < ei; x++)
      //       printf("%10.5f ", sumDp.getVal(x,y));
      //     printf("\n");
      //   }

      // LINFO("sumDnedSump");
      // for(uint y = sj; y < ej; y++)
      //   {
      //     for(uint x = si; x < ei; x++)
      //       printf("%10.5f ", sumDnedSump[0].getVal(x,y));
      //     printf("\n");
      //   }

//  LINFO("ofs");
//  for(uint y = sj; y < ej; y++)
//    {
//      for(uint x = si; x < ei; x++)
//        printf("%10.5f ", ofs[0].getVal(x,y));
//      printf("\n");
//    }


//   Image<double> sump0        = sump[0];
//   Image<double> sumDnedSump0 = sumDnedSump[0];

//   LINFO("s0  %20.10f ", sump0.getVal(0,76));
//   LINFO("sd0 %20.10f ", sumDnedSump0.getVal(0,76));
//   LINFO("f0 %20.10f ", f[0].getVal(0,76));

//   LINFO("param: %20.10f %20.10f %20.10f", 
//         model.p_A5, model.p_B5, model.p_C5);

//   Image<double> aaa = f[0]*-model.p_A5;
//   Image<double> bbb = ((f[0]*-1.0)+model.p_B5)*sump[0];
//   Image<double> ccc = (f[0]+model.p_C5)*sumDnedSump[0];

//   LINFO("aaa %20.10f ", aaa.getVal(0,76));
//   LINFO("bbb %20.10f ", bbb.getVal(0,76));
//   LINFO("ccc %20.10f ", ccc.getVal(0,76));


// LINFO("ofs   %20.10f ", ofs[0].getVal(0,76));




      // Raster::waitForKey();


      
      //clear e sump sumDp sumDnedSump f;

      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // Levels 5 & 6 are not scale specific, resize and combine scales
      // this is a combination of eqn 5.1 and the sum over s in 5.2
      // shrink function is implemented in functions section below
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //n = 2^(p.NumSpeeds-ss);
      uint n = uint(pow(2.0, model.p_NumSpeeds - s - 1));
      
      for(uint dir = 0; dir < NUM_DIRS; dir++)
        {
          //if ss == 1 
          if(s == 0)
            {
              // first speed so initialize o.m
              //  o.m = (1/n)*combine(max(o.f.(s),0), n); 
              om[dir] = 
                combine(takeMax(ofs[dir],Image<double>(w,h,ZEROS)), n)*(1.0/n);
            }
          //elseif ss == p.NumSpeeds
          else if(s == model.p_NumSpeeds-1)
            {
              // last speed so no need to resize
              //  o.m = o.m + (1/n)*max(o.f.(s),0);      
              om[dir] = 
                om[dir] + takeMax(ofs[dir],Image<double>(w,h,ZEROS))*(1.0/n);
            }
          // else
          else
            {
              // resize and sum across scales
              //  o.m = o.m + (1/n)*combine(max(o.f.(s),0), n); 
              om[dir] = 
                om[dir] + 
                combine(takeMax(ofs[dir],Image<double>(w,h,ZEROS)), n)*(1.0/n);
            }
        }






 //      uint sj = 0, ej = 20, si = 0, ei = 20;  
 //  LINFO("INSIDE om");
 //  for(uint y = sj; y < ej; y++)
 //    {
 //      for(uint x = si; x < ei; x++)
 //        printf("%10.5f ", om[0].getVal(x,y));
 //      printf("\n");
 //    }
 // LINFO("om   %20.10f ", om[0].getVal(0,19));




      oI[s] = oIs;
      oa[s] = oas;
      ob[s] = obs;
      ox[s] = oxs;
      oy[s] = oys;
      oz[s] = ozs;
      oc[s] = ocs;
      oe[s] = oes;
      of[s] = ofs;
    }

 // LINFO("om");
 // uint sj = 0, ej = 20, si = 0, ei = 20;  
 // for(uint y = sj; y < ej; y++)
 //  {
 //    for(uint x = si; x < ei; x++)
 //      printf("%10.5f ", om[0].getVal(x,y));
 //    printf("\n");
 //  }
 // LINFO("om   %20.10f ", om[0].getVal(0,19));
 

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // % there is edge effect from the directional transients 1 pixel wide
  // % around the array, remove this
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // o.m([1 end], :, :, :) = 0;
  // o.m(:, [1 end], :, :) = 0;
  uint wom = om[0].getWidth();
  uint hom = om[0].getHeight();
  for(uint dir = 0; dir < NUM_DIRS; dir++)
    {
      // take out top and bottom edge
      for(uint i = 0; i < wom; i++)
        {
          om[dir].setVal(i, 0    , 0.0);
          om[dir].setVal(i, hom-1, 0.0);
        }

      // take out left and right edge
      for(uint j = 0; j < hom; j++)
        {
          om[dir].setVal(0    , j, 0.0);
          om[dir].setVal(wom-1, j, 0.0);
        }
    }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // % Level 5, MT+      Optic Flow
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // q=in.q; r = in.r; Q = in.Q; R = in.R;
  std::vector<Image<double> > q = model.state->q;
  std::vector<Image<double> > Q = model.state->Q;
  std::vector<double> r = model.state->r;
  std::vector<double> R = model.state->R;

  std::vector<Image<double> > oq(NUM_DIRS);
  std::vector<Image<double> > oQ(NUM_DIRS);
  std::vector<Image<double> > qEx(NUM_DIRS);

  std::vector<Image<double> > m = model.state->m;

  // qEx = sumLxy(p.L, in.m).*((p.C6/p.M)*sumRz(R,p.w)+1) + p.D6*Q;
  std::vector<Image<double> > sumrz  = sumRz(R, model.p_w);
  std::vector<Image<double> > sumlxy = sumLxy(model.p_L, m);
  for(uint dir = 0; dir < NUM_DIRS; dir++)
    {
      qEx[dir] = sumlxy[dir] * (sumrz[dir]*(model.p_C6/model.p_M) +1.0) + 
        Q[dir]*model.p_D6; 
    }

  // o.q = q + tstep*(-p.A6*q + (p.B6-q).*qEx - q.*sumvD(p.vdD, Q));     % 5.2
  std::vector<Image<double> > sumvd = sumvD(model.p_vdD, Q);
  for(uint dir = 0; dir < NUM_DIRS; dir++)
    {
      oq[dir] =  q[dir] + 
        ((q[dir]*-model.p_A6) + ((q[dir]*-1.0)+model.p_B6) * 
         qEx[dir] - q[dir]*sumvd[dir])*tstep; 
    }

  // o.Q = max(q-p.theta6, 0).^2;                                        % 5.3
  uint wq = q[0].getWidth(); 
  uint hq = q[0].getHeight(); 
  for(uint dir = 0; dir < NUM_DIRS; dir++)
    {
      oQ[dir] = toPower(takeMax(q[dir]-model.p_theta6, 
                                Image<double>(wq,hq,ZEROS)), 2.0);
    }


 //   LINFO("qEx   %20.10f ", qEx[0].getVal(0,19));
 //   LINFO("srz   %20.10f ", sumrz[0].getVal(0,19));
 //   LINFO("sumvd %20.10f ", sumvd[0].getVal(0,19));
 //   LINFO("q     %20.10f ", q[0].getVal(0,19));
 //   LINFO("Q     %20.10f ", Q[0].getVal(0,19));

 //   LINFO("param: %20.10f %20.10f %20.10f %20.10f", 
 //         model.p_A6, model.p_B6, model.p_C6, model.p_D6);

 //   Image<double> aaa = q[0]*-model.p_A6;
 //   Image<double> bbb = ((q[0]*-1.0)+model.p_B6) * qEx[0];
 //   Image<double> ccc = q[0]*sumvd[0];

 //   LINFO("aaa %20.10f ", aaa.getVal(0,19));
 //   LINFO("bbb %20.10f ", bbb.getVal(0,19));
 //   LINFO("ccc %20.10f ", ccc.getVal(0,19));


 // LINFO("oQ   %20.10f \n\n\n", oQ[0].getVal(0,19));




  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // % Level 6, MSTd     Heading
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //for ri = 1:size(p.w,3)
  std::vector<double> rEx(model.p_w->size());
  for(uint ri = 0; ri < model.p_w->size(); ri++)
    {
      double total = 0.0;
      for(uint dir = 0; dir < NUM_DIRS; dir++)
        {            
          // w = p.w(:,:,ri,:);
          // rEx(ri,1) = w(1:end)*Q(1:end)';
          for(uint i = 0; i < wq; i++)
            for(uint j = 0; j < hq; j++)
              total += (*model.p_w)[ri][dir].getVal(i,j)*Q[dir].getVal(i,j);
        }
      rEx[ri] = total;
      //LINFO("total[%d]: %f ", ri, total);

    }

  // o.r = r + 
  //   tstep*(-p.A7*r + (p.B7-r).*((p.C7/p.N)*rEx + p.D7*R) - 
  //          p.E7*r.*(sum(R)-R));  % 5.6
  std::vector<double> orr(r.size());
  double sumR = 0.0;
  for(uint i = 0; i < r.size(); i++) sumR += R[i];  

  for(uint i = 0; i < r.size(); i++)
    {
      orr[i] = r[i] + tstep*(-model.p_A7*r[i] + 
                            (model.p_B7-r[i])*((model.p_C7/model.p_N)*rEx[i] + 
                                               model.p_D7*R[i]) 
                             - model.p_E7*r[i]*(sumR-R[i])); 
      // if(i ==0)
      //   {
      //     LINFO("[%d] r[i]: %f tstep: %f A7: %f B7: %f C7: %f E7: %f N: %f "
      //           "rEx[i]: %f sumR: %f R[i]: %f",
      //           i, r[i], tstep, model.p_A7, model.p_B7, model.p_C7, model.p_E7, 
      //           model.p_N, rEx[i], sumR, R[i]);
      //     LINFO("orr[%d]: %f", i, orr[i]);
      //   }
    }

   // LINFO("rEx   %20.10f ", rEx[19]);
   // LINFO("sumR  %20.10f ", sumR);
   // LINFO("r     %20.10f ", r[19]);
   // LINFO("R     %20.10f ", R[19]);

   // LINFO("param: %20.10f %20.10f %20.10f %20.10f %20.10f", 
   //       model.p_A7, model.p_B7, model.p_C7, model.p_E7, model.p_N);

   // double aaaa = (model.p_B7-r[19]);
   // double bbbb = (model.p_C7/model.p_N)*rEx[19]; 
   // double cccc = model.p_D7*r[19];

   // LINFO("aaaa %20.10f ", aaaa);
   // LINFO("bbbb %20.10f ", bbbb);
   // LINFO("cccc %20.10f ", cccc);
   // LINFO("orr  %20.10f ", orr[19]);

  
  // rminth = max(o.r-p.theta7, 0).^2;
  std::vector<double> rminth(r.size());
  for(uint i = 0; i < r.size(); i++)
    {
      rminth[i] = pow(max(orr[i]-model.p_theta7,0.0), 2.0);

      //LINFO("rminth[%d]: %f",i, rminth[i]);
    }
  
  // o.R = rminth ./ (p.G7^2 + rminth); 
  std::vector<double> oR(r.size());
  for(uint i = 0; i < r.size(); i++)
    {
      oR[i] =  rminth[i]/(pow(model.p_G7, 2.0) + rminth[i]);

      //LINFO("oR[%d]: %f", i, oR[i]);
    }

  // save it to the model
  model.state->a = oa;
  model.state->b = ob;
  model.state->x = ox;
  model.state->y = oy;
  model.state->z = oz;
  model.state->c = oc;
  model.state->e = oe;
  model.state->f = of;
  model.state->q = oq;
  model.state->Q = oQ;
  model.state->m = om;
  model.state->r = orr;
  model.state->R = oR;









  //==================================================================
  // Debug
  //uint sj = 0, ej = 20, si = 0, ei = 20;
 // LINFO("oI[0][0]");
 // for(uint y = sj; y < ej; y++)
 //   {
 //     for(uint x = si; x < ei; x++)
 //       printf("%10.4f ", oI[0][0].getVal(x,y));
 //     printf("\n");
 //   }
 // //Raster::waitForKey();

 // LINFO("oa[0][0]");
 // for(uint y = sj; y < ej; y++)
 //   {
 //     for(uint x = si; x < ei; x++)
 //       printf("%10.4f ", oa[0][0].getVal(x,y));
 //     printf("\n");
 //   }
 // Raster::waitForKey();

  // LINFO("ob[0][0]");
  // for(uint y = sj; y < ej; y++)
  //   {
  //     for(uint x = si; x < ei; x++)
  //       printf("%10.4f ", ob[0][0].getVal(x,y));
  //     printf("\n");
  //   }
  // Raster::waitForKey();

 // LINFO("ox[0][0]");
 // for(uint y = sj; y < ej; y++)
 //   {
 //     for(uint x = si; x < ei; x++)
 //     printf("%10.4f ", ox[0][0].getVal(x,y));
 //     printf("\n");
 //   }
 // Raster::waitForKey();

 // LINFO("oy[0][0]");
 // for(uint y = sj; y < ej; y++)
 //   {
 //     for(uint x = si; x < ei; x++)
 //     printf("%10.4f ", oy[0][0].getVal(x,y));
 //     printf("\n");
 //   }
 // //Raster::waitForKey();
 // LINFO("oy  %20.10f ", oy[0][0].getVal(0,76));

  // LINFO("oz[0][0]");
  // for(uint y = sj; y < ej; y++)
  //   {
  //     for(uint x = si; x < ei; x++)
  //       printf("%10.4f ", oz[0][0].getVal(x,y));
  //     printf("\n");
  //   }
  // //Raster::waitForKey();


  // LINFO("oc");
  // for(uint y = sj; y < ej; y++)
  //   {
  //     for(uint x = si; x < ei; x++)
  //       printf("%10.5f ", oc[0][0][0].getVal(x,y));
  //     printf("\n");
  //   }

  // LINFO("oe");
  // for(uint y = sj; y < ej; y++)
  //   {
  //     for(uint x = si; x < ei; x++)
  //       printf("%10.5f ", oe[0][0][0].getVal(x,y));
  //     printf("\n");
  //   }

  // LINFO("of");
  // for(uint y = sj; y < ej; y++)
  //   {
  //     for(uint x = si; x < ei; x++)
  //       printf("%10.5f ", of[0][0].getVal(x,y));
  //     printf("\n");
  //   }

 // LINFO("om");
 // for(uint y = sj; y < ej; y++)
 //  {
 //    for(uint x = si; x < ei; x++)
 //      printf("%10.5f ", om[0].getVal(x,y));
 //    printf("\n");
 //  }
 // LINFO("om   %20.10f ", om[0].getVal(0,19));
  
  // LINFO("oq");
  // for(uint y = sj; y < ej; y++)
  //   {
  //     for(uint x = si; x < ei; x++)
  //       printf("%10.5f ", oq[0].getVal(x,y));
  //     printf("\n");
  //   }

//  LINFO("oQ");
//  for(uint y = sj; y < ej; y++)
//    {
//      for(uint x = si; x < ei; x++)
//        printf("%10.5f ", oQ[0].getVal(x,y));
//      printf("\n");
//    }
//  LINFO("oQ  %20.10f ", oQ[0].getVal(0,19));

// LINFO("orr  %20.10f ", orr[19]);
// LINFO("rEx  %20.10f ", rEx[19]);
// LINFO("oR   %20.10f ",  oR[19]);

 // LINFO("or");
 // for(uint x = si; x < 20; x++)
 //   printf("%20.10f ", orr[x]);
 // printf("\n");

 // LINFO("rEx");
 // for(uint x = si; x < 20; x++)
 //   printf("%20.10f ", rEx[x]);
 // printf("\n");

 // LINFO("oR");
 // for(uint x = si; x < 20; x++)
 //   printf("%20.10f ", oR[x]);
 // printf("\n");


  //Raster::waitForKey();

          // uint sj = 0, ej = 20, si = 0, ei = 20;         
          // for(uint y = sj; y < ej; y++)
          //   {
          //     for(uint x = si; x < ei; x++)
          //       printf("%10.5f ", stim.getVal(x,y));
          //     printf("\n");
          //   }
          // LINFO("wow");


  // Heading
  uint hind = 0; double max= oR[0];
  for(uint i = 0; i < oR.size(); i++)
    if(max<oR[i]){ max = oR[i]; hind = i; }

  LINFO("[[%d]]heading[%d]:(%3d %3d) %f ", 
        mainIndex, hind, model.Rlocs[hind].i, model.Rlocs[hind].j, max);
  mainIndex++;

  return model.Rlocs[hind];
}

// ######################################################################
// sumFxy is used in the on-center off-surround networks to create a
// Guassian cell neighborhood
//function out = sumFxyI(F, I)
std::vector<Image<double> > sumFxyI(Image<double> F, std::vector<Image<double> > I)
{
  // 2D Gaussian Filter (F) across input (I), implemented as 2 1D filtering
  // operations for efficiency.

  std::vector<Image<double> > out;

  // for o = 1:2  % input has both on and off channels
  for(uint o = 0; o < NUM_CS; o++)
    {
      uint fw = F.getWidth();
      uint fh = F.getHeight();

      // scalar filter
      if(fw*fh == 1) 
        {
          LFATAL("NOT YET TESTED");
          LERROR("SumFxyI: filter is scalar.");
          out.push_back(matrixMult(F,I[o]));        
        }
      // Allow F to be passed as a 1D Gaussian and then filter as 2D
      else if((fw == 1)||(fh == 1))    
        {
          LFATAL("NOT YET TESTED");
          Image<double> temp = filter2(F/sum(F), I[o]);
          out.push_back(filter2(transpose((F/sum(F))), temp));
        }
      else // 2D Filter
        out.push_back(filter2(F, I[o]));
    }
  return out;
}

// ######################################################################
Image<double> filter2(Image<double> f, Image<double> d)
{
  uint fw = f.getWidth();
  uint fh = f.getHeight();

  uint fwh = fw/2;
  uint fhh = fh/2;

  uint dw = d.getWidth();
  uint dh = d.getHeight();
  
  Image<double> ret(dw, dh, NO_INIT);

  // 
  for(uint di = 0; di < dw; di++)
    for(uint dj = 0; dj < dh; dj++)
      {
        double val = 0.0;
        for(uint fi = 0; fi < fw; fi++)
          for(uint fj = 0; fj < fh; fj++)
            {
              int i = di+fi-fwh;
              int j = dj+fj-fhh;
              if(i >= 0 && j >= 0 && i < int(dw) && j < int(dh)) 
                {
                  val += d.getVal(i,j)*f.getVal(fi,fj);
                  
                  //LINFO("d[%3d %3d]f[%3d %3d] --> [%3d %3d]: %10.4f x %10.4f = %10.4f -> val: %10.4f", 
                  //      di,dj,fi,fj, i,j, 1.0, f.getVal(fi,fj), 1.0*f.getVal(fi,fj), val);
                }
            }       
        ret.setVal(di,dj, val);
      }
  return ret;
}

// ######################################################################
// combine is used to sum over groups of cells to shrink the array.
//function x = combine(y, f)
Image<double> combine(Image<double> y, double f)
{
  //if f > 1, ff = 1/f; 
  // else ff = f; f = 1/ff; end
  double ff;
  if(f > 1) { ff = 1/f; }
  else      { ff = f; f = 1/ff; } 

  uint w = y.getWidth();
  uint h = y.getHeight();
  uint width  = floor(w*ff);
  uint height = floor(h*ff);
  Image<double> x(width, height, NO_INIT);

  // SKIP this dimension: for t = 1:size(y,3) --> it's 1

  //for i=1:floor(size(y,1)*ff)
  // for j=1:floor(size(y,2)*ff)
  for(uint i = 0; i < width; i++)
    for(uint j = 0; j < height; j++)
      {
        // x(i,j,t,:) = sum(sum(y(i*f-f+1:i*f, j*f-f+1:j*f, t, :)));
        double val = 0.0;
        uint sk = uint((i+1)*f-f);
        uint ek = uint((i+1)*f);

        uint sl = uint((j+1)*f-f);
        uint el = uint((j+1)*f);

        for(uint k = sk; k < ek; k++)
          for(uint l = sl; l < el; l++)
            {
              val += y.getVal(k,l);
              // if(i == 0 && j == 19)
              //   {
              //     LINFO("[%3d %3d]: %20.10f -> val: %20.10f", 
              //           k,l,y. getVal(k,l), val);
                  
              //   }              
            }


        x.setVal(i,j,val);

      }


  // LINFO("om   %20.10f ", x.getVal(0,19));

  return x;
}

// ######################################################################
// sumLxy is used for the long range filter in MT
//function out = sumLxy(L, I)
std::vector<Image<double> > 
sumLxy(rutz::shared_ptr<std::vector<Image<double> > > L, 
       std::vector<Image<double> > I)
{
  std::vector<Image<double> > out(NUM_DIRS);
  // can't split the diagonal filters without rotating the input which
  // introduces other issues so use the full 2D filter
  //  for d=1:8
  for(uint dir = 0; dir < NUM_DIRS; dir++)
    {
      //     out(:,:,1,d)  = filter2(L(:,:,d), I(:,:,1,d), 'same');
      out[dir] = filter2((*L)[dir],I[dir]);
    }
  return out;
}

// ######################################################################
// sumRz is used for the feedback from MSTd to MT
//function out = sumRz(R,w)
std::vector<Image<double> > 
sumRz(std::vector<double> R, 
      rutz::shared_ptr<std::vector<std::vector<Image<double> > > > w)
{
  //out(:,:,1,:) = zeros(size(w,1), size(w,2), 1, 8);
  std::vector<Image<double> > out(NUM_DIRS);
  uint width  = (*w)[0][0].getWidth();
  uint height = (*w)[0][0].getHeight();
  for(uint dir = 0; dir < NUM_DIRS; dir++)
    out[dir] = Image<double>(width, height, ZEROS);

  //  for z = 1:length(R)
  for(uint dir = 0; dir < NUM_DIRS; dir++)
    for(uint z = 0; z < R.size(); z++)
      // out(:,:,1,:) = out(:,:,1,:) + R(z)*w(:,:,z,:);
      out[dir] = out[dir] + (*w)[z][dir]*R[z]; 

  return out;
}

// ######################################################################
// sumvD is used for competititive interactions in MT
//function out = sumvD(v,I)
std::vector<Image<double> >
sumvD(std::vector<double>v, std::vector<Image<double> > I)
{
  std::vector<Image<double> > out(NUM_DIRS);

  //for d = 1:8
  for(uint dir = 0; dir < NUM_DIRS; dir++)
    {
      int d = int(dir)+1;
      
      // LINFO("boom[%d] + (%d + %d) + (%d + %d) + (%d + %d) + %d",
      //       dir, 
      //       dof(d+1)-1, dof(d-1)-1,
      //       dof(d+2)-1, dof(d-2)-1,
      //       dof(d+3)-1, dof(d-3)-1,
      //       dof(d+4)-1);

      // out(:,:,1,d) = v(1)*I(:,:,:,d) + 
      //                v(2)*sum(I(:,:,:,dof([d+1 d-1])),4) + 
      //                v(3)*sum(I(:,:,:,dof([d+2 d-2])),4) + 
      //                v(4)*sum(I(:,:,:,dof([d+3 d-3])),4) + 
      //                v(5)*I(:,:,:,dof([d+4]));
      out[dir] = 
        I[dir]*v[0] + 
        (I[dof(d+1)-1] + I[dof(d-1)-1])*v[1] + 
        (I[dof(d+2)-1] + I[dof(d-2)-1])*v[2] + 
        (I[dof(d+3)-1] + I[dof(d-3)-1])*v[3] + 
        (I[dof(d+4)-1])*v[4];

      //LINFO("baam[%d]",dir);

    }

  return out;
}

// ######################################################################
// function y = dof(x)
int dof(int x)
{
  // y = mod(x,8);
  int y = (x+8)%8;
  //LINFO("x: %d -> mod(x,8) = %d", x, y);

  // y = (y==0)*8+y;
  y = (y==0)*8 + y;
  //LINFO("(y==0)*8 + y; = %d", y);

  return y;
}    

// ######################################################################
void report(ViSTARSmodel &model)
{
  // xs = size(model.I.s3);
  //model.state->I[model.p_NumSpeeds-1][0]

    // the actual heading that the network selected
  // heading(:,t) = [2:3:xs(2) 2:3:xs(2)]*[model.R==max(model.R)];

  // ocos(:,:,t) = model.b.s2(:,:,1)-model.b.s2(:,:,2);

  // trans(:,:, t) = model.y.s2(:,:,1) - model.y.s2(:,:,2);

  // dir(:,:,t, :) = model.f.s2(:,:,1, :);

  // MT(:,:,t, :) = model.Q;

  // MSTd(:,t) = model.R;

  // model.heading = heading;
  // model.ocos = ocos;
  // model.trans = trans;
  // model.dir = dir;
  // model.MT = MT;
  // model.MSTd = MSTd;

  // show_res1(model);
  // show_res(model, model.heading);
}
           //  uint sj = 0, ej = 20, si = 0, ei = 20;
           // for(uint i = 0; i < 8; i++)
           //   {
           //     LINFO("Dir: %d", i);
           //     for(uint y = sj; y < ej; y++)
           //       {
           //         for(uint x = si; x < ei; x++)
           //           printf("%10.5f ", temp[i].getVal(x,y));
           //         printf("\n");
           //       }
           //   }
           // Raster::waitForKey();


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
