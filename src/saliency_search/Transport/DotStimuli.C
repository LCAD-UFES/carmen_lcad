/*!@file Transport/RandomInput.C A FrameIstream subclass for
  generating random images */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RandomInput.C $
// $Id: RandomInput.C 8602 2007-07-20 23:10:44Z rjpeters $
//

#ifndef TRANSPORT_DOTSTIMULI_C_DEFINED
#define TRANSPORT_DOTSTIMULI_C_DEFINED

#define DOT_NUM          500           // 200  1000 --> Shuo: 640x480: 2000 dots

#define FOE_X            2*WIDTH/4     // W/4 
#define FOE_Y            3*HEIGHT/4    // 2H/4 +4 

#define DOT_VEL          2.0/80.0      // Shuo: .035

// Shuo: in PsychToolbox: size refers to diameter, in our code radius
// Shuo: Abs max: 21, ave DS is 9.767 across all dots in each frames
#define DOT_ORG_DSIZE    .02    // .04
#define DOT_DSIZE        .07    // .035
#define MIN_DOT_SIZE     1   
#define MAX_DOT_SIZE     5   
#define ABS_MAX_DSIZE    50
#define NFRAME           60

#define HAVE_MOTION      1
#define HAVE_TEMP_SGRAD  1
#define HAVE_SPAT_SGRAD  1

#define NUM_PYR_LEVEL    3 // 3 for 640x480
#define NUM_DIRS         8
#define NUM_SPEEDS       3

#define SIDE_LENGTH      20  

#include "Transport/DotStimuli.H"

#include "Image/Image.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Raster/Raster.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Image/DrawOps.H"

// ######################################################################
DotStimuli::DotStimuli(OptionManager& mgr)
  :
  FrameIstream(mgr, "Random Input", "RandomInput"),
  itsDims(320,240), // if you change this default value, also update
                    // the documentation of OPT_InputFrameSource in
                    // Media/MediaOpts.C

  itsGenerator(0),
  itsIsFoe(false),
  itsFoe(Point2D<int>(-1,-1))
{
  itsStep = 0;
  itsAsymmetryMode = 0;
}

// ######################################################################
DotStimuli::~DotStimuli()
{}

// ######################################################################
void DotStimuli::setConfigInfo(const std::string& options)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  LINFO("options: %s", options.c_str());
  if (options.size() == 0) return;

  // parse the options

  // compounds are separated by commas ','  
  std::string rops = options;
  std::string::size_type fcpos = rops.find_first_of(',');

  // check for resizing first option 
  // (has to be in first position)
  // --in=dots:WxH,
  std::string op1   = rops.substr(0, fcpos);
  LDEBUG("op1: %s", op1.c_str());

  std::string::size_type fxpos = rops.find_first_of('x');
  if(fcpos != std::string::npos && 
     fxpos != std::string::npos && 
     fxpos <  fcpos)
    {
      Dims d; convertFromString(op1, d);
      this->setDims(d);

      std::string temp = rops.substr(fcpos+1); rops = temp;

      LDEBUG("new stimuli dimension: %dx%d", d.w(), d.h());
    }

  // subsequent commands 
  // NOTE: the shared config parameters will be set to
  //       what the last command specifies
  while(rops.size() != 0)
    {
      fcpos = rops.find_first_of(',');
      std::string op   = rops.substr(0, fcpos);
      if(fcpos != std::string::npos)
        {
          std::string temp = rops.substr(fcpos+1); rops = temp;
        }
      else rops = std::string("");
      LDEBUG("op:%s rest:%s", op.c_str(), rops.c_str());

      // for the next iteration
      fcpos = rops.find_first_of(',');
      
      // paramaters for this option is between "[ ]"
      std::string::size_type fbpos = op.find_first_of('[');
      std::string::size_type bbpos = op.find_first_of(']');
      std::string params; 
      if(fbpos != std::string::npos)
        {
          params = op.substr(fbpos+1,bbpos - fbpos - 1);
          LDEBUG("params: %s", params.c_str());

          std::string temp = op.substr(0,fbpos); op = temp;
        }
      
      // go through all the different motion
      if(!op.compare("Random"))     setConfigRandom(params);
      else 
        if(!op.compare("Single"))   setConfigSingle(params);
        else 
          if(!op.compare("Planar"))   setConfigPlanar(params);
          else 
            if(!op.compare("AsymmetryFast")) 
              //setConfigAsymmetry(params, 0);              
              setConfigAsymmetry(params, 2);
            else 
              if(!op.compare("AsymmetrySlow")) 
                //setConfigAsymmetry(params, 1);
                setConfigAsymmetry(params, 3);
              else 
                if(!op.compare("Foe"))      setConfigFoe(params);
                else 
                  if(!op.compare("Rotation")) setConfigRotation(params);
                  else LFATAL("unknown DotStimuli option: %s", op.c_str());
    }
}

// ######################################################################
void DotStimuli::setDims(const Dims& s)
{
  itsDims = s;
}

// ######################################################################
void DotStimuli::setConfigRandom(const std::string& params)
{
  itsIsRandom = true;
  LINFO("Random Motion");

  if(params.size() == 0) return;
}

// ######################################################################
void DotStimuli::setConfigSingle(const std::string& params)
{
  itsIsSingleDot = true;

  itsSingleDotDx = 2.0;
  itsSingleDotDy = 0.0;

  float orgDotSize = 2.0;
  itsSingleDotSize = orgDotSize;

  LINFO("Single Dot Motion");

  if(params.size() == 0) return;
}

// ######################################################################
void DotStimuli::setConfigPlanar(const std::string& params)
{              
  itsIsPlanar = true;
  itsPlanarDx = 2.0;
  itsPlanarDy = 0.0;
  LINFO("Planar Motion");

  if(params.size() == 0) return;
}

// ######################################################################
void DotStimuli::setConfigAsymmetry(const std::string& params, uint mode)
{              
  itsIsPlanar = true;
  itsIsSingleDot = true;
  itsPlanarDx = 2.0;
  itsPlanarDy = 0.0;

  float orgDotSize = 1.0;
  itsSingleDotSize = orgDotSize;

  itsAsymmetryMode = mode;
  if(mode == 0 || mode == 2)
    {
      itsSingleDotDx = 4.0;
      itsSingleDotDy = 0.0;
      LINFO("Asymmetry Motion Fast");

      //if(mode == 2) itsIsSingleDot = false;
    }
  else if(mode == 1 || mode == 3)
    {
      itsSingleDotDx = 1.0;
      itsSingleDotDy = 0.0;
      LINFO("Asymmetry Motion Slow");

      //if(mode == 3) itsIsSingleDot = false;
    }
  

  if(params.size() == 0) return;
}

// ######################################################################
void DotStimuli::setConfigFoe(const std::string& params)
{              
  itsIsFoe = true;

  itsFoe = Point2D<int>(160,120);
  LINFO("Focus of Expansion");

  if(params.size() == 0) 
    {
      itsFoeHaveMotion    = HAVE_MOTION;
      itsFoeHaveTempSGrad = HAVE_TEMP_SGRAD;
      itsFoeHaveSpatSGrad = HAVE_SPAT_SGRAD;      
      return;
    }

  // parse the parameters
  std::string rops = params;
  std::string::size_type fcmpos = rops.find_first_of('/');
  while(rops.size() != 0)
    {
      std::string op   = rops.substr(0, fcmpos);
      if(fcmpos != std::string::npos)
        {
          std::string temp = rops.substr(fcmpos+1); rops = temp;
        }
      else rops = std::string("");

      LINFO("op:%s rest:%s", op.c_str(), rops.c_str());
      
      // for the next iteration
      fcmpos = rops.find_first_of(',');
      
      // paramaters for this option is after colon :
      std::string::size_type fclpos = op.find_first_of(':');
      std::string params; 
      if(fclpos != std::string::npos)
        {
          params = op.substr(fclpos+1);
          LINFO("params: %s", params.c_str());

          std::string temp = op.substr(0,fclpos); op = temp;
        }
      
      // go through all the different motion
      if(!op.compare("Shuo"))
        {
          LINFO("Shuo Stimuli");
        }
      else if(!op.compare("flags"))
        {
          int flag = atoi(params.c_str());
          itsFoeHaveMotion    = (flag & 4);
          itsFoeHaveTempSGrad = (flag & 2);
          itsFoeHaveSpatSGrad = (flag & 1);

          LINFO("flag is: %d: (%d %d %d)", flag,  
                itsFoeHaveMotion, 
                itsFoeHaveTempSGrad, 
                itsFoeHaveSpatSGrad);
        }

      else LFATAL("unknown Foe param: %s", op.c_str());
    }  
}

// ######################################################################
void DotStimuli::setConfigRotation(const std::string& params)
{              
  itsIsRotation = true;
  LINFO("Rotation Motion");

  if(params.size() == 0) return;
}

// ######################################################################
GenericFrameSpec DotStimuli::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = itsDims;
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame DotStimuli::readFrame()
{
  Image<float> result(itsDims, ZEROS);

  // add dots
  if(itsIsRandom)
    {
      result += getRandomDotStimuli(itsStep);
    }
  if(itsIsSingleDot)
    {
      result += getSingleDotStimuli(itsStep);
    }
  if(itsIsPlanar)
    {
      result += getPlanarDotStimuli(itsStep); 
    }
  if(itsIsFoe)
    {
      result += getFoeStimuli(itsStep);
    }
  if(itsIsRotation)
    {
      result += getRotationDotStimuli(itsStep); 
    }

  itsStep++;
 
  inplaceNormalize(result, 0.0F, 255.0F);  
  Image<PixRGB<byte> > temp = makeRGB(result,result,result);
  return GenericFrame(temp);
}

// ######################################################################
Image<float> DotStimuli::getRandomDotStimuli(uint step)
{
  Image<byte> temp(itsDims, ZEROS);
  //uint width  = itsDims.w();
  //uint height = itsDims.h();

  return temp;
}

// ######################################################################
Image<float> DotStimuli::getSingleDotStimuli(uint step)
{
  //LINFO("SINGLE DOT\n\n\n");

  Image<float> temp(itsDims, ZEROS);
  uint width  = itsDims.w();
  uint height = itsDims.h();

  uint sideLength = SIDE_LENGTH;
  float nHorz = width/sideLength;
  float nVert = height/sideLength;

  if(step == 0)
    {
      if(itsAsymmetryMode > 1)
        {
          uint i = uint((nHorz*rand())/(RAND_MAX + 1.0));
          uint j = uint((nVert*rand())/(RAND_MAX + 1.0));
          itsSingleDotLocation = Point2D<float>
            (sideLength/2 + i * sideLength,
             sideLength/2 + j * sideLength ); 
        }
      else
        itsSingleDotLocation = Point2D<float>
          (width  * .1,// * double(rand())/(RAND_MAX + 1.0),
           height * double(rand())/(RAND_MAX + 1.0));        
    }

  // planar motion
  if(itsSingleDotDx != 0.0 || itsSingleDotDy != 0.0)
    itsSingleDotLocation = 
      Point2D<float>(fmod(itsSingleDotLocation.i + itsSingleDotDx, width), 
                     fmod(itsSingleDotLocation.j + itsSingleDotDy, height));

  drawDisk(temp,
           Point2D<int>(itsSingleDotLocation.i, 
                        itsSingleDotLocation.j),
           itsSingleDotSize, 1.0F);
  //LINFO("(%f %f)", itsSingleDotLocation.i, itsSingleDotLocation.j);

  return temp;
}

// ######################################################################
Image<float> DotStimuli::getPlanarDotStimuli(uint step)
{
  //LINFO("PLANAR DOT\n\n\n");

  if(itsAsymmetryMode > 1)
    return getGridPlanarDotStimuli(step);
  else
    return getRandomPlanarDotStimuli(step);
}

// ######################################################################
Image<float> DotStimuli::getGridPlanarDotStimuli(uint step)
{
  Image<float> temp(itsDims, ZEROS);
  uint width  = itsDims.w();
  uint height = itsDims.h();

  //if(step > NFRAME) return Image<byte>();

  uint sideLength = SIDE_LENGTH;
  float orgDotSize = 1.0;

  // create random dot initially
  if(step == 0)
    {
      uint nHorz = width/sideLength;
      uint nVert = height/sideLength;  
      itsPlanarDots.clear(); itsPlanarDotSizes.clear();

      for(uint i = 0; i < nHorz; i++)
        {
          for(uint j = 0; j < nVert; j++)
            {
              Point2D<float> currPt
                (sideLength/2 + i*sideLength,
                 sideLength/2 + j*sideLength );
              if(currPt.i != itsSingleDotLocation.i - itsSingleDotDx ||
                 currPt.j != itsSingleDotLocation.j - itsSingleDotDy   )
                {
                  itsPlanarDots.push_back(currPt);                    
                  itsPlanarDotSizes.push_back(orgDotSize); 
                }
            }
        }
    }
  uint nDots = itsPlanarDots.size();
  
  // planar motion
  if(itsPlanarDx != 0.0 || itsPlanarDy != 0.0)
    for(uint i = 0; i < nDots; i++)
      {
        itsPlanarDots[i] = 
          Point2D<float>(fmod(itsPlanarDots[i].i + itsPlanarDx, width), 
                         fmod(itsPlanarDots[i].j + itsPlanarDy, height));
      }

  // finally draw the dots
  for(uint i = 0; i < nDots; i++)
    {
      LDEBUG("[%d] loc: %10.3f %10.3f: size: %7.3f", 
             i, itsPlanarDots[i].i, itsPlanarDots[i].j, itsPlanarDotSizes[i]);
      drawDisk(temp,Point2D<int>(itsPlanarDots[i].i, itsPlanarDots[i].j),
               itsPlanarDotSizes[i], 1.0F);
      //temp.setVal(itsPlanarDots[i].i, itsPlanarDots[i].j, byte(128));
      //LINFO("pt: %8.3f %8.3f", itsPlanarDots[i].i, itsPlanarDots[i].j);
    }

  return temp;
}


// ######################################################################
Image<float> DotStimuli::getRandomPlanarDotStimuli(uint step)
{
  Image<float> temp(itsDims, ZEROS);
  uint width  = itsDims.w();
  uint height = itsDims.h();

  //if(step > NFRAME) return Image<byte>();

  uint dotNum = DOT_NUM;

  float orgDotSize = 1.0;

  // create random dot initially
  if(step == 0)
    {
      itsPlanarDots.resize(dotNum); itsPlanarDotSizes.resize(dotNum);
      for(uint i = 0; i < dotNum; i++)
        {
          itsPlanarDots[i] = Point2D<float>
            (width  * double(rand())/(RAND_MAX + 1.0),
             height * double(rand())/(RAND_MAX + 1.0));
          itsPlanarDotSizes[i] = orgDotSize; 
        }
    }
  
  // check for out-of-bounds dot needed to be shown
  for(uint i = 0; i < dotNum; i++)
    {
      // NOTE: can also kill dots randomly before going out of the image
      if(!temp.getBounds().contains(Point2D<int>(itsPlanarDots[i].i, 
                                                 itsPlanarDots[i].j))) 
        {
          float srx = 0.0;   float sry = 0.0;
          float rx  = width; float ry  = height;
          if(itsPlanarDx < 0.0)     { srx = 7.0*width/8.0; rx  = width/8; }
          else if(itsPlanarDx > 0.0){ srx = 0.0;           rx  = width/8; }
          
          if(itsPlanarDy < 0.0)     { sry = 7.0*height/8.0; ry  = height/8; }
          else if(itsPlanarDy > 0.0){ sry = 0.0;            ry  = height/8; }

          float sx = rx * double(rand())/(RAND_MAX + 1.0) + srx; 
          float sy = ry * double(rand())/(RAND_MAX + 1.0) + sry;
          
          itsPlanarDots[i] = Point2D<float>(sx,sy);
          itsPlanarDotSizes[i] = orgDotSize; 
          LDEBUG("new dots[%3d]: %7.3f %7.3f", i, sx, sy);
        }
      else
        {
          Point2D<int> pt(itsPlanarDots[i].i, itsPlanarDots[i].j);
          LDEBUG("[%3d] it's ok: (%7.3f %7.3f) -> %3d %3d", 
                 i, itsPlanarDots[i].i, itsPlanarDots[i].j, pt.i, pt.j);
        }
    }

  // planar motion
  if(itsPlanarDx != 0.0 || itsPlanarDy != 0.0)
    for(uint i = 0; i < dotNum; i++)
      {
        itsPlanarDots[i] = Point2D<float>(itsPlanarDots[i].i + itsPlanarDx, 
                                          itsPlanarDots[i].j + itsPlanarDy);
      }

  // finally draw the dots
  for(uint i = 0; i < dotNum; i++)
    {
      LDEBUG("[%d] loc: %10.3f %10.3f: size: %7.3f", 
             i, itsPlanarDots[i].i, itsPlanarDots[i].j, itsPlanarDotSizes[i]);
      drawDisk(temp,Point2D<int>(itsPlanarDots[i].i, itsPlanarDots[i].j),
               itsPlanarDotSizes[i], 1.0F);
      //temp.setVal(itsPlanarDots[i].i, itsPlanarDots[i].j, byte(128));
    }

  return temp;
}

// ######################################################################
Image<float> DotStimuli::getFoeStimuli(uint step)
{
  float dx = 0.0; float dy = 0.0;
  //float dx = pshift.i; float dy = pshift.j;
  
  float dotOrgDSize = DOT_ORG_DSIZE; 
  
  // planar (Yaw-Pitch) motion correction
  uint dir =  0.0; float speed = 0.0;
  
  LINFO("[%3d] haveMotion: %d haveTempSGrad: %d haveSpatSGrad: %d "
        "dx dy: %7.3f %7.3f dotOrgDSize: %f comp: %d %f",
        itsStep, 
        itsFoeHaveMotion, itsFoeHaveTempSGrad, itsFoeHaveSpatSGrad, 
        dx, dy, dotOrgDSize, dir, speed);

  //if(step > NFRAME) return Image<byte>();

  Image<byte> temp(itsDims, ZEROS);
  uint width  = itsDims.w();
  uint height = itsDims.h();

  float orgDotSize = 1.0;

  // create random dot initially
  if(step == 0)
    {
      itsFoeDots.resize(DOT_NUM); itsFoeDotSizes.resize(DOT_NUM);
      for(uint i = 0; i < DOT_NUM; i++)
        {
          itsFoeDots[i] = Point2D<float>
            (width  * double(rand())/(RAND_MAX + 1.0),
             height * double(rand())/(RAND_MAX + 1.0));

          // just do it in order to get identical average size
          //float range = MAX_DOT_SIZE - MIN_DOT_SIZE;
          //dotSizes[i] = i*range/(DOT_NUM-1.0)+ double(MIN_DOT_SIZE); 
          itsFoeDotSizes[i] = orgDotSize; 
        }
    }

  // check for out-of-bounds dot needed to be shown
  for(uint i = 0; i < DOT_NUM; i++)
    {
      // NOTE: can also kill dots randomly before going out of the image
      // NOTE: how about adding new dots in the FOE quadrants
      if(!temp.getBounds().
         contains(Point2D<int>(itsFoeDots[i].i, itsFoeDots[i].j))) 
        {
          itsFoeDots[i] = Point2D<float>
            (width  * double(rand())/(RAND_MAX + 1.0),
             height * double(rand())/(RAND_MAX + 1.0) );

          // keep the sizes or 1.0?
          itsFoeDotSizes[i] = orgDotSize; 
        }
    }

  // modify sizes according to rules
  for(uint i = 0; i < DOT_NUM; i++)
    {
      if(itsFoeHaveTempSGrad && itsFoeHaveSpatSGrad)
        {
          float dist = sqrt(pow((itsFoeDots[i].i - itsFoe.i), 2.0) + 
                            pow((itsFoeDots[i].j - itsFoe.j), 2.0)  ); 
          if(itsFoeHaveMotion)
            {
              itsFoeDotSizes[i] = dotOrgDSize * dist;
            }
          // growing, FOE coherent, but not moving 
          else
            {
              if(step == 0)
                {
                  itsFoeDotSizes[i] = dotOrgDSize * dist;
                }
              else
                {
                  // increase until ABS_MAX_DSIZE, then stop
                  if(itsFoeDotSizes[i] < ABS_MAX_DSIZE)
                    {
                      itsFoeDotSizes[i] = (1.0 + DOT_DSIZE) *
                        dotOrgDSize * dist;
                    }
                  // else dot size stays the same
                }
            }
        }
      else if(itsFoeHaveTempSGrad && !itsFoeHaveSpatSGrad)
        {
          // all dot have same size just increase 
          float ds = MAX_DOT_SIZE - MIN_DOT_SIZE;
          itsFoeDotSizes[i] = step*ds/(NFRAME - 1.0)+ double(MIN_DOT_SIZE); 
        }
      else if(!itsFoeHaveTempSGrad && itsFoeHaveSpatSGrad)
        {
          float dist = sqrt(pow((itsFoeDots[i].i - itsFoe.i), 2.0) + 
                            pow((itsFoeDots[i].j - itsFoe.j), 2.0)  ); 
          itsFoeDotSizes[i] = dotOrgDSize * dist;
        }
      // else just keep size 
    }

  // move the dots if needed
  if(itsFoeHaveMotion)
    for(uint i = 0; i < DOT_NUM; i++)
      {
        itsFoeDots[i] = Point2D<float>
          (itsFoeDots[i].i + DOT_VEL * (itsFoeDots[i].i - itsFoe.i),
           itsFoeDots[i].j + DOT_VEL * (itsFoeDots[i].j - itsFoe.j));
      }

  // add lateral motion
  if(dx != 0.0 || dy != 0.0)
    for(uint i = 0; i < DOT_NUM; i++)
      {
        itsFoeDots[i] = Point2D<float>(itsFoeDots[i].i + dx, 
                                       itsFoeDots[i].j + dy);
      }

  // finally draw the dots
  for(uint i = 0; i < DOT_NUM; i++)
    {
      //LINFO("loc: %10.3f %10.3f: size: %7.3f", 
      //      dots[i].i, dots[i].j, dotSizes[i]);
      drawDisk(temp,Point2D<int>(itsFoeDots[i].i, 
                                 itsFoeDots[i].j),itsFoeDotSizes[i],
               byte(255));
      //temp.setVal(itsFoeDots[i].i, itsFoeDots[i].j, byte(128));
    }

  // draw the FOE
  //drawDisk(temp,itsFoe,2,byte(128));    

  return temp;
}

// ######################################################################
Image<float> DotStimuli::getRotationDotStimuli(uint step)
{
  Image<byte> temp(itsDims, ZEROS);
  //uint width  = itsDims.w();
  //uint height = itsDims.h();

  LINFO("Rotation Motion: FIXXX: NOT YET IMPLEMENTED");

  return temp;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RANDOMINPUT_C_DEFINED
