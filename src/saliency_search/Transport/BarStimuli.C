/*!@file Transport/BarStimuli.C A FrameIstream subclass for
  generating bar stimuli */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/BarStimuli.C $
// $Id: $
//

#ifndef TRANSPORT_BARSTIMULI_C_DEFINED
#define TRANSPORT_BARSTIMULI_C_DEFINED

#include "Transport/BarStimuli.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/MathOps.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Raster/GenericFrame.H"

// ######################################################################
BarStimuli::BarStimuli(OptionManager& mgr)
  :
  FrameIstream(mgr, "Random Input", "BarStimuli"),
  itsDims(320,240), // if you change this default value, also update
                    // the documentation of OPT_InputFrameSource in
                    // Media/MediaOpts.C
  itsGenerator(0)
{
  itsStep = 0;
  itsHaveMotion = true;

  itsIsNormal          = false;
  itsIsApertureProblem = false;
}

// ######################################################################
BarStimuli::~BarStimuli()
{}

// ######################################################################
void BarStimuli::setConfigInfo(const std::string& options)
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
  std::string::size_type fxpos = rops.find_first_of('x');
  if(fcpos != std::string::npos && fxpos != std::string::npos
     && fxpos < fcpos)
    {
      Dims d; convertFromString(op1, d);
      this->setDims(d);

      std::string temp = rops.substr(fcpos+1); rops = temp;
    }
  LINFO("Using default dimensions");

  // subsequent commands 
  // NOTE: for now the config will be the last 
  while(rops.size() != 0)
    {
      std::string op   = rops.substr(0, fcpos);
      if(fcpos != std::string::npos)
        {
          std::string temp = rops.substr(fcpos+1); rops = temp;
        }
      else rops = std::string("");

      LINFO("op:%s rest:%s", op.c_str(), rops.c_str());
      fcpos = rops.find_first_of(',');
      
      // paramaters for this option is between "[ ]"
      std::string::size_type fbpos = op.find_first_of('[');
      std::string::size_type bbpos = op.find_first_of(']');
      std::string params;
      if(fbpos != std::string::npos)
        {
          params = op.substr(fbpos+1,bbpos - fbpos - 1);
          LINFO("params: %s", params.c_str());

          std::string temp = op.substr(0,fbpos); op = temp;

        }

      // configure for that option
      if(!op.compare("Normal"))     setConfigNormal(params);
      else 
        if(!op.compare("ApertureProblem"))   setConfigApertureProblem(params);
        else LFATAL("unknown BarStimuli option: %s", op.c_str());
    }
}

// ######################################################################
void BarStimuli::setDims(const Dims& s)
{
  itsDims = s;
}

// ######################################################################
void BarStimuli::setConfigNormal(const std::string& params)
{
  itsIsNormal = true;

  LINFO("Normal (perpendicular) Bar Motion");
}

// ######################################################################
void BarStimuli::setConfigApertureProblem(const std::string& params)
{
  itsIsApertureProblem = true;

  LINFO("Aperture Problem");
}

// ######################################################################
GenericFrameSpec BarStimuli::peekFrameSpec()
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
GenericFrame BarStimuli::readFrame()
{
  Image<float> result;

  // add dots
  if(itsIsNormal)
    {
      result = getNormalBarStimuli(itsStep);
    }
  else if(itsIsApertureProblem)
    {
      result = getApertureProblemStimuli(itsStep);
    }
  itsStep++;
 
  inplaceNormalize(result, 0.0F, 255.0F);  
  Image<PixRGB<byte> > temp = makeRGB(result,result,result);
  return GenericFrame(temp);

}

// ######################################################################
Image<float> BarStimuli::getNormalBarStimuli(uint step)
{
  Image<float> temp(itsDims, ZEROS);
  uint width  = itsDims.w();
  uint height = itsDims.h();

  // initially create dots
  if(step == 0)
    {
      itsBars.resize(4); itsBarLengths.resize(4);

      // top left
      itsBars[0] = Point2D<float> (width/2, height/4);      
      itsBarLengths[0] = 1.0;  
      itsBars[1] = Point2D<float> (width/4, height/2);      
      itsBarLengths[1] = 1.0; 
      itsBars[2] = Point2D<float> (width/8, height/4);      
      itsBarLengths[2] = 1.0;      
      itsBars[3] = Point2D<float> (width/4, height/8);      
      itsBarLengths[3] = 1.0;

      // bottom right
//       itsBars[0] = Point2D<float> (7*width/8, 3*height/4);      
//       itsBarLengths[0] = 1.0;  
//       itsBars[1] = Point2D<float> (3*width/4, 7*height/8);      
//       itsBarLengths[1] = 1.0; 
//       itsBars[2] = Point2D<float> (width/2,   3*height/4);      
//       itsBarLengths[2] = 1.0;      
//       itsBars[3] = Point2D<float> (3*width/4,   height/2);      
//       itsBarLengths[3] = 1.0; 

    }

  // move the dots if needed
  if(itsHaveMotion)
    {
      //for(uint i = 0; i < dotNum; i++)
        itsBars[0] = Point2D<float>(itsBars[0].i + 2.0, itsBars[0].j);
        itsBars[1] = Point2D<float>(itsBars[1].i,       itsBars[1].j + 1.0);
        itsBars[2] = Point2D<float>(itsBars[2].i - 1.0, itsBars[2].j);
        itsBars[3] = Point2D<float>(itsBars[3].i,       itsBars[3].j - 1.0);

        //LINFO("loc: %f %f: size: %f", itsBars[i].i, itsBars[i].j, itsBarLengths[i]);      
    }
  // finally draw the itsBars
  //for(uint i = 0; i < dotNum; i++)
    //drawDisk(temp,Point2D<int>(itsBars[i].i, itsBars[i].j),itsBarLengths[i],byte(255));

    //drawDisk(temp,Point2D<int>(itsBars[0].i, itsBars[0].j), 1.0,byte(255));
    //drawDisk(temp,Point2D<int>(itsBars[1].i, itsBars[1].j), 1.0,byte(255));
    //drawDisk(temp,Point2D<int>(itsBars[2].i, itsBars[2].j), 1.0,byte(255));
    //drawDisk(temp,Point2D<int>(itsBars[3].i, itsBars[3].j), 1.0,byte(255));

  drawLine
    (temp, 
     Point2D<int>(itsBars[0].i, itsBars[0].j-15),
     Point2D<int>(itsBars[0].i, itsBars[0].j+15), 1.0F, 1);  

  // control the thickness of the line
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[0].i, itsBars[0].j-15),
//        Point2D<int>(itsBars[0].i, itsBars[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[0].i+1, itsBars[0].j-15),
//        Point2D<int>(itsBars[0].i+1, itsBars[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[0].i-1, itsBars[0].j-15),
//        Point2D<int>(itsBars[0].i-1, itsBars[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[0].i-2, itsBars[0].j-15),
//        Point2D<int>(itsBars[0].i-2, itsBars[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[0].i-3, itsBars[0].j-15),
//        Point2D<int>(itsBars[0].i-3, itsBars[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[0].i+2, itsBars[0].j-15),
//        Point2D<int>(itsBars[0].i+2, itsBars[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[0].i-4, itsBars[0].j-15),
//        Point2D<int>(itsBars[0].i-4, itsBars[0].j+15), byte(255), 1);
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[0].i+3, itsBars[0].j-15),
//        Point2D<int>(itsBars[0].i+3, itsBars[0].j+15), byte(255), 1);

    //===============================================
//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[1].i-15, itsBars[1].j),
//        Point2D<int>(itsBars[1].i+15, itsBars[1].j), byte(255), 1);

//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[2].i, itsBars[2].j-15),
//        Point2D<int>(itsBars[2].i, itsBars[2].j+15), byte(255), 1);

//     drawLine
//       (temp, 
//        Point2D<int>(itsBars[3].i-15, itsBars[3].j),
//        Point2D<int>(itsBars[3].i+15, itsBars[3].j), byte(255), 1);


             //drawDisk(temp,Point2D<int>(itsBars[i].i, itsBars[i].j),itsBarLengths[i],byte(255));
  //temp.setVal(itsBars[i].i, itsBars[i].j, byte(128));

  return temp;
}

// ######################################################################
Image<float> BarStimuli::getApertureProblemStimuli(uint step)
{
  Image<float> temp(itsDims, ZEROS);
  uint width  = itsDims.w();
  uint height = itsDims.h();

  // create dot initially
  if(step == 0)
    {
      itsBars.resize(1); 
      // top left
      itsBars[0] = Point2D<float> (width/2, height/4);      
    }

  uint len = 15;
  itsBars[0] = Point2D<float>(itsBars[0].i + 2.0, itsBars[0].j);
  drawLine
    (temp, 
     Point2D<int>(itsBars[0].i-len, itsBars[0].j-len),
     Point2D<int>(itsBars[0].i+len, itsBars[0].j+len), 1.0F, 1);

  return temp;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_BARSTIMULI_C_DEFINED
