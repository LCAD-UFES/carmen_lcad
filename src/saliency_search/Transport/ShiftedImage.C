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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/ShiftedImage.C $
// $Id: ShiftedImage.C 8602 2007-07-20 23:10:44Z rjpeters $
//

#ifndef TRANSPORT_RANDOMINPUT_C_DEFINED
#define TRANSPORT_RANDOMINPUT_C_DEFINED

#include "Transport/ShiftedImage.H"

#include "Image/Image.H"
#include "Raster/Raster.H"
#include "Image/Pixels.H"


#include "Image/ShapeOps.H"

#include "Raster/GenericFrame.H"

// ######################################################################
ShiftedImage::ShiftedImage(OptionManager& mgr)
  :
  FrameIstream(mgr, "Random Input", "ShiftedImage")
{
  itsStep = 0;

  itsIsPlanar   = false;
  itsIsFoe      = false;
  itsIsRotation = false;
}

// ######################################################################
ShiftedImage::~ShiftedImage()
{}

// ######################################################################
void ShiftedImage::setConfigInfo(const std::string& options)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  LINFO("options: %s", options.c_str());
  if (options.size() == 0) return;

  // parse the options

  // get the image location first
  std::string rops = options;
  std::string::size_type fcpos = rops.find_first_of(':');
  std::string img  = rops.substr(0, fcpos);  
  itsImage = Raster::ReadRGB(img);
  LINFO("Image: %s: (%d %d)",img.c_str(), 
        itsImage.getWidth(), itsImage.getHeight());

  // get the shifting command
  std::string command = rops.substr(fcpos+1);
  std::string::size_type fbpos = rops.find_first_of('[');
  std::string::size_type bbpos = rops.find_first_of(']');
  std::string params;
  if(fbpos != std::string::npos)
    {
      std::string temp = rops.substr(fcpos+1, fbpos - fcpos - 1); 
      command = temp;
      params  = rops.substr(fbpos+1,bbpos - fbpos - 1);      
    }
  LINFO("Command: %s Params: %s", command.c_str(), params.c_str());

  // shifting parameters
  if(!command.compare("planar"))       setConfigPlanar(params);
  else 
    if(!command.compare("foe"))   setConfigFoe(params);
    else 
      if(!command.compare("rotation")) setConfigRotation(params);
      else LFATAL("unknown ShiftedImage option: %s", command.c_str());
}

// ######################################################################
void ShiftedImage::setConfigPlanar(const std::string& params)
{              
  itsIsPlanar = true;

  itsTotalSteps = 30; 

  itsDx = 2.0; 
  itsDy = 0.0;
  LINFO("Planar Motion");
}

// ######################################################################
void ShiftedImage::setConfigFoe(const std::string& params)
{              
  itsIsFoe = true;
  itsFoe = Point2D<int>(160,120);

  itsTotalSteps = 30;

  itsDx = 0.0; 
  itsDy = 0.0;
  LINFO("Focus of Expansion");
}

// ######################################################################
void ShiftedImage::setConfigRotation(const std::string& params)
{              
  itsIsRotation = true;
  itsRotationCenter = Point2D<int>(160,120);

  LINFO("Rotation Motion");
}

// ######################################################################
GenericFrameSpec ShiftedImage::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = itsImage.getDims();
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame ShiftedImage::readFrame()
{
  Image<PixRGB<byte> > result;

  // add dots
  if(itsIsPlanar)        result = getPlanarMotionStimuli(itsStep); 
  else if(itsIsFoe)      result = getFoeStimuli(itsStep);
  else if(itsIsRotation) result = getRotationMotionStimuli(itsStep); 

  itsStep++;
 
  return GenericFrame(result);
}

// ######################################################################
Image<PixRGB<byte> > ShiftedImage::getPlanarMotionStimuli(uint step)
{
  // just loop it 
  step = step % itsTotalSteps;
  
  uint width  = itsImage.getWidth();
  uint height = itsImage.getHeight(); 
  float scale =  1.25;
  Image<PixRGB<byte> > temp = rescale(itsImage, scale*width, scale*height);
  float nwidth  = temp.getWidth();
  float nheight = temp.getHeight(); 

  float sleft = 0.0; if(itsDx < 0.0) sleft = nwidth  - 1 - width;
  float stop  = 0.0; if(itsDy < 0.0) stop  = nheight - 1 - height; 
  
  float left  = sleft + itsDx*step;
  float top   = stop  + itsDy*step; 

  //if(top  < 0.0) top  = 0.0;
  //if(left < 0.0) left = 0.0;

  Rectangle r = 
    Rectangle::tlbrI(top, left, top+height-1, left+width-1);

//   LINFO("[%3d/%3d] FOE(%7.3f %7.3f) %f p(%7.3f %7.3f) [[%7.3f %7.3f]] " 
//          "[%3d %3d %3d %3d] temp(%3d %3d) ((%3d %3d))", 
//         step, totalStep, 
//         foeX,foeY, scale, px, py, top, left,
//         r.top(), r.left(), r.bottomI(), r.rightI(), 
//         temp.getWidth(), temp.getHeight(),
//         r.width(),r.height());
  Image<PixRGB<byte> > result = crop(temp, r);
  return result;
}

// ######################################################################
 Image<PixRGB<byte> > ShiftedImage::getFoeStimuli(uint step)
{
  uint mag = 2;
  
  // just loop it 
  step = step % itsTotalSteps;

  // set original image on first step
  if(step == 0) return itsImage; 

  uint width  = itsImage.getWidth();
  uint height = itsImage.getHeight();

  float nsize = (step/(itsTotalSteps - 1.0));
  float scale =  1.0 / (1.0 - nsize*1.0/mag);
  Image<PixRGB<byte> > temp = rescale(itsImage, scale*width, scale*height);
  float nwidth  = temp.getWidth();
  float nheight = temp.getHeight(); 

  float px = float(itsFoe.i)/float(width);
  float py = float(itsFoe.j)/float(height);

  float foeX  = px*nwidth; 
  float foeY  = py*nheight; 

  LINFO("[%d] %d %d -> %f %f", step, itsFoe.i, itsFoe.j, itsDx, itsDy);

  float left  = foeX - float(itsFoe.i) + itsDx*step;
  float top   = foeY - float(itsFoe.j) + itsDy*step; 


  //if(top  < 0.0) top  = 0.0;
  //if(left < 0.0) left = 0.0;

  Rectangle r = 
    Rectangle::tlbrI(top, left, top+height-1, left+width-1);

  LINFO("[%3d/%3d] FOE(%7.3f %7.3f) %f p(%7.3f %7.3f) [[%7.3f %7.3f]] " 
         "[%3d %3d %3d %3d] temp(%3d %3d) ((%3d %3d))", 
        step, itsTotalSteps, 
        foeX,foeY, scale, px, py, top, left,
        r.top(), r.left(), r.bottomI(), r.rightI(), 
        temp.getWidth(), temp.getHeight(),
        r.width(),r.height());
  Image<PixRGB<byte> > result = crop(temp, r);

  return result;
}

// ######################################################################
Image<PixRGB<byte> > ShiftedImage::getRotationMotionStimuli(uint step)
{
  uint width  = itsImage.getWidth();
  uint height = itsImage.getHeight();
  Image<PixRGB<byte> > temp(width, height, ZEROS);

  LINFO("Rotation Motion: FIXXX: NOT YET IMPLEMENTED");

  return temp;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RANDOMINPUT_C_DEFINED
