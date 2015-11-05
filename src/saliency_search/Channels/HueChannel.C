/*!@file Channels/HueChannel.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/HueChannel.C $
// $Id: HueChannel.C 8195 2007-03-30 04:34:07Z rjpeters $
//

#ifndef HUECHANNEL_C_DEFINED
#define HUECHANNEL_C_DEFINED

#include "Channels/HueChannel.H"

#include "Image/ColorOps.H"
#include "Image/Pixels.H"

// ######################################################################
HueChannel::HueChannel(OptionManager& mgr,
                       float muR, float muG,
                       float sigR, float sigG, float rho) :
  SingleChannel(mgr, "hue", "hue", HUE,
                rutz::make_shared(new GaussianPyrBuilder<float>(5))),
  itsMuR(muR),
  itsMuG(muG),
  itsSigR(sigR),
  itsSigG(sigG),
  itsRho(rho)
{}

HueChannel::HueChannel(OptionManager& mgr,
                       const PixRGB<byte> hue,
                       float sig) :
  SingleChannel(mgr, "hue", "hue", HUE,
                rutz::make_shared(new GaussianPyrBuilder<float>(5))),
  itsSigR(sig),
  itsSigG(sig),
  itsRho(0.0f)
{
  float intens,cr,cg;
  RGBtoCIE(hue,cr,cg,intens);
  itsMuR = cr;
  itsMuG = cg;
}

HueChannel::~HueChannel() {}


// ######################################################################
void HueChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.colorFloat().initialized());

  Image<float> tmp = hueDistance(inframe.colorFloat(),itsMuR,itsMuG,
                                 itsSigR,itsSigG,itsRho);
  SingleChannel::doInput(InputFrame::fromGrayFloat(&tmp,inframe.time(),
                                                   &inframe.clipMask(),
                                                   inframe.pyrCache()));
}

// ######################################################################
Image<float> HueChannel::combineSubMaps()
{
  Image<float> output = getImage(itsLevelSpec.getVal().mapLevel());

  LDEBUG("%s: Normalizing output: %s(%f .. %f)", tagName().c_str(),
         maxNormTypeName(itsNormType.getVal()), itsOutputRangeMin.getVal(),
         itsOutputRangeMax.getVal());

  output = maxNormalize(output, itsOutputRangeMin.getVal(),
                        itsOutputRangeMax.getVal(), itsNormType.getVal());

  return output;
}

// ######################################################################
uint HueChannel::numSubmaps() const
{ return 1; }

// ######################################################################
Image<float> HueChannel::getSubmap(const uint idx) const
{
  // FIXME the fact that we need a const_cast here indicates some kind
  // of design problem...
  return const_cast<HueChannel*>(this)->getOutput();
}

// ######################################################################
std::string HueChannel::getSubmapName(const uint idx) const
{
  return std::string("Hue");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // HUECHANNEL_C_DEFINED
