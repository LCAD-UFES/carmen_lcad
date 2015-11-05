/*!@file Channels/H2SVChannel.C */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/CIELabChannel.C $
// $Id: CIELabChannel.C 10794 2009-02-08 06:21:09Z itti $
//

#ifndef CIELABCHANNEL_C_DEFINED
#define CIELABCHANNEL_C_DEFINED

#include "Channels/CIELabChannel.H"

#include "Channels/Hue1Channel.H"
#include "Channels/Hue2Channel.H"
#include "Channels/ValueIntensityChannel.H"
#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "rutz/trace.h"

// ######################################################################
// Double Opponent ColorChannel member definitions:
// ######################################################################

CIELabChannel::CIELabChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "CIELAB", "CIELab", CIELAB),
  itsL(new ValueIntensityChannel(mgr)),
  itsA(new Hue1Channel(mgr)),
  itsB(new Hue2Channel(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->addSubChan(itsL);
  this->addSubChan(itsA);
  this->addSubChan(itsB);
}

// ######################################################################
ValueIntensityChannel& CIELabChannel::L() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsL;
}

// ######################################################################
Hue1Channel& CIELabChannel::A() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsA;
}

// ######################################################################
Hue2Channel& CIELabChannel::B() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsB;
}

// ######################################################################
CIELabChannel::~CIELabChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void CIELabChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.colorFloat().initialized());

  if(!itsLimg.initialized())
  {
    itsLimg.resize(inframe.getDims());
    itsAimg.resize(inframe.getDims());
    itsBimg.resize(inframe.getDims());
  }

  Image<float>::iterator limgItr = itsLimg.beginw();
  Image<float>::iterator aimgItr = itsAimg.beginw();
  Image<float>::iterator bimgItr = itsBimg.beginw();

  Image<PixRGB<float> >::const_iterator colItr = inframe.colorFloat().begin();

  // CIELab2 is more R/G B/Y opponent in nature while
  // CIELab1 is more simple and symetric

  // To get CIELab we take a basic RGB image and convert each PixRGB pixel
  // in that image to a PixCIELab pixel. We then get the
  // L, a and b parts and split those into four gray scale images

  //Image<PixLab<float> > convert;
  //convert.resize(inframe.getDims());
  while(colItr != inframe.colorFloat().end())
  {
    const PixLab<float> pix = PixLab<float>(*colItr++);
    *limgItr++ = pix.p[0]/3.0F; // 1/3 weight
    *aimgItr++ = pix.p[1]/3.0F; // 1/3 weight
    *bimgItr++ = pix.p[2]/3.0F; // 1/3 weight
  }

  itsL->input(InputFrame::fromGrayFloat(&itsLimg,   inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsA->input(InputFrame::fromGrayFloat(&itsAimg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsB->input(InputFrame::fromGrayFloat(&itsBimg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));

  LINFO("CIELab Color channel ok.");
}

// ######################################################################
void CIELabChannel::setL(nub::ref<ValueIntensityChannel> L)
{
  this->removeSubChan(itsL);
  itsL = L;
  this->addSubChan(itsL);
}

// ######################################################################
void CIELabChannel::setA(nub::ref<Hue1Channel> A)
{
  this->removeSubChan(itsA);
  itsA = A;
  this->addSubChan(itsA);
}

// ######################################################################
void CIELabChannel::setB(nub::ref<Hue2Channel> B)
{
  this->removeSubChan(itsB);
  itsB = B;
  this->addSubChan(itsB);
}





// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COLORCHANNEL_C_DEFINED
