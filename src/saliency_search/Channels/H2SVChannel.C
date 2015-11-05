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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/H2SVChannel.C $
// $Id: H2SVChannel.C 12962 2010-03-06 02:13:53Z irock $
//

#ifndef H2SVCHANNEL_C_DEFINED
#define H2SVCHANNEL_C_DEFINED

#include "Channels/H2SVChannel.H"

#include "Channels/Hue1Channel.H"
#include "Channels/Hue2Channel.H"
#include "Channels/SaturationChannel.H"
#include "Channels/ValueIntensityChannel.H"
#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "rutz/trace.h"

// ######################################################################
// Double Opponent ColorChannel member definitions:
// ######################################################################

H2SVChannel::H2SVChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "H2SV", "H2SV", H2SV),
  itsUseH2SV1(&OPT_UseH2SV1, this),
  itsH1(new Hue1Channel(mgr)),
  itsH2(new Hue2Channel(mgr)),
  itsS(new SaturationChannel(mgr)),
  itsV(new ValueIntensityChannel(mgr)),
  itsUseSigmoid(false),
  itsSigAlpha(40.0F),
  itsSigBeta(4.0F)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->addSubChan(itsH1);
  this->addSubChan(itsH2);
  this->addSubChan(itsS);
  this->addSubChan(itsV);
}

// ######################################################################
Hue1Channel& H2SVChannel::H1() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsH1;
}

// ######################################################################
Hue2Channel& H2SVChannel::H2() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsH2;
}

// ######################################################################
SaturationChannel& H2SVChannel::S() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsS;
}

// ######################################################################
ValueIntensityChannel& H2SVChannel::V() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsV;
}

// ######################################################################
H2SVChannel::~H2SVChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void H2SVChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.colorFloat().initialized());

  if(!itsH1img.initialized())
  {
    itsH1img.resize(inframe.getDims());
    itsH2img.resize(inframe.getDims());
    itsSimg.resize(inframe.getDims());
    itsVimg.resize(inframe.getDims());
  }

  Image<float>::iterator h1imgItr = itsH1img.beginw();
  Image<float>::iterator h2imgItr = itsH2img.beginw();
  Image<float>::iterator simgItr  = itsSimg.beginw();
  Image<float>::iterator vimgItr  = itsVimg.beginw();

  Image<PixRGB<float> >::const_iterator colItr = inframe.colorFloat().begin();

  // H2SV2 is more R/G B/Y opponent in nature while
  // H2SV1 is more simple and symetric

  // To get H2SV we take a basic RGB image and convert each PixRGB pixel
  // in that image to a PixH2SV2 or PixH2SV1 pixel. We then get the
  // H1, H2, S and V parts and split those into four gray scale images

  if(itsUseH2SV1.getVal())
  {
    while(colItr != inframe.colorFloat().end())
    {
      const PixH2SV1<float> pix = PixH2SV1<float>(*colItr++);
      if(itsUseSigmoid)
      {
        const float S = logsig2(pix.p[2],itsSigAlpha,itsSigBeta);
        *h1imgItr++   = ((pix.p[0] - 0.5F)*S + 0.5F) * 85.0F; // 1/3
        *h2imgItr++   = ((pix.p[1] - 0.5F)*S + 0.5F) * 85.0F; // 1/3
      }
      else
      {
        *h1imgItr++ = pix.p[0] * 85.0F; // 1/3
        *h2imgItr++ = pix.p[1] * 85.0F; // 1/3
      }

      *simgItr++ = pix.p[2] * 1.0F;  // leave out
      *vimgItr++ = pix.p[3] * 85.0F; // 1/3
    }
  }
  else
  {
    while(colItr != inframe.colorFloat().end())
    {
      const PixH2SV2<float> pix = PixH2SV2<float>(*colItr++);

      if(itsUseSigmoid)
      {
        const float S = logsig2(pix.p[2],itsSigAlpha,itsSigBeta);
        //*h1imgItr++ = pix.p[0] * 85.0F; // 1/3
        *h1imgItr++   = ((pix.p[0] - 0.5F)*S + 0.5F) * 85.0F; // 1/3
        //*h2imgItr++   = pix.p[1] * 85.0F; // 1/3
        *h2imgItr++   = ((pix.p[1] - 0.5F)*S + 0.5F) * 85.0F; // 1/3
      }
      else
      {
        *h1imgItr++ = pix.p[0] * 85.0F; // 1/3
        *h2imgItr++ = pix.p[1] * 85.0F; // 1/3
      }

      *simgItr++ = pix.p[2] * 1.0F; // leave out
      *vimgItr++ = pix.p[3] * 85.0F; // 1/3
    }
  }

  itsH1->input(InputFrame::fromGrayFloat(&itsH1img, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsH2->input(InputFrame::fromGrayFloat(&itsH2img, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsS->input(InputFrame::fromGrayFloat(&itsSimg,   inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsV->input(InputFrame::fromGrayFloat(&itsVimg,   inframe.time(), &inframe.clipMask(), inframe.pyrCache()));

  LINFO("H2SV Color channel ok.");
}

// ######################################################################
void H2SVChannel::setH1(nub::ref<Hue1Channel> h1)
{
  this->removeSubChan(itsH1);
  itsH1 = h1;
  this->addSubChan(itsH1);
}

// ######################################################################
void H2SVChannel::setH2(nub::ref<Hue2Channel> h2)
{
  this->removeSubChan(itsH2);
  itsH2 = h2;
  this->addSubChan(itsH2);
}

// ######################################################################
void H2SVChannel::setS(nub::ref<SaturationChannel> s)
{
  this->removeSubChan(itsS);
  itsS = s;
  this->addSubChan(itsS);
}

// ######################################################################
void H2SVChannel::setV(nub::ref<ValueIntensityChannel> v)
{
  this->removeSubChan(itsV);
  itsV = v;
  this->addSubChan(itsV);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COLORCHANNEL_C_DEFINED
