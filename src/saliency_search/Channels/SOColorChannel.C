/*!@file Channels/SOColorChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SOColorChannel.C $
// $Id: SOColorChannel.C 8195 2007-03-30 04:34:07Z rjpeters $
//

#ifndef SOCOLORCHANNEL_C_DEFINED
#define SOCOLORCHANNEL_C_DEFINED

#include "Channels/SOColorChannel.H"

#include "Channels/BlueChannel.H"
#include "Channels/GreenChannel.H"
#include "Channels/RedChannel.H"
#include "Channels/SOBlueYellowChannel.H"
#include "Channels/SOGreenRedChannel.H"
#include "Channels/SORedGreenChannel.H"
#include "Channels/SOYellowBlueChannel.H"
#include "Channels/YellowChannel.H"
#include "Image/ColorOps.H"
#include "rutz/trace.h"

// ######################################################################
// Single Opponent ColorChannel member definitions:
// ######################################################################
SOColorChannel::SOColorChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "SOColor", "single-opponent-color", SOCOLOR),
  itsLumThresh("ColorChannelLuminanceThreshold", this, 25.5F),
  itsRG(makeSharedComp(new SORedGreenChannel(mgr))),
  itsGR(makeSharedComp(new SOGreenRedChannel(mgr))),
  itsBY(makeSharedComp(new SOBlueYellowChannel(mgr))),
  itsYB(makeSharedComp(new SOYellowBlueChannel(mgr))),
  itsR(makeSharedComp(new RedChannel(mgr))),
  itsG(makeSharedComp(new GreenChannel(mgr))),
  itsB(makeSharedComp(new BlueChannel(mgr))),
  itsY(makeSharedComp(new YellowChannel(mgr)))
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->addSubChan(itsRG);
  this->addSubChan(itsGR);
  this->addSubChan(itsBY);
  this->addSubChan(itsYB);

  // the following channels are computed to obtain the pyramids for
  // the above channels
  this->addSubChan(itsR);
  this->addSubChan(itsG);
  this->addSubChan(itsB);
  this->addSubChan(itsY);

  // these pure color channels are not considered for saliency
  this->setSubchanTotalWeight(*itsR, 0.0);
  this->setSubchanTotalWeight(*itsG, 0.0);
  this->setSubchanTotalWeight(*itsB, 0.0);
  this->setSubchanTotalWeight(*itsY, 0.0);
}

// ######################################################################
SORedGreenChannel&   SOColorChannel::rg() const { return *itsRG; }
SOGreenRedChannel&   SOColorChannel::gr() const { return *itsGR; }
SOBlueYellowChannel& SOColorChannel::by() const { return *itsBY; }
SOYellowBlueChannel& SOColorChannel::yb() const { return *itsYB; }
RedChannel&          SOColorChannel::r()  const { return *itsR; }
GreenChannel&        SOColorChannel::g()  const { return *itsG; }
BlueChannel&         SOColorChannel::b()  const { return *itsB; }
YellowChannel&       SOColorChannel::y()  const { return *itsY; }

// ######################################################################
SOColorChannel::~SOColorChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SOColorChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.colorFloat().initialized());

  Image<float> red, green, blue, yellow;
  getRGBY(inframe.colorFloat(), red, green, blue, yellow,
          itsLumThresh.getVal());
  // first find the pyramids for each color
  itsR->input(InputFrame::fromGrayFloat
              (&red, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsG->input(InputFrame::fromGrayFloat
              (&green, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsB->input(InputFrame::fromGrayFloat
              (&blue, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsY->input(InputFrame::fromGrayFloat
              (&yellow, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));

  const Dims d = inframe.getDims();

  // then compute the single opponent center-surround using the above
  // pyramids
  rg().singleOpponentInput(d, r().pyramid(0), g().pyramid(0),
                           inframe.time(), inframe.clipMask());
  gr().singleOpponentInput(d, g().pyramid(0), r().pyramid(0),
                           inframe.time(), inframe.clipMask());
  by().singleOpponentInput(d, b().pyramid(0), y().pyramid(0),
                           inframe.time(), inframe.clipMask());
  yb().singleOpponentInput(d, y().pyramid(0), b().pyramid(0),
                           inframe.time(), inframe.clipMask());
  LINFO("Single Opponent Color channel ok.");
}

// ######################################################################
void SOColorChannel::setRG(nub::ref<SORedGreenChannel> RG)
{ this->removeSubChan(itsRG); itsRG = RG; this->addSubChan(itsRG); }

void SOColorChannel::setGR(nub::ref<SOGreenRedChannel> GR)
{ this->removeSubChan(itsGR); itsGR = GR; this->addSubChan(itsGR); }

void SOColorChannel::setBY(nub::ref<SOBlueYellowChannel> BY)
{ this->removeSubChan(itsBY); itsBY = BY; this->addSubChan(itsBY); }

void SOColorChannel::setYB(nub::ref<SOYellowBlueChannel> YB)
{ this->removeSubChan(itsYB); itsYB = YB; this->addSubChan(itsYB); }

void SOColorChannel::setR(nub::ref<RedChannel> R)
{ this->removeSubChan(itsR); itsR = R; this->addSubChan(itsR); }

void SOColorChannel::setG(nub::ref<GreenChannel> G)
{ this->removeSubChan(itsG); itsG = G; this->addSubChan(itsG); }

void SOColorChannel::setB(nub::ref<BlueChannel> B)
{ this->removeSubChan(itsB); itsB = B; this->addSubChan(itsB); }

void SOColorChannel::setY(nub::ref<YellowChannel> Y)
{ this->removeSubChan(itsY); itsY = Y; this->addSubChan(itsY); }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SOCOLORCHANNEL_C_DEFINED
