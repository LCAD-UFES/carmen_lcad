/*!@file Channels/ImagizeColorChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ImagizeColorChannel.C $
// $Id: ImagizeColorChannel.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Channels/ImagizeColorChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ImagizeAlphaChannel.H"
#include "Channels/ImagizeBetaChannel.H"
#include "Channels/ImagizeLedChannel.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "rutz/trace.h"

const ModelOptionDef OPT_ImagizeColorChannelAweight =
  { MODOPT_ARG(float), "ImagizeColorChannelAweight", &MOC_CHANNEL, OPTEXP_CORE,
    "Weight to assign to the Alpha component of the Imagize color channel",
    "imgz-a-weight", '\0', "<float>", "1.0" };

const ModelOptionDef OPT_ImagizeColorChannelBweight =
  { MODOPT_ARG(float), "ImagizeColorChannelBweight", &MOC_CHANNEL, OPTEXP_CORE,
    "Weight to assign to the Beta component of the Imagize color channel",
    "imgz-b-weight", '\0', "<float>", "1.0" };

const ModelOptionDef OPT_ImagizeColorChannelLweight =
  { MODOPT_ARG(float), "ImagizeColorChannelLweight", &MOC_CHANNEL, OPTEXP_CORE,
    "Weight to assign to the Led component of the Imagize color channel",
    "imgz-l-weight", '\0', "<float>", "1.0" };

// ######################################################################
// ImagizeColorChannel member definitions:
// ######################################################################

ImagizeColorChannel::ImagizeColorChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "ImagizeComposite", "ImagizeComposite", IMGZCOLOR),
  itsAweight(&OPT_ImagizeColorChannelAweight, this),
  itsBweight(&OPT_ImagizeColorChannelBweight, this),
  itsLweight(&OPT_ImagizeColorChannelLweight, this),
  itsA(new ImagizeAlphaChannel(getManager(), true)),
  itsB(new ImagizeBetaChannel(getManager(), true)),
  itsL(new ImagizeLedChannel(getManager(), true))
{
GVX_TRACE(__PRETTY_FUNCTION__);

  this->addSubChan(itsA);
  this->addSubChan(itsB);
  this->addSubChan(itsL);
}

// ######################################################################
ImagizeAlphaChannel& ImagizeColorChannel::alpha() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsA;
}

// ######################################################################
ImagizeBetaChannel& ImagizeColorChannel::beta() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsB;
}

// ######################################################################
ImagizeLedChannel& ImagizeColorChannel::led() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsL;
}

// ######################################################################
ImagizeColorChannel::~ImagizeColorChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void ImagizeColorChannel::start1()
{
  LINFO("Using a weight of %f for Alpha subchannel", itsAweight.getVal());
  setSubchanTotalWeight(*itsA, itsAweight.getVal());

  LINFO("Using a weight of %f for Beta subchannel", itsBweight.getVal());
  setSubchanTotalWeight(*itsB, itsBweight.getVal());

  LINFO("Using a weight of %f for LED subchannel", itsLweight.getVal());
  setSubchanTotalWeight(*itsL, itsLweight.getVal());
}

// ######################################################################
void ImagizeColorChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // get the A, B, L components (which just come in from the camera as
  // R, G, B, respectively):
  LINFO("Converting input image to Alpha-Beta-LED colors.");
  Image<byte> aimg, bimg, limg;
  getComponents(inframe.colorByte(), aimg, bimg, limg);

  // convert to floats:
  Image<float> a = aimg, b = bimg, l = limg;

  // feed our subchannels with float versions of the images:
  itsA->input(InputFrame::fromGrayFloat(&a, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsB->input(InputFrame::fromGrayFloat(&b, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsL->input(InputFrame::fromGrayFloat(&l, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  LINFO("Imagize Color channel ok.");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
