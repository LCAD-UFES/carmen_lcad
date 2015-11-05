/*!@file Channels/DKLcolorChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DKLcolorChannel.C $
// $Id: DKLcolorChannel.C 15164 2012-02-22 03:58:13Z dberg $
//

#include "Channels/DKLcolorChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/DcolorChannel.H"
#include "Channels/KcolorChannel.H"
#include "Channels/LcolorChannel.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "rutz/trace.h"

const ModelOptionDef OPT_DKLcolorChannelDweight =
  { MODOPT_ARG(float), "DKLcolorChannelDweight", &MOC_CHANNEL, OPTEXP_CORE,
    "Weight to assign to the D component of the DKL color channel",
    "dkl-d-weight", '\0', "<float>", "1.0" };

const ModelOptionDef OPT_DKLcolorChannelKweight =
  { MODOPT_ARG(float), "DKLcolorChannelKweight", &MOC_CHANNEL, OPTEXP_CORE,
    "Weight to assign to the K component of the DKL color channel",
    "dkl-k-weight", '\0', "<float>", "1.0" };

const ModelOptionDef OPT_DKLcolorChannelLweight =
  { MODOPT_ARG(float), "DKLcolorChannelLweight", &MOC_CHANNEL, OPTEXP_CORE,
    "Weight to assign to the L component of the DKL color channel",
    "dkl-l-weight", '\0', "<float>", "1.0" };

const ModelOptionDef OPT_DKLType =
  { MODOPT_ARG(DKLType), "DKLType", &MOC_CHANNEL, OPTEXP_CORE,
    "The type of DKL model to use.",
    "dkl-type", '\0', "<DKLIsa, DKLMunoz>", "DKLIsa" };

// ######################################################################
// DKLcolorChannel member definitions:
// ######################################################################

DKLcolorChannel::DKLcolorChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "DKLcolor", "DKLcolor", DKLCOLOR),
  itsDweight(&OPT_DKLcolorChannelDweight, this),
  itsKweight(&OPT_DKLcolorChannelKweight, this),
  itsLweight(&OPT_DKLcolorChannelLweight, this),
  itsDKLTypeParam(&OPT_DKLType, this),
  itsD(new DcolorChannel(getManager(), true)),
  itsK(new KcolorChannel(getManager(), true)),
  itsL(new LcolorChannel(getManager(), true)),
  itsDKLType(DKLIsa)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  this->addSubChan(itsD);
  this->addSubChan(itsK);
  this->addSubChan(itsL);
}

// ######################################################################
DcolorChannel& DKLcolorChannel::Dcolor() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsD;
}

// ######################################################################
KcolorChannel& DKLcolorChannel::Kcolor() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsK;
}

// ######################################################################
LcolorChannel& DKLcolorChannel::Lcolor() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsL;
}

// ######################################################################
DKLcolorChannel::~DKLcolorChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void DKLcolorChannel::start1()
{
  if (itsDweight.getVal() == 0.0)
  {
    removeSubChan(itsD);
  }
  else
  {
    LINFO("Using a weight of %f for D subchannel", itsDweight.getVal());
    setSubchanTotalWeight(*itsD, itsDweight.getVal());
  }

  if (itsKweight.getVal() == 0.0)
  {
    removeSubChan(itsK);
  }
  else
  {
    LINFO("Using a weight of %f for K subchannel", itsKweight.getVal());
    setSubchanTotalWeight(*itsK, itsKweight.getVal());
  }

  if (itsLweight.getVal() == 0.0)
  {
    removeSubChan(itsL);
  }
  else
  {
    LINFO("Using a weight of %f for L subchannel", itsLweight.getVal());
    setSubchanTotalWeight(*itsL, itsLweight.getVal());
  }

  itsDKLType = itsDKLTypeParam.getVal();
}

// ######################################################################
void DKLcolorChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> dimg, kimg, limg;

  // get the D, K, L components, as signed floats:
  LINFO("Converting input image to DKL colors.");
  if (itsDKLType == DKLIsa)
    //here D is luminance, K is red-green and L is blue-yellow
    getDKL(inframe.colorByte(), dimg, kimg, limg);
  else if (itsDKLType == DKLMunoz)
    //here D is red-green, K is blue-yellow and L is luminance
    getDKLM(inframe.colorByte(), dimg, kimg, limg);
  else
    LFATAL("Unknown DKL type");

  // feed our subchannels:
  if (itsDweight.getVal() > 0.0)
    itsD->input(InputFrame::fromGrayFloat(&dimg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  if (itsKweight.getVal() > 0.0)
    itsK->input(InputFrame::fromGrayFloat(&kimg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  if (itsLweight.getVal() > 0.0)
    itsL->input(InputFrame::fromGrayFloat(&limg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  LINFO("DKL Color channel ok.");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
