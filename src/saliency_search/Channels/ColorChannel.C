/*!@file Channels/ColorChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ColorChannel.C $
// $Id: ColorChannel.C 8210 2007-03-31 00:21:06Z rjpeters $
//

#ifndef COLORCHANNEL_C_DEFINED
#define COLORCHANNEL_C_DEFINED

#include "Channels/ColorChannel.H"

#include "Channels/BlueYellowChannel.H"
#include "Channels/YellowBlueChannel.H"
#include "Channels/ChannelOpts.H"
#include "Channels/RedGreenChannel.H"
#include "Channels/GreenRedChannel.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "rutz/trace.h"

// ######################################################################
// Double Opponent ColorChannel member definitions:
// ######################################################################

ColorChannel::ColorChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "Color", "color", COLOR),
  itsLumThresh("ColorChannelLuminanceThreshold", this, 25.5F),
  itsMethod(&OPT_ColorComputeType, this)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  itsRG = nub::soft_ref<RedGreenChannel>(new RedGreenChannel(getManager(), true));
  itsBY = nub::soft_ref<BlueYellowChannel>(new BlueYellowChannel(getManager(), true));

  this->addSubChan(itsRG);
  this->addSubChan(itsBY);
}

// ######################################################################
void ColorChannel::paramChanged(ModelParamBase* const param,
                                const bool valueChanged,
                                ParamClient::ChangeStatus* status)
{
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param has become different from our number of channels,
  // let's reconfigure:
  if (param == &itsMethod && valueChanged)
  {
    switch(itsMethod.getVal())
    {
      case COLstandardFull:
        {
          LINFO("Rebuilding Color Channels");
          //remove the old channels
          this->removeAllSubChans();

          //rebuild the channels without taking the abs value
          itsRG = nub::soft_ref<RedGreenChannel>(new RedGreenChannel(getManager(), false));
          itsGR = nub::soft_ref<GreenRedChannel>(new GreenRedChannel(getManager(), false));
          itsBY = nub::soft_ref<BlueYellowChannel>(new BlueYellowChannel(getManager(), false));
          itsYB = nub::soft_ref<YellowBlueChannel>(new YellowBlueChannel(getManager(), false));

          this->addSubChan(itsRG);
          this->addSubChan(itsGR);
          this->addSubChan(itsBY);
          this->addSubChan(itsYB);
        }
      default:
        break;
    }

  }

}

// ######################################################################
RedGreenChannel& ColorChannel::rg() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsRG;
}

// ######################################################################
BlueYellowChannel& ColorChannel::by() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsBY;
}

// ######################################################################
GreenRedChannel& ColorChannel::gr() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsGR;
}

// ######################################################################
YellowBlueChannel& ColorChannel::yb() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsYB;
}

// ######################################################################
ColorChannel::~ColorChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void ColorChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> rgimg, byimg;

  switch(itsMethod.getVal())
    {
    case COLstandard:
      ASSERT(inframe.colorByte().initialized());
      LINFO("Using 'standard' method to compute color opponencies.");
      getRGBY(inframe.colorByte(), rgimg, byimg, itsLumThresh.getVal());
      break;

    case COLstandardFull:
      {
        ASSERT(inframe.colorFloat().initialized());

        LINFO("Using 'standard Full' method to compute color opponencies.");
        Image<float> grimg, ybimg;
        Image<float> r,g,b,y;
        getRGBY(inframe.colorFloat(), r, g, b, y, itsLumThresh.getVal());
        rgimg = r-g; grimg = g - r;
        byimg = b-y; ybimg = y - b;
        itsGR->input(InputFrame::fromGrayFloat
                     (&grimg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
        itsYB->input(InputFrame::fromGrayFloat
                     (&ybimg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
      }
      break;

    case COLsimple:
      ASSERT(inframe.colorByte().initialized());
      LINFO("Using 'simple' method to compute color opponencies.");
      getRGBYsimple(inframe.colorByte(), rgimg, byimg, itsLumThresh.getVal());
      break;

    default:
      LFATAL("Unknown ColorComputeType!");
      break;
    }


  itsRG->input(InputFrame::fromGrayFloat(&rgimg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  itsBY->input(InputFrame::fromGrayFloat(&byimg, inframe.time(), &inframe.clipMask(), inframe.pyrCache()));
  LINFO("Double Opponent Color channel ok.");
}

// ######################################################################
void ColorChannel::setRG(nub::ref<RedGreenChannel> rg)
{
  this->removeSubChan(itsRG);
  itsRG = rg;
  this->addSubChan(itsRG);
}

// ######################################################################
void ColorChannel::setBY(nub::ref<BlueYellowChannel> by)
{
  this->removeSubChan(itsBY);
  itsBY = by;
  this->addSubChan(itsBY);
}

// ######################################################################
void ColorChannel::setGR(nub::ref<GreenRedChannel> gr)
{
  this->removeSubChan(itsGR);
  itsGR = gr;
  this->addSubChan(itsGR);
}

// ######################################################################
void ColorChannel::setYB(nub::ref<YellowBlueChannel> yb)
{
  this->removeSubChan(itsYB);
  itsYB = yb;
  this->addSubChan(itsYB);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COLORCHANNEL_C_DEFINED
