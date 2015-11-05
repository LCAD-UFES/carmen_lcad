/*!@file Channels/GaborChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/GaborChannel.C $
// $Id: GaborChannel.C 9321 2008-02-25 05:55:31Z mundhenk $
//

#ifndef GABORCHANNEL_C_DEFINED
#define GABORCHANNEL_C_DEFINED

#include "Channels/GaborChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Image/PyramidOps.H"
#include "Util/sformat.H"

// ######################################################################
// GaborChannel member definitions:
// ######################################################################

// ######################################################################
GaborChannel::GaborChannel(OptionManager& mgr, const uint oriIndex,
                           const double ang) :
  SingleChannel(mgr, "", "", ORI, rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsOriIndex("GaborChannelOrientationIndex", this, oriIndex),
  itsOrientation("GaborChannelOrientation", this, ang),
  itsGaborIntens(&OPT_GaborChannelIntensity, this),
  itsOriCompType(&OPT_OrientComputeType, this),
  itsUseTrigTab(&OPT_UseTrigTab, this)
{
  itsNormalizeOutput.setVal(true);

  setDescriptiveName(sformat("Gabor(%d)", int(ang)));
  setTagName(sformat("ori_%d", oriIndex));
}

// ######################################################################
GaborChannel::GaborChannel(OptionManager& mgr, const uint oriIndex,
                           const double ang, const char* tag, const char* desc) :
  SingleChannel(mgr, "", "", ORI, rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsOriIndex("GaborChannelOrientationIndex", this, oriIndex),
  itsOrientation("GaborChannelOrientation", this, ang),
  itsGaborIntens(&OPT_GaborChannelIntensity, this),
  itsOriCompType(&OPT_OrientComputeType, this),
  itsUseTrigTab(&OPT_UseTrigTab, this)
{
  itsNormalizeOutput.setVal(true);

  setDescriptiveName(sformat("Gabor-%s(%d)", desc, int(ang)));
  setTagName(sformat("ori-%s_%d", tag, oriIndex));
}
// ######################################################################
void GaborChannel::start1()
{
  SingleChannel::start1();
  resetPyramid();
}

// ######################################################################
GaborChannel::~GaborChannel()
{ }

// ######################################################################
void GaborChannel::readFrom(const ParamMap& pmap)
{
  SingleChannel::readFrom(pmap);
  double gi = itsGaborIntens.getVal();
  int otype = (int)itsOriCompType.getVal();
  bool doPyrReset = false;


  if (pmap.queryDoubleParam("gaborIntens", gi) == ParamMap::CHANGED)
    {
      itsGaborIntens.setVal(gi);
      doPyrReset = true;
    }

  if (pmap.queryIntParam("oriCompType", otype) == ParamMap::CHANGED)
    {
      itsOriCompType.setVal((OrientComputeType)otype);
      doPyrReset = true;
    }

  if (doPyrReset) resetPyramid();
}

// ######################################################################
void GaborChannel::writeTo(ParamMap& pmap) const
{
  SingleChannel::writeTo(pmap);
  pmap.putDoubleParam("gaborIntens", itsGaborIntens.getVal());
  pmap.putIntParam("oriCompType", (int)itsOriCompType.getVal());
}

// ######################################################################
double GaborChannel::angle() const
{ return itsOrientation.getVal(); }

// ######################################################################
void GaborChannel::resetPyramid()
{
  switch(itsOriCompType.getVal())
    {
    case ORISteerable:
      setPyramid(rutz::make_shared(new OrientedPyrBuilder<float>
                            (9, itsOrientation.getVal(),
                             itsGaborIntens.getVal(),
                             itsUseTrigTab.getVal())));
      break;

    case ORIGabor:
      setPyramid(rutz::make_shared(new GaborPyrBuilder<float>
                            (itsOrientation.getVal(),7,1,9,0)));
       break;

    case ORIGaborEnergyNorm:
      setPyramid(rutz::make_shared(new GaborPyrBuilder<float>
                            (itsOrientation.getVal(),7,1,9,
                             DO_ENERGY_NORM)));
       break;

    default:
      LFATAL("Unknown OrientComputeType");
    }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // GABORCHANNEL_C_DEFINED
