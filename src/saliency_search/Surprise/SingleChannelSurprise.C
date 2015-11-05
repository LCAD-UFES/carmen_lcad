/*!@file Surprise/SingleChannelSurprise.C Channel for a single stream of processing. */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SingleChannelSurprise.C $
// $Id: SingleChannelSurprise.C 11562 2009-08-08 00:35:40Z dberg $
//

#include "Surprise/SingleChannelSurprise.H"

#include "Channels/ChannelOpts.H"
#include "Channels/SingleChannel.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Surprise/SurpriseOpts.H"
#include "Raster/Raster.H"
#include "Image/ShapeOps.H"
#include "Image/Normalize.H"
#include "Image/MathOps.H"
#include "rutz/demangle.h"

#include <iostream>
#include <fstream>
#include <typeinfo>

// ######################################################################
template <class SMODEL>
SingleChannelSurprise<SMODEL>::
SingleChannelSurprise(OptionManager& mgr) :
  SubmapAlgorithm(mgr, "Single Channel Surprise", "SingleChannelSurprise"),
  itsSQlen(&OPT_SingleChannelSurpriseSQlen, this),
  itsUpdateFac(&OPT_SingleChannelSurpriseUpdFac, this),
  itsNeighUpdateFac(&OPT_SingleChannelSurpriseNeighUpdFac, this),
  itsInitialVal("SingleChannelSurpriseIniVal", this, 0.0),
  itsInitialVar("SingleChannelSurpriseIniVar", this, 25.0),
  itsBgVal("SingleChannelSurpriseBgVal", this, 5.0),
  itsNeighSigma(&OPT_SingleChannelSurpriseNeighSigma, this),
  itsLocSigma(&OPT_SingleChannelSurpriseLocSigma, this),
  itsTakeSTMax(&OPT_SingleChannelSurpriseTakeSTMax, this),
  itsLogged(&OPT_SingleChannelSurpriseLogged, this),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsProbe(&OPT_SingleChannelSurpriseProbe, this),
  itsSLfac(&OPT_SingleChannelSurpriseSLfac, this),
  itsSSfac(&OPT_SingleChannelSurpriseSSfac, this),
  itsJointKLBiasTypeStr(&OPT_SingleChannelSurpriseKLBias, this),
  itsSmap(), itsLogEntry(0U)
{
  this->setDescriptiveName(std::string("Single Channel ") +
                           rutz::demangled_name(typeid(SMODEL)));
  this->setTagName(std::string("SingleChannel") +
                   rutz::demangled_name(typeid(SMODEL)));
}

// ######################################################################
template <class SMODEL>
SingleChannelSurprise<SMODEL>::~SingleChannelSurprise()
{ }

// ######################################################################
template <class SMODEL>
void SingleChannelSurprise<SMODEL>::start1()
{
  int maxind = itsLevelSpec.getVal().maxIndex();

  if(!strcmp(itsJointKLBiasTypeStr.getVal().c_str(),"Static"))
    itsJointKLBiasType = SU_KL_STATIC;
  else
    itsJointKLBiasType = SU_KL_NONE;

  SurpriseMap<SMODEL> initial;
  initial.init(itsSQlen.getVal(), itsUpdateFac.getVal(),
               itsNeighUpdateFac.getVal(),
               itsInitialVal.getVal() + itsBgVal.getVal(),
               itsInitialVar.getVal(), itsNeighSigma.getVal(),
               itsLocSigma.getVal(), itsProbe.getVal(),
               itsSLfac.getVal(), itsSSfac.getVal(),
               itsJointKLBiasType, itsTakeSTMax.getVal());

  for (int i = 0; i < maxind; i ++) itsSmap.push_back(initial);
}

// ######################################################################
template <class SMODEL>
Image<float> SingleChannelSurprise<SMODEL>::getSurpriseMap(const uint index,
                                                   const Image<float>& submap)
{
  Image<double> submapd = submap;
  if (itsBgVal.getVal()) submapd += itsBgVal.getVal(); // add bg firing rate

  // create a SurpriseImage from the submap:
  Image<double> var(submap.getDims(), NO_INIT);
  var.clear(itsInitialVar.getVal());

  SurpriseImage<SMODEL> sample(itsUpdateFac.getVal(), submapd, var);

  // now inject that into our models and get the surprise:
  return Image<float>(itsSmap[index].surprise(sample));
}

// ######################################################################
template <class SMODEL>
Image<float> SingleChannelSurprise<SMODEL>::
compute(const SingleChannel& chan, const uint i)
{
// implementation that computes surprise at full map resolution
// return chan.postProcessMap(this->getSurpriseMap(i, chan.getRawCSmap(i)), i)
  return this->getSurpriseMap
    (i, chan.postProcessMap(chan.getRawCSmap(i), i));
}

// ######################################################################
// Explicit instantiations:

template class SingleChannelSurprise<SurpriseModelSG>;
template class SingleChannelSurprise<SurpriseModelSP>;
template class SingleChannelSurprise<SurpriseModelSP1>;
template class SingleChannelSurprise<SurpriseModelSPC>;
template class SingleChannelSurprise<SurpriseModelSPF>;
template class SingleChannelSurprise<SurpriseModelCS>;
template class SingleChannelSurprise<SurpriseModelGG>;
template class SingleChannelSurprise<SurpriseModelPM>;
template class SingleChannelSurprise<SurpriseModelOD>;


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
