/*!@file Surprise/SingleChannelSurprise.H Channel for a single stream of processing. */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SingleChannelSurprise.H $
// $Id: SingleChannelSurprise.H 10107 2008-08-15 06:45:49Z mundhenk $
//

#ifndef SINGLECHANNELSURPRISE_H_DEFINED
#define SINGLECHANNELSURPRISE_H_DEFINED

#include "Channels/SubmapAlgorithm.H"
#include "Component/ModelParam.H"
#include "Image/LevelSpec.H"
#include "Surprise/SurpriseMap.H"
#include <vector>

// ######################################################################
//! SingleChannelSurprise is a Surprise extension to SingleChannel
/*! This class only provides the computation of surprise. Typically
    you would first build a SingleChannel (or derivative), then build
    a SingleChannelSurprise, then pass the SingleChannelSurprise to
    the channel's setSubmapAlgorithm(). See VisualCortexSurprise for
    example usage. */
template <class SMODEL>
class SingleChannelSurprise : public SubmapAlgorithm
{
public:
  //! Constructor
  SingleChannelSurprise(OptionManager& mgr);

  //! destructor
  virtual ~SingleChannelSurprise();

  //! get a surprise map
  /*! Pass a submap index and a submap, and you'll get back the
    associated surprise */
  Image<float> getSurpriseMap(const uint index,
                              const Image<float>& submap);

  //! Compute the i'th submap for the given channel
  virtual Image<float> compute(const SingleChannel& chan, const uint i);

protected:
  OModelParam<uint> itsSQlen;            //!< number of surprise maps
  OModelParam<double> itsUpdateFac;      //!< update factor
  OModelParam<double> itsNeighUpdateFac; //!< update factor for neighbors
  NModelParam<double> itsInitialVal;     //!< Initial values
  NModelParam<double> itsInitialVar;     //!< Initial variance
  NModelParam<double> itsBgVal;          //!< Background values
  OModelParam<float> itsNeighSigma;      //!< Neighborhood sigma
  OModelParam<float> itsLocSigma;        //!< Local neighborhood sigma
  OModelParam<bool> itsTakeSTMax;        //!< Use Max for space-time
  OModelParam<bool>  itsLogged;          //!< Do we dump data to a log file
  OModelParam<LevelSpec> itsLevelSpec;   //!< our levelspec
  OModelParam<Point2D<int> > itsProbe;   //!< location of a virtusl electrode
  OModelParam<double> itsSLfac;          //!< fact for local temporal surprise
  OModelParam<double> itsSSfac;          //!< factor for spatial surprise
  OModelParam<std::string> itsJointKLBiasTypeStr;

  // get us started (see ModelComponent.H)
  virtual void start1();

private:
  SingleChannelSurprise(const SingleChannelSurprise&); // not allowed
  SingleChannelSurprise& operator=(const SingleChannelSurprise&); // not allow
  std::vector< SurpriseMap<SMODEL> > itsSmap;

  //! log entry number
  uint       itsLogEntry;
  SU_KL_BIAS itsJointKLBiasType;
};

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
