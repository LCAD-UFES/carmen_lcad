/*!@file Channels/FeatureBiaser.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/FeatureBiaser.C $
// $Id: FeatureBiaser.C 9720 2008-04-30 07:32:00Z itti $
//

#ifndef CHANNELS_FEATUREBIASER_C_DEFINED
#define CHANNELS_FEATUREBIASER_C_DEFINED

#include "Channels/FeatureBiaser.H"

#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"

// ######################################################################
FeatureBiaser::FeatureBiaser(const double* mean, const double* sigma)
  :
  itsMean(mean),
  itsSigma(sigma),
  itsIndex(0)
{}

// ######################################################################
FeatureBiaser::~FeatureBiaser() {}

// ######################################################################
void FeatureBiaser::visitChannelBase(ChannelBase& chan)
{
  LFATAL("don't know how to handle %s", chan.tagName().c_str());
}

// ######################################################################
void FeatureBiaser::visitSingleChannel(SingleChannel& chan)
{
  chan.killCaches();
  const uint num = chan.numSubmaps();
  for (uint i = 0; i < num; ++i)
    {
      const uint subidx = itsIndex % num;
      ASSERT( chan.getLevelSpec().indexOK(subidx) );


      LFATAL("FIXME");
      ///////      chan.setMean(subidx, this->itsMean[itsIndex]);
      //////// chan.setSigma(subidx, this->itsSigma[itsIndex]);
      ++itsIndex;
    }
}

// ######################################################################
void FeatureBiaser::visitComplexChannel(ComplexChannel& chan)
{
  for (uint i = 0; i < chan.numChans(); ++i)
    chan.subChan(i)->accept(*this);
}


// ######################################################################
WeightFinder::WeightFinder() : itsMax()
{
  itsMax.push_back(0.0);
}

// ######################################################################
WeightFinder::~WeightFinder() {}

// ######################################################################
void WeightFinder::visitChannelBase(ChannelBase& chan)
{
  LFATAL("don't know how to handle %s", chan.tagName().c_str());
}

// ######################################################################
void WeightFinder::visitSingleChannel(SingleChannel& chan)
{
  chan.killCaches();

  // first clamp our coefficients to [0,255]
      LFATAL("FIXME");
      //////  chan.clampCoeffs(0.0, 255.0);

  double sum = 0.0;
  // initialise the submap weights
  for (uint idx = 0; idx < chan.numSubmaps(); ++idx)
    {
      LFATAL("FIXME");
      double wt = 0.0;/////chan.getMean(idx) / (1.0 + chan.getSigma(idx));
      sum += wt;
      itsMax.back() = std::max(itsMax.back(), wt);
    }

  // normalize submap weights so that they add to 1
  for (uint idx = 0; idx < chan.numSubmaps(); ++idx)
    {
      LFATAL("FIXME");
      ////double wt = chan.getMean(idx) / (1.0 + chan.getSigma(idx));
      ////if (sum != 0.0) chan.setCoeff(idx, wt/sum);
    }
}

// ######################################################################
void WeightFinder::visitComplexChannel(ComplexChannel& chan)
{
  double sum = 0.0;
  for (uint i = 0; i < chan.numChans(); ++i)
    {
      // first find the subchannel's weights
      itsMax.push_back(0.0);
      chan.subChan(i)->accept(*this);
      const double wt = itsMax.back();
      itsMax.pop_back();

      // initialize channel's weight with the max submap/subchan weight
      chan.setSubchanTotalWeight(i, wt);

      itsMax.back() = std::max(itsMax.back(), wt);
      sum += wt;
    }

  // normalize subchannel weights so that they add to 1
  for (uint i = 0; i < chan.numChans(); ++i)
    {
      if (sum != 0.0)
        chan.setSubchanTotalWeight
          (i, chan.getSubchanTotalWeight(i)/sum);
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_FEATUREBIASER_C_DEFINED
