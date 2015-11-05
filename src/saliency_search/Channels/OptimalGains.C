/*!@file Channels/OptimalGains.C Compute the optimal gains that maximize SNR */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/OptimalGains.C $
// $Id: OptimalGains.C 10794 2009-02-08 06:21:09Z itti $
//

#ifndef CHANNELS_OPTIMALGAINS_C_DEFINED
#define CHANNELS_OPTIMALGAINS_C_DEFINED

#include "Channels/OptimalGains.H"

#include "Channels/ChannelFacets.H"
#include "Channels/ComplexChannel.H"
#include "Channels/SingleChannel.H"
#include "Component/ParamMap.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H" // for rescale()
#include "Util/sformat.H"

#include <vector>

// ######################################################################
OptimalGainsFinder::OptimalGainsFinder(const Image<byte>& targetMask,
                                       const Image<byte>& distractorMask,
                                       rutz::shared_ptr<ParamMap> pmap,
                                       const bool doMax)
  :
  itsTargetMask(targetMask),
  itsDistractorMask(distractorMask),
  itsPmap(pmap),
  itsDoMax(doMax)
{ }

// ######################################################################
OptimalGainsFinder::~OptimalGainsFinder()
{ }

// ######################################################################
void OptimalGainsFinder::visitChannelBase(ChannelBase& chan)
{ LFATAL("don't know how to handle %s", chan.tagName().c_str()); }

// ######################################################################
void OptimalGainsFinder::visitSingleChannel(SingleChannel& chan)
{
  // get or install some ChannelFacet for the gains:
  rutz::shared_ptr<ChannelFacetGainSingle> gfacet;
  if (chan.hasFacet<ChannelFacetGainSingle>())
    gfacet = chan.getFacet<ChannelFacetGainSingle>();
  else
    { gfacet.reset(new ChannelFacetGainSingle(chan)); chan.setFacet(gfacet); }

  /* 1. for each submap i, find sT and sD, the salience of the target T
        and distractors D
     2. find SNR
     3. find g = SNR / (sum(SNR_j) / n) where n is the number of submaps. */

  const uint num = chan.numSubmaps();
  float sumSNR = 0.0f, SNR[num];

  Image<byte> tmap, dmap;
  if (itsTargetMask.initialized())
    tmap = rescale(itsTargetMask, chan.getMapDims());
  if (itsDistractorMask.initialized())
    dmap = rescale(itsDistractorMask, chan.getMapDims());

  for (uint idx = 0; idx < num; idx ++)
    {
      const Image<float> submap = chan.getSubmap(idx);
      float sT = 0.0f, sD = 0.0f; // default values if some mask is missing
      float junk1, junk2, junk3;

      // how to compute SNR? max(sT) / max(sD) or mean(sT) / mean(sD)??
      if (itsDoMax)
        {
          // SNR = max(sT) / max(sD)
          // sT is max salience within target object
          if (tmap.initialized())
            getMaskedMinMax(submap, tmap, junk1, sT, junk2, junk3);
          // sD is max salience within distractor object
          if (dmap.initialized())
            getMaskedMinMax(submap, dmap, junk1, sD, junk2, junk3);
        }
      else
        {
          // SNR = mean(sT) / mean(sD)
          // sT is mean salience within target object
          if (tmap.initialized())
            getMaskedMinMaxAvg(submap, tmap, junk1, junk2, sT);
          // sD is mean salience within distractor object
          if (dmap.initialized())
            getMaskedMinMaxAvg(submap, dmap, junk1, junk2, sD);
        }

      SNR[idx] = (sT + OPTIGAIN_BG_FIRING) / (sD + OPTIGAIN_BG_FIRING);
      sumSNR += SNR[idx];

      // store these salience values so that they can be written out later
      itsPmap->putDoubleParam(sformat("salienceT(%d)", idx), sT);
      itsPmap->putDoubleParam(sformat("salienceD(%d)", idx), sD);

      uint c = 0, s = 0; chan.getLevelSpec().indexToCS(idx, c, s);
      LDEBUG("%s(%d,%d): sT=%f, sD=%f", chan.tagName().c_str(), c, s, sT, sD);
    }
  sumSNR /= num;

  // find the optimal gains g
  for (uint idx = 0; idx < num; idx ++)
    {
      const float gain = SNR[idx] / sumSNR;
      uint clev = 0, slev = 0; chan.getLevelSpec().indexToCS(idx, clev, slev);
      LINFO("%s(%d,%d): gain = %f, SNR = %f", chan.tagName().c_str(),
            clev, slev, gain, SNR[idx]);
      gfacet->setVal(idx, gain);
    }

  // find the biased saliency map for this single channel and cache it
  // so that the parent complex channels can use it to find SNR:
  chan.killCaches();
  (void) chan.getOutput();
}

// ######################################################################
void OptimalGainsFinder::visitComplexChannel(ComplexChannel& chan)
{
  // get or install some ChannelFacet for the gains:
  rutz::shared_ptr<ChannelFacetGainComplex> gfacet;
  if (chan.hasFacet<ChannelFacetGainComplex>())
    gfacet = chan.getFacet<ChannelFacetGainComplex>();
  else
    { gfacet.reset(new ChannelFacetGainComplex(chan)); chan.setFacet(gfacet); }

  // first find the optimal gains within sub channels
  const uint num = chan.numChans();
  rutz::shared_ptr<ParamMap> pmapsave = itsPmap;
  for (uint idx = 0; idx < num; idx ++)
    {
      // visit the subchan, it will update itsPmap:
      itsPmap.reset(new ParamMap());
      chan.subChan(idx)->accept(*this);

      // store subchan index, for human consumption:
      itsPmap->putIntParam("subchanidx", idx);

      // store the submap for the subchan:
      pmapsave->putSubpmap(chan.subChan(idx)->tagName(), itsPmap);
    }
  itsPmap.swap(pmapsave);

  /* 1. for each subchannel i, find sT and sD, the salience of the target T
        and distractors D
     2. find SNR
     3. find g = SNR / (sum(SNR_j) / n) where n is the number of subchannels */
  float sumSNR = 0.0f, SNR[num];

  Image<byte> tmap, dmap;
  if (itsTargetMask.initialized())
    tmap = rescale(itsTargetMask, chan.getMapDims());
  if (itsDistractorMask.initialized())
    dmap = rescale(itsDistractorMask, chan.getMapDims());

  // next, find the optimal gains for the sub channels
  for (uint idx = 0; idx < num; idx ++)
    {
      // find the biased saliency maps for the sub channels
      const Image<float> submap = chan.subChan(idx)->getOutput();
      float sT = 0.0f, sD = 0.0f; // default values if some mask is missing
      float junk1, junk2, junk3;

      // how to compute SNR? max(sT) / max(sD) or mean(sT) / mean(sD)??
      if (itsDoMax)
        {
          // SNR = max(sT) / max(sD)
          // sT is max salience within target object
          if (tmap.initialized())
            getMaskedMinMax(submap, tmap, junk1, sT, junk2, junk3);
          // sD is max salience within distractor object
          if (dmap.initialized())
            getMaskedMinMax(submap, dmap, junk1, sD, junk2, junk3);
        }
      else
        {
          // SNR = mean(sT) / mean(sD)
          // sT is mean salience within target object
          if (tmap.initialized())
            getMaskedMinMaxAvg(submap, tmap, junk1, junk2, sT);
          // sD is mean salience within distractor object
          if (dmap.initialized())
            getMaskedMinMaxAvg(submap, dmap, junk1, junk2, sD);
        }

      SNR[idx] = (sT + OPTIGAIN_BG_FIRING) / (sD + OPTIGAIN_BG_FIRING);
      sumSNR += SNR[idx];

      // store these salience values so that they can be written out later
      itsPmap->putDoubleParam(sformat("salienceT(%d)", idx), sT);
      itsPmap->putDoubleParam(sformat("salienceD(%d)", idx), sD);

      LDEBUG("%s: sT=%f, sD=%f", chan.subChan(idx)->tagName().c_str(), sT, sD);
    }
  sumSNR /= num;

  // find the optimal gains
  for (uint idx = 0; idx < num; idx ++)
    {
      const float gain = SNR[idx] / sumSNR;
      LINFO("%s: gain = %f, SNR = %f", chan.subChan(idx)->tagName().c_str(),
            gain, SNR[idx]);
      gfacet->setVal(idx, gain);
    }

  // find the biased saliency map and cache it so that visual cortex
  // may use it to compute SNR:
  chan.killCaches();
  (void) chan.getOutput();
}

// ######################################################################
rutz::shared_ptr<ParamMap> OptimalGainsFinder::pmap() const
{ return itsPmap; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_OPTIMALGAINS_C_DEFINED
