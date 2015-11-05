/*!@file Channels/ChannelMaps.H Classes to hold maps from a Channel hierarchy */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelMaps.H $
// $Id: ChannelMaps.H 12574 2010-01-15 02:09:17Z itti $
//

#ifndef CHANNELS_CHANNELMAPS_H_DEFINED
#define CHANNELS_CHANNELMAPS_H_DEFINED

#include "Image/NamedImage.H"
#include "Image/ImageSet.H"
class ChannelBase;
class EnvVisualCortexFloat;

//######################################################################
//! ChannelMaps hold maps from a channel hierarchy
/*! These classes are used to take a copy of all the maps in a Channel
    hierarchy. Note that simply summing all the maps will not give
    rise to a saliency map, as some weighing and non-linear operations
    will be missing. The purpose here is to allow the maps to be
    transported to various places, e.g., through a SimReq
    mechanism. */
class ChannelMaps {
public:
  //! Constructor
  ChannelMaps(ChannelBase* chan, const std::string& prefix = "");

  //! Constructor
  ChannelMaps(EnvVisualCortexFloat* vcx, const std::string& prefix = "");

  //! Constructor with just an output map and no subchans/submaps
  ChannelMaps(const NamedImage<float>& outmap);

  //! Destructor
  ~ChannelMaps();

  //! Get the output map
  const NamedImage<float>& getMap() const;

  //! number of subchannels, non-zero if complex, zero if single
  uint numSubchans() const;

  //! access to the subchannels
  rutz::shared_ptr<ChannelMaps> subChanMaps(const uint idx) const;

  //! Return the number of feature maps represented by this channel and its subchans
  uint numSubmaps() const;

  //! Get the feature map at the given index, linearizing the channel hierarchy
  const NamedImage<float>& getSubmap(const uint index) const;

  //! get raw CS map, linearizing the channel hierarchy
  const NamedImage<float>& getRawCSmap(const uint idx) const;

  //! do we have a raw image pyramid?
  bool hasPyramid() const;

  //! get access to our raw pyramid
  const ImageSet<float>& getPyramid() const;

private:
  NamedImage<float> itsOutputMap;

  std::vector< NamedImage<float> > itsSubmaps;
  std::vector< NamedImage<float> > itsRawCSmaps;
  ImageSet<float> itsPyramid;

  std::vector< rutz::shared_ptr<ChannelMaps> > itsSubchanMaps;

  void lookupSubmap(const uint idx, uint& subchan, uint& subidx) const;

  ChannelMaps(const ChannelMaps&); // not allowed
  ChannelMaps& operator=(const ChannelMaps&); // not allowed
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_CHANNELMAPS_H_DEFINED
