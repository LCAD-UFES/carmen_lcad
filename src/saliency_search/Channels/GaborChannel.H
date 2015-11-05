/*!@file Channels/GaborChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/GaborChannel.H $
// $Id: GaborChannel.H 9321 2008-02-25 05:55:31Z mundhenk $
//

#ifndef GABORCHANNEL_H_DEFINED
#define GABORCHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"
#include "Channels/OrientComputeTypes.H"

// ######################################################################
//! An orientation-tuned gabor channel.
class GaborChannel : public SingleChannel
{
public:
  //! Construct with standard params plus an orientation index for this object.
  /*! @param oriIndex is used for various indexing such as with ParamMap
    and Jet interfacing.
    @param ang is the preferred orientation for this channel, in
      degrees (0..180). */
  GaborChannel(OptionManager& mgr, const uint oriIndex, const double ang);
  //! Overide the standard tag name
  GaborChannel(OptionManager& mgr, const uint oriIndex, const double ang,
               const char* tag, const char* desc);
  //! Destructor
  virtual ~GaborChannel();

  virtual void readFrom(const ParamMap& pmap);

  virtual void writeTo(ParamMap& pmap) const;

  //! Get the preferred orientation (in degrees) of this channel.
  double angle() const;

protected:
  NModelParam<uint> itsOriIndex;
  NModelParam<double> itsOrientation;
  OModelParam<double> itsGaborIntens;

  //! determines the type of computation used to compute orientation
  /*! see OrientComputeTypes.H for possible values. */
  OModelParam<OrientComputeType> itsOriCompType;

  OModelParam<bool> itsUseTrigTab;

  virtual void start1();
  void resetPyramid();
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // GABORCHANNEL_H_DEFINED
