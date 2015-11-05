/*!@file Channels/IntegerColorChannel.H */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerColorChannel.H $
// $Id: IntegerColorChannel.H 7857 2007-02-07 21:28:59Z rjpeters $
//

#ifndef CHANNELS_INTEGERCOLORCHANNEL_H_DEFINED
#define CHANNELS_INTEGERCOLORCHANNEL_H_DEFINED

#include "Channels/IntegerComplexChannel.H"
#include "Channels/IntegerSimpleChannel.H"

// ######################################################################
//! A double opponent color channel class that combines r/g, b/y subchannels.
/*! This is an example of an heterogeneous complex channel where the
  various subchannels have distinct visual features. Thus, the JetSpec
  representation for such complex channel will be a series of
  independent single-channel representations, one for each
  subchannel. */
class IntegerColorChannel : public IntegerComplexChannel
{
public:
  //! Construct with standard params.
  IntegerColorChannel(OptionManager& mgr,
                      nub::ref<IntegerMathEngine> eng);

  //! Destructor
  virtual ~IntegerColorChannel();

  //! Get access the IntegerSimpleChannel subchannel.
  virtual IntegerSimpleChannel& rg() const;

  //! Get access to the IntegerSimpleChannel subchannel.
  virtual IntegerSimpleChannel& by() const;

protected:
  NModelParam<float> itsLumThresh; //!< Luminance threshold (see getRGBY())

  virtual void doInputInt(const IntegerInput& inp,
                          const SimTime& t,
                          PyramidCache<int>* cache,
                          const Image<byte>& clipMask);

private:
  nub::ref<IntegerSimpleChannel> itsRG;
  nub::ref<IntegerSimpleChannel> itsBY;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERCOLORCHANNEL_H_DEFINED
