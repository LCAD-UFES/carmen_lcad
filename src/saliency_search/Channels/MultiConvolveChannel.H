/*!@file Channels/MultiConvolveChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MultiConvolveChannel.H $
// $Id: MultiConvolveChannel.H 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef MULTICONVOLVECHANNEL_H_DEFINED
#define MULTICONVOLVECHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"

// ######################################################################
//! A generic multi-convolution channel
/*! This channel reads a filter definition file from disk (text
  format) and builds a number of ConvolveChannel subchannels. In the
  current implementation, only 4 subchannels are supported, and their
  inputs will be the red, green, blue and yellow color components of
  the image as given by getRGBY(). Each subchannel will convolve its
  input using the convolution filter definition given in the text
  file. */
class MultiConvolveChannel : public ComplexChannel
{
public:
  //! Construct with standard params
  MultiConvolveChannel(OptionManager& mgr);

  //! Destructor
  virtual ~MultiConvolveChannel();

protected:
  NModelParam<std::string> itsFilterFname; //!< file name for the filters
  NModelParam<float> itsLumThresh; //!< Luminance threshold (see getRGBY())

  //! read filter definition and instantiate subchannels
  void start1();

  //! MultiConvolveChannel requires only color input
  virtual void doInput(const InputFrame& inframe);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MULTICONVOLVECHANNEL_H_DEFINED
