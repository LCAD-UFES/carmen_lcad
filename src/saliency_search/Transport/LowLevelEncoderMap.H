/*!@file Transport/LowLevelEncoderMap.H A partial implementation of FrameOstream, using a map between 'shortnames' and LowLevelEncoder objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/LowLevelEncoderMap.H $
// $Id: LowLevelEncoderMap.H 8905 2007-10-25 23:21:41Z rjpeters $
//

#ifndef TRANSPORT_LOWLEVELENCODERMAP_H_DEFINED
#define TRANSPORT_LOWLEVELENCODERMAP_H_DEFINED

#include "Transport/FrameOstream.H"
#include "Transport/LowLevelEncoder.H"
#include "rutz/shared_ptr.h"

class GenericFrameSpec;

/// A partial implementation of FrameOstream, using a map between 'shortnames' and LowLevelEncoder objects
/** Subclasses must implement makeEncoderForStem(), but do not need to
    implement writeFrame() or closeStream(). Subclasses may implement
    setConfigInfo() as needed. */
class LowLevelEncoderMap : public FrameOstream
{
public:
  LowLevelEncoderMap(OptionManager& mgr,
                      const std::string& descrName,
                      const std::string& tagName);

  virtual ~LowLevelEncoderMap();

  /// Send the frame to the LowLevelEncoder corresponding to shortname
  /** If no such encoder exists yet, then a new one will be created by
      calling the subclass's implementation of makeEncoderForStem(). */
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo);

  /// Close and delete the LowLevelEncoder corresponding to shortname
  /*! This means that if additional frames are later sent to that same
      shortname, then a new LowLevelEncoder object will be created for
      that shortname (possibly overwriting previous files or output
      destinations, etc.). */
  virtual void closeStream(const std::string& shortname);

protected:
  /// Must be implemented by subclasses to return a type-appropriate encoder object
  virtual rutz::shared_ptr<LowLevelEncoder>
  makeEncoder(const GenericFrameSpec& spec,
              const std::string& shortname,
              const FrameInfo& auxinfo) = 0;

  /// Overridden to destroy our encoder map
  virtual void stop2();

private:
  LowLevelEncoderMap(const LowLevelEncoderMap&);
  LowLevelEncoderMap& operator=(const LowLevelEncoderMap&);

  struct Impl;
  Impl* rep;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_LOWLEVELENCODERMAP_H_DEFINED
