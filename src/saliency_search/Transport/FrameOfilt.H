/*!@file Transport/FrameOfilt.H Generic base class for output frame filters */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/FrameOfilt.H $
// $Id: FrameOfilt.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef TRANSPORT_FRAMEOFILT_H_DEFINED
#define TRANSPORT_FRAMEOFILT_H_DEFINED

#include "Transport/FrameOstream.H"

/// Generic base class for output frame filters
/** This class is the base class for output filter classes that can be
    chained together to perform one or more operations on output
    frames before they reach their final destination (display, disk,
    network, etc.) */
class FrameOfilt : public FrameOstream
{
public:
  /// Constructor
  FrameOfilt(OptionManager& mgr,
             const std::string& descrName,
             const std::string& tagName);

  /// Destructor
  virtual ~FrameOfilt();

  /// Build our frame destination from the given string
  /** Subclasses should NOT override setConfigInfo() from
      FrameOfilt. If they need additional configuration information,
      they should get it from a command-line option through an
      OModelParam, or if more flexibility is needed, then just inherit
      from FrameOstream itself rather than from FrameOfilt. */
  virtual void setConfigInfo(const std::string& cfg);

  /// Just pass the frame number onto our frame destination
  /** Subclasses should NOT override setFrameNumber() from
      FrameOfilt. If more flexibility is needed, then just inherit
      from FrameOstream itself rather than from FrameOfilt. */
  virtual bool setFrameNumber(int n);

  /// Sends the frame through filterFrame()
  /** Subclasses should NOT override writeFrame() from FrameOfilt;
      instead, they should implement filterFrame(). If more
      flexibility is needed, then just inherit from FrameOstream
      itself rather than from FrameOfilt.  */
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo);

  /// Default version just returns the result of isVoid() from our destination
  virtual bool isVoid() const;

  /// Just pass the closeStream() request on to our frame destination
  virtual void closeStream(const std::string& shortname);

protected:
  /// Subclasses implement their filtering here
  /** A trivial pass-through implementation would be
      dest.writeFrame(frame, shortname, auxinfo) */
  virtual void filterFrame(FrameOstream& dest,
                           const GenericFrame& frame,
                           const std::string& shortname,
                           const FrameInfo& auxinfo) = 0;

private:
  nub::soft_ref<FrameOstream> itsDest;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_FRAMEOFILT_H_DEFINED
