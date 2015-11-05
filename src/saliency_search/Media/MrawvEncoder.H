/*!@file Media/MrawvEncoder.H Low-level encoder for multi-frame raw video formats */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MrawvEncoder.H $
// $Id: MrawvEncoder.H 8946 2007-11-06 01:08:41Z rjpeters $
//

#ifndef MEDIA_MRAWVENCODER_H_DEFINED
#define MEDIA_MRAWVENCODER_H_DEFINED

#include "Transport/LowLevelEncoder.H"

#include <cstdio>

class GenericFrameSpec;

/// Low-level encoder for multi-frame raw video formats
class MrawvEncoder : public LowLevelEncoder
{
public:
  /// Constructor
  MrawvEncoder(const GenericFrameSpec& spec,
               const std::string& fname,
               const bool scale255);

  virtual ~MrawvEncoder();

  /// Close any underlying streams; return 0 if OK, non-zero if error
  virtual int close();

  /// Write a frame to the low-level stream
  virtual void writeFrame(const GenericFrame& f);

private:
  std::string itsFname;
  FILE* itsFile;
  const bool itsScale255;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MRAWVENCODER_H_DEFINED
