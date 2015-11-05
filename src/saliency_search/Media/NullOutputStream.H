/*!@file Media/NullOutputStream.H Write frames to .null files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/NullOutputStream.H $
// $Id: NullOutputStream.H 8069 2007-03-08 05:01:16Z rjpeters $
//

#ifndef MEDIA_NULLOUTPUTSTREAM_H_DEFINED
#define MEDIA_NULLOUTPUTSTREAM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Transport/FrameOstream.H"

// ######################################################################
// ######################################################################
//! NullOutputStream writes nothing, it is a fake output stream
/*! This may be useful in situations where you want to guarantee that
  you are running a model in the exact same conditions as you would
  normally do when you write, e.g., a movie out with your results,
  except that you don't want to spend the time encoding the
  movie. Typically this may be useful while doing parameter searches
  where you want to process data but do not need the visual displays
  until you have found your final set of parameters. With
  NullOutputStream selected as output, you will still go through the
  motions of building output frames as you normally would, just they
  will not be saved anywhere. */

class NullOutputStream : public FrameOstream
{
public:
  //! Construct an mpegstream object for mpeg output
  NullOutputStream(OptionManager& mgr,
                   const std::string& descrName = "Null Output Stream",
                   const std::string& tagName = "NullOutputStream");

  //! Destructor
  virtual ~NullOutputStream();

  //! Write a frame to null
  /*! We just print a message and discard the frame. */
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo =
                          FrameOstream::defaultInfo);

  //! Do nothing
  virtual void closeStream(const std::string& shortname);

  // Just use the default versions of everything else from FrameOstream.
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_NULLOUTPUTSTREAM_H_DEFINED
