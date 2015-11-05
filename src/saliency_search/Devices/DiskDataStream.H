/*!@file Devices/DiskDataStream.H Multi-threaded data streamer to disk */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/DiskDataStream.H $
// $Id: DiskDataStream.H 12118 2009-12-04 22:47:43Z sychic $
//

#ifndef DISKIMAGESTREAM_H_DEFINED
#define DISKIMAGESTREAM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Transport/FrameOstream.H"
#include "rutz/shared_ptr.h"

#include <string>
#include <vector>

class GenericFrame;
class WorkThreadServer;

//! A multi-threaded data streamer to disk
/*! This class implements a cache to allow asynchronous streaming of
  series of data objects to disk. Users push their data objects as
  they become available in the main processing thread, and the class'
  internal thread will attempt to save them to disk as quickly as
  possible. */
class DiskDataStream : public FrameOstream
{
public:
  //! Constructor
  DiskDataStream(OptionManager& mgr,
                 const std::string& descrName = "Disk Data Stream",
                 const std::string& tagName = "DiskDataStream");

  //! Destructor
  ~DiskDataStream();

  //! Override from FrameOstream; just calls setSavePath()
  virtual void setConfigInfo(const std::string& path);

  //! Push a data object for asynchronous saving to disk
  /*! This version makes an initial copy of the data, so that it can
      be destroyed immediately upon return. */
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo = defaultInfo);

  //! Specify a comma-separated list of output prefixes
  /*! The string should contain a comma-separated list of filename
      stems for where to save files; e.g., 'dir0/pfx0,dir1/pfx1' will
      save even-numbered files with the prefix 'dir0/pfx0' and will
      save odd-numbered files with the prefix 'dir1/pfx1'
  */
  void setSavePath(const std::string& path);

  //! Reset the in and out frame numbers for shortname to 0
  virtual void closeStream(const std::string& shortname);

protected:
  //! start our worker thread
  virtual void start2();

  //! join our worker thread
  virtual void stop1();

private:
  OModelParam<bool>        itsUseMmap;     //!< Whether to use mmap for writing files
  OModelParam<uint>        itsNumThreads;  //!< How many worker threads to use
  OModelParam<uint>        itsSleepUsecs;  //!< How many usecs to sleep after each write
  OModelParam<uint>        itsSavePeriod;  //!< How often to save frames (default = every frame)
  OModelParam<std::string> itsSavePath;    //!< Where to save frames

  std::vector<std::string> itsFileStems;   //!< Tokenized version of itsSavePath

  WorkThreadServer*        itsServer;      //!< Our thread server

  struct Impl;
  Impl* const rep;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
