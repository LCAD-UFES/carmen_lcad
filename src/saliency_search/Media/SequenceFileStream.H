/*!@file Media/SequenceFileStream.H Read frames from .seq video files */

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
// Primary maintainer for this file: Rand Voorhies <voorhies at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/SequenceFileStream.H $
// $Id: SequenceFileStream.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef MEDIA_SEQUENCEFILESTREAM_H_DEFINED
#define MEDIA_SEQUENCEFILESTREAM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Transport/FrameIstream.H"
#include "Raster/GenericFrame.H"
#include "rutz/mappedfile.h"

class SequenceFileStream : public FrameIstream
{
public:
  //! Construct a Sequence File Stream object for .seq file input
  SequenceFileStream(OptionManager& mgr,
                     const std::string& descrName = "Sequence File Input Stream",
                     const std::string& tagName = "SeqquenceFileStream");

  //! Virtual destructor
  virtual ~SequenceFileStream();

  //! Override from FrameIstream; just calls setFileName()
  virtual void setConfigInfo(const std::string& filename);

  //! Override from FrameIstream; sets the current frame number in the sequence
  virtual bool setFrameNumber(int n);

  //! get frame specifications, peeking at the first frame if necessary
  virtual GenericFrameSpec peekFrameSpec();

  //! Configure the MGZ Stream for decoding
  /*! @param fname name of file from which frames are to be read from */
  void setFileName(const std::string& fname);

  //! Read the next frame from the seq stream
  /*! The native representation of the GenericFrame will be the same
    as what it was when the frame was stored into the stream. */
  virtual GenericFrame readFrame();

private:
  OModelParam<bool> itsUseMmap;     //!< Whether to use mmap for writing files

  GenericFrame      itsFrame;
  GenericFrameSpec  itsFrameSpec;
  bool              itsFrameSpecValid;
  int               itsFileHandle;

  off_t             itsDataOffset;
  off_t             itsImageSizeBytes;
  off_t             itsTrueImageSize;

  unsigned int      itsWidth;
  unsigned int      itsHeight;
  unsigned int      itsFrameCount;
  unsigned int      itsFrameOffset;
  unsigned int      itsFirstFrame;

  rutz::shared_ptr<rutz::mapped_infile> itsMmapFile;
};

#endif

