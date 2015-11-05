/*!@file Media/MgzInputStream.H Read frames from mgz files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MgzInputStream.H $
// $Id: MgzInputStream.H 8301 2007-04-25 18:31:34Z rjpeters $
//

#ifndef MEDIA_MGZINPUTSTREAM_H_DEFINED
#define MEDIA_MGZINPUTSTREAM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Transport/FrameIstream.H"
#include "Raster/GenericFrame.H"

class MgzDecoder;

// ######################################################################
// ######################################################################
//! MgzInputStream is a wrapper class to read frames from .mgz files
/*! mgz is a custom format that basically contains the raw frames in
  gzipped format. One mgz file can contain many frames of various
  sizes and types. */

class MgzInputStream : public FrameIstream
{
public:
  //! Construct an mgzstream object for mgz input
  MgzInputStream(OptionManager& mgr,
                 const std::string& descrName = "Mgz Input Stream",
                 const std::string& tagName = "MgzInputStream");

  //! Virtual destructor
  virtual ~MgzInputStream();

  //! Override from FrameIstream; just calls setFileName()
  virtual void setConfigInfo(const std::string& filename);

  //! get frame specifications, peeking at the first frame if necessary
  virtual GenericFrameSpec peekFrameSpec();

  //! Configure the MGZ Stream for decoding
  /*! @param fname name of file from which frames are to be read from */
  void setFileName(std::string fname);

  //! Read the next frame from the mgz stream
  /*! The native representation of the GenericFrame will be the same
    as what it was when the frame was stored into the stream. */
  virtual GenericFrame readFrame();

private:
  MgzDecoder *itsDecoder;
  GenericFrame itsFrame; // used for peeking of frame specs
  GenericFrameSpec itsFrameSpec;
  bool itsFrameSpecValid;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MGZINPUTSTREAM_H_DEFINED
