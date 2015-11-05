/*!@file Media/MpegInputStream.H Read frames from movie files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MpegInputStream.H $
// $Id: MpegInputStream.H 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef MEDIA_MPEGINPUTSTREAM_H_DEFINED
#define MEDIA_MPEGINPUTSTREAM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Transport/FrameIstream.H"
#include "Util/Types.H"      // for byte

class MovieDecoder;
class VideoFrame;

// ######################################################################
// ######################################################################
//! InputMPEGStream is a wrapper class to read video frames from mpeg-1
/*! InputMPEGStream is a wrapper class to read video frames from
    mpeg-1, mpeg-4 (aka divx), asf, avi, mov and any other format that
    mplayer can decode. Audio streams are entirely ignored. */

class InputMPEGStream : public FrameIstream
{
public:
  //! Construct an mpegstream object for mpeg input
  InputMPEGStream(OptionManager& mgr,
                  const std::string& descrName = "Input MPEG Stream",
                  const std::string& tagName = "InputMPEGStream");

  //! Virtual destructor
  virtual ~InputMPEGStream();

  //! Override from FrameIstream; just calls setFileName()
  virtual void setConfigInfo(const std::string& filename);

  //! override the base class version so we can seek to the proper frame number
  virtual bool setFrameNumber(int n);

  //! get frame specifications, peeking at the first frame if necessary
  virtual GenericFrameSpec peekFrameSpec();

  //! Configure the MPEG Stream for decoding
  /*! @param fname name of file from which frames are to be read from */
  void setFileName(std::string fname);

  //! Read the next frame from the mpeg stream
  /*! The native representation of the GenericFrame will be a
      VideoFrame. */
  virtual GenericFrame readFrame();

  //! Read the next frame from the mpeg stream
  /*! returns an image of size (0,0) at the end of the stream */
  virtual Image<PixRGB<byte> > readRGB();

  /// Read a frame from the stream and discard it
  /** If you know you are going to discard the frame (e.g. to skip
      ahead to a certain frame number, or to count the frame), then it
      is more efficient to call readAndDiscardFrame() than to call
      readVideoFrame() or readRGB() but ignore the result. */
  virtual bool readAndDiscardFrame();

  /// Get the next raw VideoFrame from the stream
  VideoFrame readVideoFrame();

protected:
  //! get stopped; close any open write stream
  void stop2();

  NModelParam<int>         itsBuffLen;
  OModelParam<std::string> itsCodecName;
  OModelParam<bool>        itsDoPreload;
  std::string              itsFileName;

private:
  //! create our Decoder object if we haven't yet done so
  void createDecoder();

  MovieDecoder* dec;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MPEGINPUTSTREAM_H_DEFINED
