/*!@file Media/UcbMpegEncoder.H Thin c++ wrapper around mpeg_encode/ppmtompeg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/UcbMpegEncoder.H $
// $Id: UcbMpegEncoder.H 8903 2007-10-25 22:56:57Z rjpeters $
//

#ifndef MEDIA_UCBMPEGENCODER_H_DEFINED
#define MEDIA_UCBMPEGENCODER_H_DEFINED

#include "Image/Dims.H"
#include "Transport/LowLevelEncoder.H"
#include "Video/VideoFormat.H"

#include <string>

class VideoFrame;

namespace rutz { class bidir_pipe; }

struct UcbMpegParams
{
  /// Get standard default encoding params
  static UcbMpegParams basic();

  /// Get high-quality encoding params
  static UcbMpegParams hq();

  /// Get super-high-quality encoding params
  static UcbMpegParams superhq();

  const char* PATTERN; //!< e.g. "IBBPBBPBBPBBPBB"
  int         GOP_SIZE; //!< e.g. 30; group-of-pictures; multiple of PATTERN length
  int         SLICES_PER_FRAME; //!< e.g. 1; must divide evenly into SIZE.h()
  const char* PIXEL; //!< either "HALF" or "FULL"
  int         RANGE; //!< e.g. 10; search radius
  const char* PSEARCH_ALG; //!< ["LOGARITHMIC"], "EXHAUSTIVE", "TWOLEVEL", "SUBSAMPLE"
  const char* BSEARCH_ALG; //!< ["CROSS2"], "SIMPLE", "EXHAUSTIVE"
  int         IQSCALE; //!< quality scale for I-frames; best quality == 1
  int         PQSCALE; //!< quality scale for P-frames; best quality == 1
  int         BQSCALE; //!< quality scale for B-frames; best quality == 1
  const char* REFERENCE_FRAME; //!< ["DECODED"], "ORIGINAL"
};

/// Thin c++ wrapper around mpeg_encode/ppmtompeg
/** The implementation opens a pipe to an mpeg_encode or ppmtompeg
    subprocess, using the "INPUT_DIR stdin" configuration option, and
    sends frames over the pipe to be encoded into the resulting mpeg
    file.

    Ucb stands for "University of California-Berkeley", where
    mpeg_encode was originally written.
*/
class UcbMpegEncoder : public LowLevelEncoder
{
public:
  /// Construct an encoder
  /** @param exename full path to the mpeg_encode or ppmtompeg
      executable */
  UcbMpegEncoder(const std::string& exename,
                 const std::string& outname,
                 const UcbMpegParams& params,
                 const double framerate = 29.97 /* Hz */);

  ~UcbMpegEncoder();

  virtual int close();

  void writeVideoFrame(const VideoFrame& f);

  virtual void writeFrame(const GenericFrame& f);

private:
  UcbMpegEncoder(const UcbMpegEncoder&); // not implemented
  UcbMpegEncoder& operator=(const UcbMpegEncoder&); // not implemented

  void makeParmsFile(const VideoFrame& f);

  std::string itsExeName;
  std::string itsOutFname;
  UcbMpegParams itsParams;
  std::string itsParamFname;
  double itsFrameRate; //!< 23.976, 24, 25, 29.97, 30, 50, 59.94, 60 Hz
  VideoFormat itsVideoFormat;
  rutz::bidir_pipe* itsSubprocess;
  Dims itsDims;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_UCBMPEGENCODER_H_DEFINED
