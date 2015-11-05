/*!@file Video/VideoFormatCoercion.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/VideoFormatCoercion.H $
// $Id: VideoFormatCoercion.H 9248 2008-02-09 00:29:28Z rjpeters $
//

#ifndef VIDEO_VIDEOFORMATCOERCION_H_DEFINED
#define VIDEO_VIDEOFORMATCOERCION_H_DEFINED

#include "Util/Types.H"
#include "Video/VideoFormat.H"

#include <deque>

class Dims;
class VideoFrame;

struct VideoFormatConverter
{
  typedef VideoFrame Func(const byte* data, const size_t length,
                          const Dims& dims, const bool byteswap);

  VideoFormatConverter(Func* f, unsigned int w,
                       VideoFormat s, VideoFormat d,
                       const char* fn);

  VideoFrame apply(const VideoFrame& in) const;

  Func* func;
  unsigned int weight;
  VideoFormat src;
  VideoFormat dst;
  std::string fname;
};

class VideoFormatCoercion
{
public:
  VideoFormatCoercion();

  VideoFormatCoercion(unsigned int w, VideoFormat s, VideoFormat d,
                      VideoFormatConverter::Func* f, const char* fname);

  std::string describe() const;

  VideoFrame apply(const VideoFrame& src) const;

  bool isDirect() const { return nodes.size() == 1; }

  std::deque<VideoFormatConverter> nodes;
};

/// Find a direct or indirect converter for a pair of VideoFormat values
const VideoFormatCoercion& findConverter(const VideoFormat srcformat,
                                         const VideoFormat dstformat);

/// Coerce a VideoFrame into a different VideoFormat
VideoFrame coerceVideoFormat(const VideoFrame& src,
                             const VideoFormat dstformat);

/// Print a quick summary table of available coercions and their path-lengths to stderr
void printCoercionTable();

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // VIDEO_VIDEOFORMATCOERCION_H_DEFINED
