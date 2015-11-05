/*!@file Raster/YuvParser.C Parse raw YUV image files. */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Rob Peters <rjpeters@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/YuvParser.C $
// $Id: YuvParser.C 8897 2007-10-24 22:22:40Z rjpeters $
//

#include "Raster/YuvParser.H"

#include "Raster/GenericFrame.H"
#include "Util/FileUtil.H"
#include "Util/StringConversions.H"
#include "Video/VideoFrame.H"
#include "rutz/bzip2stream.h"
#include "rutz/error_context.h"
#include "rutz/gzstreambuf.h"
#include "rutz/sfmt.h"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"

// ######################################################################
namespace dummy_namespace_to_avoid_gcc411_bug_YuvParser_C
{
  Dims defaultDims(640, 480);
  bool strictLength = true;

  enum CompType
    {
      COMP_NONE,
      COMP_GZIP,
      COMP_BZIP2
    };

  struct VideoFileInfo
  {
    VideoFormat format;
    Dims dims;
    CompType ctype;
    bool beStrict;
  };

  VideoFileInfo getVideoFileInfoFromFilename(const std::string& fname)
  {
    VideoFileInfo result;

    std::string base;
    std::string ext = nodotExtension(fname, &base);

    LDEBUG("ext is %s", ext.c_str());

    if (ext.compare("gz") == 0)
      {
        ext = nodotExtension(base, &base);
        LDEBUG("new ext is %s", ext.c_str());
        result.ctype = COMP_GZIP;
      }
    else if (ext.compare("bz2") == 0)
      {
        ext = nodotExtension(base, &base);
        LDEBUG("new ext is %s", ext.c_str());
        result.ctype = COMP_BZIP2;
      }
    else
      {
        result.ctype = COMP_NONE;
      }

    const std::string dimsstr = nodotExtension(base);

    LDEBUG("dimsstr is '%s'", dimsstr.c_str());

    if (dimsstr.size() == 0)
      {
        LERROR("no <width>x<height> specification found in '%s'; "
               "assuming default dims of %dx%d instead",
               fname.c_str(), defaultDims.w(), defaultDims.h());
        result.dims = defaultDims;

        // we didn't get explicit dims, so let's be picky about the
        // file size matching the defaultDims (--yuv-dims), unless the
        // user also requests loose matching (--yuv-dims-loose)
        result.beStrict = strictLength;
      }
    else
      {
        result.dims = fromStr<Dims>(dimsstr);
        LDEBUG("parsed dims as %dx%d",
               result.dims.w(), result.dims.h());

        // OK, the user gave us some explicit dims, so let's not be
        // picky about whether the file size matches the
        // dims+pixformat
        result.beStrict = false;
      }

    result.format = fromStr<VideoFormat>(ext);

    return result;
  }
}

using namespace dummy_namespace_to_avoid_gcc411_bug_YuvParser_C;

// ######################################################################
struct YuvParser::Rep
{
  Rep(const char* fname_) :
    fname(fname_),
    info(getVideoFileInfoFromFilename(fname))
  {}

  const std::string fname;
  const VideoFileInfo info;
};

// ######################################################################
YuvParser::YuvParser(const char* fname) :
  rep(new Rep(fname))
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
YuvParser::~YuvParser()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  delete rep;
}

// ######################################################################
void YuvParser::setDefaultDims(const Dims& d)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  defaultDims = d;
}

// ######################################################################
Dims YuvParser::getDefaultDims()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return defaultDims;
}

// ######################################################################
void YuvParser::setStrictDims(bool v)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  strictLength = v;
}

// ######################################################################
bool getStrictDims()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return strictLength;
}

// ######################################################################
GenericFrameSpec YuvParser::getFrameSpec() const
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::VIDEO;
  result.videoFormat = rep->info.format;
  result.videoByteSwap = false;
  result.dims = rep->info.dims;
  result.floatFlags = 0;

  return result;
}

// ######################################################################
std::string YuvParser::getComments() const
{ return ""; /* no comments in raw YUV files */ }

// ######################################################################
uint YuvParser::getTagCount() const
{ return 0; }

// ######################################################################
bool YuvParser::getTag(uint tag, std::string &name, std::string &value) const
{ return false; }

// ######################################################################
GenericFrame YuvParser::getFrame()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const bool byteswap = false;

  const VideoFormat vf = rep->info.format;

  if (rep->info.ctype == COMP_GZIP)
    {
      GVX_ERR_CONTEXT(rutz::sfmt("reading gzipped video file '%s'",
                                 rep->fname.c_str()));

      rutz::shared_ptr<std::istream> strm =
        rutz::igzopen(rep->fname.c_str());

      return GenericFrame
        (VideoFrame::fromStream(*strm, rep->info.dims, vf, byteswap));
    }

  else if (rep->info.ctype == COMP_BZIP2)
    {
      GVX_ERR_CONTEXT(rutz::sfmt("reading bzip2-compressed "
                                 "video file '%s'",
                                 rep->fname.c_str()));

      rutz::shared_ptr<std::istream> strm =
        rutz::ibzip2open(rep->fname.c_str());

      return GenericFrame
        (VideoFrame::fromStream(*strm, rep->info.dims, vf, byteswap));
    }

  // else...
  GVX_ERR_CONTEXT(rutz::sfmt("reading raw video file '%s'",
                             rep->fname.c_str()));

  return GenericFrame
    (VideoFrame::fromFile(rep->fname.c_str(), rep->info.dims,
                          vf, byteswap, rep->info.beStrict));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
