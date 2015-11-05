/*!@file Media/MrawvDecoder.C decoder for various raw video formats */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MrawvDecoder.C $
// $Id: MrawvDecoder.C 9108 2007-12-30 06:14:30Z rjpeters $
//

#include "Media/MrawvDecoder.H"
#include "Util/FileUtil.H"
#include "Util/StringConversions.H"
#include "Video/VideoFrame.H"
#include "rutz/bzip2stream.h"
#include "rutz/error_context.h"
#include "rutz/gzstreambuf.h"
#include "rutz/sfmt.h"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"
#include <fstream>

// ######################################################################
namespace
{
  enum CompType
    {
      COMP_NONE,
      COMP_GZIP,
      COMP_BZIP2
    };
}

// ######################################################################
struct MrawvDecoder::Rep
{
  Rep(const std::string& fname_) :
    fname(fname_), dims(), ct(COMP_NONE), vf(VIDFMT_AUTO), strm()
  {}

  const std::string fname;
  Dims dims;
  CompType ct;
  VideoFormat vf;
  rutz::shared_ptr<std::istream> strm;
};

// ######################################################################
MrawvDecoder::MrawvDecoder(const std::string& fname)
  :
  rep(new Rep(fname))
{
  // decode the extension and open the stream:
  std::string base;
  std::string ext = nodotExtension(fname, &base);

  if (ext.compare("gz") == 0)
    {
      ext = nodotExtension(base, &base);
      rep->ct = COMP_GZIP;
      rep->strm = rutz::igzopen(fname.c_str());
    }
  else if (ext.compare("bz2") == 0)
    {
      ext = nodotExtension(base, &base);
      rep->ct = COMP_BZIP2;
      rep->strm = rutz::ibzip2open(fname.c_str());
    }
  else
    {
      std::ifstream *ifs = new std::ifstream(fname.c_str());
      if (ifs->is_open() == false)
        {
          delete ifs;
          LFATAL("Could not open '%s' for reading", fname.c_str());
        }
      rep->strm.reset(ifs);
    }

  const std::string dimsstr = nodotExtension(base);

  LDEBUG("dimsstr is '%s'", dimsstr.c_str());

  if (dimsstr.size() == 0)
    LFATAL("expected a filename like stem.WWWxHHH.ext[.gz|.bz2], "
           "but got '%s' (missing WWWxHHH dims?)", fname.c_str());

  rep->dims = fromStr<Dims>(dimsstr);
  LDEBUG("parsed dims as %dx%d", rep->dims.w(), rep->dims.h());

  if (rep->dims.isEmpty())
    LFATAL("expected non-empty dims in filename '%s', but got %dx%d",
           fname.c_str(), rep->dims.w(), rep->dims.h());

  // strip first char of extension:
  if (ext[0] != 'm')
    LFATAL("Invalid extension of '%s', must start with 'm'.", fname.c_str());

  // get the video format:
  rep->vf = fromStr<VideoFormat>(ext.erase(0,1));
}

// ######################################################################
MrawvDecoder::~MrawvDecoder()
{
  delete rep;
}

// ######################################################################
GenericFrame MrawvDecoder::readFrame()
{
  const bool byteswap = false; // not handled for now...

  if (rep->ct == COMP_GZIP)
    GVX_ERR_CONTEXT(rutz::sfmt("reading gzipped video frame from file '%s'",
                               rep->fname.c_str()));
  else if (rep->ct == COMP_BZIP2)
    GVX_ERR_CONTEXT(rutz::sfmt("reading bzip2-compressed "
                               "video frame from file '%s'",
                               rep->fname.c_str()));
  else
    GVX_ERR_CONTEXT(rutz::sfmt("reading raw video frame from file '%s'",
                               rep->fname.c_str()));

  return GenericFrame
    (VideoFrame::fromStream(*(rep->strm), rep->dims, rep->vf,
                            byteswap, false)); // fail is fatal here, just EOF
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
