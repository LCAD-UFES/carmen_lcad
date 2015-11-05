/*!@file Media/MrawvEncoder.C Low-level encoder for multi-frame raw video formats */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MrawvEncoder.C $
// $Id: MrawvEncoder.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef MEDIA_MRAWVENCODER_C_DEFINED
#define MEDIA_MRAWVENCODER_C_DEFINED

#include <string>

#include "Media/MrawvEncoder.H"

#include "Image/Normalize.H"
#include "Raster/GenericFrame.H"
#include "Util/StringUtil.H"
#include "Util/sformat.H"

#include <sys/types.h>
#include <sys/stat.h>  // for open()
#include <fcntl.h>

// ######################################################################
namespace
{
  std::string extensionFor(const Dims& dims, VideoFormat mode)
  {
    return sformat(".%dx%d.m%s",
                   dims.w(), dims.h(),
                   toLowerCase(convertToString(mode)).c_str());
  }

  std::string addExtension(const std::string& fstem,
                           const Dims& dims, VideoFormat mode)
  {
    const std::string ext = extensionFor(dims, mode);

    std::string result(fstem);

    if (result.size() < ext.size() ||
        result.compare(result.size() - ext.size(), result.npos,
                       ext) != 0)
      result += ext;

    ASSERT(result.compare(result.size() - ext.size(), result.npos,
                          ext) == 0);

    return result;
  }
}

// ######################################################################
MrawvEncoder::MrawvEncoder(const GenericFrameSpec& spec,
                           const std::string& fstem,
                           const bool scale255)
  :
  itsFname(),
  itsFile(0),
  itsScale255(scale255)
{
  itsFname =
    addExtension(fstem, spec.dims, spec.getActualVideoFormat());

#ifdef HAVE_O_LARGEFILE
  // in order to handle large files (>2GiB) on 32-bit linux systems,
  // we need to use the os-level open() call rather than fopen(), so
  // that we can pass in the O_LARGEFILE flag; the resulting file
  // descriptor is passed to fdopen() to leave us with a normal FILE*
  const int fd = open(itsFname.c_str(), O_WRONLY | O_CREAT | O_LARGEFILE,
                      (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP
                       | S_IROTH | S_IWOTH));

  if (fd == -1)
    PLFATAL("Cannot open() %s for writing", itsFname.c_str());

  itsFile = fdopen(fd, "w");

  if (itsFile == 0)
    PLFATAL("Cannot fdopen() %s for writing", itsFname.c_str());

#else // !defined(HAVE_O_LARGEFILE)

  itsFile = fopen(itsFname.c_str(), "w");

  if (itsFile == 0)
    PLFATAL("Cannot fopen() %s for writing", itsFname.c_str());
#endif
}

// ######################################################################
MrawvEncoder::~MrawvEncoder()
{
  this->close();
}

// ######################################################################
int MrawvEncoder::close()
{
  int err = 0;

  if (itsFile)
    {
      err = fclose(itsFile);
      if (err)
        PLERROR("Error closing file %s", itsFname.c_str());

      itsFile = 0;
    }

  return err;
}

// ######################################################################
void MrawvEncoder::writeFrame(const GenericFrame& f_)
{
  GenericFrame f = f_;
  if (itsScale255)
    f.setFloatFlags(f.floatFlags() | FLOAT_NORM_0_255);

  const VideoFrame vf = f.asVideo();

  const size_t siz = vf.getBufSize();

  const size_t n = fwrite(vf.getBuffer(), 1, siz, itsFile);

  LDEBUG("wrote %" ZU "/%" ZU " bytes to %s", n, siz, itsFname.c_str());

  if (n != siz)
    PLFATAL("fwrite() failed for %s", itsFname.c_str());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MRAWVENCODER_C_DEFINED
