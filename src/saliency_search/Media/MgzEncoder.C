/*!@file Media/MgzEncoder.C Low-level class to decode mgz files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MgzEncoder.C $
// $Id: MgzEncoder.C 8906 2007-10-25 23:30:51Z rjpeters $
//

#ifndef MEDIA_MGZENCODER_C_DEFINED
#define MEDIA_MGZENCODER_C_DEFINED

#include "Media/MgzEncoder.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

#include <sys/errno.h>

// ######################################################################
MgzEncoder::MgzEncoder(const std::string& fname, const int complev)
{
  // build the open mode, including compression level:
  if (complev < 1 || complev > 9)
    LFATAL("Invalid compression level %d: must be in [1..9]", complev);
  const std::string m = sformat("wb%d", complev);

  // open the file for writing:
  itsFile = gzopen(fname.c_str(), m.c_str());

  if (itsFile == NULL)
    {
      if (errno) PLFATAL("Could not open '%s' for writing", fname.c_str());
      else LFATAL("No memory to open '%s' for writing", fname.c_str());
    }
}

// ######################################################################
MgzEncoder::~MgzEncoder()
{
  this->close();
}

// ######################################################################
int MgzEncoder::close()
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  int err = 0;

  if (itsFile)
    {
      //LDEBUG("Closing and flushing output buffers...");
      err = gzclose(itsFile);
      if (err == Z_ERRNO)
        PLERROR("Error closing file");
      else if (err)
        LERROR("Error closing file: %s", gzerror(itsFile, &err));

      itsFile = NULL;
    }

  return err;
}

// ######################################################################
void MgzEncoder::writeFrame(const GenericFrame& frame)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsFile == NULL) LFATAL("No file open for writing");
  const size_t siz = frame.getDims().sz();

  // NOTE: This is not portable!
  uint32 wht[3];
  wht[0] = frame.getDims().w(); wht[1] = frame.getDims().h();
  wht[2] = uint32(frame.nativeType());

  // write the dims and native type:
  if (gzwrite(itsFile, wht, 3 * sizeof(uint32)) != 3 * sizeof(uint32))
    PLFATAL("Error writing to file");

  switch(frame.nativeType())
    {

    case GenericFrame::GRAY_F32:
      {
        // first write the float flags:
        int32 flags = int32(frame.floatFlags());
        if (gzwrite(itsFile, &flags, sizeof(int32)) != int(sizeof(int32)))
          PLFATAL("Error writing to file");

        // now the image data:
        Image<float> f = frame.asFloat();
        const size_t s = siz * sizeof(float);
        if (gzwrite(itsFile, f.getArrayPtr(), s) != int(s))
          PLFATAL("Error writing to file");
      }
      break;

    case GenericFrame::GRAY_U8:
      {
        Image<byte> f = frame.asGray();
        const size_t s = siz * sizeof(byte);
        if (gzwrite(itsFile, f.getArrayPtr(), s) !=  int(s))
          PLFATAL("Error writing to file");
      }
      break;

    case GenericFrame::RGB_U8:
      {
        Image< PixRGB<byte> > f = frame.asRgb();
        const size_t s = siz * 3 * sizeof(byte);
        if (gzwrite(itsFile, f.getArrayPtr(), s) != int(s))
          PLFATAL("Error writing to file");
      }
      break;

    case GenericFrame::RGB_F32:
      {
        // first write the float flags:
        int32 flags = int32(frame.floatFlags());
        if (gzwrite(itsFile, &flags, sizeof(int32)) != int(sizeof(int32)))
          PLFATAL("Error writing to file");

        // now the image data:
        Image< PixRGB<float> > f = frame.asRgbF32();
        const size_t s = siz * sizeof(PixRGB<float>);
        if (gzwrite(itsFile, f.getArrayPtr(), s) != int(s))
          PLFATAL("Error writing to file");
      }
      break;

    case GenericFrame::VIDEO:
      {
        const VideoFrame f = frame.asVideo();

        int32 vidformat = int32(f.getMode());
        if (gzwrite(itsFile, &vidformat, sizeof(vidformat)) !=
            int(sizeof(vidformat)))
          PLFATAL("Error writing video format to file");

        int32 byteswap = int32(f.getByteSwap());
        if (gzwrite(itsFile, &byteswap, sizeof(byteswap)) !=
            int(sizeof(byteswap)))
          PLFATAL("Error writing byteswap to file");

        if (gzwrite(itsFile, (void*)(f.getBuffer()), f.getBufSize()) !=
            int(f.getBufSize()))
          PLFATAL("Error writing video data to file");
      }
      break;

    case GenericFrame::NONE:
      // nothing to do herexo
      break;

    default:
      LFATAL("Cannot write frames of type %s", frame.nativeTypeName().c_str());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MGZENCODER_C_DEFINED
