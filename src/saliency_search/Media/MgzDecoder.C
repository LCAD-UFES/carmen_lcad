/*!@file Media/MgzDecoder.C Low-level decoder for multi-frame "mgz" file format */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MgzDecoder.C $
// $Id: MgzDecoder.C 15099 2011-11-30 01:19:51Z dberg $
//

#ifndef MEDIA_MGZDECODER_C_DEFINED
#define MEDIA_MGZDECODER_C_DEFINED

#include "Media/MgzDecoder.H"
#include <sys/errno.h>

// ######################################################################
MgzDecoder::MgzDecoder(const std::string& fname)
  :
  itsFileName(fname)
{
  // open the file:
  itsFile = gzopen(fname.c_str(), "rb");

  if (itsFile == NULL)
    {
      if (errno) PLFATAL("Could not open '%s' for reading", fname.c_str());
      else LFATAL("No memory to open '%s' for reading", fname.c_str());
    }
}

// ######################################################################
MgzDecoder::~MgzDecoder()
{
  if (itsFile)
    {
      int err = gzclose(itsFile);
      if (err == Z_ERRNO)
        PLFATAL("Error closing file");
      else if (err)
        LFATAL("Error closing file: %s", gzerror(itsFile, &err));
    }
}

// ######################################################################
GenericFrame MgzDecoder::readFrame()
{
  if (itsFile == NULL) LFATAL("No file open for reading");

  // NOTE: This is not portable!
  // read the dims:
  uint32 wht[3];
  int nr = gzread(itsFile, wht, 3 * sizeof(uint32));
  if (nr == 0 && gzeof(itsFile))
    return GenericFrame(); // input exhausted
  else if (nr == -1)
    {
      if (errno == 0) return GenericFrame(); // undocumented EOF condition?
      else PLFATAL("Error reading image header from file");
    }
  else if (nr != 3 * sizeof(uint32))
    LFATAL("Short read on image header");

  const Dims dims(wht[0], wht[1]);
  const GenericFrame::NativeType typ = GenericFrame::NativeType(wht[2]);
  const size_t siz = dims.sz();

  GenericFrame frame;

  switch(typ)
    {
    case GenericFrame::GRAY_F32:
      {
        // read the flags:
        int32 flags;
        if (gzread(itsFile, &flags, sizeof(int32)) != int(sizeof(int32)))
          PLFATAL("Error reading image flags from file");

        // read the image:
        Image<float> f(dims, NO_INIT);
        const size_t s = siz * sizeof(float);
        if (gzread(itsFile, f.getArrayPtr(), s) != int(s))
          PLFATAL("Error reading image data from file");
        frame = GenericFrame(f, flags);
      }
      break;

    case GenericFrame::GRAY_U8:
      {
        Image<byte> f(dims, NO_INIT);
        const size_t s = siz * sizeof(byte);
        if (gzread(itsFile, f.getArrayPtr(), s) !=  int(s))
          PLFATAL("Error reading image data from file");
        frame = GenericFrame(f);
      }
      break;

    case GenericFrame::RGB_U8:
      {
        Image< PixRGB<byte> > f(dims, NO_INIT);
        const size_t s = siz * 3 * sizeof(byte);
        if (gzread(itsFile, f.getArrayPtr(), s) != int(s))
          PLFATAL("Error reading image data from file");
        frame = GenericFrame(f);
      }
      break;

    case GenericFrame::RGB_F32:
      {
        // read the flags:
        int32 flags;
        if (gzread(itsFile, &flags, sizeof(int32)) != int(sizeof(int32)))
          PLFATAL("Error reading image flags from file");

        // read the image:
        Image< PixRGB<float> > f(dims, NO_INIT);
        const size_t s = siz * sizeof(PixRGB<float>);
        if (gzread(itsFile, f.getArrayPtr(), s) != int(s))
          PLFATAL("Error reading image data from file");
        frame = GenericFrame(f, flags);
      }
      break;

    case GenericFrame::VIDEO:
      {
        int32 vidformat;
        if (gzread(itsFile, &vidformat, sizeof(vidformat)) !=
            int(sizeof(vidformat)))
          PLFATAL("Error reading video format from file");

        if (vidformat < 0 || vidformat > VIDFMT_AUTO)
          PLFATAL("Invalid VideoFormat value %d in mgz file %s",
                  int(vidformat), itsFileName.c_str());

        int32 byteswap;
        if (gzread(itsFile, &byteswap, sizeof(byteswap)) !=
            int(sizeof(byteswap)))
          PLFATAL("Error reading byteswap from file");

        const size_t s = getFrameSize(VideoFormat(vidformat), dims);

        ArrayHandle<byte> f(new ArrayData<byte>(Dims(s,1), NO_INIT));
        if (gzread(itsFile, f.uniq().dataw(), s) != int(s))
          PLFATAL("Error reading video data from file");

        frame = GenericFrame(VideoFrame(f, dims,
                                        VideoFormat(vidformat),
                                        bool(byteswap)));
      }
      break;

    case GenericFrame::NONE:
      // nothing to read here, just leave the GenericFrame as empty
      break;

    default:
      LFATAL("Cannot read frames of type %d", int(typ));
    }

  return frame;
}

// ######################################################################
bool MgzDecoder::skipFrame()
{
  if (itsFile == NULL) LFATAL("No file open for reading");
  
  // NOTE: This is not portable!
  // read the dims:
  uint32 wht[3];
  int nr = gzread(itsFile, wht, 3 * sizeof(uint32));
  if (nr == 0 && gzeof(itsFile))
    return false;
  else if (nr == -1)
    {
      if (errno == 0) return false; // undocumented EOF condition?
      else PLFATAL("Error reading image header from file");
    }
  else if (nr != 3 * sizeof(uint32))
    LFATAL("Short read on image header");

  const Dims dims(wht[0], wht[1]);
  const GenericFrame::NativeType typ = GenericFrame::NativeType(wht[2]);
  const size_t siz = dims.sz();

  switch(typ)
    {
    case GenericFrame::GRAY_F32:
      {
        const size_t s = sizeof(int32) + siz * sizeof(float);
        if (gzseek(itsFile, s, SEEK_CUR) < 0)
          PLFATAL("Error skipping past image data from file");
      }
      break;

    case GenericFrame::GRAY_U8:
      {
        const size_t s = siz * sizeof(byte);
        if (gzseek(itsFile, s, SEEK_CUR) < 0)
          PLFATAL("Error skipping past image data from file");
      }
      break;

    case GenericFrame::RGB_U8:
      {
        const size_t s = siz * 3 * sizeof(byte);
        if (gzseek(itsFile, s, SEEK_CUR) < 0)
          PLFATAL("Error skipping past image data from file");
      }
      break;

    case GenericFrame::RGB_F32:
      {
        const size_t s = sizeof(int32) + siz * sizeof(PixRGB<float>);
        if (gzseek(itsFile, s, SEEK_CUR) < 0)
          PLFATAL("Error skipping past image data from file");
      }
      break;

    case GenericFrame::VIDEO:
      {
        int32 vidformat;
        if (gzread(itsFile, &vidformat, sizeof(vidformat)) !=
            int(sizeof(vidformat)))
          PLFATAL("Error reading video format from file");
        
        if (vidformat < 0 || vidformat > VIDFMT_AUTO)
          PLFATAL("Invalid VideoFormat value %d in mgz file %s",
                  int(vidformat), itsFileName.c_str());

        const size_t s = sizeof(int32) + getFrameSize(VideoFormat(vidformat), dims);
        if (gzseek(itsFile, s, SEEK_CUR) < 0)
          PLFATAL("Error skipping past image data from file");
      }
      break;

    case GenericFrame::NONE:
      // nothing to read here, just leave the GenericFrame as empty
      break;

    default:
      LFATAL("Cannot skip frames of type %d", int(typ));
    }
  return true;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MGZDECODER_C_DEFINED
