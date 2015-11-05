/*!@file Media/UcbMpegEncoder.C Thin c++ wrapper around mpeg_encode/ppmtompeg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/UcbMpegEncoder.C $
// $Id: UcbMpegEncoder.C 8903 2007-10-25 22:56:57Z rjpeters $
//

#ifndef MEDIA_UCBMPEGENCODER_C_DEFINED
#define MEDIA_UCBMPEGENCODER_C_DEFINED

#include "Media/UcbMpegEncoder.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Raster/PnmWriter.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"
#include "rutz/pipe.h"
#include "rutz/trace.h"

#include <fstream>

UcbMpegParams UcbMpegParams::basic()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  UcbMpegParams parms;

  parms.PATTERN = "IBBPBBPBBPBBPBB";
  parms.GOP_SIZE = 30;
  parms.SLICES_PER_FRAME = 1;
  parms.PIXEL = "HALF";
  parms.RANGE = 10;
  parms.PSEARCH_ALG = "LOGARITHMIC";
  parms.BSEARCH_ALG = "CROSS2";
  parms.IQSCALE = 8;
  parms.PQSCALE = 10;
  parms.BQSCALE = 25;
  parms.REFERENCE_FRAME = "ORIGINAL";

  return parms;
}

UcbMpegParams UcbMpegParams::hq()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  UcbMpegParams parms;

  parms.PATTERN = "IBBPBBPBBPBBPBB";
  parms.GOP_SIZE = 30;
  parms.SLICES_PER_FRAME = 1;
  parms.PIXEL = "HALF";
  parms.RANGE = 10;
  parms.PSEARCH_ALG = "LOGARITHMIC";
  parms.BSEARCH_ALG = "CROSS2";
  parms.IQSCALE = 1;
  parms.PQSCALE = 1;
  parms.BQSCALE = 1;
  parms.REFERENCE_FRAME = "DECODED";

  return parms;
}

UcbMpegParams UcbMpegParams::superhq()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  UcbMpegParams parms;

  parms.PATTERN = "I";
  parms.GOP_SIZE = 1;
  parms.SLICES_PER_FRAME = 1;
  parms.PIXEL = "HALF";
  parms.RANGE = 10;
  parms.PSEARCH_ALG = "LOGARITHMIC";
  parms.BSEARCH_ALG = "CROSS2";
  parms.IQSCALE = 1;
  parms.PQSCALE = 1;
  parms.BQSCALE = 1;
  parms.REFERENCE_FRAME = "DECODED";

  return parms;
}

UcbMpegEncoder::UcbMpegEncoder(const std::string& exename,
                               const std::string& outname,
                               const UcbMpegParams& params,
                               const double framerate) :
  itsExeName(exename),
  itsOutFname(outname),
  itsParams(params),
  itsParamFname(),
  itsFrameRate(framerate),
  itsVideoFormat(VIDFMT_AUTO),
  itsSubprocess(0),
  itsDims()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

UcbMpegEncoder::~UcbMpegEncoder()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->close();

  delete itsSubprocess;
}

int UcbMpegEncoder::close()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  int result = 0;

  if (itsSubprocess != 0)
    {
      itsSubprocess->close_out();

      result = itsSubprocess->exit_status();

      LINFO("%s exited with status %d", itsExeName.c_str(), result);

      delete itsSubprocess;
      itsSubprocess = 0;
    }

  if (itsParamFname.length() > 0)
    {
      remove(itsParamFname.c_str());
      itsParamFname = std::string();
    }

  return result;
}

void UcbMpegEncoder::makeParmsFile(const VideoFrame& f)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  static int counter = 0;

  if (f.getDims().w() % 16 != 0)
    LFATAL("%s requires the image width to be a multiple of 16, "
           "but the actual image width was %d",
           itsExeName.c_str(), f.getDims().w());

  if (f.getDims().h() % 16 != 0)
    LFATAL("%s requires the image height to be a multiple of 16, "
           "but the actual image height was %d",
           itsExeName.c_str(), f.getDims().h());

  itsDims = f.getDims();

  itsParamFname = sformat("/tmp/mpegencode-params-%d-%d.txt",
                         int(getpid()), counter++);

  FILE* pf = fopen(itsParamFname.c_str(), "w");

  if (pf == 0)
    LFATAL("couldn't open %s for writing", itsParamFname.c_str());

  fprintf(pf, "PATTERN                 %s\n", itsParams.PATTERN);
  fprintf(pf, "GOP_SIZE                %d\n", itsParams.GOP_SIZE);
  fprintf(pf, "SLICES_PER_FRAME        %d\n", itsParams.SLICES_PER_FRAME);
  fprintf(pf, "PIXEL                   %s\n", itsParams.PIXEL);
  fprintf(pf, "RANGE                   %d\n", itsParams.RANGE);
  fprintf(pf, "PSEARCH_ALG             %s\n", itsParams.PSEARCH_ALG);
  fprintf(pf, "BSEARCH_ALG             %s\n", itsParams.BSEARCH_ALG);
  fprintf(pf, "IQSCALE                 %d\n", itsParams.IQSCALE);
  fprintf(pf, "PQSCALE                 %d\n", itsParams.PQSCALE);
  fprintf(pf, "BQSCALE                 %d\n", itsParams.BQSCALE);
  fprintf(pf, "REFERENCE_FRAME         %s\n", itsParams.REFERENCE_FRAME);

  fprintf(pf, "FORCE_ENCODE_LAST_FRAME 1\n");
  fprintf(pf, "FRAME_RATE              %.2f\n", itsFrameRate);
  fprintf(pf, "OUTPUT                  %s\n", itsOutFname.c_str());
  fprintf(pf, "SIZE                    %dx%d\n", f.getDims().w(), f.getDims().h());

  switch (f.getMode())
    {
    case VIDFMT_UYVY:
    case VIDFMT_YUV422:
      fprintf(pf, "BASE_FILE_FORMAT        YUV\n");
      fprintf(pf, "YUV_FORMAT              ABEKAS\n");
      itsVideoFormat = f.getMode();
      LINFO("writing frames in YUV/ABEKAS format");
      break;

      // NOTE: "YUV_FORMAT PHILLIPS" would be YVYU, but we don't have
      // a VIDFMT code for that layout

    case VIDFMT_YUV420P:
      fprintf(pf, "BASE_FILE_FORMAT        YUV\n");
      fprintf(pf, "YUV_FORMAT              UCB\n");
      itsVideoFormat = f.getMode();
      LINFO("writing frames in YUV/UCB format");
      break;

    default:
      fprintf(pf, "BASE_FILE_FORMAT        PPM\n");
      itsVideoFormat = VIDFMT_AUTO; // just convert through toRgb()
      LINFO("writing frames in PPM format");
      break;
    }

  fprintf(pf, "INPUT_DIR               stdin\n");
  fprintf(pf, "INPUT_CONVERT           *\n");
  fprintf(pf, "INPUT\n");
  fprintf(pf, "END_INPUT\n");

  fclose(pf);

  // now echo the params file back to the user for debugging:
  {
    std::ifstream ifs(itsParamFname.c_str());
    int lineno = 0;
    std::string line;
    while (std::getline(ifs, line))
      {
        LDEBUG("params:%02d: %s", ++lineno, line.c_str());
      }
  }
}

void UcbMpegEncoder::writeVideoFrame(const VideoFrame& f)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsSubprocess == 0)
    {
      this->makeParmsFile(f);
      itsSubprocess = new rutz::bidir_pipe;

      itsSubprocess->block_child_sigint();

      itsSubprocess->init(itsExeName.c_str(),
                          "-realquiet",
                          itsParamFname.c_str(),
                          0);

      LDEBUG("initialized subprocess '%s -realquiet %s'",
             itsExeName.c_str(), itsParamFname.c_str());
    }

  if (f.getDims() != itsDims)
    LFATAL("invalid image dimensions (got %dx%d, expected %dx%d)",
           f.getDims().w(), f.getDims().h(), itsDims.w(), itsDims.h());

  switch (itsVideoFormat)
    {
    case VIDFMT_UYVY:
    case VIDFMT_YUV422:
    case VIDFMT_YUV420P:
      if (f.getMode() != itsVideoFormat)
        LFATAL("input frame in wrong format (expected %s, got %s)",
               convertToString(itsVideoFormat).c_str(),
               convertToString(f.getMode()).c_str());

      itsSubprocess->out_stream()
        .write(reinterpret_cast<const char*>(f.getBuffer()),
               f.getBufSize());

      break;

    default:
      {
        const Image<PixRGB<byte> > rgb = f.toRgb();
        PnmWriter::writeRGB(rgb, itsSubprocess->out_stream());
      }
    }

  if (itsSubprocess->out_stream().fail())
    LFATAL("pipe stream error while sending frame");
}

void UcbMpegEncoder::writeFrame(const GenericFrame& f)
{
  this->writeVideoFrame(f.asVideo());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_UCBMPEGENCODER_C_DEFINED
