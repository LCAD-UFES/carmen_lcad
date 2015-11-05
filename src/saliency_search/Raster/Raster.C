/*!@file Raster/Raster.C Read/write/display raster images */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/Raster.C $
// $Id: Raster.C 15290 2012-05-13 14:06:48Z kai $
//

#include "Raster/Raster.H"

#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Raster/CcodeWriter.H"
#include "Raster/DpxParser.H"
#include "Raster/GenericFrame.H"
#include "Raster/JpegParser.H"
#include "Raster/PfmParser.H"
#include "Raster/PfmWriter.H"
#include "Raster/PfzParser.H"
#include "Raster/PfzWriter.H"
#include "Raster/PlaintextWriter.H"
#include "Raster/PngParser.H"
#include "Raster/PngWriter.H"
#include "Raster/JpegWriter.H"
#include "Raster/PnmParser.H"
#include "Raster/PnmWriter.H"
#include "Raster/QuartzQuickTimeParser.H"
#include "Raster/RawWriter.H"
#include "Raster/YuvParser.H"
#include "Raster/YuvWriter.H"
#include "Util/Assert.H"
#include "Util/FileUtil.H"
#include "Util/log.H"
#include "Video/VideoFrame.H"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"

#include <cerrno>
#include <cstring> // for strcat(), strcasecmp()
#include <fcntl.h>
#include <list>
#include <sys/stat.h>
#include <sys/types.h> // for open()

// ######################################################################
// ########## Raster helper functions:
// ######################################################################
namespace
{
  const int MODE_R = (1 << 0);
  const int MODE_W = (1 << 1);

  // ######################################################################
  // A simple mapping from filename extensions to RasterFileFormat values
  struct RasFileExtension
  {
    const char* extension; // including the dot!
    RasterFileFormat format;
    int modes;
  };

  // ######################################################################
  // List of all valid extensions
  const RasFileExtension allExtensions[] =
    {
      { ".pnm",             RASFMT_PNM, MODE_R | MODE_W },
      { ".ppm",             RASFMT_PNM, MODE_R | MODE_W },
      { ".pgm",             RASFMT_PNM, MODE_R | MODE_W },
      { ".pbm",             RASFMT_PNM, MODE_R | MODE_W },

      { ".pnm.gz",          RASFMT_PNM, MODE_R },
      { ".ppm.gz",          RASFMT_PNM, MODE_R },
      { ".pgm.gz",          RASFMT_PNM, MODE_R },
      { ".pbm.gz",          RASFMT_PNM, MODE_R },

      { ".pnm.bz2",         RASFMT_PNM, MODE_R },
      { ".ppm.bz2",         RASFMT_PNM, MODE_R },
      { ".pgm.bz2",         RASFMT_PNM, MODE_R },
      { ".pbm.bz2",         RASFMT_PNM, MODE_R },

      { ".pfm",             RASFMT_PFM, MODE_R | MODE_W },
      { ".pfm.gz",          RASFMT_PFM, MODE_R | MODE_W },
      { ".pfm.bz2",         RASFMT_PFM, MODE_R | MODE_W },

      { ".png",             RASFMT_PNG, MODE_R | MODE_W },
      { ".pfz",             RASFMT_PFZ, MODE_R | MODE_W },
      { ".jpg",             RASFMT_JPEG, MODE_R | MODE_W},
      { ".jpeg",            RASFMT_JPEG, MODE_R | MODE_W},

      { ".txt",             RASFMT_TXT, MODE_W },
      { ".C",               RASFMT_CCODE, MODE_W },
      { ".dpx",             RASFMT_DPX, MODE_R },

      { ".rgb555",          RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".rgb565",          RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".rgb24",           RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".rgb32",           RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv24",           RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuyv",            RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".uyvy",            RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv444",          RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv422",          RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv411",          RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv420",          RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv410",          RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv444p",         RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv422p",         RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv411p",         RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv420p",         RASFMT_RAW_VIDEO, MODE_R | MODE_W },
      { ".yuv410p",         RASFMT_RAW_VIDEO, MODE_R | MODE_W },

      { ".bayer",           RASFMT_RAW_IMAGE, MODE_R | MODE_W },

      { ".rgb555.gz",       RASFMT_RAW_VIDEO, MODE_R },
      { ".rgb565.gz",       RASFMT_RAW_VIDEO, MODE_R },
      { ".rgb24.gz",        RASFMT_RAW_VIDEO, MODE_R },
      { ".rgb32.gz",        RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv24.gz",        RASFMT_RAW_VIDEO, MODE_R },
      { ".yuyv.gz",         RASFMT_RAW_VIDEO, MODE_R },
      { ".uyvy.gz",         RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv444.gz",       RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv422.gz",       RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv411.gz",       RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv420.gz",       RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv410.gz",       RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv444p.gz",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv422p.gz",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv411p.gz",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv420p.gz",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv410p.gz",      RASFMT_RAW_VIDEO, MODE_R },

      { ".rgb555.bz2",      RASFMT_RAW_VIDEO, MODE_R },
      { ".rgb565.bz2",      RASFMT_RAW_VIDEO, MODE_R },
      { ".rgb24.bz2",       RASFMT_RAW_VIDEO, MODE_R },
      { ".rgb32.bz2",       RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv24.bz2",       RASFMT_RAW_VIDEO, MODE_R },
      { ".yuyv.bz2",        RASFMT_RAW_VIDEO, MODE_R },
      { ".uyvy.bz2",        RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv444.bz2",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv422.bz2",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv411.bz2",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv420.bz2",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv410.bz2",      RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv444p.bz2",     RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv422p.bz2",     RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv411p.bz2",     RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv420p.bz2",     RASFMT_RAW_VIDEO, MODE_R },
      { ".yuv410p.bz2",     RASFMT_RAW_VIDEO, MODE_R },

      // make sure this entry stays last!
      { 0, RASFMT_AUTO, 0 }
    };

  // ######################################################################
  struct RasFileDefaultExtension
  {
    const char* extension; // including the dot!
    RasterFileFormat format;
  };

  // ######################################################################
  // List of the default extensions for each format
  const RasFileDefaultExtension defaultExtensions[] =
    {
      { ".pnm",             RASFMT_PNM },
      { ".png",             RASFMT_PNG },
      { ".pfm",             RASFMT_PFM },
      { ".pfz",             RASFMT_PFZ },
      { ".jpg",             RASFMT_JPEG },
      { ".txt",             RASFMT_TXT },
      { ".C",               RASFMT_CCODE },
      { ".dpx",             RASFMT_DPX },

      // no entries for RASFMT_RAW_VIDEO or RASFMT_RAW_IMAGE, because
      // those formats have indeterminate file extensions

      // make sure this entry stays last!
      { 0, RASFMT_AUTO }
    };

  // ######################################################################
  bool hasMatchingExtension(const std::string& fname,
                            const RasterFileFormat ftype,
                            const int mode)
  {
    for (const RasFileExtension* p = &allExtensions[0];
         p->extension != 0; ++p)
      {
        if (p->format == ftype
            && hasExtension(fname, p->extension)
            && ((p->modes & mode) != 0))
          return true;
      }

    return false;
  }

  // ######################################################################
  const char* defaultExtensionFor(const RasterFileFormat ftype)
  {
    for (const RasFileDefaultExtension* p = &defaultExtensions[0];
         p->extension != 0; ++p)
      {
        if (p->format == ftype)
          return p->extension;
      }

    // no matching entry found, so just return an empty string:
    return "";
  }

  // ######################################################################
  // get a string containing a sorted and comma-separated list of all
  // known raster file extensions
  std::string getExtensionListString(const int mode)
  {
    std::list<std::string> xlist;
    for (const RasFileExtension* p = &allExtensions[0];
         p->extension != 0; ++p)
      if ((p->modes & mode) != 0)
        xlist.push_back(p->extension);

    xlist.sort();

    std::string xstr;
    bool first = true;

    for (std::list<std::string>::iterator
           itr = xlist.begin(), stop = xlist.end();
         itr != stop; ++itr)
      {
        if (!first)
          xstr += ", ";
        xstr += '\'';
        xstr += *itr;
        xstr += '\'';
        first = false;
      }

    return xstr;
  }

  // ######################################################################
  rutz::shared_ptr<RasterParser> makeParser(RasterFileFormat ftype,
                                     const char* fname)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    switch (ftype)
      {
      case RASFMT_PNM: return rutz::shared_ptr<RasterParser>(new PnmParser(fname));
#if defined(INVT_HAVE_LIBPNG)
      case RASFMT_PNG: return rutz::shared_ptr<RasterParser>(new PngParser(fname));
#elif defined(HAVE_QUICKTIME_QUICKTIME_H)
      case RASFMT_PNG: return rutz::shared_ptr<RasterParser>(new QuartzQuickTimeParser(fname));
#endif
      case RASFMT_PFM: return rutz::shared_ptr<RasterParser>(new PfmParser(fname));
      case RASFMT_PFZ: return rutz::shared_ptr<RasterParser>(new PfzParser(fname));
      case RASFMT_RAW_VIDEO: return rutz::shared_ptr<RasterParser>(new YuvParser(fname));
#if defined(INVT_HAVE_LIBJPEG)
      case RASFMT_JPEG: return rutz::shared_ptr<RasterParser>(new JpegParser(fname));
#elif defined(HAVE_QUICKTIME_QUICKTIME_H)
      case RASFMT_JPEG: return rutz::shared_ptr<RasterParser>(new QuartzQuickTimeParser(fname));
#endif
      case RASFMT_DPX: return rutz::shared_ptr<RasterParser>(new DpxParser(fname));
      default:
        {
          LFATAL("Unsupported file type: %s (%d) for reading filename '%s'",
                 convertToString(ftype).c_str(), int(ftype), fname);
        }
      }
    /* can't happen */ return rutz::shared_ptr<RasterParser>();
  }

  // ######################################################################
  rutz::shared_ptr<RasterWriter> makeWriter(RasterFileFormat ftype,
                                     const char* fname)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    switch (ftype)
      {
      case RASFMT_PNM: return rutz::make_shared(new PnmWriter(hasExtension(fname, ".pbm"), 128));
#ifdef INVT_HAVE_LIBPNG
      case RASFMT_PNG: return rutz::make_shared(new PngWriter);
#endif
#if defined(INVT_HAVE_LIBJPEG)
      case RASFMT_JPEG: return rutz::make_shared(new JpegWriter);
#endif
      case RASFMT_PFM: return rutz::make_shared(new PfmWriter);
      case RASFMT_PFZ: return rutz::make_shared(new PfzWriter);
      case RASFMT_RAW_VIDEO: return rutz::make_shared(new YuvWriter);
      case RASFMT_RAW_IMAGE: return rutz::make_shared(new RawWriter);
      case RASFMT_TXT: return rutz::make_shared(new PlaintextWriter);
      case RASFMT_CCODE: return rutz::make_shared(new CcodeWriter);
      default:
        {
          LFATAL("Unsupported file type: %s (%d) for writing filename '%s'",
                 convertToString(ftype).c_str(), int(ftype), fname);
        }
      }
    /* can't happen */ return rutz::shared_ptr<RasterWriter>();
  }

  // ######################################################################
  RasterFileFormat addFileExtension(std::string& fname,
                                    const RasterFileFormat ftype,
                                    const int mode,
                                    const bool fatal)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    // If an automatic file type was requested, then we try to infer
    // the file type from the extension of the file name; otherwise
    // it's an error:
    if (ftype == RASFMT_AUTO)
      {
        for (const RasFileExtension* p = &allExtensions[0];
             p->extension != 0; ++p)
          {
            if (((p->modes & mode) != 0)
                && hasExtension(fname, p->extension))
              return p->format;
          }

        const std::string xlist = getExtensionListString(mode);

        if (fatal)
          LFATAL("couldn't determine RasterFileFormat from filename '%s'"
                 " (known raster file extensions: [%s])",
                 fname.c_str(), xlist.c_str());
        else
          return RASFMT_AUTO;
      }

    if (!hasMatchingExtension(fname, ftype, mode))
      fname += defaultExtensionFor(ftype);

    return ftype;
  }

} // end anonymous namespace


// ######################################################################
// ########## Raster functions:
// ######################################################################


// ######################################################################
bool Raster::fileExists(std::string fname,
                        const RasterFileFormat ftype,
                        const bool chkFormat)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  RasterFileFormat ff = addFileExtension(fname, ftype, MODE_R, /* no fatal errors */ false);
  if (chkFormat && ff == RASFMT_AUTO) //check if we support this file format
    return false;
  const int fd = open(fname.c_str(), O_RDONLY);
  if (fd == -1)
    {
      if (errno != ENOENT) // no such file or dir is ok, other errors not
        PLERROR("Cannot open '%s'", fname.c_str());
      return false;
    }
  close(fd);
  return true;
}

// ######################################################################
std::string Raster::getImageComments(std::string fname,
                                     const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
 RasterFileFormat newftype = addFileExtension(fname, ftype, MODE_R, true);

  rutz::shared_ptr<RasterParser> pp(makeParser(newftype, fname.c_str()));
  return pp->getComments();
}

// ######################################################################
Dims Raster::getImageDims(std::string fname,
                          const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  GenericFrameSpec spec = Raster::getFrameSpec(fname, ftype);
  return spec.dims;
}

// ######################################################################
GenericFrameSpec Raster::getFrameSpec(std::string fname,
                                      const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
 RasterFileFormat newftype = addFileExtension(fname, ftype, MODE_R, true);

  rutz::shared_ptr<RasterParser> pp(makeParser(newftype, fname.c_str()));
  return pp->getFrameSpec();
}

// ######################################################################
GenericFrame Raster::ReadFrame(std::string fname,
                               RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ftype = addFileExtension(fname, ftype, MODE_R, true);
  LDEBUG("reading raster file: %s", fname.c_str());

  rutz::shared_ptr<RasterParser> pp(makeParser(ftype, fname.c_str()));
  return pp->getFrame();
}

// ######################################################################
Image<PixRGB<byte> > Raster::ReadRGB(std::string fname,
                                     const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return Raster::ReadFrame(fname, ftype).asRgb();
}

// ######################################################################
Image<byte> Raster::ReadGray(std::string fname,
                             const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return Raster::ReadFrame(fname, ftype).asGray();
}

// ######################################################################
Image<byte> Raster::ReadGrayNTSC(std::string fname,
                             const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return Raster::ReadFrame(fname, ftype).asGrayNTSC();
}


// ######################################################################
Image<float> Raster::ReadFloat(std::string fname,
                               const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return Raster::ReadFrame(fname, ftype).asFloat();
}

// ######################################################################
VideoFrame Raster::ReadVideo(std::string fname,
                             const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return Raster::ReadFrame(fname, ftype).asVideo();
}

// ######################################################################
std::string Raster::WriteFrame(const GenericFrame& image,
                               std::string fname,
                               RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!image.initialized())
    LFATAL("image was empty when trying to write to %s", fname.c_str());

  ftype = addFileExtension(fname, ftype, MODE_W, true);
  LDEBUG("writing raster file: %s", fname.c_str());

  rutz::shared_ptr<RasterWriter> writer = makeWriter(ftype, fname.c_str());
  return writer->writeFrame(image, fname);
}

// ######################################################################
std::string Raster::WriteRGB(const Image< PixRGB<byte> >& image,
                             std::string fname,
                             const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return Raster::WriteFrame(GenericFrame(image), fname, ftype);
}

// ######################################################################
std::string Raster::WriteGray(const Image<byte>& image,
                              std::string fname,
                              const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return Raster::WriteFrame(GenericFrame(image), fname, ftype);
}

// ######################################################################
std::string Raster::WriteFloat(const Image<float>& image,
                               const int flags,
                               std::string fname,
                               const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return Raster::WriteFrame(GenericFrame(image, flags), fname, ftype);
}

namespace
{
  bool USING_XV = true;
}

// ######################################################################
void Raster::NoXV()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  USING_XV = false;
}

// ######################################################################
void Raster::Display(const char* file)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (USING_XV)
    {
      std::string command("xv \"");
      command += file;
      command += "\" &";
      int ret = system(command.c_str());
      if (ret) LERROR("Display command returned: %d", ret);
    }
}

// ######################################################################
void Raster::waitForKey()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  char tmp[100];
  printf("<<<<< Press <Return> to continue >>>>>\n");

  //Just get the ret char* to avoid compile warning
  char* ret = fgets(tmp, 100, stdin);
  if (ret);
}

// ######################################################################
void Raster::VisuRGB(const Image< PixRGB<byte> >& image,
                     std::string fname,
                     const RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  fname = Raster::WriteRGB(image, fname, ftype);

  byte mi, ma; getMinMaxC(image, mi, ma);
  LINFO("%s [%d..%d]", fname.c_str(), int(mi), int(ma));

  Raster::Display(fname.c_str());
}

// ######################################################################
void Raster::VisuGray(const Image<byte>& image,
                      std::string fname,
                      RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  fname = Raster::WriteGray(image, fname, ftype);

  byte mi, ma; getMinMax(image, mi, ma);
  LINFO("%s [%d..%d]", fname.c_str(), int(mi), int(ma));

  Raster::Display(fname.c_str());
}

// ######################################################################
void Raster::VisuFloat(const Image<float>& image,
                       const int flags,
                       std::string fname,
                       RasterFileFormat ftype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  fname = Raster::WriteFloat(image, flags, fname, ftype);

  float mi, ma; getMinMax(image, mi, ma);
  if (flags & FLOAT_NORM_0_255)
    LINFO("%s [%f..%f] -- Normalized", fname.c_str(), mi, ma);
  else
    LINFO("%s [%f..%f]", fname.c_str(), mi, ma);

  Raster::Display(fname.c_str());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
