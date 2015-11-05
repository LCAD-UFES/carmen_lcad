/*!@file Transport/RasterOutputSeries.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RasterOutputSeries.C $
// $Id: RasterOutputSeries.C 15290 2012-05-13 14:06:48Z kai $
//

#ifndef TRANSPORT_RASTEROUTPUTSERIES_C_DEFINED
#define TRANSPORT_RASTEROUTPUTSERIES_C_DEFINED

#include "Transport/RasterOutputSeries.H"

#include "Component/GlobalOpts.H"
#include "Component/OptionManager.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Transport/TransportOpts.H"
#include "Util/FileUtil.H"
#include "Util/TextLog.H"
#include "Util/sformat.H"

// ######################################################################
RasterOutputSeries::RasterOutputSeries(OptionManager& mgr)
  :
  FrameOstream(mgr, "Raster Output Series", "RasterOutputSeries"),
  itsLogFile(&OPT_TextLogFile, this),
  itsPrefix(""),
  itsFrameNumber(-1),
  itsNumericFieldWidth(6)
{}

// ######################################################################
RasterOutputSeries::~RasterOutputSeries()
{}

// ######################################################################
void RasterOutputSeries::setConfigInfo(const std::string& filestem)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  this->setFileStem(filestem);
}

// ######################################################################
bool RasterOutputSeries::setFrameNumber(int n)
{
  ASSERT(n >= 0);
  itsFrameNumber = n;

  return true;
}

// ######################################################################
void RasterOutputSeries::writeFrame(const GenericFrame& frame,
                                    const std::string& shortname,
                                    const FrameInfo& auxinfo)
{
  // figure out the file name to use:
  std::string fname(computeOutputFileName(shortname));

  // find out file format:
  const RasterFileFormat ff = getRasterFileFormat();

  // write the image:
  fname = Raster::WriteFrame(frame, fname, ff);

  const std::string& logfl = itsLogFile.getVal();

  switch (frame.nativeType())
    {
    case GenericFrame::NONE:     textLog(logfl, "WriteNil", fname); break;
    case GenericFrame::RGB_U8:   textLog(logfl, "WriteRGB", fname); break;
    case GenericFrame::RGB_F32:  textLog(logfl, "WriteRgbF32", fname); break;
    case GenericFrame::GRAY_U8:  textLog(logfl, "WriteGray", fname); break;
    case GenericFrame::GRAY_F32: textLog(logfl, "WriteFloat", fname); break;
    case GenericFrame::VIDEO:    textLog(logfl, "WriteVideo", fname); break;
    case GenericFrame::RGB_U16:  textLog(logfl, "WriteRGBU16", fname); break;
    case GenericFrame::GRAY_U16: textLog(logfl, "WriteGrayU16", fname); break;
    default: break;
    }
}

// ######################################################################
void RasterOutputSeries::closeStream(const std::string& shortname)
{
  /* nothing to see here, move along */
}

// ######################################################################
void RasterOutputSeries::setFileStem(const std::string& s)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  itsPrefix = s;

  // look for a single hash or a pair of hashes; if present, these
  // will specify the numeric field width to use when formatting the
  // frame numbers into the final filenames
  const std::string::size_type hashpos1 = s.find_first_of('#');

  // ok, got at least one hash:
  if (hashpos1 != s.npos)
    {
      // check for a second hash:
      const std::string::size_type hashpos2 =
        (hashpos1 + 1 < s.size())
        ? s.find_first_of('#', hashpos1 + 1)
        : s.npos;

      if (hashpos2 == hashpos1 + 1)
        {
          // ok, we got "##" which is equivalent to "#0#"
          itsPrefix.replace(hashpos1, 2, "");
          itsNumericFieldWidth = 0;
        }
      else if (hashpos2 != s.npos)
        {
          // ok, we got "#nnn#" where we expect nnn to be an integer
          ASSERT(hashpos2 > hashpos1);

          const std::string::size_type flen = hashpos2-hashpos1-1;

          const std::string format = s.substr(hashpos1+1, flen);
          int width = -1;
          if (sscanf(format.c_str(), "%d", &width) != 1
              || width < 0
              || width >= 256)
            LFATAL("invalid number format '#%s#' in file stem %s; "
                   "expected '#nnn#' where nnn is a non-negative integer "
                   "less than 256",
                   format.c_str(), s.c_str());
          itsPrefix.replace(hashpos1, flen+2, "");
          itsNumericFieldWidth = width;
        }
      else
        {
          // ok, we got "#" which by tradition is treated as "#6#"
          itsPrefix.replace(hashpos1, 1, "");
          itsNumericFieldWidth = 6;
        }
    }
}

// ######################################################################
std::string RasterOutputSeries::
computeOutputFileName(const std::string& key) const
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  if (key.length() == 0)
    LFATAL("output filename key must be non-empty");

  // first split itsPrefix into path and file components:
  std::string pfxdir, pfxtail;
  splitPath(itsPrefix, pfxdir, pfxtail);

  // also split the given key into path and file components:
  std::string keydir, keytail;
  splitPath(key, keydir, keytail);

  ASSERT(itsFrameNumber >= 0);

  ASSERT(itsNumericFieldWidth >= 0);
  ASSERT(itsNumericFieldWidth < 256);

  // if the 'pfxtail' part of our prefix is non-empty, then put a
  // hyphen after it before the key and the frame number:
  if (pfxtail.length() > 0)
    return sformat("%s%s%s-%s%0*d",
                   pfxdir.c_str(), keydir.c_str(),
                   pfxtail.c_str(), keytail.c_str(),
                   itsNumericFieldWidth, itsFrameNumber);
  else
    return sformat("%s%s%s%0*d",
                   pfxdir.c_str(), keydir.c_str(),
                   keytail.c_str(),
                   itsNumericFieldWidth, itsFrameNumber);
}

// ######################################################################
GenericRasterOutputSeries::GenericRasterOutputSeries(OptionManager& mgr)
  :
  RasterOutputSeries(mgr),
  itsRasterFileFormat(&OPT_OutputRasterFileFormat, this)
{}

// ######################################################################
GenericRasterOutputSeries::~GenericRasterOutputSeries() {}

// ######################################################################
RasterFileFormat GenericRasterOutputSeries::getRasterFileFormat() const
{
  return itsRasterFileFormat.getVal();
}

// ######################################################################
template <RasterFileFormat F>
FixedRasterOutputSeries<F>::FixedRasterOutputSeries(OptionManager& mgr)
  :
  RasterOutputSeries(mgr)
{}

// ######################################################################
template <RasterFileFormat F>
FixedRasterOutputSeries<F>::~FixedRasterOutputSeries() {}

// ######################################################################
template <RasterFileFormat F>
RasterFileFormat FixedRasterOutputSeries<F>::getRasterFileFormat() const
{
  return F;
}

template class FixedRasterOutputSeries<RASFMT_PNM>;
template class FixedRasterOutputSeries<RASFMT_PNG>;
template class FixedRasterOutputSeries<RASFMT_JPEG>;
template class FixedRasterOutputSeries<RASFMT_PFM>;
template class FixedRasterOutputSeries<RASFMT_RAW_VIDEO>;
template class FixedRasterOutputSeries<RASFMT_RAW_IMAGE>;
template class FixedRasterOutputSeries<RASFMT_TXT>;
template class FixedRasterOutputSeries<RASFMT_CCODE>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RASTEROUTPUTSERIES_C_DEFINED
