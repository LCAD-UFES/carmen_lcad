/*!@file Transport/RasterInputSeries.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RasterInputSeries.C $
// $Id: RasterInputSeries.C 12962 2010-03-06 02:13:53Z irock $
//

#ifndef TRANSPORT_RASTERINPUTSERIES_C_DEFINED
#define TRANSPORT_RASTERINPUTSERIES_C_DEFINED

#include "Transport/RasterInputSeries.H"

#include "Component/GlobalOpts.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Transport/RasterInputOptions.H"
#include "Transport/TransportOpts.H"
#include "Util/TextLog.H"
#include "Util/sformat.H"

#include <cstdio>  // for sscanf()

namespace
{
  // ######################################################################
  //! compute input filename for current frame
  std::string computeInputFileName(const std::string& stem,
                                   const int framenumber)
  {
    // NOTE: if you modify any behavior here, then please update the
    // corresponding documentation for the global "--in" option inside
    // the OPT_InputFrameSource definition in Media/MediaOpts.C

    ASSERT(framenumber >= 0);

    // if there is a '#' in the stem, replace it with the frame number
    const std::string::size_type hashpos = stem.find_first_of('#');

    if (hashpos != stem.npos)
      {
        std::string fname = stem;

        // now, if there is a second '#' in the stem, then the
        // characters in between the two hashes will determine the
        // numeric format
        const std::string::size_type hashpos2 =
          (hashpos + 1 < stem.size())
          ? stem.find_first_of('#', hashpos + 1)
          : stem.npos;

        if (hashpos2 == hashpos + 1)
          {
            fname.replace(hashpos, 2, sformat("%d", framenumber));
          }
        else if (hashpos2 != stem.npos)
          {
            ASSERT(hashpos2 > hashpos);

            const std::string::size_type flen = hashpos2-hashpos-1;

            const std::string format = stem.substr(hashpos+1, flen);
            int width = -1;
            if (sscanf(format.c_str(), "%d", &width) != 1
                || width < 0
                || width >= 256)
              LFATAL("invalid number format '#%s#' in file stem %s; "
                     "expected '#nnn#' where nnn is a non-negative integer "
                     "less than 256",
                     format.c_str(), stem.c_str());
            fname.replace(hashpos, flen+2,
                          sformat("%0*d", width, framenumber));
          }
        else
          {
            fname.replace(hashpos, 1, sformat("%06d", framenumber));
          }

        return fname;
      }

    // else... no '#', so just return the filename as-is
    return stem;
  }
}

// ######################################################################
RasterInputSeries::RasterInputSeries(OptionManager& mgr)
  :
  FrameIstream(mgr, "Raster Input Series", "RasterInputSeries"),
  itsOptions(new RasterInputOptions(mgr)),
  itsLogFile(&OPT_TextLogFile, this),
  itsRasterFileFormat(&OPT_InputRasterFileFormat, this),
  itsStem(""),
  itsPrevFilename(""),
  itsFrameNumber(-1)
{
  this->addSubComponent(itsOptions);
}

// ######################################################################
RasterInputSeries::~RasterInputSeries()
{}

// ######################################################################
void RasterInputSeries::setConfigInfo(const std::string& filestem)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  this->setFileStem(filestem);
}

// ######################################################################
bool RasterInputSeries::setFrameNumber(int n)
{
  ASSERT(n >= 0);
  itsFrameNumber = n;

  return true;
}

// ######################################################################
GenericFrameSpec RasterInputSeries::peekFrameSpec()
{
  if (itsFrameNumber < 0)
    LFATAL("frame number is %d, but peekFrameSpec() requires a "
           "non-negative frame number", itsFrameNumber);

  const std::string fname(computeInputFileName(itsStem, itsFrameNumber));

  return Raster::getFrameSpec(fname, itsRasterFileFormat.getVal());
}

// ######################################################################
GenericFrame RasterInputSeries::readFrame()
{
  if (itsFrameNumber < 0)
    LFATAL("frame number is %d, but readFrame() requires a "
           "non-negative frame number", itsFrameNumber);

  // figure out the file name to use:
  const std::string fname(computeInputFileName(itsStem, itsFrameNumber));

  // check if we've already read that file; if so, then return an
  // empty image
  if (fname == itsPrevFilename)
  {
    LINFO("repeated input file skipped: %s", fname.c_str());
    return GenericFrame();
  }

  // check whether the file exists; if not, then return an empty image:
  if (!Raster::fileExists(fname, itsRasterFileFormat.getVal()))
    {
      LINFO("no such file: %s", fname.c_str());
      return GenericFrame();
    }

  // load the image:
  const GenericFrame ima =
    Raster::ReadFrame(fname, itsRasterFileFormat.getVal());

  itsPrevFilename = fname;

  textLog(itsLogFile.getVal(), "ReadFrame", fname);

  return ima;
}

// ######################################################################
void RasterInputSeries::setFileStem(const std::string& s)
{
  itsStem = s;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RASTERINPUTSERIES_C_DEFINED
