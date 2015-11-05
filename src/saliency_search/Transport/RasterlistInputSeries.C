/*!@file Transport/RasterlistInputSeries.C Reads image files from a list of filenames contained in a separate file */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/RasterlistInputSeries.C $
// $Id: RasterlistInputSeries.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef TRANSPORT_RASTERLISTINPUTSERIES_C_DEFINED
#define TRANSPORT_RASTERLISTINPUTSERIES_C_DEFINED

#include "Transport/RasterlistInputSeries.H"

#include "Raster/Raster.H"

#include <fstream>

// ######################################################################
RasterlistInputSeries::RasterlistInputSeries(OptionManager& mgr)
  :
  FrameIstream(mgr, "Raster-List Input Series", "RasterlistInputSeries"),
  itsListFname(),
  itsFiles(),
  itsFrameNumber(-1)
{}

// ######################################################################
RasterlistInputSeries::~RasterlistInputSeries()
{}

// ######################################################################
void RasterlistInputSeries::setConfigInfo(const std::string& fname)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  this->setListFile(fname);
}

// ######################################################################
bool RasterlistInputSeries::setFrameNumber(int n)
{
  ASSERT(n >= 0);
  itsFrameNumber = n;

  return true;
}

// ######################################################################
GenericFrameSpec RasterlistInputSeries::peekFrameSpec()
{
  return itsFrameSpec;
}

// ######################################################################
GenericFrame RasterlistInputSeries::readFrame()
{
  if (itsFrameNumber < 0)
    LFATAL("expected a non-negative frame number, but got %d",
           itsFrameNumber);

  if (size_t(itsFrameNumber) >= itsFiles.size())
    // out of input; end of stream
    return GenericFrame();

  if (!Raster::fileExists(itsFiles[itsFrameNumber]))
    {
      LINFO("at line %d of %s: no such file: %s",
            itsFrameNumber, itsListFname.c_str(),
            itsFiles[itsFrameNumber].c_str());
      return GenericFrame();
    }

  const GenericFrame result =
    Raster::ReadFrame(itsFiles[itsFrameNumber]);

  //if (result.frameSpec() != itsFrameSpec)
  //  LFATAL("at line %d of %s: expected %s to have format '%s', "
  //         "but got format '%s'",
  //         itsFrameNumber, itsListFname.c_str(),
  //         itsFiles[itsFrameNumber].c_str(),
  //         itsFrameSpec.getDescription().c_str(),
  //         result.frameSpec().getDescription().c_str());

  return result;
}

// ######################################################################
void RasterlistInputSeries::setListFile(const std::string& s)
{
  if (s.length() == 0)
    LFATAL("expected a valid filename, but got an empty string");

  std::ifstream ifs(s.c_str());

  if (!ifs.is_open())
    LFATAL("couldn't open file '%s' for reading", s.c_str());

  std::vector<std::string> names;

  std::string line;
  while (std::getline(ifs, line))
    names.push_back(line);

  LINFO("read %" ZU " filenames from %s", names.size(), s.c_str());

  if (names.size() == 0)
    LFATAL("Expected at least one image filename in %s, "
           "but found none", s.c_str());

  itsFrameSpec = Raster::getFrameSpec(names[0]);
  itsListFname = s;
  itsFiles.swap(names);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_RASTERLISTINPUTSERIES_C_DEFINED
