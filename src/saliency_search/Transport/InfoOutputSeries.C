/*!@file Transport/InfoOutputSeries.C Very simple output source that just prints basic info about the frames it receives */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/InfoOutputSeries.C $
// $Id: InfoOutputSeries.C 8864 2007-10-19 19:11:17Z rjpeters $
//

#ifndef TRANSPORT_INFOOUTPUTSERIES_C_DEFINED
#define TRANSPORT_INFOOUTPUTSERIES_C_DEFINED

#include "Transport/InfoOutputSeries.H"

#include "Component/GlobalOpts.H"
#include "Image/Dims.H"
#include "Raster/GenericFrame.H"
#include "Util/FileUtil.H"
#include "Util/log.H"
#include "rutz/time.h"

#include <map>

// ######################################################################
struct InfoOutputSeries::Impl
{
  struct ChanInfo
  {
    ChanInfo() : spec(), nframes(0) {}

    GenericFrameSpec spec;
    int nframes;
    rutz::time first_time;
    rutz::time last_time;
  };

  typedef std::map<std::string, ChanInfo> map_type;

  map_type infoMap;
  std::string fileName;
};

// ######################################################################
InfoOutputSeries::InfoOutputSeries(OptionManager& mgr)
  :
  FrameOstream(mgr, "InfoOutputSeries", "Info Output Series"),
  itsTestMode(&OPT_TestMode, this),
  rep(new Impl)
{}

// ######################################################################
InfoOutputSeries::~InfoOutputSeries()
{
  delete rep;
}

// ######################################################################
void InfoOutputSeries::setConfigInfo(const std::string& filename)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  this->setFileName(filename);
}

// ######################################################################
void InfoOutputSeries::writeFrame(const GenericFrame& frame,
                                  const std::string& shortname,
                                  const FrameInfo& auxinfo)
{
  const rutz::time t = rutz::time::wall_clock_now();

  Impl::ChanInfo& info = rep->infoMap[shortname];

  info.spec = frame.frameSpec();
  if (info.nframes == 0)
    info.first_time = t;
  info.last_time = t;
  info.nframes++;
}

// ######################################################################
void InfoOutputSeries::stop2()
{
  try
    {
      bool owned = false;
      FILE* f = stdOutputFileOpen(rep->fileName, &owned); // may throw

      for (Impl::map_type::const_iterator
             itr = rep->infoMap.begin(),
             stop = rep->infoMap.end();
           itr != stop; ++itr)
        {
          const double secs =
            ((*itr).second.last_time - (*itr).second.first_time).sec();

          const double fps =
            (*itr).second.nframes >= 2
            ? ((*itr).second.nframes - 1) / secs
            : 0.0;

          if (itsTestMode.getVal())
            fprintf(f, "InfoOutputSeries: summary[%s]: %d frames (%s)\n",
                    (*itr).first.c_str(),
                    (*itr).second.nframes,
                    (*itr).second.spec.getDescription().c_str());
          else
            fprintf(f, "InfoOutputSeries: summary[%s]: %d frames @ %.2ffps (%s)\n",
                    (*itr).first.c_str(),
                    (*itr).second.nframes,
                    fps,
                    (*itr).second.spec.getDescription().c_str());
        }

      fflush(f);

      if (owned)
        fclose(f);
    }
  catch (...)
    {
      // don't propagate exceptions here since we are potentially
      // inside a destructor chain
    }
}

// ######################################################################
void InfoOutputSeries::closeStream(const std::string& shortname)
{
  Impl::map_type::iterator itr = rep->infoMap.find(shortname);
  if (itr != rep->infoMap.end())
    rep->infoMap.erase(itr);
}

// ######################################################################
void InfoOutputSeries::setFileName(const std::string& filename)
{
  rep->fileName = filename;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_INFOOUTPUTSERIES_C_DEFINED
