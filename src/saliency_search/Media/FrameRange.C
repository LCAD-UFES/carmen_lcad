/*!@file Media/FrameRange.C A range of frames at a given inter-frame delay */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/FrameRange.C $
// $Id: FrameRange.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Media/FrameRange.H"

#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/log.H"
#include "Util/sformat.H"

#include <iterator> // for back_inserter()
#include <limits>
#include <vector>
#include <cstdio>  // for fopen() and friends

// ######################################################################
FrameRange::FrameRange() :
  itsFirst(0),
  itsStep(1),
  itsLast(0),
  itsDelayTimes(),
  itsEventTriggered(false)
{
  // make sure there is always at least one element in the vector:
  itsDelayTimes.push_back(SimTime::ZERO());
}

FrameRange::FrameRange(int first, int step, int last, std::vector<SimTime> delays, bool eventTriggered) :
  itsFirst(first),
  itsStep(step),
  itsLast(last),
  itsDelayTimes(delays),
  itsEventTriggered(eventTriggered)
{
  // make sure there is always at least one element in the vector:
  if(itsDelayTimes.size() == 0)
  {
    itsDelayTimes.push_back(SimTime::ZERO());
  }
}

// ######################################################################
std::string FrameRange::toString() const
{
  const std::string range =
    itsStep == 1
    ? sformat("%d-%d", itsFirst, itsLast)
    : sformat("%d-%d-%d", itsFirst, itsStep, itsLast);

  if (itsEventTriggered)
    {
      return sformat("%s@EVENT", range.c_str());
    }
  else
    {
      ASSERT(itsDelayTimes.size() > 0);
      if (itsDelayTimes.size() == 1)
        {
          return sformat("%s@%.2fms",
                         range.c_str(), itsDelayTimes[0].msecs());
        }
      else
        {
          SimTime totstime = itsDelayTimes[0];

          for (size_t i = 1; i < itsDelayTimes.size(); ++i)
            totstime += itsDelayTimes[i] - itsDelayTimes[i-1];

          // average inter-frame delay:
          const double aifd =
            totstime.msecs() / double(itsDelayTimes.size());
          return sformat("%s@variable delay "
                         "(average inter-frame delay %.2fms)",
                         range.c_str(), aifd);
        }
    }
}

// ######################################################################
FrameRange FrameRange::fromString(const std::string& str)
{
  const std::string::size_type at = str.find('@');
  if (at == str.npos)
    conversion_error::raise<FrameRange>
      (str, "missing '@'; expected [first-[last]]@delay");

  const std::string range = str.substr(0, at);
  const std::string sdelay = str.substr(at+1, str.npos);

  int xfirst = -2, xstep = -2, xlast = -2;

  if (range.length() == 0)
    {
      xfirst = 0;
      xstep = 1;
      xlast = std::numeric_limits<int>::max();
    }
  else
    {
#define EXPECTED_FORMAT "expected [[first[-step]]-[last]]@delay"

      std::vector<std::string> tokens;
      split(range, "-", std::back_inserter(tokens));

      if (tokens.size() != 2 && tokens.size() != 3)
        conversion_error::raise<FrameRange>
          (str, "missing '-'; " EXPECTED_FORMAT);

      const std::string sfirst = tokens.front();
      const std::string slast = tokens.back();

      if (sfirst.length() == 0)
        xfirst = 0;
      else if (sscanf(sfirst.c_str(), "%d", &xfirst) != 1)
        conversion_error::raise<FrameRange>
          (str, "bogus 'first'; " EXPECTED_FORMAT);

      if (slast.length() == 0
          || slast.compare("MAX") == 0
          || slast.compare("max") == 0)
        {
          xlast = std::numeric_limits<int>::max();
        }
      else if (sscanf(slast.c_str(), "%d", &xlast) != 1)
        {
          conversion_error::raise<FrameRange>
            (str, "bogus 'last'; " EXPECTED_FORMAT);
        }

      if (tokens.size() == 3)
        {
          if (sscanf(tokens[1].c_str(), "%d", &xstep) != 1)
            conversion_error::raise<FrameRange>
              (str, "bogus 'step'; " EXPECTED_FORMAT);

          if (xstep <= 0)
            conversion_error::raise<FrameRange>
              (str, "bogus 'step'; must be strictly greater than zero");
        }
      else
        xstep = 1;
    }

  FrameRange result;

  result.itsFirst = xfirst;
  result.itsStep = xstep;
  result.itsLast = xlast;
  result.itsEventTriggered = false;
  result.itsDelayTimes.resize(0);

  LDEBUG("sdelay: %s", sdelay.c_str());

  // check for magic values of sdelay:
  if (sdelay.compare("EVENT") == 0) {
      result.itsEventTriggered = true;
      // make sure there is always at least one element in the vector:
      result.itsDelayTimes.push_back(SimTime::ZERO());
  } else {
          // ok, sdelay didn't have a magic value
          // check if it's a filename
      const std::string::size_type suffix_pos = sdelay.find(".fl");
      if (suffix_pos == str.npos) {
              // no suffix, so assume sdelay is a time string
              // pass "ms" to indicate that the default units are milliseconds
          LDEBUG("didn't find fl suffix");
          result.itsDelayTimes.push_back(SimTime::fromString(sdelay, "ms"));
      } else {
              // the delay should be a .fl filename (necessary to account for variable framerates)
              // the expected format of .fl files is defined in the help message of OPT_InputFrameRange (FrameSeries.C)
          LDEBUG("found fl suffix");
          std::string fl_filename=sdelay;
          FILE *fl = fopen(fl_filename.c_str(), "r");
          if (fl == NULL) LFATAL("Cannot open %s", fl_filename.c_str());
          // double prevstime; // previous simulation time
          double nextstime; // next simulation time
          int i=1; // index of lines in fl file
          if (fscanf(fl, "%*s %lf\n", &nextstime) != 1) // read first line
            {
              fclose(fl);
              LFATAL("Error reading from %s[%i]",
                     fl_filename.c_str(), i);
            }
          if (nextstime!=0)
            {
              fclose(fl);
              LFATAL("Simulation start time must be 0, "
                     "but is %.2f in file: %s[%d]",
                     nextstime, fl_filename.c_str(), i);
            }
          while (!feof(fl)) {
              i++;
              //prevstime=nextstime;
              if (fscanf(fl, "%*s %lf\n", &nextstime) != 1)
                {
                  fclose(fl);
                  LFATAL("Error reading from %s[%d]",
                         fl_filename.c_str(), i);
                }
              result.itsDelayTimes.push_back(SimTime::MSECS(nextstime));
          }
      }
      LDEBUG("added %" ZU " delays to itsDelayTimes", result.itsDelayTimes.size());
  }

  LDEBUG("parsed FrameRange string '%s' as '%s'",
         str.c_str(), result.toString().c_str());

  return result;
}

// ######################################################################
bool FrameRange::operator==(const FrameRange& that) const
{
  return (this->itsFirst == that.itsFirst
          && this->itsStep == that.itsStep
          && this->itsLast == that.itsLast
          && this->itsDelayTimes == that.itsDelayTimes
          && this->itsEventTriggered == that.itsEventTriggered);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
