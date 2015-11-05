/*!@file Psycho/EyeSFile.C Read data from a .eyeS eye position file */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeSFile.C $
// $Id: EyeSFile.C 14730 2011-04-14 18:28:58Z lior $
//

#ifndef PSYCHO_EYESFILE_C_DEFINED
#define PSYCHO_EYESFILE_C_DEFINED

#include "Psycho/EyeSFile.H"

#include "Component/ModelOptionDef.H"
#include "Media/MediaOpts.H"
#include "Psycho/PsychoOpts.H"
#include "rutz/trace.h"

#include <fstream>
#include <iomanip>
#include <sstream>

// Used by: EyeSFile
static const ModelOptionDef OPT_EyeSFileName =
  { MODOPT_ARG_STRING, "EyeSFileName", &MOC_EYETRACK, OPTEXP_CORE,
    "Name of the .eyeS file from which to read eye movement samples",
    "eyeS-fname", '\0', "<fname.eyeS>", "" };

// Used by: EyeSFile
static const ModelOptionDef OPT_EyeSNumSkip =
  { MODOPT_ARG(int), "EyeSNumSkip", &MOC_EYETRACK, OPTEXP_CORE,
    "Number of leading samples to skip in the .eyeS file",
    "eyeS-num-skip", '\0', "<int>", "0" };

// Used by: EyeSFile
const ModelOptionDef OPT_EyeSPeriod =
  { MODOPT_ARG(SimTime), "EyeSPeriod", &MOC_EYETRACK, OPTEXP_CORE,
    "Eye tracker sampling period in the .eyeS file",
    "eyeS-period", '\0', "<float>", "240Hz" };

// Used by: EyeSFile
const ModelOptionDef OPT_EyeSDims =
  { MODOPT_ARG(Dims), "EyeSDims", &MOC_EYETRACK, OPTEXP_CORE,
    "Stimulus dimensions for the eye samples in the .eyeS file",
    "eyeS-dims", '\0', "<w>x<h>", "0x0" };

EyeSFile::EyeSFile(OptionManager& mgr)
  :
  ModelComponent(mgr, "EyeSFile", "EyeSFile"),
  itsEyeFname(&OPT_EyeSFileName, this),
  itsEyeTrash(&OPT_EyeSNumSkip, this),
  itsEyePeriod(&OPT_EyeSPeriod, this),
  itsRawInputDims(&OPT_EyeSDims, this),
  itsFile(0),
  itsEyeSample(0),
  itsPos(0, 0)
{}

EyeSFile::~EyeSFile()
{}

void EyeSFile::start2()
{
  ASSERT(itsFile == 0);

  if (itsEyeFname.getVal().length() == 0)
    LFATAL("No --%s given!", itsEyeFname.getOptionDef()->longoptname);

  itsFile = new std::ifstream(itsEyeFname.getVal().c_str());
  if (!itsFile->is_open())
    LFATAL("Couldn't open .eyeS file '%s' for reading",
           itsEyeFname.getVal().c_str());

  // FIXME merge all this stuff with EyeTrackerSaccadeController

  // Look for auxiliary data files associated with the main .eyeS
  // file, and read data from them if present. Specifically, if
  // itsEyeFname is e.g. "foo.eyeS", then we will look for a file
  // "foo.eyeS.ntrash" to contain a value for itsEyeTrash, and we will
  // look for a file "foo.eyeS.rate" to contain a value for
  // itsEyePeriod. That way, those files can be generated once at the
  // same time that the .eyeS file itself is generated (probably from
  // some perl/matlab scripts). Note that the .rate file should
  // contain a suffix to indicate what units the value is in,
  // e.g. "240.19Hz" or "4.16337ms".

  {
    std::ifstream f((itsEyeFname.getVal() + ".npts").c_str());
    if (f.is_open())
      {
        uint npts = 0;
        f >> npts;
        if (!f.fail())
          LINFO("%s.npts = %u", itsEyeFname.getVal().c_str(), npts);
      }
  }

  {
    std::ifstream f((itsEyeFname.getVal() + ".ntrash").c_str());
    if (f.is_open())
      {
        int ntrash = 0;
        f >> ntrash;
        if (!f.fail())
          {
            LINFO("%s.ntrash = %d", itsEyeFname.getVal().c_str(), ntrash);
            const int oldval = itsEyeTrash.getVal();
            itsEyeTrash.setVal(ntrash);
            LINFO("reset --%s from %d to %d",
                  itsEyeFname.getOptionDef()->longoptname,
                  oldval, itsEyeTrash.getVal());
          }
      }
  }

  {
    std::ifstream f((itsEyeFname.getVal() + ".rate").c_str());
    if (f.is_open())
      {
        std::string rate;
        f >> rate;
        if (!f.fail())
          {
            LINFO("%s.rate = %s", itsEyeFname.getVal().c_str(), rate.c_str());
            const SimTime oldval = itsEyePeriod.getVal();
            itsEyePeriod.setValString(rate);
            LINFO("reset --%s from %fs (%fHz) to %fs (%fHz)",
                  itsEyePeriod.getOptionDef()->longoptname,
                  oldval.secs(), oldval.hertz(),
                  itsEyePeriod.getVal().secs(),
                  itsEyePeriod.getVal().hertz());
          }
      }
  }

  for (int i = 0; i < itsEyeTrash.getVal(); ++i)
    {
      std::string line;
      std::getline(*itsFile, line);
    }

  // count the number of lines AFTER the initial skipped lines
  itsEyeSample = 0;

  ASSERT(itsRawInputDims.getVal().isNonEmpty());
}

void EyeSFile::stop1()
{
  ASSERT(itsFile != 0);
  itsFile->close();
  delete itsFile;
  itsFile = 0;
}

Point2D<int> EyeSFile::readUpTo(const SimTime& stime)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(itsFile != 0);

  if (itsFile->eof())
    return Point2D<int>(-1,-1);

  double xsum = 0.0, ysum = 0.0;
  int pcount = 0;

  while (itsEyeSample * itsEyePeriod.getVal() < stime)
    {
      ++itsEyeSample;
      std::string line;
      std::getline(*itsFile, line);
      *itsFile >> std::ws;
      if (line.find("NaN") == line.npos)
        {
          double x, y, targetx, targety, ampl;
          int status, fixlen;
          std::istringstream iss(line);

          // see findSaccades.m for details on how the columns of
          // the .eyeS file are generated
          iss >> x >> y >> status >> targetx >> targety >> ampl >> fixlen;
          if (iss.fail())
            LFATAL("error while scanning %s:%d:\n%s",
                   itsEyeFname.getVal().c_str(), itsEyeSample,
                   line.c_str());
          xsum += x;
          ysum += y;
          ++pcount;
        }

      if (itsFile->eof())
        break;

      if (itsFile->fail())
        LFATAL("input failed at %s:%d",
               itsEyeFname.getVal().c_str(), itsEyeSample);
    }

  if (pcount > 0)
    {
      const int x = int(xsum/pcount + 0.5);
      const int y = int(ysum/pcount + 0.5);

      if (x >= 0 && x < itsRawInputDims.getVal().w() &&
          y >= 0 && y < itsRawInputDims.getVal().h())
        {
          itsPos = Point2D<int>(int(xsum/pcount), int(ysum/pcount));
        }
    }

  return itsPos;
}

Point2D<float> EyeSFile::getPos()
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(itsFile != 0);

  if (itsFile->eof())
    return Point2D<float>(-1,-1);

  std::string line;
  std::getline(*itsFile, line);
  *itsFile >> std::ws;

  std::istringstream iss(line);

  double x, y, targetx, targety, ampl;
  int status, fixlen;
  iss >> x >> y >> status >> targetx >> targety >> ampl >> fixlen;
  if (iss.fail())
    LFATAL("error while scanning %s:%d:\n%s",
        itsEyeFname.getVal().c_str(), itsEyeSample,
        line.c_str());

  if (itsFile->eof())
    return Point2D<float>(-1,-1);

  if (itsFile->fail())
    LFATAL("input failed at %s:%d",
        itsEyeFname.getVal().c_str(), itsEyeSample);

  return Point2D<float>(x,y);
}

int EyeSFile::lineNumber() const
{
  return itsEyeSample;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYESFILE_C_DEFINED
