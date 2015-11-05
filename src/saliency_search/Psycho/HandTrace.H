/*!@file Psycho/HandTrace.H A hand movement trace on a steering wheel */
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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL: 
// #Id:

#ifndef PSYCHO_HANDTRACE_H_DEFINED
#define PSYCHO_HANDTRACE_H_DEFINED

#include "Image/Pixels.H"
#include "Psycho/HandData.H"
#include "Util/SimTime.H"
#include "rutz/shared_ptr.h"
#include "Component/ParamMap.H"

#include <string>
#include <vector>


//! A hand movement trace
/*! This class encapsulates a simple hand movement trace and provides
  I/O functions. */
class HandTrace {
  /*! Public functions, please see how to call these below the class */
public:
  // ######################################################################
  /*! @name Constructors, destructor and initialization */

  //! Constructor from a file on disk, use given display color
  HandTrace(const std::string& filename, const PixRGB<byte>& color);

  //! Destructor
  ~HandTrace();

  // ######################################################################
  /*! @name Data access functions */

  //! Did sample of given index happen before given time?
  /*! Typically one would keep incrementing the index until we return
    false here, at which point it would be time to increment the
    time. Note that we will always return false after the index gets
    past all available samples in the trace. */
  bool hasData(const size_t index, const SimTime& t) const;

  //! Do we have a sample for the given index?
  /*! This will return true as long as the index is not past the
    available amount of data. */
  bool hasData(const size_t index) const;

  //! Get the data for a given index
  /*! Note that this will throw a fatal error if we don't have data
    for that index; so be sure to check with hasData() first. */
  rutz::shared_ptr<HandData> data(const size_t index) const;

  //We need to be able to export the RawExtraData as well and pass it through...

  //! Get our display color
  PixRGB<byte> color() const;

  //! Get number of available (non-trash) samples
  size_t size() const;
  //! Get number of parsed events (if any)
  size_t numEvents() const;
  
  // ######################################################################
  /*! @name Metadata access functions */

  //! Get sampling period
  SimTime period() const;

  //! Get filename, with extension if any
  std::string filename() const;

  //! Get filename, without extension if any
  std::string basename() const;
  
private:
  // forbid assignment and copy-construction:
  HandTrace& operator=(const HandTrace& e);
  HandTrace(const HandTrace& e);

  // add data from an unformatted line
  bool pushData(const std::string line);
  bool isEventField(const std::string &field) const;

  //! To add/remove special field
  std::string special(std::string field) const;
  std::string unspecial(std::string field) const;

  // ## variables #########################################################
  std::string itsFilename;
  PixRGB<byte> itsColor;
  SimTime itsPeriod;
  size_t itsTrash;
  uint itsNumEvents;

  std::vector<std::string> itsFields;

  int nativeX, nativeY;
  // for internal use only!
  struct RawHandData {
    int x,y;
    std::vector<bool> b;
    int mx,my,nmx,nmy;
    bool mlb, mmb, mrb;
    std::string kbch;
    rutz::shared_ptr<ParamMap> extraData; // may be NULL if no extra data
  };
  std::vector<RawHandData> itsData;
  std::vector<rutz::shared_ptr<ParamMap> > itsEvents;
};

// ######################################################################
// ## Inline functions (public)
// ######################################################################
inline bool HandTrace::hasData(const size_t index, const SimTime& t) const {
  return ( index < itsData.size() && itsPeriod * int(index) < t ); }

// ######################################################################
inline bool HandTrace::hasData(const size_t index) const {
  return ( index < itsData.size() ); }

// ######################################################################
inline rutz::shared_ptr<HandData> HandTrace::data(const size_t index) const {
  if (index >= itsData.size()) LFATAL("Index past end of trace");
  rutz::shared_ptr<HandData> ret;
  //ret.reset(new HandData(itsData[index].x, itsData[index].y, itsData[index].b));
  ret.reset(new HandData(itsData[index].kbch.c_str(),
                         itsData[index].mx, itsData[index].my,
                         itsData[index].nmx, itsData[index].nmy,
                         itsData[index].mlb, itsData[index].mmb,
                         itsData[index].mrb,
                         itsData[index].x, itsData[index].y, itsData[index].b));
  return ret; }

// ######################################################################
inline PixRGB<byte> HandTrace::color() const {
  return itsColor; }

// ######################################################################
inline size_t HandTrace::size() const {
  return itsData.size(); }

// ######################################################################
inline size_t HandTrace::numEvents() const {
  return itsNumEvents; }

// ######################################################################
inline SimTime HandTrace::period() const {
  return itsPeriod; }

// ######################################################################
inline std::string HandTrace::filename() const {
  return itsFilename; } // return filename with extension if exist
 
// ######################################################################
inline std::string HandTrace::basename() const { // trace filename to extension
  size_t idx = itsFilename.rfind('.');
  if (idx != itsFilename.npos) return itsFilename.substr(0, idx);
  return itsFilename; } // no extension filename?



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_HANDTRACE_H_DEFINED
