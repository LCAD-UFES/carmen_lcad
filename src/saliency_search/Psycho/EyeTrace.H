/*!@file Psycho/EyeTrace.H An eye movement trace */

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
// Primary maintainer for this file: John Shen <shenjohn@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrace.H $
//

#ifndef PSYCHO_EYETRACE_H_DEFINED
#define PSYCHO_EYETRACE_H_DEFINED

#include "Image/Pixels.H"
#include "Psycho/PixPerDeg.H"
#include "Psycho/EyeData.H"
//#include "Psycho/HandData.H" //remove me
#include "Util/SimTime.H"
#include "rutz/shared_ptr.h"
#include "Component/ParamMap.H"

#include <string>
#include <vector>

//! An eye movement trace
/*! This class encapsulates a simple eye movement trace and provides
  I/O functions. */
class EyeTrace {
public:

  // ######################################################################
  /*! @name Constructors, destructor and initialization */
  //@{

  //! Constructor from a file on disk, use given display color
  EyeTrace(const std::string& filename, const PixRGB<byte>& color);

  //! Destructor
  ~EyeTrace();

  //@}

  // ######################################################################
  /*! @name Data access functions */
  //@{

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
  rutz::shared_ptr<EyeData> data(const size_t index) const;

  //We need to be able to export the RawExtraData as well and pass it through...

  //! Get our display color
  PixRGB<byte> color() const;

  //! Get number of available (non-trash) samples
  size_t size() const;
  //! Get number of parsed events (if any)
  size_t numEvents() const;
  //! Deprecated command, as new markup may apply to more than saccades
  size_t numSaccades() const;
  
  //@}

  // ######################################################################
  /*! @name Metadata access functions */
  //@{

  //! Get sampling period
  SimTime period() const;

  //! Get filename
  std::string filename() const;

  //! Get filename, without extension if any
  std::string basename() const;

  //! Get pixels-per-degree of the display
  PixPerDeg ppd() const;
  //@}

private:
  // forbid assignment and copy-construction:
  EyeTrace& operator=(const EyeTrace& e);
  EyeTrace(const EyeTrace& e);

  //decode status to SaccadeState
  const SaccadeState getStatus( const int) const;

  // add data from an unformatted line
  bool pushData(const std::string line);
  bool isEventField(const std::string field) const;
  std::string itsFilename;
  PixRGB<byte> itsColor;
  SimTime itsPeriod;
  size_t itsTrash;
  PixPerDeg itsPPD;
  uint itsNumEvents;

  std::vector<std::string> itsFields;

  // for internal use only!
  struct RawEyeData {
    float x, y, diam;
    int status;
    rutz::shared_ptr<ParamMap> extraData; // may be NULL if no extra data
  };
  std::vector<RawEyeData> itsData;
  std::vector<rutz::shared_ptr<ParamMap> > itsEvents;
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACE_H_DEFINED
