/*!@file Transport/InfoOutputSeries.H Very simple output source that just prints basic info about the frames it receives */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/InfoOutputSeries.H $
// $Id: InfoOutputSeries.H 8444 2007-05-27 15:24:29Z rjpeters $
//

#ifndef TRANSPORT_INFOOUTPUTSERIES_H_DEFINED
#define TRANSPORT_INFOOUTPUTSERIES_H_DEFINED

#include "Component/ModelParam.H"
#include "Transport/FrameOstream.H"

/// Very simple output source that just prints basic info about the frames it receives
class InfoOutputSeries : public FrameOstream
{
public:
  //! Constructor
  InfoOutputSeries(OptionManager& mgr);

  //! Destructor
  virtual ~InfoOutputSeries();

  //! Override from FrameOstream; just calls setFileName()
  virtual void setConfigInfo(const std::string& filename);

  //! Write image statistics to the output file
  virtual void writeFrame(const GenericFrame& frame,
                          const std::string& shortname,
                          const FrameInfo& auxinfo);

  //! Forget about any info we have collected so far for shortname
  virtual void closeStream(const std::string& shortname);

  //! Specify the output filename for where the info should be written
  void setFileName(const std::string& filename);

private:
  virtual void stop2();

  InfoOutputSeries(const InfoOutputSeries&);
  InfoOutputSeries& operator=(const InfoOutputSeries&);

  OModelParam<bool> itsTestMode; // don't print frames-per-second in
                                 // test mode since it is not
                                 // reproducible

  struct Impl;
  Impl* rep;
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_INFOOUTPUTSERIES_H_DEFINED
