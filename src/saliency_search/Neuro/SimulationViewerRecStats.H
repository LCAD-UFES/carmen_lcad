/*!@file Neuro/SimulationViewerRecStats.H View/save a bunch of recognition stats */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerRecStats.H $
// $Id: SimulationViewerRecStats.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef NEURO_SIMULATIONVIEWERRECSTATS_H_DEFINED
#define NEURO_SIMULATIONVIEWERRECSTATS_H_DEFINED

#include "Neuro/SimulationViewer.H"
#include "Neuro/NeuroSimEvents.H"
#include "Media/TestImages.H"
#include <fstream>

class SimulationViewerRecStats : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerRecStats(OptionManager& mgr,
                        const std::string& descrName="Simulation Viewer Stats",
                        const std::string& tagName="SimulationViewerRecStats");
  //! Destructor
  virtual ~SimulationViewerRecStats();

  //@}

protected:
  //! Callback for when a new attention shift occurs
  SIMCALLBACK_DECLARE(SimulationViewerRecStats, SimEventWTAwinner);

  OModelParam<std::string> itsStatsFname; //!< Filename for the stats

  //! Get started:
  virtual void start2();

  //! Get stopped:
  virtual void stop2();

  //! apply an advisory lock on a file
  void lockFile(const std::string fileName, int &fd, struct flock &fl) const;

  //! remove an advisory lock on a file
  void unlockFile(const std::string fileName, const int fd,
                         struct flock &fl) const;

  std::string getObjNameAtLoc(const std::vector<TestImages::ObjData> &objects,
      const Point2D<int>& loc);

private:
  bool itsUseStatsFile;
  std::ofstream *itsStatsFile;    // file to write stats to
  unsigned long totalRec; //the total number we processed
  unsigned long totalRecScenes; //the total scenes recognized
  unsigned long totalRecObj; //the total obj recognized

};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_SIMULATIONVIEWERSTATS_H_DEFINED
