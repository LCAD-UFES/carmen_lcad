/*!@file Neuro/SimulationViewerStats.H View/save a bunch of stats */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerStats.H $
// $Id: SimulationViewerStats.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef NEURO_SIMULATIONVIEWERSTATS_H_DEFINED
#define NEURO_SIMULATIONVIEWERSTATS_H_DEFINED

#include "Neuro/SimulationViewer.H"
#include "Simulation/SimEvents.H"
#include <fstream>

//! View/save a bunch of stats about the saliency map
/*! This SimulationViewer's purpose is mainly to save a variety of
  stats about the saliency map, including about locations of highest
  peak, mean/variance of saliency, number of peaks, etc. */
class SimulationViewerStats : public SimulationViewer {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerStats(OptionManager& mgr,
                        const std::string& descrName="Simulation Viewer Stats",
                        const std::string& tagName="SimulationViewerStats");
  //! Destructor
  virtual ~SimulationViewerStats();

  //@}

protected:
  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SimulationViewerStats, SimEventSaveOutput);

  OModelParam<std::string> itsStatsFname; //!< Filename for the stats
  OModelParam<bool> itsSaveXcombo;        //!< Save images too
  OModelParam<bool> itsSaveYcombo;        //!< Save images too
  OModelParam<bool> itsComputeAGStats;    //!< Save Attention Gate Stats
  OModelParam<int>  itsAGTargetFrame;     //!< Which frame is the target?
  OModelParam<std::string> itsAGMaskFile; //!< Which file has the Ground Truth
  OModelParam<std::string> itsAGStatsSaveFile; //!< File save AG stats to
  /*! @name Compatable statistics with SingleChannels stats */
  //@{

  //! Save basic single channel stats
  /*! we also save final saliency stats using the same format */
  OModelParam<bool>         itsGetSingleChannelStats;

  //! File name for single channel stats
  /*! we also save final saliency stats using the same format */
  OModelParam<std::string> itsGetSingleChannelStatsFile;

  //! Should we save frequency information using 2D FFT?
  OModelParam<bool>        itsSaveStatsPerChannelFreq;

  //@}

  //! Get started:
  virtual void start2();

  //! Get stopped:
  virtual void stop2();

  //! apply an advisory lock on a file
  void lockFile(const std::string fileName, int &fd, struct flock &fl) const;

  //! remove an advisory lock on a file
  void unlockFile(const std::string fileName, const int fd,
                  struct flock &fl) const;

  //! Save our various results
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! Compute and save target stats with mask and AttentionGate?
  /*! This method requires a mask file for ground truth. If none is
      avialable, then use computeLAMStats to save basic attention gate
      stats only
  */
  virtual void computeAGStats(SimEventQueue& q);

  //! Compute Last Attention Map stats from AttentionGate?
  virtual void computeLAMStats(const Image<float> &img);

  //! Save our various results in a channel compatable format used in SingleChannel.C
  virtual void saveCompat(const Image<float>& img,
                          const std::string suffix = ".final.txt",
                          const int frameOffset = 0);
  //! Save Attention Gate stats if requested
  virtual void saveAGMaskStats(const Image<float> &img,
                               const std::string caller,
                               const std::string suffix = ".final-AGmask.txt");

private:
  std::ofstream *itsStatsFile;      // file to write stats to
  uint          itsFrameIdx;        // for logging purposes
  uint          itsSizeX, itsSizeY; // Frame image size, local copy
  uint          itsFrameNumber;     // What is the actual frame?
  uint itsMaskCount, itsLamCount, itsOverlapCount, itsTotalCount;

  Image<bool>          itsMask;
  Image<bool>          itsLamMask;
  Image<PixRGB<byte> > itsOverlap;
  Image<PixRGB<byte> > itsVisualSegments;
  Image<byte>          itsVisualCandidates;
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_SIMULATIONVIEWERSTATS_H_DEFINED
