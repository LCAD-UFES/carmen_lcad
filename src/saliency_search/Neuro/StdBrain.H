/*!@file Neuro/StdBrain.H A canonical brain, which a standard set of channels. */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/StdBrain.H $
// $Id: StdBrain.H 13657 2010-07-12 22:43:55Z beobot $
//

#ifndef STDBRAIN_H_DEFINED
#define STDBRAIN_H_DEFINED

#include "Neuro/Brain.H"
class ModelManager;
class PrefrontalCortexConfigurator;
class RetinaConfigurator;
class EyeHeadControllerConfigurator;
class HandControllerConfigurator;
class VisualCortexConfigurator;
class SaliencyMapConfigurator;
class TaskRelevanceMapConfigurator;
class AttentionGuidanceMapConfigurator;
class WinnerTakeAllConfigurator;
class SimulationViewerConfigurator;
class InferoTemporalConfigurator;
class GistEstimatorConfigurator;
class AttentionGateConfigurator;
class VisualBufferConfigurator;

class TargetChecker;
class ShapeEstimator;

//! A subclass of Brain that automatically sets up the standard modules
/*! StdBrain contains a VisualCortex, ShapeEstimator,
  EyeHeadController, SaliencyMap, TaskRelevanceMap,
  AttentionGuidanceMap, WinnerTakeAll and SimulationViewer as
  "modules" (see Brain.H). Module types to be used can be configured
  via the command-line thanks to the use of the appropriate
  Configurators here. */
class StdBrain : public Brain
{
public:
  //! Construct using a VisualCortex object with standard channels
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human useage
      @param tagName name for ParamMap usage */
  StdBrain(OptionManager& mgr, const std::string& descrName = "Brain",
           const std::string& tagName = "Brain");

  //! virtual destructor for safe inheritance
  virtual ~StdBrain();

private:
  nub::ref<PrefrontalCortexConfigurator> pfc;
  nub::ref<RetinaConfigurator> rec;
  nub::ref<VisualCortexConfigurator> vcc;
  nub::ref<GistEstimatorConfigurator> gec;
  nub::ref<SaliencyMapConfigurator> smc;
  nub::ref<TaskRelevanceMapConfigurator> trmc;
  nub::ref<AttentionGuidanceMapConfigurator> agmc;
  nub::ref<AttentionGateConfigurator> agc;
  nub::ref<WinnerTakeAllConfigurator> wtac;
  nub::ref<TargetChecker> tc;   ///// need a configurator!
  nub::ref<ShapeEstimator> se;   ///// need a configurator!
  nub::ref<EyeHeadControllerConfigurator> scc;
  nub::ref<HandControllerConfigurator> hand;
  nub::ref<InferoTemporalConfigurator> it;
  nub::ref<VisualBufferConfigurator> vbc;
  nub::ref<SimulationViewerConfigurator> svc;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
