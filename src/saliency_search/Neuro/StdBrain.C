/*!@file Neuro/StdBrain.C A canonical brain, which a standard set of channels. */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/StdBrain.C $
// $Id: StdBrain.C 13657 2010-07-12 22:43:55Z beobot $
//

#include "Neuro/StdBrain.H"
#include "Channels/ChannelBase.H"
#include "Component/OptionManager.H"
#include "Neuro/AttentionGate.H"
#include "Neuro/AttentionGuidanceMap.H"
#include "Neuro/GistEstimator.H"
#include "Neuro/GistEstimatorConfigurator.H"
#include "Neuro/InferoTemporal.H"
#include "Neuro/InferoTemporalConfigurator.H"
#include "Neuro/PrefrontalCortex.H"
#include "Neuro/Retina.H"
#include "Neuro/EyeHeadControllerConfigurator.H"
#include "Neuro/HandControllerConfigurator.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/SaliencyMapConfigurator.H"
#include "Neuro/ShapeEstimator.H"
#include "Neuro/SimulationViewer.H"
#include "Neuro/SimulationViewerConfigurator.H"
#include "Neuro/TargetChecker.H"
#include "Neuro/TaskRelevanceMap.H"
#include "Neuro/VisualBuffer.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/VisualCortexConfigurator.H"
#include "Neuro/WinnerTakeAll.H"
#include "Neuro/WinnerTakeAllConfigurator.H"

// ######################################################################
StdBrain::StdBrain(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName) :
  Brain(mgr, descrName, tagName),
  pfc(new PrefrontalCortexConfigurator(mgr)),
  rec(new RetinaConfigurator(mgr)),
  vcc(new VisualCortexConfigurator(mgr)),
  gec(new GistEstimatorConfigurator(mgr)),
  smc(new SaliencyMapConfigurator(mgr)),
  trmc(new TaskRelevanceMapConfigurator(mgr)),
  agmc(new AttentionGuidanceMapConfigurator(mgr)),
  agc(new AttentionGateConfigurator(mgr)),
  wtac(new WinnerTakeAllConfigurator(mgr)),
  tc(new TargetChecker(mgr)),
  se(new ShapeEstimator(mgr)),
  scc(new EyeHeadControllerConfigurator(mgr)),
  hand(new HandControllerConfigurator(mgr)),
  it(new InferoTemporalConfigurator(mgr)),
  vbc(new VisualBufferConfigurator(mgr)),
  svc(new SimulationViewerConfigurator(mgr))
{
  // NOTE: if callback priorities are all zero, the order here defines
  // the order in which the callbacks will be called when several
  // modules are catching a given event:
  addSubComponent(tc);
  addSubComponent(se);
  addSubComponent(pfc);  // make sure it can get configured
  addSubComponent(rec);  // make sure it can get configured
  addSubComponent(scc);  // make sure it can get configured
  addSubComponent(hand);  // make sure it can get configured
  addSubComponent(vcc);  // make sure it can get configured
  addSubComponent(smc);  // make sure it can get configured
  addSubComponent(gec);  // make sure it can get configured
  addSubComponent(trmc); // make sure it can get configured
  addSubComponent(agmc); // make sure it can get configured
  addSubComponent(agc);  // make sure it can get configured
  addSubComponent(wtac); // make sure it can get configured
  addSubComponent(it);   // make sure it can get configured
  addSubComponent(vbc);  // make sure it can get configured
  addSubComponent(svc);  // make sure it can get configured
}

// ######################################################################
StdBrain::~StdBrain()
{  }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
