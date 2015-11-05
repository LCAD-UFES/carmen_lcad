/*!@file Simulation/SimEventQueueConfigurator.H Pick a SimEventQueue */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventQueueConfigurator.H $
// $Id: SimEventQueueConfigurator.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef SIMULATION_SIMEVENTQUEUECONFIGURATOR_H_DEFINED
#define SIMULATION_SIMEVENTQUEUECONFIGURATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
class SimEventQueue;

class SimEventQueueConfigurator : public ModelComponent
{
public:
  //! Constructor
  SimEventQueueConfigurator(OptionManager& mgr,
                            const std::string& descrName =
                            "SimEvent Queue Configurator",
                            const std::string& tagName =
                            "SimEventQueueConfigurator");

  //! destructor
  virtual ~SimEventQueueConfigurator();

  //! Get the chosen SimEventQueue
  /*! You should call this during start() of the ModelComponent that
    needs the queue. CAUTION: this may be a rutz::shared_ptr(NULL) if
    no queue has been chosen. */
  nub::ref<SimEventQueue> getQ() const;

  //! Intercept people changing our itsSEQname ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a queue of
    the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

protected:
  OModelParam<std::string> itsSEQname; //!< Type of queue

private:
  nub::ref<SimEventQueue> itsQ; // the queue
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMEVENTQUEUECONFIGURATOR_H_DEFINED
