/*!@file Neuro/HandControllerConfigurator.H Configure hand controller */

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
// $Id:
//

#ifndef NEURO_HANDCONTROLLERCONFIGURATOR_H_DEFINED
#define NEURO_HANDCONTROLLERCONFIGURATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

class HandController;

// ######################################################################
/*!>HandController configurator for hand
 *  This will export the --hand-type=XX command-line option and will
 *  instantiate a HandController of the desired type as the option
 *  gets assigned a value. As this happens, new options may become
 *  available in the command-line. To see them, use --help AFTER you
 *  have chosen the name of the controller to use. The current
 *  HandController may be retrieved using getHand().
 */
class HandControllerConfigurator : public ModelComponent
{
public:
  //! Constructor
  HandControllerConfigurator (OptionManager& mgr,
                              const std::string& descrName =
                              "Hand Controller Configurator",
                              const std::string& tagName =
                              "HandControllerConfigurator");
  
  //! destructor
  virtual ~HandControllerConfigurator();
  
  //! Get the chosen HandController
  /*! You should call this during start() of the ModelComponent that
    needs the HandController. CAUTION: this may be a
    rutz::shared_ptr(NULL) if no controller has been chosen. */
  nub::ref<HandController> getHand() const;

  //! Intercept people changing our itsHandctrlName ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);
  
protected:
  OModelParam<std::string> itsHandctrlType; //!< Type of the controller
  
private:
  nub::ref<HandController> itsHand; // the controller
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_HANDCONTROLLERCONFIGURATOR_H_DEFINED
