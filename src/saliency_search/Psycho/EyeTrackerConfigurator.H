/*!@file Psycho/EyeTrackerConfigurator.H Select an EyeTracker type */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerConfigurator.H $
// $Id: EyeTrackerConfigurator.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef PSYCHO_EYETRACKERCONFIGURATOR_H_DEFINED
#define PSYCHO_EYETRACKERCONFIGURATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

class EyeTracker;

//! EyeTracker configurator
/*! This will export the --et-type=XX command-line option and will
  instantiate an EyeTracker of the desired type as the option gets
  assigned a value. As this happens, new options may become available
  in the command-line. To see them, use --help AFTER you have chosen
  the type to use. The current EyeTracker may be retrieved using
  getET(). */
class EyeTrackerConfigurator : public ModelComponent
{
public:
  //! Constructor
  EyeTrackerConfigurator(OptionManager& mgr,
                         const std::string& descrName =
                         "Eye Tracker Configurator",
                         const std::string& tagName =
                         "EyeTrackerConfigurator");

  //! destructor
  virtual ~EyeTrackerConfigurator();

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    tracker of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Get the chosen EyeTracker
  /*! You should call this during start() of the ModelComponent that
    needs the EyeTracker. CAUTION: this may be a rutz::shared_ptr(NULL) if
    no tracker has been chosen. */
  nub::ref<EyeTracker> getET() const;

protected:
  OModelParam<std::string> itsETtype; //!< type of eye tracker

private:
  nub::ref<EyeTracker> itsET; // the eye tracker
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKERCONFIGURATOR_H_DEFINED
