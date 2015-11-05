/*!@file Component/ParamClient.H */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ParamClient.H $
// $Id: ParamClient.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef COMPONENT_PARAMCLIENT_H_DEFINED
#define COMPONENT_PARAMCLIENT_H_DEFINED

#include "Component/ParamFlags.H"

class ModelParamBase;
class OptionedModelParam;

//! An abstract interface class for clients of model params
/*! Using this abstract interface class allows us to decouple the
    concrete model param classes (NModelParam and OModelParam) from
    the concrete client class (just ModelComponent, for now). */
class ParamClient
{
public:
  //! Default constructor
  ParamClient();

  //! Virtual destructor for proper base class destruction.
  virtual ~ParamClient();

  //! Our parameters will register with us upon construction
  virtual void registerParam(ModelParamBase* mp) = 0;

  //! Our parameters will register with us upon construction
  /*! @param flags Pass USE_MY_VAL here if you want the current value
      of the model param to be taken as the new default value,
      otherwise pass 0 for flags. */
  virtual void registerOptionedParam(OptionedModelParam* mp,
                                     const ParamFlag flags) = 0;

  //! Our parameters will un-register with us upon destruction
  virtual void unregisterParam(const ModelParamBase* mp) = 0;

  //! Return type for paramChange() indicating whether a param change was allowed
  enum ChangeStatus
    {
      CHANGE_ACCEPTED,
      CHANGE_REJECTED
    };

  //! Our params call us back with this function whenever they change value
  /*! Base classes should override this function if they need to take
      special action when param changes value.

      @param param the address of the ModelParamBase that changed;
             subclasses can compare this with the addresses of their
             model param members to figure out which is the relevant
             param

      @param valueChanged true if the value actually changed, false if
             the value was "set" but the new value is the same as the
             old value; some clients may want to avoid re-doing
             expensive operations if the value did not actually change

      @param status the subclass that implements paramChanged() should
             set *status to CHANGE_REJECTED if it wishes to reject a
             particular parameter change; the caller of paramChanged()
             is expected to set *status to CHANGE_ACCEPTED prior to
             calling paramChanged(), so the implentation of
             paramChanged() does not need to set *status
             CHANGE_ACCEPTED if it wishes to allow the change since
             that will already be the default status
  */
  virtual void paramChanged(ModelParamBase* param,
                            const bool valueChanged,
                            ChangeStatus* status) = 0;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_PARAMCLIENT_H_DEFINED
