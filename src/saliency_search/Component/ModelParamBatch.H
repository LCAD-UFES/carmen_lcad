/*!@file Component/ModelParamBatch.H Batch setting and restoring of model parameter values */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelParamBatch.H $
// $Id: ModelParamBatch.H 8762 2007-09-06 22:58:08Z rjpeters $
//

#ifndef COMPONENT_MODELPARAMBATCH_H_DEFINED
#define COMPONENT_MODELPARAMBATCH_H_DEFINED

#include "Component/ModelParamBase.H" // for TRefHolder
#include "rutz/shared_ptr.h"

#include <string>
#include <utility>
#include <vector>

class ModelComponent;

/// A helper class for batch setting and restoring groups of parameter values
/** Typical usage would be like this:

    \code
    ModelParamBatch params;
    params.addParamValue("SomeParam", int(1));
    params.addParamValue("SomeFlag", true);
    params.addParamValue("SomeString", "hello");

    ModelManager manager;

    params.installValues(&manager); // params are now 1,true,"hello"
    // ... do some work

    // ... later:
    params.restoreValues(&manager); // params are now back to their original values
    \endcode

 */
class ModelParamBatch
{
public:
  ModelParamBatch();

  ~ModelParamBatch();

  /// Add a new parameter name and associated value to our list of managed parameters
  template <class T>
  void addParamValue(const std::string& paramname,
                     const T& val)
  {
    addParamValueAux(paramname,
                     rutz::shared_ptr<RefHolder>
                     (new TRefHolder<const T>(val)));
  }

  /// Install our stored parameter values in the given component
  /** The component's current values for those parameters will be
      remembered so that they can later be restored with
      restoreValue(). */
  void installValues(ModelComponent* comp);

  /// Restore parameter values saved during a previous installValues() call
  void restoreValues(ModelComponent* comp);

  /// Auxiliary implementation function for addParamValue()
  void addParamValueAux(const std::string& paramname,
                        const rutz::shared_ptr<RefHolder>& valref);

private:
  std::vector<std::vector<std::pair<std::string, std::string> > > itsRestoreStack;
  std::vector<std::pair<std::string, rutz::shared_ptr<RefHolder> > > itsParamValues;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_MODELPARAMBATCH_H_DEFINED
