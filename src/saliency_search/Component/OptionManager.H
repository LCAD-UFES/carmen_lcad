/*!@file Component/OptionManager.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/OptionManager.H $
// $Id: OptionManager.H 6466 2006-04-13 19:12:17Z rjpeters $
//

#ifndef COMPONENT_OPTIONMANAGER_H_DEFINED
#define COMPONENT_OPTIONMANAGER_H_DEFINED

#include "Util/Types.H" // for uint

#include <string>

class ModelOptionDef;
class OptionedModelParam;
class SimTime;

//! Abstract interface for the option-managing part of ModelManager.
class OptionManager
{
public:
  //! Default constructor.
  OptionManager();

  //! Virtual destructor for proper base class destruction.
  virtual ~OptionManager();

  // ######################################################################
  /*! @name Command-line options */
  //@{

  //! ModelComponent objects may request a command-line option here
  /*! @param p the ModelParam data member of c corresponding to the
             option

      @param useMyVal if true, we will do a setOptionValString() using
             the current value of the parameter, so that this value
             will be propagated to all OptionedModelParam objects that
             have requested this option and will become the new
             command-line default value. If false, the ModelParam p
             will be updated to the current value of the option. So,
             use useMyVal=true if your component wishes to push its
             current parameter value as the default for everyone using
             this option, and use false if your component wishes to
             use the current default instead. */
  virtual void requestOption(OptionedModelParam& p,
                             const bool useMyVal = false) = 0;

  //! Request the removal of a param from the command-line options
  /*! @param p the ModelParam data member of c corresponding to the option */
  virtual void unRequestOption(OptionedModelParam& p) = 0;

  //! Users may request model option aliases
  /*! @param name the name of the option alias, from ModelOptionDefs.C */
  virtual void requestOptionAlias(const ModelOptionDef* def) = 0;

  //! Set an option value
  /*! All ModelParam objects associated this ModelOptionDef will be
      updated with the new value. Will throw a fatal error if the
      model has been started (see ModelComponent::start()). */
  virtual void setOptionValString(const ModelOptionDef* def,
                                  const std::string& val) = 0;

  //! Get an option value
  /*! The value returned here is the value currently held by the
      ModelManager. This value is updated in three cases: 1) when a
      new component requests the option using requestOption() with
      useMyVal true; 2) when setOptionValString() is called, and 3)
      when the command-line arguments are parsed by ParseCommandLine,
      if a value for this option has been specified in the command
      line. If ModelComponent objects modify their local ModelParam
      values for their local parameter with this name, the value
      returned here will not be affected. */
  virtual std::string getOptionValString(const ModelOptionDef* def) = 0;

  //! Check if anybody has requested the given option.
  virtual bool isOptionRequested(const ModelOptionDef* def) const = 0;

  //@}

  // ######################################################################
  /*! @name Alternative access to internal options

      These functions may be useful for alternative methods of
      displaying the help for command-line options (e.g. in the Qt
      ModelManagerWizard).
   */
  //@{

  //! Get the number of ModelOptionDef objects in our list.
  virtual uint numOptionDefs() const = 0;

  //! Get ModelOptionDef objects from our list, up to the size of the given array.
  /*! Returns the number of array entries that were actually filled
    with valid ModelOptionDef pointers. */
  virtual uint getOptionDefs(const ModelOptionDef** arr, uint narr) = 0;

  //! Get the ModelOptionDef for the given name.
  /*! Aborts if there is no such ModelOptionDef. */
  virtual const ModelOptionDef* findOptionDef(const char* name) const = 0;

  //! Query if the given ModelOptionDef is used in this program.
  virtual bool isOptionDefUsed(const ModelOptionDef* def) const = 0;

  //@}
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPONENT_OPTIONMANAGER_H_DEFINED
