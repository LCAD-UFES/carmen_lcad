/*!@file Component/ModelParamBase.H A tunable ModelComponent parameter base class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelParamBase.H $
// $Id: ModelParamBase.H 14376 2011-01-11 02:44:34Z pez $
//

#ifndef MODELPARAMBASE_H_DEFINED
#define MODELPARAMBASE_H_DEFINED

#include "Component/ParamFlags.H"

#include <string>
#include <typeinfo>
#include <cstddef>

class ModelOptionDef;
class ParamMap;

//! Base class for TRefHolder
class RefHolder
{
public:
  RefHolder();
  virtual ~RefHolder();

  virtual const std::type_info& type() const = 0;

private:
  RefHolder(const RefHolder&); // not implemented
  RefHolder& operator=(const RefHolder&); // not implemented
};

//! Template implementation of RefHolder
/*! Essentially this class just provides a way to dynamic_cast to any
    type. We do this by wrapping the type in a TRefHolder, which is
    dynamic_cast-able by virtue of having a virtual function. Thus we
    can construct for example a TRefHolder<int>, pass it somewhere as
    a RefHolder, and then somewhen can later dynamic_cast it back to a
    TRefHolder<int> and get the original int& back out. */
template <class T>
class TRefHolder : public RefHolder
{
public:
  TRefHolder(T& r) : ref(r) {}
  virtual ~TRefHolder() {}

  virtual const std::type_info& type() const { return typeid(T); }

  T& ref;
};

// ######################################################################
//! ModelParamBase is a persistent ModelComponent parameter base class
/*! Various model parameter types can be derived from this base, see
    the NModelParam and OModelParam template class
    definitions. ModelParamBase is persistent in the sense that it can
    be saved/loaded into a ParamMap, which itself provides disk
    save/load capabilities. ModelParamBase is intended to be used as a
    data member in a ModelComponent (or derived)
    object. ModelComponent provides an interface by which
    ModelParamBase derivatives may also be tuned via command-line
    options. */
class ModelParamBase
{
public:
  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  //! Constructor
  /*! @param flags Will be checked for ALLOW_ONLINE_CHANGES to set
      up our return value from allowsOnlineChanges() appropriately. */
  ModelParamBase(const ParamFlag flags);

  //! Destructor
  virtual ~ModelParamBase();

  //@}

  // ######################################################################
  /*! @name Basic access functions */
  //@{

  //! get the parameter's name
  virtual std::string getName() const = 0;

  //! get the parameter's name with spaces separating words
  /*! This function decodes the CamelCase string that is typically
      returned by getName() into space-separated words, using some
      heuristics to figure out where to place word boundaries. It is
      not guaranteed to be perfectly accurate in finding word
      boundaries; this result is only intended for user-interface
      display to the end user and should not be relied on for
      programmatic usage. */
  virtual std::string getNameWithSpaces() const;

  //! Get current value as a string (must be implemented by derived classes)
  virtual std::string getValString() const = 0;

  //! Set the parameter value from a textual representation, notify client of the change
  /*! @return true if the change succeeded; false otherwise. */
  virtual bool setValString(const std::string& textval) = 0;

  //! Get the current value through a dynamically-typed RefHolder.
  virtual void getValGeneric(RefHolder& ref) const = 0;

  //! Set the current value through a dynamically-typed RefHolder, notify client of the change
  /*! @return true if the change succeeded; false otherwise. */
  virtual bool setValGeneric(const RefHolder& ref) = 0;

  //! Check whether this param is allowed to have its value changed while the model is active
  bool allowsOnlineChanges() const { return itsFlags & ALLOW_ONLINE_CHANGES; }

  //! Change the param's active/inactive setting
  void setInactive(bool v) { if (v) itsFlags |= MP_INACTIVE; else itsFlags &= ~MP_INACTIVE; }

  //! Check the param's active/inactive setting
  bool getInactive() const { return itsFlags & MP_INACTIVE; }

  //! Change the param's hidden/visible setting
  void setHidden(bool v) { if (v) itsFlags |= MP_HIDDEN; else itsFlags &= ~MP_HIDDEN; }

  //! Check the param's hidden/visible setting
  bool getHidden() const { return itsFlags & MP_HIDDEN; }

  //@}

  // ######################################################################
  /*! @name Input/Output functions */
  //@{

  //! Print out our name and contents, mostly for debugging
  virtual void printout(std::ostream& s,
                        const std::string& prefix = "") const = 0;

  //! Write parameter value to ParamMap
  virtual void writeTo(ParamMap& pmap) const = 0;

  //! Get parameter value from ParamMap, notify client of the value change
  /*! @param noerr will not generate an error message if the parameter
    does not exist in ParamMap. */
  virtual void readFrom(const ParamMap& pmap,
                        const bool noerr = true) = 0;

  //@}

private:
  ModelParamBase(const ModelParamBase& m);      //!< forbid copy-contruction
  ModelParamBase& operator=(const ModelParamBase& m); //!< forbid copy

  ParamFlag itsFlags;
};

// ######################################################################
//! OptionedModelParam extends ModelParamBase for params associated with a ModelOptionDef
class OptionedModelParam : public ModelParamBase
{
public:
  OptionedModelParam(const ParamFlag flags);
  virtual ~OptionedModelParam();

  virtual const ModelOptionDef* getOptionDef() const = 0;
};

#endif // !MODELPARAMBASE_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
