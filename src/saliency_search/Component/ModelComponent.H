/*!@file Component/ModelComponent.H base class for parameterized model components */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelComponent.H $
// $Id: ModelComponent.H 11019 2009-03-11 20:57:11Z itti $
//

#ifndef MODELCOMPONENT_H_DEFINED
#define MODELCOMPONENT_H_DEFINED

#include "Component/ParamClient.H"
#include "Component/ModelParamBase.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "nub/object.h"
#include "nub/ref.h"
#include "rutz/shared_ptr.h"

#include <iosfwd>
#include <string>

//! A convenience function for making a rutz::shared_ptr out of a raw pointer.
template <class T>
inline nub::ref<T> makeSharedComp(T* t) { return nub::ref<T>(t); }

class ParamMap;
class OptionManager;

//! Type for bitwise-or'ed combinations of ModelComponent flags
typedef int ModelFlag;

//! placeholder for "no flags"
const ModelFlag MC_NO_FLAGS = 0;

//! do operations recursively on subcomponents
const ModelFlag MC_RECURSE = (1 << 0);

//! ignore missing param values, if possible (not all ops allow this flag)
const ModelFlag MC_IGNORE_MISSING = (1 << 1);

//! Provide some information about howto save() things
/*! Base class is empty, but derivations of ModelComponent, like, for
  example SimModule and derivatives, may use a derivation of
  ModelComponentSaveInfo which actually contains some info: */
class ModelComponentSaveInfo {
public:
  //! Default constructor
  ModelComponentSaveInfo() { }

  //! Virtual destructor to ensure safe destruction of derived objects
  virtual ~ModelComponentSaveInfo() { }
};

//! Base class for parameterized model components
/*! The goal is to provide a unified interface by which tunable
  parameters can be set, read, or exported as command-line options or
  in parameter maps or files, so that executables using a variable
  collection of model components can easily be built and configured
  using command-line options and config files. To this end,
  ModelComponent provides a set of facilities that allow it to hold a
  number of ModelParamBase data members. A ModelComponent also holds a
  list of sub-components that are ModelComponent as well, and many of
  the ModelComponent functions will first apply to the component
  itself, then recursively to its subcomponents. The ModelParamBase
  are the tunable parameters that are persistent and may be associated
  with a command-line option; see ModelParam.H for details. A
  ModelComponent is attached to a OptionManager that will handle
  option parsing and ModelParamBase load/save. A ModelComponent may
  request its master to associate some of its ModelParamBase data
  members with command-line options. Command-line option definitions
  are typically given in one master file per source directory
  (e.g. Devices/DeviceOpts.C, Media/MediaOpts.C, Neuro/NeuroOpts.C),
  although some components may also define private command-line
  options internally in their own .C files. Command-line parsing will
  set the value of ALL ModelParamBase objects in all ModelComponent
  objects and their subcomponents that have requested the option to
  the value specified on the command line. So this mechanism should be
  used sparingly, for parameters that may be used across a variety of
  models and a variety of ModelComponent derivatives.

  There is one trick to be aware of with respect to the basic
  operation and setup of ModelComponent. For speed reasons, we want
  our ModelParam parameters to be true data members, so that we can
  access them as efficiently as normal data members (rather than
  having to first look them up by name). On the other hand, the
  ModelComponent must also be able to produce a list of all its
  ModelParam members, so that they can be saved to a ParamMap. To
  resolve this, ModelComponent both holds each ModelParam as a regular
  data member, and a list of pointers to all its ModelParam
  members. In addition, this list keeps track of which model params
  are affiliated with command-line options, as well as which should
  actually be exported on the command-line. This list is automatically
  maintained when the ModelParam objects are constructed and
  destroyed, so that the user does not have to worry about it. */
class ModelComponent : public ParamClient, public virtual nub::object
{
public:
  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  //! Constructor
  /*! Typically, in the constructor you should perform all
    initializations that are independent of the values of your
    ModelParam data members. Since the command-line usually has not
    been parsed yet at the time you construct your various
    ModelComponent objects, however, the start() function below is
    provided to allow you to perform additional initializations after
    the command-line has been parsed and the ModelParam data members
    have been configured.
    @param mgr our option manager; see OptionManager.H
    @param descrName a human-readable descriptive name
    @param tagname a computer-oriented unique name (used for storage in ParamMap)
    @param crealm the realm in which this component lives, used for partitioning
      the SimEvent space into possibly distinct realms. */
  ModelComponent(OptionManager& mgr, const std::string& descrName,
                 const std::string& tagName,
                 const std::string& crealm = "World");

  //! Constructor overload without an OptionManager
  /*! In this case, you must call setManager() before using any other
      functions. In fact, if you aren't ModelManager, you probably
      shouldn't be using this constructor! (ModelManager needs it due
      to an implementation quirk related to multiple inheritance). */
  ModelComponent(const std::string& descrName,
                 const std::string& tagName,
                 const std::string& crealm = "World");

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~ModelComponent();

  //! Start the component
  /*! This will start the component by triggering the following
    sequence of actions:
        - Check if we are already started and emit a fatal error if so
        - Call start1() that may be overloaded by derived classes
        - Call start() on all our subcomponents
        - Call start2() that may be overloaded by derived classes
        - Switch us to "started" state, in which modifying our ModelParam
          values externally is forbidden

    Derived classes should overload start1() and start2() as a way to
    get the component started, once the command-line options and
    config files have been parsed and all the ModelParam objects have
    been configured. Default no-op implementations are provided for
    start1() and start2(), so if you don't need to do anything to
    start your component, you don't need to implement start1() and
    start2(). Because this is a recursive call on our sub-components,
    typically you only need to call it once on your root
    ModelComponent (probably ModelManager) to get your entire
    hierarchy of ModelComponent objects started. */
  void start();

  //! Did we already get a call to start()?
  bool started() const;

  //! Stop the component
  /*! This will stop the component by triggering the following
    sequence of actions:
        - Check if we are already started and emit a fatal error if not
        - Call stop1() that may be overloaded by derived classes
        - Call stop() on all our subcomponents
        - Call stop2() that may be overloaded by derived classes
        - Switch us out of "started" state, so that modifying our ModelParam
          values externally is allowed

    Derived classes should overload stop1() and stop2() as a way to
    get the component stopped, once the command-line options and
    config files have been parsed and all the ModelParam and objects
    have been configured. Default no-op implementations are provided
    for stop1() and stop2(), so if you don't need to do anything to
    stop your component, you don't need to implement stop1() and
    stop2(). Because this is a recursive call on our sub-component,
    typically you only need to call it once on your root
    ModelComponent (probably ModelManager) to get the entire hierarchy
    of ModelComponent objects stopped. */
  void stop();

  //! Signify that our ModelManager just got destroyed
  /*! This should not be called manually, but will be called from
      within our parent ModelComponent's destructor. It will ensure
      that we do not try to unregister with our ModelManager if the
      manager is destroyed before us. */
  void managerDestroyed();

  //! Get our manager.
  OptionManager& getManager() const;
  //@}

  // ######################################################################
  /*! @name Name functions */
  //@{

  //! Get a human-readable descriptive name for this component.
  /*! This is NOT intended to be a name for programmatic use (such as
      for indexing into associative arrays). It is not necessarily
      unique on a per-object basis; it's possible that all objects of
      the same class return the same descriptiveName(). */
  std::string descriptiveName() const;

  //! Change this component's descriptive name.
  void setDescriptiveName(const std::string& name);

  //! Get a programmer-usable tag name for this object.
  /*! This is intended to be unique among component objects, so that
      this string could be used for associate-array indexing, for
      example. */
  std::string tagName() const;

  //! Change this component's tag name.
  void setTagName(const std::string& name);

  //! Get the realm
  std::string realm() const;

  //! Set the realm
  /*! This will throw a fatal exception if the component is in started
      state. This is a recursive call which will propagate to all
      subcomponents. */
  void setRealm(const std::string& crealm);

  //@}

  // ######################################################################
  /*! @name Sub-Component management functions */
  //@{

  //! Get this component's parent ModelComponent, or NULL if it has no parent.
  /*! Internally, each component stores a weak smart pointer to its
      parent, while the parent stores a strong smart pointer to the
      child. This avoids unbreakable reference-counting cycles, but
      still lets us traverse the parent-child in both directions
      component tree using getParent() and subComponent(). */
  ModelComponent* getParent() const;

  //! Get this component's root parent (the parent of all parents in our tree)
  /*! If this component has no parent, then getRootObject() returns a
      pointer to this component itself. */
  ModelComponent* getRootObject();

  //! Get this component's root parent (the parent of all parents in our tree)
  /*! If this component has no parent, then getRootObject() returns a
      pointer to this component itself. */
  const ModelComponent* getRootObject() const;

  //! Add a subcomponent to our internal list; return its index
  /*! NOTE that by default this will set the realm of the subcomponent (and its subs) to our realm */
  uint addSubComponent(const nub::ref<ModelComponent>& subc, const bool propagate_realm = true);

  //! Remove a subcomponent from our internal list, by address
  /*! This will only remove the component from our list of registered
      subcomponents, but will not attempt to destroy it. If removeall
      is true, then all components matching subc are removed;
      otherwise, only the first is removed. Returns the number of
      subcomponents that were removed. */
  int removeSubComponent(const ModelComponent& subc, bool removeall = false);

  //! Remove a subcomponent by nub:ref.
  void removeSubComponent(const nub::ref<ModelComponent>& subc);

  //! Remove a subcomponent from our internal list, by index
  /*! This will only remove the component from our list of registered
    subcomponents, but will not attempt to destroy it. */
  void removeSubComponent(const uint idx);

  //! Remove a subcomponent from our internal list, by tagName
  /*! This will only remove the component from our list of registered
    subcomponents, but will not attempt to destroy it. */
  void removeSubComponent(const std::string& tagname);

  //! Remove all our subcomponents
  void removeAllSubComponents();

  //! Access a sub-component by index
  nub::ref<ModelComponent> subComponent(const uint idx) const;

  //! Access a sub-component by tagname
  /*! @param flags if containing MC_RECURSE, will look not only in our
    subcomponents but also recursively in their subcomponents
    (depth-first search)*/
  nub::ref<ModelComponent> subComponent(const std::string& tagname,
                                        const ModelFlag flags = 0) const;

  //! Get current number of subcomponents
  uint numSubComp() const;

  //! Return true if we have a subcomponent by that tagname
  /*! @param flags if containing MC_RECURSE, will look not only in our
    subcomponents but also recursively in their subcomponents
    (depth-first search)*/
  bool hasSubComponent(const std::string& tagname,
                       const ModelFlag flags = 0) const;

  //! Return true if we have the pointee as subcomponent
  /*! @param flags if containing MC_RECURSE, will look not only in our
    subcomponents but also recursively in their subcomponents
    (depth-first search)*/
  bool hasSubComponent(const nub::soft_ref<ModelComponent>& c,
                       const ModelFlag flags = 0) const;

  //! Show our ModelParam internals and those of our subcomponents
  void printout(std::ostream& s, const std::string& prefix = "") const;

  //! Reset a model component and propagate the request to the sub-components
  /*! For each ModelComponent reset should set it into the state that
      it was just after calling the constructor. It will depend on the
      component what exactly that involves.

      @param flags if containing MC_RECURSE, then we recursively
             reset() our subcomponents as well

      PROGRAMMER NOTE: This function is not virtual, so you should not
      try to override this function in your derived class; instead
      override reset1() and/or reset2(). In those overrides, be sure
      to call your base class's version of reset1() or reset2().
  */
  void reset(const ModelFlag flags);

  //! Save/display our current state
  /*! Over the course of a program run, once in a while it may be
    desirable to broadcast an order to all ModelComponents to save
    and/or display their current state or internals. This function
    achieves this. Like for start(), stop() and reset(), this is a
    usually recursive call, and derived classes should not override this
    function, but should instead override save1() (called before we
    recurse through our subcomponents) and save2() (called after we
    have recursed through our subcomponents). Default implementations
    for save1() and save2() are provided which are no-ops. */
  void save(const ModelComponentSaveInfo& sinfo,
            const ModelFlag flags = MC_RECURSE);

  //@}

  // ######################################################################
  /*! @name Access to tunable ModelParam's */
  //@{

  //! Export some of our ModelParam parameters as command-line-options
  /*! This should be called once on the model manager to decide which
      sets of ModelParam paremeters should become command-line
      options; the call will then recurse through all ModelComponent
      objects that have been registered with the manager. This is
      provided so that external meta-knowledge about what a collection
      of ModelComponent objects forming a computational model can be
      used to decide on which command-line options are relevant: for
      instance, if you build a model that has no OutputFrameSeries,
      then you will not be able to call saveResults() on your Brain or
      other objects, so you would not want to export command-line
      options that relate to which results should be saved. This is
      handled by the exportFlag member of each ModelOptionDef, which
      must match the ModelManager's current export mask (set by
      ModelManager::allowOptions()) in order for that option to
      actually be exported.

      HISTORICAL NOTE: It used to be that every subclass of
      ModelComponent would need to override exportOptions(), and
      explicitly request that each model param be exported. This is no
      longer the case -- instead, when each OModelParam is constructed
      in the subclass's constructor, it ends up calling
      registerOptionedParam() on the ModelComponent, which adds the
      param to our list of params, but also marks the param for
      command-line export whenever exportOptions() is finally
      called. For those rare cases where you do not want an
      OModelParam exported to the command-line, you can call
      forgetExports(), and then explicitly re-export any desired
      options using doRequestOption().
  */
  void exportOptions(const ModelFlag flags);

  //! Check for named param in this component (and possibly its subcomponents)
  /*! Use this to verify that a param exists before doing a
      getModelParamVal() or setModelParamVal(), since those functions
      will throw an exception if the named param is non-existent. */
  bool hasModelParam(const std::string& name, const ModelFlag flags = 0) const;

  //! Parse a parameter value given as string and set the param
  /*! Should not be needed in normal operation, inefficient text-based
      interface. In normal operation, the internals of the
      ModelComponent derivative will directly access the ModelParam,
      not external people. If you want to provide external access to
      your ModelParam values, you should implement a wrapper for them
      in your ModelComponent derivative (e.g., implement a function
      setSize() that will set the value of your Size ModelParam).
      Returns true if the ModelParam was found and set. Throws a fatal
      error if the ModelComponent was started, to avoid external
      people messing around with our ModelParam values during
      operation of the component.

      @param name the ModelParam tagname
      @param value the parameter value, to be parsed
      @param flags bitwise-or'ed combination of MC_RECURSE, MC_IGNORE_MISSING

      HISTORICAL NOTE: In the past, subclasses would override
      setModelParamString() if they needed special handling when param
      values changed, this is no longer the case -- instead subclasses
      should override paramChanged(); see additional documentation
      there.
  */
  void setModelParamString(const std::string& name,
                           const std::string& value,
                           const ModelFlag flags = 0);

  //! Set a parameter value
  /*! This is slow, as it will search for the ModelParam by name. If
      you need efficient access to ModelParam values, you should
      define a wrapper method in your ModelComponent derivative. In
      normal operation, indeed, only the methods of the ModelComponent
      derivative should directly access the ModelParam, not external
      people. Returns true if the ModelParam was found and set. Throws
      a fatal error if the ModelComponent was started, to avoid
      external people messing around with our ModelParam values during
      operation of the component.

      This function will set the param value for ALL matching model
      params found in us (and in our subcomponents, if flags contains
      MC_RECURSE).

      If the parameter is not found in us (or in any of our
      subcomponents, if flags contains MC_RECURSE), this function will
      not return (will abort or throw an exception) -- to avoid such
      an error, call hasModelParam() first to check that the
      particular param exists.

      @param name the ModelParam tagname
      @param val the parameter value
      @param flags bitwise-or'ed combination MC_RECURSE, MC_IGNORE_MISSING */
  template <class T> inline
  void setModelParamVal(const std::string& name, const T& val,
                        const ModelFlag flags = 0);

  //! Auxiliary implementation function for setModelParamVal()
  void setModelParamValAux(const std::string& name,
                           const RefHolder& val,
                           const ModelFlag flags);

  //! Get a parameter value
  /*! Because the parameter may exist both in us and in several of our
      subcomponents, we will stop and return the value of the first
      instance encountered, searching first in us, then recursively in
      our subcomponents (depth-first search, if flags contains
      MC_RECURSE), in the order in which they were added.

      If the parameter is not found in us (or in any of our
      subcomponents, if flags contains MC_RECURSE), this function will
      not return (will abort or throw an exception) -- to avoid such
      an error, call hasModelParam() first to check that the
      particular param exists.

      It is ok to call this on a started ModelComponent. */
  std::string getModelParamString(const std::string& name,
                                  const ModelFlag flags = 0) const;

  //! Get a parameter value
  /*! This is slow, as it will search for the ModelParam by name. If
      you need efficient access to ModelParam values, you should
      define a wrapper method in your ModelComponent derivative. In
      normal operation, indeed, only the methods of the ModelComponent
      derivative should directly access the ModelParam, not external
      people. Because the parameter may exist both in us and in
      several of our subcomponents, we will stop and return the value
      of the first instance encountered, searching first in us, then
      recursively in our subcomponents (depth-first search, if flags
      contains MC_RECURSE), in the order in which they were added.

      If the parameter is not found in us (or in any of our
      subcomponents, if flags contains MC_RECURSE), this function will
      not return (will abort or throw an exception) -- to avoid such
      an error, call hasModelParam() first to check that the
      particular param exists.

      It is ok to call this on a started ModelComponent. */
  template <class T> inline
  T getModelParamVal(const std::string& name, const ModelFlag flags = 0) const;

  //! Auxiliary implementation function for getModelParamVal()
  void getModelParamValAux(const std::string& name,
                           RefHolder& val,
                           const ModelFlag flags) const;

  //! External request that one of our ModelParam becomes an option
  /*! This is to allow external callers to selectively decide that
    some of our internal ModelParam parameters should be exported as
    command-line options. If recurse is true, will look for the
    ModelParam in all our subcomponents, and will link all matching
    ModelParam in us and our subcomponents to the option (so that if a
    command-line value is provided, it will affect all matching
    ModelParam objects in us and all our subcomponents). Will return
    false (and generate a warning message if warn is true) if we (or
    our subcomponents if recurse is true) don't have a ModelParam of
    the specified name. Will generate a fatal error if we have the
    ModelParam but no known command-line option exists for that name
    in ModelOptionDefs.C. Returns true if the ModelParam was found and
    added as a command-line option. Throws a fatal error is we are
    started. */
  bool doRequestOption(const ModelOptionDef* opt, const bool useMyVal = false,
                       const bool recurse = true, const bool warn = true);

  //! External request that we hide a command-line option
  /*! In some rare cases (see, e.g., VisualCortexConfigurator), we
      want to create a component but avoid exporting some of its
      ModelParam members as command-line options. Use with caution,
      since the ModelParam still looks and feels like an option, just
      the ModelManager will ignore it. This must be called immediately
      after construction of the ModelComponent and before
      exportOptions() is called (either explicitly or automatically
      when the command-line is parsed). Throws a fatal error if
      exportOptions() has already been called, we don't have the
      option, or we are started. */
  void hideOption(const ModelOptionDef* opt);

  //! Return the number of params that we have
  size_t getNumModelParams() const;

  //! Get non-const access to the i'th model parameter
  const ModelParamBase* getModelParam(size_t i) const;

  //! Get non-const access to the i'th model parameter
  ModelParamBase* getModelParam(size_t i);

  //@}

  // ######################################################################
  /*! @name ParamMap-based configuration functions */
  //@{

  //! Read params from the ParamMap
  /*! Throws a fatal error is we are started.
      @param noerr will not generate an error message if some of our
             parameters do not exist in ParamMap */
  void readParamsFrom(const ParamMap& pmap, const bool noerr = true);

  //! Write params to the ParamMap
  /*! Prints out a warning message if we are started. */
  void writeParamsTo(ParamMap& pmap) const;

  //@}

  // ######################################################################
  /*! @name ParamClient interface */
  //@{

  //! our parameters will register with us upon construction
  /*! override of ParamClient's pure virtual function */
  virtual void registerParam(ModelParamBase* mp);

  //! Our parameters will register with us upon construction
  /*! @param flags Pass USE_MY_VAL here if you want the current value
      of the model param to be taken as the new default value,
      otherwise pass 0 for flags. */
  virtual void registerOptionedParam(OptionedModelParam* mp,
                                     const ParamFlag flags);

  //! our parameters will un-register with us upon destruction
  /*! override of ParamClient's pure virtual function */
  virtual void unregisterParam(const ModelParamBase* mp);

  //! Called whenever a ModelParamBase has its value changed
  /*! Subclasses of ModelComponent should override this function if
      they need to do any internal reconfiguration when their
      parameters change value (whether due to a command-line option,
      or a setModelParamString(), or a readParamsFrom()). See
      OrientationChannel::paramChanged() in
      Channels/OrientationChannel.C for an example; also see
      SaccadeControllerConfigurator::paramChanged() in
      Neuro/SaccadeControllers.C for for how to use this to select
      subcomponents at runtime.

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

      HISTORICAL NOTE: In the past, subclasses would override
      setModelParamString() for this same purpose, but in the new
      setup, setModelParamString() is NOT virtual and hence should not
      be (cannot be) overridden by subclasses. This setup is cleaner
      (and potentially more efficient) because subclasses don't have
      to compare string names to figure out which param changed;
      instead they are passed the address of the param
      itself. Furthermore, the new setup is more robust, since we can
      handle param value changes that come from any source (e.g.,
      setModelParamVal(), or ModelParamBase::setValString(), or
      OModelParam::setVal()), and not just those that come through
      setModelParamString().
  */
  virtual void paramChanged(ModelParamBase* param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Clear our list of command-line options to be exported
  void forgetExports();

  //@}

protected:
  //! Set our manager.
  void setManager(OptionManager& mgr);

  //! Check if our options have already been exported.
  bool hasBeenExported() const;

  //! This is called from within start() before the subcomponents start
  virtual void start1();

  //! This is called from within start() after the subcomponents have started
  virtual void start2();

  //! This is called from within stop() before the subcomponents stop
  virtual void stop1();

  //! This is called from within stop() after the subcomponents have stopped
  virtual void stop2();

  //! This is called from within reset() before the subcomponents are reset
  virtual void reset1();

  //! This is called from within reset() after the subcomponents are reset
  virtual void reset2();

  //! This is called from within save() before the subcomponents save
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! This is called from within save() after the subcomponents have saved
  virtual void save2(const ModelComponentSaveInfo& sinfo);

  //! Default constructor without arguments. DO NOT USE!
  /*! This constructor exists only so that we don't have to explicitly
    call the parameterized ModelComponent constructor in each and
    every object that has a ModelComponent virtual base somewhere in
    its inheritance hierarchy. This constructor, however, will leave
    the object uninitialized and you must call init() before you can
    use the object. Here is what you should do when building an
    inheritance hierarchy that has ModelComponent as virtual base: 1)
    in the constructor of the class that virtually derives from
    ModelComponent, call init(). See for example Channels/ChannelBase;
    2) in classes that concretely derive from the latter one, just
    construct as usual; see for example Channels/SingleChannel,
    Channels/BlueChannel, etc. */
  ModelComponent();

  //! Make sure you call this if you have used the default constructor
  void init(OptionManager& mgr, const std::string& descrName,
            const std::string& tagName, const std::string& crealm = "World");

private:
  class Impl;
  Impl* const rep;
};

// ######################################################################
// #################### INLINED METHODS:
// ######################################################################

template <class T> inline
void ModelComponent::setModelParamVal(const std::string& name,
                                      const T& val,
                                      const ModelFlag flags)
{
  TRefHolder<const T> ref(val);
  return setModelParamValAux(name, ref, flags);
}

// ######################################################################
template <class T> inline
T ModelComponent::getModelParamVal(const std::string& name,
                                   const ModelFlag flags) const
{
  T val;
  TRefHolder<T> ref(val);
  getModelParamValAux(name, ref, flags);
  return val;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
