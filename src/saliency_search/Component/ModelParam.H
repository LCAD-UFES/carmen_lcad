/*!@file Component/ModelParam.H A template ModelComponent parameter class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelParam.H $
// $Id: ModelParam.H 8782 2007-09-20 22:34:54Z rjpeters $
//

#ifndef MODELPARAM_H_DEFINED
#define MODELPARAM_H_DEFINED

#include "Component/ModelParamBase.H"
#include "Component/ParamClient.H"
#include "Component/ParamFlags.H"
#include "Util/StringConversions.H"
#include "rutz/fileposition.h" // for SRC_POS
#include "rutz/mutex.h"
#include "rutz/shared_ptr.h"
#include "rutz/stderror.h" // for rutz::throw_bad_cast()

#include <iosfwd>
#include <string>
#include <typeinfo>

#include <pthread.h>

class ModelOptionDef;
class ParamMap;

// ######################################################################
//! Helper implementation class for NModelParam and OModelParam
class ModelParamAuxImpl
{
public:
  //! Construct from a ModelOptionDef*
  /*! This model param will get its name from the ModelOptionDef*.

      @param self pointer to the object that is using this impl
      @param client the ParamClient we are attached to
      @param nam the name of the TModelParamBase, that will be used to
         store its value in a ParamMap and fetch command-line option
         configuration data.
      @param flags can be set to USE_MY_VAL in order to take this
         param's value as the new default value, otherwise pass 0 for flags
      @param valtype the typeid of the parameter; we will check to
         make sure this matches the typeid declared by the
         ModelOptionDef
  */
  ModelParamAuxImpl(OptionedModelParam* self,
                    const ModelOptionDef* def,
                    ParamClient* client,
                    const ParamFlag flags,
                    const std::type_info& valtype);

  //! Construct with a string name only
  /*! This model param will NOT be associated with any ModelOptionDef
      -- to get that, use the other constructor that takes a
      ModelOptionDef* instead of a name.

      @param self pointer to the object that is using this impl
      @param client the ParamClient we are attached to
      @param nam the name of the ModelParamAuxImpl, that will be used to
         store its value in a ParamMap and fetch command-line option
         configuration data. */
  ModelParamAuxImpl(ModelParamBase* self,
                    const std::string& nam, ParamClient* client);

  //! Destructor
  ~ModelParamAuxImpl();

  //! get the ModelParamAuxImpl's name
  std::string getName() const;

  //! get the associated option def
  const ModelOptionDef* getOptionDef() const;

  //! Print out our name and contents, mostly for debugging
  void printout(std::ostream& s, const std::string& prefix) const;

  //! Write parameter value to ParamMap
  void writeTo(ParamMap& pmap) const;

  //! Get parameter value from ParamMap
  /*! @param noerr will not generate an error message if the parameter
      does not exist in ParamMap. */
  void readFrom(const ParamMap& pmap, const bool noerr);

  //! Call paramChanged() on our client
  ParamClient::ChangeStatus sendChangedMessage(bool didchange);

  //! Convenience function to retrieve def->defval.
  /*! The reason we have this here is so that we don't have to
      #include ModelOptionDef.H from this widely-used header file. */
  static const char* defaultValueOf(const ModelOptionDef* def);

  //! Get a mutex for locking the parameter value for reading
  /*! We only return an actual mutex if we were passed
      ALLOW_ONLINE_CHANGES in our constructor (i.e.,
      ModelParamBase::allowsOnlineChanges() returns true); that's
      because we assume that if online parameter changes are disabled,
      then there is no need for between-thread synchronization of our
      parameter value.

      The returned mutex, if non-null, will be a recursive mutex so
      that we can allow reentrant calls to getVal() from within a
      setVal() in the same thread, while still blocking getVal() calls
      from other threads during a setVal() call.
  */
  pthread_mutex_t* readLock() const
  { return itsLocks ? &itsLocks[READLOCK] : 0; }

  //! Get a mutex for locking the parameter value for writing
  /*! We only return an actual mutex if we were passed
      ALLOW_ONLINE_CHANGES in our constructor (i.e.,
      ModelParamBase::allowsOnlineChanges() returns true); that's
      because we assume that if online parameter changes are disabled,
      then there is no need for between-thread synchronization of our
      parameter value.

      The returned mutex, if non-null, will be an error-checking mutex
      so that we can detect attempts to acquire the lock more than
      once from within the same thread. These attempts would represent
      reentrant setVal() calls, such as when setVal() triggers a
      paramChanged() call that in turn triggers another setVal() of
      the same parameter (which is forbidden by design).
  */
  pthread_mutex_t* writeLock() const
  { return itsLocks ? &itsLocks[WRITELOCK] : 0; }

private:
  // Yes, there is no parameter value data proper and no way to get
  // the parameter value directly! This is only a helper class. The
  // NModelParam and OModelParam classes provide concrete paramter
  // values.

  //! indices into the optional 2-element itsLocks array
  enum { READLOCK = 0, WRITELOCK = 1 };

  ModelParamBase*       const itsSelf;   //!< our owner
  ParamClient*          const itsClient; //!< keep track of our client
  ModelOptionDef const* const itsOption; //!< may be null
  std::string           const itsName;   //!< the parameter name as used in ParamMap
  pthread_mutex_t*      const itsLocks;  //!< optional 2-element array of {read,write} locks for getVal()/setVal()
  bool                        itsInCallback; //!< whether we're currently doing a paramChanged() callback

  ModelParamAuxImpl(const ModelParamAuxImpl& m);      //!< forbid copy-contruction
  ModelParamAuxImpl& operator=(const ModelParamAuxImpl& m); //!< forbid copy
};

// ######################################################################
//! Helper class to provide transactional semantics for a value change
/*! The provisionally assigns a new value to a variable, but upon
    destruction (e.g. at block exit), the original value is restored
    unless the transaction has been explicitly accepted. */
template <class T>
class ValueChangeTransaction
{
public:
  ValueChangeTransaction(T* p, const T& newval)
    : oldval(*p), var(p), accepted(false)
  {
    *var = newval;
  }

  ~ValueChangeTransaction()
  {
    if (!accepted)
      // Note, if this destructor is triggered because of exception
      // propagation, and the *var=oldval assignment also causes an
      // exception, then we will crash with std::terminate(). In
      // practice, this shouldn't be much of a problem since
      // operator=() should generally be exception-free for the kinds
      // of types that are held in model params.
      *var = oldval;
  }

  void acceptChange() { accepted = true; }

private:
  T const oldval;
  T* const var;
  bool accepted;
};

// ######################################################################
//! A class for params of type T that don't have command-line options
/*! NModelParam delegates its implementation to
    ModelParamAuxImpl. NModelParam<T> offers getVal() and setVal() for
    direct access to the T value.

    Note that the implementation is entirely inline so that we don't
    have to explicitly instantiate NModelParam<T> anywhere; plus these
    functions are small so any compile time hit from having the
    functions inline is small.

    Note that we have two similar template types: NModelParam and
    OModelParam. The 'O' is for "command-line Option", and the 'N' is
    for "Not a command-line option". NModelParam only implements
    ModelParamBase and is constructed with a string for its parameter
    name. On the other hand, OModelParam implements
    OptionedModelParam, which means that it offers getOptionDef(), and
    it takes a ModelOptionDef* in its constructor instead of a string
    (the parameter name is taken from the ModelOptionDef).
*/
template <class T> class NModelParam : public ModelParamBase
{
public:
  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  //! Construct with a string name only
  /*! Same as ModelParamAuxImpl plus an initial parameter value

      @param flags If flags contains ALLOW_ONLINE_CHANGES, then this
      param will be marked as being able to be changed while the model
      is active.
  */
  inline NModelParam(const std::string& nam, ParamClient* client,
                     const T& initval, const ParamFlag flags = 0);

  //! A "pseudo-constructor" that makes a rutz::shared_ptr<NModelParam>.
  /*! This can be used in places that need arrays of NModelParam
      objects. Since NModelParam doesn't have a default constructor, we
      can't make an array of NModelParam objects. But we can make an
      array of rutz::shared_ptr<NModelParam> objects. Then we can initialize
      each in a loop by doing arr[i] = NModelParam<T>::make(...).

      Note that an alternative to C-style arrays (where the type must
      have a default constructor) is to use std::vector. Then we can
      do vec.push_back(NModelParam<T>(...)) and build the vector
      incrementally, again without needed a default constructor.

      Historical note: this substitutes for the previous strategy of
      giving NModelParam a default constructor plus an init() method
      that would be called in the array-initialization loop like
      arr[i].init(...), plus an initialized() methed. The current
      approach avoids the need for either init() or initialized(),
      instead we can let rutz::shared_ptr worry about whether the object is
      initialized -- we can test rutz::shared_ptr<NModelParam<T>>::get() != 0
      to see if the param is initialized.
  */
  static inline rutz::shared_ptr<NModelParam<T> >
  make(const std::string& nam, ParamClient* client, const T& initval,
       const ParamFlag flags = 0)
  { return rutz::shared_ptr<NModelParam<T> >(new NModelParam<T>(nam, client, initval, flags)); }

  //@}

  // ######################################################################
  /*! @name Access functions */
  //@{

  //! get the ModelParamAuxImpl's name
  virtual std::string getName() const
  { return impl.getName(); }

  //! Get the value
  /*! Note that there is no non-const function to get a non-const
      reference; that's because if we handed out a non-const reference
      then we'd have no way to know if callers changed the value, and
      we wouldn't be able to properly trigger paramChanged() on our
      ParamClient. Thus if you need to change the value, you should
      call getVal() to get a copy of the value, then do your
      modifications, then apply those changes with setVal(), which
      will trigger a paramChanged(). */
  T getVal() const
  {
    GVX_MUTEX_LOCK(impl.readLock());
    return this->val;
  }

  //! Set the value
  /*! @return true if the change succeeded; false otherwise. */
  bool setVal(const T& v)
  {
    // acquire a write lock for the entire duration of the setVal()
    // call, so that no other clients can enter setVal() concurrently;
    // in addition, since the write lock is an error-checking mutex,
    // we will get an error if setVal() is called recursively from
    // within the same thread such that a deadlock would otherwise
    // occur
    GVX_MUTEX_LOCK(impl.writeLock());

    // acquire a read lock to block out readers in other threads while
    // we update this->val (this is a recursive lock so we only block
    // out other threads but don't block ourselves from calling
    // getVal() from within the sendChangedMessage() call):
    GVX_MUTEX_LOCK(impl.readLock());

    const bool didchange = !(this->val == v);
    ValueChangeTransaction<T> tx(&this->val, v);

    if (ParamClient::CHANGE_REJECTED
        != impl.sendChangedMessage(didchange))
      {
        tx.acceptChange();
        return true;
      }
    return false;
  }

  //! Get the value as a string
  virtual std::string getValString() const
  {
    return convertToString(this->getVal());
  }

  //! Set the parameter value from a textual representation, notify clients of the change
  /*! @return true if the change succeeded; false otherwise. */
  virtual bool setValString(const std::string& textval)
  {
    T newval;
    convertFromString(textval, newval);
    // don't update this->val until after we are sure that the
    // convertFromString() has succeeded; that way if
    // convertFromString() fails we won't end up with a partially
    // modified or garbled value in this->val
    return this->setVal(newval);
  }

  //! Get the current value through a dynamically-typed RefHolder.
  inline virtual void getValGeneric(RefHolder& ref) const;

  //! Set the current value through a dynamically-typed RefHolder.
  /*! @return true if the change succeeded; false otherwise. */
  inline virtual bool setValGeneric(const RefHolder& ref);

  //@}


  // ######################################################################
  /*! @name Input/Output functions */
  //@{

  //! Print out our name and contents, mostly for debugging
  virtual void printout(std::ostream& s,
                        const std::string& prefix = "") const
  { impl.printout(s, prefix); }

  //! Write parameter value to ParamMap
  virtual void writeTo(ParamMap& pmap) const
  { impl.writeTo(pmap); }

  //! Get parameter value from ParamMap
  /*! @param noerr will not generate an error message if the parameter
    does not exist in ParamMap. */
  virtual void readFrom(const ParamMap& pmap,
                        const bool noerr = true)
  { impl.readFrom(pmap, noerr); }

  //@}

private:
  ModelParamAuxImpl impl;
  T val;     // the parameter value
  NModelParam(const NModelParam<T>& m);      //!< forbid copy-contruction
  NModelParam& operator=(const NModelParam<T>& m); //!< forbid copy
};



// ######################################################################
//! A class for params of type T that have a command-line option
/*! OModelParam is just like NModelParam except that it offers
    getOptionDef() and takes a ModelOptionDef* in its constructor. See
    the NModelParam documentation for all other details.
*/
template <class T> class OModelParam : public OptionedModelParam
{
public:
  // ######################################################################
  /*! @name Constructors and Destructors

      see NModelParam for more details
  */
  //@{

  //! Construct, using the initial value from a ModelOptionDef
  /*! If you want to specify a different initial value, use the other
      constructor which takes an initial value plus a USE_MY_VAL flag.

      @param flags If flags contains ALLOW_ONLINE_CHANGES, then this
      param will be marked as being able to be changed while the model
      is active.
  */
  inline OModelParam(const ModelOptionDef* def, ParamClient* client,
                     const ParamFlag flags = 0);

  //! Construct, using a specified initial value
  /*! @param flags If flags contains USE_MY_VAL, then the given
      initval will be pushed into the OptionManager as the new default
      value for the given ModelOptionDef. If you just want to take the
      existing default value from the ModelOptionDef as the initial
      value for this OModelParam, then use the other constructor that
      doesn't take an initval parameter. If flags contains
      ALLOW_ONLINE_CHANGES, then this param will be marked as being
      able to be changed while the model is active. */
  inline OModelParam(const ModelOptionDef* def, ParamClient* client,
                     const T& initval, const ParamFlag flags);

  //! A "pseudo-constructor" that makes a rutz::shared_ptr<OModelParam>.
  static inline rutz::shared_ptr<OModelParam<T> >
  make(const ModelOptionDef* def, ParamClient* client,
       const ParamFlag flags = 0)
  {
    return rutz::shared_ptr<OModelParam<T> > (new OModelParam<T>(def, client, flags));
  }

  //! A "pseudo-constructor" that makes a rutz::shared_ptr<OModelParam>.
  static inline rutz::shared_ptr<OModelParam<T> >
  make(const ModelOptionDef* def, ParamClient* client,
       const T& initval, const ParamFlag flags)
  {
    return rutz::shared_ptr<OModelParam<T> >
      (new OModelParam<T>(def, client, initval, flags));
  }

  //@}

  // ######################################################################
  /*! @name Access functions

      see NModelParam for more details
  */
  //@{

  //! get the ModelParamAuxImpl's name
  virtual std::string getName() const
  { return impl.getName(); }

  //! get the associated option def
  virtual const ModelOptionDef* getOptionDef() const
  { return impl.getOptionDef(); }

  //! Get the value
  /*! Note that there is no non-const function to get a non-const
      reference; that's because if we handed out a non-const reference
      then we'd have no way to know if callers changed the value, and
      we wouldn't be able to properly trigger paramChanged() on our
      ParamClient. Thus if you need to change the value, you should
      call getVal() to get a copy of the value, then do your
      modifications, then apply those changes with setVal(), which
      will trigger a paramChanged(). */
  T getVal() const
  {
    GVX_MUTEX_LOCK(impl.readLock());
    return this->val;
  }

  //! Set the value
  /*! @return true if the change succeeded; false otherwise. */
  bool setVal(const T& v)
  {
    // acquire a write lock for the entire duration of the setVal()
    // call, so that no other clients can enter setVal() concurrently;
    // in addition, since the write lock is an error-checking mutex,
    // we will get an error if setVal() is called recursively from
    // within the same thread such that a deadlock would otherwise
    // occur
    GVX_MUTEX_LOCK(impl.writeLock());

    // acquire a read lock to block out readers in other threads while
    // we update this->val (this is a recursive lock so we only block
    // out other threads but don't block ourselves from calling
    // getVal() from within the sendChangedMessage() call):
    GVX_MUTEX_LOCK(impl.readLock());

    const bool didchange = !(this->val == v);
    ValueChangeTransaction<T> tx(&this->val, v);

    if (ParamClient::CHANGE_REJECTED
        != impl.sendChangedMessage(didchange))
      {
        tx.acceptChange();
        return true;
      }
    return false;
  }

  //! Get the value as a string
  virtual std::string getValString() const
  {
    return convertToString(this->getVal());
  }

  //! Set the parameter value from a textual representation, notify clients of the change
  /*! @return true if the change succeeded; false otherwise. */
  virtual bool setValString(const std::string& textval)
  {
    T newval;
    convertFromString(textval, newval);
    // don't update this->val until after we are sure that the
    // convertFromString() has succeeded; that way if
    // convertFromString() fails we won't end up with a partially
    // modified or garbled value in this->val
    return this->setVal(newval);
  }

  //! Get the current value through a dynamically-typed RefHolder.
  inline virtual void getValGeneric(RefHolder& ref) const;

  //! Set the current value through a dynamically-typed RefHolder.
  /*! @return true if the change succeeded; false otherwise. */
  inline virtual bool setValGeneric(const RefHolder& ref);

  //@}


  // ######################################################################
  /*! @name Input/Output functions

      see NModelParam for more details
  */
  //@{

  //! Print out our name and contents, mostly for debugging
  virtual void printout(std::ostream& s,
                        const std::string& prefix = "") const
  { impl.printout(s, prefix); }

  //! Write parameter value to ParamMap
  virtual void writeTo(ParamMap& pmap) const
  { impl.writeTo(pmap); }

  //! Get parameter value from ParamMap
  virtual void readFrom(const ParamMap& pmap,
                        const bool noerr = true)
  { impl.readFrom(pmap, noerr); }

  //@}

private:
  ModelParamAuxImpl impl;
  T val;     // the parameter value
  OModelParam(const OModelParam<T>& m);      //!< forbid copy-contruction
  OModelParam& operator=(const OModelParam<T>& m); //!< forbid copy
};



// ######################################################################
// ############# NModelParam<T> implementation
// ######################################################################

// ######################################################################
template <class T> inline
NModelParam<T>::NModelParam(const std::string& nam,
                            ParamClient* client,
                            const T& initval,
                            const ParamFlag flags) :
  ModelParamBase(flags),
  impl(this, nam, client),
  val(initval)
{}

// ######################################################################
template <class T> inline
void NModelParam<T>::getValGeneric(RefHolder& ref) const
{
  typedef TRefHolder<T> reftype;
  reftype* tref = dynamic_cast<reftype*>(&ref);
  if (tref == 0)
    rutz::throw_bad_cast(typeid(T), ref.type(), SRC_POS);
  tref->ref = this->getVal();
}

// ######################################################################
template <class T> inline
bool NModelParam<T>::setValGeneric(const RefHolder& ref)
{
  typedef TRefHolder<const T> reftype;
  const reftype* tref = dynamic_cast<const reftype*>(&ref);
  if (tref == 0)
    rutz::throw_bad_cast(typeid(T), ref.type(), SRC_POS);
  return this->setVal(tref->ref);
}


// ######################################################################
// ############# OModelParam<T> implementation
// ######################################################################

// ######################################################################
template <class T> inline
OModelParam<T>::OModelParam(const ModelOptionDef* def,
                            ParamClient* client,
                            const ParamFlag flags) :
  OptionedModelParam(flags),
  impl(this, def, client, /*flags*/ 0, typeid(T)),
  val(fromStr<T>(ModelParamAuxImpl::defaultValueOf(def)))
{}

// ######################################################################
template <class T> inline
OModelParam<T>::OModelParam(const ModelOptionDef* def,
                            ParamClient* client,
                            const T& initval,
                            const int flags) :
  OptionedModelParam(flags),
  impl(this, def, client, flags, typeid(T)),
  val(initval)
{}

// ######################################################################
template <class T> inline
void OModelParam<T>::getValGeneric(RefHolder& ref) const
{
  typedef TRefHolder<T> reftype;
  reftype* tref = dynamic_cast<reftype*>(&ref);
  if (tref == 0)
    rutz::throw_bad_cast(typeid(T), ref.type(), SRC_POS);
  tref->ref = this->getVal();
}

// ######################################################################
template <class T> inline
bool OModelParam<T>::setValGeneric(const RefHolder& ref)
{
  typedef TRefHolder<const T> reftype;
  const reftype* tref = dynamic_cast<const reftype*>(&ref);
  if (tref == 0)
    rutz::throw_bad_cast(typeid(T), ref.type(), SRC_POS);
  return this->setVal(tref->ref);
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
