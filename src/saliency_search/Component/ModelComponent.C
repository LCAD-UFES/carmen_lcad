/*!@file Component/ModelComponent.C base class for parameterized model components */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelComponent.C $
// $Id: ModelComponent.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Component/ModelComponent.H"

#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "rutz/error_context.h"
#include "rutz/sfmt.h"
#include "rutz/shared_ptr.h"

#include <exception>
#include <list>
#include <pthread.h>
#include <set>
#include <vector>

namespace dummy_namespace_to_avoid_gcc411_bug_ModelComponent_C
{
  struct ParamInfo;

  // NOTE! It is often NOT safe to iterate over a PinfoList with
  // iterators in the ModelComponent implementation, because often we
  // end up modifying the PinfoList (rep->pinfos) indirectly as a
  // result of the code in the loop body -- when that happens,
  // iterators are likely to be invalidated and so continuing the loop
  // will result in a crash. Thus it is necessary to iterate over
  // PinfoList using array indexes and a live test against
  // rep->pinfos.size() at each loop iteration to guard against online
  // changes to rep->pinfos itself.
  typedef std::vector<ParamInfo> PinfoList;

  typedef std::vector< nub::ref<ModelComponent> > SubCompVec;

  /// Helper class to keep track of possibly-exported command-line options
  struct ParamInfo
  {
    ParamInfo(ModelComponent* c, ModelParamBase* mp)
      : client(c), param(mp), oparam(0), exportMe(false), useMyVal(false) {}

    ParamInfo(ModelComponent* c, OptionedModelParam* omp, bool useme)
      : client(c), param(omp), oparam(omp), exportMe(true), useMyVal(useme) {}

    ModelComponent*     client;
    ModelParamBase*     param;
    OptionedModelParam* oparam;
    bool                exportMe;
    bool                useMyVal;
  };

  /// Helper class to ensure that no one tries to do rutz::shared_ptr<ModelComponent>.
  /** Since ModelComponent derives from nub::object, its
      reference-count is handled internally and should be managed by
      nub::ref or nub::soft_ref smart pointers. Catastrophe would be
      the result if we ended up with both a nub::soft_ref and a
      rutz::shared_ptr to the same object. So, to avoid that, we use the
      following solution: rutz::shared_ptr offers to call a hook function
      everytime somebody tries to construct a rutz::shared_ptr, and we here
      install such a hook function that checks to see if we recognize
      the attempted rutz::shared_ptr address as a live ModelComponent
      object. If so, we issue an LFATAL(). */
  struct PtrChecker
  {
    typedef std::set<const void*> ComponentSet;

    ComponentSet itsSet;

    void addPtr(const void* p)          { itsSet.insert(p); }
    bool hasPtr(const void* p)    const { return itsSet.find(p) != itsSet.end(); }
    void forgetPtr(const void* p)       { itsSet.erase(p); }
  };

  // we actually want to "leak" this pointer, so that the object it
  // points to never gets deleted... that's because we're going to be
  // using the PtrChecker object in ~ModelComponent(), and if the
  // PtrChecker gets destroyed before some final call to
  // ~ModelComponent() (e.g., likely to be ~ModelManager() that gets
  // called last) then we'll get some crash
  PtrChecker* g_checker = 0;

  PtrChecker& getChecker()
  {
    ASSERT(g_checker != 0);
    return *g_checker;
  }

  /// Here is the hook function itself.
  /** This hook function is registered with rutz::shared_ptr by
      ModelComponent::Impl() when Impl() is called the first time. */
  void checkNoSuchComponent(const void* p)
  {
    if (getChecker().hasPtr(p))
      LFATAL("\n\tLooks like you were trying to construct a\n"
             "\trutz::shared_ptr<ModelComponent> (or some derivative of\n"
             "\tModelComponent) -- but instead you should use\n"
             "\tnub::ref<ModelComponent>.");
  }

  pthread_once_t checker_init_once = PTHREAD_ONCE_INIT;
  void checker_init()
  {
    ASSERT(g_checker == 0);
    g_checker = new PtrChecker;
    rutz::shared_ptr_aux::set_check_function(&checkNoSuchComponent);
  }
}

using namespace dummy_namespace_to_avoid_gcc411_bug_ModelComponent_C;

// ######################################################################
// ######################################################################
// ModelComponent::Impl member functions
// ######################################################################
// ######################################################################

//! Private "pimpl" implementation class for ModelComponent
struct ModelComponent::Impl
{
  Impl(OptionManager* manager,
       const std::string& descrName,
       const std::string& tag,
       const std::string& crealm)
    :
    mgr(manager),
    dname(descrName),
    tname(tag),
    realm(crealm),
    started(false),
    hasBeenExported(false),
    parent()
  {
    pthread_once(&checker_init_once, &checker_init);
  }

  // add a new ParamInfo, but only if we don't already have the same one
  void addParam(const ParamInfo& pinfo)
  {
    // check to see if we already have this param in our list
    for (size_t i = 0; i < this->pinfos.size(); ++i)
      if (this->pinfos[i].param == pinfo.param)
        {
          // this can't be an LFATAL() because there is at least one
          // place (Beowulf/Beowulf.C) where the same param
          // (itsSlaveNames) can be registered/unregistered multiple
          // times, and it's difficult to ensure that it would never
          // be registered twice in a row
          LERROR("duplicate attempt to add a model param (%s)"
                 " -- old info overwritten",
                 pinfo.param->getName().c_str());

          // overwrite the old info so we avoid having duplicate
          // records in pinfos:
          this->pinfos[i] = pinfo;
          return;
        }

    // ok, we haven't seen this param before, so just add it to the
    // end of our list:
    this->pinfos.push_back(pinfo);
  }

  void findMatchingParams(const std::string& name,
                          const ModelFlag flags,
                          std::vector<ParamInfo>& matches) const
  {
    // first look in our own params:
    for (size_t i = 0; i < this->pinfos.size(); ++i)
      if (name.compare(this->pinfos[i].param->getName()) == 0)
        matches.push_back(this->pinfos[i]);

    // now check the subcomps:
    if (flags & MC_RECURSE)
      for (SubCompVec::const_iterator citr = this->subComps.begin();
           citr != this->subComps.end();
           ++citr)
        (*citr)->rep->findMatchingParams(name,
                                         flags | MC_IGNORE_MISSING,
                                         matches);

    // now see if the caller wanted to treat an empty result as an error:
    if (matches.size() == 0 && !(flags & MC_IGNORE_MISSING))
      LFATAL("No parameter named '%s'", name.c_str());
  }

  OptionManager* mgr;           //<! our manager
  std::string dname;            //<! descriptive name
  std::string tname;            //<! tag name
  std::string realm;            //<! our realm
  bool started;                 //<! did we start()?
  PinfoList pinfos;             //<! our model parameters
  SubCompVec subComps;          //<! our sub-components
  bool hasBeenExported;         //<! did we exportOptions()?
  nub::soft_ref<ModelComponent> parent; //<! our parent object, or null

private:
  Impl(const Impl&); // not implemented
  Impl& operator=(const Impl&); // not implemented
};

// ######################################################################
// ######################################################################
// ModelComponent member functions
// ######################################################################
// ######################################################################

// ######################################################################
ModelComponent::ModelComponent(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tag,
                               const std::string& crealm) :
  rep(new Impl(&mgr, descrName, tag, crealm))
{
  CLDEBUG(">>>> Constructed <<<<");

  getChecker().addPtr(rutz::full_object_cast(this));
}

// ######################################################################
ModelComponent::ModelComponent(const std::string& descrName,
                               const std::string& tag,
                               const std::string& crealm) :
  rep(new Impl(0, descrName, tag, crealm))
{
  getChecker().addPtr(rutz::full_object_cast(this));
}

// ######################################################################
ModelComponent::ModelComponent() :
  rep(new Impl(0, "BUG!", "BUG!", "World"))
{
  //LERROR(
  //    "\n\n##########\n"
  //    "Oh No! You probably forgot to include a call to the\n"
  //    "ModelComponent constructor in some class' initialization list.\n"
  //    "##########\n"
  //    );
}

// ######################################################################
void ModelComponent::init(OptionManager& mgr, const std::string& descrName,
                          const std::string& tagName, const std::string& crealm)
{
  rep->mgr = &mgr;
  rep->dname = descrName;
  rep->tname = tagName;
  rep->realm = crealm;
}

// ######################################################################
ModelComponent::~ModelComponent()
{
  getChecker().forgetPtr(rutz::full_object_cast(this));

  CLDEBUG(">>>> Destroying...");

  // stop ourselves if necessary:
  if (rep->started) stop();

  // strictly speaking these calls aren't necessary since the vectors
  // will be destroyed anyway during "delete rep"; but if we call them
  // here then it is easier for us to trace the destruction of
  // subcomponents within our ">>>> Destroying"/"<<<< Destroyed" calls
  rep->subComps.clear();
  rep->pinfos.clear();

  CLDEBUG("<<<< Destroyed");

  delete rep;
}

// ######################################################################
void ModelComponent::start()
{
  if (rep->started)
    CLFATAL("ModelComponent '%s' already started", rep->dname.c_str());

  if (rep->mgr == 0)
    CLFATAL("I need an OptionManager!");

  if (!rep->hasBeenExported)
    // If exportOptions() isn't called on us, then our OModelParams
    // won't be hooked up to their command-line options and won't have
    // the correct values. This is normally not a problem for objects
    // that are created in main() prior to command-line parsing,
    // because exportOptions() is called recursively inside
    // ModelManager::parseCommandLine(). However, failure to
    // exportOptions() is a likely problem with ModelComponent child
    // objects that are created in a paramChanged() callback during
    // command-line parsing as a result of some command-line option
    // that has been seen; the owner of that object needs be sure to
    // call exportOptions(MC_RECURSE) on that newly-created object.
    CLFATAL("Oops; I need to have exportOptions() called on me "
            "before I am started");

  GVX_ERR_CONTEXT(rutz::sfmt("starting %s '%s'",
                             this->obj_typename().c_str(),
                             this->tagName().c_str()));

  CLDEBUG(">>>> Starting...");

  // do our startup that is before our subs have been started:
  this->start1();

  // keep track of which subcomponents have been started, so that if
  // we get any exceptions, we can stop() those subcomponents
  SubCompVec started_comps;

  try
    {
      // start our subcomponents in order of addition:
      SubCompVec::iterator citr = rep->subComps.begin();
      while (citr != rep->subComps.end())
        {
          (*citr)->start();
          started_comps.push_back(*citr);
          ++citr;
        }

      // do our startup that is after our subs have been started:
      this->start2();

      // we are started:
      rep->started = true;
      CLDEBUG("<<<< Started");
    }
  catch (...)
    {
      // ok, one of our subcomponents threw an exception during its
      // start(), so give any subcomponents that were already
      // start()-ed a chance to clean up by calling stop on them,
      // before we allow the exception to propagate
      while (started_comps.size() > 0)
        {
          // since stop() is analogous to destructors, we enforce the
          // rule that no exceptions can be thrown during destruction
          try
            {
              started_comps.back()->stop();
            }
          catch (...)
            {
              ASSERT(!"exception caught in stop()!");
            }
          started_comps.pop_back();
        }

      // now re-throw the exception
      throw;
    }
}

// ######################################################################
bool ModelComponent::started() const
{ return rep->started; }

// ######################################################################
void ModelComponent::stop()
{
  if (rep->started == false)
    CLFATAL("ModelComponent '%s' not started", rep->dname.c_str());

  GVX_ERR_CONTEXT(rutz::sfmt("stopping %s '%s'",
                             this->obj_typename().c_str(),
                             this->tagName().c_str()));

  CLDEBUG(">>>> Stopping...");

  // do our shutdown that is before our subs have been stopped:
  this->stop1();

  // stop our subcomponents in reverse order of addition:
  SubCompVec::iterator citr = rep->subComps.end();
  while(citr != rep->subComps.begin()) { -- citr;  (*citr)->stop(); }

  // do our shutdown that is after our subs have been stopped:
  this->stop2();

  // we are stopped:
  rep->started = false;
  CLDEBUG("<<<< Stopped");
}

// ######################################################################
void ModelComponent::managerDestroyed()
{
  //CLINFO("managerDestroyed");

  rep->mgr = NULL;

  // also let all our subcomponents know about this sad situation:
  SubCompVec::iterator citr = rep->subComps.begin();
  while(citr != rep->subComps.end())
    { (*citr)->managerDestroyed(); citr ++; }
}

// ######################################################################
void ModelComponent::printout(std::ostream& s, const std::string& prefix) const
{
  // add our tag name to the prefix:
  std::string newprefix;
  if (prefix.size())
    {
      if (isspace(prefix[prefix.size()-1]))
        newprefix = prefix + rep->realm + ":" + rep->tname;
      else
        newprefix = prefix + "." + rep->realm + ":" + rep->tname;
    }
  else
    newprefix = rep->realm + ":" + rep->tname;

  // first, call printout() on all our ModelParam's, in the order in which
  // they registered with us:
  for (size_t i = 0; i < rep->pinfos.size(); ++i)
    rep->pinfos[i].param->printout(s, newprefix);

  // then call printout() on all our subcomponents, in the order in
  // which thery were added:
  SubCompVec::const_iterator citr = rep->subComps.begin();
  while(citr != rep->subComps.end())
    { (*citr)->printout(s, newprefix); citr ++; }
}

// ######################################################################
void ModelComponent::start1()
{ }

// ######################################################################
void ModelComponent::start2()
{ }

// ######################################################################
void ModelComponent::stop1()
{ }

// ######################################################################
void ModelComponent::stop2()
{ }

// ######################################################################
void ModelComponent::reset1()
{ }

// ######################################################################
void ModelComponent::reset2()
{ }

// ######################################################################
void ModelComponent::save1(const ModelComponentSaveInfo& sinfo)
{ }

// ######################################################################
void ModelComponent::save2(const ModelComponentSaveInfo& sinfo)
{ }

// ######################################################################
std::string ModelComponent::descriptiveName() const
{ return rep->dname; }

// ######################################################################
void ModelComponent::setDescriptiveName(const std::string& name)
{ rep->dname = name; }

// ######################################################################
std::string ModelComponent::tagName() const
{ return rep->tname; }

// ######################################################################
void ModelComponent::setTagName(const std::string& name)
{ rep->tname = name; }

// ######################################################################
std::string ModelComponent::realm() const
{ return rep->realm; }

// ######################################################################
void ModelComponent::setRealm(const std::string& crealm)
{
  if (started()) CLFATAL("Too late to set realm after start()...");
  rep->realm = crealm;

  // recurse through all subcomponents:
  SubCompVec::const_iterator itr = rep->subComps.begin();
  while(itr != rep->subComps.end()) (*itr++)->setRealm(crealm);
}

// ######################################################################
ModelComponent* ModelComponent::getParent() const
{
  // will return null if the smart pointer has no pointee
  return rep->parent.get_weak();
}

// ######################################################################
ModelComponent* ModelComponent::getRootObject()
{
  ModelComponent* p = this;
  while (p->getParent() != 0)
    p = p->getParent();
  return p;
}

// ######################################################################
const ModelComponent* ModelComponent::getRootObject() const
{
  const ModelComponent* p = this;
  while (p->getParent() != 0)
    p = p->getParent();
  return p;
}

// ######################################################################
uint ModelComponent::addSubComponent(const nub::ref<ModelComponent>& subc, const bool propagate_realm)
{
  if (subc->getParent() != 0)
    LERROR("ModelComponent %s is already a subcomponent of %s, creating loop in component tree",
           subc->tagName().c_str(), subc->getParent()->tagName().c_str());
  else
    // make the parent a WEAK pointer to this; that we we avoid
    // cyclic references: parent holds a strong pointer to child,
    // child holds a weak pointer to parent
    subc->rep->parent = nub::soft_ref<ModelComponent>(this, nub::WEAK);

  // recursively set the realm of the sub
  if (propagate_realm) subc->setRealm(rep->realm);

  // add it to our list of subs:
  rep->subComps.push_back(subc);
  return rep->subComps.size() - 1;
}

// ######################################################################
int ModelComponent::removeSubComponent(const ModelComponent& subc,
                                        bool removeall)
{
  SubCompVec::iterator citr = rep->subComps.begin();
  int numremoved = 0;
  while (citr != rep->subComps.end())
    {
      if ((*citr).get() == &subc)
        {
          citr = rep->subComps.erase(citr);
          ++numremoved;

          if (!removeall)
            break;
        }
      else
        citr ++;
    }

  if (0 == numremoved)
    CLERROR("Request to erase unknown subcomponent -- IGNORED");

  return numremoved;
}

// ######################################################################
void ModelComponent::removeSubComponent(const uint idx)
{
  if (idx >= rep->subComps.size())
    CLERROR("Attempt to remove subcomp whith index beyond range -- IGNORED");
  else {
    SubCompVec::iterator citr = rep->subComps.begin();
    citr += idx; rep->subComps.erase(citr);
  }
}

// ######################################################################
void ModelComponent::removeSubComponent(const std::string& tagname)
{
  SubCompVec::iterator citr = rep->subComps.begin();
  while(citr != rep->subComps.end()) {
    if (tagname.compare((*citr)->tagName()) == 0)
      { rep->subComps.erase(citr); return; }
    citr ++;
  }
  CLFATAL("Cannot find subComponent '%s'", tagname.c_str());
}

// ######################################################################
void ModelComponent::removeSubComponent(const nub::ref<ModelComponent>& subc)
{
  removeSubComponent(subc->tagName().c_str());
}

// ######################################################################
void ModelComponent::removeAllSubComponents()
{ while(rep->subComps.size()) rep->subComps.pop_back(); }

// ######################################################################
nub::ref<ModelComponent> ModelComponent::subComponent(const uint idx) const
{
  if (idx >= rep->subComps.size())
    CLFATAL("%s: request for subcomponent %u but I have only %" ZU " -- FATAL",
           rep->dname.c_str(), idx, rep->subComps.size());
  return rep->subComps[idx];
}

// ######################################################################
nub::ref<ModelComponent>
ModelComponent::subComponent(const std::string& tagname,
                             const ModelFlag flags) const
{
  SubCompVec::const_iterator
    itr = rep->subComps.begin();
  while(itr != rep->subComps.end()) {
    // is the one we are looking for one of our subcomponents?
    if (tagname.compare((*itr)->tagName()) == 0) return *itr;

    // otherwise, if recursive, see whether it's a subcomp of the current:
    if ((flags & MC_RECURSE) && (*itr)->hasSubComponent(tagname, flags))
      return (*itr)->subComponent(tagname, flags);

    itr ++;
  }
  CLFATAL("Cannot find subComponent '%s'", tagname.c_str());
  return nub::ref<ModelComponent>((ModelComponent*)0);  // keep g++ happy
}

// ######################################################################
uint ModelComponent::numSubComp() const
{ return rep->subComps.size(); }

// ######################################################################
bool ModelComponent::hasSubComponent(const std::string& tagname,
                                     const ModelFlag flags) const
{
  SubCompVec::const_iterator
    itr = rep->subComps.begin();
  while(itr != rep->subComps.end()) {
    // is the one we are looking for one of our subcomponents?
    if (tagname.compare((*itr)->tagName()) == 0) return true;

    // otherwise, if recursive, see whether it's a subcomp of the current:
    if ((flags & MC_RECURSE) && (*itr)->hasSubComponent(tagname, flags))
      return true;
    itr ++;
  }
  // nowhere to be found
  return false;
}

// ######################################################################
bool ModelComponent::hasSubComponent(const nub::soft_ref<ModelComponent>& c,
                                     const ModelFlag flags) const
{
  SubCompVec::const_iterator
    itr = rep->subComps.begin();
  while(itr != rep->subComps.end()) {
    // is the one we are looking for one of our subcomponents?
    if (c.getWeak() == itr->get()) return true;

    // otherwise, if recursive, see whether it's a subcomp of the current:
    if ((flags & MC_RECURSE) && (*itr)->hasSubComponent(c, flags))
      return true;
    itr ++;
  }
  // nowhere to be found
  return false;
}

// ######################################################################
void ModelComponent::exportOptions(const ModelFlag flags)
{
  // in the base class implementation, we do nothing for us proper...
  CLDEBUG(">>>> Exporting Options...");

  /* if recursive, let's propagate to the subcomps

     NOTE! We must do this BEFORE exporting our own options!

     Why? Suppose we exported our own options before exporting subcomp
     options, and consider this situation: we have one option X, and
     we have one subcomponent A that has its own option Y. Suppose
     that in our paramChanged() function, when we detect that X has
     been changed, we propagate some related change to A's Y
     option. Now if our options our exported first, then when X is
     first exported that will trigger a paramChanged() cause us to set
     A's Y, but when we then propagate exportOptions() to
     subcomponents, A's Y option will be overwritten with its default
     option value, and our desired value based on X will have been
     lost. Therefore we need to export subcomp options before our
     own. This fits with the general rule that subcomps should be
     fully constructed/useable before the owning object tries to use
     them (just like c++ subobjects and base classes must be fully
     constructed before the containing object uses them).
  */
  if (flags & MC_RECURSE)
    {
      SubCompVec::iterator citr=rep->subComps.begin();
      while(citr != rep->subComps.end()) {
        (*citr)->exportOptions(flags);
        citr ++;
      }
    }

  // loop over our param list, looking for params that have
  // command-line options (i.e., are of class OptionedModelParam
  // rather than plain ModelParamBase):

  for (size_t i = 0; i < rep->pinfos.size(); ++i)
    {
      if (rep->pinfos[i].oparam != 0 && rep->pinfos[i].exportMe == true)
        rep->mgr->requestOption(*(rep->pinfos[i].oparam), rep->pinfos[i].useMyVal);
    }

  CLDEBUG("<<<< Exported Options");

  rep->hasBeenExported = true;
}

// ######################################################################
bool ModelComponent::hasModelParam(const std::string& name,
                                   const ModelFlag flags) const
{
  std::vector<ParamInfo> matches;
  rep->findMatchingParams(name, flags, matches);
  return (matches.size() > 0);
}

// ######################################################################
void ModelComponent::setModelParamString(const std::string& name,
                                         const std::string& value,
                                         const ModelFlag flags)
{
  std::vector<ParamInfo> matches;
  rep->findMatchingParams(name, flags, matches);

  for (uint i = 0; i < matches.size(); ++i)
    {
      GVX_ERR_CONTEXT
        (rutz::sfmt
         ("setting parameter '%s' in component '%s'",
          name.c_str(), matches[i].client->descriptiveName().c_str()));

      if (rep->started && !matches[i].param->allowsOnlineChanges())
        LFATAL("Cannot change parameter '%s' values while started",
               matches[i].param->getName().c_str());

      matches[i].param->setValString(value);
    }
}

// ######################################################################
std::string ModelComponent::getModelParamString(const std::string& name,
                                                const ModelFlag flags) const
{
  if (flags & MC_IGNORE_MISSING)
    CLFATAL("MC_IGNORE_MISSING not allowed for getting param values");

  std::vector<ParamInfo> matches;
  rep->findMatchingParams(name, flags, matches);

  // we don't allow MC_IGNORE_MISSING, so we should always get a
  // non-empty set back from findMatchingParams():
  ASSERT(matches.size() > 0);

  GVX_ERR_CONTEXT
    (rutz::sfmt
     ("getting parameter '%s' from component '%s'",
      name.c_str(), matches[0].client->descriptiveName().c_str()));

  // by convention, we use the value from the first matching param
  // (even though there may have been more than one)
  return matches[0].param->getValString();
}

// ######################################################################
bool ModelComponent::doRequestOption(const ModelOptionDef* opt,
                                     const bool useMyVal,
                                     const bool recurse,
                                     const bool warn)
{
  if (rep->started)
    CLFATAL("Cannot request an option while started");
  bool gotit = false;

  // look a ParamInfo that has an OptionedModelParam with a matching
  // ModelOptionDef
  for (size_t i = 0; i < rep->pinfos.size(); ++i)
    {
      if (rep->pinfos[i].oparam != 0 && rep->pinfos[i].oparam->getOptionDef() == opt)
        {
          rep->pinfos[i].exportMe = true;
          rep->pinfos[i].useMyVal = useMyVal;

          rep->mgr->requestOption(*(rep->pinfos[i].oparam), useMyVal);
          gotit = true;
        }
    }

  // if recurse is true, let's also see if our subcomps have it:
  if (recurse)
    {
      SubCompVec::iterator citr=rep->subComps.begin();
      while(citr != rep->subComps.end()) {
        gotit |= (*citr)->doRequestOption(opt, useMyVal, recurse, false);
        citr ++;
      }
    }

  if (warn && gotit == false)
    CLERROR("No ModelParam named '%s' -- IGNORED", opt->name);
  return gotit;
}

// ######################################################################
void ModelComponent::hideOption(const ModelOptionDef* opt)
{
  if (rep->started) CLFATAL("Cannot hide an option while started");
  if (rep->hasBeenExported) CLFATAL("Cannot hide an option that has already been exported");

  // lookup a ParamInfo that has an OptionedModelParam with a matching ModelOptionDef:
  for (size_t i = 0; i < rep->pinfos.size(); ++i)
    if (rep->pinfos[i].oparam != 0 && rep->pinfos[i].oparam->getOptionDef() == opt)
      {
        rep->pinfos[i].exportMe = false;
        return;
      }

  CLFATAL("No ModelParam named '%s'", opt->name);
}

// ######################################################################
size_t ModelComponent::getNumModelParams() const
{
  return rep->pinfos.size();
}

// ######################################################################
const ModelParamBase* ModelComponent::getModelParam(size_t i) const
{
  if (i >= rep->pinfos.size())
    LFATAL("Oops! Request to access model param %" ZU ", but I have only "
           "%" ZU " params", i, rep->pinfos.size());

  return rep->pinfos[i].param;
}

// ######################################################################
ModelParamBase* ModelComponent::getModelParam(size_t i)
{
  if (i >= rep->pinfos.size())
    LFATAL("Oops! Request to access model param %" ZU ", but I have only "
           "%" ZU " params", i, rep->pinfos.size());

  return rep->pinfos[i].param;
}

// ######################################################################
void ModelComponent::readParamsFrom(const ParamMap& pmap, const bool noerr)
{
  if (rep->started)
    CLFATAL("Cannot read ModelParam values while started");

  // if we want to ignore missing pinfos, check first whether we have one:
  if (noerr && pmap.hasParam(rep->tname) == false) return;

  // we should have a submap for all our stuff (will trigger error otherwise):
  rutz::shared_ptr<ParamMap> submap = pmap.getSubpmap(rep->tname);

  // ok, do our own pinfos:
  for (size_t i = 0; i < rep->pinfos.size(); ++i)
    rep->pinfos[i].param->readFrom(*submap, noerr);

  // then go over our subcomponents, and recurse:
  SubCompVec::iterator citr = rep->subComps.begin();
  while(citr != rep->subComps.end())
    { (*citr)->readParamsFrom(*submap, noerr); citr ++; }
}

// ######################################################################
void ModelComponent::writeParamsTo(ParamMap& pmap) const
{
  //  if (rep->started)
  //  CLERROR("It is not safe to write ModelParam values while started");

  // first, create a submap for all our stuff:
  rutz::shared_ptr<ParamMap> submap(new ParamMap);

  // then, do our own pinfos:
  for (size_t i = 0; i < rep->pinfos.size(); ++i)
    rep->pinfos[i].param->writeTo(*submap);

  // then go over our subcomponents, and recurse:
  SubCompVec::const_iterator citr = rep->subComps.begin();
  while(citr != rep->subComps.end())
    { (*citr)->writeParamsTo(*submap); citr ++; }

  // finally add this new submap to our incoming pmap:
  pmap.putSubpmap(rep->tname, submap);
}

// ######################################################################
void ModelComponent::registerParam(ModelParamBase* mp)
{
  rep->addParam(ParamInfo(this, mp));
}

// ######################################################################
void ModelComponent::registerOptionedParam(OptionedModelParam* mp,
                                           const ParamFlag flags)
{
  // don't do rep->mgr->requestOption() right away, instead we'll wait
  // and do that inside ModelComponent::exportOptions() where we loop
  // over rep->pinfos
  if (flags & USE_MY_VAL)
    rep->addParam(ParamInfo(this, mp, true));
  else
    rep->addParam(ParamInfo(this, mp, false));
}

// ######################################################################
void ModelComponent::unregisterParam(const ModelParamBase* mp)
{
  bool gotit = false;

  for (size_t i = 0; i < rep->pinfos.size(); /* increment in loop body */)
    {
      if (rep->pinfos[i].param == mp)
        {
          if (rep->mgr != 0 && rep->pinfos[i].oparam != 0)
            rep->mgr->unRequestOption(*rep->pinfos[i].oparam);
          rep->pinfos.erase(rep->pinfos.begin() + i);
          gotit = true;
        }
      else
        ++i;
    }

  if (gotit == false)
    CLERROR("Request to unregister unknown parameter");
}

// ######################################################################
void ModelComponent::paramChanged(ModelParamBase* const param,
                                  const bool valueChanged,
                                  ParamClient::ChangeStatus* status)
{}

// ######################################################################
void ModelComponent::forgetExports()
{
  for (size_t i = 0; i < rep->pinfos.size(); ++i)
    rep->pinfos[i].exportMe = false;
}

// ######################################################################
void ModelComponent::reset(const ModelFlag flags)
{
  if (rep->started == false)
    CLFATAL("ModelComponent '%s' must be started before reset()",
            rep->dname.c_str());

  this->reset1();

  // if recursive, let's propagate to the subcomps:
  if (flags & MC_RECURSE)
    {
      for (SubCompVec::iterator
             citr = rep->subComps.begin(),
             stop = rep->subComps.end();
           citr != stop; ++citr)
        (*citr)->reset(flags);
    }

  this->reset2();
}

// ######################################################################
void ModelComponent::save(const ModelComponentSaveInfo& sinfo,
                          const ModelFlag flags)
{
  if (rep->started == false)
    CLFATAL("ModelComponent '%s' must be started before save()",
            rep->dname.c_str());

  this->save1(sinfo);

  // if recursive, let's propagate to the subcomps:
  if (flags & MC_RECURSE)
    {
      for (SubCompVec::iterator
             citr = rep->subComps.begin(),
             stop = rep->subComps.end();
           citr != stop; ++citr)
        (*citr)->save(sinfo, flags);
    }

  this->save2(sinfo);
}

// ######################################################################
OptionManager& ModelComponent::getManager() const
{
  return *rep->mgr;
}

// ######################################################################
void ModelComponent::setManager(OptionManager& mgr)
{
  ASSERT(rep->mgr == 0);
  rep->mgr = &mgr;
}

// ######################################################################
bool ModelComponent::hasBeenExported() const
{
  return rep->hasBeenExported;
}

// ######################################################################
void ModelComponent::setModelParamValAux(const std::string& name,
                                         const RefHolder& ref,
                                         const ModelFlag flags)
{
  std::vector<ParamInfo> matches;
  rep->findMatchingParams(name, flags, matches);

  // change the param val for all matching model params (this will
  // include params from our subcomponents if 'recurse' is true)
  for (uint i = 0; i < matches.size(); ++i)
    {
      GVX_ERR_CONTEXT
        (rutz::sfmt
         ("setting parameter '%s' in component '%s'",
          name.c_str(), matches[i].client->descriptiveName().c_str()));

      if (rep->started && !matches[i].param->allowsOnlineChanges())
        LFATAL("Cannot change parameter '%s' values while started",
               matches[i].param->getName().c_str());

      matches[i].param->setValGeneric(ref);
    }
}

// ######################################################################
void ModelComponent::getModelParamValAux(const std::string& name,
                                         RefHolder& ref,
                                         const ModelFlag flags) const
{
  if (flags & MC_IGNORE_MISSING)
    CLFATAL("MC_IGNORE_MISSING not allowed for getting param values");

  std::vector<ParamInfo> matches;
  rep->findMatchingParams(name, flags, matches);

  // we don't allow MC_IGNORE_MISSING, so we should always get a
  // non-empty set back from findMatchingParams():
  ASSERT(matches.size() > 0);

  GVX_ERR_CONTEXT
    (rutz::sfmt
     ("getting parameter '%s' from component '%s'",
      name.c_str(), matches[0].client->descriptiveName().c_str()));

  // by convention, we use the value from the first matching param
  // (even though there may have been more than one)
  matches[0].param->getValGeneric(ref);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
