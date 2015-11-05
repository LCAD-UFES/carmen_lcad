/*!@file Component/ModelParam.C A tunable ModelComponent parameter base class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelParam.C $
// $Id: ModelParam.C 8746 2007-09-05 23:17:08Z rjpeters $
//

#include "Component/ModelParam.H"

#include "Component/ModelOptionDef.H"
#include "Component/ParamClient.H"
#include "Component/ParamMap.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "rutz/demangle.h"

#include <ostream>

namespace
{
  void initMutex(pthread_mutex_t* mut, int type)
  {
    pthread_mutexattr_t attr;
    if (0 != pthread_mutexattr_init(&attr))
      PLFATAL("pthread_mutexattr_init() failed");

    if (0 != pthread_mutexattr_settype(&attr, type))
      {
        pthread_mutexattr_destroy(&attr);
        PLFATAL("pthread_mutexattr_settype() failed");
      }

    if (0 != pthread_mutex_init(mut, &attr))
      {
        pthread_mutexattr_destroy(&attr);
        PLFATAL("pthread_mutex_init() failed");
      }

    if (0 != pthread_mutexattr_destroy(&attr))
      PLERROR("pthread_mutexattr_destroy() failed");
  }
}

// ######################################################################
// ############# ModelParamAuxImpl implementation
// ######################################################################

// ######################################################################
ModelParamAuxImpl::ModelParamAuxImpl(OptionedModelParam* self,
                                     const ModelOptionDef* def,
                                     ParamClient* client,
                                     const int flags,
                                     const std::type_info& valtype)
  :
  itsSelf(self),
  itsClient(client),
  itsOption(def),
  itsName(def->name),
  itsLocks(itsSelf->allowsOnlineChanges() ? new pthread_mutex_t[2] : 0),
  itsInCallback(false)
{
  ASSERT(itsSelf != 0);
  ASSERT(itsClient != 0);
  ASSERT(itsOption != 0);

  if (itsLocks != 0)
    {
      initMutex(&itsLocks[READLOCK], PTHREAD_MUTEX_RECURSIVE);
      initMutex(&itsLocks[WRITELOCK], PTHREAD_MUTEX_ERRORCHECK);
    }

  // register ourselves with our master:
  itsClient->registerOptionedParam(self, flags);

  if (def->type.argtype == 0)
    LFATAL("ModelOptionDef '%s' had a null argtype type_info",
           def->name);

  if (*def->type.argtype != valtype)
    LFATAL("the ModelParam '%s' "
           "being registered by '%s' has type '%s', "
           "but the corresponding ModelOptionDef has type '%s'",
           def->name,
           rutz::demangled_name(typeid(*client)),
           rutz::demangled_name(valtype),
           rutz::demangled_name(*def->type.argtype));
}

// ######################################################################
ModelParamAuxImpl::ModelParamAuxImpl(ModelParamBase* self,
                                     const std::string& nam,
                                     ParamClient* client)
  :
  itsSelf(self),
  itsClient(client),
  itsOption(0),
  itsName(nam),
  itsLocks(itsSelf->allowsOnlineChanges() ? new pthread_mutex_t[2] : 0),
  itsInCallback(false)
{
  ASSERT(itsSelf != 0);
  ASSERT(itsClient != 0);

  if (itsLocks != 0)
    {
      initMutex(&itsLocks[READLOCK], PTHREAD_MUTEX_RECURSIVE);
      initMutex(&itsLocks[WRITELOCK], PTHREAD_MUTEX_ERRORCHECK);
    }

  // register ourselves with our master:
  itsClient->registerParam(itsSelf);
}

// ######################################################################
ModelParamAuxImpl::~ModelParamAuxImpl()
{
  ASSERT(itsClient != 0);

  // un-register ourselves with our master:
  itsClient->unregisterParam(itsSelf);

  if (itsLocks != 0)
    {
      if (0 != pthread_mutex_destroy(&itsLocks[READLOCK]))
        PLERROR("pthread_mutex_destroy() failed for read lock");
      if (0 != pthread_mutex_destroy(&itsLocks[WRITELOCK]))
        PLERROR("pthread_mutex_destroy() failed for write lock");
      delete [] itsLocks;
    }
}

// ######################################################################
std::string ModelParamAuxImpl::getName() const
{
  ASSERT(itsClient != 0);

  return itsName;
}

// ######################################################################
const ModelOptionDef* ModelParamAuxImpl::getOptionDef() const
{
  ASSERT(itsClient != 0);

  ASSERT(itsOption != 0);

  return itsOption;
}

// ######################################################################
void ModelParamAuxImpl::printout(std::ostream& s, const std::string& prefix) const
{
  ASSERT(itsClient != 0);

  s<<prefix<<": "<<itsName<<" = "<<itsSelf->getValString()<<std::endl;
}

// ######################################################################
void ModelParamAuxImpl::writeTo(ParamMap& pmap) const
{
  ASSERT(itsClient != 0);

  pmap.putStringParam(itsName, itsSelf->getValString());
}

// ######################################################################
void ModelParamAuxImpl::readFrom(const ParamMap& pmap, const bool noerr)
{
  ASSERT(itsClient != 0);

  std::string res = itsSelf->getValString();
  ParamMap::ReturnCode rc = pmap.queryStringParam(itsName, res);
  if (rc == ParamMap::CHANGED)
    // will cause a paramChanged() to be sent to our client
    itsSelf->setValString(res);

  if (noerr == false && rc == ParamMap::MISSING)
    LERROR("Cannot find parameter %s in ParamMap", itsName.c_str());
}

// ######################################################################
ParamClient::ChangeStatus
ModelParamAuxImpl::sendChangedMessage(bool didchange)
{
  if (itsInCallback)
    LFATAL("Oops! Got a re-entrant paramChanged() call for '%s' -- "
           "make sure you aren't re-setting %s from within its own "
           "paramChanged() callback, either directly with setVal(), "
           "or indirectly via exportOptions() or something similar",
           itsName.c_str(), itsName.c_str());

  itsInCallback = true;
  ParamClient::ChangeStatus status = ParamClient::CHANGE_ACCEPTED;
  try {
    itsClient->paramChanged(itsSelf, didchange, &status);
  } catch (...) {
    itsInCallback = false;
    throw;
  }
  itsInCallback = false;
  return status;
}

// ######################################################################
const char* ModelParamAuxImpl::defaultValueOf(const ModelOptionDef* def)
{
  return def->defval;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
