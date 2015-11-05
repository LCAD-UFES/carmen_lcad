/*!@file Component/ModelManager.C manages a model as collection of ModelComponents */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelManager.C $
// $Id: ModelManager.C 14253 2010-11-20 01:52:00Z rand $
//

#include "Component/ModelManager.H"

#include "Component/CmdlineOptionManager.H"
#include "Component/GlobalOpts.H"
#include "Component/ParamMap.H"
#include "Util/AllocAux.H"
#include "Util/CpuTimer.H"
#include "Util/MathFunctions.H"
#include "Util/fpe.H"
#include "Util/fpu.H"
#include "Util/log.H"
#include "Util/terminate.H"
#include "Util/version.H"
#include "rutz/atomic.h"
#include "rutz/demangle.h"
#include "rutz/prof.h"

#include <cstdlib>         // for getenv()
#include <iostream>
#include <string>
#include <typeinfo>
#include <cstdio>          // for fopen()

using std::cerr;
using std::endl;
using std::string;

// ##############################################################
//! This is the internal implementation struct for ModelManager.
/*! We use the implementation approach so that clients of ModelManager
    aren't exposed to its implementation details (and thus they don't
    have to recompile everytime something changes in ModelManager's
    implementation). */
struct ModelManager::Impl
{
  Impl(ModelManager* owner)
    :
    paramShowHelpMsg(&OPT_ShowHelpMessage, owner),
    paramShowVersion(&OPT_ShowVersion, owner),
    paramShowSvnVersion(&OPT_ShowSvnVersion, owner),
    paramCheckPristine(&OPT_CheckPristine, owner),
    paramDebugMode(&OPT_DebugMode, owner),
    paramUsingFPE(&OPT_UsingFPE, owner),
    paramFpuPrecision(&OPT_FpuPrecision, owner),
    paramFpuRoundingMode(&OPT_FpuRoundingMode, owner),
    paramTestMode(&OPT_TestMode, owner),
    paramProfileFile(&OPT_ProfileOutFile, owner),
    paramLogVerb(&OPT_LogVerb, owner),
    paramEchoArgs(&OPT_EchoArgs, owner),
    paramMemCaching(&OPT_MemCaching, owner),
    userLogVerb(MYLOGVERB)
  {
    // WARNING! Don't call any code here; instead call code from
    // ModelManager's constructor. That's because if we call any code
    // here that ends up back in some other ModelManager function, the
    // rep will not yet be initialized -- it will be null or garbage
    // -- since we haven't yet returned from the Impl constructor.
  }

  CmdlineOptionManager com;

  OModelParam<bool> paramShowHelpMsg;   //!< print help message?
  OModelParam<bool> paramShowVersion;   //!< print full version info?
  OModelParam<bool> paramShowSvnVersion;//!< print svn repo version?
  OModelParam<bool> paramCheckPristine; //!< check if the source is pristine?
  OModelParam<bool> paramDebugMode;     //!< use debug mode?
  OModelParam<bool> paramUsingFPE;      //!< use floating-point exceptions?
  OModelParam<FpuPrecision> paramFpuPrecision; //!< floating-point precision
  OModelParam<FpuRoundingMode> paramFpuRoundingMode; //!< floating-point rounding mode
  OModelParam<bool> paramTestMode;      //!< use test mode?
  OModelParam<string> paramProfileFile; //!< Where to save profiling information
  rutz::shared_ptr<OModelParam<string> > paramLoadConfigFname; //!< Name of a config file to load up
  rutz::shared_ptr<OModelParam<string> > paramSaveConfigFname; //!< Name of a config file to save to
  OModelParam<string> paramLogVerb; //!< Log verbosity level
  OModelParam<bool> paramEchoArgs; //!< echo command-line args during start()?
  OModelParam<bool> paramMemCaching;

  bool autoLoadConfig; //!< Load ~/.execname just before parsing command-line

  bool didSave;      //!< was saveConfig() already called?

  const int userLogVerb; //!< the log verbosity level set by the user before the ModelManager was created

  CpuTimer timer;
};


// ##############################################################
// ##############################################################
// ModelManager member functions
// ##############################################################
// ##############################################################

// ##############################################################
ModelManager::ModelManager(const string& descrName,
                           const string& tag,
                           const bool loadCfgOpt,
                           const bool saveCfgOpt,
                           const bool autoLoadCfg) :
  ModelComponent(descrName, tag),
  rep(new Impl(this))
{
  // install a fancy termination handler that will print a backtrace
  invt_install_fancy_terminate();

  this->setManager(rep->com);

  // NOTE no need to do explicit requestOption() here anymore;
  // instead, requestOption() during ModelComponent's implementation
  // of exportOptions() -- nevertheless, request the debug mode option
  // right away so that we aren't needlessly noisy at the beginning of
  // the program run:
  this->requestOption(rep->paramDebugMode);

  // do we want an option to load a config file?
  if (loadCfgOpt) {
    rep->paramLoadConfigFname =
      OModelParam<string>::make(&OPT_LoadConfigFile, this);
  }

  // do we want an option to save a config file?
  if (saveCfgOpt) {
    rep->paramSaveConfigFname =
      OModelParam<string>::make(&OPT_SaveConfigFile, this);
  }

  // keep track of autoload:
  rep->autoLoadConfig = autoLoadCfg;

  // keep track of whether saveConfig() has been called:
  rep->didSave = false;

  // pick up settings from relevant environment variables now, before
  // the command-line is parsed, so that command-line options can
  // override the environment variables -- NOTE that if this becomes a
  // commonly-used idiom, then it might make sense to add a new
  // setOptionFromEnvironment() virtual function to OptionManager, and
  // implement it here in ModelManager
  const char* precision = getenv("INVT_FPU_PRECISION");

  if (precision != 0)
    {
      this->setOptionValString(rep->paramFpuPrecision.getOptionDef(),
                               precision);
    }

  const char* rounding = getenv("INVT_FPU_ROUNDING_MODE");

  if (rounding != 0)
    {
      this->setOptionValString(rep->paramFpuRoundingMode.getOptionDef(),
                               rounding);
    }
}

// ##############################################################
ModelManager::~ModelManager()
{
  if (this->started())
    this->stop();

  // NOTE: one could be tempted to call saveConfig() here for
  // automatic persistence of our ModelParam values. But that would be
  // dangerous as some of our ModelComponents may already have been
  // destroyed by the time we get here. So, saveConfig() should be
  // called manually, at a point in the code where all ModelComponents
  // are still alive.

  // let our components know that we are gone:
  managerDestroyed();

  delete rep;

  // ugly const_cast to set rep=0 here, but the ugliness is justified
  // the fact that it allows us to ASSERT(rep!=0) in our other
  // functions to make sure that people don't try to use us after we
  // have already been destroyed
  *(const_cast<Impl**>(&rep)) = 0;
}

// ##############################################################
void ModelManager::allowOptions(const int mask)
{
  ASSERT(rep != 0);
  rep->com.allowOptions(mask);
}

// ##############################################################
void ModelManager::requestOption(OptionedModelParam& p,
                                 const bool useMyVal)
{
  ASSERT(rep != 0);
  rep->com.requestOption(p, useMyVal);
}

// ##############################################################
void ModelManager::unRequestOption(OptionedModelParam& p)
{
  if (rep == 0)
    LERROR("Oops, it looks like some ModelComponent is trying "
           "to unrequest an OptionedModelParam during its "
           "destructor, but doesn't know that the ModelManager "
           "has already been destroyed. A likely cause of this "
           "is that the ModelComponent in question was never "
           "passed to addSubComponent().");
  ASSERT(rep != 0);
  rep->com.unRequestOption(p);
}

// ##############################################################
void ModelManager::requestOptionAlias(const ModelOptionDef* def)
{
  ASSERT(rep != 0);
  rep->com.requestOptionAlias(def);
}

// ##############################################################
void ModelManager::setOptionValString(const ModelOptionDef* def,
                                      const string& val)
{
  ASSERT(rep != 0);
  rep->com.setOptionValString(def, val);
}

// ##############################################################
string ModelManager::getOptionValString(const ModelOptionDef* def)
{
  ASSERT(rep != 0);
  return rep->com.getOptionValString(def);
}

// ##############################################################
bool ModelManager::isOptionRequested(const ModelOptionDef* def) const
{
  ASSERT(rep != 0);
  return rep->com.isOptionRequested(def);
}

// ##############################################################
bool ModelManager::parseCommandLine(const int argc,
                                    const char** argv,
                                    const char* usage,
                                    const int minarg,
                                    const int maxarg)
{
  ASSERT(rep != 0);

  // export options if that hasn't been done already
  if (this->hasBeenExported() == false)
    exportOptions(MC_RECURSE);

  // let's get our application name:
  if (argc <= 0)
    LFATAL("expected argc >= 1, got argc=%d", argc);
  string procname(argv[0]);
  uint ii = procname.rfind('/'); // skip the path; get just the filename:
  if (ii < procname.size()) procname = procname.substr(ii + 1);

  // if we wanted to automatically load a config from ~/.execname,
  // let's do that now:
  if (rep->autoLoadConfig && getenv("HOME"))
    {
      string fname = string(getenv("HOME")) + "/." + procname;
      FILE* tryit = fopen(fname.c_str(), "r");
      if (tryit) {   // config file exists; let's load it:
        fclose(tryit);
        LINFO("Autoloading configuration from '%s'", fname.c_str());
        ParamMap pmap; pmap.load(fname.c_str());
        readParamsFrom(pmap);
      }
    }

  return rep->com.parseCommandLine(argc, argv, usage, minarg, maxarg);
}

// ##############################################################
bool ModelManager::parseCommandLine(const int argc,
                                    char* const* argv,
                                    const char* usage,
                                    const int minarg,
                                    const int maxarg)
{
  std::vector<const char*> argv2;
  while (*argv != 0)
    argv2.push_back(*argv++);
  argv2.push_back(0);

  ASSERT(int(argv2.size()) == (argc + 1));

  return this->parseCommandLine(argc, &argv2[0], usage, minarg, maxarg);
}

// ##############################################################
bool ModelManager::parseCommandLineCore(const int argc, const char** argv)
{
  ASSERT(rep != 0);
  return rep->com.parseCommandLineCore(argc, argv);
}

// ##############################################################
bool ModelManager::parseCommandLineCore(const char* args)
{
  ASSERT(rep != 0);
  return rep->com.parseCommandLineCore(args);
}

// ##############################################################
uint ModelManager::numExtraArgs() const
{
  ASSERT(rep != 0);
  return rep->com.numExtraArgs();
}

// ##############################################################
string ModelManager::getExtraArg(const uint num) const
{
  ASSERT(rep != 0);
  return rep->com.getExtraArg(num);
}

// ##############################################################
uint ModelManager::numOptionDefs() const
{
  ASSERT(rep != 0);
  return rep->com.numOptionDefs();
}

// ##############################################################
uint ModelManager::getOptionDefs(const ModelOptionDef** arr, uint narr)
{
  ASSERT(rep != 0);
  return rep->com.getOptionDefs(arr, narr);
}

// ##############################################################
const ModelOptionDef* ModelManager::findOptionDef(const char* name) const
{
  return rep->com.findOptionDef(name);
}

// ##############################################################
bool ModelManager::isOptionDefUsed(const ModelOptionDef* def) const
{
  ASSERT(rep != 0);
  return rep->com.isOptionDefUsed(def);
}

// ##############################################################
void ModelManager::loadConfig()
{
  ASSERT(rep != 0);
  if (rep->paramLoadConfigFname.get() != 0 &&
      rep->paramLoadConfigFname->getVal().empty() == false)
    this->loadConfig(rep->paramLoadConfigFname->getVal());
}

// ##############################################################
void ModelManager::loadConfig(const string& fname)
{
  LINFO("Loading configuration from '%s'", fname.c_str());
  ParamMap pmap; pmap.load(fname.c_str());
  this->readParamsFrom(pmap);
}

// ##############################################################
void ModelManager::saveConfig() const
{
  ASSERT(rep != 0);
  if (rep->paramSaveConfigFname.get() != 0 &&
      rep->paramSaveConfigFname->getVal().empty() == false)
    this->saveConfig(rep->paramSaveConfigFname->getVal());
}

// ##############################################################
void ModelManager::saveConfig(const string& fname) const
{
  LINFO("Saving configuration to '%s'", fname.c_str());
  ParamMap pmap; this->writeParamsTo(pmap);
  pmap.format(fname);
}

// ##############################################################
void ModelManager::start1()
{
  ASSERT(rep != 0);

  // echo the command-line if the user requested that:
  if (rep->paramEchoArgs.getVal())
    {
      for (uint i = 0; i < rep->com.numArgs(); ++i)
        LINFO("argv[%u]='%s'", i, rep->com.getArg(i).c_str());
    }

  // report a bunch of settings:
  LINFO("Debug: %s, FPE: %s, FpuPrecision: %s, FpuRounding: %s, "
        "TestMode: %s, AtomicIntType: %s",
        rep->paramDebugMode.getVal() ? "ON" : "OFF",
        rep->paramUsingFPE.getVal() ? "ON" : "OFF",
        convertToString(getFpuPrecision()).c_str(),
        convertToString(getFpuRoundingMode()).c_str(),
        rep->paramTestMode.getVal() ? "ON" : "OFF",
        rutz::demangled_name(typeid(rutz::atomic_int_t)));

  // did we want a debug printout?
  if (rep->paramDebugMode.getVal()) {
    std::cerr<<"==================== MODEL PARAMETERS ====================\n";
    printout(std::cerr);
    std::cerr<<"==========================================================\n";
  }

  // if saveConfig() has not been called yet, let's do it now just
  // before we start all our components:
  if (rep->didSave == false) { rep->didSave = true; saveConfig(); }
}

// ##############################################################
void ModelManager::start2()
{
  ASSERT(rep != 0);

  rep->timer.reset();
}

// ##############################################################
void ModelManager::stop1()
{
  ASSERT(rep != 0);

  rep->timer.mark();
  rep->timer.report("from start() to stop()");
}

// ##############################################################
void ModelManager::paramChanged(ModelParamBase* const param,
                                const bool valueChanged,
                                ParamClient::ChangeStatus* status)
{
  ASSERT(rep != 0);

  ModelComponent::paramChanged(param, valueChanged, status);

  // change log verbosity level?
  if (param == &rep->paramLogVerb) {
    string v = rep->paramLogVerb.getVal();

    const int oldLogVerb = MYLOGVERB;

    if (v.compare("Debug") == 0) MYLOGVERB = LOG_DEBUG;
    else if (v.compare("Info") == 0) MYLOGVERB = LOG_INFO;
    else if (v.compare("Error") == 0) MYLOGVERB = LOG_ERR;
    else if (v.compare("Fatal") == 0) MYLOGVERB = LOG_CRIT;
    else if (v.compare("Default") == 0) MYLOGVERB = rep->userLogVerb;
    else
      LFATAL("Invalid log verbosity value '%s' (valid are: "
             "[Debug|Info|Error|Fatal|Default])", v.c_str());

    if (MYLOGVERB != oldLogVerb)
      LERROR("Switching log verbosity to %s", v.c_str());
  }

  // show full version info?
  if (param == &rep->paramShowVersion) {
    if (rep->paramShowVersion.getVal()) {
      // print the version to stdout and exit
      fprintf(stdout, "%s\n", fullversion());
      exit(0);
    }
  }

  // show svn repo version?
  else if (param == &rep->paramShowSvnVersion) {
    if (rep->paramShowSvnVersion.getVal()) {
      // print the version to stdout and exit
      fprintf(stdout, "%s\n", svnversion());
      exit(0);
    }
  }

  // check if the source is pristine?
  else if (param == &rep->paramCheckPristine) {
    if (rep->paramCheckPristine.getVal()) {
      exit(isSourcePristine() ? 0 : 1);
    }
  }

  // debug mode on/off?
  else if (param == &rep->paramDebugMode) {
    if (rep->paramDebugMode.getVal()) MYLOGVERB = LOG_DEBUG;
    else MYLOGVERB = rep->userLogVerb;
  }

  // floating-point exceptions on/off?
  else if (param == &rep->paramUsingFPE && valueChanged) {
    if (rep->paramUsingFPE.getVal()) fpExceptionsOn();
    else fpExceptionsOff();
  }

  // floating-point precision?
  else if (param == &rep->paramFpuPrecision) {
    setFpuPrecision(rep->paramFpuPrecision.getVal());
  }

  // floating-point rounding mode?
  else if (param == &rep->paramFpuRoundingMode) {
    setFpuRoundingMode(rep->paramFpuRoundingMode.getVal());
  }

  // test mode on/off?
  else if (param == &rep->paramTestMode) {
    if (rep->paramTestMode.getVal()) {
      // will produce deterministic pseudo-random sequences
      initRandomNumbersZero();
      setOptionValString(&OPT_UseRandom, "false");
    } else {
      initRandomNumbers();
      setOptionValString(&OPT_UseRandom, "true");
    }
  }

  // save profiling information?
  else if (param == &rep->paramProfileFile) {
    // NOTE: we require an explicit "none" instead of just the empty
    // string to request no profiling output -- that way, if certain
    // programs want to set up their own default location for the
    // profile file, then we won't accidentally mess up their default
    // with an empty string.
    if (rep->paramProfileFile.getVal().compare("none") == 0)
      {
        rutz::prof::print_at_exit(false);
      }
    else if (rep->paramProfileFile.getVal().empty())
      {
        // do nothing if it's an empty string, just leave things as
        // they are
      }
    else
      {
        rutz::prof::print_at_exit(true);
        rutz::prof::prof_summary_file_name
          (rep->paramProfileFile.getVal().c_str());

        // NOTE: we could also have a command-line option to control
        // whether the timing mode is RUSAGE (i.e. cpu usage) or
        // WALLCLOCK (i.e., real elapsed time)
        rutz::prof::set_timing_mode(rutz::prof::RUSAGE);
      }
  }

  else if (param == rep->paramLoadConfigFname.get())
    {
      // if processing this option just changed our loadConfigFile
      // parameter, that means that we just parsed an option that
      // specified that value. Let's load up the file immediately:
      loadConfig();
    }

  else if (param == &rep->paramMemCaching)
    {
      invt_allocation_allow_caching(rep->paramMemCaching.getVal());
    }
}

// ##############################################################
bool ModelManager::debugMode() const
{
  ASSERT(rep != 0);
  return rep->paramDebugMode.getVal();
}

// ##############################################################
void ModelManager::setFPE(bool val)
{
  ASSERT(rep != 0);
  rep->paramUsingFPE.setVal(val);
}

// ##############################################################
void ModelManager::clearExtraArgs()
{
  ASSERT(rep != 0);
  rep->com.clearExtraArgs();
}

// ##############################################################
void ModelManager::unRequestTestMode()
{ unRequestOption(rep->paramTestMode); }

// ##############################################################
void ModelManager::unRequestUsingFPE()
{ unRequestOption(rep->paramUsingFPE); }

// ##############################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
