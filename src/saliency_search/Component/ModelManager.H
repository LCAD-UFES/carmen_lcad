/*!@file Component/ModelManager.H manages a model as collection of ModelComponents */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ModelManager.H $
// $Id: ModelManager.H 9335 2008-02-27 04:52:34Z rjpeters $
//

#ifndef MODELMANAGER_H_DEFINED
#define MODELMANAGER_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Util/StringConversions.H"

struct ModelOptionDef;
class ParamMap;

//! Manages a collection of ModelComponent objects forming a model
/*! ModelManager should be implemented once in each executable, and
  will facilitate the management of a collection of ModelComponent
  objects, load/save of tunable ModelParamBase parameters that may be
  in those objects, and generation/parsing of command-line
  options. ModelComponent objects must manually register with the
  ModelManager to fall under its management.  Typically, when a
  ModelComponent has subcomponents, only the top-level component
  should be registered with the ModelManager, as all ModelParamBase
  access functions will recurse through the ModelComponent's
  subcomponents. See test-model.C and test-audioGrab.C for simple
  examples of how to write executables that are managed by
  ModelManager. */
class ModelManager : public OptionManager, public ModelComponent
{
public:
  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  //! Default constructor
  /*! @param descrName descriptine name
    @param tagName tag name (see ModelComponent.H)
    @param loadCfgOpt create a --load-config-from=X option if true
    @param saveCfgOpt create a --save-config-to=X option if true
    @param autoLoadCfg try to load config from ~/.execname if true */
  ModelManager(const std::string& descrName = "The Model",
               const std::string& tagName = "model",
               const bool loadCfgOpt = true, const bool saveCfgOpt = true,
               const bool autoLoadCfg = true);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~ModelManager();

  //! Returns true if we are running in debug mode
  /*! Debug mode can be invoked via the --debug command-line
    option. Programs can use this function to do additional debug
    outputs if the manager is in debug mode. In addition, we will
    generate a debug printout of all model parameters just before
    start() and we will force the log level to LOG_DEBUG (see log.H)
    if debug mode is on. */
  bool debugMode() const;

  //@}

  // ######################################################################
  /*! @name Model option database */
  //@{

  //! Only allow ModelOptionDef's that match the given mask
  /*! This mask is used to filter requestOption() calls -- if the
      ModelOptionDef's filterFlag isn't part of ModelManager's current
      mask, then the requestOption() is ignored. */
  void allowOptions(const int mask);

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
                             const bool useMyVal = false);

  //! Request the removal of a param from the command-line options
  /*! @param p the ModelParam data member of c corresponding to the option */
  virtual void unRequestOption(OptionedModelParam& p);

  //! Users may request model option aliases
  /*! @param name the name of the option alias, from ModelOptionDefs.C */
  virtual void requestOptionAlias(const ModelOptionDef* def);

  //! Set an option value
  /*! All ModelParam objects associated this ModelOptionDef will be
      updated with the new value. Will throw a fatal error if the
      model has been started (see ModelComponent::start()). */
  virtual void setOptionValString(const ModelOptionDef* def,
                                  const std::string& val);

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
  virtual std::string getOptionValString(const ModelOptionDef* def);

  //! Check if anybody has requested the given option.
  virtual bool isOptionRequested(const ModelOptionDef* def) const;

  //@}

  // ######################################################################
  /*! @name Alternative access to internal options

      These functions may be useful for alternative methods of
      displaying the help for command-line options (e.g. in the Qt
      ModelManagerWizard).
   */
  //@{

  //! Get the number of ModelOptionDef objects in our list.
  virtual uint numOptionDefs() const;

  //! Get ModelOptionDef objects from our list, up to the size of the given array.
  /*! Returns the number of array entries that were actually filled
    with valid ModelOptionDef pointers. */
  virtual uint getOptionDefs(const ModelOptionDef** arr, uint narr);

  //! Get the ModelOptionDef for the given name.
  /*! Aborts if there is no such ModelOptionDef. */
  virtual const ModelOptionDef* findOptionDef(const char* name) const;

  //! Query if the given ModelOptionDef is used in this program.
  virtual bool isOptionDefUsed(const ModelOptionDef* def) const;

  //@}

  // ######################################################################
  /*! @name Command-line parsing */
  //@{

  //! Parse all command-line options and set our params
  /*! This function will create a set of command-line options from our
      current collection of OptionedModelParam objects. It will then
      parse the command-line arguments and set the various option
      values accordingly, using a call to setOptionValString(). Any
      left-over args (i.e., not formatted as options) will be
      available through getExtraArg(), and if the number of such extra
      args is not within the specified minarg/maxarg bounds, a usage
      message and optionlist will be printed out (also printed out if
      a '--help' option is encountered). To decide which options
      should be exported by your various ModelComponent objects,
      typically you will call exportOptions() on the manager, which
      will propagate the call to all registered components. Here we
      have an additional default behavior that if exportOptions() has
      not been called on us, we will call it with an order to export
      all options, just before we start parsing the command-line. So,
      if you want to export all options, you don't have to do
      anything; if you want to only export special classes of options,
      then you will need to call exportOptions() manually some time
      before you parse the command-line. Returns true upon fully
      successful parse.

      @param argc the number of command-line args
      @param argv the command-line args
      @param usage a textual description of the expected usage format,
        describing only what the non-option arguments should be (e.g.,
        "<input.ppm> <output.ppm>")
      @param minarg the minimum number of non-optional arguments
      @param maxarg the maximum number of non-optional args, or -1 if
      unbounded */
  bool parseCommandLine(const int argc, const char **argv,
                        const char *usage,
                        const int minarg, const int maxarg);

  //! Overload that takes a argv array of non-const strings
  bool parseCommandLine(const int argc, char* const* argv,
                        const char* usage,
                        const int minarg, const int maxarg);

  //! The core of parseCommandLine().
  /*! This may be called recursively to handle command-line
      aliases. Note: beware that the first element of argv will be
      ignored (as it usually is the program name). */
  bool parseCommandLineCore(const int argc, const char** argv);

  bool parseCommandLineCore(const char* args);


  //! Get the number of available extra args
  uint numExtraArgs() const;

  //! Get an extra arg, by number
  std::string getExtraArg(const uint num) const;

  //! Get an extra arg converted to type T
  template <class T>
  inline T getExtraArgAs(const uint num) const;

  //! Clear our list of leftover command-line arguments
  void clearExtraArgs();

  //@}

  // ######################################################################
  /*! @name Persistent ModelParamBase management */
  //@{

  //! Load all our ModelParamBase values from file
  /*! This function is automatically called during parseCommandLine()
    if --load-config-from=XX option is encountered or autoload was
    specified in our constructor (thus ensuring that command-line
    options encountered after the option specifying which config file
    to load are applied after the config file has been loaded). It may
    also be called manually. This function is a no-op if no filename
    is in our loadConfigFile internal ModelParam (this parameter can
    be set by the --load-config-from=X command-line option).
    Otherwise, load the file (which should be a ParamMap) and
    configure all our parameters. */
  void loadConfig();

  //! Explicitly load all our ModelParams from file
  /*! This function will always load the params from the given file,
    even is --load-config-from=X was not specified. */
  void loadConfig(const std::string& fname);

  //! Save all our ModelParams to file
  /*! This function is a no-op if no filename is in our saveConfigFile
    internal ModelParam (this parameter can be set by the
    --save-config-to=X command-line option). Otherwise, save all our
    params to the file as a ParamMap. Returns true on success. You
    should typically call this either some time before you start()
    your model, or after you have stop()'ed it.  NOTE: There is a
    default behavior which is to call this as the first thing in
    start(), i.e., just before all our registered components will be
    started, unless you have manually called it yourself earlier, in
    which case it is not called again. */
  void saveConfig() const;

  //! Explicitly save all our ModelParams to file
  /*! This function will always save the params to the given file,
    even is --save-config-to=X was not specified. */
  void saveConfig(const std::string& fname) const;

  //@}

  //! Intercept people changing our ModelParam's
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

protected:
  // these are essentially here for the benefit of MexModelManager (we
  // offer these protected member functions, so that all of our member
  // data can be private)
  void setFPE(bool val);
  void unRequestTestMode();
  void unRequestUsingFPE();

  // start and saveConfig first if not saved yet
  virtual void start1();

  // start cpu timer
  virtual void start2();

  // stop cpu timer
  virtual void stop1();

private:
  ModelManager(const ModelManager&);
  ModelManager& operator=(const ModelManager&);

  class Impl;
  Impl* const rep;
};

// ######################################################################
// Inline functions

// ######################################################################
template <class T>
inline T ModelManager::getExtraArgAs(const uint num) const
{
  T val;
  convertFromString(this->getExtraArg(num), val);
  return val;
}

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
