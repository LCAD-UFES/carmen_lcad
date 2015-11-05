/*!@file Neuro/EnvVisualCortex.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvVisualCortex.H $
// $Id: EnvVisualCortex.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef NEURO_ENVVISUALCORTEX_H_DEFINED
#define NEURO_ENVVISUALCORTEX_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Envision/env_params.h"
#include "Envision/env_types.h"
#include "Envision/env_visual_cortex.h"
#include "Image/Image.H"
#include "Image/LevelSpec.H"
#include "Image/Pixels.H"
#include "rutz/shared_ptr.h"

class WorkThreadServer;

extern const ModelOptionDef OPT_EvcMaxnormType;
extern const ModelOptionDef OPT_EvcScaleBits;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
extern const ModelOptionDef OPT_EvcNumDirections;
extern const ModelOptionDef OPT_EvcMotionThresh;
extern const ModelOptionDef OPT_EvcFlickerThresh;
extern const ModelOptionDef OPT_EvcMultiScaleFlicker;
extern const ModelOptionDef OPT_EvcShowMemStats;
#endif
extern const ModelOptionDef OPT_EvcNumOrientations;
extern const ModelOptionDef OPT_EvcType;
extern const ModelOptionDef OPT_EvcColorSmoothing;
extern const ModelOptionDef OPT_EvcOutputFactor;

// ######################################################################
//! Base class for a VisualCortex using the fast envision saliency computations
class EnvVisualCortexBase : public ModelComponent
{
public:
  //! Constructor
  EnvVisualCortexBase(OptionManager& mgr, const std::string& descrName = "EnvVisualCortexBase",
                  const std::string& tagName = "EnvVisualCortexBase");

  //! virtual destructor for safe inheritance
  virtual ~EnvVisualCortexBase();

  //! Return the pyramid level of the saliency map
  inline env_size_t getMapLevel() const;

  //! Compute saliency map from an input image
  /*! Use the getVCXmap(), getImap(), etc to get the results. In the base class input() is pure virtual and the
      getVCXmap() and others do not exist. Derived classes implement all these and you should use these derived
      classes. */
  virtual void input(const Image<PixRGB<byte> >& rgbin) = 0;

  inline byte getIweight();
  inline byte getCweight();
  inline byte getOweight();
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  inline byte getFweight();
  inline byte getMweight();
#endif

  //! Access our internal envision parameters
  inline const struct env_params& getParams() const;

  inline bool getColorSmoothing() const;
  inline void setColorSmoothing(bool val);

  inline void setIweight(byte val);
  inline void setCweight(byte val);
  inline void setOweight(byte val);
  inline void setNumOrientations(byte val);

#ifdef ENV_WITH_DYNAMIC_CHANNELS
  inline void setFweight(byte val);
  inline void setMweight(byte val);
  inline void setFthresh(byte val);
  inline void setMthresh(byte val);
  inline void setMultiScaleFlicker(bool val);
  inline void setNumDirections(byte val);
#endif

#ifdef ENV_WITH_VISIT_CHANNEL
  inline void setSubmapPreProc(int (*submapPreProc)(const char* tagName,
            env_size_t clev, env_size_t slev, struct env_image *submap,
            const struct env_image *center, const struct env_image *surround));


  inline void setSubmapPostNormProc(int (*submapPostNormProc)(const char* tagName,
            env_size_t clev, env_size_t slev, struct env_image *submap,
            const struct env_image *center, const struct env_image *surround));

  inline void setSubmapPostProc(int (*submapPostProc)(const char* tagName,
            struct env_image *cmap));
#endif

protected:
  virtual void paramChanged(ModelParamBase* const param, const bool valueChanged,
                            ParamClient::ChangeStatus* status);
  virtual void start1();
  virtual void stop2();


  NModelParam<byte> itsIweight;
  NModelParam<byte> itsCweight;
  NModelParam<byte> itsOweight;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  NModelParam<byte> itsFweight;
  NModelParam<byte> itsMweight;
#endif
  OModelParam<bool> itsMultithreaded;
  OModelParam<std::string> itsMaxnormType;
  OModelParam<byte> itsScaleBits;
  OModelParam<byte> itsNumOrientations;
  OModelParam<bool> itsColorSmoothing;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  OModelParam<byte> itsNumDirections;
  OModelParam<byte> itsMotionThresh;
  OModelParam<byte> itsFlickerThresh;
  OModelParam<int> itsRangeThresh;
  OModelParam<bool> itsMultiScaleFlicker;
  OModelParam<bool> itsShowMemStats;
#endif
  OModelParam<LevelSpec> itsLevelSpec;
  OModelParam<std::string> itsType;

  struct env_params envp;
  struct env_visual_cortex ivc;
  env_size_t npixels;
  int framenum;
  Image<PixRGB<byte> > itsPrevRgb;
  rutz::shared_ptr<WorkThreadServer> itsThreadServer;
};

// ######################################################################
//! A VisualCortex using the fast envision saliency computations, version with byte outputs
class EnvVisualCortex : public EnvVisualCortexBase
{
public:
  //! Constructor
  EnvVisualCortex(OptionManager& mgr, const std::string& descrName = "EnvVisualCortex",
                  const std::string& tagName = "EnvVisualCortex");

  //! virtual destructor for safe inheritance
  virtual ~EnvVisualCortex();

  //! Compute saliency map from an input image
  /*! Use the getVCXmap(), getImap(), etc to get the results. */
  virtual void input(const Image<PixRGB<byte> >& rgbin);

  inline Image<byte> getVCXmap() const;

  inline Image<byte> getImap() const;
  inline Image<byte> getCmap() const;
  inline Image<byte> getOmap() const;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  inline Image<byte> getFmap() const;
  inline Image<byte> getMmap() const;
#endif

protected:
  Image<byte> itsVCXmap;

  Image<byte> itsImap;
  Image<byte> itsCmap;
  Image<byte> itsOmap;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  Image<byte> itsFmap;
  Image<byte> itsMmap;
#endif

private:
  intg32 chanmi, chanma;
  intg32 vcxmi, vcxma;
};

// ######################################################################
//! A VisualCortex using the fast envision saliency computations, version with float outputs
class EnvVisualCortexFloat : public EnvVisualCortexBase
{
public:
  //! Constructor
  EnvVisualCortexFloat(OptionManager& mgr, const std::string& descrName = "EnvVisualCortexFloat",
                       const std::string& tagName = "EnvVisualCortexFloat");

  //! virtual destructor for safe inheritance
  virtual ~EnvVisualCortexFloat();

  //! Compute saliency map from an input image
  /*! Use the getVCXmap(), getImap(), etc to get the results. */
  virtual void input(const Image<PixRGB<byte> >& rgbin);

  inline Image<float> getVCXmap() const;

  inline Image<float> getImap() const;
  inline Image<float> getCmap() const;
  inline Image<float> getOmap() const;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  inline Image<float> getFmap() const;
  inline Image<float> getMmap() const;
#endif

protected:
  OModelParam<float> itsOutputFactor;   //!< output range factor

  Image<float> itsVCXmap;
  Image<float> itsImap;
  Image<float> itsCmap;
  Image<float> itsOmap;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  Image<float> itsFmap;
  Image<float> itsMmap;
#endif
};


// ######################################################################
// inline functions for EnvVisualCortexBase:

env_size_t EnvVisualCortexBase::getMapLevel() const { return this->envp.output_map_level; }
byte EnvVisualCortexBase::getIweight() { return envp.chan_i_weight; }
byte EnvVisualCortexBase::getCweight() { return envp.chan_c_weight; }
byte EnvVisualCortexBase::getOweight() { return envp.chan_o_weight; }
#ifdef ENV_WITH_DYNAMIC_CHANNELS
byte EnvVisualCortexBase::getFweight() { return envp.chan_f_weight; }
byte EnvVisualCortexBase::getMweight() { return envp.chan_m_weight; }
#endif
const struct env_params& EnvVisualCortexBase::getParams() const { return envp; }
bool EnvVisualCortexBase::getColorSmoothing() const { return itsColorSmoothing.getVal(); }
void EnvVisualCortexBase::setIweight(byte val) { envp.chan_i_weight = val; }
void EnvVisualCortexBase::setCweight(byte val) { envp.chan_c_weight = val; }
void EnvVisualCortexBase::setOweight(byte val) { envp.chan_o_weight = val; }
void EnvVisualCortexBase::setNumOrientations(byte val) { envp.num_orientations = val; }

#ifdef ENV_WITH_DYNAMIC_CHANNELS
void EnvVisualCortexBase::setFweight(byte val) { envp.chan_f_weight = val; }
void EnvVisualCortexBase::setMweight(byte val) { envp.chan_m_weight = val; }
void EnvVisualCortexBase::setFthresh(byte val) { envp.flicker_thresh = val; }
void EnvVisualCortexBase::setMthresh(byte val) { envp.motion_thresh = val; }
void EnvVisualCortexBase::setMultiScaleFlicker(bool val) { envp.multiscale_flicker = val ? 1 : 0; }
void EnvVisualCortexBase::setNumDirections(byte val) { envp.num_motion_directions = val; }
#endif

void EnvVisualCortexBase::setColorSmoothing(bool val) { itsColorSmoothing.setVal(val); }

#ifdef ENV_WITH_VISIT_CHANNEL
void EnvVisualCortexBase::setSubmapPreProc(int (*submapPreProc)(const char* tagName,
            env_size_t clev, env_size_t slev, struct env_image *submap,
            const struct env_image *center, const struct env_image *surround))
{ envp.submapPreProc = submapPreProc; }

void EnvVisualCortexBase::setSubmapPostNormProc(int (*submapPostNormProc)(const char* tagName,
            env_size_t clev, env_size_t slev, struct env_image *submap,
            const struct env_image *center, const struct env_image *surround))
{ envp.submapPostNormProc = submapPostNormProc; }

void EnvVisualCortexBase::setSubmapPostProc(int (*submapPostProc)(const char* tagName,
            struct env_image *cmap))
{ envp.submapPostProc = submapPostProc; }
#endif


// ######################################################################
// Inline functions for EnvVisualCortex

Image<byte> EnvVisualCortex::getVCXmap() const { return itsVCXmap; }
Image<byte> EnvVisualCortex::getImap() const { return itsImap; }
Image<byte> EnvVisualCortex::getCmap() const { return itsCmap; }
Image<byte> EnvVisualCortex::getOmap() const { return itsOmap; }
#ifdef ENV_WITH_DYNAMIC_CHANNELS
Image<byte> EnvVisualCortex::getFmap() const { return itsFmap; }
Image<byte> EnvVisualCortex::getMmap() const { return itsMmap; }
#endif

// ######################################################################
// Inline functions for EnvVisualCortexFloat

Image<float> EnvVisualCortexFloat::getVCXmap() const { return itsVCXmap; }
Image<float> EnvVisualCortexFloat::getImap() const { return itsImap; }
Image<float> EnvVisualCortexFloat::getCmap() const { return itsCmap; }
Image<float> EnvVisualCortexFloat::getOmap() const { return itsOmap; }
#ifdef ENV_WITH_DYNAMIC_CHANNELS
Image<float> EnvVisualCortexFloat::getFmap() const { return itsFmap; }
Image<float> EnvVisualCortexFloat::getMmap() const { return itsMmap; }
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_ENVVISUALCORTEX_H_DEFINED
