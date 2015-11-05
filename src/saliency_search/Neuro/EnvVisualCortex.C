/*!@file Neuro/EnvVisualCortex.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvVisualCortex.C $
// $Id: EnvVisualCortex.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef NEURO_ENVVISUALCORTEX_C_DEFINED
#define NEURO_ENVVISUALCORTEX_C_DEFINED

#include "Neuro/EnvVisualCortex.H"

#include "Component/ModelOptionDef.H"
#include "Envision/env_c_math_ops.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_job_server.h"
#include "Envision/env_log.h"
#include "Envision/env_mt_visual_cortex.h"
#include "Envision/env_pthread_interface.h"
#include "Envision/env_stdio_interface.h"
#include "Neuro/EnvOpts.H"
#include "Util/AllocAux.H"
#include "Util/JobWithSemaphore.H"
#include "Util/StringConversions.H"
#include "Util/WorkThreadServer.H"

#include <cstdio>
#include <sstream>
#include <vector>

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcMaxnormType =
  { MODOPT_ARG(std::string), "EvcMaxnormType", &MOC_ENVISION, OPTEXP_CORE,
    "Type of normalization to use",
    "evc-maxnorm-type", '\0', "<None|Maxnorm>", "Maxnorm" };

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcScaleBits =
  { MODOPT_ARG(byte), "EvcScaleBits", &MOC_ENVISION, OPTEXP_CORE,
    "Number of bits of dynamic range to use",
    "evc-scale-bits", '\0', "<byte>", "16" };

#ifdef ENV_WITH_DYNAMIC_CHANNELS

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcNumDirections =
  { MODOPT_ARG(byte), "EvcNumDirections", &MOC_ENVISION, OPTEXP_CORE,
    "Number of motion directions to use",
    "evc-num-directions", '\0', "<byte>", "4" };

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcMotionThresh =
  { MODOPT_ARG(byte), "EvcMotionThresh", &MOC_ENVISION, OPTEXP_CORE,
    "Low threshold cutoff for motion channel",
    "evc-motion-thresh", '\0', "<byte>", "12" };

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcFlickerThresh =
  { MODOPT_ARG(byte), "EvcFlickerThresh", &MOC_ENVISION, OPTEXP_CORE,
    "Low threshold cutoff for flicker channel",
    "evc-flicker-thresh", '\0', "<byte>", "20" };

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcRangeThresh =
  { MODOPT_ARG(int), "EvcRangeThresh", &MOC_ENVISION, OPTEXP_CORE,
    "Low threshold cutoff for normalizing maps. "
    "If the range of the map is bellow this threshold "
    "then the value would be set to 0.",
    "evc-range-thresh", '\0', "<int>", "0" };

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcMultiScaleFlicker =
  { MODOPT_FLAG, "EvcMultiScaleFlicker", &MOC_ENVISION, OPTEXP_CORE,
    "Whether to use a true multi-scale flicker channel",
    "evc-multiscale-flicker", '\0', "", "true" };

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcShowMemStats =
  { MODOPT_FLAG, "EvcShowMemStats", &MOC_ENVISION, OPTEXP_CORE,
    "Whether to show memory usage",
    "evc-show-memstats", '\0', "", "false" };

#endif

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcNumOrientations =
  { MODOPT_ARG(byte), "EvcNumOrientations", &MOC_ENVISION, OPTEXP_CORE,
    "Number of orientation channels to use",
    "evc-num-orientations", '\0', "<byte>", "4" };

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcType =
  { MODOPT_ARG(std::string), "EvcType", &MOC_ENVISION, OPTEXP_CORE,
    "A string containing one or more of the characters "
#ifdef ENV_WITH_DYNAMIC_CHANNELS
    "'I', 'C', 'O', 'F', and 'M', "
#else
    "'I', 'C', and 'O', "
#endif
    "indicating which of the intensity, color, orientation, flicker, "
    "and motion channels should be included, respectively. Additionally, "
    "each character can optionally be followed by a ':' and a "
    "floating-point number between 0.0 and 1.0 indicating the weight for "
    "that channel. By default, each channel receives a weight of 1.0.",
    "evc-type", '\0',
#ifdef ENV_WITH_DYNAMIC_CHANNELS
    "<I:wC:wO:wF:wM:w>", "ICOFM"
#else
    "<I:wC:wO:w>", "ICO"
#endif
  };

// Used by: EnvVisualCortex
const ModelOptionDef OPT_EvcColorSmoothing =
  { MODOPT_FLAG, "EvcColorSmoothing", &MOC_ENVISION, OPTEXP_CORE,
    "Whether to do two-frame smoothing of the color channel",
    "evc-color-smoothing", '\0', "", "false" };

const ModelOptionDef OPT_EvcOutputFactor =
  { MODOPT_ARG(float), "EvcOutputFactor", &MOC_ENVISION, OPTEXP_CORE,
    "Factor applied to outputs of EnvVisualCortexFloat to scale them to Amps of "
    "synaptic input currents to saliency map",
    "evc-outfac", '\0', "<float>", "5.0e-18" };

// ######################################################################
// Thunk to convert from env_size_t to size_t
static void* malloc_thunk(env_size_t n)
{
  return malloc(n);
}

// ######################################################################
static Image<byte> convert_gray(const struct env_image* iimage, const struct env_dims dims)
{
  if (!env_img_initialized(iimage)) return Image<byte>(dims.w, dims.h, ZEROS);

  Image<byte> result(iimage->dims.w, iimage->dims.h, NO_INIT);

  const intg32* const src = env_img_pixels(iimage);
  const env_size_t sz = env_img_size(iimage);
  byte* bimage = result.getArrayPtr();

  for (env_size_t i = 0; i < sz; ++i)
    {
      // the caller is supposed to have already ensured that the intg32 image has been downsampled to a [0,255] range,
      // so let's verify that:
      ENV_ASSERT(src[i] >= 0 && src[i] <= 255);
      bimage[i] = byte(src[i]);
    }

  return result;
}

// ######################################################################
static Image<float> convert_gray_float(const struct env_image* iimage,
                                       const struct env_dims dims, const float factor)
{
  if (!env_img_initialized(iimage)) return Image<float>(dims.w, dims.h, ZEROS);

  Image<float> result(iimage->dims.w, iimage->dims.h, NO_INIT);

  const intg32* src = env_img_pixels(iimage);
  const env_size_t sz = env_img_size(iimage);
  float* fimage = result.getArrayPtr();
  if (factor == 1.0F) for (env_size_t i = 0; i < sz; ++i) { *fimage = *src; ++fimage; ++src; }
  else for (env_size_t i = 0; i < sz; ++i) { *fimage = (*src) * factor; ++fimage; ++src; }

  return result;
}

// ######################################################################
namespace
{
  class EnvisionJob : public JobWithSemaphore
  {
  public:
    EnvisionJob(const struct env_job* j) : envJob(*j) {}

    virtual ~EnvisionJob() {}

    virtual void run()
    {
      (*envJob.callback)(envJob.userdata);
      this->markFinished();
    }

    virtual const char* jobType() const { return "EnvisionJob"; }

    const struct env_job envJob;
  };
}

// ######################################################################
static void workthread_job_server(void* job_server_data,
                                  const struct env_job* jobs,
                                  const env_size_t njobs)
{
  if (njobs == 0) return;

  WorkThreadServer* srv = static_cast<WorkThreadServer*>(job_server_data);

  std::vector<rutz::shared_ptr<EnvisionJob> > ejobs;
  for (env_size_t i = 0; i < njobs; ++i)
    {
      ejobs.push_back(rutz::make_shared(new EnvisionJob(jobs + i)));
      srv->enqueueJob(ejobs.back());
    }

  for (size_t i = 0; i < ejobs.size(); ++i) ejobs[i]->wait();
}

// ######################################################################
EnvVisualCortexBase::EnvVisualCortexBase(OptionManager& mgr, const std::string& descrName,
                                         const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsIweight("EvcIntensityWeight", this, 255, ALLOW_ONLINE_CHANGES),
  itsCweight("EvcColorWeight", this, 255, ALLOW_ONLINE_CHANGES),
  itsOweight("EvcOrientationWeight", this, 255, ALLOW_ONLINE_CHANGES),
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  itsFweight("EvcFlickerWeight", this, 255, ALLOW_ONLINE_CHANGES),
  itsMweight("EvcMotionWeight", this, 255, ALLOW_ONLINE_CHANGES),
#endif
  itsMultithreaded(&OPT_EvcMultithreaded, this, ALLOW_ONLINE_CHANGES),
  itsMaxnormType(&OPT_EvcMaxnormType, this),
  itsScaleBits(&OPT_EvcScaleBits, this),
  itsNumOrientations(&OPT_EvcNumOrientations, this, ALLOW_ONLINE_CHANGES),
  itsColorSmoothing(&OPT_EvcColorSmoothing, this, ALLOW_ONLINE_CHANGES),
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  itsNumDirections(&OPT_EvcNumDirections, this, ALLOW_ONLINE_CHANGES),
  itsMotionThresh(&OPT_EvcMotionThresh, this, ALLOW_ONLINE_CHANGES),
  itsFlickerThresh(&OPT_EvcFlickerThresh, this, ALLOW_ONLINE_CHANGES),
  itsRangeThresh(&OPT_EvcRangeThresh, this, ALLOW_ONLINE_CHANGES),
  itsMultiScaleFlicker(&OPT_EvcMultiScaleFlicker, this, ALLOW_ONLINE_CHANGES),
  itsShowMemStats(&OPT_EvcShowMemStats, this, ALLOW_ONLINE_CHANGES),
#endif
  itsLevelSpec(&OPT_EnvLevelSpec, this),
  itsType(&OPT_EvcType, this)
{
  env_params_set_defaults(&this->envp);
}

// ######################################################################
EnvVisualCortexBase::~EnvVisualCortexBase()
{
  itsThreadServer.reset(0);
  env_set_job_server(0, 0);
  env_allocation_cleanup();
}

// ######################################################################
void EnvVisualCortexBase::start1()
{
  env_assert_set_handler(&env_stdio_assert_handler);
  env_allocation_init(&malloc_thunk, &free);

  LINFO(".scale_bits = %u", (unsigned int) itsScaleBits.getVal());

  this->envp.scale_bits = itsScaleBits.getVal();
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  this->envp.num_motion_directions = itsNumDirections.getVal();
  this->envp.motion_thresh = itsMotionThresh.getVal();
  this->envp.flicker_thresh = itsFlickerThresh.getVal();
  this->envp.range_thresh = itsRangeThresh.getVal();
  this->envp.multiscale_flicker = itsMultiScaleFlicker.getVal() ? 1 : 0;
#endif
  this->envp.num_orientations = itsNumOrientations.getVal();
  this->envp.cs_lev_min = itsLevelSpec.getVal().levMin();
  this->envp.cs_lev_max = itsLevelSpec.getVal().levMax();
  this->envp.cs_del_min = itsLevelSpec.getVal().delMin();
  this->envp.cs_del_max = itsLevelSpec.getVal().delMax();;
  this->envp.output_map_level = itsLevelSpec.getVal().mapLevel();

  env_visual_cortex_init(&this->ivc, &this->envp);

  this->npixels = 0;

  this->framenum = 0;

  itsIweight.setVal(this->envp.chan_i_weight);
  itsCweight.setVal(this->envp.chan_c_weight);
  itsOweight.setVal(this->envp.chan_o_weight);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  itsFweight.setVal(this->envp.chan_f_weight);
  itsMweight.setVal(this->envp.chan_m_weight);
#endif
}

// ######################################################################
void EnvVisualCortexBase::stop2()
{
  if (itsShowMemStats.getVal())
    {
      struct env_alloc_stats stats;
      env_allocation_get_stats(&stats);
      env_stdio_print_alloc_stats(&stats, this->npixels ? this->npixels : 1);
      invt_allocation_show_stats(1, "final", this->npixels);
    }

  env_visual_cortex_destroy(&this->ivc);
}

// ######################################################################
void EnvVisualCortexBase::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  if (this->started() && valueChanged)
    LDEBUG("online change of %s", param->getName().c_str());

  if      (param == &itsIweight) { this->envp.chan_i_weight = itsIweight.getVal(); }
  else if (param == &itsCweight) { this->envp.chan_c_weight = itsCweight.getVal(); }
  else if (param == &itsOweight) { this->envp.chan_o_weight = itsOweight.getVal(); }
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  else if (param == &itsFweight) { this->envp.chan_f_weight = itsFweight.getVal(); }
  else if (param == &itsMweight) { this->envp.chan_m_weight = itsMweight.getVal(); }
#endif
  else if (param == &itsMultithreaded)
    {
      env_set_job_server(0, 0);

      if (itsMultithreaded.getVal())
        {
          // once this is turned on, don't ever turn it off again
          env_init_pthread_alloc();

          itsThreadServer.reset(new WorkThreadServer("EnvVisualCortex", 12));
          env_set_job_server(&workthread_job_server, static_cast<void*>(itsThreadServer.get()));
        }
      else itsThreadServer.reset(0);
    }
  else if (param == &itsMaxnormType)
    {
      if (itsMaxnormType.getVal().compare("None") == 0) this->envp.maxnorm_type = ENV_VCXNORM_NONE;
      else if (itsMaxnormType.getVal().compare("Maxnorm") == 0) this->envp.maxnorm_type = ENV_VCXNORM_MAXNORM;
      else LFATAL("Invalid maxnorm type '%s' -- must be either 'Maxnorm' or 'None'", itsMaxnormType.getVal().c_str());
    }
  else if (param == &itsType)
    {
      // first, set all weights to zero:
      this->envp.chan_i_weight = 0;
      this->envp.chan_c_weight = 0;
      this->envp.chan_o_weight = 0;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
      this->envp.chan_f_weight = 0;
      this->envp.chan_m_weight = 0;
#endif

      const std::string type = itsType.getVal();
      const size_t len = itsType.getVal().length();

      for (size_t i = 0; i < len; /* incr in loop */)
        {
          const char chantype = type[i];
          byte* bweightptr = 0;

          switch (chantype)
            {
            case 'I': bweightptr = &this->envp.chan_i_weight; break;
            case 'C': bweightptr = &this->envp.chan_c_weight; break;
            case 'O': bweightptr = &this->envp.chan_o_weight; break;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
            case 'F': bweightptr = &this->envp.chan_f_weight; break;
            case 'M': bweightptr = &this->envp.chan_m_weight; break;
#endif
            default:  LFATAL("Invalid channel specifier '%c'", chantype);
            }

          ++i;

          if (type[i] != ':') *bweightptr = 255;
          else {
            ++i;
            const size_t end = type.find_first_not_of(".0123456789", i);
            const std::string weightstr = type.substr(i, end - i);
            double weight = 1.0;

            std::stringstream s; s << weightstr; s >> weight;
            if (s.fail()) LFATAL("couldn't parse '%c' channel weight from '%s'", chantype, weightstr.c_str());
            i = end;

            if (weight < 0.0 || weight > 1.0)
              LFATAL("invalid weight for channel '%c': got %s but expected a value between 0.0 and 1.0",
                     chantype, weightstr.c_str());

            *bweightptr = byte(weight * 255.0 + 0.5);
          }
        }
    }
  else if (param == &itsNumOrientations)
    {
      if (itsNumOrientations.getVal() > 99) *status = ParamClient::CHANGE_REJECTED;
      else this->envp.num_orientations = itsNumOrientations.getVal();
    }
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  else if (param == &itsNumDirections)
    {
      if (itsNumDirections.getVal() > 99) *status = ParamClient::CHANGE_REJECTED;
      else this->envp.num_motion_directions = itsNumDirections.getVal();
    }
  else if (param == &itsMotionThresh)
    {
      this->envp.motion_thresh = itsMotionThresh.getVal();
    }
  else if (param == &itsFlickerThresh)
    {
      this->envp.flicker_thresh = itsFlickerThresh.getVal();
    }
  else if (param == &itsMultiScaleFlicker)
    {
      this->envp.multiscale_flicker = itsMultiScaleFlicker.getVal() ? 1 : 0;
    }
#endif
}

// ######################################################################
// ######################################################################
// ######################################################################
EnvVisualCortex::EnvVisualCortex(OptionManager& mgr, const std::string& descrName,
                                 const std::string& tagName) :
  EnvVisualCortexBase(mgr, descrName, tagName)
{
  this->chanmi = INTG32_MAX; this->chanma = INTG32_MIN;
  this->vcxmi = INTG32_MAX; this->vcxma = INTG32_MIN;
}

// ######################################################################
EnvVisualCortex::~EnvVisualCortex()
{ }

// ######################################################################
void EnvVisualCortex::input(const Image<PixRGB<byte> >& rgbin)
{
  struct env_image ivcout = env_img_initializer;
  struct env_image intens = env_img_initializer;
  struct env_image color = env_img_initializer;
  struct env_image ori = env_img_initializer;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  struct env_image flicker = env_img_initializer;
  struct env_image motion = env_img_initializer;
#endif

  ++this->framenum;

  struct env_dims indims = { size_t(rgbin.getWidth()), size_t(rgbin.getHeight()) };

  npixels = indims.w * indims.h;

  const struct env_rgb_pixel* src = (const struct env_rgb_pixel*) rgbin.getArrayPtr();

  const struct env_rgb_pixel* src2 = (const struct env_rgb_pixel*)
    (itsColorSmoothing.getVal() == true && itsPrevRgb.initialized() ? itsPrevRgb.getArrayPtr() : 0);

  env_mt_visual_cortex_input(itsMultithreaded.getVal() ? 1 : 0,
                          &this->ivc, &this->envp,
                          "visualcortex",
                          src, src2,
                          indims,
                          0, //&print_chan_status,
                          0, //&userdata,
                          &ivcout,
                          &intens, &color, &ori
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                          , &flicker, &motion
#endif
                          );

  env_merge_range(&ivcout, &vcxmi, &vcxma);
  env_rescale_range_inplace(&ivcout, vcxmi, vcxma);

  env_visual_cortex_merge_ranges(&intens, &color, &ori,
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                                 &flicker, &motion,
#endif
                                 &chanmi, &chanma);

  env_rescale_range_inplace(&intens, chanmi, chanma);
  env_rescale_range_inplace(&color, chanmi, chanma);
  env_rescale_range_inplace(&ori, chanmi, chanma);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  env_rescale_range_inplace(&flicker, chanmi, chanma);
  env_rescale_range_inplace(&motion, chanmi, chanma);
#endif

  this->itsVCXmap = convert_gray(&ivcout, ivcout.dims);

  this->itsImap = convert_gray(&intens, ivcout.dims);
  this->itsCmap = convert_gray(&color, ivcout.dims);
  this->itsOmap = convert_gray(&ori, ivcout.dims);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  this->itsFmap = convert_gray(&flicker, ivcout.dims);
  this->itsMmap = convert_gray(&motion, ivcout.dims);
#endif

  env_img_make_empty(&ivcout);
  env_img_make_empty(&intens);
  env_img_make_empty(&color);
  env_img_make_empty(&ori);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  env_img_make_empty(&flicker);
  env_img_make_empty(&motion);
#endif

  itsPrevRgb = rgbin;
}

// ######################################################################
// ######################################################################
// ######################################################################
EnvVisualCortexFloat::EnvVisualCortexFloat(OptionManager& mgr, const std::string& descrName,
                                           const std::string& tagName) :
  EnvVisualCortexBase(mgr, descrName, tagName),
  itsOutputFactor(&OPT_EvcOutputFactor, this)
{ }

// ######################################################################
EnvVisualCortexFloat::~EnvVisualCortexFloat()
{ }

// ######################################################################
// ######################################################################
void EnvVisualCortexFloat::input(const Image<PixRGB<byte> >& rgbin)
{
  struct env_image ivcout = env_img_initializer;
  struct env_image intens = env_img_initializer;
  struct env_image color = env_img_initializer;
  struct env_image ori = env_img_initializer;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  struct env_image flicker = env_img_initializer;
  struct env_image motion = env_img_initializer;
#endif

  ++this->framenum;

  struct env_dims indims = { size_t(rgbin.getWidth()), size_t(rgbin.getHeight()) };

  npixels = indims.w * indims.h;

  const struct env_rgb_pixel* src = (const struct env_rgb_pixel*) rgbin.getArrayPtr();

  const struct env_rgb_pixel* src2 = (const struct env_rgb_pixel*)
    (itsColorSmoothing.getVal() == true && itsPrevRgb.initialized() ? itsPrevRgb.getArrayPtr() : 0);

  env_mt_visual_cortex_input(itsMultithreaded.getVal() ? 1 : 0,
                             &this->ivc, &this->envp,
                             "visualcortex",
                             src, src2,
                             indims,
                             0, //&print_chan_status,
                             0, //&userdata,
                             &ivcout,
                             &intens, &color, &ori
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                             , &flicker, &motion
#endif
                             );

  this->itsVCXmap = convert_gray_float(&ivcout, ivcout.dims, itsOutputFactor.getVal());

  this->itsImap = convert_gray_float(&intens, ivcout.dims, 1.0F);
  this->itsCmap = convert_gray_float(&color, ivcout.dims, 1.0F);
  this->itsOmap = convert_gray_float(&ori, ivcout.dims, 1.0F);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  this->itsFmap = convert_gray_float(&flicker, ivcout.dims, 1.0F);
  this->itsMmap = convert_gray_float(&motion, ivcout.dims, 1.0F);
#endif

  env_img_make_empty(&ivcout);
  env_img_make_empty(&intens);
  env_img_make_empty(&color);
  env_img_make_empty(&ori);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
  env_img_make_empty(&flicker);
  env_img_make_empty(&motion);
#endif

  itsPrevRgb = rgbin;
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_ENVVISUALCORTEX_C_DEFINED
