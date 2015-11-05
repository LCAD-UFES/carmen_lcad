/*!@file Channels/SpectralResidualChannel.C "Spectral Residual" channel based on Hou&Zhang (CVPR 2007) */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SpectralResidualChannel.C $
// $Id: SpectralResidualChannel.C 12820 2010-02-11 05:44:51Z itti $
//

#ifndef CHANNELS_SPECTRALRESIDUALCHANNEL_C_DEFINED
#define CHANNELS_SPECTRALRESIDUALCHANNEL_C_DEFINED

#include "Channels/SpectralResidualChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/ChannelVisitor.H"
#include "Component/ModelOptionDef.H"
#include "Image/FilterOps.H"
#include "Image/FourierEngine.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "rutz/trace.h"

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelSaveOutputMap =
  { MODOPT_FLAG, "SpectralResidualChannelSaveOutputMap", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save output maps from the Spectral Residual channel (\"SRS\")",
    "save-srs-output", '\0', "", "false" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelSaveExtraOutput =
  { MODOPT_FLAG, "SpectralResidualChannelSaveExtraOutput", &MOC_CHANNEL, OPTEXP_SAVE,
    "Save additional output maps from the Spectral Residual channel",
    "save-srs-extra-output", '\0', "", "false" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelResizeSpec =
  { MODOPT_ARG(ResizeSpec), "SpectralResidualChannelResizeSpec", &MOC_CHANNEL, OPTEXP_CORE,
    "Specification for how the spectral residual channel should "
    "resize the input prior to the fft and subsequent operations. ",
    "srs-resize", '\0', "<None | WxH | *WFxHF | /WFxHF>", "64x64" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelSpectralBlur =
  { MODOPT_ARG(uint), "SpectralResidualChannelSpectralBlur", &MOC_CHANNEL, OPTEXP_CORE,
    "Size (in pixels) of the blur filter applied to the FFT "
    "log-magnitude image to produce the spectral residual image",
    "srs-spectral-blur", '\0', "<uint>", "3" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelOutputBlur =
  { MODOPT_ARG(float), "SpectralResidualChannelOutputBlur", &MOC_CHANNEL, OPTEXP_CORE,
    "Standard deviation of the Gaussian blur filter applied to the "
    "Spectral Residual channel output",
    "srs-output-blur", '\0', "<float>", "3.5" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelOutputBlurFactor =
  { MODOPT_ARG(double), "SpectralResidualOutputChannelBlurFactor", &MOC_CHANNEL, OPTEXP_CORE,
    "If non-zero, then this option overrides any value set with "
    "--srs-output-blur, such that the output blur filter width&height "
    "is set to this factor times the width&height of the channel's "
    "resized input.",
    "srs-output-blur-factor", '\0', "<double>", "0.0" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelDownSizeFilterWidth =
  { MODOPT_ARG(int), "SpectralResidualChannelDownSizeFilterWidth", &MOC_CHANNEL, OPTEXP_CORE,
    "Low-pass filter width used when downsizing input prior to "
    "processing in the Spectral Residual channel",
    "srs-lowpass-filter-width", '\0', "<int>", "8" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelAttenuationWidth =
  { MODOPT_ARG(double), "SpectralResidualChannelAttenuationWidth", &MOC_CHANNEL, OPTEXP_CORE,
    "Width across which the borders of the Spectral Residual output "
    "should be attenuated, expressed as a proportion of the output "
    "image size.",
    "srs-attenuation-width", '\0', "<double>", "0.0" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelOutputResize =
  { MODOPT_FLAG, "SpectralResidualChannelOutputResize", &MOC_CHANNEL, OPTEXP_CORE,
    "Whether or not to do any output resizing; if yes, then choose output "
    "dims according to --srs-output-resize-spec.",
    "srs-output-resize", '\0', "", "false" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelOutputResizeSpec =
  { MODOPT_ARG(ResizeSpec), "SpectralResidualChannelOutputResizeSpec", &MOC_CHANNEL, OPTEXP_CORE,
    "Specification for how the spectral residual channel should "
    "resize its output, relative to the original input size.",
    "srs-output-resize-spec", '\0', "<None | WxH | *WFxHF | /WFxHF>", "None" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelHiboostBypass =
  { MODOPT_FLAG, "SpectralResidualChannelHiboostBypass", &MOC_CHANNEL, OPTEXP_CORE,
    "Whether to bypass spectral-residual computations with a simple "
    "high-frequency boost instead.",
    "srs-hiboost-bypass", '\0', "", "false" };

// Used by: SpectralResidualChannel
const ModelOptionDef OPT_SpectralResidualChannelGradientBypass =
  { MODOPT_FLAG, "SpectralResidualChannelGradientBypass", &MOC_CHANNEL, OPTEXP_CORE,
    "Whether to bypass spectral-residual computations with a simple "
    "gradient magnitude computation instead.",
    "srs-gradient-bypass", '\0', "", "false" };

// ######################################################################
static Image<complexd> joinLogampliPhase(const Image<float>& logampli,
                                         const Image<double>& phase)
{
GVX_TRACE("SpectralResidualChannel::joinLogampliPhase");

  ASSERT(logampli.getDims() == phase.getDims());

  Image<complexd> newFFT(phase.getDims(), NO_INIT);

  const int size = newFFT.getSize();

  Image<complexd>::iterator const dptr = newFFT.beginw();
  Image<float>::const_iterator const aptr = logampli.begin();
  Image<double>::const_iterator const pptr = phase.begin();

  // normalization factor so that we get approximately the same
  // range of values regardless of the rescale size:
  const double div = 1.0 / sqrt(size);

  for (int i = 0; i < size; ++i)
    dptr[i] = std::exp(complexd(aptr[i], pptr[i])) * div;

  return newFFT;
}

// ######################################################################
SpectralResidualChannel::Downsizer::Downsizer()
  :
  itsInput(), itsFilterWidth(-1), itsPyr()
{
  if (0 != pthread_mutex_init(&itsMutex, NULL))
    PLFATAL("pthread_mutex_init() failed");
}

// ######################################################################
SpectralResidualChannel::Downsizer::~Downsizer()
{
  if (0 != pthread_mutex_destroy(&itsMutex))
    PLERROR("pthread_mutex_destroy() failed");
}

// ######################################################################
Image<float> SpectralResidualChannel::Downsizer::
getDownsized(const Image<float>& x, int filtwidth, const Dims& newdims)
{
GVX_TRACE("SRS::Downsizer::getDownsized");

  GVX_MUTEX_LOCK(&itsMutex);

  if (!itsInput.hasSameData(x) || filtwidth != itsFilterWidth)
    {
      itsInput = x;
      itsFilterWidth = filtwidth;
      itsPyr.resize(0);
      itsPyr.push_back(x);
      LINFO("initializing downsize cache");
    }
  else
    {
      LINFO("reusing downsize cache");
    }

  while (itsPyr.back().getWidth() > newdims.w() * 2
         && itsPyr.back().getHeight() > newdims.h() * 2)
    {
      Image<float> nextlev = itsPyr.back();
      if (itsFilterWidth == 1)
        {
          nextlev = decX(nextlev);
          nextlev = decY(nextlev);
        }
      else if (itsFilterWidth == 2)
        {
          nextlev = quickLocalAvg2x2(nextlev);
        }
      else
        {
          nextlev = decX(lowPassX(itsFilterWidth, nextlev));
          nextlev = decY(lowPassY(itsFilterWidth, nextlev));
        }
      itsPyr.push_back(nextlev);
    }

  size_t pyrlev = 0;

  while (itsPyr[pyrlev].getWidth() > newdims.w() * 2
         && itsPyr[pyrlev].getHeight() > newdims.h() * 2)
    {
      ASSERT(pyrlev + 1 < itsPyr.size());
      ++pyrlev;
    }

  ASSERT(pyrlev < itsPyr.size());
  ASSERT(pyrlev == 0 || itsPyr[pyrlev].getWidth() >= newdims.w());
  ASSERT(pyrlev == 0 || itsPyr[pyrlev].getHeight() >= newdims.h());
  ASSERT(itsPyr[pyrlev].getWidth() <= newdims.w() * 2
         || itsPyr[pyrlev].getHeight() <= newdims.h() * 2);

  return rescaleBilinear(itsPyr[pyrlev], newdims);
}

// ######################################################################
SpectralResidualChannel::SpectralResidualChannel
(OptionManager& mgr,
 const std::string& descrName,
 const std::string& tagName)
  :
  ChannelBase(mgr, descrName, tagName, UNKNOWN),
  itsSaveOutput(&OPT_SpectralResidualChannelSaveOutputMap, this),
  itsSaveExtraOutput(&OPT_SpectralResidualChannelSaveExtraOutput, this),
  itsResizeSpec(&OPT_SpectralResidualChannelResizeSpec, this),
  itsSpectralBlur(&OPT_SpectralResidualChannelSpectralBlur, this),
  itsOutputBlur(&OPT_SpectralResidualChannelOutputBlur, this),
  itsOutputBlurFactor(&OPT_SpectralResidualChannelOutputBlurFactor, this),
  itsDownSizeFilterWidth(&OPT_SpectralResidualChannelDownSizeFilterWidth, this),
  itsAttenuationWidth(&OPT_SpectralResidualChannelAttenuationWidth, this),
  itsDoResizeOutput(&OPT_SpectralResidualChannelOutputResize, this),
  itsOutputResizeSpec(&OPT_SpectralResidualChannelOutputResizeSpec, this),
  itsNormType(&OPT_MaxNormType, this), // see Channels/ChannelOpts.{H,C}
  itsOutputRangeMin(&OPT_ChannelOutputRangeMin, this),
  itsOutputRangeMax(&OPT_ChannelOutputRangeMax, this),
  itsHiboostBypass(&OPT_SpectralResidualChannelHiboostBypass, this),
  itsGradientBypass(&OPT_SpectralResidualChannelGradientBypass, this),
  itsDownsizer(new Downsizer),
  itsFFT(0),
  itsIFFT(0),
  itsInput(),
  itsOutput()
{
}

// ######################################################################
SpectralResidualChannel::~SpectralResidualChannel()
{
  delete itsFFT;
  delete itsIFFT;
}

// ######################################################################
void SpectralResidualChannel::accept(ChannelVisitor& v)
{
  v.visitChannelBase(*this);
}

// ######################################################################
bool SpectralResidualChannel::isHomogeneous() const
{
  return true;
}

// ######################################################################
void SpectralResidualChannel::readFrom(const ParamMap& pmap)
{
  ChannelBase::readFrom(pmap);
}

// ######################################################################
void SpectralResidualChannel::writeTo(ParamMap& pmap) const
{
  ChannelBase::writeTo(pmap);
}

// ######################################################################
bool SpectralResidualChannel::outputAvailable() const
{
  return itsInput.initialized();
}

// ######################################################################
Dims SpectralResidualChannel::getMapDims() const
{
  return
    itsDoResizeOutput.getVal()
    ? itsOutputResizeSpec.getVal().transformDims(this->getInputDims())
    : itsResizeSpec.getVal().transformDims(this->getInputDims());
}

// ######################################################################
uint SpectralResidualChannel::numSubmaps() const
{
  return 1;
}

// ######################################################################
Image<float> SpectralResidualChannel::getSubmap(const uint index) const
{
  if (index == 0)
    return const_cast<SpectralResidualChannel*>(this)->getOutput();

  LFATAL("submap index %u out of range; I have only %u submap(s)",
         index, this->numSubmaps());

  /* can't happen */ return Image<float>();
}

// ######################################################################
std::string SpectralResidualChannel::getSubmapName(const uint index) const
{
  if (index == 0) return "SpectralResidual";

  LFATAL("submap index %u out of range; I have only %u submap(s)",
         index, this->numSubmaps());

  /* can't happen */ return std::string();
}

// ######################################################################
std::string SpectralResidualChannel::getSubmapNameShort(const uint index) const
{
  if (index == 0) return "SpecRes";

  LFATAL("submap index %u out of range; I have only %u submap(s)",
         index, this->numSubmaps());

  /* can't happen */ return std::string();
}

// ######################################################################
void SpectralResidualChannel::getFeatures(const Point2D<int>& locn,
                                          std::vector<float>& mean) const
{
  LFATAL("not implemented");
}

// ######################################################################
void SpectralResidualChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                               std::vector<std::vector<float> > *mean,
                                               int *count) const
{
  LFATAL("not implemented");
}

// ######################################################################
Image<float> SpectralResidualChannel::getOutput()
{
GVX_TRACE("SRS::getOutput");

  if (!itsInput.initialized())
    LFATAL("I have no input yet!");

  if (!itsOutput.initialized())
    {
      const Dims newdims =
        itsResizeSpec.getVal().transformDims(itsInput.getDims());

      itsRescaledInput =
        itsDownsizer->getDownsized(itsInput,
                                   itsDownSizeFilterWidth.getVal(),
                                   newdims);

      if (itsFFT == 0)
        itsFFT = new FourierEngine<double>(itsRescaledInput.getDims());

      if (itsIFFT == 0)
        itsIFFT = new FourierInvEngine<double>(itsRescaledInput.getDims());

      if (itsGradientBypass.getVal())
        {
          if (itsHiboostBypass.getVal())
            LFATAL("can't use both --%s and --%s",
                   itsHiboostBypass.getOptionDef()->longoptname,
                   itsGradientBypass.getOptionDef()->longoptname);

          itsProtoSaliencyMap = gradientmag(itsRescaledInput);
        }
      else
        {
          const Image<complexd> myFFT = itsFFT->fft(itsRescaledInput);
          itsLogMagnitude = Image<float>(logmagnitude(myFFT));
          itsPhase = phase(myFFT);

          const int size = myFFT.getSize();

          if (itsHiboostBypass.getVal())
            {
              if (itsGradientBypass.getVal())
                LFATAL("can't use both --%s and --%s",
                       itsHiboostBypass.getOptionDef()->longoptname,
                       itsGradientBypass.getOptionDef()->longoptname);

              // multiply each fourier magnitude by the spatial
              // frequency; or, add log(freq) to the log magnitude

              itsSpectralResidual.resize(itsLogMagnitude.getDims());

              const int w = itsLogMagnitude.getWidth();
              const int h = itsLogMagnitude.getHeight();

              Image<float>::const_iterator sptr = itsLogMagnitude.begin();
              Image<float>::iterator dptr = itsSpectralResidual.beginw();

              const double logsize = log(size);

              for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x)
                  {
                    const int yf = std::min(y, h-y);

                    const double fsq = x*x + yf*yf;

                    *dptr++ = (*sptr++) + 0.5 * log(fsq) - logsize;
                  }
            }
          else if (itsSpectralBlur.getVal() < 1)
            {
              LFATAL("--%s must be >= 1",
                     itsSpectralBlur.getOptionDef()->longoptname);
            }
          else if (itsSpectralBlur.getVal() == 1)
            {
              itsSpectralResidual.resize(itsLogMagnitude.getDims(), true);
            }
          else
            {
              Image<float> filt(itsSpectralBlur.getVal(), 1, ZEROS);
              filt.clear(1.0f / itsSpectralBlur.getVal());

              itsSpectralResidual =
                itsLogMagnitude - sepFilter(itsLogMagnitude, filt, filt,
                                            CONV_BOUNDARY_REPLICATE);
            }

          const Image<complexd> newFFT = joinLogampliPhase(itsSpectralResidual, itsPhase);

          itsProtoSaliencyMap = itsIFFT->ifft(newFFT);
        }

      if (MYLOGVERB >= LOG_DEBUG)
        {
          float mi1, ma1; getMinMax(itsRescaledInput, mi1, ma1);
          float mi, ma; getMinMax(itsProtoSaliencyMap, mi, ma);
          LDEBUG("input range %f .. %f; proto range %f .. %f",
                 mi1, ma1, mi, ma);
        }

      itsProtoSaliencyMap = squared(itsProtoSaliencyMap);

      if (itsAttenuationWidth.getVal() > 0.0)
        {
          const int w = int(0.5
                            + itsAttenuationWidth.getVal()
                            * itsProtoSaliencyMap.getDims().max());

          inplaceAttenuateBorders(itsProtoSaliencyMap, w);
        }

      float fw =
        itsOutputBlurFactor.getVal() > 0.0
        ? itsOutputBlurFactor.getVal() * itsProtoSaliencyMap.getWidth()
        : itsOutputBlur.getVal();

      float fh =
        itsOutputBlurFactor.getVal() > 0.0
        ? itsOutputBlurFactor.getVal() * itsProtoSaliencyMap.getHeight()
        : itsOutputBlur.getVal();

      itsOutput = itsProtoSaliencyMap;

      const Dims mapdims = this->getMapDims();
      ASSERT(mapdims.isNonEmpty());

      // if we're going to be downscaling the image eventually, then
      // let's avoid convolving with huge gaussian filters in
      // sepFilter() below by first averaging+decimating the image
      // and, in parallel, shrinking the corresponding gaussian
      // filters, until we get close to the final desired dims
      while (itsOutput.getWidth() >= mapdims.w()*2
             && itsOutput.getHeight() >= mapdims.h()*2)
        {
          itsOutput = quickLocalAvg2x2(itsOutput);
          fw /= 2.0f;
          fh /= 2.0f;
        }

      Image<float> wfilt = gaussian<float>(0.0f, fw, 0, 1.0f);
      wfilt = wfilt / float(sum(wfilt));

      Image<float> hfilt = gaussian<float>(0.0f, fh, 0, 1.0f);
      hfilt = hfilt / float(sum(hfilt));

      LDEBUG("wfilt is %dx%d, hfilt is %dx%d",
             wfilt.getWidth(), wfilt.getHeight(),
             hfilt.getWidth(), hfilt.getHeight());

      itsOutput = sepFilter(itsOutput, wfilt, hfilt,
                            CONV_BOUNDARY_ZERO);

      // now resize the output to the requested ouput dims, or no-op
      // if the dims already match:
      itsOutput = rescaleBilinear(itsOutput, mapdims);

      itsOutput = maxNormalize(itsOutput,
                               itsOutputRangeMin.getVal(),
                               itsOutputRangeMax.getVal(),
                               itsNormType.getVal());
      LINFO("%s OK: in=%dx%d -> resize %s -> internal=%dx%d -> resize %s -> out=%dx%d; "
            "spectral blur %u; output blur %gx%g; lpwidth %d; atten width %g",
            this->descriptiveName().c_str(),
            itsInput.getWidth(), itsInput.getHeight(),
            convertToString(itsResizeSpec.getVal()).c_str(),
            newdims.w(), newdims.h(),
            itsDoResizeOutput.getVal()
            ? convertToString(itsOutputResizeSpec.getVal()).c_str()
            : "skip",
            mapdims.w(), mapdims.h(),
            itsSpectralBlur.getVal(), fw, fh,
            itsDownSizeFilterWidth.getVal(),
            itsAttenuationWidth.getVal());
    }

  return itsOutput;
}

// ######################################################################
void SpectralResidualChannel::saveResults(const nub::ref<FrameOstream>& ofs)
{
  const std::string tag = this->tagName();

  if (itsSaveOutput.getVal())
    ofs->writeFloat(this->getOutput(), FLOAT_NORM_PRESERVE,
                    tag+"-",
                    FrameInfo("Spectral Residual output",
                              SRC_POS));

  if (itsSaveExtraOutput.getVal())
    {
      ofs->writeGray(Image<byte>(itsInput), tag+"-input",
                     FrameInfo("Spectral Residual input", SRC_POS));

      ofs->writeFloat(itsRescaledInput, FLOAT_NORM_PRESERVE,
                      tag+"-rescaled-input",
                      FrameInfo("Spectral Residual rescaled input", SRC_POS));

      ofs->writeFloat(itsLogMagnitude, FLOAT_NORM_PRESERVE,
                      tag+"-logmagnitude",
                      FrameInfo("Spectral Residual fft log-magnitude",
                                SRC_POS));

      ofs->writeFloat(itsPhase, FLOAT_NORM_PRESERVE,
                      tag+"-phase",
                      FrameInfo("Spectral Residual fft phase",
                                SRC_POS));

      ofs->writeFloat(itsSpectralResidual, FLOAT_NORM_PRESERVE,
                      tag+"-spectral-residual",
                      FrameInfo("Spectral Residual fft spectral residual",
                                SRC_POS));

      ofs->writeFloat(itsProtoSaliencyMap, FLOAT_NORM_PRESERVE,
                      tag+"-proto-saliency",
                      FrameInfo("Spectral Residual proto-saliency",
                                SRC_POS));
    }
}

// ######################################################################
void SpectralResidualChannel::killCaches()
{
  itsOutput = Image<float>();

  // kill any previous intermediary results:
  itsRescaledInput.freeMem();
  itsLogMagnitude.freeMem();
  itsPhase.freeMem();
  itsSpectralResidual.freeMem();
  itsProtoSaliencyMap.freeMem();
}

// ######################################################################
void SpectralResidualChannel::doInput(const InputFrame& inframe)
{
  if (!inframe.grayFloat().initialized())
    LFATAL("Oops! I need luminance input");

  itsInput = inframe.grayFloat();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_SPECTRALRESIDUALCHANNEL_C_DEFINED
