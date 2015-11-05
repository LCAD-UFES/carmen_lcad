/*!@file Channels/SoxChannel.C Shortrange Orientation Interactions channel */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SoxChannel.C $
// $Id: SoxChannel.C 8857 2007-10-18 23:38:04Z rjpeters $
//

#include "Channels/SoxChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Component/ParamMap.H"
#include "Image/MathOps.H" // for toPower
#include "Image/ShapeOps.H" // for rescale
#include "Image/fancynorm.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/log.H"

#include <algorithm> // for std::swap
#include <cmath> // for exp

static const ModelOptionDef OPT_SoxThetaPoolWidth =
  { MODOPT_ARG(double), "SoxThetaPoolWidth", &MOC_CHANNEL, OPTEXP_CORE,
    "The width of the inhibitory pool in the orientation dimension in the "
    "SoxChannel (short-range orientation interactions), corresponding to "
    "'Sigma_theta' in section 2.7 of VisRes2005.",
    "sox-theta-pool-width", '\0', "<double>", "20.0" };

static const ModelOptionDef OPT_SoxOmegaPoolWidth =
  { MODOPT_ARG(double), "SoxOmegaPoolWidth", &MOC_CHANNEL, OPTEXP_CORE,
    "The width of the inhibitory pool in the frequency dimension in the "
    "SoxChannel (short-range orientation interactions), corresponding to "
    "'Sigma_omega' in section 2.7 of VisRes2005.",
    "sox-omega-pool-width", '\0', "<double>", "0.001" };

static const ModelOptionDef OPT_SoxInhibExponent =
  { MODOPT_ARG(double), "SoxInhibExponent", &MOC_CHANNEL, OPTEXP_CORE,
    "The exponent to which inhibitory contributions are raised in the "
    "SoxChannel (short-range orientation interactions), corresponding to "
    "'delta' section 2.7 of VisRes2005.",
    "sox-inhib-exponent", '\0', "<double>", "1.5" };

static const ModelOptionDef OPT_SoxExcitExponent =
  { MODOPT_ARG(double), "SoxExcitExponent", &MOC_CHANNEL, OPTEXP_CORE,
    "The exponent to which excitatory contributions are raised in the "
    "SoxChannel (short-range orientation interactions), corresponding to "
    "'gamma' section 2.7 of VisRes2005.",
    "sox-excit-exponent", '\0', "<double>", "2.0" };

static const ModelOptionDef OPT_SoxSemiSaturation =
  { MODOPT_ARG(double), "SoxSemiSaturation", &MOC_CHANNEL, OPTEXP_CORE,
    "The semi-saturation constant added to the divisive inhibition term "
    "in the SoxChannel (short-range orientation interactions), corresponding "
    "to 'S' section 2.7 of VisRes2005.",
    "sox-semi-saturation", '\0', "<double>", "1.0" };

static const ModelOptionDef OPT_SoxCutoff =
  { MODOPT_ARG(double), "SoxCutoff", &MOC_CHANNEL, OPTEXP_CORE,
    "As a performance optimization, in the SoxChannel (short-range "
    "orientation interactions), inhibitory terms will be dropped from "
    "further consideration if their weight ('W' in section 2.7 of "
    "VisRes2005) is less than this cutoff value.",
    "sox-cutoff", '\0', "<double>", "0.00001" };

static const ModelOptionDef OPT_ALIASsoxModel0037 =
  { MODOPT_ALIAS, "ALIASsoxModel0037", &MOC_ALIAS, OPTEXP_CORE,
    "Sox model #0037",
    "sox-model-0037", '\0', "",
    "--num-orient=6 "
    "--sox-cutoff=1e-4 "
    "--sox-excit-exponent=4.0 "
    "--sox-inhib-exponent=3.5 "
    "--sox-omega-pool-width=0.8 "
    "--sox-semi-saturation=1.0 "
    "--sox-theta-pool-width=20.0 "
  };

// ######################################################################
SoxChannel::SoxChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "Sox", "sox", ORI),
  itsNumOrients(&OPT_NumOrientations, this, 6, USE_MY_VAL),
  thetaPoolWidth(&OPT_SoxThetaPoolWidth, this),
  omegaPoolWidth(&OPT_SoxOmegaPoolWidth, this),
  inhibExponent(&OPT_SoxInhibExponent, this),
  excitExponent(&OPT_SoxExcitExponent, this),
  semiSaturation(&OPT_SoxSemiSaturation, this),
  cutoff(&OPT_SoxCutoff, this)
{
  mgr.requestOptionAlias(&OPT_ALIASsoxModel0037);

  // let's build our channels; we may have to re-build them if
  // itsNumOrient get changed on us before we start():
  buildSubChans();
}

// ######################################################################
void SoxChannel::buildSubChans()
{
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our Gabor subchannels now that we know how many
  // we want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  LINFO("Using %d orientations spanning [0..180]deg", itsNumOrients.getVal());
  for (uint ori = 0; ori < itsNumOrients.getVal(); ++ori)
    {
      nub::ref<GaborChannel> chan
        (makeSharedComp
         (new GaborChannel(getManager(),
                           ori, 180.0 * double(ori) /
                           double(itsNumOrients.getVal()))));

      chan->setComputeFullPyramid(true);

      this->addSubChan(chan);

      // let's export options on the newly built channel:
      chan->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void SoxChannel::paramChanged(ModelParamBase* const param,
                              const bool valueChanged,
                              ParamClient::ChangeStatus* status)
{
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumOrients &&
      numChans() != itsNumOrients.getVal())
    buildSubChans();
}

// ######################################################################
nub::ref<GaborChannel> SoxChannel::gabor(const uint idx) const
{ return dynCast<GaborChannel>(subChan(idx)); }

// ######################################################################
SoxChannel::~SoxChannel()
{  }

// ######################################################################
uint SoxChannel::numScales() const
{ return gabor(0)->getLevelSpec().maxDepth(); }

// ######################################################################
void SoxChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.grayFloat().initialized());

  // compute oriented gabor pyramids in several basis directions:
  for (uint ii = 0; ii < numChans(); ++ii)
    {
      gabor(ii)->input(inframe);
      LINFO("Orientation pyramid (%d/%d) ok.", ii+1, numChans());
    }
}

// ######################################################################
Image<float> SoxChannel::getLinearResponse(int ori, int scl)
{
  const Image<float> result = gabor(ori)->getImage(scl);
  ASSERT(result.initialized());
  return result;
}

// ######################################################################
Image<float> SoxChannel::getNonlinearResponse(int exc_ori, int exc_scl)
{
  const Dims inp_dims = getInputDims();

  const Image<float> exc_img = getLinearResponse(exc_ori, exc_scl);

  ASSERT(exc_img.initialized());

  const double exc_theta = gabor(exc_ori)->angle();

  const double exc_frq = log(exc_img.getWidth() / double(inp_dims.w()));

  Image<float> inh_pool(exc_img.getDims(), NO_INIT);
  inh_pool.clear(semiSaturation.getVal());

  int kept = 0;
  int skipped = 0;

  LINFO("orientation %d/%d, scale %d/%d",
        exc_ori+1, numChans(),
        exc_scl+1, numScales());

  double totalFactorW = 0.0;

  for (uint inh_ori = 0; inh_ori < numChans(); ++inh_ori)
    {
      const double inh_theta = gabor(inh_ori)->angle();

      for (uint inh_scl = 0; inh_scl < numScales(); ++inh_scl)
        {
          const Image<float> inh_img = getLinearResponse(inh_ori, inh_scl);

          const double inh_frq = log(inh_img.getWidth()/double(inp_dims.w()));

          double theta_diff = exc_theta-inh_theta;
          if (theta_diff <= -90.0) theta_diff += 180.0;
          else if (theta_diff > 90.0) theta_diff -= 180.0;

          const double factorW =
            exp(-squareOf(theta_diff)/(2*squareOf(thetaPoolWidth.getVal()))
                -squareOf(exc_frq-inh_frq)/(2*squareOf(omegaPoolWidth.getVal())));

          totalFactorW += factorW;

          // Performance optimization: we choose some cutoff level for
          // factorW (which determines how much inhibitory weights the
          // other orientation/scale combinations have on the current
          // orientation/scale); below the cutoff level, we just ignore the
          // other orientation/scale entirely.
          if (factorW < cutoff.getVal())
            {
              ++skipped;
            }
          else
            {
              ++kept;

              LDEBUG("factorW: (%.1f,%f,%.1f,%f) %f",
                     exc_theta, exc_frq, inh_theta, inh_frq, factorW);

              // FIXME speedup by caching the exponentiated img's
              inplaceAddWeighted(inh_pool,
                                 getInhib(inh_ori, inh_scl, exc_scl,
                                          inh_pool.getDims(), inh_img),
                                 factorW);
            }
        }
    }

  LINFO("total inh weight: %f", totalFactorW);

  LINFO("cutoff (%f): kept %d, skipped %d", cutoff.getVal(), kept, skipped);

  Image<float> result = toPower(exc_img, excitExponent.getVal()) / inh_pool;

  return result;
}

// ######################################################################
Image<float> SoxChannel::combineOutputs()
{
  Dims dims = getMapDims();

  Image<float> result(dims, ZEROS);

  for (uint exc_ori = 0; exc_ori < numChans(); ++exc_ori)
    {
      for (uint exc_scl = 0; exc_scl < numScales(); ++exc_scl)
        {
          Image<float> resp = getNonlinearResponse(exc_ori, exc_scl);

          result += rescale(resp, dims);
        }
    }

  if (itsNormType.getVal() == VCXNORM_MAXNORM)
    return maxNormalize(result, MAXNORMMIN, MAXNORMMAX, VCXNORM_MAXNORM);
  else
    return maxNormalize(result, 0.0f, 0.0f, itsNormType.getVal());
}

// ######################################################################
void SoxChannel::killCaches()
{
  ComplexChannel::killCaches(); // call our base class's version

  std::vector<Cache>().swap(inhibCaches);
}

// ######################################################################
Image<float> SoxChannel::getInhib(int ori, int scl, int exc_scl,
                                  const Dims& dims,
                                  const Image<float>& linearResponse)
{
  if (int(inhibCaches.size()) <= exc_scl)
    inhibCaches.resize(exc_scl+1);

  Cache& cache = inhibCaches[exc_scl];

  const LevelSpec ls = gabor(0)->getModelParamVal<LevelSpec>("LevelSpec");
  const unsigned int nscale = ls.maxDepth();

  unsigned int index = scl * nscale + ori;

  if (cache.size() <= index)
    cache.resize(index+1);

  if (!cache[index].img.initialized())
    {
      cache[index] =
        CacheElem(rescale(toPower(linearResponse, inhibExponent.getVal()),
                          dims),
                  ori, scl, exc_scl);
    }

  const CacheElem& elem = cache[index];

  ASSERT(elem.ori == ori);
  ASSERT(elem.scl == scl);
  ASSERT(elem.exc_scl == exc_scl);

  return elem.img;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
