/*!@file Surprise/SurpriseMap.C a surprise map */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseMap.C $
// $Id: SurpriseMap.C 11562 2009-08-08 00:35:40Z dberg $
//

#include "Surprise/SurpriseMap.H"

#include "Image/Kernels.H"  // for gaussianBlob()
#include "Image/MathOps.H"
#include "Util/Assert.H"

// ######################################################################
template <class T>
SurpriseMap<T>::SurpriseMap() :
  itsModels(), itsQlen(0), itsInitialModel(),
  itsNeighSigma(0.0f), itsLocSigma(0.0f), itsNweights(), itsNWmin(0.0f),
  itsNeighUpdFac(0.7), itsProbe(-1, -1), itsSLfac(1.0), itsSSfac(0.1),
  itsJointKLBiasType(SU_KL_NONE),
  itsTakeSTMax(false)
{ }

// ######################################################################
template <class T>
void SurpriseMap<T>::init(const uint qlen, const double updatefac,
                          const double neighupdatefac,
                          const double sampleval, const double samplevar,
                          const float neighsigma, const float locsigma,
                          const Point2D<int>& probe, const double slfac,
                          const double ssfac, const SU_KL_BIAS klbias,
                          const bool takeSTMax)
{
  itsModels.clear();
  itsNweights.freeMem();
  itsInitialModel.init(updatefac, sampleval, samplevar);

  itsQlen            = qlen;
  itsNeighSigma      = neighsigma;
  itsLocSigma        = locsigma;
  itsNeighUpdFac     = neighupdatefac;
  itsProbe           = probe;
  itsSLfac           = slfac;
  itsSSfac           = ssfac;
  itsJointKLBiasType = klbias;
  itsTakeSTMax       = takeSTMax;
}

// ######################################################################
template <class T>
SurpriseMap<T>::~SurpriseMap()
{ }

// ######################################################################
template <class T>
void SurpriseMap<T>::reset()
{
  for (uint i = 0; i < itsModels.size(); i ++)
    itsModels[i].reset();
}

// ######################################################################
template <class T>
void SurpriseMap<T>::initModels(const SurpriseImage<T>& sample,
                                const bool setBias)
{
  // resize and reset our queue of models:
  SurpriseImage<T> models(sample.getDims()); // NOTE: uninitialized models

  if(setBias)
    itsInitialModel.setBias(itsSLfac,itsSSfac,itsJointKLBiasType);

  models.clear(itsInitialModel);
  models.reset();

  itsModels.clear();

  for (uint i = 0; i < itsQlen; i ++) itsModels.push_back(models);

  // compute our Difference-of-Gaussians mask of weights:
  const int   w     = sample.getWidth();
  const int   h     = sample.getHeight();
  const float sigma = itsNeighSigma * float(std::max(w, h));
  const Dims          d(w * 2 + 1, h * 2 + 1);
  const Point2D<int>  p(w, h);

  itsNweights  = gaussianBlob<float>(d, p, sigma, sigma);
  itsNweights -= gaussianBlob<float>(d, p, itsLocSigma, itsLocSigma) *
    (itsLocSigma*itsLocSigma / (sigma * sigma) * 1.5f);

  inplaceRectify(itsNweights);  // eliminate negative values

  float mi, ma;
  getMinMax(itsNweights, mi, ma);
  itsNWmin = 0.01f * ma;

  // zero low weights
  for(Image<float>::iterator w = itsNweights.beginw();
      w != itsNweights.endw(); w++)
    if(*w <= itsNWmin) *w = 0;

}

// ######################################################################
template <>
Image<double> SurpriseMap<SurpriseModelPM>::
surprise(const SurpriseImage<SurpriseModelPM>& sample)
{
  // the generic version for other models is implemented in the next function

  // is it the first time we are called? if so, we need to setup our
  // size and reset our neighborhood cache:
  if (itsModels.empty())
    initModels(sample);
  else if (itsModels[0].isSameSize(sample) == false)
    LFATAL("Inconsistent input size!");

  // each model feeds into the next one. The first (fastest) model
  // receives the sample from the image as input, and is updated. The
  // updated model then serves as input to the next slower model, and
  // so on. Total surprise is the product from all models:
  SurpriseImage<SurpriseModelPM> input(sample);
  Image<double> s;
  const bool doprobe = input.coordsOk(itsProbe);
  double locmean = 0.0, locvar = 0.0, neimean = 0.0, neivar = 0.0; // for probe

  for (uint i = 0; i < itsModels.size(); i ++)
    {
      itsModels[i].resetUpdFac(0.85F);
      // For covariant models we may need to analyze the covarance factors
      // and update the hyper parameters from the updates before we go
      // ahead and compute surprise as usual.
      itsModels[i].preComputeHyperParams(input);
      // In this instance, the model is covaried with itself.
      // Spatial and temporal concerns are computed at one time.
      itsModels[i].neighborhoods(itsModels[i], itsNweights, true);

      // show values at a probe location?
      if (doprobe)
        {
          locmean = itsModels[i].getMean().getVal(itsProbe);
          locvar  = itsModels[i].getVar().getVal(itsProbe);
        }

      // update local models and compute the local temporal surprise:
      const Image<double> sl = itsModels[i].surprise(input);


      // the total surprise is a weighted sum of local temporal
      // surprise and spatial surprise:
      Image<double> stot(sl.getDims(), ZEROS);
      if (itsSLfac)
        {
          if (itsSLfac != 1.0) stot = sl * itsSLfac; else stot = sl;
        }

      // total surprise combines multiplicatively across models with
      // different time scales:
      if (i == 0) s = stot; else s *= stot;

      // save debug output from a probe location:
      if (doprobe)
        {
          LERROR("MODELS: %d   %g %g   %g %g   %g %g", i,
                 input.getMean().getVal(itsProbe),
                 input.getVar().getVal(itsProbe),
                 locmean, locvar, neimean, neivar);
          LERROR("SURPRISE: %d %g %g", i, sl.getVal(itsProbe),
                 sl.getVal(itsProbe));
        }

      // the updated models are the input to the next iteration:
      input = itsModels[i];
    }

  // the results here should always be positive but sometimes turn
  // negative due to rounding errors in the surprise computation.
  // Let's clamp just to be sure, otherwise we'll end up with some
  // NaNs at later stages:
  inplaceRectify(s);

  // calm down total surprise and preserve units of wows:
  // We multiply times 10 to scale with other surprise better
  s = toPower(s, 1.0 / (3.0 * double(itsModels.size()))) * 5;

  // return total surprise:
  return s;

}

// ######################################################################
template <>
Image<double> SurpriseMap<SurpriseModelGG>::
surprise(const SurpriseImage<SurpriseModelGG>& sample)
{
  // is it the first time we are called? if so, we need to setup our
  // size and reset our neighborhood cache:
  if (itsModels.empty())
    initModels(sample,true);
  else if (itsModels[0].isSameSize(sample) == false)
    LFATAL("Inconsistent input size!");

  // each model feeds into the next one. The first (fastest) model
  // receives the sample from the image as input, and is updated. The
  // updated model then serves as input to the next slower model, and
  // so on. Total surprise is the product from all models:
  SurpriseImage<SurpriseModelGG> input(sample);
  Image<double> s;
  const bool doprobe = input.coordsOk(itsProbe);
  double locmean = 0.0, locvar = 0.0, neimean = 0.0, neivar = 0.0; // for probe

  for (uint i = 0; i < itsModels.size(); i ++)
    {
      // compute neighborhood models from our current (old) local models:
      itsModels[i].neighborhoods(input, itsNweights);
      //itsModels[i].neighborhoods(input, itsNweights, itsNWmin, true);
      //itsModels[i].neighborhoods(input,20);

      // show values at a probe location?
      if (doprobe)
        {
          locmean = itsModels[i].getMean().getVal(itsProbe);
          locvar  = itsModels[i].getVar().getVal(itsProbe);
        }

      // update local models and compute the local temporal surprise:
      const Image<double> sl = itsModels[i].surprise(input);

      // the total surprise is a weighted sum of local temporal
      // surprise and spatial surprise:
      Image<double> stot(sl.getDims(), ZEROS);
      if (itsSLfac)
        {
          if (itsSLfac != 1.0) stot = sl * itsSLfac; else stot = sl;
        }

      // total surprise combines multiplicatively across models with
      // different time scales:
      if (i == 0) s = stot; else s *= stot;

      // save debug output from a probe location:
      if (doprobe)
        {
          LERROR("MODELS: %d   %g %g   %g %g   %g %g", i,
                 input.getMean().getVal(itsProbe),
                 input.getVar().getVal(itsProbe),
                 locmean, locvar, neimean, neivar);
          LERROR("SURPRISE: %d %g", i, sl.getVal(itsProbe));
        }

      // the updated models are the input to the next iteration:
      input = itsModels[i];
    }

  // the results here should always be positive but sometimes turn
  // negative due to rounding errors in the surprise computation.
  // Let's clamp just to be sure, otherwise we'll end up with some
  // NaNs at later stages:
  inplaceRectify(s);

  // calm down total surprise and preserve units of wows:
  s = toPower(s, 1.0 / (3.0 * double(itsModels.size())));
  s = logSig(s,1,1);
  //double min, max;
  //getMinMax(s,min,max);
  //LINFO("S max %f min %f",min,max);

  // return total surprise:
  return s;
}

// ######################################################################
template <class T>
Image<double> SurpriseMap<T>::surprise(const SurpriseImage<T>& sample)
{
  // is it the first time we are called? if so, we need to setup our
  // size and reset our neighborhood cache:
  if (itsModels.empty())
    initModels(sample);
  else if (itsModels[0].isSameSize(sample) == false)
    LFATAL("Inconsistent input size!");

  // each model feeds into the next one. The first (fastest) model
  // receives the sample from the image as input, and is updated. The
  // updated model then serves as input to the next slower model, and
  // so on. Total surprise is the product from all models:
  SurpriseImage<T> input(sample);
  Image<double> s;
  const bool doprobe = input.coordsOk(itsProbe);
  double locmean = 0.0, locvar = 0.0, neimean = 0.0, neivar = 0.0; // for probe

  // Combine space and time using Max or should we combine them as a product?
  if(itsTakeSTMax)
    {
      Image<double> t;
      for (uint i = 0; i < itsModels.size(); i ++)
        {
          // compute neighborhood models from our current (old) local models:
          SurpriseImage<T> neigh;
          neigh.neighborhoods(input, itsNweights);
          if (itsNeighUpdFac != 0.0) // use different update fac for neighs?
            neigh.resetUpdFac(itsNeighUpdFac); // higher fac -> stronger popout

          // show values at a probe location?
          if (doprobe)
            {
              locmean = itsModels[i].getMean().getVal(itsProbe);
              locvar  = itsModels[i].getVar().getVal(itsProbe);
              neimean = neigh.getMean().getVal(itsProbe);
              neivar  = neigh.getVar().getVal(itsProbe);
            }

          // update local models and compute the local temporal surprise:
          const Image<double> sl = itsModels[i].surprise(input) * itsSLfac;
          // compute spatial surprise:
          const Image<double> ss = neigh.surprise(input)        * itsSSfac;

          // Compute product of space and time seperatly
          if (i == 0) { s  = ss; t  = sl; }
          else        { s *= ss; t *= sl; }

          // save debug output from a probe location:
          if (doprobe)
            {
              LERROR("MODELS: %d   %g %g   %g %g   %g %g", i,
                     input.getMean().getVal(itsProbe),
                     input.getVar().getVal(itsProbe),
                     locmean, locvar, neimean, neivar);
              LERROR("SURPRISE: %d %g %g", i, sl.getVal(itsProbe),
                     ss.getVal(itsProbe));
            }

          // the updated models are the input to the next iteration:
          input = itsModels[i];
        }
      // take the max of either space or temporal surprise
      s = takeMax(s,t);
    }
  else
    {
      for (uint i = 0; i < itsModels.size(); i ++)
        {
          // compute neighborhood models from our current (old) local models:
          SurpriseImage<T> neigh;
          neigh.neighborhoods(input, itsNweights);
          if (itsNeighUpdFac != 0.0) // use different update fac for neighs?
            neigh.resetUpdFac(itsNeighUpdFac); // higher fac -> stronger popout

          // show values at a probe location?
          if (doprobe)
            {
              locmean = itsModels[i].getMean().getVal(itsProbe);
              locvar  = itsModels[i].getVar().getVal(itsProbe);
              neimean = neigh.getMean().getVal(itsProbe);
              neivar  = neigh.getVar().getVal(itsProbe);
            }

          // update local models and compute the local temporal surprise:
          const Image<double> sl = itsModels[i].surprise(input);
          // compute spatial surprise:
          const Image<double> ss = neigh.surprise(input);

          // the total surprise is a weighted sum of local temporal
          // surprise and spatial surprise:
          Image<double> stot(sl.getDims(), ZEROS);
          if (itsSLfac)
            {
              if (itsSLfac != 1.0) stot = sl * itsSLfac; else stot = sl;
            }
          if (itsSSfac)
            {
              if (itsSSfac != 1.0) stot += ss * itsSSfac; else stot += ss;
            }

          if (i == 0) s  = stot;
          else        s *= stot;

          // save debug output from a probe location:
          if (doprobe)
            {
              LERROR("MODELS: %d   %g %g   %g %g   %g %g", i,
                     input.getMean().getVal(itsProbe),
                     input.getVar().getVal(itsProbe),
                     locmean, locvar, neimean, neivar);
              LERROR("SURPRISE: %d %g %g", i, sl.getVal(itsProbe),
                     ss.getVal(itsProbe));
            }

          // the updated models are the input to the next iteration:
          input = itsModels[i];
        }
    }


  // the results here should always be positive but sometimes turn
  // negative due to rounding errors in the surprise computation.
  // Let's clamp just to be sure, otherwise we'll end up with some
  // NaNs at later stages:
  inplaceRectify(s);

  // calm down total surprise and preserve units of wows:
  s = toPower(s, 1.0 / (3.0 * double(itsModels.size())));

  // return total surprise:
  return s;
}

// ######################################################################
template <class T>
const SurpriseImage<T>& SurpriseMap<T>::getSurpriseImage(const
                                                         uint index) const
{
  ASSERT(index < itsModels.size());
  return itsModels[index];
}


// ######################################################################
// explicit instantiations:
template class SurpriseMap<SurpriseModelSG>;
template class SurpriseMap<SurpriseModelSP>;
template class SurpriseMap<SurpriseModelSP1>;
template class SurpriseMap<SurpriseModelSPC>;
template class SurpriseMap<SurpriseModelSPF>;
template class SurpriseMap<SurpriseModelCS>;
template class SurpriseMap<SurpriseModelGG>;
template class SurpriseMap<SurpriseModelPM>;
template class SurpriseMap<SurpriseModelOD>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
