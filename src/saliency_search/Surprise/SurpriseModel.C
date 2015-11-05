/*!@file Surprise/SurpriseModel.C a local (single-point) model of surprise */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseModel.C $
// $Id: SurpriseModel.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Surprise/SurpriseModel.H"

#include "Util/Assert.H"
#include "Util/log.H"

// ######################################################################
// ######################################################################
// SurpriseModel implementation
// ######################################################################
// ######################################################################

SurpriseModel::SurpriseModel(const double updatefac, const double sampleval,
                             const double samplevar) :
  itsUpdateFac(updatefac), itsInitialVal(sampleval), itsInitialVar(samplevar)
{ }

// ######################################################################
SurpriseModel::~SurpriseModel()
{ }

// ######################################################################
inline void SurpriseModel::init(const double updfac,
                                const double sampleval,
                                const double samplevar)
{ itsInitialVal = sampleval; itsInitialVar = samplevar; itsUpdateFac = updfac;}

// ######################################################################
inline void SurpriseModel::resetUpdFac(const double updfac)
{ itsUpdateFac = updfac; }

// ######################################################################
inline double SurpriseModel::surprise(const SurpriseModel& other)
{ LFATAL("Unimplemented! use derived classes instead"); return 0.0; }

// ######################################################################
inline void SurpriseModel::preComputeHyperParams(const SurpriseModel& sample)
{ LFATAL("Unimplemented! use derived classes instead"); }

// ######################################################################
inline void SurpriseModel::combineFrom(const Image<SurpriseModel>& models,
                                       const Image<float>& weights)
{ LFATAL("Unimplemented! use derived classes instead"); }

// ######################################################################
inline void SurpriseModel::combineFrom(const Image<SurpriseModel>& models,
                                       const Image<float>& weights,
                                       const Point2D<int>& pos,
                                       const int width,
                                       const int height,
                                       const int offset)
{ LFATAL("Unimplemented! use derived classes instead"); }

// ######################################################################
inline void SurpriseModel::setBias(const double slfac,
                                   const double ssfac,
                                   const SU_KL_BIAS klbias)
{ LFATAL("Unimplemented! use derived classes instead"); }

// ######################################################################
#define COMBINE_FROM(models,weights)                          \
  ASSERT(models.isSameSize(weights));                         \
  COMBINE_INIT                                                \
  Image<COMBINE_TYPE>::const_iterator m = models.begin(),     \
    stop = models.end();                                      \
  Image<float>::const_iterator w = weights.begin();           \
  double wsum = 0.0;                                          \
  while(m != stop)                                            \
    {                                                         \
      const double weight = double(*w++);                     \
      COMBINE_UPDATE                                          \
      wsum += weight;                                         \
      ++m;                                                    \
    }                                                         \
  COMBINE_FINISH

// ######################################################################
// Notes:
// (A) weight has been zero'd for low value so that if(weight) will
//     skip what would have been a below threshold weight
// (B) offset gives us the formula:
//     w += width - pos.i + (2 * width + 1) * (height - pos.j)
/*
#define COMBINE_FROM_WMIN(models,weights,pos,width,height,offset) \
  COMBINE_INIT                                                \
  Image<COMBINE_TYPE>::const_iterator m = models.begin();     \
  Image<float>::const_iterator w = weights.begin();           \
  w += offset - pos.i;                                        \
  double wsum = 0.0F;                                         \
  for (int j = 0; j < height; j ++)                           \
    {                                                         \
      for (int i = 0; i < width; i ++)                        \
        {                                                     \
          const double weight = *w++;                         \
          if (weight)                                         \
            {                                                 \
              COMBINE_UPDATE                                  \
              wsum += weight;                                 \
            }                                                 \
          ++m;                                                \
        }                                                     \
      w += width + 1;                                         \
    }                                                         \
  COMBINE_FINISH
*/
#define COMBINE_FROM_WMIN(models,weights,pos,width,height,offset) \
  COMBINE_INIT                                                \
  Image<COMBINE_TYPE>::const_iterator m = models.begin();     \
  Image<float>::const_iterator w = weights.begin();           \
  w += offset - pos.i;                                        \
  double wsum = 0.0F;                                         \
  for (int j = 0; j < height; j ++)                           \
    {                                                         \
      for (int i = 0; i < width; i ++)                        \
        {                                                     \
          const double weight = *w++;                         \
          COMBINE_UPDATE                                      \
          wsum += weight;                                     \
          ++m;                                                \
        }                                                     \
      w += width + 1;                                         \
    }                                                         \
  COMBINE_FINISH

// ######################################################################
// ######################################################################
// SurpriseModelSG implementation
// ######################################################################
// ######################################################################

SurpriseModelSG::SurpriseModelSG(const double updatefac,
                                 const double sampleval,
                                 const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsN(1),
  itsMean(sampleval),
  itsVar(samplevar)
{ }

// ######################################################################
SurpriseModelSG::~SurpriseModelSG()
{ }

// ######################################################################
inline void SurpriseModelSG::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelSG::init(const double updfac,
                                  const double sampleval,
                                  const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  reset();
}

// ######################################################################
inline void SurpriseModelSG::load(const double sampleval,
                                  const double samplevar)
{ itsMean = sampleval; itsVar = samplevar; }

// ######################################################################
inline double SurpriseModelSG::surprise(const SurpriseModelSG& sample)
{
  const double mean1 = itsMean, var1 = itsVar / itsUpdateFac;
  const double mean2 = sample.itsMean, var2 = sample.itsVar;

  const double ivar1 = var1 < 1.0e-10 ? 1.0e10 : 1.0 / var1;
  const double ivar2 = var2 < 1.0e-10 ? itsN * 1.0e10 : itsN / var2;

  // let's find out the mean and variance of our model once we update
  // it with the incoming sample (i.e., let's compute the posterior).
  // We use the unknown mean / known variance case:
  const double newMean = (mean1 * ivar1 + mean2 * ivar2) / (ivar1 + ivar2);
  const double newVar = 1.0 / (ivar1 + ivar2);

  // surprise is KL(new || old):
  // KL(Gq || Gp) = 1/2 [Sq^2/Sp^2 - 1 - log(Sq^2/Sp^2) + (Mq-Mp)^2/Sp^2]
  // here, new = Gq = G(newMean, newVar) and old = Gp = G(mean1, var1)
  const double x = newVar / var1, mm = newMean - mean1;
  const double s = 0.5 * (x - 1.0 - log(x) + mm * mm /var1);

  // the posterior becomes our new prior:
  itsMean = newMean; itsVar = newVar;

  // return the surprise, in units of wows:
  return s / M_LN2;
}

// ######################################################################
inline void SurpriseModelSG::preComputeHyperParams(const SurpriseModelSG& sample)
{ LFATAL("Unimplemented in this model!"); }

// ######################################################################

#define COMBINE_INIT itsMean = 0.0; itsVar = 0.0;
#define COMBINE_TYPE SurpriseModelSG
#define COMBINE_UPDATE                                \
  const double mmean = m->itsMean;                    \
  itsMean += weight * mmean;                          \
  itsVar += weight * (m->itsVar + mmean * mmean);
#define COMBINE_FINISH                                \
  itsMean /= wsum; itsVar /= wsum;                    \
  itsVar -= itsMean * itsMean;

// ######################################################################
inline void SurpriseModelSG::combineFrom(const Image<SurpriseModelSG>& models,
                                         const Image<float>& weights)
{
  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:
  COMBINE_FROM(models,weights)
}

// ######################################################################
inline void SurpriseModelSG::combineFrom(const Image<SurpriseModelSG>& models,
                                         const Image<float>& weights,
                                         const Point2D<int>& pos,
                                         const int width,
                                         const int height,
                                         const int offset)
{
  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:
  COMBINE_FROM_WMIN(models,weights,pos,width,height,offset)
}

// ######################################################################

#undef COMBINE_INIT
#undef COMBINE_TYPE
#undef COMBINE_UPDATE
#undef COMBINE_FINISH

// ######################################################################
inline double SurpriseModelSG::getMean() const
{ return itsMean; }

// ######################################################################
inline double SurpriseModelSG::getVar() const
{ return itsVar; }

// ######################################################################
inline double SurpriseModelSG::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
// ######################################################################
// SurpriseModelSP implementation
// ######################################################################
// ######################################################################

SurpriseModelSP::SurpriseModelSP(const double updatefac,
                                 const double sampleval,
                                 const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsN(1),
  itsAlpha(itsN * sampleval / (1.0 - updatefac)),
  itsBeta(itsN / (1.0 - updatefac))
{ }

// ######################################################################
SurpriseModelSP::~SurpriseModelSP()
{ }

// ######################################################################
inline void SurpriseModelSP::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelSP::init(const double updfac,
                                  const double sampleval,
                                  const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  load(sampleval, samplevar);
}

// ######################################################################
inline void SurpriseModelSP::load(const double sampleval,
                                  const double samplevar)
{
  itsAlpha = itsN * sampleval / (1.0 - itsUpdateFac);
  itsBeta  = itsN / (1.0 - itsUpdateFac);
}

// ######################################################################
inline double SurpriseModelSP::surprise(const SurpriseModelSP& sample)
{
  // first, decay alpha and beta:
  itsAlpha *= itsUpdateFac; itsBeta *= itsUpdateFac;

  // a zero response can trip us up since phi(x) >= 0.0
  // Added by Nate to avoid certain crashes
  if (itsAlpha <= 0.0) itsAlpha = 0.0000001;

  // let's find out the alpha and beta of our model once we update
  // it with the incoming sample (i.e., let's compute the posterior):
  const double newAlpha = itsAlpha + itsN * sample.itsAlpha / sample.itsBeta;
  const double newBeta  = itsBeta  + itsN;

  // surprise is KL(new || old):
  const double s = KLgamma<double>(newAlpha,newBeta,itsAlpha,itsBeta,true);

  // the posterior becomes our new prior:
  itsAlpha = newAlpha; itsBeta = newBeta;

  return s;
}

// ######################################################################
inline void SurpriseModelSP::preComputeHyperParams(const SurpriseModelSP& sample)
{ LFATAL("Unimplemented in this model!"); }

// ######################################################################
#define COMBINE_INIT itsAlpha = 0.0; itsBeta = 0.0;
#define COMBINE_TYPE SurpriseModelSP
// Note alpha has been preset in its value so what we have here is
// functionally: itsAlpha += weight * m->itsAlpha/m->itsBeta
// See preSetAlpha() which does this short cut step
#define COMBINE_UPDATE                            \
  itsAlpha += weight * m->itsAlpha;               \
  itsBeta  += weight * m->itsBeta;
#define COMBINE_FINISH                            \
  wsum      = 1/wsum;                             \
  itsBeta  *= wsum;                               \
  itsAlpha *= itsBeta * wsum;

// ######################################################################
inline void SurpriseModelSP::combineFrom(const Image<SurpriseModelSP>& models,
                                         const Image<float>& weights)
{
  // combined alpha is the weighted sum of means, and combined beta is
  // the sum of weights (both possibly scaled by a factor):

  COMBINE_FROM(models,weights)
}

// ######################################################################
inline void SurpriseModelSP::combineFrom(const Image<SurpriseModelSP>& models,
                                         const Image<float>& weights,
                                         const Point2D<int>& pos,
                                         const int width,
                                         const int height,
                                         const int offset)
{
  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:

  COMBINE_FROM_WMIN(models,weights,pos,width,height,offset)
}

// ######################################################################

#undef COMBINE_INIT
#undef COMBINE_TYPE
#undef COMBINE_UPDATE
#undef COMBINE_FINISH

// ######################################################################
inline double SurpriseModelSP::getMean() const
{ return itsAlpha / itsBeta; }

// ######################################################################
inline double SurpriseModelSP::getVar() const
{ return itsAlpha / (itsBeta * itsBeta); }

// ######################################################################
inline double SurpriseModelSP::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
inline double SurpriseModelSP::getAlpha() const
{ return itsAlpha; }

// ######################################################################
inline double SurpriseModelSP::getBeta() const
{ return itsBeta; }

// ######################################################################
void SurpriseModelSP::preSetAlpha()
{
  itsAlpha = itsAlpha/itsBeta;
}

// ######################################################################
// ######################################################################
// SurpriseModelSP1 implementation
// ######################################################################
// ######################################################################

SurpriseModelSP1::SurpriseModelSP1(const double updatefac,
                                 const double sampleval,
                                 const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsN(1),
  itsAlpha(itsN * sampleval / (1.0 - updatefac)),
  itsBeta(itsN / (1.0 - updatefac))
{ }

// ######################################################################
SurpriseModelSP1::~SurpriseModelSP1()
{ }

// ######################################################################
inline void SurpriseModelSP1::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelSP1::init(const double updfac,
                                  const double sampleval,
                                  const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  load(sampleval, samplevar);
}

// ######################################################################
inline void SurpriseModelSP1::load(const double sampleval,
                                  const double samplevar)
{
  itsAlpha = itsN * sampleval / (1.0 - itsUpdateFac);
  itsBeta  = itsN / (1.0 - itsUpdateFac);
}

// ######################################################################
inline double SurpriseModelSP1::surprise(const SurpriseModelSP1& sample)
{
  // a zero response can trip us up since phi(x) >= 0.0
  // Added by Nate to avoid certain crashes
  if (itsAlpha <= 0.0) itsAlpha = 0.0000001;

  // let's find out the alpha and beta of our model once we update
  // it with the incoming sample (i.e., let's compute the posterior):
  const double newAlpha = itsAlpha * itsUpdateFac +
    itsN * sample.itsAlpha / sample.itsBeta;
  const double newBeta  = itsBeta * itsUpdateFac  +
    itsN;

  // surprise is KL(new || old):
  const double s = KLgamma<double>(newAlpha,newBeta,itsAlpha,itsBeta,true);

  // the posterior becomes our new prior:
  itsAlpha = newAlpha; itsBeta = newBeta;

  return s;
}

// ######################################################################
inline void SurpriseModelSP1::preComputeHyperParams(const SurpriseModelSP1& sample)
{ LFATAL("Unimplemented in this model!"); }

// ######################################################################
#define COMBINE_INIT itsAlpha = 0.0; itsBeta = 0.0;
#define COMBINE_TYPE SurpriseModelSP1
// Note alpha has been preset in its value so what we have here is
// functionally: itsAlpha += weight * m->itsAlpha/m->itsBeta
// See preSetAlpha() which does this short cut step
#define COMBINE_UPDATE                            \
  itsAlpha += weight * m->itsAlpha;               \
  itsBeta  += weight * m->itsBeta;
#define COMBINE_FINISH                            \
  wsum      = 1/wsum;                             \
  itsBeta  *= wsum;                               \
  itsAlpha *= itsBeta * wsum;

// ######################################################################
inline void SurpriseModelSP1::combineFrom(const Image<SurpriseModelSP1>& models,
                                         const Image<float>& weights)
{
  // combined alpha is the weighted sum of means, and combined beta is
  // the sum of weights (both possibly scaled by a factor):

  COMBINE_FROM(models,weights)
}

// ######################################################################
inline void SurpriseModelSP1::combineFrom(const Image<SurpriseModelSP1>& models,
                                         const Image<float>& weights,
                                         const Point2D<int>& pos,
                                         const int width,
                                         const int height,
                                         const int offset)
{
  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:

  COMBINE_FROM_WMIN(models,weights,pos,width,height,offset)
}

// ######################################################################

#undef COMBINE_INIT
#undef COMBINE_TYPE
#undef COMBINE_UPDATE
#undef COMBINE_FINISH

// ######################################################################
inline double SurpriseModelSP1::getMean() const
{ return itsAlpha / itsBeta; }

// ######################################################################
inline double SurpriseModelSP1::getVar() const
{ return itsAlpha / (itsBeta * itsBeta); }

// ######################################################################
inline double SurpriseModelSP1::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
inline double SurpriseModelSP1::getAlpha() const
{ return itsAlpha; }

// ######################################################################
inline double SurpriseModelSP1::getBeta() const
{ return itsBeta; }

// ######################################################################
void SurpriseModelSP1::preSetAlpha()
{
  itsAlpha = itsAlpha/itsBeta;
}

// ######################################################################
// ######################################################################
// SurpriseModelSPC implementation
// ######################################################################
// ######################################################################

SurpriseModelSPC::SurpriseModelSPC(const double updatefac,
                                   const double sampleval,
                                   const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsN(1),
  itsAlpha(itsN * sampleval / (1.0 - updatefac)),
  itsBeta(M_PI)
{ }

// ######################################################################
SurpriseModelSPC::~SurpriseModelSPC()
{ }

// ######################################################################
inline void SurpriseModelSPC::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelSPC::init(const double updfac,
                                   const double sampleval,
                                   const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  load(sampleval, samplevar);
}

// ######################################################################
inline void SurpriseModelSPC::load(const double sampleval,
                                   const double samplevar)
{
  itsAlpha = itsN * sampleval / (1.0 - itsUpdateFac);
  itsBeta  = M_PI;
}

// ######################################################################
inline double SurpriseModelSPC::surprise(const SurpriseModelSPC& sample)
{
  // first, decay alpha:
  itsAlpha *= itsUpdateFac;

  // a zero response can trip us up since phi(x) >= 0.0
  // Added by Nate to avoid certain crashes
  if (itsAlpha <= 0.0) itsAlpha = 0.0000001;

  // let's find out the alpha and beta of our model once we update
  // it with the incoming sample (i.e., let's compute the posterior):
  const double newAlpha = itsAlpha + itsN * sample.itsAlpha / sample.itsBeta;

  // surprise is KL(new || old):
  static const double itsC1 = D_SQRT_2/2.0F - 1.0F;
  static const double itsC2 = D_LOG_SQRT_2;

  const double s = KLgammaConst<double>(newAlpha,itsAlpha,itsC1,itsC2,true);

  // the posterior becomes our new prior:
  itsAlpha = newAlpha;

  return s;
}

// ######################################################################
inline void SurpriseModelSPC::preComputeHyperParams(const SurpriseModelSPC& sample)
{ LFATAL("Unimplemented in this model!"); }

// ######################################################################
#define COMBINE_INIT itsAlpha = 0.0; itsBeta = 0.0;
#define COMBINE_TYPE SurpriseModelSPC
// Note alpha has been preset in its value so what we have here is
// functionally: itsAlpha += weight * m->itsAlpha/m->itsBeta
// See preSetAlpha() which does this short cut step
#define COMBINE_UPDATE                            \
  itsAlpha += weight * m->itsAlpha;               \
  itsBeta  += weight * m->itsBeta;
#define COMBINE_FINISH                            \
  wsum      = 1/wsum;                             \
  itsBeta  *= wsum;                               \
  itsAlpha *= itsBeta * wsum;

// ######################################################################
inline void SurpriseModelSPC::combineFrom(const Image<SurpriseModelSPC>& models,
                                          const Image<float>& weights)
{
  // combined alpha is the weighted sum of means, and combined beta is
  // the sum of weights (both possibly scaled by a factor):

  COMBINE_FROM(models,weights)
}

// ######################################################################
inline void SurpriseModelSPC::combineFrom(const Image<SurpriseModelSPC>& models,
                                          const Image<float>& weights,
                                          const Point2D<int>& pos,
                                          const int width,
                                          const int height,
                                          const int offset)
{
  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:

  COMBINE_FROM_WMIN(models,weights,pos,width,height,offset)
}

// ######################################################################

#undef COMBINE_INIT
#undef COMBINE_TYPE
#undef COMBINE_UPDATE
#undef COMBINE_FINISH

// ######################################################################
inline double SurpriseModelSPC::getMean() const
{ return itsAlpha / itsBeta; }

// ######################################################################
inline double SurpriseModelSPC::getVar() const
{ return itsAlpha / (itsBeta * itsBeta); }

// ######################################################################
inline double SurpriseModelSPC::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
inline double SurpriseModelSPC::getAlpha() const
{ return itsAlpha; }

// ######################################################################
inline double SurpriseModelSPC::getBeta() const
{ return itsBeta; }

// ######################################################################
void SurpriseModelSPC::preSetAlpha()
{
  itsAlpha = itsAlpha/itsBeta;
}
// ######################################################################
// ######################################################################
// SurpriseModelSPF implementation
// ######################################################################
// ######################################################################

SurpriseModelSPF::SurpriseModelSPF(const double updatefac,
                                   const double sampleval,
                                   const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsN(1),
  itsAlpha(itsN * sampleval / (1.0 - updatefac)),
  itsBeta(itsN / (1.0 - updatefac))
{ }

// ######################################################################
SurpriseModelSPF::~SurpriseModelSPF()
{ }

// ######################################################################
inline void SurpriseModelSPF::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelSPF::init(const double updfac,
                                   const double sampleval,
                                   const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  load(sampleval, samplevar);
}

// ######################################################################
inline void SurpriseModelSPF::load(const double sampleval,
                                   const double samplevar)
{
  itsAlpha = itsN * sampleval / (1.0 - itsUpdateFac);
  itsBeta  = itsN / (1.0 - itsUpdateFac);
}

// ######################################################################
inline double SurpriseModelSPF::surprise(const SurpriseModelSPF& sample)
{
  // first, decay alpha and beta:
  itsAlpha *= itsUpdateFac; itsBeta *= itsUpdateFac;

  // a zero response can trip us up since phi(x) >= 0.0
  // Added by Nate to avoid certain crashes
  if (itsAlpha <= 0.0) itsAlpha = 0.0000001;

  // let's find out the alpha and beta of our model once we update
  // it with the incoming sample (i.e., let's compute the posterior):
  const double newAlpha = itsAlpha + itsN * sample.itsAlpha / sample.itsBeta;
  const double newBeta  = itsN * sample.itsAlpha / itsAlpha;

  // surprise is KL(new || old):
  const double s = KLgamma<double>(newAlpha,newBeta,itsAlpha,itsBeta,true);

  // the posterior becomes our new prior:
  itsAlpha = newAlpha; itsBeta = newBeta;

  return s;
}

// ######################################################################
inline void SurpriseModelSPF::preComputeHyperParams(const SurpriseModelSPF& sample)
{ LFATAL("Unimplemented in this model!"); }

// ######################################################################
#define COMBINE_INIT itsAlpha = 0.0; itsBeta = 0.0;
#define COMBINE_TYPE SurpriseModelSPF
// Note alpha has been preset in its value so what we have here is
// functionally: itsAlpha += weight * m->itsAlpha/m->itsBeta
// See preSetAlpha() which does this short cut step
#define COMBINE_UPDATE                            \
  itsAlpha += weight * m->itsAlpha;               \
  itsBeta  += weight * m->itsBeta;
#define COMBINE_FINISH                            \
  wsum      = 1/wsum;                             \
  itsBeta  *= wsum;                               \
  itsAlpha *= itsBeta * wsum;

// ######################################################################
inline void SurpriseModelSPF::combineFrom(const Image<SurpriseModelSPF>& models,
                                          const Image<float>& weights)
{
  // combined alpha is the weighted sum of means, and combined beta is
  // the sum of weights (both possibly scaled by a factor):

  COMBINE_FROM(models,weights)
}

// ######################################################################
inline void SurpriseModelSPF::combineFrom(const Image<SurpriseModelSPF>& models,
                                          const Image<float>& weights,
                                          const Point2D<int>& pos,
                                          const int width,
                                          const int height,
                                          const int offset)
{
  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:

  COMBINE_FROM_WMIN(models,weights,pos,width,height,offset)
}

// ######################################################################

#undef COMBINE_INIT
#undef COMBINE_TYPE
#undef COMBINE_UPDATE
#undef COMBINE_FINISH

// ######################################################################
inline double SurpriseModelSPF::getMean() const
{ return itsAlpha / itsBeta; }

// ######################################################################
inline double SurpriseModelSPF::getVar() const
{ return itsAlpha / (itsBeta * itsBeta); }

// ######################################################################
inline double SurpriseModelSPF::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
inline double SurpriseModelSPF::getAlpha() const
{ return itsAlpha; }

// ######################################################################
inline double SurpriseModelSPF::getBeta() const
{ return itsBeta; }

// ######################################################################
void SurpriseModelSPF::preSetAlpha()
{
  itsAlpha = itsAlpha/itsBeta;
}

// ######################################################################
// ######################################################################
// SurpriseModelCS implementation
// ######################################################################
// ######################################################################

SurpriseModelCS::SurpriseModelCS(const double updatefac,
                                 const double sampleval,
                                 const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsN(1),
  itsAlpha(itsN * sampleval / (1.0 - updatefac)),
  itsBeta(0.5)
{ }

// ######################################################################
SurpriseModelCS::~SurpriseModelCS()
{ }

// ######################################################################
inline void SurpriseModelCS::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelCS::init(const double updfac,
                                  const double sampleval,
                                  const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  load(sampleval, samplevar);
}

// ######################################################################
inline void SurpriseModelCS::load(const double sampleval,
                                  const double samplevar)
{
  itsAlpha = itsN * sampleval / (1.0 - itsUpdateFac);
  itsBeta  = 0.5;
}

// ######################################################################
inline double SurpriseModelCS::surprise(const SurpriseModelCS& sample)
{
  // first, decay alpha and beta:
  itsAlpha *= itsUpdateFac;

  // a zero response can trip us up since phi(x) >= 0.0
  // Added by Nate to avoid certain crashes
  if (itsAlpha <= 0.0) itsAlpha = 0.0000001;

  // let's find out the alpha and beta of our model once we update
  // it with the incoming sample (i.e., let's compute the posterior):
  const double newAlpha = itsAlpha + itsN * sample.itsAlpha / sample.itsBeta;

  // surprise is KL(new || old):
  const double s = KLgamma<double>(newAlpha,itsAlpha,true);

  // the posterior becomes our new prior:
  itsAlpha = newAlpha;

  return s;
}

// ######################################################################
inline void SurpriseModelCS::preComputeHyperParams(const SurpriseModelCS& sample)
{ LFATAL("Unimplemented in this model!"); }

// ######################################################################
#define COMBINE_INIT itsAlpha = 0.0; itsBeta = 0.0;
#define COMBINE_TYPE SurpriseModelCS
// Note alpha has been preset in its value so what we have here is
// functionally: itsAlpha += weight * m->itsAlpha/m->itsBeta
// See preSetAlpha() which does this short cut step
#define COMBINE_UPDATE                            \
  itsAlpha += weight * m->itsAlpha/m->itsBeta;
#define COMBINE_FINISH                            \
  wsum      = 1/wsum;                             \
  itsAlpha *= itsBeta * wsum;

// ######################################################################
inline void SurpriseModelCS::combineFrom(const Image<SurpriseModelCS>& models,
                                         const Image<float>& weights)
{
  // combined alpha is the weighted sum of means, and combined beta is
  // the sum of weights (both possibly scaled by a factor):

  COMBINE_FROM(models,weights)
}

// ######################################################################
inline void SurpriseModelCS::combineFrom(const Image<SurpriseModelCS>& models,
                                         const Image<float>& weights,
                                         const Point2D<int>& pos,
                                         const int width,
                                         const int height,
                                         const int offset)
{
  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:

  COMBINE_FROM_WMIN(models,weights,pos,width,height,offset)
}

// ######################################################################

#undef COMBINE_INIT
#undef COMBINE_TYPE
#undef COMBINE_UPDATE
#undef COMBINE_FINISH

// ######################################################################
inline double SurpriseModelCS::getMean() const
{ return itsAlpha / itsBeta; }

// ######################################################################
inline double SurpriseModelCS::getVar() const
{ return itsAlpha / (itsBeta * itsBeta); }

// ######################################################################
inline double SurpriseModelCS::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
inline double SurpriseModelCS::getAlpha() const
{ return itsAlpha; }

// ######################################################################
inline double SurpriseModelCS::getBeta() const
{ return itsBeta; }

// ######################################################################
void SurpriseModelCS::preSetAlpha()
{
  itsAlpha = itsAlpha/itsBeta;
}

// ######################################################################
// ######################################################################
// SurpriseModelGG implementation
// ######################################################################
// ######################################################################

SurpriseModelGG::SurpriseModelGG(const double updatefac,
                                 const double sampleval,
                                 const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsCombineFromRun(false),
  itsN(1),
  itsJointKLBiasType(SU_KL_STATIC),
  itsAlpha(itsN * sampleval / (1.0 - updatefac)),
  itsBeta(itsN / (1.0 - updatefac)),
  itsSLfac(1.0f),
  itsSSfac(1.0f)

{ }

// ######################################################################
SurpriseModelGG::~SurpriseModelGG()
{ }

// ######################################################################
inline void SurpriseModelGG::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelGG::init(const double updfac,
                                  const double sampleval,
                                  const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  load(sampleval, samplevar);
}

// ######################################################################
inline void SurpriseModelGG::load(const double sampleval,
                                  const double samplevar)
{
  itsAlpha = itsN * sampleval / (1.0 - itsUpdateFac);
  itsBeta  = itsN / (1.0 - itsUpdateFac);
}

// ######################################################################
inline double SurpriseModelGG::surprise(const SurpriseModelGG& sample)
{
  ASSERT(itsCombineFromRun);
  // first, decay alpha and beta:
  itsAlpha *= itsUpdateFac; itsBeta *= itsUpdateFac;

  // a zero response can trip us up since phi(x) >= 0.0
  // Added by Nate to avoid certain crashes
  if (itsAlpha <= 0.0) itsAlpha = 0.0000001;

  // let's find out the alpha and beta of our model once we update
  // it with the incoming sample (i.e., let's compute the posterior):
  // NOTICE that we compute alpha and beta the same as with the SP model
  // for temporal surprise.
  const double newAlpha = itsAlpha + itsN * sample.itsAlpha / sample.itsBeta;
  const double newBeta  = itsBeta + itsN;

  // New Expected value of the mean, we add the new sample in to create
  // the new gauss distribution over space. Notice that the values
  // will go away next epoch like with space in the SP model.
  const double newExp   = sample.itsAlpha  / sample.itsBeta;
  const double newMean  = (itsSum + newExp)/(itsWeightSum + 1);

  // Compute new space variance
  const double newSS    = (newExp*newExp + itsSS)/(itsWeightSum + 1);
  const double newVar   = newSS - newMean * newMean;

  // compute sigma from variance
  itsSig = sqrt(itsVar);
  const double newSig = sqrt(newVar);

  double s = 0.0000001;

  // we hack sigma to prevent nan in a uniform image.
  if(itsSig > 0.0 && newSig > 0.0)
    {
      // surprise is KL(new || old):
      // use the joint gamma / gaussian surprise
      // Computed in util/MathFunction.H
      if(itsJointKLBiasType == SU_KL_NONE)
        s = KLjointGammaGauss<double>(newAlpha, newMean,
                                      newBeta,  newSig,
                                      itsAlpha, itsMean,
                                      itsBeta,  itsSig,
                                      true);
      else if(itsJointKLBiasType == SU_KL_STATIC)
        s = KLjointGammaGauss<double>(newAlpha, newMean,
                                      newBeta,  newSig,
                                      itsAlpha, itsMean,
                                      itsBeta,  itsSig,
                                      true,
                                      itsSLfac,
                                      itsSSfac);
      else
        LERROR("Invalid bias type %d given for KL bias",itsJointKLBiasType);
    }


  //LINFO("KL S %f NA %f NM %f NB %f NS %f OA %f OM %f OB %f OS %f",s,newAlpha, newMean,
  //    newBeta,  newSig,
  //                                         itsAlpha, itsMean,
  //    itsBeta,  itsSig);
  // the posterior becomes our new prior:
  itsAlpha = newAlpha; itsBeta = newBeta;

  // force to recompute neighborhood in the next iteration.
  itsCombineFromRun = false;

  return s;
}

// ######################################################################
inline void SurpriseModelGG::preComputeHyperParams(const SurpriseModelGG& sample)
{ LFATAL("Unimplemented in this model!"); }

// ######################################################################
#define COMBINE_INIT itsMean = 0.0; itsVar = 0.0;
#define COMBINE_TYPE SurpriseModelGG

// The expected value is alpha/beta which we use as the value for computing
// the space surprise in a gaussian manner.

#define COMBINE_UPDATE                            \
  const double expVal = m->itsAlpha / m->itsBeta; \
  itsMean += weight * expVal;                     \
  itsVar  += weight * expVal * expVal;
#define COMBINE_FINISH                            \
  itsSum       = itsMean;                         \
  itsSS        = itsVar;                          \
  itsWeightSum = wsum;                            \
  itsMean /= wsum; itsVar /= wsum;                \
  itsVar -= itsMean * itsMean;                    \
  itsCombineFromRun = true;

// ######################################################################
inline void SurpriseModelGG::combineFrom(const Image<SurpriseModelGG>& models,
                                         const Image<float>& weights)
{
  // combined alpha is the weighted sum of means, and combined beta is
  // the sum of weights (both possibly scaled by a factor):

  COMBINE_FROM(models,weights)
}

// ######################################################################
inline void SurpriseModelGG::combineFrom(const Image<SurpriseModelGG>& models,
                                         const Image<float>& weights,
                                         const Point2D<int>& pos,
                                         const int width,
                                         const int height,
                                         const int offset)
{
  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:

  COMBINE_FROM_WMIN(models,weights,pos,width,height,offset)
}

// ######################################################################

#undef COMBINE_INIT
#undef COMBINE_TYPE
#undef COMBINE_UPDATE
#undef COMBINE_FINISH

// ######################################################################
inline double SurpriseModelGG::getMean() const
{ return itsAlpha / itsBeta; }

// ######################################################################
inline double SurpriseModelGG::getVar() const
{ return itsAlpha / (itsBeta * itsBeta); }

// ######################################################################
inline double SurpriseModelGG::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
inline double SurpriseModelGG::getAlpha() const
{ return itsAlpha; }

// ######################################################################
inline double SurpriseModelGG::getBeta() const
{ return itsBeta; }

// ######################################################################
void SurpriseModelGG::setBias(const double     slfac,
                              const double     ssfac,
                              const SU_KL_BIAS klbias)
{
  itsSLfac           = slfac;
  itsSSfac           = ssfac;
  itsJointKLBiasType = klbias;
}

// ######################################################################
// SurpriseModelPM implementation
// ######################################################################
SurpriseModelPM::SurpriseModelPM(const double updatefac,
                                 const double sampleval,
                                 const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsN(1),
  itsSample(itsN * sampleval / (1.0 - updatefac)),
  itsBeta0(1)
  //itsBeta0(itsN / (1.0 - updatefac))
{
  itsAlpha0       = itsSample;
  itsAlpha1       = itsSample;
  itsAlpha2       = itsSample;
  itsBeta1        = itsBeta0;
  itsBeta2        = itsBeta0;
  itsInitBeta     = itsBeta0;
  itsExpectAlpha1 = 1;
  itsExpectAlpha2 = 1;
  itsXBar1        = 0;
  itsXBar2        = 0;
}

// ######################################################################
SurpriseModelPM::~SurpriseModelPM()
{ }

// ######################################################################
inline void SurpriseModelPM::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelPM::init(const double updfac,
                                  const double sampleval,
                                  const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  load(sampleval, samplevar);
}

// ######################################################################
inline void SurpriseModelPM::load(const double sampleval,
                                  const double samplevar)
{
  itsSample = sampleval;
}

// ######################################################################
inline double SurpriseModelPM::surprise(const SurpriseModelPM& sample)
{
  // a zero response can trip us up since phi(x) >= 0.0
  // Added by Nate to avoid certain crashes
  if (itsAlpha1 <= 0.0) itsAlpha1 = 0.0000001;
  if (itsAlpha2 <= 0.0) itsAlpha2 = 0.0000001;

  // surprise is KL(new || old):
  return KLgamma<double>(itsAlpha2,itsBeta2,itsAlpha1,itsBeta1,true);
}

// ######################################################################
inline void SurpriseModelPM::preComputeHyperParams(const SurpriseModelPM& sample)
{
  // the posterior becomes our new prior:
  // Here we store values from current time at 2 to two iterations back at
  // time 0 so that we can compute a covaraince between terms over time.

  itsAlpha0       = itsAlpha1;
  itsAlpha1       = itsAlpha2;

  const double alphaUpdate0 = itsAlpha0 * itsUpdateFac;
  const double alphaUpdate1 = itsAlpha1 * itsUpdateFac;

  const double data         = sample.itsSample / itsBeta2;

  //itsAlpha2       = alphaUpdate1 + data;
  //if(alphaUpdate1 > 0)
  //  {
  //    itsAlpha2       = alphaUpdate1 +
  //      (log(itsBeta2) - psi(alphaUpdate1) + log(sample.itsSample)) /
  //     itsBeta1;
  //  }
  //else
  //  {
      itsAlpha2       = alphaUpdate1 + data;
      //  }

  itsLgamma1      = itsLgamma2;
  itsLgamma2      = lgamma(itsAlpha2);

  itsBeta1        = itsBeta2;

  itsExpectAlpha1 = alphaUpdate0 + itsXBar1;
  itsExpectAlpha2 = alphaUpdate1 + itsXBar2;

  itsXBar2        = (sample.itsSample + itsXBar1*itsUpdateFac) /
                    (1 + itsUpdateFac);
  itsXBar1        = itsXBar2;
}

// ######################################################################
inline void SurpriseModelPM::combineFrom(const Image<SurpriseModelPM>& models,
                                         const Image<float>& weights)
{
  LFATAL("This method not supported for this type of object");
}

// ######################################################################
inline void SurpriseModelPM::combineFrom(const Image<SurpriseModelPM>& models,
                                         const Image<float>& weights,
                                         const Point2D<int>& pos,
                                         const int width,
                                         const int height,
                                         const int offset)
{
  ASSERT(weights.getWidth()  == 2 * width  + 1);
  ASSERT(weights.getHeight() == 2 * height + 1);
  ASSERT(models.coordsOk(pos));

  // combined mean is just the weighted sum of all the means; the
  // formula for combined variance is barely more complicated:

  double wsum = 0.0F; // find out how much total weight we use
  double cov  = 0.0F; // the covariance term
  Image<SurpriseModelPM>::const_iterator m = models.begin();
  Image<float>::const_iterator           w = weights.begin();

  // define bounds of our scan in the weight mask and go for it:
  w += offset - pos.i;
  if((itsExpectAlpha2 != 0) && (itsExpectAlpha1 != 0))
  {
    for (ushort j = 0; j < height; j++)
    {
      for (ushort i = 0; i < width; i++)
      {
        const double weight = *w++;
        if (weight)
        {
          // We use floats since precision in this part is less important
          // and we may be able to fit all the terms into registers at
          // the same time which will speed this up a whole lot.
          const double aRatio2 = m->itsExpectAlpha2 / itsExpectAlpha2;
          const double aRatio1 = m->itsExpectAlpha1 / itsExpectAlpha1;
          const double ratDiff = aRatio2 - aRatio1;
          cov  += (weight * ratDiff) * itsAlpha1;
          wsum += weight;
        }
        ++m;
      }
      // skip to next line:
      w += width + 1;
    }
  }
  else
  {
    cov = itsAlpha2; wsum = 1;
  }
  const double combinedAlpha2 = cov            / wsum;
  const double ratio          = combinedAlpha2 / itsAlpha2;
  // scale alpha and beta so that if all local models were the same we
  // get the same parameters for our neighborhood model:

  //LINFO("ALPHA1 %f ALPHA2 %f",itsAlpha1,itsAlpha2);

  if((itsAlpha2 != 0) && (ratio != 0))
  {
    //LINFO("ALPHA1 %f ALPHA2 %f ratio %f itsBeta1 %f UDF %f",
    //      itsAlpha1,itsAlpha2,ratio,itsBeta1,itsUpdateFac);
    itsBeta2 = itsInitBeta * ((ratio * itsBeta1 * itsUpdateFac) + 1);
    //itsBeta2 = ratio + (itsBeta1 * itsUpdateFac + 1);

    //itsBeta2 = (ratio * (1/itsBeta1))*((itsBeta1 * itsUpdateFac) + 1);
    //itsBeta2 =  ratio + 1 + (itsBeta1 * itsUpdateFac);
    //itsBeta2 = itsInitBeta * ((ratio * itsBeta1) + 1);
  }
  else
    itsBeta2 = itsBeta1 * itsUpdateFac + 1;

}

// ######################################################################
inline double SurpriseModelPM::getMean() const
{ return itsAlpha2 / itsBeta2; }

// ######################################################################
inline double SurpriseModelPM::getVar() const
{ return itsAlpha2 / (itsBeta2 * itsBeta2); }

// ######################################################################
inline double SurpriseModelPM::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
// SurpriseModelOD implementation
// ######################################################################
SurpriseModelOD::SurpriseModelOD(const double updatefac,
                                 const double sampleval,
                                 const double samplevar) :
  SurpriseModel(updatefac, sampleval, samplevar),
  itsN(1),
  itsLambda(sampleval)
{ }

// ######################################################################
SurpriseModelOD::~SurpriseModelOD()
{ }

// ######################################################################
inline void SurpriseModelOD::reset()
{ load(itsInitialVal, itsInitialVar); }

// ######################################################################
inline void SurpriseModelOD::init(const double updfac,
                                  const double sampleval,
                                  const double samplevar)
{
  SurpriseModel::init(updfac, sampleval, samplevar);
  load(sampleval, samplevar);
}

// ######################################################################
inline void SurpriseModelOD::load(const double sampleval,
                                  const double samplevar)
{
  itsLambda = sampleval;
}

// ######################################################################
inline double SurpriseModelOD::surprise(const SurpriseModelOD& sample)
{
  // let's compute how likely the data is given our current model:
  double ret = poisson(int(sample.itsLambda+0.4999), itsLambda);

  // now update our lambda:
  itsLambda = itsLambda + itsUpdateFac * (sample.itsLambda - itsLambda);

  // return the "surprise":
  ret = 1.0 / (ret + 1.0e-5) - 1.0;
  if (ret < 0.0) ret = 0.0;

  return ret * 0.00001;
}

// ######################################################################
inline void SurpriseModelOD::preComputeHyperParams(const SurpriseModelOD& sample)
{ LFATAL("Unimplemented in this model!"); }

// ######################################################################
inline void SurpriseModelOD::combineFrom(const Image<SurpriseModelOD>&
                                         models, const Image<float>& weights)
{
  ASSERT(models.isSameSize(weights));

  // combined lambda is the weighted sum of lambdas:
  itsLambda = 0.0;
  Image<SurpriseModelOD>::const_iterator m = models.begin(),
    stop = models.end();
  double wsum = 0.0;  // find out how much total weight we use
  Image<float>::const_iterator w = weights.begin();
  while(m != stop)
    {
      const double weight = double(*w++);
      itsLambda += weight * m->itsLambda;
      wsum += weight;
      ++m;
    }
}

// ######################################################################
inline void SurpriseModelOD::combineFrom(const Image<SurpriseModelOD>&
                                         models, const Image<float>& weights,
                                         const Point2D<int>& pos,
                                         const int width,
                                         const int height,
                                         const int offset)
{
  ASSERT(weights.getWidth() == 2 * width + 1);
  ASSERT(weights.getHeight() == 2 * height + 1);
  ASSERT(models.coordsOk(pos));

  // combined lambda is the weighted sum of lambdas:
  itsLambda = 0.0;
  Image<SurpriseModelOD>::const_iterator m = models.begin();
  Image<float>::const_iterator w = weights.begin();
  double wsum = 0.0;  // find out how much total weight we use

  // define bounds of our scan in the weight mask and go for it:
  w += offset - pos.i;
  for (int j = 0; j < height; j ++)
    {
      for (int i = 0; i < width; i ++)
        {
          const double weight = double(*w++);
          if (weight)
            {
              itsLambda += weight * m->itsLambda;
              wsum += weight;
            }
          ++m;
        }
      // skip to next line:
      w += width + 1;
    }
}

// ######################################################################
inline double SurpriseModelOD::getMean() const
{ return itsLambda; }

// ######################################################################
inline double SurpriseModelOD::getVar() const
{ return itsLambda; }

// ######################################################################
inline double SurpriseModelOD::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
