/*!@file Surprise/SurpriseImage.C a 2D array of SurpriseModel objects */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseImage.C $
// $Id: SurpriseImage.C 11562 2009-08-08 00:35:40Z dberg $
//

#include "Surprise/SurpriseImage.H"

#include "Image/Kernels.H"  // for gaussianBlob()
#include "Util/Assert.H"

// ######################################################################
template <class T>
SurpriseImage<T>::SurpriseImage() :
  Image<T>()
{ }

// ######################################################################
template <class T>
SurpriseImage<T>::SurpriseImage(const Dims& dims) :
  Image<T>(dims, NO_INIT)
{ }

// ######################################################################
template <class T>
SurpriseImage<T>::SurpriseImage(const double updfac,
                                const Image<double>& sampleval,
                                const Image<double>& samplevar) :
  Image<T>(sampleval.getDims(), NO_INIT)
{ init(updfac, sampleval, samplevar); }

// ######################################################################
template <class T>
SurpriseImage<T>::~SurpriseImage()
{ }

// ######################################################################
template <class T>
void SurpriseImage<T>::reset()
{
  typename SurpriseImage<T>::iterator
    itr = this->beginw(), stop = this->endw();
  while (itr != stop) (itr++)->reset();
}

// ######################################################################
template <class T>
void SurpriseImage<T>::init(const double updfac,
                            const Image<double>& sampleval,
                            const Image<double>& samplevar)
{
  ASSERT(this->isSameSize(sampleval) && this->isSameSize(samplevar));

  typename SurpriseImage<T>::iterator
    itr = this->beginw(), stop = this->endw();
  Image<double>::const_iterator s = sampleval.begin(), se = samplevar.begin();

  while (itr != stop) (itr++)->init(updfac, *s++, *se++);
}

// ######################################################################
template <class T>
void SurpriseImage<T>::resetUpdFac(const double updfac)
{
  typename SurpriseImage<T>::iterator i = this->beginw(), stop = this->endw();
  while(i != stop) (i++)->resetUpdFac(updfac);
}

// ######################################################################
template <class T>
Image<double> SurpriseImage<T>::surprise(const SurpriseImage<T>& other)
{
  ASSERT(this->isSameSize(other));

  Image<double> ret(this->getDims(), NO_INIT);

  typename SurpriseImage<T>::iterator i = this->beginw(), stop = this->endw();
  typename SurpriseImage<T>::const_iterator o = other.begin();
  Image<double>::iterator r = ret.beginw();

  while(i != stop) *r++ = (i++)->surprise(*o++);

  return ret;
}
// ######################################################################
template <class T>
void SurpriseImage<T>::preComputeHyperParams(const SurpriseImage<T>& models)
{
  typename SurpriseImage<T>::iterator i = this->beginw(), stop = this->endw();
  typename SurpriseImage<T>::const_iterator m = models.begin();

  while(i != stop) (i++)->preComputeHyperParams(*m++);
}

// ######################################################################
template <class T>
void SurpriseImage<T>::neighborhoods(const SurpriseImage<T>& models,
                                     const float neighsigma)
{
  *this = models; // set us to same size as models and inherit their params

  // this is a very inefficient implementation... will be optimized some day:
  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;
  typename SurpriseImage<T>::iterator itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      {
        const Image<float> g =
          gaussianBlob<float>(this->getDims(), pos, neighsigma, neighsigma);
        (itr++)->combineFrom(models, g);
      }
}

// ######################################################################
template <class T>
void SurpriseImage<T>::neighborhoods(const SurpriseImage<T>& models,
                                     const Image<float>& weights)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;
  typename SurpriseImage<T>::iterator itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models, weights, pos, w, h, o);
  }
}

// ######################################################################
template <>
void SurpriseImage<SurpriseModelSP>::neighborhoods(
                                  const SurpriseImage<SurpriseModelSP>& models,
                                  const Image<float>& weights)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;

  // get a copy of the models then pre-compute alpha/beta
  SurpriseImage<SurpriseModelSP> models2       = models;
  SurpriseImage<SurpriseModelSP>::iterator itr = models2.beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->preSetAlpha();

  itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models2, weights, pos, w, h, o);
  }
}

// ######################################################################
template <>
void SurpriseImage<SurpriseModelSP1>::neighborhoods(
                                  const SurpriseImage<SurpriseModelSP1>& models,
                                  const Image<float>& weights)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;

  // get a copy of the models then pre-compute alpha/beta
  SurpriseImage<SurpriseModelSP1> models2       = models;
  SurpriseImage<SurpriseModelSP1>::iterator itr = models2.beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->preSetAlpha();

  itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models2, weights, pos, w, h, o);
  }
}

// ######################################################################
template <>
void SurpriseImage<SurpriseModelSPC>::neighborhoods(
                                  const SurpriseImage<SurpriseModelSPC>& models,
                                  const Image<float>& weights)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;

  // get a copy of the models then pre-compute alpha/beta
  SurpriseImage<SurpriseModelSPC> models2       = models;
  SurpriseImage<SurpriseModelSPC>::iterator itr = models2.beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->preSetAlpha();

  itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models2, weights, pos, w, h, o);
  }
}

// ######################################################################
template <>
void SurpriseImage<SurpriseModelSPF>::neighborhoods(
                                  const SurpriseImage<SurpriseModelSPF>& models,
                                  const Image<float>& weights)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;

  // get a copy of the models then pre-compute alpha/beta
  SurpriseImage<SurpriseModelSPF> models2       = models;
  SurpriseImage<SurpriseModelSPF>::iterator itr = models2.beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->preSetAlpha();

  itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models2, weights, pos, w, h, o);
  }
}

// ######################################################################
template <class T>
void SurpriseImage<T>::neighborhoods(const SurpriseImage<T>& models,
                                     const Image<float>& weights,
                                     const bool NO_INIT)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  if(!NO_INIT)
    *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;
  typename SurpriseImage<T>::iterator itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models, weights, pos, w, h, o);
  }
}

// ######################################################################
template <>
void SurpriseImage<SurpriseModelSP>::neighborhoods(
                                  const SurpriseImage<SurpriseModelSP>& models,
                                  const Image<float>& weights,
                                  const bool NO_INIT)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  if(!NO_INIT)
    *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;

  // get a copy of the models then pre-compute alpha/beta
  SurpriseImage<SurpriseModelSP> models2       = models;
  SurpriseImage<SurpriseModelSP>::iterator itr = models2.beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->preSetAlpha();

  itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models2, weights, pos, w, h, o);
  }
}


// ######################################################################
template <>
void SurpriseImage<SurpriseModelSP1>::neighborhoods(
                                  const SurpriseImage<SurpriseModelSP1>& models,
                                  const Image<float>& weights,
                                  const bool NO_INIT)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  if(!NO_INIT)
    *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;

  // get a copy of the models then pre-compute alpha/beta
  SurpriseImage<SurpriseModelSP1> models2       = models;
  SurpriseImage<SurpriseModelSP1>::iterator itr = models2.beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->preSetAlpha();

  itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models2, weights, pos, w, h, o);
  }
}

// ######################################################################
template <>
void SurpriseImage<SurpriseModelSPC>::neighborhoods(
                                  const SurpriseImage<SurpriseModelSPC>& models,
                                  const Image<float>& weights,
                                  const bool NO_INIT)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  if(!NO_INIT)
    *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;

  // get a copy of the models then pre-compute alpha/beta
  SurpriseImage<SurpriseModelSPC> models2       = models;
  SurpriseImage<SurpriseModelSPC>::iterator itr = models2.beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->preSetAlpha();

  itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models2, weights, pos, w, h, o);
  }
}

// ######################################################################
template <>
void SurpriseImage<SurpriseModelSPF>::neighborhoods(
                                  const SurpriseImage<SurpriseModelSPF>& models,
                                  const Image<float>& weights,
                                  const bool NO_INIT)
{
  ASSERT(weights.getWidth()  == 2 * models.getWidth()  + 1);
  ASSERT(weights.getHeight() == 2 * models.getHeight() + 1);

  if(!NO_INIT)
    *this = models; // set us to same size as models and inherit their params

  const int w = this->getWidth(), h = this->getHeight(); Point2D<int> pos;

  // get a copy of the models then pre-compute alpha/beta
  SurpriseImage<SurpriseModelSPF> models2       = models;
  SurpriseImage<SurpriseModelSPF>::iterator itr = models2.beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->preSetAlpha();

  itr = this->beginw();
  for (pos.j = 0; pos.j < h; pos.j ++)
  {
    const int o = w + (2 * w + 1) * (h - pos.j);
    for (pos.i = 0; pos.i < w; pos.i ++)
      (itr++)->combineFrom(models2, weights, pos, w, h, o);
  }
}

// ######################################################################
template <class T>
Image<double> SurpriseImage<T>::getMean() const
{
  Image<double> ret(this->getDims(), NO_INIT);

  typename SurpriseImage<T>::const_iterator
    i = this->begin(), stop = this->end();
  Image<double>::iterator r = ret.beginw();

  while(i != stop) *r++ = (i++)->getMean();

  return ret;
}

// ######################################################################
template <class T>
Image<double> SurpriseImage<T>::getVar() const
{
  Image<double> ret(this->getDims(), NO_INIT);

  typename SurpriseImage<T>::const_iterator
    i = this->begin(), stop = this->end();
  Image<double>::iterator r = ret.beginw();

  while(i != stop) *r++ = (i++)->getVar();

  return ret;
}

// ######################################################################
// explicit instantiations:
template class SurpriseImage<SurpriseModelSG>;
template class SurpriseImage<SurpriseModelSP>;
template class SurpriseImage<SurpriseModelSP1>;
template class SurpriseImage<SurpriseModelSPC>;
template class SurpriseImage<SurpriseModelSPF>;
template class SurpriseImage<SurpriseModelCS>;
template class SurpriseImage<SurpriseModelGG>;
template class SurpriseImage<SurpriseModelPM>;
template class SurpriseImage<SurpriseModelOD>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
