/*!@file Channels/Jet.H a simple jet (i.e., cortical hypercolumn) */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/Jet.H $
// $Id: Jet.H 15310 2012-06-01 02:29:24Z itti $
//

#ifndef JET_H_DEFINED
#define JET_H_DEFINED

#include "Util/Assert.H"
#include "Util/Types.H"
#include "Channels/VisualFeatures.H"
#include "Util/Promotions.H"
#include "Util/log.H"
#include "rutz/shared_ptr.h"
#include <cstdarg>
#include <vector>

template <class T> class Image;

// ######################################################################
//! This class specifies the features, scales and indices represented in a Jet
/*! A Jet object requires a valid JetSpec object at construction */
class JetSpec
{
public:
  /*! @name Constructors, destructor and operators */
  //@{

  //! Empty constructor; starts with an empty specification
  JetSpec();

  //! Copy constructor
  JetSpec(const JetSpec& js);

  //! Assignment operator
  JetSpec& operator=(const JetSpec& js);

  //! Free all allocated memory and un-initialize
  void freeMem();

  //! Destructor
  ~JetSpec();
  //@}

  /*! @name Build methods */
  //@{
  //! Add a new feature; returns true on success
  bool addFeature(const VisualFeature f);

  //! Add a new representation type for an existing feature; true on success
  bool addFeatureType(const VisualFeature f, const VisualFeatureType t);

  //! Add a new index range for existing feature and type; true on success
  /*! Valid indices will be in [indexmin..indexmax] inclusive on both sides. */
  bool addIndexRange(const VisualFeature f, const VisualFeatureType t,
                     const int indexmin, const int indexmax);
  //@}

  /*! @name Access methods */
  //@{
  //! Get the number of components in a jet with our specs
  inline int getJetSize() const;

  //! Query whether we have some representation for given feature
  inline bool hasFeature(const VisualFeature f) const;

  //! Query whether we have some representation for given feature/type
  inline bool hasFeatureType(const VisualFeature f,
                             const VisualFeatureType t) const;

  //! Get index of first Jet element for given feature & type
  /*! Also returns the total size allocated in the Jet for this feature
    and type */
  inline int getIndexBase(const VisualFeature f, const VisualFeatureType t,
                          int& siz) const;

  //! Get number of index ranges
  inline int getNbIndexRanges(const VisualFeature f,
                              const VisualFeatureType t) const;

  //! Get an index range; true if range exists in JetSpec
  inline bool getIndexRange(const VisualFeature f, const VisualFeatureType t,
                            const int idxnum, int& idxmin, int& idxmax) const;

  //! Debugging printout to stdout
  void print() const;

  //@}

  /*! @name Jet data access methods */
  //@{
  //! Get index in Jet array data for given feature, type and set of indices
  /*! The ... arguments should be as many ints as we have have
    addIndexRange()'d when building the JetSpec, and their values
    should be within the ranges specified at the time of
    addIndexRange(). Returns -1 if feature or type unknown or if any
    of the indices is out of range. */
  inline int getIndex(const VisualFeature f,
                      const VisualFeatureType t, ...) const;

  //! getIndex() with va_list parameters
  inline int getIndexA(const VisualFeature f, const VisualFeatureType t,
                va_list& a) const;

  //! getIndex() with vector of parameters
  inline int getIndexV(const VisualFeature f, const VisualFeatureType t,
                       const std::vector<int>& v) const;
  //@}

private:
  //! Chained list element that holds index range defs for given feature type
  struct JFT {
    VisualFeatureType ftype; //!< feature type
    int indexBase;           //!< index in Jet of first data element
    int siz;                 //!< total number of elements
    int nbidx;               //!< number of indices
    int *idxmin;             //!< minimum index value
    int *idxmax;             //!< max index val; valid range: idxmin<=i<idxmax
    JFT *next;               //!< pointer to next feature type for that feature
  };

  //! chained list element that holds JTF definitions for given feature
  struct JF {
    VisualFeature feature;   //!< feature
    JFT *jft;                //!< pointer to list of feature types
    JF *next;                //!< pointer to next feature in this specification
  };

  JF *spec;  //!< Specification as tree of JF's holding JFT's

  //! accelerated access to individula feature/type defs
  JFT *dataIdx[NBVISUALFEATURES][NBVISUALFEATURETYPES];
  int jetSize;  //!< size of a Jet using our specs

  //! update our accelerated access array in dataIdx
  void updateDataIdx();
};


// ######################################################################
//! This class specifies a Jet, i.e., array of feature values for one location
/*! The features represented in the Jet are defined by a JetSpec
object that must be passed at construction of the Jet. Jet extends
Image and inherits all of its functionality. The JetSpec is passed as
a rutz::shared_ptr, so that the original can run out of scope and be
destroyed (and the actual data will be freed upon destruction of the
last Jet built from that JetSpec). */

template <class T>
class Jet : public Image<T> {
public:
  /*! @name Constructors, initialization and destructor */
  //@{
  //! Constructor; must provide a fully-configured JetSpec
  Jet(const rutz::shared_ptr<JetSpec> js);

  //! Uninitialized constructor (for arrays of Jets)
  Jet();

  //! Initialization of an uninitialized Jet
  void init(const rutz::shared_ptr<JetSpec> js);

  //! Load from an Image, presenving our JetSpec
  /*! The Jet must have been initialized previously and its size must
    match that of the Image. This is potentially dangerous, and mostly
    provided so that we can inherit all of the Image free functions
    that return images (e.g., those of Image_MathOps.H). This is a
    cheap operation thanks to the copy-on-write/ref-counting */
  inline Jet<T>& operator=(const Image<T>& img);

  //! Load from an image, preserving our JetSpec and doing a clamped convert
  /*! The Jet must have been initialized previously and its size must
    match that of the Image. This is potentially dangerous, and mostly
    provided so that we can inherit all of the Image free functions
    that return images (e.g., those of Image_MathOps.H). This is an
    expensive operation in which we convert each pixel in turn. */
  template <class TT> inline Jet<T>& operator=(const Image<TT>& img);

  //! Destructor
  ~Jet();
  //@}

  /*! @name Access functions */
  //@{
  //! Get a value, for given feature, type and indices
  inline T getVal(const VisualFeature f, const VisualFeatureType t, ...) const;

  //! Get a value, for given feature, type and vector of indices
  inline T getValV(const VisualFeature f, const VisualFeatureType t,
            const std::vector<int>& v) const;

  //! Get a value by absolute index in the Jet array. Use with caution!
  inline T getVal(const int idx) const;

  //! Set a value, for given feature, type and indices
  inline void setVal(const T& val, const VisualFeature f,
                     const VisualFeatureType t, ...);

  //! Set a value, for given feature, type and vector of indices
  inline void setValV(const T& val, const VisualFeature f,
                      const VisualFeatureType t, const std::vector<int>& v);

  //! Set a value by absolute index in the Jet array. Use with caution!
  inline void setVal(const int idx, const T& val);

  //! Retrieve the JetSpec in case your original has run out of scope
  inline rutz::shared_ptr<JetSpec> getSpec() const;
  //@}

  /*! @name Convenience JetSpec Access Methods */
  //@{
  //! Query whether we have some representation for given feature
  inline bool hasFeature(const VisualFeature f) const;

  //! Query whether we have some representation for given feature/type
  inline bool hasFeatureType(const VisualFeature f,
                             const VisualFeatureType t) const;

  //! Get index of first Jet element for given feature & type
  /*! Also returns total size allocated in the Jet for this feature/type */
  inline int getIndexBase(const VisualFeature f,
                          const VisualFeatureType t, int& siz) const;

  //! Get number of index ranges
  inline int getNbIndexRanges(const VisualFeature f,
                              const VisualFeatureType t) const;

  //! Get an index range; true if range exists in JetSpec
  inline bool getIndexRange(const VisualFeature f, const VisualFeatureType t,
                            const int idxnum, int& idxmin, int& idxmax) const;
  //@}

private:
  rutz::shared_ptr<JetSpec> spec;
};

// ######################################################################
//! Compute distance between two Jets like Rao et al., Vis Res, 2002
double raodistance(const Jet<float>& j1, const Jet<float>& j2,
                   const int idxmin, const int idxmax);

// ######################################################################
template <class Stream, class T>
inline Stream& operator<<(Stream& s, const Jet<T> &j)
{
  int siz = j.getSize();
  for (int i = 0; i < siz; i ++) s<<j.Image<T>::getVal(i)<<' ';
  return s;
}

// ######################################################################
template <class Stream, class T>
inline Stream& operator>>(Stream& s, Jet<T> &j)
{
  int siz = j.getSize(); float x;
  for (int i = 0; i < siz; i ++) { s>>x; j.Image<T>::setVal(i, x); }
  return s;
}

// ######################################################################
// ######################################################################
// ########## INLINED METHODS for JetSpec:
// ######################################################################
// ######################################################################

// ######################################################################
inline int JetSpec::getJetSize() const
{ return jetSize; }

// ######################################################################
inline bool JetSpec::hasFeature(const VisualFeature f) const
{
  JF *jfp = spec;
  while(jfp) if (jfp->feature == f) return true; else jfp = jfp->next;
  return false;
}

// ######################################################################
inline bool JetSpec::hasFeatureType(const VisualFeature f,
                                    const VisualFeatureType t) const
{ return (dataIdx[f][t] != NULL); }

// ######################################################################
inline int JetSpec::getIndexBase(const VisualFeature f,
                                 const VisualFeatureType t, int& siz) const
{
  ASSERT(dataIdx[f][t]);
  siz = dataIdx[f][t]->siz;
  return dataIdx[f][t]->indexBase;
}

// ######################################################################
inline int JetSpec::getNbIndexRanges(const VisualFeature f,
                                     const VisualFeatureType t) const
{
  ASSERT(dataIdx[f][t]);
  return dataIdx[f][t]->nbidx;
}

// ######################################################################
inline bool JetSpec::getIndexRange(const VisualFeature f,
                                   const VisualFeatureType t,
                                   const int idxnum, int& idxmin,
                                   int& idxmax) const
{
  if (dataIdx[f][t] == NULL || idxnum < 0 || idxnum >= dataIdx[f][t]->nbidx)
    return false;
  idxmin = dataIdx[f][t]->idxmin[idxnum];
  idxmax = dataIdx[f][t]->idxmax[idxnum];
  return true;
}

// ######################################################################
inline int JetSpec::getIndex(const VisualFeature f, const VisualFeatureType t,
                             ...) const
{
  va_list a; va_start(a, t); int idx = getIndexA(f, t, a); va_end(a);
  return idx;
}

// ######################################################################
inline int JetSpec::getIndexA(const VisualFeature f,
                              const VisualFeatureType t, va_list& a) const
{
  // do a rapid fetch of data for given feature & type:
  JFT *jftp = dataIdx[f][t];
  if (jftp == NULL) {
    LERROR("Unknown feature/type: %s/%s", featureName(f), featureTypeName(t));
    return -1;
  }

  // FIXME: how can we check that the number of va_args matches the
  // number of indices?

  // add offset according to parameters passed by caller:
  int index = 0;
  for (int i = 0; i < jftp->nbidx; i ++) {
    int arg = va_arg(a, int);
    if (arg < jftp->idxmin[i] || arg > jftp->idxmax[i])
      { LERROR("Index %d value %d out of range [%d..%d]", i, arg,
               jftp->idxmin[i], jftp->idxmax[i]); return -1; }
    index = index * (jftp->idxmax[i] - jftp->idxmin[i] + 1) +
      (arg - jftp->idxmin[i]);
  }
  return index + jftp->indexBase;
}

// ######################################################################
inline int JetSpec::getIndexV(const VisualFeature f, const VisualFeatureType t,
                              const std::vector<int>& v) const
{
  // do a rapid fetch of data for given feature & type:
  JFT *jftp = dataIdx[f][t];
  if (jftp == NULL) {
    LERROR("Unknown feature/type: %s/%s", featureName(f), featureTypeName(t));
    return -1;
  }

  // check that caller passed right number of indices:
  if (int(v.size()) != jftp->nbidx) {
    LERROR("Passed %" ZU " indices but %s/%s has %d", v.size(),
           featureName(f), featureTypeName(t), jftp->nbidx);
    return -1;
  }

  //std::cout<<featureName(f)<<'/'<<featureTypeName(t)<<"[ ";
  //for (int i = 0; i < v.size(); i ++) std::cout<<v[i]<<' ';

  // add offset according to parameters passed by caller:
  int index = 0;
  for (int i = 0; i < jftp->nbidx; i ++) {
    int arg = v[i];
    if (arg < jftp->idxmin[i] || arg > jftp->idxmax[i])
      { LERROR("Index %d value %d out of range [%d..%d]", i, arg,
               jftp->idxmin[i], jftp->idxmax[i]); return -1; }
    index = index * (jftp->idxmax[i] - jftp->idxmin[i] + 1) +
      (arg - jftp->idxmin[i]);
  }

  //std::cout<<"] = "<<index + jftp->indexBase<<std::endl;

  return index + jftp->indexBase;
}


// ######################################################################
// ######################################################################
// ########## INLINED METHODS for Jet:
// ######################################################################
// ######################################################################

// ######################################################################
template <class T> inline
Jet<T>& Jet<T>::operator=(const Image<T>& img)
{
  ASSERT(this->isSameSize(img));
  Image<T> A_copy(img);
  this->swap(A_copy);
  return *this;
}

// ######################################################################
template <class T> template <class TT> inline
Jet<T>& Jet<T>::operator=(const Image<TT>& img)
{
  ASSERT(this->isSameSize(img));
  Image<T>::operator=(img); return *this;
}

// ######################################################################
template <class T> inline
T Jet<T>::getVal(const VisualFeature f, const VisualFeatureType t, ...) const
{

  va_list a; va_start(a, t); int idx = spec->getIndexA(f, t, a); va_end(a);
  return Image<T>::getVal(idx);
}

// ######################################################################
template <class T> inline
T Jet<T>::getValV(const VisualFeature f, const VisualFeatureType t,
                  const std::vector<int>& v) const
{
  int idx = spec->getIndexV(f, t, v);
  return Image<T>::getVal(idx);
}

// ######################################################################
template <class T> inline
T Jet<T>::getVal(const int idx) const
{ return this->begin()[idx]; }

// ######################################################################
template <class T> inline
void Jet<T>::setVal(const T& val, const VisualFeature f,
                    const VisualFeatureType t, ...)
{
  va_list a; va_start(a, t);
  setVal(spec->getIndexA(f, t, a), val);
  va_end(a);
}

// ######################################################################
template <class T> inline
void Jet<T>::setValV(const T& val, const VisualFeature f,
                     const VisualFeatureType t, const std::vector<int>& v)
{
  setVal(spec->getIndexV(f, t, v), val);
}

// ######################################################################
template <class T> inline
void Jet<T>::setVal(const int idx, const T& val)
{ this->beginw()[idx] = val; }

// ######################################################################
template <class T> inline
rutz::shared_ptr<JetSpec> Jet<T>::getSpec() const
{ return spec; }

// ######################################################################
template <class T> inline
bool Jet<T>::hasFeature(const VisualFeature f) const
{ return spec->hasFeature(f); }

// ######################################################################
template <class T> inline
bool Jet<T>::hasFeatureType(const VisualFeature f,
                            const VisualFeatureType t) const
{ return spec->hasFeatureType(f, t); }

// ######################################################################
template <class T> inline
int Jet<T>::getIndexBase(const VisualFeature f, const VisualFeatureType t,
                         int& siz) const
{ return spec->getIndexBase(f, t, siz); }


// ######################################################################
template <class T> inline
int Jet<T>::getNbIndexRanges(const VisualFeature f,
                             const VisualFeatureType t) const
{ return spec->getNbIndexRanges(f, t); }


// ######################################################################
template <class T> inline
bool Jet<T>::getIndexRange(const VisualFeature f, const VisualFeatureType t,
                           const int idxnum, int& idxmin, int& idxmax) const
{ return spec->getIndexRange(f, t, idxnum, idxmin, idxmax); }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
