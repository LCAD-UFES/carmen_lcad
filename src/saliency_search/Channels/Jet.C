/*!@file Channels/Jet.C a simple jet */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/Jet.C $
// $Id: Jet.C 7367 2006-11-01 00:47:02Z rjpeters $
//

#include "Channels/Jet.H"

#include "Util/Assert.H"
#include "Image/Image.H"
#include "rutz/compat_cmath.h"

#include <iostream>

// ######################################################################
JetSpec::JetSpec()
{ spec = NULL; freeMem(); }

// ######################################################################
JetSpec::JetSpec(const JetSpec& js)
{ spec = NULL; freeMem(); *this = js; }

// ######################################################################
JetSpec& JetSpec::operator=(const JetSpec& js)
{
  freeMem(); JF *jfp = js.spec;
  while(jfp) {
    if (addFeature(jfp->feature) == false)
      LFATAL("Cannot add feature");
    JFT *jftp = jfp->jft;
    while(jftp) {
      if (addFeatureType(jfp->feature, jftp->ftype) == false)
        LFATAL("Cannot add feature type");
      for (int i = 0; i < jftp->nbidx; i++)
        if (addIndexRange(jfp->feature, jftp->ftype,
                          jftp->idxmin[i], jftp->idxmax[i]) == false)
          LFATAL("Cannot add index range");
      jftp = jftp->next;
    }
    jfp = jfp->next;
  }
  return *this;
}

// ######################################################################
void JetSpec::freeMem()
{
  JF *jfp = spec;
  while(jfp) {
    JFT *jftp = jfp->jft;
    while(jftp) {
      if (jftp->nbidx) { delete [] jftp->idxmin; delete [] jftp->idxmax; }
      JFT *del = jftp; jftp = jftp->next; delete del;
    }
    JF *del = jfp; jfp = jfp->next; delete del;
  }
  spec = NULL; jetSize = 0;
  for (int f = 0; f < NBVISUALFEATURES; f ++)
    for (int t = 0; t < NBVISUALFEATURETYPES; t ++)
      dataIdx[f][t] = NULL; // undefined
}

// ######################################################################
JetSpec::~JetSpec()
{ freeMem(); }

// ######################################################################
bool JetSpec::addFeature(const VisualFeature f)
{
  JF *jfp = spec;
  if (spec == NULL) { spec = new JF; jfp = spec; } // first feature ever added
  else  // add to end of chained list
    while(true) {
      if (jfp->feature == f)
        { LERROR("Feature %s already exists!", featureName(f)); return false; }
      if (jfp->next) jfp = jfp->next;
      else { jfp->next = new JF; jfp = jfp->next; break; }
    }
  jfp->feature = f; jfp->jft = NULL; jfp->next = NULL;
  //LDEBUG("Added feature %s", featureName(f));
  return true;
}

// ######################################################################
bool JetSpec::addFeatureType(const VisualFeature f, const VisualFeatureType t)
{
  // if we don't have this feature yet, add it:
  if (hasFeature(f) == false) addFeature(f);

  // if we already have this feature/type, error:
  if (hasFeatureType(f, t)) {
    LERROR("Already have %s/%s", featureName(f), featureTypeName(t));
    return false;
  }

  // find the feature:
  JF *jfp = spec;
  while(jfp) {
    if (jfp->feature == f) { // ok, this is our feature; add type
      JFT *jftp = jfp->jft;
      if (jfp->jft == NULL)
        { jfp->jft = new JFT; jftp = jfp->jft; } // first type ever added
      else  // add to end of chained list
        while(true) {
          if (jftp->ftype == t)
            { LERROR("Feature type %s for feature %s already exists!",
                     featureTypeName(t), featureName(f)); return false; }
          if (jftp->next) jftp = jftp->next;
          else { jftp->next = new JFT; jftp = jftp->next; break; }
        }
      jftp->ftype = t; jftp->nbidx = 0; jftp->idxmin = NULL;
      jftp->idxmax = NULL; jftp->next = NULL;
      updateDataIdx();  // will update indexBase, siz, and data pointers
      //LDEBUG("Added feature type %s/%s", featureName(f), featureTypeName(t));
      return true;
    }
    jfp = jfp->next;
  }
  LERROR("Feature %s not found?", featureName(f));
  return false;
}

// ######################################################################
bool JetSpec::addIndexRange(const VisualFeature f, const VisualFeatureType t,
                            const int indexmin, const int indexmax)
{
  // if we don't have this feature yet, add it:
  if (hasFeature(f) == false) addFeature(f);

  // if we don't have this feature type yet, add it:
  if (hasFeatureType(f, t) == false) addFeatureType(f, t);

  // let's add the range:
  JFT *jftp = dataIdx[f][t];
  int n = jftp->nbidx;
  int *nmin = new int[n + 1], *nmax = new int[n + 1];
  if (n) {
    memcpy(nmin, jftp->idxmin, n * sizeof(int));
    memcpy(nmax, jftp->idxmax, n * sizeof(int));
    delete [] jftp->idxmin; delete [] jftp->idxmax;
  }
  nmin[n] = indexmin; nmax[n] = indexmax;
  jftp->nbidx ++; jftp->idxmin = nmin; jftp->idxmax = nmax;
  updateDataIdx();  // will update indexBase, siz and data pointers
  //LDEBUG("Added %s/%s range %d = [%d..%d]", featureName(f),
  //       featureTypeName(t), jftp->nbidx-1, indexmin, indexmax);
  return true;
}

// ######################################################################
void JetSpec::print() const
{
  JF *jfp = spec; std::cout<<"=== JetSpec [size = "<<jetSize<<']'<<std::endl;
  while(jfp) {
    std::cout<<featureName(jfp->feature)<<':'<<std::endl;
    JFT *jftp = jfp->jft;
    while(jftp) {
      std::cout<<"  "<<featureTypeName(jftp->ftype)<<": [base = "<<
        jftp->indexBase<<", size = "<<jftp->siz<<"]: ";
      for (int i = 0; i < jftp->nbidx; i ++)
        std::cout<<'['<<jftp->idxmin[i]<<".."<<jftp->idxmax[i]<<"] ";
      std::cout<<std::endl;
      jftp = jftp->next;
    }
    jfp = jfp->next;
  }
}

// ######################################################################
void JetSpec::updateDataIdx()
{
  JF *jfp = spec; jetSize = 0;
  while(jfp) {
    JFT *jftp = jfp->jft;
    while(jftp) {
      // update indexBase for this feature/type:
      jftp->indexBase = jetSize;

      // store data pointer in our accelerated-access array
      dataIdx[jfp->feature][jftp->ftype] = jftp;

      // compute size for this feature type
      int siz = 0;
      if (jftp->nbidx) {
        siz = 1;
        for (int i = 0; i < jftp->nbidx; i ++)
          siz *= jftp->idxmax[i] - jftp->idxmin[i] + 1;
      }
      jftp->siz = siz;
      jetSize += siz;

      // skip to next feature type for this feature
      jftp = jftp->next;
    }
    // skip to nect feature
    jfp = jfp->next;
  }
}

// ######################################################################
// ######################################################################
template <class T>
Jet<T>::Jet(const rutz::shared_ptr<JetSpec> js) : Image<T>()
{ spec = js; this->resize(spec->getJetSize(), 1); }

// ######################################################################
template <class T>
Jet<T>::Jet() : Image<T>(), spec(NULL)
{ }

// ######################################################################
template <class T>
void Jet<T>::init(const rutz::shared_ptr<JetSpec> js)
{ ASSERT(spec.get()); spec = js; this->resize(spec->getJetSize(), 1); }

// ######################################################################
template <class T>
Jet<T>::~Jet()
{ this->freeMem(); }

// ######################################################################
double raodistance(const Jet<float>& j1, const Jet<float>& j2,
                   const int idxmin, const int idxmax)
{
  // this is a sum of jet distances computed separately for each scale
  double sumsq[idxmax - idxmin + 1];
  // initialize it
  for (int i = 0; i < idxmax - idxmin + 1; i++)
    sumsq[i] = 0.0;

  for (int f = 0; f < NBVISUALFEATURES; f ++)
    {
      VisualFeature ff = VisualFeature(f);
      if (j1.hasFeatureType(ff, RAW))
        {
          int nr = j1.getNbIndexRanges(ff, RAW);
          switch(nr)
            {
            case 1:
              {
                int imin = 0, imax = 0;
                j1.getIndexRange(ff, RAW, 0, imin, imax);
                ASSERT(idxmin >= imin && idxmax <= imax);
                for (int i = idxmin; i <= idxmax; i ++)
                  {
                    float val1 = j1.getVal(ff, RAW, i);
                    float val2 = j2.getVal(ff, RAW, i);
                    double val;
                    if (isnan(val1) || isnan(val2))
                      val = 0.0;
                    else
                      val = double(val1 - val2);
                    sumsq[i - idxmin] += val * val;
                  }
                //LINFO("%s: [%d .. %d]",featureName(ff),imin, imax);
              }
              break;
            case 2:
              {
                int rmin = 0, rmax = 0;
                j1.getIndexRange(ff, RAW, 0, rmin, rmax);
                for (int r = rmin; r <= rmax; r ++)
                  {
                    int imin = 0, imax = 0;
                    j1.getIndexRange(ff, RAW, 1, imin, imax);
                    ASSERT(idxmin >= imin && idxmax <= imax);
                    for (int i = idxmin; i <= idxmax; i ++)
                      {
                        float val1 = j1.getVal(ff, RAW, r, i);
                        float val2 = j2.getVal(ff, RAW, r, i);
                        double val;
                        if (isnan(val1) || isnan(val2))
                          val = 0.0;
                        else
                          val = double(val1 - val2);
                        sumsq[i - idxmin] += val * val;
                      }
                    //LINFO("%s:(%d) [%d .. %d]",featureName(ff),r,imin,imax);
                  }
              }
              break;
            default:
              LFATAL("Deep hierarchical Jets not supported");
            }
        }
    }
  double d = 0.0;
  for (int i = 0; i <= idxmax - idxmin; i ++) d += sumsq[i];
  return d;
}


// ######################################################################
#ifdef INVT_INST_BYTE
template class Jet<byte>;
#endif
#ifdef INVT_INST_INT16
template class Jet<int16>;
#endif
#ifdef INVT_INST_INT32
template class Jet<int32>;
#endif
#ifdef INVT_INST_FLOAT
template class Jet<float>;
#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
