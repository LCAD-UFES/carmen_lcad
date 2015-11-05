/*!@file Image/TensorOps.H Mathematical Tensor operations */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/TensorOps.H $
// $Id: TensorOps.H 14490 2011-02-11 19:46:05Z lior $
//

#ifndef IMAGE_TENSOROPS_H_DEFINED
#define IMAGE_TENSOROPS_H_DEFINED

#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/FilterOps.H"
#include "Util/Promotions.H"
#include <vector>


struct EigenSpace
{
  ImageSet<float> e1;
  ImageSet<float> e2;
  Image<float> l1;
  Image<float> l2;
};

//!Here we represent tensors as an image set with 4 images indexed as:
//| t1  t2 |
//| t3  t4 |
struct TensorField
{
  Image<float> t1;
  Image<float> t2;
  Image<float> t3;
  Image<float> t4;

  TensorField()
  {
  }

  TensorField(Dims dims, InitPolicy policy)
  {
    t1 = Image<float>(dims, policy);
    t2 = Image<float>(dims, policy);
    t3 = Image<float>(dims, policy);
    t4 = Image<float>(dims, policy);
  }

  TensorField operator+(const TensorField& _tf) const
  {
    TensorField tf(t1.getDims(), NO_INIT);

    tf.t1 = t1 + _tf.t1;
    tf.t2 = t2 + _tf.t2;
    tf.t3 = t3 + _tf.t3;
    tf.t4 = t4 + _tf.t4;

    return tf;
  }

  TensorField& operator+=(const TensorField& tf)
  {
    t1 += tf.t1;
    t2 += tf.t2;
    t3 += tf.t3;
    t4 += tf.t4;

    return *this;
  }

  TensorField& operator/=(const float s)
  {
    t1 /= s;
    t2 /= s;
    t3 /= s;
    t4 /= s;

    return *this;
  }

  void setVal(const uint i, float val)
  {
    t1.setVal(i, val);
    t2.setVal(i, val);
    t3.setVal(i, val);
    t4.setVal(i, val);
  }

  void setVal(const uint i, const uint j, float val)
  {
    t1.setVal(i,j, val);
    t2.setVal(i,j, val);
    t3.setVal(i,j, val);
    t4.setVal(i,j, val);
  }

  //! Get another tensor field, and set this tensor field to the max between the two
  void setMax(const TensorField& tf)
  {
    ASSERT(tf.t1.size() == t1.size());

    for(uint i=0; i<tf.t1.size(); i++)
    {
      //The input feature value
      //trace/2
      double trace = (tf.t1.getVal(i) + tf.t4.getVal(i))/2;

      double a = tf.t1.getVal(i) - trace;
      double b = tf.t2.getVal(i);

      double ab = sqrt((a*a) + (b*b));

      //The local feature value
      //trace/2
      double trace2 = (t1.getVal(i) + t4.getVal(i))/2;

      double a2 = t1.getVal(i) - trace2;
      double b2 = t2.getVal(i);

      double ab2 = sqrt((a2*a2) + (b2*b2));

      if (ab+ab > ab2+ab2)
      {
        t1.setVal(i, tf.t1.getVal(i));
        t2.setVal(i, tf.t2.getVal(i));
        t3.setVal(i, tf.t3.getVal(i));
        t4.setVal(i, tf.t4.getVal(i));
      }

    }

  }
};



//! Get the tensor of an image by measuring the gradient
template <class T>
TensorField getTensor(const Image<T>& img, int kernelSize=5);

//! Get the tensor magnitude value
Image<float> getTensorMag(const TensorField& tf);

//! Get the tensor of an image from eigenvectors and eigenvalues
TensorField getTensor(const EigenSpace& eigen);

//! Get the eigenvectors and values from a tensor
EigenSpace getTensorEigen(const TensorField& tf);

//! Perform non maximal surpression
void nonMaxSurp(TensorField& tf, float radius=1.5);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_TENSOROPS_H_DEFINED
