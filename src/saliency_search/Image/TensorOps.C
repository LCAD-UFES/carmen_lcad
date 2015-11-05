/*!@file Image/TensorOps.C Mathematical Tensor operations               */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/TensorOps.C $
// $Id: TensorOps.C 14490 2011-02-11 19:46:05Z lior $

#ifndef IMAGE_TENSOROPS_C_DEFINED
#define IMAGE_TENSOROPS_C_DEFINED

#include "Image/TensorOps.H"

// WARNING: Try not include any other "Image*Ops.H" headers here -- if
// you find that you are needing such a header, then the function
// you're writing probably belongs outside Image_TensorOps.C.
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Image/MatrixOps.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/safecopy.H"
#include "rutz/trace.h"
#include "GUI/DebugWin.H"

#include <algorithm>
#include <cmath>
#include <climits>
#include <cfloat>
#include <numeric> // for std::accumulate(), etc.

#if defined(INVT_USE_MMX) || defined(INVT_USE_SSE) || defined(INVT_USE_SSE2)
#include "Util/mmx-sse.H"
#endif

//Some functions inspired from a matlab version of tensor voting by Trevor Linton
// (http://www.mathworks.com/matlabcentral/fileexchange/21051-tensor-voting-framework)


// ######################################################################
template <class T>
TensorField getTensor(const Image<T>& img, int kernelSize)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typedef typename promote_trait<T, float>::TP TF;

  ASSERT( (kernelSize == 3 ) | (kernelSize == 5));
  TensorField tensorField(img.getDims(), NO_INIT);

  bool useScharr = true; 

  typename Image<T>::const_iterator src = img.begin();

  const int w = img.getWidth(), h = img.getHeight();
  typename Image<TF>::iterator t1 = tensorField.t1.beginw();
  typename Image<TF>::iterator t2 = tensorField.t2.beginw();
  typename Image<TF>::iterator t3 = tensorField.t3.beginw();
  typename Image<TF>::iterator t4 = tensorField.t4.beginw();
  TF zero = TF();

  // first rows are all zeros:
  switch(kernelSize) {
    case 3:
      for (int i = 0; i < w; i ++)
      {
        *t1++ = zero; *t2++ = zero;
        *t3++ = zero; *t4++ = zero;
      }
      src += w;
      break;
    case 5: //Zero out the two rows
      for (int i = 0; i < w*2; i ++)
      {
        *t1++ = zero; *t2++ = zero;
        *t3++ = zero; *t4++ = zero;
      }
      src += w*2;
      break;
  }

  // loop over inner rows:
  switch(kernelSize)
  {
    case 3:
      for (int j = 1; j < h-1; j ++)
      {
        // leftmost pixel is zero:
        *t1++ = zero; *t2++ = zero;
        *t3++ = zero; *t4++ = zero;
        ++src;
        // loop over inner columns:
        if (useScharr)
        {
          for (int i = 1; i < w-1; i ++)
          {
            TF valx = -3*src[-1*w + -1] + 0*src[-1*w + 0] + 3*src[-1*w + 1]
                    + -10*src[ 0*w + -1] + 0*src[ 0*w + 0] + 10*src[ 0*w + 1]
                    + -3*src[ 1*w + -1] + 0*src[ 1*w + 0] + 3*src[ 1*w + 1];

            TF valy =  3*src[-1*w + -1] +  10*src[-1*w + 0] +  3*src[-1*w + 1]
                    +  0*src[ 0*w + -1] +  0*src[ 0*w + 0] +  0*src[ 0*w + 1]
                    + -3*src[ 1*w + -1] + -10*src[ 1*w + 0] + -3*src[ 1*w + 1];

            *t1++ = valx*valx; *t2++ = valx*valy;
            *t3++ = valx*valy; *t4++ = valy*valy;
            ++ src;
          }
        } else {
          //Use the sobel operator
          for (int i = 1; i < w-1; i ++)
          {
            TF valx = -1*src[-1*w + -1] + 0*src[-1*w + 0] + 1*src[-1*w + 1]
              + -2*src[ 0*w + -1] + 0*src[ 0*w + 0] + 2*src[ 0*w + 1]
              + -1*src[ 1*w + -1] + 0*src[ 1*w + 0] + 1*src[ 1*w + 1];

            TF valy =  1*src[-1*w + -1] +  2*src[-1*w + 0] +  1*src[-1*w + 1]
              +  0*src[ 0*w + -1] +  0*src[ 0*w + 0] +  0*src[ 0*w + 1]
              + -1*src[ 1*w + -1] + -2*src[ 1*w + 0] + -1*src[ 1*w + 1];

            *t1++ = valx*valx; *t2++ = valx*valy;
            *t3++ = valx*valy; *t4++ = valy*valy;
            ++ src;
          }
        }

        // rightmost pixel is zero:
        *t1++ = zero; *t2++ = zero;
        *t3++ = zero; *t4++ = zero;
        ++src;
      }
      break;
    case 5:
      for (int j = 2; j < h-2; j ++)
      {
        // leftmost pixel is zero:
        *t1++ = zero; *t2++ = zero;
        *t3++ = zero; *t4++ = zero;
        ++src;
        // loop over inner columns:
        for (int i = 2; i < w-2; i ++)
        {
          TF valx = -1*src[-2*w + -2] +  -2*src[-2*w + -1] + 0*src[-2*w + 0] +  2*src[-2*w + 1] + 1*src[-2*w + 2]
            + -4*src[-1*w + -2] +  -8*src[-1*w + -1] + 0*src[-1*w + 0] +  8*src[-1*w + 1] + 4*src[-1*w + 2]
            + -6*src[ 0*w + -2] + -12*src[ 0*w + -1] + 0*src[ 0*w + 0] + 12*src[ 0*w + 1] + 6*src[ 0*w + 2]
            + -4*src[ 1*w + -2] +  -8*src[ 1*w + -1] + 0*src[ 1*w + 0] +  8*src[ 1*w + 1] + 4*src[ 1*w + 2]
            + -1*src[ 2*w + -2] +  -2*src[ 2*w + -1] + 0*src[ 2*w + 0] +  2*src[ 2*w + 1] + 1*src[ 2*w + 2];

          TF valy =  1*src[-2*w + -2] +  4*src[-2*w + -1] +   6*src[-2*w + 0] +  4*src[-2*w + 1] +  1*src[-2*w + 2]
            +  2*src[-1*w + -2] +  8*src[-1*w + -1] +  12*src[-1*w + 0] +  8*src[-1*w + 1] +  2*src[-1*w + 2]
            +  0*src[ 0*w + -2] +  0*src[ 0*w + -1] +   0*src[ 0*w + 0] +  0*src[ 0*w + 1] +  0*src[ 0*w + 2]
            + -2*src[ 1*w + -2] + -8*src[ 1*w + -1] + -12*src[ 1*w + 0] + -8*src[ 1*w + 1] + -2*src[ 1*w + 2]
            + -1*src[ 2*w + -2] + -4*src[ 2*w + -1] +  -6*src[ 2*w + 0] + -4*src[ 2*w + 1] + -1*src[ 2*w + 2];
          
          *t1++ = valx*valx; *t2++ = valx*valy;
          *t3++ = valx*valy; *t4++ = valy*valy;
          ++ src;
        }
        // rightmost pixel is zero:
        *t1++ = zero; *t2++ = zero;
        *t3++ = zero; *t4++ = zero;
        ++src;
      }
      break;
  }


  // last rows are all zeros:
  switch(kernelSize)
  {
    case 3:
      for (int i = 0; i < w; i ++)
      {
        *t1++ = zero; *t2++ = zero;
        *t3++ = zero; *t4++ = zero;
      }
      break;
    case 5: //last two rows
      for (int i = 0; i < w*2; i ++)
      {
        *t1++ = zero; *t2++ = zero;
        *t3++ = zero; *t4++ = zero;
      }
      break;
  }


  return tensorField;

}


// ######################################################################
Image<float> getTensorMag(const TensorField& tf)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<float> mag(tf.t1.getDims(), NO_INIT);

  Image<float>::iterator magPtr = mag.beginw();

  Image<float>::const_iterator t1 = tf.t1.begin();
  Image<float>::const_iterator t4 = tf.t4.begin();
  Image<float>::const_iterator stop = tf.t1.end();

  while (t1 != stop)
    {
      *magPtr = sqrt(*t1 + *t4);

      magPtr++;
      ++t1;
      ++t4;
    }

  return mag;
}


// ######################################################################
TensorField getTensor(const EigenSpace& eigen)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  TensorField tf(eigen.l1.getDims(), NO_INIT);

  //Convert from eigen space to tensor, only 2D for now
  for(uint i=0; i<eigen.l1.size(); i++)
  {
    tf.t1.setVal(i, (eigen.l1.getVal(i)*eigen.e1[0].getVal(i)*eigen.e1[0].getVal(i)) +
                        (eigen.l2.getVal(i)*eigen.e2[0].getVal(i)*eigen.e2[0].getVal(i)) );

    tf.t2.setVal(i, (eigen.l1.getVal(i)*eigen.e1[0].getVal(i)*eigen.e1[1].getVal(i)) +
                        (eigen.l2.getVal(i)*eigen.e2[0].getVal(i)*eigen.e2[1].getVal(i)) );

    tf.t3.setVal(i, tf.t2.getVal(i)); //Should this be optimized out?

    tf.t4.setVal(i, (eigen.l1.getVal(i)*eigen.e1[1].getVal(i)*eigen.e1[1].getVal(i)) +
                        (eigen.l2.getVal(i)*eigen.e2[1].getVal(i)*eigen.e2[1].getVal(i)) );
  }

  return tf;

}

// ######################################################################
EigenSpace getTensorEigen(const TensorField& tf)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  EigenSpace eigen;

  eigen.e1 = ImageSet<float>(2, tf.t1.getDims(), NO_INIT);
  eigen.e2 = ImageSet<float>(2, tf.t1.getDims(), NO_INIT);
  eigen.l1 = Image<float>(tf.t1.getDims(), NO_INIT);
  eigen.l2 = Image<float>(tf.t1.getDims(), NO_INIT);

  for(uint i=0; i<tf.t1.size(); i++)
  {
    //trace/2
    double trace = (tf.t1.getVal(i) + tf.t4.getVal(i))/2;

    double a = tf.t1.getVal(i) - trace;
    double b = tf.t2.getVal(i);

    double ab2 = sqrt((a*a) + (b*b));

    eigen.l1.setVal(i, ab2 + trace);
    eigen.l2.setVal(i, -ab2 + trace);

    double theta = atan2 (ab2-a, b);

    eigen.e1[0].setVal(i, cos(theta));
    eigen.e1[1].setVal(i, sin(theta));
    eigen.e2[0].setVal(i, -sin(theta));
    eigen.e2[1].setVal(i, cos(theta));
  }

  return eigen;

}


void nonMaxSurp(TensorField& tf, float radius)
{
  EigenSpace eigen = getTensorEigen(tf);
  Image<float> features = eigen.l1-eigen.l2;

  for(int j=0; j<features.getHeight(); j++)
    for(int i=0; i<features.getWidth(); i++)
    {
      float val = features.getVal(i,j);
      if(val > 0)
      {
        float u = eigen.e1[1].getVal(i,j);
        float v = eigen.e1[0].getVal(i,j);
        float ang=atan(-u/v); 

        //Look to each direction of the edge and
        //check if its a max
        int dx = int(radius*cos(ang));
        int dy = int(radius*sin(ang));

        if (features.coordsOk(i+dx, j-dy) &&
            features.coordsOk(i-dx, j+dy) )
        {
          //Remove the tensor if its not a local maxima
          if (val < features.getVal(i+dx, j-dy) ||
              val <= features.getVal(i-dx, j+dy) )
            tf.setVal(i,j,0);
        }
      }
    }
}


// Include the explicit instantiations
#include "inst/Image/TensorOps.I"


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_TENSOROPS_C_DEFINED
