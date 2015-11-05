/*!@file ModelNeuron/NeuralLayer.C Class implementation of neural weights*/

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/Weights.C $

#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "Image/LowPass.H"
#include "Image/LowPassLpt.H"

#include "ModelNeuron/Weights.H"

// ######################################################################
// some convolution functions. Same as in Convolutions.H but for type double
// ######################################################################
namespace WeightFilter
{
Image<double>
convolveHmax(const Image<double>& src, const Image<double>& filter)
{
  ASSERT(src.initialized());
  const int Nx = filter.getWidth(), Ny = filter.getHeight();
  ASSERT((Nx & 1) && (Ny & 1));

  const int w = src.getWidth(), h = src.getHeight();

  Image<double> result(w, h, NO_INIT);
  Image<double>::const_iterator sptr = src.begin();
  Image<double>::iterator dptr = result.beginw();
  Image<double>::const_iterator const filt = filter.begin();

  int kkk = Nx * Ny - 1;
  int Nx2 = (Nx - 1) / 2, Ny2 = (Ny - 1) / 2;
  // very inefficient implementation; one has to be crazy to use non
  // separable filters anyway...
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      {
        double sum = 0.0, sumw = 0.0;
        for (int kj = 0; kj < Ny; ++kj)
          {
            int kjj = kj + j - Ny2;
            if (kjj >= 0 && kjj < h)
              for (int ki = 0; ki < Nx; ++ki)
                {
                  int kii = ki + i - Nx2;
                  if (kii >= 0 && kii < w)
                    {
                      double fil = filt[kkk - ki - Nx*kj];
                      double img = sptr[kii + w * kjj];
                      sum += img * fil; sumw += img * img;
                    }
                }
          }
        //if (sumw > 0.0F) sum /= fastSqrt(sumw);
        if (sumw > 0.0) sum = fabs(sum)/sqrt(sumw);
        *dptr++ = sum;
      }
  return result;
}

// ######################################################################
Image<double>
optConvolve(const Image<double>& src, const Image<double>& f)
{
  ASSERT(src.initialized());

  const int src_w = src.getWidth();
  const int src_h = src.getHeight();

  Image<double>::const_iterator filter = f.begin();
  const int fil_w = f.getWidth();
  const int fil_h = f.getHeight();

  ASSERT((fil_w & 1) && (fil_h & 1));

  Image<double> result(src_w, src_h, NO_INIT);
  Image<double>::const_iterator sptr = src.begin();
  Image<double>::iterator dptr = result.beginw();

  const int fil_end = fil_w * fil_h - 1;
  const int Nx2 = (fil_w - 1) / 2;
  const int Ny2 = (fil_h - 1) / 2;

  const int srow_skip = src_w-fil_w;

  for (int dst_y = 0; dst_y < src_h; ++dst_y)
    {
      // Determine if we're safely inside the image in the y-direction:
      const bool y_clean = dst_y >= Ny2 && dst_y <  (src_h - Nx2);

      for (int dst_x = 0; dst_x < src_w; ++dst_x, ++dptr)
        {
          // Determine if we're safely inside the image in the x-direction:
          const bool x_clean = dst_x >= Nx2 && dst_x <  (src_w - Nx2);

          // Here is where we pick whether we can use the optimized inner
          // loop (in cases where the filter and image patch overlap
          // completely) or whether we must use the inner loop that can
          // handle boundary conditions.

          if (x_clean && y_clean)
            {
              double dst_val = 0.0;

              Image<double>::const_iterator fptr = filter+fil_end;

              Image<double>::const_iterator srow_ptr =
                sptr + src_w*(dst_y-Nx2) + dst_x - Nx2;

              for (int f_y = 0; f_y < fil_h; ++f_y)
                {
                  for (int f_x = 0; f_x < fil_w; ++f_x)
                    dst_val += (*srow_ptr++) * (*fptr--);

                  srow_ptr += srow_skip;
                }
              *dptr = dst_val;
              continue;
            }
          else
            {
              // OK, we're at an image boundary, so what do we do to make
              // up for the missing pixels? The approach here is to
              // compute the average value of the non-missing pixels, and
              // proceed as if the missing pixels had that value. This
              // minimizes the introduction of edge artifacts e.g. when
              // convolving with an oriented filter.

              double dst_val = 0.0;
              double src_sum = 0.0;
              int src_cnt = 0;
              double fil_sum_skipped = 0.0;

              for (int f_y = 0; f_y < fil_h; ++f_y)
                {
                  const int src_y = f_y + dst_y - Ny2;
                  if (src_y >= 0 && src_y < src_h)
                    {
                      for (int f_x = 0; f_x < fil_w; ++f_x)
                        {
                          const double fil = filter[fil_end - f_x - fil_w*f_y];

                          const int src_x = f_x + dst_x - Nx2;
                          if (src_x >= 0 && src_x < src_w)
                            {
                              const double src_val = sptr[src_x + src_w * src_y];
                              dst_val += src_val * fil;
                              src_sum += src_val;
                              ++src_cnt;
                            }
                          else
                            {
                              fil_sum_skipped += fil;
                            }
                        }
                    }
                  else
                    {
                      for (int f_x = 0; f_x < fil_w; ++f_x)
                        fil_sum_skipped += filter[fil_end - f_x - fil_w*f_y];
                    }
                }
              const double src_avg = src_sum / src_cnt;
              *dptr = dst_val + (fil_sum_skipped * src_avg);
            }
        }
    }
  return result;
}

  // ######################################################################
  static Image<double>
  xFilter(const Image<double>& src, const double* hFilt, const int hfs,
          ConvolutionBoundaryStrategy boundary)
  {
    const int w = src.getWidth(), h = src.getHeight();
    
    if (hfs == 0) return src;  // no filter
    Image<double> result(w, h, NO_INIT);
    Image<double>::const_iterator sptr = src.begin();
    Image<double>::iterator dptr = result.beginw();

    const int hfs2 = (hfs - 1) / 2;

    // flip the filter to accelerate convolution:
    double *hFilter = new double[hfs]; double sumh = 0.0;
    for (int x = 0; x < hfs; x ++)
      { sumh += hFilt[x]; hFilter[hfs-1-x] = hFilt[x]; }

    // *** horizontal pass ***
    if (w < hfs)  // special function for very small images
      {
        switch (boundary)
          {
          case CONV_BOUNDARY_ZERO:
            {
              for (int j = 0; j < h; j ++)
                for (int i = 0; i < w; i ++)
                  {
                    double val = 0;
                    for (int k = 0; k < hfs; k ++)
                      if (i + k - hfs2 >= 0 && i + k - hfs2 < w)
                        {
                          val += sptr[k - hfs2] * hFilter[k];
                        }
                    *dptr++ = val; sptr ++;
                  }
            }
            break;
          case CONV_BOUNDARY_CLEAN:
            {
              for (int j = 0; j < h; j ++)
                for (int i = 0; i < w; i ++)
                  {
                    double val = 0.0; double sum = 0.0;
                    for (int k = 0; k < hfs; k ++)
                      if (i + k - hfs2 >= 0 && i + k - hfs2 < w)
                        {
                          val += sptr[k - hfs2] * hFilter[k];
                          sum += hFilter[k];
                        }
                    *dptr++ = val * sumh / sum; sptr ++;
                  }
            }
            break;
          case CONV_BOUNDARY_REPLICATE:
            {
              for (int j = 0; j < h; j ++)
                for (int i = 0; i < w; i ++)
                  {
                    double val = 0.0;
                    for (int k = 0; k < hfs; k ++)
                      if (i + k - hfs2 < 0)
                        {
                          val += sptr[-i] * hFilter[k];
                        }
                      else if (i + k - hfs2 >= w)
                        {
                          val += sptr[w-1-i] * hFilter[k];
                        }
                      else
                        {
                          val += sptr[k - hfs2] * hFilter[k];
                        }
                    *dptr++ = val; sptr ++;
                  }
            }
            break;
          default:
            LFATAL("invalid convolution boundary strategy %d",
                   (int) boundary);
          }
      }
    else  // function for reasonably large images
      for (int jj = 0; jj < h; jj ++)
        {
          // leftmost points:
          switch (boundary)
            {
            case CONV_BOUNDARY_ZERO:
              {
                for (int j = hfs2; j > 0; --j)
                  {
                    double val = 0.0;
                    for (int k = j; k < hfs; ++k)
                      {
                        val += sptr[k - j] * hFilter[k];
                      }
                    *dptr++ = val;
                  }
              }
              break;
            case CONV_BOUNDARY_CLEAN:
              {
                for (int j = hfs2; j > 0; --j)
                  {
                    double val = 0.0; double sum = 0.0;
                    for (int k = j; k < hfs; ++k)
                      {
                        val += sptr[k - j] * hFilter[k];
                        sum += hFilter[k];
                      }
                    *dptr++ = val * sumh / sum;
                  }
              }
              break;
            case CONV_BOUNDARY_REPLICATE:
              {
                for (int j = hfs2; j > 0; --j)
                  {
                    double val = 0.0;
                    for (int k = 0; k < j; ++k)
                      {
                        val += sptr[0] * hFilter[k];
                      }
                    for (int k = j; k < hfs; ++k)
                      {
                        val += sptr[k - j] * hFilter[k];
                      }
                    *dptr++ = val;
                  }
              }
              break;
            default:
              LFATAL("invalid convolution boundary strategy %d",
                     (int) boundary);
            }
          // bulk (far from the borders):
          for (int i = 0; i < w - hfs + 1; i ++)
            {
              double val = 0.0;
              for (int k = 0; k < hfs; k ++) val += sptr[k] * hFilter[k];
              *dptr++ = val; sptr++;
            }
          // rightmost points:
          switch (boundary)
            {
            case CONV_BOUNDARY_ZERO:
              {
                for (int j = hfs - 1; j > hfs2; --j)
                  {
                    double val = 0.0;
                    for (int k = 0; k < j; ++k)
                      {
                        val += sptr[k] * hFilter[k];
                      }
                    *dptr++ = val;
                    sptr++;
                  }
              }
              break;
            case CONV_BOUNDARY_CLEAN:
              {
                for (int j = hfs - 1; j > hfs2; --j)
                  {
                    double val = 0.0; double sum = 0.0;
                    for (int k = 0; k < j; ++k)
                      {
                        val += sptr[k] * hFilter[k];
                        sum += hFilter[k];
                      }
                    *dptr++ = val * sumh / sum;
                    sptr++;
                  }
              }
              break;
            case CONV_BOUNDARY_REPLICATE:
              {
                for (int j = hfs - 1; j > hfs2; --j)
                  {
                    double val = 0.0;
                    for (int k = 0; k < j; ++k)
                      {
                        val += sptr[k] * hFilter[k];
                      }
                    for (int k = j; k < hfs; ++k)
                      {
                        val += sptr[j-1] * hFilter[k];
                      }
                    *dptr++ = val;
                    sptr++;
                  }
              }
              break;
            default:
              LFATAL("invalid convolution boundary strategy %d",
                     (int) boundary);
            }
          sptr += hfs2;  // sptr back to same as dptr (start of next line)
        }
    delete [] hFilter;
    return result;
  }

  // ######################################################################
  static Image<double>
  yFilter(const Image<double>& src, const double* vFilt, const int vfs,
          ConvolutionBoundaryStrategy boundary)
  {
    const int w = src.getWidth(), h = src.getHeight();

    if (vfs == 0) return src;  // no filter
    Image<double> result(w, h, NO_INIT);
    Image<double>::const_iterator sptr = src.begin();
    Image<double>::iterator dptr = result.beginw();

    const int vfs2 = (vfs - 1) / 2;

    // flip the filter to accelerate convolution:
    double *vFilter = new double[vfs]; double sumv = 0.0;
    for (int x = 0; x < vfs; x ++)
      { sumv += vFilt[x]; vFilter[vfs-1-x] = vFilt[x]; }

    int ww[vfs];
    for (int i = 0; i < vfs; i ++) ww[i] = w * i; // speedup precompute

    if (h < vfs)  // special function for very small images
      {
        switch (boundary)
          {
          case CONV_BOUNDARY_ZERO:
            {
              for (int j = 0; j < h; j ++)
                for (int i = 0; i < w; i ++)
                  {
                    double val = 0.0;
                    for (int k = 0; k < vfs; k ++)
                      if (j + k - vfs2 >= 0 && j + k - vfs2 < h)
                        {
                          val += sptr[w * (k - vfs2)] * vFilter[k];
                        }
                    *dptr++ = val; sptr ++;
                  }
            }
            break;
          case CONV_BOUNDARY_CLEAN:
            {
              for (int j = 0; j < h; j ++)
                for (int i = 0; i < w; i ++)
                  {
                    double val = 0.0; double sum = 0.0;
                    for (int k = 0; k < vfs; k ++)
                      if (j + k - vfs2 >= 0 && j + k - vfs2 < h)
                        {
                          val += sptr[w * (k - vfs2)] * vFilter[k];
                          sum += vFilter[k];
                        }
                    *dptr++ = val * sumv / sum; sptr ++;
                  }
            }
            break;
          case CONV_BOUNDARY_REPLICATE:
            {
              for (int j = 0; j < h; j ++)
                for (int i = 0; i < w; i ++)
                  {
                    double val = 0.0;
                    for (int k = 0; k < vfs; k ++)
                      if (j + k - vfs2 < 0)
                        {
                          val += sptr[w * (-j)] * vFilter[k];
                        }
                      else if (j + k - vfs2 >= h)
                        {
                          val += sptr[w * (h-1-j)] * vFilter[k];
                        }
                      else
                        {
                          val += sptr[w * (k - vfs2)] * vFilter[k];
                        }
                    *dptr++ = val; sptr ++;
                  }
            }
            break;
          default:
            LFATAL("invalid convolution boundary strategy %d",
                   (int) boundary);
          }
      }
    else  // function for reasonably large images
      {
        // top points:
        switch (boundary)
          {
          case CONV_BOUNDARY_ZERO:
            {
              for (int j = vfs2; j > 0; --j)
                {
                  for (int i = 0; i < w; ++i)  // scan all points of given horiz
                    {
                      double val = 0.0;
                      for (int k = j; k < vfs; ++k)
                        {
                          val += sptr[ww[k - j]] * vFilter[k];
                        }
                      *dptr++ = val;
                      sptr++;
                    }
                  sptr -= w;   // back to top-left corner
                }
            }
            break;
          case CONV_BOUNDARY_CLEAN:
            {
              for (int j = vfs2; j > 0; --j)
                {
                  for (int i = 0; i < w; ++i)  // scan all points of given horiz
                    {
                      double val = 0.0; double sum = 0.0;
                      for (int k = j; k < vfs; ++k)
                        {
                          val += sptr[ww[k - j]] * vFilter[k];
                          sum += vFilter[k];
                        }
                      *dptr++ = val * sumv / sum;
                      sptr++;
                    }
                  sptr -= w;   // back to top-left corner
                }
            }
            break;
          case CONV_BOUNDARY_REPLICATE:
            {
              for (int j = vfs2; j > 0; --j)
                {
                  for (int i = 0; i < w; ++i)  // scan all points of given horiz
                    {
                      double val = 0.0;
                      for (int k = 0; k < j; ++k)
                        {
                          val += sptr[ww[0]] * vFilter[k];
                        }
                      for (int k = j; k < vfs; ++k)
                        {
                          val += sptr[ww[k - j]] * vFilter[k];
                        }
                      *dptr++ = val;
                      sptr++;
                    }
                  sptr -= w;   // back to top-left corner
                }
            }
            break;
          default:
            LFATAL("invalid convolution boundary strategy %d",
                   (int) boundary);
          }
        // bulk (far from edges):
        for (int j = 0; j < h - vfs + 1; j ++)
          for (int i = 0; i < w; i ++)
            {
              double val = 0.0;
              for (int k = 0; k < vfs; k ++) val += sptr[ww[k]] * vFilter[k];
              *dptr++ = val; sptr ++;
            }
        // bottommost points:
        switch (boundary)
          {
          case CONV_BOUNDARY_ZERO:
            {
              for (int j = vfs - 1; j > vfs2; --j)
                for (int i = 0; i < w; ++i)
                  {
                    double val = 0.0;
                    for (int k = 0; k < j; ++k)
                      {
                        val += sptr[ww[k]] * vFilter[k];
                      }
                    *dptr++ = val;
                    sptr ++;
                  }
            }
            break;
          case CONV_BOUNDARY_CLEAN:
            {
              for (int j = vfs - 1; j > vfs2; --j)
                for (int i = 0; i < w; ++i)
                  {
                    double val = 0.0; double sum = 0.0;
                    for (int k = 0; k < j; ++k)
                      {
                        val += sptr[ww[k]] * vFilter[k];
                        sum += vFilter[k];
                      }
                    *dptr++ = val * sumv / sum;
                    sptr ++;
                  }
            }
            break;
          case CONV_BOUNDARY_REPLICATE:
            {
              for (int j = vfs - 1; j > vfs2; --j)
                for (int i = 0; i < w; ++i)
                  {
                    double val = 0.0;
                    for (int k = 0; k < j; ++k)
                      {
                        val += sptr[ww[k]] * vFilter[k];
                      }
                    for (int k = j; k < vfs; ++k)
                      {
                        val += sptr[ww[j-1]] * vFilter[k];
                      }
                    *dptr++ = val;
                    sptr ++;
                  }
            }
            break;
          default:
            LFATAL("invalid convolution boundary strategy %d",
                   (int) boundary);
          }
      }
    delete [] vFilter;
    return result;
  }

  // ######################################################################
  Image<double>
  sepFilter(const Image<double>& src, const Image<double>& hFilter,
            const Image<double>& vFilter,
            ConvolutionBoundaryStrategy boundary)
  {
    ASSERT(hFilter.is1D() || hFilter.getSize() == 0);
    ASSERT(vFilter.is1D() || vFilter.getSize() == 0);

    Image<double> result = src;
    const int hfs = hFilter.getSize();
    const int vfs = vFilter.getSize();
    if (hfs > 0) result = xFilter(result, hFilter.getArrayPtr(), 
                                  hfs, boundary);
    if (vfs > 0) result = yFilter(result, vFilter.getArrayPtr(), 
                                  vfs, boundary);
    
    return result;
  }
}//end namespace WeightFilter

// ######################################################################
// WeightsAll implementation
// ######################################################################
WeightsAll::WeightsAll() : WeightsDerived<WeightsAll>(), itsW(0.0) { };

// ######################################################################
WeightsAll::WeightsAll(const double& weight) : WeightsDerived<WeightsAll>(), 
                                               itsW(weight) { itsInit = true; }

// ######################################################################
Image<double> WeightsAll::compute(const Image<double>& in)
{
  double s = sum(in);
  s *= itsW;
  Image<double> out(in.getDims(), NO_INIT);
  out.clear(s);
  return out;
}

// ######################################################################
// WeightsUniform implementation
// ######################################################################
WeightsUniform::WeightsUniform() : WeightsDerived<WeightsUniform>(), itsW(0.0) { };

// ######################################################################
WeightsUniform::WeightsUniform(const double& weight) : WeightsDerived<WeightsUniform>(), itsW(weight) 
{ itsInit = true; }

// ######################################################################
Image<double> WeightsUniform::compute(const Image<double>& in)
{
  Image<double> out = in * itsW;
  return out;
}

// ######################################################################
// WeightsBinomial implementation
// ######################################################################
WeightsBinomial::WeightsBinomial() : WeightsDerived<WeightsBinomial>(),
                                     itsTaps(0), itsIter(0), subCenter(false), 
                                     itsCenter(0.0), itsWeight(0.0), 
                                     itsBp(NONE), itsSetup(false) { };

// ######################################################################
WeightsBinomial::WeightsBinomial(const double& std, const double& weight, 
                                 const bool doSubCenter, const BorderPolicy bp,
                                 const uint taps) :
  itsTaps(taps), itsIter(0), subCenter(doSubCenter), itsCenter(0.0), 
  itsWeight(weight), itsBp(bp), itsSetup(false)
{
  //get required iterations
  itsIter = uint((std*std) / (double(taps - 1)/4.0));
  itsInit = true;
}

// ######################################################################
Image<double> WeightsBinomial::compute(const Image<double>& in)
{
  Image<double> out = in;
  if (!itsSetup) 
    {
      Image<double> impulse(in.getDims(), ZEROS);
      uint cx = (uint) (in.getWidth() / 2) - 1;
      uint cy = (uint) (in.getHeight() / 2) - 1;
      impulse.setVal(cx, cy, 1.0);
      impulse = filterBinomial(impulse,itsIter, itsTaps, itsBp);
      itsCenter  = 1.0 / impulse.getVal(cx,cy);
      if (!subCenter)
        itsCenter*= itsWeight;
      
      itsSetup = true;
    }

  //do the filtering
  out = filterBinomial(out, itsIter, itsTaps, itsBp);

  //subtract energy from self excitation, then scale
  if (subCenter)  
    {
      out *= itsCenter;
      out -= in;
      out *= itsWeight;
    }
  else
    out *= itsCenter;
  
  return out;
}

// ######################################################################
// WeightsCS implementation
// ######################################################################
WeightsCS::WeightsCS() :  WeightsDerived<WeightsCS>(), itsW(0.0, 0.0, false, NONE, 0), 
                          itsInh(0.0) { };

// ######################################################################
WeightsCS::WeightsCS(const double& estd, const double& eweight, 
                     const double& inhibit, const bool doSubCenter, 
                     const BorderPolicy bp, const uint taps) :
  WeightsDerived<WeightsCS>(), itsW(estd, eweight, doSubCenter, bp, taps), itsInh(inhibit)
{ itsInit = true; }

// ######################################################################
Image<double> WeightsCS::compute(const Image<double>& in)
{
  Image<double> e = itsW.compute(in);
  const double s = sum(in) * itsInh;
  e -= s;
  return e;
}    

// ######################################################################
// WeightsDoG implementation
// ######################################################################
WeightsDoG::WeightsDoG() : WeightsDerived<WeightsDoG>(), itsE(0.0, 0.0, false, NONE, 0), 
                           itsI(0.0, 0.0, false, NONE, 0) { };

// ######################################################################
WeightsDoG::WeightsDoG(const double& estd, const double& istd, 
                       const double& ew, const double& iw, 
                       const bool subCenter, const BorderPolicy bp,
                       const uint taps) : WeightsDerived<WeightsDoG>(),
                                          itsE(estd, ew, subCenter, bp, taps), 
                                          itsI(istd, iw, subCenter, bp, taps)
{  
  itsInit = true;
}

// ######################################################################
Image<double> WeightsDoG::compute(const Image<double>& in)
{
  Image<double> e = itsE.compute(in);
  const Image<double> i = itsI.compute(in);
  e -= i;
  return e;
}    

/*
// ######################################################################
// WeightsGaussian implementation
// ######################################################################
WeightsGaussian::WeightsGaussian() : WeightsDerived<WeightsGaussian>(), subCenter(false), itsCenter(0.0), 
                                     itsStrat(CONV_BOUNDARY_CLEAN), itsKernel() { }

// ######################################################################
WeightsGaussian::WeightsGaussian(const double& std, const double& weight, 
                                 const bool doSubCenter, 
                                 const ConvolutionBoundaryStrategy strat) :
    WeightsDerived<WeightsGaussian>(),
    subCenter(doSubCenter), itsCenter(weight), itsStrat(strat), 
    itsKernel()
{
  itsKernel = gaussian<double>((float)weight, (float)std, 0, 1.0F);
}

// ######################################################################
Image<double> WeightsGaussian::compute(const Image<double>& in)
{
  Image<double> out = in;  
  
  //apply the filter bank 
  out = WeightFilter::sepFilter(out, itsKernel, itsKernel, itsStrat);
  
  //subtract energy from self excitation 
  if (subCenter)
    out = out - (in * itsCenter); 
  
  return out;      
}
*/

// ######################################################################
// Weights1D implementation
// ######################################################################
Weights1D::Weights1D() : WeightsDerived<Weights1D>(), subCenter(false), itsCenter(0.0), 
                         itsStrat(CONV_BOUNDARY_CLEAN), itsX(), itsY(), 
                         itsSetup(false) { }

// ######################################################################
Weights1D::Weights1D(const bool doSubCenter, 
                     const ConvolutionBoundaryStrategy strat) :
  WeightsDerived<Weights1D>(),
  subCenter(doSubCenter), itsCenter(0.0), itsStrat(strat), 
  itsX(), itsY(), itsSetup(false) { }

// ######################################################################
void Weights1D::addWeight(const Image<double>& wxy)
{
  itsX.push_back(wxy);
  itsY.push_back(wxy);
  itsInit = true;
}

// ######################################################################
void Weights1D::addWeight(const Image<double>& wx, const Image<double>& wy)
{
    itsX.push_back(wx);
    itsY.push_back(wy);  
    itsInit = true;
}

// ######################################################################
void Weights1D::clear()
{
  itsX.clear();
  itsY.clear();
  itsCenter = 0.0;
  itsSetup = false;
}

// ######################################################################
Image<double> Weights1D::compute(const Image<double>& in)
{

  if (!itsSetup && subCenter)
    {
      Image<double> impulse(in.getDims(), ZEROS);
      uint cx = (uint) (in.getWidth() / 2) - 1;
      uint cy = (uint) (in.getHeight() / 2) - 1;
      impulse.setVal(cx, cy, 1.0);
      //filter to get response at center
      for (uint jj = 0; jj < itsX.size(); ++jj)
        impulse = WeightFilter::sepFilter(impulse,itsX[jj],itsY[jj],itsStrat);
      itsCenter = impulse.getVal(cx,cy);
      itsSetup = true;
    }
  
  Image<double> out = in;  
  //apply the filter bank 
  for (uint jj = 0; jj < itsX.size(); ++jj)
    out = WeightFilter::sepFilter(out, itsX[jj], itsY[jj], itsStrat);
  
  //subtract energy from self excitation 
  if (subCenter)
    out = out - (in * itsCenter); 
  
  return out;      
}

// ######################################################################
// Weights2D implementation
// ######################################################################
Weights2D::Weights2D(const ConvolutionBoundaryStrategy strat) 
  : WeightsDerived<Weights2D>(), itsStrat(strat), itsW()
{ 
}

// ######################################################################
void Weights2D::addWeight(const Image<double>& w)
{
  if (w.is1D())
    LFATAL("Filter needs to be 2D for this Weights type");
  else
    itsW.push_back(w);
  itsInit = true;
}

// ######################################################################
void Weights2D::clear()
{
  itsW.clear();
  itsInit = false;
}

// ######################################################################
Image<double> Weights2D::compute(const Image<double>& in)
{
  Image<double> out = in;
  //apply the filter bank itsIter iterations
  if (itsStrat == CONV_BOUNDARY_CLEAN)
    for (uint jj = 0; jj < itsW.size(); ++jj)
      out = WeightFilter::convolveHmax(out, itsW[jj]); 
  else
    for (uint jj = 0; jj < itsW.size(); ++jj)
      out = WeightFilter::optConvolve(out, itsW[jj]);
  
  return out;      
}

// ######################################################################
// WeightsMask implementation
// ######################################################################}
WeightsMask::WeightsMask() : WeightsDerived<WeightsMask>()
{ 
}

// ######################################################################
void WeightsMask::addWeight(const Image<double>& w)
{
  itsW.push_back(w);
  itsInit = true;
}

// ######################################################################
void WeightsMask::clear()
{
  itsW.clear();
  itsInit = false;
}

// ######################################################################}
Image<double> WeightsMask::compute(const Image<double>& in)
{
  ASSERT((itsW.size() == in.size()) && in.size() > 0);

  Image<double> out(in.getDims(), ZEROS);
  std::vector<Image<double> >::const_iterator iter(itsW.begin()), end(itsW.end());
  Image<double>::iterator outiter(out.beginw());
  
  while (iter != end)
    {
      ASSERT(iter->size() == in.size());
      (*outiter++) = sum(in * (*iter++));
    }

return Image<double>(in.getDims(),ZEROS); 
}

// ######################################################################
// helper implementation
// ######################################################################
Image<double> filterBinomial(const Image<double>& in, const uint iter, 
                             const uint taps, const BorderPolicy bp)
{
  Image<double> out = in;
  if (bp == NONE)
    switch(taps)
      {
      case 3:
        for (uint i = 0; i < iter; ++i)
          out = lowPass3(out);
        break;
      case 5:
        for (uint i = 0; i < iter; ++i)
          out = lowPass5(out);
        break;
      default:
        {
          Image<double> k = binomialKernel(taps);
          for (uint i = 0; i < iter; ++i)
            out = WeightFilter::sepFilter(out, k, k, CONV_BOUNDARY_CLEAN);
        }
      }
  else
    for (uint i = 0; i < iter; ++i)
      out = lowPassLpt(out, taps, bp);    
  
  return out;
}

// ######################################################################
template <class W>
void printImpulse(const uint w, const uint h, W& weights)
{
  Image<double> in(w,h,ZEROS);
  int cx = in.getWidth() / 2 -1;
  int cy = in.getHeight() / 2 -1;
  in.setVal(cx,cy,1.0);
  in = weights.compute(in);
  printImage(in);
}

// ######################################################################
void printImage(const Image<double>& in)
{
  int cx = in.getWidth() / 2 - 1;

  Image<double>::const_iterator iter(in.begin() + cx);
  for (int i = 0; i < in.getHeight(); ++i)
    {
      LINFO("%3.5f", *iter);
      iter += in.getWidth();
    }
}

template 
void printImpulse(const uint w, const uint h, WeightsUniform& weights);
template 
void printImpulse(const uint w, const uint h, WeightsBinomial& weights);
template 
void printImpulse(const uint w, const uint h, WeightsCS& weights);
template 
void printImpulse(const uint w, const uint h, WeightsDoG& weights);
template 
void printImpulse(const uint w, const uint h, Weights1D& weights);
template 
void printImpulse(const uint w, const uint h, Weights2D& weights);
template 
void printImpulse(const uint w, const uint h, WeightsMask& weights);
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
