/*!@file Surprise/SurpriseMapFFT.C a surprise map */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseMapFFT.C $
// $Id: SurpriseMapFFT.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifdef HAVE_FFTW3_H

#include "Surprise/SurpriseMapFFT.H"

#include "Image/All.H"       // for gaussianBlob()
#include "Image/Kernels.H"
#include "Image/Conversions.H"
#include "Image/MathOps.H"
#include "Util/Assert.H"
#include "Raster/Raster.H"
#include "Image/Pixels.H"

#ifdef HAVE_FFTW3_H
#include <fftw3.h>
#endif

// ######################################################################
template <class T>
SurpriseMapFFT<T>::SurpriseMapFFT() :
  itsModels(), itsQlen(0), itsInitialModel(),
  itsNeighSigma(0.0f), itsLocSigma(0.0f), itsNweights(), itsNWmin(0.0f),
  itsNeighUpdFac(0.7), itsProbe(-1, -1), itsSLfac(1.0), itsSSfac(0.1),
  itsSFSfac(1.0),itsSFPfac(1.0), itsDescrName("blank"),itsTagName("blank"),
  itsCounter(0)
{}

// ######################################################################
template <class T>
void SurpriseMapFFT<T>::init(const uint qlen, const double updatefac,
                          const double neighupdatefac,
                          const double sampleval, const double samplevar,
                          const float neighsigma, const float locsigma,
                          const Point2D<int>& probe, const double slfac,
                          const double ssfac, const double sfsfac,
                          const double sfpfac,
                          const std::string& descrName,
                          const std::string& tagName)
{
  itsQlen = qlen;
  itsModels.clear();
  for(uint j = 0; j < itsFFTPModels.size(); j ++)
  {
    itsFFTSModels[j].clear();
    itsFFTPModels[j].clear();
  }
  itsInitialModel.init(updatefac, sampleval, samplevar);
  itsNeighSigma = neighsigma; itsLocSigma = locsigma;
  itsNweights.freeMem();
  itsNeighUpdFac = neighupdatefac;
  itsProbe = probe;
  itsVariance = samplevar;
  itsUpdatefac = updatefac;
  itsSLfac = slfac; itsSSfac = ssfac; itsSFSfac = sfsfac; itsSFPfac = sfpfac;
  itsDescrName = descrName;
  itsTagName = tagName;
  const int N = FFT_TIME_SLICE;
  LINFO("NEW FFT IN");
  in  = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * N));
  LINFO("NEW FFT OUT");
  out = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * N));
  //p   = fftw_plan_dft_1d(FFT_TIME_SLICE, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
  LINFO("NEW FFT PLAN");
  p   = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_PATIENT);
  LINFO("DONE");
  itsCounter = 0;
}

// ######################################################################
template <class T>
SurpriseMapFFT<T>::~SurpriseMapFFT()
{}

// ######################################################################
template <class T>
void SurpriseMapFFT<T>::reset()
{

  for(uint j = 0; j < itsFFTSModels.size(); j ++)
  {
    itsFFTSModels[j].reset();
    itsFFTPModels[j].reset();
  }

  LINFO("DESTROY FFT PLAN");
  fftw_destroy_plan(p);
  fftw_free(in);
  fftw_free(out);
  LINFO("DONE");
}

// ######################################################################
template <class T>
Image<double> SurpriseMapFFT<T>::surprise(const SurpriseImage<T>& sample,
                                          const Image<double>& inputI,
                                          const Image<double>& var)
{
  // set up a stack STL deque of to contain all images over time
  // for use in fft
  itsVarianceImage = var;
  if(fftStack.size() != FFT_TIME_SLICE)
  {
    fftStackCount = 0;
    fftStackReady = false;
    // reduce the number of FFT models by octave sized bins
    fftBins       = (unsigned char)(floor(log(FFT_TIME_SLICE/2)/log(2)));
    unsigned char tempsize = 1;
    // create octive sized bins and store thier size;
    for(unsigned char i = 0; i < fftBins; i++)
    {
      fftBinSize.push_back(tempsize);
      tempsize = tempsize * 2;
    }
    Image<double> temp;
    fftStack.resize(FFT_TIME_SLICE,temp);
  }
  // copy the sample onto the deque
  fftStack.pop_front();
  fftStack.push_back(inputI);


  // is it the first time we are called? if so, we need to setup our
  // size and reset our neighborhood cache:
  if (itsModels.empty())
  {
    // resize and reset our queue of models:
    SurpriseImage<T> models(sample.getDims()); // NOTE: uninitialized models
    models.clear(itsInitialModel);
    models.reset();
    itsFFTSModels.clear();
    itsFFTPModels.clear();
    itsModels.clear();
    for (uint i = 0; i < fftBins; i ++)
    {
      itsFFTSModels.push_back(models);
      itsFFTPModels.push_back(models);
      itsModels.clear();
    }

    // compute our Difference-of-Gaussians mask of weights:
    const int w = sample.getWidth(), h = sample.getHeight();
    const float sigma = itsNeighSigma * float(std::max(w, h));
    Dims d(w * 2 + 1, h * 2 + 1); Point2D<int> p(w, h);
    itsNweights = gaussianBlob<float>(d, p, sigma, sigma);
    itsNweights -= gaussianBlob<float>(d, p, itsLocSigma,
                                       itsLocSigma) *
      (itsLocSigma*itsLocSigma / (sigma * sigma) * 1.5f);

    inplaceRectify(itsNweights);  // eliminate negative values

    float mi, ma; getMinMax(itsNweights, mi, ma);
      itsNWmin = 0.01f * ma;
  }
  else if (itsModels[0].isSameSize(sample) == false)
    LFATAL("Inconsistent input size!");
  else if (itsFFTSModels[0].isSameSize(sample) == false)
    LFATAL("Inconsistent FFT signal mag input size!");
  else if (itsFFTPModels[0].isSameSize(sample) == false)
    LFATAL("Inconsistent FFT phase input size!");
  // each model feeds into the next one. The first (fastest) model
  // receives the sample from the image as input, and is updated. The
  // updated model then serves as input to the next slower model, and
  // so on. Total surprise is the product from all models:
  SurpriseImage<T> input(sample);
  Image<double> s;

  Image<float> locstot(input.getDims(), ZEROS);
  std::vector<Image<double> > sfs;
  std::vector<Image<double> > sfp;

  if(fftStackReady == true)
  {
    //float mi, ma;
    Image<double> temp;
    setFFTModels(input, itsNweights, itsNWmin, sfs, sfp);
    /*
    for(uint j = 0; j < itsFFTSModels.size(); j++)
    {
      //LINFO("FFT %d",j);
      //temp = itsFFTSModels[i][j].surprise(input);
      //LINFO("NEW S ****************************** %d %d",i-1,j);
      itsFFTSModels[j].neighborhoods(itsFFTSModels[j], itsNweights, itsNWmin);
      if (itsNeighUpdFac != 0.0) // use different update fac for neighs?
        itsFFTSModels[j].resetUpdFac(itsNeighUpdFac);
      // higher updfac -> stronger popout
      temp = itsFFTSModels[j].surprise(itsFFTSModels[j]);
      sfs.push_back(temp);
    }
    */
    /*
    //itsFFTSModels[1].neighborhoods(itsFFTSModels[1], itsNweights, itsNWmin);
    if (itsNeighUpdFac != 0.0) // use different update fac for neighs?
      itsFFTSModels[1].resetUpdFac(itsNeighUpdFac);
    // higher updfac -> stronger popout
    temp = itsFFTSModels[1].surprise(itsFFTSModels[0]);
    sfs.push_back(temp);
    */
    // the total surprise is roughly the pointwise product between
    // spatial and temporal surprises, for the current model, with a
    // small pedestal of spatial surprise to account for tonic
    // responses to static stimuli:

    Image<double> stot(sfs[0].getDims(), ZEROS);
    float mi, ma;
    Image<float> locstot(sfs[0].getDims(), ZEROS);
    float mi1, ma1;
    Image<float> lsf(sfs[0].getDims(), ZEROS);
    if (itsSFSfac)
    {
      if (itsSFSfac != 1.0)
      {
        for(uint j = 0; j < sfs.size() - 2; j++)
        {
          //LINFO("AA %f %d",itsSFSfac,sfs.size());
          stot += (sfs[j] * itsSFSfac)/sfs.size();
          locstot = stot;
          lsf = sfs[j];
          getMinMax(locstot, mi, ma);
          getMinMax(lsf, mi1, ma1);
          Image<byte> foo = sfs[j];
          //Raster::WriteGray(foo,sformat("Sout.%d.%d.pgm",i,j));
          //LINFO("FS1 min %f max %f",mi,ma);
          //LINFO("FS1  min %f max %f",mi1,ma1);
        }
      }
      else
      {
        for(uint j = 0; j < sfs.size() - 2; j++)
        {
          //LINFO("BB %f %d",itsSFSfac,sfs.size());
          stot += sfs[j]/sfs.size();
          locstot = stot;
          lsf = sfs[j];
          getMinMax(locstot, mi, ma);
          getMinMax(lsf, mi1, ma1);
          Image<byte> foo = sfs[j];
          //Raster::WriteGray(foo,sformat("Sout.%d.%d.pgm",i,j));
          //LINFO("FS2 min %f max %f",mi,ma);
          //LINFO("FS2  min %f max %f",mi1,ma1);
        }
      }
    }

    // total surprise combines multiplicatively across models with
    // different time scales:
    s = stot;
    locstot = s;
    getMinMax(locstot, mi, ma);
    //LINFO("S1 min %f max %f",mi,ma);


    // the results here should always be positive but sometimes turn
    // negative due to rounding errors in the surprise computation.
    // Let's clamp just to be sure, otherwise we'll end up with some
    // NaNs at later stages:
    inplaceRectify(s);

    // calm down total surprise:
    //s = toPower(s, 1.0 / (3.0*double(itsFFTSModels.size())));
    //s = s*255.0F;
    locstot = s;
    getMinMax(locstot, mi, ma);
    //LINFO("S2 min %f max %f",mi,ma);
  }
  else
  {
    s = locstot;
  }
  itsCounter++;

  //LINFO("S3 min %f max %f",mi,ma);

  // Do we have a full set of frames for fft?
  if(fftStackCount < FFT_TIME_SLICE)
    fftStackCount++;
  else
    fftStackReady = true;


  // return total surprise:
  return s;
}

// ######################################################################
template <class T>
const SurpriseImage<T>& SurpriseMapFFT<T>::getSurpriseImage(const
                                                         uint index) const
{
  ASSERT(index < itsModels.size());
  return itsModels[index];
}

// ######################################################################
template <class T>
const SurpriseImage<T>& SurpriseMapFFT<T>::getSurpriseImageSFFT(
                                        const uint i) const
{
  ASSERT(i < itsFFTSModels.size());
  return itsFFTSModels[i];
}

// ######################################################################
template <class T>
const SurpriseImage<T>& SurpriseMapFFT<T>::getSurpriseImagePFFT(
                                        const uint i) const
{
  ASSERT(i < itsFFTPModels.size());
  return itsFFTPModels[i];
}
// ######################################################################
template <class T>
void SurpriseMapFFT<T>::setFFTModels(const SurpriseImage<T>& models,
                                     const Image<float>& weights,
                                     const float wmin,
                                     std::vector<Image<double> >& sfs,
                                     std::vector<Image<double> >& sfp)
{
#ifndef HAVE_FFTW3_H
  LFATAL("this program requires fftw3, "
         "but <fftw3.h> was not found during the configure process");
#else
  // initialize FFT things
  const int    N               = FFT_TIME_SLICE;
  const double veryBigNumber   = pow(10,10);
  const double verySmallNumber = 0.0000000000001F;

  //in  = (fftw_complex*)(fftw_malloc(sizeof(fftw_complex) * N));
  //out = (fftw_complex*)(fftw_malloc(sizeof(fftw_complex) * N));

  //fftw_print_plan(p);
  //std::cerr << "\n";
  // initialize holder images to put FFT results into
  Image<double> SSpaces,  PSpaces;
  SSpaces.resize(itsFFTSModels[0].getWidth(),
                 itsFFTSModels[0].getHeight());
  PSpaces.resize(itsFFTSModels[0].getWidth(),
                 itsFFTSModels[0].getHeight());

  Image<double>::iterator iSSpace = SSpaces.beginw();
  Image<double>::iterator iPSpace = PSpaces.beginw();
  const unsigned int tpixels = itsFFTSModels[0].getWidth() *
                               itsFFTSModels[0].getHeight();
  for(unsigned int i = 0; i < tpixels ; i++ , ++iSSpace, ++iPSpace)
  {
    *iSSpace = 0; *iPSpace = 0;
  }

  Image<float> temps;

  /*
  float mis, mas;
  std::deque<Image<double> >::iterator ifftStack = fftStack.begin;

  for(int k = 0; k < N; k++, ++ifftStack)
  {
    temps = ifftStack;
    getMinMax(temps, mis, mas);
    //LINFO("DEQUE %d min %f max %f",k,mis,mas);
  }
  */
  // stack holder images into a vector
  std::vector<Image<double> > VSSpaces(itsFFTSModels.size(),SSpaces);
  std::vector<Image<double> > VPSpaces(itsFFTSModels.size(),PSpaces);

  for(int i = 0; i < itsFFTSModels[0].getWidth(); i++)
  {
    for(int j = 0; j < itsFFTSModels[0].getHeight(); j++)
    {
      for(unsigned int k = 0; k < (unsigned)N; k++)
      {
        in[k][1] = fftStack[k].getVal(i,j);
        in[k][0] = fftStack[k].getVal(i,j);
        //std::cerr << "INPUT: " << k << " " << in[k][1] << "\n";
      }
      //LINFO("DOING PLAN");
      fftw_execute(p);
      unsigned char binSize    = 1;
      unsigned char binCount   = 1;
      unsigned char bin        = 0;
      float         val        = 0.0F;
      int           k          = 0;
      //LINFO("Process FFT");
      while(bin != (unsigned char)itsFFTSModels.size())
      {
        // fftw returns a very large number if there is no response on
        // some given frequency what so ever. For instance, a constant
        // function yields a dirac delta and the rest are all large
        // numbers
        //LINFO("%f %f",(float)out[k][0],(float)out[k][1]);

        if((fabs(out[k][1]) < veryBigNumber) &&
           (fabs(out[k][0]) < veryBigNumber))
        {
          if(out[k][0] != 0)
            val = VPSpaces[bin].getVal(i,j) + fabs(atan(out[k][1]/out[k][0]));
          else
            val = 0.0F;
        }
        else
        {
          // compute phase from FFT
          val = verySmallNumber;
        }
        VPSpaces[bin].setVal(i,j,val);

        if(fabs(out[k][1]) < veryBigNumber)
          out[k][1] = out[k][1];
        else
          out[k][1] = verySmallNumber;

        if(fabs(out[k][0]) < veryBigNumber)
          out[k][0] = out[k][0];
        else
          out[k][0] = verySmallNumber;

        // compute sprectral mag. from FFT
        val = VSSpaces[bin].getVal(i,j)
          + sqrt(pow(out[k][0],2) + pow(out[k][1],2));
        //std::cerr << val << " S " << out[k][0] << " " << out[k][1] << "\n";
        VSSpaces[bin].setVal(i,j,val);
        // reduce size and noise by bin-ing by octave
        if(binCount == binSize)
        {
          // we normalize specta by 1/(N * sqrt(2)) to give a spectra
          // in the same range of values as a normal pixel
          val = (VSSpaces[bin].getVal(i,j)/binSize) * 1/(N * sqrt(2));
          VSSpaces[bin].setVal(i,j,val);
          val = (VPSpaces[bin].getVal(i,j)/binSize) * (255.0F/(3.14159F/2.0F));
          VPSpaces[bin].setVal(i,j,val);
          binSize  = binSize * 2;
          binCount = 1;
          bin++;
        }
        else
        {
          binCount++;
        }
        k++;
      }
    }
  }


  // get our stack of fft images into a start model then
  // surprise the current on-going model with it.
  Image<double> temp;
  Image<float>  holder;
  float mi, ma;
  for(int bin = 0; bin < (unsigned char)itsFFTSModels.size(); bin++)
  {
    //LINFO("%d",bin);
    Image<double> invar(VSSpaces[bin].getDims(), NO_INIT);
    //invar.clear(itsVariance);
    //std::cerr << itsUpdatefac << " " << itsVariance << "\n";
    holder = VSSpaces[bin];
    getMinMax(holder, mi, ma);
    //LINFO("BIN S %d min %f max %f",bin,mi,ma);
    holder = VPSpaces[bin];
    getMinMax(holder, mi, ma);
    //LINFO("BIN P %d min %f max %f",bin,mi,ma);

    Image<float> outImageO = VSSpaces[bin];
    Image<PixRGB<float> > outImageF;
    outImageO = rescale(outImageO,200,(int)round(200.0F*
                        ((float)outImageO.getHeight()/
                         (float)outImageO.getWidth())));
    outImageF = normalizeWithScale(outImageO,0,255.0F,255.0F,1,3);
    Image<PixRGB<byte> > outImage = outImageF;
    Raster::WriteRGB(outImage,sformat("out.fft.spec.frame%d.%s.%d.%dx%d.png",
                                          itsCounter,
                                          itsTagName.c_str(),
                                          bin,
                                          VSSpaces[bin].getWidth(),
                                          VSSpaces[bin].getHeight()
                                          ));


    Image<float> outImageO2 = VPSpaces[bin];
    Image<PixRGB<float> > outImageF2;
    outImageO2 = rescale(outImageO2,200,(int)round(200.0F*
                       ((float)outImageO2.getHeight()/
                       (float)outImageO2.getWidth())));
    outImageF2 = normalizeWithScale(outImageO2,0,255.0F,255.0F,1,3);
    outImage  = outImageF2;
    Raster::WriteRGB(outImage,sformat("out.fft.phas.frame%d.%s.%d.%dx%d.png",
                                          itsCounter,
                                          itsTagName.c_str(),
                                          bin,
                                          VPSpaces[bin].getWidth(),
                                          VPSpaces[bin].getHeight()
                                          ));


    SurpriseImage<T> SSSpaces(itsNeighUpdFac,
                              VSSpaces[bin],itsVarianceImage);
    SurpriseImage<T> SPSpaces(itsNeighUpdFac,
                              VPSpaces[bin],itsVarianceImage);
    //LINFO("A");
    //LINFO("FFT S ****************************** bin %d",bin);
    //SSSpaces.neighborhoods(SSSpaces, itsNweights, itsNWmin);
    //if (itsNeighUpdFac != 0.0) // use different update fac for neighs?
    //  SSSpaces.resetUpdFac(itsNeighUpdFac); // higher updfac -> stronger popout
    //Image<double> temp2(SSSpaces);
    temp = itsFFTSModels[bin].surprise(SSSpaces);
    //SSSpaces.neighborhoods(temp, itsNweights, itsNWmin);
    //temp = SSSpaces;
    sfs.push_back(temp);
    //LINFO("B");
    //LINFO("FFT P ****************************** bin %d",bin);
    temp = itsFFTPModels[bin].surprise(SPSpaces);
    sfp.push_back(temp);
  }
  //LINFO("DONE");
#endif
}






// ######################################################################
// explicit instantiations:
template class SurpriseMapFFT<SurpriseModelSG>;
template class SurpriseMapFFT<SurpriseModelSP>;
template class SurpriseMapFFT<SurpriseModelOD>;

#endif // HAVE_FFTW3_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
