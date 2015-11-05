/*!@file Channels/SpectralResidualChannel.H "Spectral Residual" channel based on Hou&Zhang (CVPR 2007) */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SpectralResidualChannel.H $
// $Id: SpectralResidualChannel.H 12820 2010-02-11 05:44:51Z itti $
//

#ifndef CHANNELS_SPECTRALRESIDUALCHANNEL_H_DEFINED
#define CHANNELS_SPECTRALRESIDUALCHANNEL_H_DEFINED

#include "Channels/ChannelBase.H"
#include "Image/ResizeSpec.H"
#include "Image/fancynorm.H" // for MaxNormType
#include "rutz/shared_ptr.h"

#include <pthread.h>
#include <vector>

template <class T> class FourierEngine;
template <class T> class FourierInvEngine;

extern const ModelOptionDef OPT_SpectralResidualChannelSaveOutputMap;
extern const ModelOptionDef OPT_SpectralResidualChannelSaveExtraOutput;
extern const ModelOptionDef OPT_SpectralResidualChannelResizeSpec;
extern const ModelOptionDef OPT_SpectralResidualChannelSpectralBlur;
extern const ModelOptionDef OPT_SpectralResidualChannelOutputBlur;
extern const ModelOptionDef OPT_SpectralResidualChannelOutputBlurFactor;
extern const ModelOptionDef OPT_SpectralResidualChannelDownSizeFilterWidth;
extern const ModelOptionDef OPT_SpectralResidualChannelAttenuationWidth;
extern const ModelOptionDef OPT_SpectralResidualChannelOutputResize;
extern const ModelOptionDef OPT_SpectralResidualChannelOutputResizeSpec;
extern const ModelOptionDef OPT_SpectralResidualChannelHiboostBypass;
extern const ModelOptionDef OPT_SpectralResidualChannelGradientBypass;

/// "Spectral Residual" channel based on Hou&Zhang (CVPR 2007)
/** See also http://bcmi.sjtu.edu.cn/~houxiaodi */
class SpectralResidualChannel : public ChannelBase
{
public:
  /// This helps share downsizing calculations across multiple subchannels in a MultiSpectralResidualChannel
  class Downsizer
  {
  public:
    Downsizer();
    ~Downsizer();

    Image<float> getDownsized(const Image<float>& x, int filtwidth,
                              const Dims& newdims);

  private:
    Downsizer(const Downsizer&); // not implemented
    Downsizer& operator=(const Downsizer&); // not implemented

    pthread_mutex_t itsMutex;
    Image<float> itsInput;
    int itsFilterWidth;
    std::vector<Image<float> > itsPyr;
  };

  /// Default constructor
  SpectralResidualChannel(OptionManager& mgr,
                          const std::string& descrName = "Spectral Residual",
                          const std::string& tagName = "SRS");

  /// Virtual destructor
  virtual ~SpectralResidualChannel();

  /// Calls visitChannelBase() on the ChannelVisitor.
  virtual void accept(ChannelVisitor& v);

  /// Returns true
  virtual bool isHomogeneous() const;

  /// Read params from the ParamMap.
  virtual void readFrom(const ParamMap& pmap);

  /// Write params to the ParamMap.
  virtual void writeTo(ParamMap& pmap) const;

  /// Returns true if we've gotten any input
  virtual bool outputAvailable() const;

  /// Return the dimensions of the output image.
  virtual Dims getMapDims() const;

  /// Return the number of feature maps represented by this channel.
  virtual uint numSubmaps() const;

  /// Get the feature map at the given index.
  virtual Image<float> getSubmap(const uint index) const;

  /// Get the name/description of the feature map at the given index.
  virtual std::string getSubmapName(const uint index) const;

  /// Get the name/description of the feature map at the given index, short version.
  virtual std::string getSubmapNameShort(const uint index) const;

  /// Not implemented
  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const;

  /// Not implemented
  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const;

  /// Return the spectral residual saliency map
  virtual Image<float> getOutput();

  /// Save our various maps using a FrameOstream
  virtual void saveResults(const nub::ref<FrameOstream>& ofs);

  /// Subclasses override this to clear any caches they may maintain.
  virtual void killCaches();

  ResizeSpec getResizeSpec() const { return itsResizeSpec.getVal(); }
  void setResizeSpec(const ResizeSpec& s) { itsResizeSpec.setVal(s); }

  float getOutputBlur() const { return itsOutputBlur.getVal(); }
  void setOutputBlur(float f) { itsOutputBlur.setVal(f); }

  ResizeSpec getOutputResizeSpec() const
  { return itsOutputResizeSpec.getVal(); }

  void setOutputResizeSpec(bool on, const ResizeSpec& s)
  {
    itsDoResizeOutput.setVal(on);
    itsOutputResizeSpec.setVal(s);
  }

  rutz::shared_ptr<Downsizer> getDownsizer() const
  { return itsDownsizer; }

  void setDownsizer(rutz::shared_ptr<Downsizer> d)
  { itsDownsizer = d; }

protected:
  virtual void doInput(const InputFrame& inframe);

private:
  OModelParam<bool> itsSaveOutput;
  OModelParam<bool> itsSaveExtraOutput;
  OModelParam<ResizeSpec> itsResizeSpec;
  OModelParam<uint> itsSpectralBlur;
  OModelParam<float> itsOutputBlur;
  OModelParam<double> itsOutputBlurFactor;
  OModelParam<int> itsDownSizeFilterWidth;
  OModelParam<double> itsAttenuationWidth;
  OModelParam<bool> itsDoResizeOutput;
  OModelParam<ResizeSpec> itsOutputResizeSpec;
  OModelParam<MaxNormType> itsNormType;
  OModelParam<float> itsOutputRangeMin;
  OModelParam<float> itsOutputRangeMax;
  OModelParam<bool> itsHiboostBypass;
  OModelParam<bool> itsGradientBypass;

  rutz::shared_ptr<Downsizer> itsDownsizer;

  FourierEngine<double>* itsFFT;
  FourierInvEngine<double>* itsIFFT;

  Image<float> itsInput;
  Image<float> itsOutput;

  Image<float> itsRescaledInput;
  Image<float> itsLogMagnitude;
  Image<double> itsPhase;
  Image<float> itsSpectralResidual;
  Image<float> itsProtoSaliencyMap;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_SPECTRALRESIDUALCHANNEL_H_DEFINED
