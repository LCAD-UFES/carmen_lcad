/*!@file Channels/SoxChannel.H Shortrange Orientation Interactions channel */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SoxChannel.H $
// $Id: SoxChannel.H 8831 2007-10-12 23:13:54Z rjpeters $
//

#ifndef SOXCHANNEL_H_DEFINED
#define SOXCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Channels/GaborChannel.H"
#include <vector>

//! A short-range orientation interactions channel.
/*! This class implements a pool of orientation-tuned units whose responses
    are shaped by divisive inhibition in proportion to population activity. */
class SoxChannel : public ComplexChannel
{
public:
  //! Construct
  SoxChannel(OptionManager& mgr);

  //! Virtual destructor.
  virtual ~SoxChannel();

  //! Get the number of scales used.
  uint numScales() const;

  //! Get the linear response of the units for a given orientation+scale.
  Image<float> getLinearResponse(int ori, int scl);

  //! Get the non-linear response of the units for a given orientation+scale.
  /*! This is (rougly speaking) given by a power of the linear response,
      divided by an inhibitory term that pools over all orientations and
      scales. */
  Image<float> getNonlinearResponse(int ori, int scl);

  //! Wraps ComplexChannel's subChan() and casts the result to GaborChannel.
  nub::ref<GaborChannel> gabor(const uint idx) const;

protected:
  OModelParam<uint> itsNumOrients;  //!< number of Gabor subchannels
  OModelParam<double> thetaPoolWidth;
  OModelParam<double> omegaPoolWidth;
  OModelParam<double> inhibExponent;
  OModelParam<double> excitExponent;
  OModelParam<double> semiSaturation;
  OModelParam<double> cutoff;

  //! (re-)build our subchannels
  void buildSubChans();

  //! SoxChannel requires only luminance input.
  virtual void doInput(const InputFrame& inframe);

  //! Combine all of the nonlinear responses.
  virtual Image<float> combineOutputs();

  //! Clear our cached inhibitory pool maps.
  virtual void killCaches();

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  // a private struct for caching stuff:
  struct CacheElem
  {
    CacheElem() : img(), ori(-1), scl(-1), exc_scl(-1)
    {}

    CacheElem(const Image<float>& i, int o, int s, int e) :
      img(i), ori(o), scl(s), exc_scl(e)
    {}

    Image<float> img;
    int ori;
    int scl;
    int exc_scl;
  };

  typedef std::vector<CacheElem> Cache;
  std::vector<Cache> inhibCaches;

  Image<float> getInhib(int ori, int scl, int exc_scl, const Dims& dims,
                        const Image<float>& linearResponse);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !SOXCHANNEL_H_DEFINED
