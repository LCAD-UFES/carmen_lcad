/*!@file Channels/MotionSpatioTemporalChannel.H */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MotionSpatioTemporalChannel.H $
// $Id: $
//

#ifndef MOTIONSPATIOTEMPORALCHANNEL_H_DEFINED
#define MOTIONSPATIOTEMPORALCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Image/PyramidTypes.H"
#include "rutz/shared_ptr.h"

#include "Robots/Beobot2/Navigation/FOE_Navigation/MiddleTemporal.H"
#include "Robots/Beobot2/Navigation/FOE_Navigation/SpatioTemporalEnergy.H"
#include "Robots/Beobot2/Navigation/FOE_Navigation/FoeDetector.H"

#include "Channels/DirectionSpatioTemporalChannel.H"

class DirectionSpatioTemporalChannel;

// ######################################################################
//! A composite channel containing a set of direction spatio temporal channels
class MotionSpatioTemporalChannel : public ComplexChannel
{
public:
  //! Construct
  MotionSpatioTemporalChannel(OptionManager& mgr);

  //! destructor
  virtual ~MotionSpatioTemporalChannel();

  //! returns a specific DirectionSpatioTemporalChannel
  virtual DirectionSpatioTemporalChannel& dirChan(const uint idx) const;

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Form output by combining output of subchannels.
  //! Sums the outputs of the subchannels and then optionally
  //! maxNormalize()'s the result
  virtual Image<float> getOutput();

protected:
  //! type of pyramid to use in our DirectionChannel subcomponents
  NModelParam<PyramidType> itsPyrType;

  //! number of DirectionSpatioTemporalChannel subcomponents
  OModelParam<uint> itsNumDirs;
  OModelParam<uint> itsNumSpeeds;

  //! its Medial Temporal module
  rutz::shared_ptr<MiddleTemporal> itsMT;

  //! its Focus of Expansion detector
  nub::ref<FoeDetector> itsFoeDetector;

  //! (re-)build our subchannels
  virtual void buildSubChans();

  //! MotionSpatioTemporalChannel only required luminosity input
  virtual void doInput(const InputFrame& inframe);
  void computeConspicuityMap();

  Image<float> getV1ObjectMotionMap();
  Image<float> getMTObjectMotionMap();

  Image<float> downSizeMax(Image<float> img, uint scale);

  //! the various directional pyrbuilders
  std::vector<std::vector<nub::ref<DirectionSpatioTemporalChannel> > >
  itsDirectionSpatioTemporalChannels; 

  //! raw motion energy for each direction
  std::vector<std::vector<ImageSet<float> > > itsRawSpatioTemporalEnergy;
  std::vector<ImageSet<float> > itsSpatioTemporalEnergy;
  std::vector<ImageSet<float> > itsSpatioTemporalEnergyOptimalShift;

  Image<byte> itsCurrentImage;
  Image<float> itsConspicuityMap;

  int itsCurrentFoeMapIndex;
 
  rutz::shared_ptr<XWinManaged> itsWin;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MOTIONSPATIOTEMPORALCHANNEL_H_DEFINED
