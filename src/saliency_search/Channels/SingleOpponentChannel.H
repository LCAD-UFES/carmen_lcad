/*!@file Channels/SingleOpponentChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SingleOpponentChannel.H $
// $Id: SingleOpponentChannel.H 10746 2009-02-03 07:09:00Z itti $
//

#ifndef CHANNELS_SINGLEOPPONENTCHANNEL_H_DEFINED
#define CHANNELS_SINGLEOPPONENTCHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"

class SingleOpponentChannel : public SingleChannel
{
public:
  //! Constructor. See ChannelBase.H
  /*! @param mgr our ModelManager (see ModelManager.H)
      @param descrName descriptive name for human usage
      @param tagName name for ParamMap usage
      @param vs The VisualFeature implemented by the channel
      @param pyr The type of pyramid that should be used. */
  SingleOpponentChannel(OptionManager& mgr,
                        const std::string& descrName,
                        const std::string& tagName,
                        const VisualFeature vs,
                        rutz::shared_ptr<PyrBuilder<float> > pyr);

  //! Reset SingleOpponentChannel
  virtual void reset1();

  //! Get the center/surround image for the given levels
  virtual Image<float> centerSurround(const uint cntr, const uint surr) const;

  //! Get the center/surround, split into positive and negative parts
  virtual void centerSurround(const uint cntr, const uint surr,
                              Image<float>& pos, Image<float>& neg) const;

  virtual void getFeatures(const Point2D<int>& locn,
                           std::vector<float>& mean) const;

  virtual void getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                std::vector<std::vector<float> > *mean,
                                int *count) const;

  //! Single-opponent SingleChannel requires center and surround pyramids
  void singleOpponentInput(const Dims& dims,
                           const ImageSet<float>& centerPyr,
                           const ImageSet<float>& surroundPyr,
                           const SimTime& t,
                           const Image<byte>& clipMask);

protected:
  //! Stores p as the channel's surround pyramid at time t
  void storePyramid2(const ImageSet<float>& p, const SimTime& t);

private:
  std::deque<TPyr> itsPq2; // temporal queue of surround pyramids
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_SINGLEOPPONENTCHANNEL_H_DEFINED
