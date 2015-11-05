/*!@file Channels/DisparityChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DisparityChannel.H $
// $Id: DisparityChannel.H 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef DISPARITYCHANNEL_H_DEFINED
#define DISPARITYCHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"

// ######################################################################
//! An disparity channel
class DisparityChannel : public SingleChannel
{
public:

  //! Construct with standard params plus a disparity index for this object.
  /*! @param dIndex is used for various indexing such as with ParamMap
    and Jet interfacing.
    @param ang is the preferred orientation disparity for this channel, in
      degrees (0..180).
    @param pha is the preferred phase disparity for this channel, in
      degrees (0..360). */
  DisparityChannel(OptionManager& mgr, const uint dIndex, const double ang,
                   const double pha);

  //! Destructor
  virtual ~DisparityChannel();

  //! Get the preferred orientation (in degrees) of this channel.
  double angle() const;

  //! Get the preferred phase (in degrees) of this channel.
  double phase() const;

  //! set the raw filtered images from stereo channel to avoid redundancy
  void setRawFilteredImages(Image<float> ***fImgLE, Image<float> ***fImgRI);

  //! access to the raw filtered images for easy viewing
  void getRawFilteredImages(Image<float> ****fImgLE,Image<float> ****fImgRI);

  //! access to the disparity even before stored in the queue (SingleChannel)
  void getDispMap(ImageSet<float> *dMap);

  //!  normalize the disparity map to [0, 1]
  void normalizeDispMap(ImageSet<float> tDispMap, int nChan);

  //! to get the dimensions of the raw filtered images from stereo channel
  void setNumPhase(int n);
  void setNumTheta(int t);

  //! perform the disparity calculation
  void doInput(const InputFrame& inframe);

  void storePyramid(const SimTime& t);

  //! overload start1()
  void start1();

protected:
  NModelParam<uint>   itsIndex;
  NModelParam<double> itsOrientation;
  NModelParam<double> itsPhase;

private:

  Image<float> ***fImgL;
  Image<float> ***fImgR;

  ImageSet<float> dispMap;

  uint depth, nPhase, nTheta;

  const static float period   =  5.0;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DISPARITYCHANNEL_H_DEFINED
