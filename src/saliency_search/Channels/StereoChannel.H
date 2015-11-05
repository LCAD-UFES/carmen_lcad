/*!@file Channels/StereoChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/StereoChannel.H $
// $Id: StereoChannel.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef STEREOCHANNEL_H_DEFINED
#define STEREOCHANNEL_H_DEFINED

#include "Channels/ComplexChannel.H"
#include "Image/ImageSet.H"
#include "Image/PyramidTypes.H"

class DisparityChannel;

// ######################################################################
//! A composite channel containing a set of Disparity channels
class StereoChannel : public ComplexChannel
{
public:
  //! Construct
  StereoChannel(OptionManager& mgr);

  //! destructor
  virtual ~StereoChannel();

  //! Wraps ComplexChannel's subChan() and casts the result to DisparityChannel
  virtual DisparityChannel& dispChan(const uint idO,
                                          const uint idP) const;

  //! Overload so that we can reconfigure when our params get changed
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! set the second image to complete stereo data
  void setSecondImage(const Image<float>* bwimg);

  //! access method for easy viewing
  void getRawFilteredImages(Image<float> ****fImgLE,Image<float> ****fImgRI);

protected:
  //! type of pyramid to use in our DisparityChannel subcomponents
  NModelParam<PyramidType> itsPyrType;

  //! number of orientations of disparity subchannels
  OModelParam<uint> itsNumTheta;

  //! number of phases of each orientations of disparity subchannels
  NModelParam<uint> itsNumPhase;

  //! (re-)build our subchannels
  virtual void buildSubChans();

  //! StereoChannel only required luminosity input AT THIS POINT
  virtual void doInput(const InputFrame& inframe);

  //! overload start1()
  virtual void start1();

private:

  void applyGabor();
  void calcDispResponse();
  void selectMax();
  void createPhasedGaborPyramids();

  // depth of the feature map pyramid
  uint itsDepth;

  // captured color images
  Image< PixRGB<byte> > rgbImgL;
  Image< PixRGB<byte> > rgbImgR;

  // luminance values of the captured images
  Image<float> imgL;
  Image<float> imgR;

  // image pyramid for both images
  ImageSet<float> pyrL;
  ImageSet<float> pyrR;

  // TUNED PARAMETERS
  const static uint  nPhase   = 4;
  const static uint  nTheta   = 4;
  const static float dPhase   = 360.0/nPhase;
  const static float dTheta   = 180.0/nTheta;
  const static float stddev   =  1.4;
  const static float period   =  4.2;

  Image<float> ***fImgL;
  Image<float> ***fImgR;

};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // STEREOCHANNEL_H_DEFINED
