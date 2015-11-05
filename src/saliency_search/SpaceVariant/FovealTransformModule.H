/*!@file SpaceVariant/FovealTransformModule.H a model component that embodies a foveal image transforms. */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/SpaceVariant/FovealTransformModule.H $

#ifndef FOVEALTRANSFORMMODULE_H_DEFINED
#define FOVEALTRANSFORMMODULE_H_DEFINED

#include "SpaceVariant/SpaceVariantModule.H"
#include "Psycho/PixPerDeg.H"

// ###################################################################### 
// a Model Component that embodies a foveal transform. See
// Image/SpaceVariantTransforms.H for details on the computation. 
// ######################################################################
class FovealTransformModule : public SpaceVariantModule
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  FovealTransformModule(OptionManager& mgr, const std::string& descrName = "FovealTransform Module",
                        const std::string& tagName = "FovealTransformModule");

  //! Destructor
  virtual ~FovealTransformModule();
  
  //! reset transforms to the desired input image size
  virtual void clear(const Dims& inp_dims);

  /*! separate the foveal and peripherial region  */
  template <class T_or_RGB>
  void getFoveaPeriphery(const Image<T_or_RGB>& ret_image, 
                         Image<T_or_RGB>& fovea, Image<T_or_RGB>& periphery);

  //!get the dims from the command line
  Dims getSVDims() const;
  
protected:
  OModelParam<std::string> itsSvScale;//scale type
  OModelParam<float> itsSvFovea; //fovea size
  OModelParam<float> itsSvSfac; //compression factor if scale is NONE
  OModelParam<float> itsSvBeta; //offset of log
  OModelParam<float> itsSvGain; //rf scaling factor
  OModelParam<float> itsSvExponent; //rf vs eccentricty steapnes
  OModelParam<float> itsSvOffset; //offset of rf size with eccentircity 
  OModelParam<float> itsFoveaCutoff;//value of du/dr to consider fovea
  OModelParam<Dims> itsSvDims; //dims of transform
  OModelParam<PixPerDeg> itsPPD;//pixels per degree

  OModelParam<LevelSpec> itsLevelSpec; //get size of sm for inverse mapping
};

#endif //FOVEALTRANSFORMMODULE
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
