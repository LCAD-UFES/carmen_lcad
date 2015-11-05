/*!@file Robots/Beobot2/Navigation/FOE_Navigation/MotionOps.H
  various motion related functions. 
  For example: Lucas&Kanade, Horn&Schunck optical flow                  */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/MotionOps.H $
// $Id: $

// ######################################################################

#ifndef MOTIONOPS_H_DEFINED
#define MOTIONOPS_H_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Robots/Beobot2/Navigation/FOE_Navigation/OpticalFlowTypes.H"

#include "Robots/Beobot2/Navigation/FOE_Navigation/OpticalFlow.H"

// ######################################################################

// FIXXXXXX: To do:
//     Horn & Schunck implementation      
//

//! get clean optical flow using:
//! o(x,y) = 1/Z(x,y)*A(x,y)Trans + B(x,y)Rot
rutz::shared_ptr<OpticalFlow> getCleanOpticFlow(Image<byte> img);
void saveCleanOpticFlow(Image<byte> img);

//! get the Lucas Kanade optic flow for motion
//! from image1 to image2
rutz::shared_ptr<OpticalFlow> getLucasKanadeOpticFlow
(Image<byte> image1, Image<byte> image2);

//! draw the optic flow given a set of correspondences 
Image<PixRGB<byte> > drawOpticFlow
(Image<PixRGB<byte> > img, rutz::shared_ptr<OpticalFlow> flow);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !MOTIONOPS_H_DEFINED
