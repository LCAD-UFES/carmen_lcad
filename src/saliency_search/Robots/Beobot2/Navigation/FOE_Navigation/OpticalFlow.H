/*!@file Beobot/Landmark.H Landmark class for localization */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/OpticalFlow.H  $
// $Id: $
//

#ifndef MOTION_OPTICALFLOW_DEFINED
#define MOTION_OPTICALFLOW_DEFINED

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"

#include "Robots/LoBot/util/triple.hh"
#include "rutz/shared_ptr.h"

// ######################################################################
//! a flow vector struct to describe 1 flow vector
//! it also provide motion/flow strength value as well
struct FlowVector
{
  FlowVector() { };

  FlowVector(const Point2D<float> inP1,
             const Point2D<float> inP2,
             const float inVal) :
    p1(inP1),
    p2(inP2),
    val(inVal)
  {
    angle = atan2(double(p2.j - p1.j), double(p2.i - p1.i))/M_PI * 180;
    angle = fmod(angle +360.0, 360.0);

    mag   = sqrt( pow(p2.j - p1.j, 2.0) + pow(p2.i - p1.i, 2.0) );
  }

  FlowVector(const Point2D<float> inP1,
             const float inAngle,
             const float inMag,
             const float inVal) :
    p1(inP1),
    angle(inAngle),
    mag(inMag),
    val(inVal)
  {
    float ang = angle/180.0F * M_PI;

    float i = p1.i + (mag * cos(ang));
    float j = p1.j + (mag * sin(ang));
    p2 = Point2D<float>(i,j);
  }

  Point2D<float> p1;
  Point2D<float> p2;

  float angle; // should be in degrees
  float mag;
  float val;
};

//! an Optical flow class that can return both:
//!   a vector of FlowVector: for sparse flow 
//!     (note: also feed in image size) 
//!   or a triple of images for direction, length, and strength
class OpticalFlow
{
public:
  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! Constructor taking in sparse flow vector
  /*! @param flow the vector of flow vectors in the image
      @param dims size of the image
  */
  OpticalFlow(std::vector<rutz::shared_ptr<FlowVector> > flowVectors, 
	      Dims dims);

  //! Constructor taking in dense flow field
  /*! @param flow field of the image
  */
  OpticalFlow
  (lobot::triple<Image<float>, Image<float>, Image<float> > flowField);

  //! Destructor
  ~OpticalFlow();

  //@}

  // ######################################################################
  //! @name Access functions
  //@{

  //! get the flow vectors
  std::vector<rutz::shared_ptr<FlowVector> > getFlowVectors();

  //! get the flow vector locations
  std::vector<Point2D<float> > getFlowLocations();
  
  //! get the image dimensions
  Dims getImageDims();

  //! get the flow field
  lobot::triple<Image<float>,Image<float>,Image<float> > 
  getFlowField();

  //! get the direction field
  Image<float> getDirectionField();

  //! get the vector length field
  Image<float> getVectorLengthField();

  //! get the strength field
  Image<float> getFlowStrengthField();

  //@}

 private:

  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! compute functions to compute (sparse) flow field
  //! given flow vectors
  void computeFlowField();

  //! compute functions to compute flow vectors
  //! given a (what should be a sparse) flow field 
  void computeFlowVectors();

  //! put the flow locations in one vector
  void computeFlowLocations();

  //@}  

  //! flag to check if the flow field or flow vector is computed
  bool itsFlowFieldComputed;
  bool itsFlowVectorsComputed;
  bool itsFlowLocationsComputed;

  std::vector<rutz::shared_ptr<FlowVector> > itsFlowVectors;

  //! dimension of the image of the flow
  Dims itsImageDims;

  //! its flow locations
  std::vector<Point2D<float> > itsFlowLocations;

  //! Flow field represented by a triple of images:
  //! first:  direction
  //! second: length
  //! third:  flow strength
  //! usually for dense maps
  lobot::triple<Image<float>,Image<float>,Image<float> > itsFlowField;
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
