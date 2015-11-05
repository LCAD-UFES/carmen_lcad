/*!@file Devices/VectorHistField.H Vector field class to compute vector
histogram field */

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
// Primary maintainer for this file: Farhan Baluch
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/VectorHistField.H $
// $Id: VectorHistField.H 15272 2012-05-07 20:40:14Z kai $
//

#ifndef VectorHistField_H_DEFINED
#define VectorHistField_H_DEFINED

#include "Image/VectorField.H"
using namespace geom;

class VectorHistField : public VectorField
{
public:
  //! Default constructor; see VectorField.H
  VectorHistField(OptionManager& mgr,
      const std::string& descrName = "VectorHistField",
              const std::string& tagName = "VectorHistField",
              const int width=300, const int height=300);

  //! Destructor
  ~VectorHistField();

  /*!updateField() -- update the current field based on some sensor data
    of type Point2D<int> a; where a.i = distance a.j=angle of obstacle
    since this can be pretty implementation specific we keep this virtual
    so that different algorithms can implement their own implementation of
    this method.
  */
  Image<PixRGB<byte> > updateField(std::vector<Point2D<float> > sensorData,
                                   Point2D<int>robotPos, float robotOrient,
                                   float robotDist,Point2D<int> goal,
                                   Image<geom::vec2f> itsObsTemplate);

  //!obstacleTemplate -- creates an obstacle template field
  Image<geom::vec2f> obstacleTemplate(float sigma,float amp);

  //!shiftField -- shifts the field depending on how the robot moved
  void shiftField(int dist, int ang);

	void setHistRatio(float hist);
	void setGoalRatio(float goal);

protected:
  int itsDist;   //keep track of dist since last field shift
  int itsAng;    //keep track of angle since last field shift
  
	float itsHistRatio; //the mix ratio of history and current vector
	float itsGoalRatio; //the mix ratio of goal and current vector
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
