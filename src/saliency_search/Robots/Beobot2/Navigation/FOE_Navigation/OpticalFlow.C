/*!@file Beobot/Landmark.C Landmark class for localization */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beobot/Landmark.C $
// $Id: Landmark.C 14116 2010-10-08 08:34:50Z siagian $
//

#include "Robots/Beobot2/Navigation/FOE_Navigation/OpticalFlow.H"

// ######################################################################
OpticalFlow::OpticalFlow
(std::vector<rutz::shared_ptr<FlowVector> > flowVectors, 
 Dims dims):
  itsFlowVectors(flowVectors),
  itsImageDims(dims)
{
  itsFlowFieldComputed   = false;
  itsFlowVectorsComputed = true;

  computeFlowLocations();
  itsFlowLocationsComputed = true;
}

// ######################################################################
OpticalFlow::OpticalFlow
(lobot::triple<Image<float>,Image<float>,Image<float> > flowField):
  itsFlowField(flowField)
{
  itsFlowFieldComputed   = true;
  itsFlowVectorsComputed = false;
  itsFlowLocationsComputed = false;

  itsImageDims = itsFlowField.first.getDims();
}

// ######################################################################
OpticalFlow::~OpticalFlow()
{ }

// ######################################################################
void OpticalFlow::computeFlowLocations()
{
  // must have flow vectors first
  if(!itsFlowVectorsComputed)
    {
      computeFlowVectors();
    }

  itsFlowLocations.clear();
  for(uint i = 0; i < itsFlowVectors.size(); i++)
    {
      itsFlowLocations.push_back(itsFlowVectors[i]->p1);
    }
  itsFlowLocationsComputed = true;
}

// ######################################################################
std::vector<Point2D<float> > OpticalFlow::getFlowLocations()
{
  // must have flow locations first
  if(!itsFlowLocationsComputed)
    {
      computeFlowLocations();
    }
  
  return itsFlowLocations;
}

// ######################################################################
std::vector<rutz::shared_ptr<FlowVector> > OpticalFlow::getFlowVectors()
{
  // compute the flow vectors from the flow field first
  if(!itsFlowVectorsComputed)
    {
      computeFlowVectors();
    }

  return itsFlowVectors;
}

// ######################################################################
Dims OpticalFlow::getImageDims()
{
  return itsImageDims;
}

// ######################################################################
void OpticalFlow::computeFlowVectors()
{      
  LFATAL("NEED DEBUGGING TO CONFIRM THAT IT WORKS - then delete this line");

  itsFlowVectors.clear();

  // get iterators of the to go through all the flow field
  Image<float> dir = itsFlowField.first;
  Image<float> len = itsFlowField.second;
  Image<float> val = itsFlowField.third;
  Image<float>::iterator dptr = dir.beginw(), stop = dir.endw();
  Image<float>::iterator lptr = len.beginw();
  Image<float>::iterator vptr = val.beginw();

  //uint width  = dir.getWidth();
  uint height = dir.getHeight();
  uint i = 0;   uint j = 0;
  while(dptr != stop)
    {
      float fval = *vptr++;
      
      if(fval > 0.0)
        {                   
          rutz::shared_ptr<FlowVector>
            (new FlowVector
             (Point2D<float>(i,j), *dptr++, *lptr++, fval));
        }

      if(j >= height){ j = 0; i++; }
    }
  itsImageDims = dir.getDims();

  itsFlowVectorsComputed = true;
}

// ######################################################################
void OpticalFlow::computeFlowField()
{  
  LFATAL("NEED DEBUGGING TO CONFIRM THAT IT WORKS - then delete this line");

  // get iterators of the to go through all the flow field
  Image<float> dir(itsImageDims, ZEROS);
  Image<float> len(itsImageDims, ZEROS);
  Image<float> val(itsImageDims, ZEROS);

  // we are going to assume we have a sparse flow field
  // and random access on the field
  for(uint v = 0; v < itsFlowVectors.size(); v++)
    {
      Point2D<float> pt = itsFlowVectors[v]->p1;
      uint i = pt.i;
      uint j = pt.j;
      float ang  = itsFlowVectors[v]->angle;
      float mag  = itsFlowVectors[v]->mag;
      float fval = itsFlowVectors[v]->val;
      
      if(fval > 0.0)
        {
          dir.setVal(i,j, ang );
          len.setVal(i,j, mag );
          val.setVal(i,j, fval);            
        }
    }
  itsImageDims = dir.getDims();

  itsFlowFieldComputed = true;
}


// ######################################################################
lobot::triple<Image<float>,Image<float>,Image<float> > 
OpticalFlow::getFlowField()
{
  // compute the flow vectors from the flow field first
  if(!itsFlowFieldComputed)
    {
      computeFlowField();
    }

  return itsFlowField;
}

// ######################################################################
Image<float> OpticalFlow::getDirectionField()
{
  // compute the flow vectors from the flow field first
  if(!itsFlowFieldComputed)
    {
      computeFlowField();
    }

  return itsFlowField.first;
}

// ######################################################################
Image<float> OpticalFlow::getVectorLengthField()
{
  // compute the flow vectors from the flow field first
  if(!itsFlowFieldComputed)
    {
      computeFlowField();
    }

  return itsFlowField.second;
}

// ######################################################################
Image<float> OpticalFlow::getFlowStrengthField()
{
  // compute the flow vectors from the flow field first
  if(!itsFlowFieldComputed)
    {
      computeFlowField();
    }

  return itsFlowField.third;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
