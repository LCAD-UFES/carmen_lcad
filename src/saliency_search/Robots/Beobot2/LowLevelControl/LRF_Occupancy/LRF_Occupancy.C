/*!@file Robots2/Beobot2/Hardware/LRF_Occupancy.C Ice Module to log data    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/LRF_Occupancy.C
// $ $Id: LRF_Occupancy.C 12962 2010-03-06 02:13:53Z irock $
//
//////////////////////////////////////////////////////////////////////////


#include "Image/MatrixOps.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Ice/BeobotEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include "Image/ShapeOps.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/MorphOps.H"
#include "Image/Kernels.H"
#include "Raster/Raster.H"
#include "Robots/Beobot2/LowLevelControl/LRF_Occupancy/LRF_Occupancy.H"
#include "Util/sformat.H"

const ModelOptionCateg MOC_LRF_Occupancy = {
        MOC_SORTPRI_3, "LRF Occupancy Grid Related Options" };

const ModelOptionDef OPT_GridDims =
{ MODOPT_ARG(Dims), "Grid Dimensions", &MOC_LRF_Occupancy, OPTEXP_CORE,
        "The size of the occupancy grid in centimeters. Note that the resolution is 1cm",
        "grid-dims", '\0', "<w x h>", "300x500"};

// ######################################################################
LRF_Occupancy::LRF_Occupancy(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
        itsOccupancyMapDims(&OPT_GridDims, this, 0)
{
        addSubComponent(itsOfs);
}

// ######################################################################
LRF_Occupancy::~LRF_Occupancy()
{ }

// ######################################################################
void LRF_Occupancy::start1()
{
}

// ######################################################################
void LRF_Occupancy::registerTopics()
{
  // subscribe to all laser range finder data
  this->registerSubscription("LRFMessageTopic");
 // this->registerPublisher("MotorRequestTopic");
}

// ######################################################################
void LRF_Occupancy::evolve()
{
        std::vector<double> distances;
        std::vector<double> angles;

        itsLRFDataMutex.lock();
        {
                distances = itsDistances;
                angles    = itsAngles;
        }
        itsLRFDataMutex.unlock();

        computeOccupancy(distances, angles);
}



// ######################################################################
Image<bool> LRF_Occupancy::computeOccupancy(std::vector<double> distances, std::vector<double> angles)
{
        ASSERT(distances.size() == angles.size());

        //Clear out the old occupancy grid
        Image<byte> occupancyMap(itsOccupancyMapDims.getVal(), ZEROS);

        std::vector<double>::iterator distIt  = distances.begin();
        std::vector<double>::iterator angleIt = angles.begin();

        float angle, distance, x, y;

        int mapW = occupancyMap.getWidth();

        Image<byte> se = twofiftyfives(3);

        for(;distIt != distances.end() && angleIt != angles.end(); ++distIt, ++angleIt)
        {
                angle    = (*angleIt)*M_PI/180.0 - M_PI/2.0;
                distance = *distIt;

                if(distance == -1)
                        continue;

                //Find the (x,y) coordinates of the lrf reading in millimeters,
                //and convert to centimeters
                x = distance*cos(angle)/10.0;
                y = distance*sin(angle)/10.0;

                //Offest the x by half the grid size
                x += itsOccupancyMapDims.getVal().w()/2.0;
                y += itsOccupancyMapDims.getVal().h();

                //Convert to centimeters.

                if(occupancyMap.coordsOk(x,y))
                {
//                        occupancyMap.setVal(x,y,255);


                        //Draw a ray from the detected point to the bounds of the image.
                        //If the point is to the left of the center of the image, then
                        //draw the ray to the left, and same for the right.
                        Point2D<int> end;
                        if(abs(x - mapW/2.0) < 1)
                                end = Point2D<int>(mapW/2,0);
                        else if(x>mapW/2.0)
                                end = Point2D<int>(mapW-1, y+(mapW-x)*tan(angle));
                        else if(x<mapW/2.0)
                                end = Point2D<int>(0, y-x*tan(angle));

                        drawLine(occupancyMap, Point2D<int>(x,y), end, byte(255), 5);
                }

        } //End LRF Data
        occupancyMap = flipHoriz(occupancyMap);

        Image<byte> scaledOccMap = rescaleNI(occupancyMap, occupancyMap.getDims()/10);
        dilateImg(scaledOccMap, se);



        scaledOccMap = rescaleNI(scaledOccMap, occupancyMap.getDims()/3);

        if(!itsOfs->isVoid())
        {
                itsOfs->writeGray(scaledOccMap,"occupancy map");
                itsOfs->updateNext();
        }



        return occupancyMap;
}

// ######################################################################
void LRF_Occupancy::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  // LRF message
  if(eMsg->ice_isA("::BeobotEvents::LRFMessage"))
  {
    // we got LRF data
    BeobotEvents::LRFMessagePtr lrfMsg =
      BeobotEvents::LRFMessagePtr::dynamicCast(eMsg);

                //Set the class distance and angle measures
                //so that the evolve loop can work on them
                itsLRFDataMutex.lock();
                {
                        itsDistances = lrfMsg->distances;
                        itsAngles    = lrfMsg->angles;
                }
                itsLRFDataMutex.unlock();
  }
}

// ######################################################################

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
