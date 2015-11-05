/*!@file Devices/VectorHistField.C implementation of vector histogram
field algorithm routines */
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
// Primary maintainer for this file: farhan baluch fbaluch@usc.edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/VectorHistField.C $
// $Id: VectorHistField.C 15272 2012-05-07 20:40:14Z kai $
//

#include "VectorHistField.H"
#include "Component/OptionManager.H"
#include "Devices/Serial.H"
#include "Image/DrawOps.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include <cmath>
#include <iterator>

// ######################################################################
VectorHistField::VectorHistField(OptionManager& mgr, const std::string& descrName,
                         const std::string& tagName, const int width,
                         const int height) :
        VectorField(mgr, descrName, tagName,width,height)
{
	itsDist = 0;
	itsAng  = 0;
	itsGoalRatio = 4.5;
	itsHistRatio = 3.0;

}

// ######################################################################
VectorHistField::~VectorHistField()
{ }

// ######################################################################
Image<PixRGB<byte> > VectorHistField::updateField
(std::vector<Point2D<float> > sensorData, 
 Point2D<int> robotPos, 
 float robotOrient, 
 float robotDist,Point2D<int> goal,
 Image<geom::vec2f> itsObsTemplate)
{

    geom::vec2f tmp,tmp2,goalVec;
    Image<float> final(itsField.getDims(),ZEROS);
        //Point2D<int> robotPos(itsField.getWidth()/2,itsField.getHeight() - 1);
        //in cell space so getHeight()-1 should move it up one cell

    robotPos.i = itsField.getWidth()/2;
    robotPos.j = (itsField.getHeight())-5;
        //robotPos.j = itsField.getHeight()/2;

    itsSensorData = sensorData;
    itsRobotPos = robotPos;
    itsRobotOrientation = robotOrient;

    goalVec.set_polar_rad((float)goal.i,geom::deg2rad((float)goal.j));


    itsGoalField.clear(goalVec);

    Image<float> finblob(itsField.getDims(),ZEROS);

    scaleFieldBy(itsHistRatio);
    scaleGoalFieldBy(itsGoalRatio);

    //rotate added vectors depending on how the robot turned
    //rotateField(itsRobotOrientation);
    rotateGoalField(itsRobotOrientation);

    goalVec.rotate_deg(itsRobotOrientation);
    itsGoal.i = goalVec.length();
    itsGoal.j = goalVec.theta_deg();


    //lets create a goal field and add that to the mix
    itsField = itsField + itsGoalField;

    //shiftField((int)robotDist, (int)itsRobotOrientation);

    Image<geom::vec2f>::iterator aptrF;

    for (uint i=0;i<sensorData.size();i++)
    {
        tmp.set_polar_rad(sensorData[i].i,geom::deg2rad(sensorData[i].j));

          //make obstacle position relative to robot
        Point2D<int> center(tmp.x() + robotPos.i,robotPos.j - tmp.y());

        //LINFO("gauss %d center %d,%d", i,center.i, center.j);
        //paste the template at the location of obstacle and add to field

        geom::vec2f bg(0,0);
        itsField = itsField + shiftClean(itsObsTemplate,
                                center.i-itsField.getWidth()/2,
                              center.j-itsField.getHeight()/2,bg);
    }

    //Image<geom::vec2f>::iterator aptrF;
    aptrF = itsField.beginw();
    while(aptrF != itsField.end())
    {
        tmp2 = *aptrF;
        tmp2/=(float)sensorData.size()+1; //taking an average
        tmp2.set_length(20.0);//(tmp2.length()+5.0);
        *aptrF++ = tmp2;
    }

    inplaceNormalize<float>(finblob, 0.0,150.0);
    return toRGB<byte>(finblob);
}

// ######################################################################
Image<geom::vec2f> VectorHistField::obstacleTemplate(float sigma,float amp)
{

  Image<geom::vec2f> result(itsField.getDims(),NO_INIT);
  Point2D<int> center(itsField.getWidth()/2, itsField.getHeight()/2);
  Image<float> blob = gaussianBlob<float>(itsField.getDims(),center,sigma,
                                                sigma);

  inplaceNormalize<float>(blob, 0.0,amp);
  Image<geom::vec2f>::iterator aptrF = result.beginw();
  Image<float>::iterator blobPtr = blob.beginw();

  geom::vec2f tmp2;
  int idx = 0;
  float minAng = 0.0, maxAng=0.0;
  while(aptrF!=result.end())
    {
      Point2D<int> endPt;
      endPt.i = idx % itsField.getWidth();
      endPt.j = (int) idx/itsField.getHeight();
      float angle = atan2(center.j - endPt.j, center.i-endPt.i);

      if (angle < minAng)
          minAng = angle;

      if (angle > maxAng)
          maxAng = angle;

      tmp2.set_polar_rad(*blobPtr++,  angle);

      *aptrF++ = tmp2;
      idx++;
    }

  return result;

}


// ######################################################################
void VectorHistField::setHistRatio(float hist)
{
	itsHistRatio = hist;	
}
// ######################################################################
void VectorHistField::setGoalRatio(float goal)
{
	itsGoalRatio = goal;	
}
// ######################################################################
void VectorHistField::shiftField(int dist,int ang)
{

  float x = itsDist*sin(geom::deg2rad(itsAng));
  float y = itsDist*cos(geom::deg2rad(itsAng));
  int cellSize = 3000/30;  //grid size 30 max sonar reading 3000
  int cellHalf = cellSize/2;

  int shiftX = 0, shiftY=0;
  int north = 1; int south=-1;
  int east =1, west=-1;
  geom::vec2f bg;
  bg.set_polar_rad((float)itsGoal.i,geom::deg2rad((float)itsGoal.j));

  if (x < cellHalf && x > -cellHalf)
    {
      shiftX = 0;
      if(y > cellHalf)
        shiftY = north;                       //shift NORTH
      else if(y < -cellHalf)
        shiftY = south;                       //shift SOUTH
    }


  else if(y < cellHalf && y > -cellHalf)
    {
      shiftY = 0;
      if(x > cellHalf)
        shiftX = west;                        //shift WEST
      else if(x < -cellHalf)
        shiftX = east;                        //shift EAST
    }


  else if(y > cellHalf)
    {
      shiftY = north;
      if(x > cellHalf)
        shiftX = west;                         //shift NW
      else if(x < -cellHalf)
        shiftX = east;                         //shift NE
    }



  else if(y < -cellHalf)
    {
      shiftY = south;
      if(x > cellHalf)
        shiftX = west;                         //shift SW
      else if(x < -cellHalf)
        shiftX = east;                          //shift SE
    }

  if ((shiftX !=0) | (shiftY!=0))
    {
      LINFO("Shifting field %d,%d", shiftX,shiftY);
      itsField = shiftClean(itsField,shiftX, shiftY);
      itsDist = 0;
      itsAng = 0;
    }
  else
    {
      LINFO("No shift total dist=%d,total ang=%d", itsDist,itsAng);
      itsDist+=dist;
      itsAng+=ang;

    }
}





// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
