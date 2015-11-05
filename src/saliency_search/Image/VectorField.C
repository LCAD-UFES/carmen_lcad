/*!@file Devices/VectorField.C implementation of vector histogram
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/VectorField.C $
// $Id: VectorField.C 15272 2012-05-07 20:40:14Z kai $
//

#include "Image/VectorField.H"
#include "Component/OptionManager.H"
#include "Devices/Serial.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include <iterator>


// ######################################################################
VectorField::VectorField(OptionManager& mgr, const std::string& descrName,
                         const std::string& tagName, const int width,
                         const int height) :
  ModelComponent(mgr, descrName, tagName)
{
    itsDims = Dims(width,height);
    itsField = Image<geom::vec2f> (itsDims,NO_INIT);
    itsGoalField = Image<geom::vec2f>(itsDims,NO_INIT);

    geom::vec2f tmp;
    tmp.set_polar_rad(20.0,geom::deg2rad(90.0));
    itsField.clear(tmp);

    LINFO("vector field initialized");
}


// ######################################################################
VectorField::~VectorField()
{ }

// ######################################################################
Image<geom::vec2f> VectorField::getField()
{
    return itsField;

}
// ######################################################################
Image<geom::vec2f> VectorField::getGoalField()
{
    return itsGoalField;
}

// ######################################################################
Image<PixRGB<byte> > VectorField::plotField(int spacing)
{

    Image<PixRGB<byte> > result(itsDims.w(),itsDims.h(),ZEROS);

    Image<geom::vec2f>::iterator itr = itsField.beginw();

    int idx= 0;

    while(itr!=itsField.endw())
        {
          Point2D<int> startPt, endPt;
          startPt.i = idx % (itsDims.w());
          startPt.j = (int)idx/(itsDims.w());
           geom::vec2f tmp = *itr;

          if((idx > spacing*itsDims.w()) && (startPt.i % spacing == 0)
             && (startPt.j % spacing ==0))
            {
                endPt.i = startPt.i + tmp.x();
                endPt.j = startPt.j + tmp.y();
                if(result.coordsOk(startPt) && result.coordsOk(endPt))
                {
                    drawLine(result,  startPt, endPt,PixRGB<byte>(0,255,0),1);
                    drawDisk(result, startPt, 2, PixRGB<byte>(255,0,0));
                }
            }
          else
            {
                    // drawPoint(result, idx % itsDims.w(), idx/itsDims.w(),
                    // PixRGB<byte>(0,255,0) );
            }
           itr++;
           idx++;
        }

        // lets also draw the vectors pointing to obstacles detected
    for(uint i=0; i<itsSensorData.size();i++)
      {
        geom::vec2f tmp;
        tmp.set_polar_rad(itsSensorData[i].i,geom::deg2rad(itsSensorData[i].j));
        Point2D<int> center(tmp.x()+itsRobotPos.i, itsRobotPos.j - tmp.y());
        drawLine(result,itsRobotPos,center,PixRGB<byte> (0,0,255),1);
    }
    return result;
}


// ######################################################################
Image<PixRGB<byte> > VectorField::plotGridField(int spacing)
{
  Image<PixRGB<byte> > result(itsDims.w()*spacing,itsDims.h()*spacing,ZEROS);

  // first draw the vectors pointing to obstacles detected
  Point2D<int> robotPos;
  robotPos.i = itsRobotPos.i*spacing + spacing/2;
  robotPos.j = itsRobotPos.j*spacing + spacing/2;
  for(uint i=0; i<itsSensorData.size();i++)
    {
      geom::vec2f tmp;
      tmp.set_polar_rad(itsSensorData[i].i,geom::deg2rad(itsSensorData[i].j));
      Point2D<int> center(tmp.x() + itsRobotPos.i, itsRobotPos.j - tmp.y());
        
      //center.i = center.i + itsRobotPos.i;
      //center.j = itsRobotPos.j - center.j;

      center.i = center.i*spacing + spacing/2;
      center.j = center.j*spacing + spacing/2;
      drawCircle(result, center,2,PixRGB<byte>(255,255,0),3);
      //drawLine(result,robotPos,center,PixRGB<byte> (255,255,255),1);
    }
    
  //then draw grid on this image
  drawGrid(result, spacing, spacing, 1, 1, PixRGB<byte>(255,0,0));
  Image<geom::vec2f>::iterator itr = itsField.beginw();
    
  int idx= 0;
  while(itr!=itsField.endw())
    {
      Point2D<int> vecImg,startPt, endPt;
      vecImg.i = idx % (itsDims.w());
      vecImg.j = (int)idx/(itsDims.w());
      geom::vec2f tmp = *itr;
        
      startPt.i = vecImg.i*spacing + spacing/2;
      startPt.j = vecImg.j*spacing + spacing/2;
        
      endPt.i = startPt.i + tmp.x();
      endPt.j = startPt.j + tmp.y();
        
      if(result.coordsOk(startPt) && result.coordsOk(endPt))
        {
          drawLine(result,  startPt, endPt,PixRGB<byte>(0,255,0),1);
          drawDisk(result, startPt, 2, PixRGB<byte>(255,0,0));
        }
        
      itr++;
      idx++;
    }
    
    
  drawCircle(result, robotPos,5,PixRGB<byte>(150,255,0),1);
    
  return result;
}


// ######################################################################
Image<PixRGB<byte> > VectorField::plotGoalField(int spacing)
{
  Image<PixRGB<byte> > result(itsDims.w()*spacing,itsDims.h()*spacing,ZEROS);
    
  //then draw grid on this image
  drawGrid(result, spacing, spacing, 1, 1, PixRGB<byte>(255,0,0));
  Image<geom::vec2f>::iterator itr = itsGoalField.beginw();
    
  int idx= 0;
  while(itr!=itsGoalField.endw())
    {
      Point2D<int> vecImg,startPt, endPt;
      vecImg.i = idx % (itsDims.w());
      vecImg.j = (int)idx/(itsDims.w());
      geom::vec2f tmp = *itr;
        
      startPt.i = vecImg.i*spacing + spacing/2;
      startPt.j = vecImg.j*spacing + spacing/2;
        
      endPt.i = startPt.i + tmp.x();
      endPt.j = startPt.j + tmp.y();
        
      if(result.coordsOk(startPt) && result.coordsOk(endPt))
        {
          drawLine(result,  startPt, endPt,PixRGB<byte>(0,255,0),1);
          drawDisk(result, startPt, 2, PixRGB<byte>(255,0,0));
        }
        
      itr++;
      idx++;
    }  
    
  return result;
}


// ######################################################################
geom::vec2f VectorField::getVectorAt(Point2D<int> location)
{
    return itsField.getVal(location.i,location.j);
}


// ######################################################################
void VectorField::setVectorAt(Point2D<int> location, geom::vec2f val)
{
    itsField.setVal(location.i,location.j,val);
}


// ######################################################################

    //!scaleby -- scale vector field by factor
void VectorField::scaleFieldBy(float factor)
{
    Image<geom::vec2f>::iterator aptr = itsField.beginw();
    while(aptr != itsField.end())
    {
        geom::vec2f tmp = *aptr;
        tmp.scale_by(factor);
        *aptr++= tmp;
    }
}


// ######################################################################

    //!scaleby -- scale vector field by factor
void VectorField::scaleGoalFieldBy(float factor)
{
    Image<geom::vec2f>::iterator aptr = itsGoalField.beginw();
    while(aptr != itsGoalField.end())
    {
        geom::vec2f tmp = *aptr;
        tmp.scale_by(factor);
        *aptr++= tmp;
    }
}
// ######################################################################
void VectorField::makeUnitLength()
{
    Image<geom::vec2f>::iterator aptr = itsField.beginw();

    while(aptr != itsField.end())
    {
        geom::vec2f tmp = *aptr;
        *aptr++ = make_unit_length(tmp);
    }

}

// ######################################################################
void VectorField::normalizeTo(float maxFactor)
{
    Image<geom::vec2f>::iterator aptr = itsField.beginw();

    while(aptr != itsField.end())
    {
        geom::vec2f tmp = *aptr;
        *aptr++= tmp/=maxFactor;
    }

}


// ######################################################################
void VectorField::rotateField(float angle)
{
        //rotate the image
    itsField = rotate(itsField,itsRobotPos.i,itsRobotPos.j,geom::deg2rad(angle));
        //rotate individual vectors

    Image<geom::vec2f>::iterator aptrF;
    geom::vec2f tmp;
    aptrF = itsField.beginw();
    while(aptrF != itsField.end())
    {
        tmp = *aptrF;
        tmp.rotate_deg(angle);
        *aptrF++ = tmp;
    }

}


// ######################################################################
void VectorField::rotateGoalField(float angle)
{
        //since we have a general goal direction only no need to rotate
        //entire field just individual vectos

       //rotate individual vectors
    Image<geom::vec2f>::iterator aptrF;
    geom::vec2f tmp;
    aptrF = itsGoalField.beginw();
    while(aptrF != itsGoalField.end())
    {
        tmp = *aptrF;
        tmp.rotate_deg(angle);
        *aptrF++ = tmp;
    }

}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
