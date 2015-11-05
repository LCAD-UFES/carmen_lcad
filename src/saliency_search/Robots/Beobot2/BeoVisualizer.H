/*!@file Robots2/Beobot2/BeoVisualizer.H Ice Module to Log data    */
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
// Primary maintainer for this file: Chin-Kai Chang<chinkaic@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Service/BeoVisualizer.H
// $Id: BeoVisualizer.H 15190 2012-02-29 04:00:13Z beobot $
//

#ifndef BEOVISUALIZER_H
#define BEOVISUALIZER_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/BeobotEvents.ice.H"
#include <IceUtil/Thread.h>
#include "Media/FrameSeries.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"   // for rescale()
#include "Image/CutPaste.H"   // for inplacePaste()
#include <vector>

class Panel{ 
	private: 

	Point2D<int> itsCenterPoint;
	Point2D<int> itsTopLeftPoint;
	Point2D<int> itsTopRightPoint;
	Point2D<int> itsBottomLeftPoint;
	Point2D<int> itsBottomRightPoint;
	Dims 				 itsDims;
	Rectangle		 itsRectangle;
	PixRGB<byte> itsBgColor;
	PixRGB<byte> itsBorderColor;
	PixRGB<byte> itsLabelColor;
	std::string  itsLabel;
	int 				 itsLabelFontSize;
	int 				 itsBorderThickness;
  Image<PixRGB<byte> > itsImage;	
	public: //Initialize the variables 
	Panel(){itsLabelFontSize = 20;};
  Panel(Point2D<int> topLeft,Dims dims,std::string label = "")
  {

    itsTopLeftPoint = topLeft;
    itsCenterPoint     = Point2D<int>(topLeft.i + dims.w()/2,topLeft.j + dims.h()/2);

    itsTopRightPoint   = Point2D<int>(topLeft.i + dims.w(),topLeft.j);
    itsBottomLeftPoint = Point2D<int>(topLeft.i           ,topLeft.j + dims.h());
    itsBottomRightPoint= Point2D<int>(topLeft.i + dims.w(),topLeft.j + dims.h());
    itsDims = dims;
    itsRectangle = Rectangle(topLeft,dims);
    itsBgColor = PixRGB<byte>(0,255,0);
    itsBorderColor = PixRGB<byte>(255,255,255);
    itsLabel = label;
    itsLabelColor		 = PixRGB<byte>(255,255,255);
    itsLabelFontSize = 20;
    itsBorderThickness = 1;

  } 
  Point2D<int> tl(){ return itsTopLeftPoint; }
  Point2D<int> tr(){ return itsTopRightPoint; }
  Point2D<int> bl(){ return itsBottomLeftPoint; }
  Point2D<int> br(){ return itsBottomRightPoint; }
  Point2D<int> ct(){ return itsCenterPoint; }
	//Handles events and set the button's sprite region 
  bool handle_events(Point2D<int> mouseClickPoint)
  { 
    int x = mouseClickPoint.i;
    int y = mouseClickPoint.j;
    int lx = itsTopLeftPoint.i;//left x
    int rx = lx	+	itsDims.w();//right x

    int ty = itsTopLeftPoint.j;//top y
    int by = ty	+	itsDims.h();//bottom y

    if( (x  > lx ) && ( x < rx ) && ( y > ty ) && ( y < by ) )
    { 
      return true;
    } 
    return false;
  }
	void setLabel(std::string label) { itsLabel = label;	}
	void setLabelColor(PixRGB<byte> color){ itsLabelColor = color; }

  //update backgroundimage
  void updateImage(Image<PixRGB<byte> >img)
  {
    //check size of image match panel size
    if(img.getDims() == itsDims)
      itsImage = img;
    else 
      itsImage = rescale(img,itsDims);
  }
	//Shows the button on the screen 
  void draw(Image<PixRGB<byte> > &disp,bool title = false)
  {
    if(itsImage.initialized())
      inplacePaste(disp,itsImage,itsTopLeftPoint);

    if(title)
    {
      //drawFilledRect(disp, itsRectangle, itsBgColor);
      writeText(disp,itsTopLeftPoint,itsLabel.c_str(),itsLabelColor,
          itsBgColor,SimpleFont::FIXED(itsLabelFontSize),false,ANCHOR_TOP_LEFT);
      drawRect(disp, itsRectangle, itsBorderColor,itsBorderThickness);
    }
  }
};


class BeoVisualizer : public RobotBrainComponent
{
public:

  BeoVisualizer(OptionManager& mgr,
         const std::string& descrName = "BeoVisualizer",
         const std::string& tagName = "BeoVisualizer");

  ~BeoVisualizer();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();

  // initialize the log folder
  void start1();

  // publish the log folder name
  void start3();

private:
  Timer itsTimer;
  int itsCurrMessageID;
  nub::soft_ref<OutputFrameSeries> itsOfs;
  Image<PixRGB<byte> > itsDispImage;
  
  Dims itsHandycamDim;
  Dims itsLocalizerDim;
  Dims itsNavigatorDim;
  Dims itsImageMatchDim;
  Dims itsRoadFinderDim;

  Panel itsHandycam;
  Panel itsLocalizer;
  Panel itsNavigator;
  Panel itsImageMatch;
  Panel itsRoadFinder;
  Panel itsPilot;
  Panel itsSim;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
