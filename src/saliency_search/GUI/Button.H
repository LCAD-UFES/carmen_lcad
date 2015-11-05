/*!@file GUI/Button.H Simple Button implemented  using SDL */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/Button.H $
// $Id: Button.H 14170 2010-10-27 07:04:19Z ilink $
//
#ifndef GUI_BUTTON_H_DEFINED
#define GUI_BUTTON_H_DEFINED

#ifdef HAVE_SDL_SDL_H

#include "Component/ModelComponent.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/Timer.H"          // for Timer
#include "Video/VideoFormat.H"
#include "Component/EventLog.H"

#include <list>

//The button 
class Button { 
	private: 

	Point2D<int> itsTopLeftPoint;
	Point2D<int> itsCenterPoint;
	Dims 				 itsDims;
	Rectangle		 itsRectangle;
	PixRGB<byte> itsBgColor;
	PixRGB<byte> itsBorderColor;
	PixRGB<byte> itsLabelColor;
	std::string  itsLabel;
	int 				 itsFontSize;
	int 				 itsBorderThickness;
	
	public: //Initialize the variables 
	Button(){};
	Button(Point2D<int> topLeft,Dims dims ); 

	//Handles events and set the button's sprite region 
	bool handle_events(Point2D<int> mouseClickPoint); 

	void setLabel(std::string label);
	void setLabelColor(PixRGB<byte> color);
	void setButtonBgColor(PixRGB<byte> color);
	void setFrontSize(int size);

	//Shows the button on the screen 
	void show(Image<PixRGB<byte> > &buttonImg);
};
#endif // HAVE_SDL_SDL_H


#endif
