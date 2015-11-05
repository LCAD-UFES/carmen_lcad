/*!@file Psycho/PsychoDisplay.C Display psychophysics stimuli */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/PsychoDisplay.C $
// $Id: PsychoDisplay.C 14376 2011-01-11 02:44:34Z pez $
//

#ifdef HAVE_SDL_SDL_H

#include "Psycho/PsychoDisplay.H"

#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "GUI/SDLdisplay.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Psycho/EyeTracker.H"
#include "Psycho/PsychoOpts.H"
#include "Util/sformat.H"
#include <vector>

// Used by: PsychoDisplay
const ModelOptionDef OPT_PsychoDisplayBackgroundColor =
  { MODOPT_ARG(PixRGB<byte>), "PsychoDisplayBackgroundColor", &MOC_PSYCHODISP, OPTEXP_CORE,
    "Background grey color for PsychoDisplay",
    "psycho-background-color", '\0', "<r,g,b>", "128,128,128" };

// Used by: PsychoDisplay
const ModelOptionDef OPT_PsychoDisplayTextColor =
  { MODOPT_ARG(PixRGB<byte>), "PsychoDisplayTextColor", &MOC_PSYCHODISP, OPTEXP_CORE,
    "Foreground text color for PsychoDisplay",
    "psycho-text-color", '\0', "<r,g,b>", "0,0,0" };

// Used by: PsychoDisplay
const ModelOptionDef OPT_PsychoDisplayBlack =
  { MODOPT_ARG(PixRGB<byte>), "PsychoDisplayBlack", &MOC_PSYCHODISP, OPTEXP_CORE,
    "'Black' color for PsychoDisplay",
    "psycho-black", '\0', "<r,g,b>", "0,0,0" };

// Used by: PsychoDisplay
const ModelOptionDef OPT_PsychoDisplayWhite =
  { MODOPT_ARG(PixRGB<byte>), "PsychoDisplayWhite", &MOC_PSYCHODISP, OPTEXP_CORE,
    "'White' color for PsychoDisplay",
    "psycho-white", '\0', "<r,g,b>", "255,255,255" };

// Used by: PsychoDisplay
const ModelOptionDef OPT_PsychoDisplayFixationIcon =
  { MODOPT_ARG_STRING, "PsychoDisplayFixationIcon", &MOC_PSYCHODISP, OPTEXP_CORE,
    "Filename of fixation image icon, instead of displaying fixation cross",
    "psycho-fixation-icon", '\0', "<filename.jpg>", "" };


// ######################################################################
PsychoDisplay::PsychoDisplay(OptionManager& mgr, const std::string& descrName,
                             const std::string& tagName) :
  SDLdisplay(mgr, descrName, tagName),
  itsBackgroundColor(&OPT_PsychoDisplayBackgroundColor, this),
  itsTextColor(&OPT_PsychoDisplayTextColor, this),
  itsBlack(&OPT_PsychoDisplayBlack, this),
  itsWhite(&OPT_PsychoDisplayWhite, this),
  itsFixationIcon(&OPT_PsychoDisplayFixationIcon, this),
  itsFixSiz("PsychoDisplayFixSiz", this, 11),
  itsFixThick("PsychoDisplayFixThick", this, 1),
  itsEyeTracker()
{  }

// ######################################################################
PsychoDisplay::~PsychoDisplay()
{  }

// ######################################################################
void PsychoDisplay::setEyeTracker(nub::soft_ref<EyeTracker> e)
{ itsEyeTracker = e; }

// ######################################################################
void PsychoDisplay::setFixationSize(const int size)
{
  itsFixSiz.setVal(size);
}

// ######################################################################
void PsychoDisplay::clearScreen(const bool vsync)
{ SDLdisplay::clearScreen(itsBackgroundColor.getVal(), vsync); }

// ######################################################################
void PsychoDisplay::displayFixationIcon(const Image< PixRGB<byte> >& image,
                const int x, const int y, const bool vsync)
{
  const int siz = image.getWidth();
  int i, j; // position of center of cross
  int siz2 = (siz - 1) / 2; // half cross size
  int w = itsDims.getVal().w(), h = itsDims.getVal().h();

  if (x == -1 || y == -1) {
    i = itsDims.getVal().w() / 2 - 1;
    j = itsDims.getVal().h() / 2 - 1;
  } else {
    i = x; j = y;
    if (i < siz2) i = siz2; else if (i >= w - siz2) i = w - siz2 - 1;
    if (j < siz2) j = siz2; else if (j >= h - siz2) j = h - siz2 - 1;
  }
  pushEventBegin(sformat("displayFixationIcon (%d, %d)", i, j));

  Point2D<int> xy(i-siz2, j-siz2);
  displayImagePatch(image, xy, -2, true, true);

  syncScreen(vsync, false, true);
  pushEventEnd("displayFixationIcon");
}

// ######################################################################
void PsychoDisplay::displayFixation(const int x, const int y, const bool vsync)
{
  SDL_Rect rect;
  const int thick = itsFixThick.getVal(), siz = itsFixSiz.getVal();
  int i, j; // position of center of cross
  int siz2 = (siz - 1) / 2; // half cross size
  int w = itsDims.getVal().w(), h = itsDims.getVal().h();

  if (x == -1 || y == -1) {
    i = itsDims.getVal().w() / 2 - 1;
    j = itsDims.getVal().h() / 2 - 1;
  } else {
    i = x; j = y;
    if (i < siz2) i = siz2; else if (i >= w - siz2) i = w - siz2 - 1;
    if (j < siz2) j = siz2; else if (j >= h - siz2) j = h - siz2 - 1;
  }
  pushEventBegin(sformat("displayFixation (%d, %d)", i, j));

  // horizontal bar
  rect.x = i - siz2;
  rect.y = j - thick / 2;
  rect.w = siz;
  rect.h = thick;
  SDL_FillRect(itsScreen, &rect, getBlackUint32());

  // vertical bar
  rect.x = i - thick / 2;
  rect.y = j - siz2;
  rect.w = thick;
  rect.h = siz;
  SDL_FillRect(itsScreen, &rect, getBlackUint32());

  syncScreen(vsync, false, true);
  pushEventEnd("displayFixation");
}

// ######################################################################
void PsychoDisplay::displayCircle(const int x, const int y, 
								                  const int radius,  
																	const PixRGB<byte> color, const bool vsync)
{
  int i, j; // position of center of circle

	// center of circle
  if (x == -1 || y == -1) {
    i = itsDims.getVal().w() / 2 - 1;
    j = itsDims.getVal().h() / 2 - 1;
  } else {
    i = x; j = y;
  }

	// draw circle
  pushEventBegin(sformat("displayCircle (%d, %d)", i, j));
	aacircleRGBA(itsScreen, i, j, radius, color.red(), color.green(), color.blue(), 0XFF);
  syncScreen(vsync, false, true);
  pushEventEnd(sformat("displayCircle (%d, %d)", i, j));
}

// ######################################################################
void PsychoDisplay::displayFilledCircle(const int x, const int y, 
								                        const int radius,  
																	      const PixRGB<byte> color, const bool vsync)
{
  int i, j; // position of center of circle

	// center of circle
  if (x == -1 || y == -1) {
    i = itsDims.getVal().w() / 2 - 1;
    j = itsDims.getVal().h() / 2 - 1;
  } else {
    i = x; j = y;
  }

	// draw circle
  pushEventBegin(sformat("displayCircle (%d, %d)", i, j));
	filledCircleRGBA(itsScreen, i, j, radius, color.red(), color.green(), color.blue(), 0XFF);
  syncScreen(vsync, false, true);
  pushEventEnd(sformat("displayCircle (%d, %d)", i, j));
}

// ######################################################################
void PsychoDisplay::displayFilledCircleBlink(const int x, const int y,
								                             const int radius,
																						 const PixRGB<byte> color,
                                             const int iter, const int delay)
{
  pushEventBegin("displayFilledCircleBlink");
  for (int i = 0; i < iter; i ++)
    {
			displayFilledCircle(x, y, radius, color, true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
      displayFilledCircle(x, y, radius, itsBackgroundColor.getVal(), true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
    }
  pushEventEnd("displayFilledCircleBlink");
}

// ######################################################################
void PsychoDisplay::displayFixationIconBlink(const Image< PixRGB<byte> >& image,
                                         const int x, const int y,
                                         const int iter, const int delay)
{
  pushEventBegin("displayFixationBlink");
  for (int i = 0; i < iter; i ++)
    {
      displayFixationIcon(image, x, y, true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
      clearScreen(true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
    }
  pushEventEnd("displayFixationBlink");
}

// ######################################################################
void PsychoDisplay::displayFixationBlink(const int x, const int y,
                                         const int iter, const int delay)
{
  pushEventBegin("displayFixationBlink");
  for (int i = 0; i < iter; i ++)
    {
      displayFixation(x, y, true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
      clearScreen(true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
    }
  pushEventEnd("displayFixationBlink");
}

// ######################################################################
void PsychoDisplay::displayColorDotFixationBlink(const int x, const int y,
                                              const int iter, const int delay,
                                              const PixRGB<byte> color)
{
  pushEventBegin("displayColorDotFixationBlink");
  for (int i = 0; i < iter; i ++)
    {
      displayColorDotFixation(x, y, color, true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
      //clearScreen(true);
      displayColorDotFixation(x, y, itsBackgroundColor.getVal(), true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
    }
  pushEventEnd("displayColorDotFixationBlink");
}

// ######################################################################
void PsychoDisplay::displayColorDotFixation(const int x, const int y,
                                            const PixRGB<byte> color, const bool vsync)
{
  SDL_Rect rect;
  const int siz = itsFixSiz.getVal() / 2;
  int i, j; // position of center of cross
  int siz2 = (siz - 1) / 2; // half cross size
  int w = itsDims.getVal().w(), h = itsDims.getVal().h();

  if (x == -1 || y == -1) {
    i = itsDims.getVal().w() / 2 - 1;
    j = itsDims.getVal().h() / 2 - 1;
  } else {
    i = x; j = y;
    if (i < siz2) i = siz2; else if (i >= w - siz2) i = w - siz2 - 1;
    if (j < siz2) j = siz2; else if (j >= h - siz2) j = h - siz2 - 1;
  }

  rect.x = i - siz2; rect.y = j - siz2; rect.w = siz; rect.h = siz;
  SDL_FillRect(itsScreen, &rect, getUint32color(color));

  syncScreen(vsync, false, true);
}

// ######################################################################
void PsychoDisplay::displayRedDotFixation(const int x, const int y,
                                          const bool vsync)
{
  SDL_Rect rect;
  const int siz = itsFixSiz.getVal() / 2;
  int i, j; // position of center of cross
  int siz2 = (siz - 1) / 2; // half cross size
  int w = itsDims.getVal().w(), h = itsDims.getVal().h();

  if (x == -1 || y == -1) {
    i = itsDims.getVal().w() / 2 - 1;
    j = itsDims.getVal().h() / 2 - 1;
  } else {
    i = x; j = y;
    if (i < siz2) i = siz2; else if (i >= w - siz2) i = w - siz2 - 1;
    if (j < siz2) j = siz2; else if (j >= h - siz2) j = h - siz2 - 1;
  }
  pushEventBegin(sformat("displayRedDotFixation (%d, %d)", i, j));

  rect.x = i - siz2; rect.y = j - siz2; rect.w = siz; rect.h = siz;
  SDL_FillRect(itsScreen, &rect, getUint32color(PixRGB<byte>(255, 0, 0)));

  syncScreen(vsync, false, true);
  pushEventEnd("displayRedDotFixation");
}

// ######################################################################
void PsychoDisplay::displayWhiteDotFixation(const int x, const int y,
                                            const bool vsync)
{
  SDL_Rect rect;
  const int siz = itsFixSiz.getVal() / 2;
  int i, j; // position of center of cross
  int siz2 = (siz - 1) / 2; // half cross size
  int w = itsDims.getVal().w(), h = itsDims.getVal().h();

  if (x == -1 || y == -1) {
    i = itsDims.getVal().w() / 2 - 1;
    j = itsDims.getVal().h() / 2 - 1;
  } else {
    i = x; j = y;
    if (i < siz2) i = siz2; else if (i >= w - siz2) i = w - siz2 - 1;
    if (j < siz2) j = siz2; else if (j >= h - siz2) j = h - siz2 - 1;
  }
  pushEventBegin(sformat("displayWhiteDotFixation (%d, %d)", i, j));

  rect.x = i - siz2; rect.y = j - siz2; rect.w = siz; rect.h = siz;
  SDL_FillRect(itsScreen, &rect, getUint32color(PixRGB<byte>(255, 255, 255)));

  syncScreen(vsync, false, true);
  pushEventEnd("displayWhiteDotFixation");
}

// ######################################################################
void PsychoDisplay::displayRedDotFixationBlink(const int x, const int y,
                                               const int iter, const int delay)
{
  pushEventBegin("displayRedDotFixationBlink");
  for (int i = 0; i < iter; i ++)
    {
      displayRedDotFixation(x, y, true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
      clearScreen(true);
      for (int i = 0; i < delay; ++i) waitNextRequestedVsync(false, true);
    }
  pushEventEnd("displayRedDotFixationBlink");
}

// ######################################################################
void PsychoDisplay::displayRedDotFixationBlink(Image< PixRGB<byte> > img, const int x, const int y,
                                               const int iter, const int delay)
{
  //make a copy of our original surface
  SDL_Surface *imgs = makeBlittableSurface(img);
  SDL_Rect screct;screct.w = 640; screct.h = 480;
  pushEventBegin("displayRedDotFixationBlink");
  for (int i = 0; i < iter; i ++)
    {
      displayImage(img);
      displayRedDotFixation(x, y, true);
      for (int i = 0; i < delay*4; ++i) waitNextRequestedVsync(false, true);
      SDL_BlitSurface(imgs, NULL, itsScreen, &screct);
      SDL_UpdateRect(itsScreen,screct.x,screct.y,screct.w,screct.h);
      for (int i = 0; i < delay*4; ++i) waitNextRequestedVsync(false, true);
    }
  pushEventEnd("displayRedDotFixationBlink");
}

// ######################################################################
void PsychoDisplay::displayISCANcalib()
{
  pushEventBegin("ISCANcalibration");
  int w = itsDims.getVal().w(), h = itsDims.getVal().h();

  // get the coords of the fixation points in a 640x480 image:
  const int npts = 5;
  int px[npts] = { 320, 100, 540, 100, 540 };
  int py[npts] = { 240, 80,  80,  380, 380 };

  // rescale the coords of the fixation points if image size not 640x480:
  if (w != 640)
    for (int i = 0; i < npts; i ++)
      px[i] = int(round(float(px[i]) * float(w) / 640.0f));
  if (h != 480)
    for (int i = 0; i < npts; i ++)
      py[i] = int(round(float(py[i]) * float(h) / 480.0f));

  // let's loop over the points and display them all at once:
  SDL_Rect rect; rect.w = 5; rect.h = 5;
  SDL_Rect rect2; rect2.w = 3; rect2.h = 3;
  for (int i = 0; i < npts; i ++)
    {
      rect.x = px[i] - 1; rect.y = py[i] - 1; rect2.x = px[i]; rect2.y = py[i];

      SDL_FillRect(itsScreen, &rect, getBlackUint32());
      SDL_FillRect(itsScreen, &rect2, getWhiteUint32());
    }
  syncScreen(true, false);
  pushEventEnd("ISCANcalibration");
}

//#######################################################################
void PsychoDisplay::displayMovingDotBackground(SDL_Surface *img, const int startX, const int startY,
                                  const int endX, const int endY, const float speed, const PixRGB<byte> color)
{

 if (itsEyeTracker.isValid() == false)
    LFATAL("You need to set an EyeTracker using setEyeTracker() first");



 //get color in RGB space
 Uint32 col = getUint32color(color);

 // calulate number of frames (the unit of speed is pixels/frame
 float frameNum = sqrt(pow(endX-startX,2)+pow(endY-startY,2))/speed;
 std::vector<Point2D<int> > pts;
 float unitX = (endX-startX)/frameNum, unitY = (endY-startY)/frameNum;
 for(int i=0; i<frameNum; i++){
    int x = static_cast<int>(startX + i * unitX);
    int y = static_cast<int>(startY + i * unitY);
    pts.push_back(Point2D<int>(x, y));
 }

 //lets create a rectangle for the whole screem
 SDL_Rect screct;screct.w = 640; screct.h = 480;
 screct.x = 0; screct.y = 0;

 //let's loop over the points and display them
  SDL_Rect rect; rect.w = 7; rect.h = 7;
  for(unsigned int i=0; i<pts.size(); i++){
    Point2D<int> p = pts[i];
    rect.x = p.i-3; rect.y = p.j-3;

    //make a copy of our original surface
    SDL_BlitSurface(img, NULL, itsScreen, &screct);
    //display new square
    SDL_FillRect(itsScreen, &rect, col);
    //update screen here
    waitNextRequestedVsync(false, true);
    SDL_UpdateRect(itsScreen,screct.x,screct.y,screct.w,screct.h);

    // log moving square position:
    pushEvent(sformat("Dot_Position at (%d, %d)", p.i, p.j));
  }
}

//######################################################################
void PsychoDisplay::displayMovingDotTrain(const Image< PixRGB<byte> >& img, int location[][2], int num_loc,
                            float speed[], int stay[], const  PixRGB<byte> color)
{
  SDL_Surface *sdlimg = makeBlittableSurface(img);
  pushEventBegin("Display Dot Movement Train");
  //clearScreen();

  // foreach transition location
  for(int i=0; i<num_loc-1; i++){
    // stay
    pushEventBegin(sformat("Stay at location (%d, %d)", location[i][0], location[i][1]));

    for(int j=0; j<stay[i]; j++) {waitNextRequestedVsync(false, true);}

    pushEventEnd(sformat("Stay at location (%d, %d)", location[i][0], location[i][1]));

    // move
    pushEventBegin(sformat("Move from (%d, %d) to (%d, %d) at %f pixel/frame",
                    location[i][0], location[i][1], location[i+1][0],
                    location[i+1][1], speed[i]));
    displayMovingDotBackground(sdlimg,location[i][0], location[i][1],
                                 location[i+1][0], location[i+1][1], speed[i],color);
    pushEventEnd(sformat("Move from (%d, %d) to (%d, %d) at %f pixel/frame",
                    location[i][0], location[i][1], location[i+1][0],
                    location[i+1][1], speed[i]));
  }

  // stay for the last location
  int l = num_loc-1;
  pushEventBegin(sformat("Stay at location (%d, %d)", location[l][0], location[l][1]));
  for(int j=0; j<stay[l]; j++){ waitNextRequestedVsync(false, true);}
  pushEventEnd(sformat("Stay at location (%d, %d)", location[l][0], location[l][1]));
  pushEventEnd("Display Dot Movement Train");

}





// ######################################################################
void PsychoDisplay::displaySmoothPursuitGroupCalibration(int location[][2], int num_loc, float speed[], int num_speed)

{
  pushEventBegin("Eye Tracker Calibration by Smooth Pursuit");

  clearScreen();

  // start the eye tracker
  itsEyeTracker->track(true);

  // for each speed
  for(int i=0; i<num_speed; i++){

  // display a fixation point at the beginning of the path with different speed
  displayFixation(location[0][0], location[0][1]);
  waitForKey();
  clearScreen();

    // for each start/end pair
    for(int j=0; j<num_loc-1; j++){
      displaySmoothPursuitCalibration(location[j][0], location[j][1],
                   location[j+1][0], location[j+1][1], speed[i]);
     }
  }

  // stop the eye tracker
  itsEyeTracker->track(false);

  clearScreen();

  pushEventEnd("Eye Tracker Calibration by Smooth Pursuit");
}

// ######################################################################
void PsychoDisplay::displaySmoothPursuitCalibration(
                const int startX, const int startY,
                const int endX, const int endY, const float speed, uint color)
{

  if (itsEyeTracker.isValid() == false)
    LFATAL("You need to set an EyeTracker using setEyeTracker() first");

  // calulate number of frames (the unit of speed is pixels/frame
  float frameNum = sqrt(pow(endX-startX,2)+pow(endY-startY,2))/speed;
  std::vector<Point2D<int> > pts;
  float unitX = (endX-startX)/frameNum, unitY = (endY-startY)/frameNum;
  for(int i=0; i<frameNum; i++){
    int x = static_cast<int>(startX + i * unitX);
    int y = static_cast<int>(startY + i * unitY);
    pts.push_back(Point2D<int>(x, y));
  }

  //let's loop over the points and display them
  SDL_Rect rect; rect.w = 7; rect.h = 7;
  SDL_Rect preRect; preRect.w = rect.w; preRect.h = rect.h; preRect.x = 0; preRect.y = 0;
  for(unsigned int i=0; i<pts.size(); i++){
    Point2D<int> p = pts[i];
    rect.x = p.i-3; rect.y = p.j-3;

    //erase the square displayed in previous frame
    SDL_FillRect(itsScreen, &preRect, getGreyUint32());

    //display new square
    SDL_FillRect(itsScreen, &rect, color);

    waitNextRequestedVsync(false, true);
    SDL_UpdateRect(itsScreen, preRect.x, preRect.y, preRect.w, preRect.h);
    SDL_UpdateRect(itsScreen, rect.x, rect.y, rect.w, rect.h);

    // update previous rectangle location to the current one
    preRect.x = rect.x; preRect.y = rect.y;

    // log moving square position:
    pushEvent(sformat("smoothPursuit_squarePosition at (%d, %d)", p.i, p.j));
  }

  // erase the lastest square displayed
  SDL_FillRect(itsScreen, &preRect,
               getUint32color(itsBackgroundColor.getVal()));

  waitNextRequestedVsync(false, true);
  SDL_UpdateRect(itsScreen, preRect.x, preRect.y, preRect.w, preRect.h);
}

// ######################################################################
void PsychoDisplay::displaySmoothPursuitFancyTrace(int location[][2], int num_loc,
                float speed[], int stay[], uint color)
{
  pushEventBegin("Display Fancy Trace for Smooth Pursuit");
  clearScreen();

  // start the eye-tracker
  //  itsEyeTracker->track(true);

  // foreach transition location
  for(int i=0; i<num_loc-1; i++){
    // stay
    pushEventBegin(sformat("Stay at location (%d, %d)", location[i][0], location[i][1]));
    SDL_Rect rect; rect.w = 7; rect.h = 7;
    rect.x=location[i][0]-3; rect.y=location[i][1]-3;
    SDL_FillRect(itsScreen, &rect, color);
    SDL_UpdateRect(itsScreen, rect.x, rect.y, rect.w, rect.h);
    for(int j=0; j<stay[i]; j++){
      waitNextRequestedVsync(false, true);
    }
    SDL_FillRect(itsScreen, &rect, getGreyUint32());
    SDL_UpdateRect(itsScreen, rect.x, rect.y, rect.w, rect.h);
    pushEventEnd(sformat("Stay at location (%d, %d)", location[i][0], location[i][1]));

    // move
    pushEventBegin(sformat("Move from (%d, %d) to (%d, %d) at %f pixel/frame",
                    location[i][0], location[i][1], location[i+1][0],
                    location[i+1][1], speed[i]));
    displaySmoothPursuitCalibration(location[i][0], location[i][1],
                                 location[i+1][0], location[i+1][1], speed[i],color);
    pushEventEnd(sformat("Move from (%d, %d) to (%d, %d) at %f pixel/frame",
                    location[i][0], location[i][1], location[i+1][0],
                    location[i+1][1], speed[i]));
  }

  // stay for the last location
  int l = num_loc-1;
  pushEventBegin(sformat("Stay at location (%d, %d)", location[l][0], location[l][1]));
  SDL_Rect rect; rect.w = 7; rect.h = 7;
  rect.x=location[l][0]-3; rect.y=location[l][1]-3;
  SDL_FillRect(itsScreen, &rect, color);
  SDL_UpdateRect(itsScreen, rect.x, rect.y, rect.w, rect.h);
  for(int j=0; j<stay[l]; j++){
  waitNextRequestedVsync(false, true);
  }
  SDL_FillRect(itsScreen, &rect, getGreyUint32());
  SDL_UpdateRect(itsScreen, rect.x, rect.y, rect.w, rect.h);
  pushEventEnd(sformat("Stay at location (%d, %d)", location[l][0], location[l][1]));

  // stop the eye-tracker
  // itsEyeTracker->track(false);
  clearScreen();

  pushEventEnd("Display Fancy Trace for Smooth Pursuit");
}




// ######################################################################
void PsychoDisplay::displayEyeTrackerCalibration(const int nptshoriz,
                                                 const int nptsvertic,
                                                 const int timefactor,
                                                 const bool mouserespond )
{
  if (itsEyeTracker.isValid() == false)
    LFATAL("You need to set an EyeTracker using setEyeTracker() first");

  pushEventBegin("EyeTrackerCalibration");
  int w = itsDims.getVal().w(), h = itsDims.getVal().h();
  int deltax = w / (nptshoriz + 1), deltay = h / (nptsvertic + 1);

  // list all the points we want:
  std::vector<Point2D<int> > pts;
  for (int j = deltay-1; j < h - deltay; j += deltay)
    for (int i = deltax-1; i < w - deltax; i += deltax)
      pts.push_back(Point2D<int>(i, j));

  // randomize these babies:
  for (uint i = 0; i < pts.size(); i ++)
    {
      uint j = i + randomUpToNotIncluding(pts.size() - i);
      Point2D<int> tmp = pts[i]; pts[i] = pts[j]; pts[j] = tmp;
    }

  // let's loop over the points and display them:
  SDL_Rect rect; rect.w = 3; rect.h = 3;
  SDL_Rect rect2; rect2.w = 1; rect2.h = 1;
  while (pts.size()) {
    // get current point:
    Point2D<int> p = pts.back(); pts.pop_back();
    rect.x = p.i - 1; rect.y = p.j - 1; rect2.x = p.i; rect2.y = p.j;

    // show fixation and wait for key:
    clearScreen();
    displayFixation();
    if(!mouserespond){
            waitForKey();
    }else{
            waitForMouseClick();
    }

    waitNextRequestedVsync(false, true);

    // start the eye tracker:
    itsEyeTracker->track(true);

    // blink the fixation:
    displayFixationBlink(-1, -1, 5, 2*timefactor);

    // log fixation position:
    pushEventBegin(sformat("eyeTrackerCalibration at (%d, %d)", p.i, p.j));

    // let's flash a marker at the desired position:
    for (int k = 0; k < 9; k ++) {
      SDL_FillRect(itsScreen, &rect, getBlackUint32());
      SDL_FillRect(itsScreen, &rect2, getWhiteUint32());
      syncScreen(true, false, true);
      for (int i = 0; i < 2*timefactor; ++i)
        waitNextRequestedVsync(false, true); // sleep a bit

      SDL_FillRect(itsScreen, &rect, getWhiteUint32());
      SDL_FillRect(itsScreen, &rect2, getBlackUint32());
      syncScreen(true, false, true);
      for (int i = 0; i < 2*timefactor; ++i)
        waitNextRequestedVsync(false, true); // sleep a bit
    }
    pushEventEnd(sformat("eyeTrackerCalibration at (%d, %d)", p.i, p.j));

    // stop the eye tracker:
    itsEyeTracker->track(false);
  }

  // done!
  clearScreen();
  pushEventEnd("EyeTrackerCalibration");
}

// ######################################################################
void PsychoDisplay::displayRandomText(const int stringlength, const int fontsize, const bool vsync , int ind )
{
	// generating a random string
	char random_str[stringlength];
	for(int i=0; i<stringlength; i++){
		random_str[i] = 65 + (int)(26 * (rand() / (RAND_MAX + 1.0)));
	}
	std::string msg (random_str);
	LINFO("random text: %s", msg.c_str());


	// display the string
  SDLdisplay::displayText(msg, vsync, itsTextColor.getVal(),
                          itsBackgroundColor.getVal(), ind, fontsize );
}

// ######################################################################
void PsychoDisplay::displayText(const std::string& msg, const bool vsync , int ind, const int fontsize )
{
  SDLdisplay::displayText(msg, vsync, itsTextColor.getVal(),
                          itsBackgroundColor.getVal(), ind, fontsize );
}

// ######################################################################
void PsychoDisplay::displayText(const std::string& msg, const Point2D<int>& p , const PixRGB<byte> txtcol ,const PixRGB<byte> bgcol , const bool vsync)
{
  SDLdisplay::displayText(msg, p,txtcol ,bgcol,vsync) ;
}

// ######################################################################
PixRGB<byte> PsychoDisplay::getGrey() const
{ return itsBackgroundColor.getVal(); }

// ######################################################################
void PsychoDisplay::drawCloud(DOT *clouds, const int numDots,
                              const Uint8 r, const Uint8 g, const Uint8 b,
                              const bool vsync)
{
  // Map the color white to this display (R=0xff, G=0xFF, B=0xFF)
  // Note:  If the display is palettized, you must set the palette first.
  Uint32 color = getUint32color(PixRGB<byte>(r, g, b));

  // Lock the screen for direct access to the pixels
  if ( SDL_MUSTLOCK(itsScreen) ) {
    if ( SDL_LockSurface(itsScreen) < 0 ) {
      LINFO("Can't lock screen: %s", SDL_GetError());
      return;
    }
  }

  for (int j = 0; j < 25; j ++){
    for (int i = 0; i < numDots; i++){
      DOT* dot = clouds + j * numDots + i;
      if (dot != NULL) putPixel32(dot->x, dot->y, color);
      }
    }

  if ( SDL_MUSTLOCK(itsScreen) )
    SDL_UnlockSurface(itsScreen);

  if (vsync) syncScreen(vsync, true, false);
}

// ######################################################################
void PsychoDisplay::drawClouds(DOT *newClouds, DOT *oldClouds,
                               const int numDots,
                               const Uint8 r, const Uint8 g, const Uint8 b,
                               const bool vsync)
{
  // Map the color white to this display (R=0xff, G=0xFF, B=0xFF)
  // Note:  If the display is palettized, you must set the palette first.
  Uint32 color = getUint32color(PixRGB<byte>(r, g, b));
  Uint32 black = getUint32color(PixRGB<byte>(0, 0, 0));

  // Lock the screen for direct access to the pixels
  if ( SDL_MUSTLOCK(itsScreen) ) {
    if ( SDL_LockSurface(itsScreen) < 0 ) {
      LINFO("Can't lock screen: %s", SDL_GetError());
      return;
    }
  }

  for (int j = 0; j < 25; j ++){
    for (int i = 0; i < numDots; i++){
      DOT* newdot = newClouds + j * numDots + i;
      putPixel32(newdot->x, newdot->y, color);
      DOT* olddot = oldClouds + j * numDots + i;
      putPixel32(olddot->x, olddot->y, black);
    }
  }
  if ( SDL_MUSTLOCK(itsScreen) )
    SDL_UnlockSurface(itsScreen);

  if (vsync) syncScreen(vsync, true, false);
}

// ######################################################################
int PsychoDisplay::displayNumbers(const int targetRow,
                                   const int targetCol, const bool vsync)
{
  // let's get a white image:
  Image<PixRGB<byte> > cimg(itsDims.getVal(), NO_INIT);
  cimg.clear(itsBackgroundColor.getVal());

  // randomize the cell indices
  int index[25];
  for (int i = 0; i < 25; i++)
    index[i] = i;
  randShuffle (index, 25);

  int targetNum = -1;
  // for each random cell, display a number
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++){
      int row = index[i*5+j]/5, col = index[i*5+j]%5;
      Point2D<int> p; p.i = col * 128 + 64; p.j = row * 96 + 48;
      if (p.i < 0) LERROR("Text does not fit on screen!");
      char msg[5];
      snprintf(msg, sizeof(msg), "%d%d", i+1, j+1);
      writeText(cimg, p, msg,
                itsTextColor.getVal(),
                itsBackgroundColor.getVal());
      if (row == targetRow && col == targetCol)
        targetNum = (i+1)*10 + (j+1);
    }
  // let's convert it to something we can blit:
  SDL_Surface *surf =
    SDL_CreateRGBSurfaceFrom(cimg.getArrayPtr(), cimg.getWidth(),
                             cimg.getHeight(), 24, 3 * cimg.getWidth(),
                             0x0000ff, 0x00ff00, 0xff0000, 0x0);
  SDL_Surface *surf2 = SDL_DisplayFormat(surf);
  SDL_FreeSurface(surf);
  displaySurface(surf2, -1, vsync);
  SDL_FreeSurface(surf2);
  return targetNum;
}


// ######################################################################
int PsychoDisplay::drawPoint(const Point2D<int> thePoint)
{
  if (itsEyeTracker.isValid() == false)
    LFATAL("You need to set an EyeTracker using setEyeTracker() first");

  //int w = itsDims.getVal().w(), h = itsDims.getVal().h();


   // let's loop over the points and display them:
  SDL_Rect rect; rect.w = 10; rect.h = 10; rect.x=thePoint.i; rect.y = thePoint.j;

    // log fixation position:
    pushEventBegin(sformat("eye tracker instantaneous eye position %d, %d", thePoint.i, thePoint.j));

      SDL_FillRect(itsScreen, &rect, getUint32color(PixRGB<byte>(255, 0, 0)));

      syncScreen(true, false, true);
      SDL_UpdateRect(itsScreen, rect.x, rect.y, rect.w, rect.h);
       waitNextRequestedVsync(false, true); // sleep a bit
       return 1;
}


//######################################################################

int PsychoDisplay::drawPointColor(const Point2D<int> thePoint, PixRGB<byte> color)
{
  if (itsEyeTracker.isValid() == false)
    LFATAL("You need to set an EyeTracker using setEyeTracker() first");

  //int w = itsDims.getVal().w(), h = itsDims.getVal().h();


   // let's loop over the points and display them:
  SDL_Rect rect; rect.w = 10; rect.h = 10; rect.x=thePoint.i; rect.y = thePoint.j;

    // log fixation position:
    pushEventBegin(sformat("eye tracker instantaneous eye position %d, %d", thePoint.i, thePoint.j));

      SDL_FillRect(itsScreen, &rect, getUint32color(color));

      syncScreen(true, false, true);
      SDL_UpdateRect(itsScreen, rect.x, rect.y, rect.w, rect.h);
       waitNextRequestedVsync(false, true); // sleep a bit
       return 1;
}

//#############################################################################
int PsychoDisplay::drawCalibPoint(const Point2D<int> thePoint)
{
  SDL_Rect rect; rect.w = 5; rect.h = 5;
  SDL_Rect rect2; rect2.w = 3; rect2.h = 3;
  rect.x = thePoint.i - 1; rect.y = thePoint.j - 1; rect2.x = thePoint.i; rect2.y = thePoint.j;
  SDL_FillRect(itsScreen, &rect, getBlackUint32());
  SDL_FillRect(itsScreen, &rect2, getWhiteUint32());
  return 1;
}

// ######################################################################
Uint32 PsychoDisplay::getGreyUint32() const
{ return getUint32color(itsBackgroundColor.getVal()); }

// ######################################################################
Uint32 PsychoDisplay::getBlackUint32() const
{ return getUint32color(itsBlack.getVal()); }

// ######################################################################
Uint32 PsychoDisplay::getWhiteUint32() const
{ return getUint32color(PixRGB<byte>(itsWhite.getVal())); }

// ######################################################################
void PsychoDisplay::changeBackgroundColor(PixRGB<byte> c)
{ itsBackgroundColor.setVal(c); }

#endif // HAVE_SDL_SDL_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
