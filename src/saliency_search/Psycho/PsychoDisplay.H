/*!@file Psycho/PsychoDisplay.H Display psychophysics stimuli */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/PsychoDisplay.H $
// $Id: PsychoDisplay.H 14176 2010-10-28 04:28:19Z ilink $
//

#ifndef PSYCHODISPLAY_H_DEFINED
#define PSYCHODISPLAY_H_DEFINED

#ifdef HAVE_SDL_SDL_H

#include "GUI/SDLdisplay.H"

class EyeTracker;

//! for drawing clouds of dots
typedef struct{
  int x;
  int y;
  int coherent;
  int age;
  int dx;
  int dy;
} DOT;

//! Class fo do various psychophysics displays
/*! This class is to facilitate the display of various psychophysics
  stimuli, with an initial emphasis on playing movies at a controlled
  framerate. Programs using this class, such as psycho-movie.C, should
  run as root if SCHED_FIFO scheduling is required (and I highly
  recommend it, as it will make timing reliable). The class uses the
  SDLdisplay class to do the displays, and PsychoDisplay derives from
  SDLdisplay. See GUI/SDLdisplay.H for basic SDL info. */

class PsychoDisplay : public SDLdisplay
{
public:
  // ######################################################################
  /*! @name Constructors, destructor and initialization */
  //@{

  //! Constructor
  PsychoDisplay(OptionManager& mgr,
                const std::string& descrName = "Psycho Display",
                const std::string& tagName = "PsychoDisplay");

  //! Destructor
  ~PsychoDisplay();

  //! Optionally we may work in conjunction with an EyeTracker
  void setEyeTracker(nub::soft_ref<EyeTracker> e);

  //!Set the size of the fixation point
  void setFixationSize(const int size);
//@}

  // ######################################################################
  /*! @name Display and interaction functions */
  //@{

  //! Clear screen using our default grey
  void clearScreen(const bool vsync = true);

	//! display a circle
  /*! @param x horizontal position of center of cross, or -1 to have
             it at the center of the screen
      @param y vertical position of center of cross, or -1 to have
             it at the center of the screen
			@param radius radius of the circle
			@param ringwidth width of the ring
			@param color of the circle
      @param vsync will attempt to sync with vertical blanking if true */
	void displayCircle(const int x = -1, const int y = -1,
                     const int radius = 10, 
                     const PixRGB<byte> color = PixRGB<byte>(255,255,255), 
										 const bool vsync = true);

	//! display a filled circle
  /*! @param x horizontal position of center of cross, or -1 to have
             it at the center of the screen
      @param y vertical position of center of cross, or -1 to have
             it at the center of the screen
			@param radius radius of the circle
			@param ringwidth width of the ring
			@param color of the circle
      @param vsync will attempt to sync with vertical blanking if true */
	void displayFilledCircle(const int x = -1, const int y = -1,
                           const int radius = 10, 
                           const PixRGB<byte> color = PixRGB<byte>(255,255,255), 
										       const bool vsync = true);

	void displayFilledCircleBlink(const int x = -1, const int y = -1,
                                const int radius = 10, 
                                const PixRGB<byte> color = PixRGB<byte>(255,255,255), 
										            const int iter = 5, const int delay = 2);

  //! display a fixation image icon
  /*! @param image icon that replaces fixation cross
      @param x horizontal position of center of cross, or -1 to have
             it at the center of the screen
      @param y vertical position of center of cross, or -1 to have
             it at the center of the screen
      @param vsync will attempt to sync with vertical blanking if true */
  void displayFixationIcon(const Image< PixRGB<byte> >& image,
                  const int x = -1, const int y = -1,
                  const bool vsync = true);

  //! blink a fixation image icon
  /*! @param image icon that replaces fixation cross
      @param iter number of times we should blink the cross
      @param delay blink duration, in frames as per our frame
      duration ModelParam */
  void displayFixationIconBlink(const Image< PixRGB<byte> >& image,
                  const int x = -1, const int y = -1,
                  const int iter = 5, const int delay = 2);

  //! display a fixation cross
  /*! @param x horizontal position of center of cross, or -1 to have
             it at the center of the screen
      @param y vertical position of center of cross, or -1 to have
             it at the center of the screen
      @param vsync will attempt to sync with vertical blanking if true */
  void displayFixation(const int x = -1, const int y = -1,
                       const bool vsync = true);

  //! blink a fixation cross
  /*! @param iter number of times we should blink the cross
      @param delay blink duration, in frames as per our frame
      duration ModelParam */
  void displayFixationBlink(const int x = -1, const int y = -1,
                            const int iter = 5, const int delay = 2);

  //! display a fixation red dot
  /*! @param x horizontal position of center of dot, or -1 to have
             it at the center of the screen
      @param y vertical position of center of cross, or -1 to have
             it at the center of the screen
      @param color RGB to specifiy the color
      @param vsync will attempt to sync with vertical blanking if true */
  void displayColorDotFixation(const int x = -1, const int y = -1,
                               const PixRGB<byte> color = PixRGB<byte>(255,255,255), 
															 const bool vsync = true);

  //! blink a fixation color dot
  /*! @param iter number of times we should blink the cross
      @param delay blink duration, in frames as per our frame
      duration ModelParam */
  void displayColorDotFixationBlink(const int x = -1, 
                                  const int y = -1,
                                  const int iter = 5, 
                                  const int delay = 2,
                                  const PixRGB<byte> color = PixRGB<byte>(255,255,255));

  //! display a fixation red dot
  /*! @param x horizontal position of center of dot, or -1 to have
             it at the center of the screen
      @param y vertical position of center of cross, or -1 to have
             it at the center of the screen
      @param vsync will attempt to sync with vertical blanking if true */
  void displayRedDotFixation(const int x = -1, const int y = -1,
                             const bool vsync = true);

  //! display a fixation white dot
  /*! @param x horizontal position of center of dot, or -1 to have
             it at the center of the screen
      @param y vertical position of center of cross, or -1 to have
             it at the center of the screen
      @param vsync will attempt to sync with vertical blanking if true */
  void displayWhiteDotFixation(const int x = -1, const int y = -1,
                               const bool vsync = true);

  //! blink a fixation red dot
  /*! @param iter number of times we should blink the cross
      @param delay blink duration, in frames as per our frame
      duration ModelParam */
  void displayRedDotFixationBlink(const int x = -1, 
                                  const int y = -1,
                                  const int iter = 5, 
                                  const int delay = 2);

  void displayRedDotFixationBlink(Image< PixRGB<byte> > img, 
                                  const int x = -1, 
                                  const int y = -1,
                                  const int iter = 5, 
                                  const int delay = 2);

  //! display a static low-level 5-point ISCAN calibration array
  void displayISCANcalib();

  //! display a bunch of eye-tracker calibration dots
  /*! Caution: calling this function will lead to a fatal error if
    setEyeTracker() has not been called previously with a valid
    EyeTracker. */
  void displayEyeTrackerCalibration(const int nptshoriz = 7,
                                    const int nptsvertic = 5,
                                    const int timefactor = 1,
                                    const bool mouserespond = false);


  //!display a dot moving on a bacground image
  void displayMovingDotBackground(SDL_Surface *img,
                                  const int startX = 320, 
                                  const int startY = 240,
                                  const int endX = 320, 
                                  const int endY = 100,
                                  const float speed = 10.0, 
                                  const PixRGB<byte> color = PixRGB<byte>(255,255,255));

  //!string together many dot animations
  void displayMovingDotTrain(const Image< PixRGB<byte> >& img, 
                             int location[][2], 
                             int num_loc, 
                             float speed[], 
                             int stay[], 
                             const PixRGB<byte> color = PixRGB<byte>(255,255,255));

  //! display a square that moves along the path in different speed each time
  /*! @param location points that segment path. The first index is point
      and the second index is the x or y position of the point
      @param num_loc number of locations specified
      @param speed pixels/frame. If more than one speed is specified, then
            it displays the same path with differnt speed
      @param num_speed number of speeds specified */
  void displaySmoothPursuitGroupCalibration(int location[][2],
                  int num_loc, float speed[], int num_speed);

  //! display a square moving from start point to end point for
  //  eye-tracker calibration by smooth pursuit
  /*! @param startX x position of the starting point
      @param startY y position of the starting point
      @param endX x position of the end point
      @param endY y position of the end point
      @param speed pixels/secs */
  void displaySmoothPursuitCalibration(const int startX = 10, const int startY = 10,
                  const int endX = 500, const int endY = 500, const float speed = 10.0, uint color = 0);

  //! display a square moving along the path with different speed from transition location
  //  to location, and stay different duration at each transition point
  /*! @param location transition points that segment path. The first index is point
      and the second index is the x or y position of the point
      @param num_loc number of transition points specified
      @param speed pixels/frame. It specify the speed from each transition point to point
      @param stay number of frames to stay
   */
  void displaySmoothPursuitFancyTrace(int location[][2], int num_loc, float speed[], int stay[], uint color);

  //! display a cloud of dots
  void drawCloud(DOT *clouds, const int numDots = -1,
                 const Uint8 r = 0xff, const Uint8 g = 0xff,
                 const Uint8 b = 0xff, const bool vsync = true);

  //! display a cloud of dots
  void drawClouds(DOT *newClouds, DOT *oldClouds, const int numDots = -1,
                  const Uint8 r = 0xff, const Uint8 g = 0xff,
                  const Uint8 b = 0xff, const bool vsync = true);

  //! show a text message using our default colors
  //! show a text message
   /*! @param ind ind=0 displays the text in the middle of screen, ind =1 displays the text on top or the screen
  and ind = -1 displays the message at the buttom of the display, ind=-2 for random placement of the message
    */
  void displayText(const std::string& msg, const bool vsync = true , int ind = 0 , const int fontsize = 10);

 	void displayText(const std::string& msg, const Point2D<int>& p , const PixRGB<byte> txtcol ,const PixRGB<byte> bgcol , const bool vsync=true) ;

	void displayRandomText(const int stringlength = 8, const int fontsize = 6, const bool vsync = true, int ind = 0);

  //! display a 5x5 grid of random numbers
  /*! This is flashed after every trial and subject is asked to report
   the number that appeared at the target location.*/
  int displayNumbers(const int targetRow = -1,
                   const int targetCol = -1, const bool vsync = true);

  int drawPoint(const Point2D<int> thePoint);

  int drawPointColor(const Point2D<int> thePoint,PixRGB<byte> color);

  int drawCalibPoint(const Point2D<int> thePoint);

  // ######################################################################
  /*! @name Access functions */
  //@{

  //! Get background grey value
  PixRGB<byte> getGrey() const;

  //! change background color
  void changeBackgroundColor(PixRGB<byte> c);


  //@}

protected:

  //! Get our background grey as Uint32 color (for use with SDL)
  Uint32 getGreyUint32() const;

  //! Get our black as Uint32 color (for use with SDL)
  Uint32 getBlackUint32() const;

  //! Get our white as Uint32 color (for use with SDL)
  Uint32 getWhiteUint32() const;

  OModelParam<PixRGB<byte> > itsBackgroundColor; //!< background grey color
  OModelParam<PixRGB<byte> > itsTextColor; //!< text foreground color
  OModelParam<PixRGB<byte> > itsBlack;     //!< "black" color
  OModelParam<PixRGB<byte> > itsWhite;     //!< "white" color
  OModelParam<std::string> itsFixationIcon;     //!< image icon for fixation

  NModelParam<int> itsFixSiz;     //!< size of fixation cross
  NModelParam<int> itsFixThick;   //!< thickness of fixation cross

private:
  nub::soft_ref<EyeTracker> itsEyeTracker;
};

#endif // HAVE_SDL_SDL_H

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
