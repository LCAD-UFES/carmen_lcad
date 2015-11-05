/*!@file Psycho/DisplayController.H a thread to abstract the gathering
   of stimulus commands from a shared buffer, and render them into the
   current SDL display. */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/DisplayController.H $

#ifndef PSYCHO_DISPLAYCONTROLLER_H_FEFINED
#define PSYCHO_DISPLAYCONTROLLER_H_FEFINED

#include "Component/EventLog.H"
#include "Util/JobServer.H"
#include "Util/SharedBuffer.H"
#include "Util/Semaphore.H"
#include "Raster/GenericFrame.H"
#include "Psycho/PsychoDisplay.H"
#include "Psycho/StimData.H"

#include <map>

// ######################################################################
//!A thread to control the psycho display from an external controller
// ######################################################################
/*!This class is to be used by a StimController to to control the SDL
   display. Display commands are read through a shared buffer. Calling
   programs should no longer try to handle display themselves, but
   merely call the setBackground() to 'inject' the calling programs
   image into the DisplayController thread. The image will be rendered
   on the next display cycle in the background of the
   DisplayControllers current drawing instructions. */

class DisplayController : public JobServer::Job
{
public:
  //!c'tor
  DisplayController();

  //!d'tor
  virtual ~DisplayController();

  //!set the display from the main calling programs display
  void setDisplay(nub::soft_ref<PsychoDisplay> display);

  //!set the buffer to be shared with StimListener
  void setBuffer(rutz::shared_ptr<SharedBuffer<RexData*> > buffer);

  //!set the log file
  void setEventLog(nub::soft_ref<EventLog> log);

  //!push an event to the log, if uselinfo is true also display it to the screen
  void pushEvent(const std::string& msg, const bool& uselinfo = false);

  //!halt the display controller's run loop
  void pause(bool pausestate);

  //!set the current background
  void setBackground(const GenericFrame& gf, const int framenumber);

  //! start the componenet and do any pre-display stuff.
  virtual void start();

  //!clean up and stop
  virtual void stop();

  // ######################################################################
  // from JobServer::Job
  // ######################################################################
  //!the actual work of the thread is here
  virtual void run();

  //!job name
  virtual const char* jobType() const;

  //!return priority, 0 is highest
  virtual int priority() const;

private:
  // ######################################################################
  // private drawing primitives for rendering onto the current SDL display
  // ######################################################################
  //! wait for refresh draw, and reset the backGround image;
  void draw(const byte* parameters = NULL);

  //! draw a circle on the sdl Display
  void drawCircle(const byte* parameters);

  //!clear the current screen
  void clearScreen(const byte* parameters);

  //!create a pop-out array, with possible multiple targets
  void createPopout(const byte* parameters);

  // ######################################################################
  // our private and functions
  // ######################################################################
  DisplayController& operator=(const DisplayController& d);
  DisplayController(const DisplayController& d);

  //!Display a video frame with an image overlayed, ignoring the
  //!transparent pixels.
  void overlayVideo(const Image<PixRGB<byte> >& over, const VideoFrame& frame,
                    const PixRGB<byte>& transparentcolor, const uint framenum);

  //!our shared display
  nub::soft_ref<PsychoDisplay> itsDisplay;

  //! a shared buffer
  rutz::shared_ptr<SharedBuffer<RexData*> > itsBuffer;

  //! a log file
  nub::soft_ref<EventLog> itsLog;

  //! suspend until reset.
  volatile bool isPaused;

  //!a flag to see if someone told us to stop
  volatile bool isDone;

  //!a flag set when the rex system has called a vsync
  bool isRexReady;

  //!This flag controls who actually calls syncScreen to flip the
  //!backbuffer. Normally, for static backgrounds Rex will decide when
  //!to render graphics to the screen, by sending a vertical blank
  //!code which will call our draw() function. When displaying video,
  //!to ensure proper frame rate, setBackground calls the syncscreen
  //!checking isRexReady and blitting any rex image before actually
  //!flipping the buffer.
  bool isVideoStream;

  //!just to count the number of frames sent from rex
  long itsFrameCount;

  //!to hold our background image, or video frame
  VideoFrame itsBackgroundFrame;

  //!the current rex drawing.
  Image<PixRGB<byte> > itsRexImage;

  //!A semaphore to actual do the pausing.
  Semaphore itsPauser;

  //! A map to store function pointers to primitive drawing routines,
  //! that are keyed by the byte code from the Rex system that
  //! indicates the primitive type (e.g. circle, square etc)
  std::map<unsigned char, void (DisplayController::*)(unsigned char*)> itsFuncMap;
};

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
