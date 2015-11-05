/*!@file Psycho/DisplayController.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/DisplayController.C $

#include "Psycho/DisplayController.H"

// ######################################################################
DisplayController::DisplayController() : itsDisplay(), itsBuffer(),
                                         itsLog(), isPaused(false),
                                         isDone(false), isRexReady(false),
                                         isVideoStream(), itsFrameCount(0),
                                         itsBackgroundFrame(), itsRexImage(),
                                         itsPauser(), itsFuncMap()
{
}

// ######################################################################
DisplayController::~DisplayController()
{
}

// ######################################################################
void DisplayController::setDisplay(nub::soft_ref<PsychoDisplay> d)
{
  itsDisplay = d;
}

// ######################################################################
void DisplayController::setBuffer(rutz::shared_ptr<SharedBuffer<RexData*> > buff)
{
  itsBuffer = buff;
}

// ######################################################################
void DisplayController::setEventLog(nub::soft_ref<EventLog> log)
{
  itsLog = log;
}

// ######################################################################
void DisplayController::pushEvent(const std::string& msg, const bool& useLinfo)
{
  if (itsLog.isValid())
    itsLog->pushEvent(msg);

  if (useLinfo)
    LINFO(msg.c_str());
}

// ######################################################################
void DisplayController::pause(bool pausestate)
{
  isPaused = pausestate;
  if (itsDisplay->hasYUVoverlay())
    itsDisplay->destroyYUVoverlay();
  if (!isPaused)
    itsPauser.post();
  itsFrameCount = 0;
}

// ######################################################################
void DisplayController::setBackground(const GenericFrame& gf,
                                      const int framenum)
{
  //if we don't already have an overlay, set one up
  if (!itsDisplay->hasYUVoverlay())
    itsDisplay->createVideoOverlay(VIDFMT_YUV420P,gf.getWidth(),
                                   gf.getHeight());

  //get our frame as a video, converting as necessary
  VideoFrame vid = gf.asVideo();

  //are we in video streaming mode or what?
  if (gf.nativeType() == GenericFrame::VIDEO)
    {
      //this means the caller is in control and we display now.
      isVideoStream = true;
      if (isRexReady)
        {
          isRexReady=false;
          overlayVideo(itsRexImage, vid, PixRGB<byte>(1,1,1), framenum);
        }
      else
        itsDisplay->displayVideoOverlay(vid, framenum, SDLdisplay::NEXT_VSYNC);
    }
  else
    {
      //this means rex is in control, we display when it says so
      isVideoStream = false;
      //save a copy for later, will be used by draw()
      itsBackgroundFrame = VideoFrame::deepCopyOf(vid);
    }
  itsFrameCount = framenum;
}

// ######################################################################
void DisplayController::start()
{
  if ( !itsBuffer.is_valid() ||  !itsDisplay.isValid())
    LFATAL("I need a buffer and a display to start");
  isDone = false;

}

// ######################################################################
void DisplayController::stop()
{
  pause(false);

  if (itsDisplay->hasYUVoverlay())
    itsDisplay->destroyYUVoverlay();

  isDone = true;
  itsBuffer->stopWaiting();
}

// ######################################################################
void DisplayController::run()
{
  //loop until someone tells us to quite or the buffer returns NULL data
  while (!isDone)
    {
      //check to see if someone told us to pause;
      if (isPaused)
        {
          itsPauser.wait();
        }

      //start popping data off of the queue, waiting if no data
      RexData* d = itsBuffer->pop();

      //if our buffer is returning empty data lets exit
      if (d->buflen == 0)
        {
          isDone = true;
          delete d;
          break;
        }

      //!once we have the data look up the function in our map, calling it
      (this->*itsFuncMap[d->status])(d->buffer);

      //!delete the data as we are done with it
      delete d;
    }
}

// ######################################################################
const char* DisplayController::jobType() const
{
  return "DisplayController";
}

// ######################################################################
int DisplayController::priority() const
{
  return 0;
}

// ######################################################################
void DisplayController::draw(const byte* parameters)
{
  //if we are in video streaming mode just flip the ready flag
  if (isVideoStream)
    isRexReady = true;
  else
    {
      VideoFrame tmp = itsBackgroundFrame;
      overlayVideo(itsRexImage, tmp, PixRGB<byte>(1,1,1), itsFrameCount);
      itsFrameCount++;
    }
}

// ######################################################################
void DisplayController::drawCircle(const byte* parameters)
{

}

// ######################################################################
void DisplayController::clearScreen(const byte* parameters)
{
  itsDisplay->displayVideoOverlay(itsBackgroundFrame, itsFrameCount,
                                  SDLdisplay::NEXT_VSYNC);
  itsFrameCount++;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


