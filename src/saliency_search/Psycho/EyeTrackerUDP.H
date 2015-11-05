/*!@file Psycho/EyeTrackerUDP.H Eye tracker for Doug munoz lab that
   uses UDP packets instead of the parallel port */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerUDP.H $

#ifndef PSYCHO_EYETRACKERUDP_H_DEFINED
#define PSYCHO_EYETRACKERUDP_H_DEFINED

#define BUFSIZE 128

#include "Psycho/EyeTracker.H"
#include <sys/socket.h>
#include <netdb.h>

//! Interface to eye-tracker used in Doug Munoz' lab
/*! This is a monkey scleral search coil type, with trigger over
    ethernet (UDP packets) from the REX experimental control system. */
class EyeTrackerUDP : public EyeTracker {
public:
  //! Constructor
  EyeTrackerUDP(OptionManager& mgr,
                const std::string& descrName = "Eye Tracker UDP",
                const std::string& tagName = "EyeTrackerUDP");
  //! Destructor
  virtual ~EyeTrackerUDP();

  //! Start the tracker
  /*! Tracking is started by sending a packet to REX. */
  void startTracking();

  //! Stop the tracker
  /*! Tracking is stopped by sending a different packet. */
  void stopTracking();

  //! Is the subject fixating?
  bool isFixating();

  //! Did the subject initiate a saccade?
  bool isSaccade();
  
  //! clear all our eye position (isFixating and isSaccade) states?
  void clearEyeStatus();
  
  //! Get current eye position
  /*! Not supported by this tracker. */
  Point2D<int> getEyePos() const;

  //! Get current fixation position
  /*! Not supported by this tracker. */
  Point2D<int> getFixationPos() const;

 //!Get the calibration set
 /*! Not implemented here */
  CalibrationTransform::Data getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const;

protected:
  //! codes for eye states
  enum {FIXCODE = 249, SACCODE = 250};

  void start1(); //!< get started
  void start2(); //!< get started after subcomponents get started
  void stop1();  //!< get things stopped
  
private:
  bool checkForData(); //!< Check for host data and set any internal flags
  void sendMessage(const char* msg); //!< send a message
  void pushEvent(const std::string& ev); //!< push string to event queue
  void setEyeFlags(const unsigned char msg); //!< set internal eye state flags bassed on a byte
  void resetEyeFlags(); //!< reset all eye state flags to false

  //messages and flags
  char *startMsg, *stopMsg;
  bool isConnected, isFix, isSac;

  //stuff for UDP communicaton
  int sockfd;   //our socket
  struct addrinfo *myInfo, *hostInfo; //address info
  struct sockaddr lasthost_addr;  //info about the last host
  socklen_t lasthost_addrlen; //last host address length
  fd_set master, read_fs;    // master, and reading file descriptor list
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKERUDP_H_DEFINED
