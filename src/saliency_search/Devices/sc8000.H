/*!@file sc8000.H Interface to a serial-servo-controller
 derived from ssc
 */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/sc8000.H $
// $Id: sc8000.H 6990 2006-08-11 18:13:51Z rjpeters $
//

#ifndef SC8000_H_DEFINED
#define SC8000_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

class Serial;

#define SSCNUMSERVOS 8

//! Interface to a serial-servo-controller
/*! The sc8000.C and sc8000.H files has the sc-8000 controller code that
  help control the servos via the controller.  The class is SC8000
  class. Its functions include moving a servo to a specified position,
  using a serial port. There are two ways to move the servos and thet
  their current positions: either using calibrated values in
  [-1.0..1.0], or using raw positions in [0..65535]. Calibrated
  positions are converted to raw using two linear ramps (one for each
  side of the neutral position) that can be set using the calibrate()
  function. EXAMPLE: if you do a calibrate(servo, 100, 20, 150), then,
  subsequently, move(servo, 0.0F) will be equivalent to moveRaw(servo,
  100), move(servo, -1.0F) will be equivalent to moveRaw(servo, 20)
  and move(servo, 1.0F) will be equivalent to moveRaw(servo, 150). */
class SC8000 : public ModelComponent
{
public:
  //! Default constructor; see ModelComponent.H
  SC8000(OptionManager& mgr,
      const std::string& descrName = "SC8000 Servo Driver",
      const std::string& tagName = "SC8000driver",
      const char *defdev = "/dev/ttyUSB0");

  //! Destructor
  ~SC8000();

  //! Moves servo # to given position in [-1.0 .. 1.0]
  /*! Returns true on success, false if some serial error occurred. */
  bool move(const int servo, const float position);

  //! Gets the current position of given servo
  float getPosition(const int servo) const;

  //! Calibrate a servo
  /*! Calibration will be made so that move(servo, 0.0F) will send the
    value 'neutralval' to the servo, move(servo, -1.0F) will send
    minval and move(servo, 1.0F) will send maxval. */
  void calibrate(const int servo, const short neutralval, const short minval,
                 const short maxval);

  //! Moves servo # to given RAW (uncalibrated) position in [0..65535]
  /*! Returns true on success, false if some serial error occurred. */
  bool moveRaw(const int servo, const short rawpos);

  //! Gets the current RAW (uncalibrated) position [0..65535] of given servo
  short getPositionRaw(const int servo) const;

protected:
  nub::soft_ref<Serial> itsPort;   //!< Serial port to use
  rutz::shared_ptr<NModelParam<float> > *zero;     //!< zero calibration values
  rutz::shared_ptr<NModelParam<float> > *posmult;  //!< positive multiplier calib
  rutz::shared_ptr<NModelParam<float> > *negmult;  //!< negative multiplier calib
  short *pos;                   //!< raw servo positions [0..65535]

  //! Convert from raw (0..65535) to calibrated (-1.0..1.0) position
  float rawToCalib(const int servo, const short rawpos) const;

  //! Convert from calibrated (-1.0..1.0) to raw (0..65535) position
  short calibToRaw(const int servo, const float position) const;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
