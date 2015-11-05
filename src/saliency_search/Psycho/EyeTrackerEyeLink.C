/*!@file Psycho/EyeTrackerEyeLink.C Abstraction for an EyeLink eye-tracker */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerEyeLink.C $
// $Id: EyeTrackerEyeLink.C 15106 2011-12-10 02:12:08Z pohetsn $
//

#include "Psycho/EyeTrackerEyeLink.H"

#include "Component/OptionManager.H"
#include "GUI/GUIOpts.H"
#include "Psycho/PsychoOpts.H"
#include "Util/sformat.H"
#include "Component/EventLog.H"
#include "Psycho/PsychoDisplay.H"
#include <SDL/SDL.h>
#include "Raster/Raster.H"

#ifdef HAVE_EYELINK
#include <eyelink/eyelink.h>
#include <eyelink/sdl_expt.h>
#include <eyelink/sdl_text_support.h>
ALLF_DATA evt; //buffer to hold sample and event data
#endif

// used by: eyelink only
const ModelOptionDef OPT_EyelinkBackgroundColor =
  { MODOPT_ARG(PixRGB<byte>), "EyelinkBackgroundColor", &MOC_EYETRACK, OPTEXP_CORE,
    "Background grey color for Eyelink1000 calibration or drift correction",
    "eyelink-background-color", '\0', "<r,g,b>", "0,0,0" };

// used by: eyelink only
const ModelOptionDef OPT_EyelinkForegroundColor =
  { MODOPT_ARG(PixRGB<byte>), "EyelinkForegroundColor", &MOC_EYETRACK, OPTEXP_CORE,
    "Foreground color for Eyelink1000 calibration or drift correction",
    "eyelink-foreground-color", '\0', "<r,g,b>", "192,192,192" };


SDL_Color target_background_color = { 0, 0, 0};
SDL_Color target_foreground_color = { 192, 192, 192 };

// ######################################################################
EyeTrackerEyeLink::EyeTrackerEyeLink(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName) :
  EyeTracker(mgr, descrName, tagName),
  itsEDFfname(&OPT_EyeTrackerEDFfname, this),
  itsDims(&OPT_SDLdisplayDims, this),
	itsEyelinkBackgroundColor(&OPT_EyelinkBackgroundColor, this),
	itsEyelinkForegroundColor(&OPT_EyelinkForegroundColor, this)
{ 
	/* 
	target_background_color.r = itsEyelinkBackgroundColor.getVal().red();
	target_background_color.g = itsEyelinkBackgroundColor.getVal().green();
	target_background_color.b = itsEyelinkBackgroundColor.getVal().blue();
	*/
}

// ######################################################################
EyeTrackerEyeLink::~EyeTrackerEyeLink()
{  }

// ######################################################################
void EyeTrackerEyeLink::start1()
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
  // get the link to the eye-tracker going:
  if (open_eyelink_connection(0))
    LFATAL("Cannot open link to EyeLink tracker - make sure your "
           "hostname resolves to 100.1.1.2 (check /etc/hosts)");

  // Basic initializations:
  set_offline_mode();
  flush_getkey_queue();/* initialize getkey() system */

  // Open EyeLink version of SDL
  openSDL();

  // set the display characteristics: 
  DISPLAYINFO di; // defined in eyelink/core_expt.h
  di.width = itsDims.getVal().w(); di.height = itsDims.getVal().h();
  di.left = 0; di.top = 0; di.right = di.width-1; di.bottom = di.height-1;
  di.bits = 24; di.palsize = 0; di.pages = 0; di.refresh = 60.0F;
  di.winnt = 0;

  set_calibration_colors(&target_foreground_color, &target_background_color);

	// open EDF file if set:
  if (itsEDFfname.getVal().empty() == false)
    {
      if (open_data_file((char *)(itsEDFfname.getVal().c_str())) != 0)
        LFATAL("Cannot open EDF file '%s'", itsEDFfname.getVal().c_str());

      eyecmd_printf(const_cast<char*>("add_file_preamble_text 'RECORDED BY iLab code' "));
    }

  // Add resolution to EDF file:
  eyemsg_printf(const_cast<char*>("DISPLAY_COORDS %ld %ld %ld %ld"),
                di.left, di.top, di.right, di.bottom);

  /* SET UP TRACKER CONFIGURATION */
  /* set parser saccade thresholds (conservative settings) */
  /* 0 = standard sensitivity */
  eyecmd_printf(const_cast<char*>("select_parser_configuration 0"));

  // set EDF file contents:
  eyecmd_printf(const_cast<char*>("file_event_filter = LEFT,RIGHT,FIXATION,SACCADE,"
                                  "BLINK,MESSAGE,BUTTON"));
  eyecmd_printf(const_cast<char*>("file_sample_data = LEFT,RIGHT,GAZE,AREA,GAZERES,STATUS"));

  // set link data (used for gaze cursor):
  eyecmd_printf(const_cast<char*>("link_event_filter = LEFT,RIGHT,FIXATION,FIXUPDATE,SACCADE,BLINK,BUTTON"));
  eyecmd_printf(const_cast<char*>("link_sample_data = LEFT,RIGHT,GAZE,GAZERES,AREA,STATUS"));

  // Program button #5 for use in drift correction:
  eyecmd_printf(const_cast<char*>("button_function 5 'accept_target_fixation'"));
  
	// setup eyelink filtering: default is "1 2"
	eyecmd_printf(const_cast<char*>("heuristic_filter = 1 2"));
	eyelink_wait_for_mode_ready(500);

  // make sure we're still alive:
  if (!eyelink_is_connected() || break_pressed())
    LFATAL("Connection to EyeLink broken or aborted");

  /* TRIAL_VAR_LABELS message is recorded for EyeLink Data Viewer analysis
     It specifies the list of trial variables for the trial
     This should be written once only and put before the recording of
     individual trials */
  eyemsg_printf(const_cast<char*>("TRIAL_VAR_LABELS CONDITION"));

  // Configure EyeLink to send fixation updates every 50 msec:
  //eyecmd_printf("link_event_filter = LEFT,RIGHT,FIXUPDATE");
  //eyecmd_printf("fixation_update_interval = 50");
  //eyecmd_printf("fixation_update_accumulate = 50");

  EyeTracker::start1();
#endif
}

// ######################################################################
void EyeTrackerEyeLink::openSDL()
{
#ifdef HAVE_EYELINK
  // set the display characteristics:
	DISPLAYINFO di;
  di.width = itsDims.getVal().w(); di.height = itsDims.getVal().h();
  di.left = 0; di.top = 0; di.right = di.width-1; di.bottom = di.height-1;
  di.bits = 24; di.palsize = 0; di.pages = 0; di.refresh = 60.0F;
  di.winnt = 0;

  // open the display with our wishes:
  if (init_expt_graphics(NULL, &di))
    LFATAL("Cannot open display");

  // see what we actually got:
  get_display_information(&di);
  LINFO("DISPLAYINFO: [%dx%d}: (%d,%d)-(%d,%d) %dbpp %.1fHz",
        int(di.width), int(di.height), int(di.left), int(di.top),
        int(di.right), int(di.bottom), int(di.bits), di.refresh);
  if (di.palsize) LFATAL("Paletized color modes not supported");

  // Prepare for calibration and other settings:
  set_target_size(10, 2);
  set_calibration_colors(&target_foreground_color, &target_background_color);
  set_cal_sounds(const_cast<char*>(""), const_cast<char*>(""), const_cast<char*>(""));
  set_dcorr_sounds(const_cast<char*>(""), const_cast<char*>("off"), const_cast<char*>("off"));

  // Setup calibration type:
  eyecmd_printf(const_cast<char*>("calibration_type = HV9"));

  // Configure tracker for display:
  eyecmd_printf(const_cast<char*>("screen_pixel_coords = %ld %ld %ld %ld"),
                di.left, di.top, di.right, di.bottom);

        if (di.refresh > 40.0F)
    eyemsg_printf(const_cast<char*>("FRAMERATE %1.2f Hz."), di.refresh);
#endif
}

// ######################################################################
void EyeTrackerEyeLink::stop1()
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
  set_offline_mode();
  pump_delay(500); // give some time to tracker to stop

  // close EDF data file if any:
  if (itsEDFfname.getVal().empty() == false)
    eyecmd_printf(const_cast<char*>("close_data_file"));
	

  // shutdown EyeLink properly:
  close_expt_graphics();

  // transfer the EDF file if any:
  if (itsEDFfname.getVal().empty() == false)
    {
      LINFO("Transferring EDF file to '%s'", itsEDFfname.getVal().c_str());
      receive_data_file((char *)itsEDFfname.getVal().c_str(),
                        (char *)itsEDFfname.getVal().c_str(), 0);
    }

  // Disconnect from eyelink:
  close_eyelink_connection();

  EyeTracker::stop1();
#endif
}

// ######################################################################
void EyeTrackerEyeLink::calibrate(nub::soft_ref<PsychoDisplay> d)
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
	//create SDL_Surface of a image that replaces calibration point
	if(!d->getModelParamString("PsychoDisplayFixationIcon").empty()){
		Image< PixRGB<byte> > fixicon = 
			Raster::ReadRGB(d->getModelParamString("PsychoDisplayFixationIcon"));
		SDL_Surface *img = d->makeBlittableSurface(fixicon, false, 
										PixRGB<byte>(target_background_color.r, target_background_color.g, target_background_color.b));
		set_cal_target_surface(img);
	}
	
  // Do camera setup and calibration:
  do_tracker_setup();

	// Get calibration result
	char message[256];
	eyelink_cal_message(message);
	eyemsg_printf(message);
#endif
}

// ######################################################################
void EyeTrackerEyeLink::calibrate2(nub::soft_ref<PsychoDisplay> d)
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
	//create SDL_Surface of a image that replaces calibration point
	if(!d->getModelParamString("PsychoDisplayFixationIcon").empty()){
		Image< PixRGB<byte> > fixicon = 
			Raster::ReadRGB(d->getModelParamString("PsychoDisplayFixationIcon"));
		SDL_Surface *img = d->makeBlittableSurface(fixicon, false, 
										PixRGB<byte>(target_background_color.r, target_background_color.g, target_background_color.b));
		set_cal_target_surface(img);
	}
	
  // Do camera setup and calibration:
 	do_tracker_setup();

//	INT16 x = 1024;
//	INT16 y = 768;
//	draw_cal_target(x, y);

	// Get calibration result
	char message[256];
	eyelink_cal_message(message);
	eyemsg_printf(message);
#endif
}

// ######################################################################
void EyeTrackerEyeLink::setBackgroundColor(nub::soft_ref<PsychoDisplay> d)
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
	SDL_Color bgcolor = { d->getGrey().red(), d->getGrey().green(), d->getGrey().blue()};
	SDL_Color fgcolor = { 192, 192, 192};
	
	set_calibration_colors(&fgcolor, &bgcolor);
	LINFO("RGB: %i %i %i", d->getGrey().red(), d->getGrey().green(), d->getGrey().blue());
#endif
}

// ######################################################################
void EyeTrackerEyeLink::manualDriftCorrection(Point2D<double> eyepos, 
								                              Point2D<double> targetpos)
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
	char message[256];
	eyecmd_printf(const_cast<char*>("drift_correction %ld %ld %ld %ld"), 
									                 targetpos.i-eyepos.i, targetpos.j-eyepos.j,
																	 targetpos.i, targetpos.j);

	// Get drift correction result
	eyelink_cal_message(message);
	eyemsg_printf(message);
#endif
}

// ######################################################################
void EyeTrackerEyeLink::recalibrate(nub::soft_ref<PsychoDisplay> d,int repeats)
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
	//create SDL_Surface of a image that replaces calibration point
	if(!d->getModelParamString("PsychoDisplayFixationIcon").empty()){
 		Image< PixRGB<byte> > fixicon =
	  	Raster::ReadRGB(d->getModelParamString("PsychoDisplayFixationIcon"));
    SDL_Surface *img = d->makeBlittableSurface(fixicon, false, 
										PixRGB<byte>(target_background_color.r, target_background_color.g, target_background_color.b));
	  set_cal_target_surface(img);
	}

	do_drift_correct(d->getWidth()/2, d->getHeight()/2, 1, 1);
	
	// Get calibration result
	char message[256];
	eyelink_cal_message(message);
	eyemsg_printf(message);

        // set offline mode to ensure trackign
        set_offline_mode();
#endif
}

// ######################################################################
void EyeTrackerEyeLink::closeSDL()
{
#ifdef HAVE_EYELINK
  close_expt_graphics();
#endif
}


// ######################################################################
void EyeTrackerEyeLink::startTracking()
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
  // Show message at bottom of tracker display:
  eyecmd_printf(const_cast<char*>("record_status_message 'Recording [%d]...' "), getSession());

	// set FIXUPDATE event property
  eyecmd_printf(const_cast<char*>("fixation_update_interval = 50"));
  eyecmd_printf(const_cast<char*>("fixation_update_accumulate = 50"));

  // Always send a TRIALID message before starting to record.  It
  // should contain trial condition data required for analysis:
  eyemsg_printf(const_cast<char*>("TRIALID EYETRACK"));

  // TRIAL_VAR_DATA message is recorded for EyeLink Data Viewer
  // analysis It specifies the list of trial variables value for the
  // trial This must be specified within the scope of an individual
  // trial (i.e., after "TRIALID" and before "TRIAL_RESULT"):
  eyemsg_printf(const_cast<char*>("!V TRIAL_VAR_DATA EYETRACKING"));

  // Log eye tracker time at onset of clip:
  if (itsEventLog.isValid())
    itsEventLog->pushEvent(sformat("Eye Tracker Time = %d", int(current_msec())));

  // Actually start the recording:
  if (start_recording(1, 1, 1, 1))
    LFATAL("Error trying to start recording");
#endif
}

// ######################################################################
void EyeTrackerEyeLink::stopTracking()
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
#else
  // report response result: 0=timeout, else button number:
  eyemsg_printf(const_cast<char*>("TRIAL_RESULT 1"));

  // stop the tracker:
  stop_recording();

	// set FIXUPDATE event property (close it)
  eyecmd_printf(const_cast<char*>("fixation_update_interval = 0"));
  eyecmd_printf(const_cast<char*>("fixation_update_accumulate = 0"));

	set_offline_mode();
#endif
}

// ######################################################################
bool EyeTrackerEyeLink::isFixating()
{
  LFATAL("Unimplemented for now");
  return false;
}

// ######################################################################
bool EyeTrackerEyeLink::isSaccade()
{
  LFATAL("Unimplemented for now");
  return false;
}

// ######################################################################
Point2D<int> EyeTrackerEyeLink::getEyePos() const
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
	return Point2D<int>(-1,-1);
#else
	// get gaze position
	eyelink_newest_float_sample(&evt);

	// make sure pupil is present
	if(evt.fs.gx[RIGHT_EYE]!=MISSING_DATA && 
		 evt.fs.gy[RIGHT_EYE]!=MISSING_DATA && evt.fs.pa[RIGHT_EYE]>0){
		return Point2D<int>((int)evt.fs.gx[RIGHT_EYE], (int)evt.fs.gy[RIGHT_EYE]);
	} else {
		return Point2D<int>(-1, -1);
	}
#endif
}

// ######################################################################
Point2D<int> EyeTrackerEyeLink::getFixationPos() const
{
#ifndef HAVE_EYELINK
  LFATAL("Proprietary EyeLink developer API not installed");
	return Point2D<int>(-1,-1);
#else
	int i, x, y;				

	while (1) { // using while loop to consume the eye data queue 
    i = eyelink_get_next_data(NULL);   // check for new data item
		if (i == FIXUPDATE) {
    	eyelink_get_float_data(&evt);
			x = (int)evt.fe.gavx;
			y = (int)evt.fe.gavy;
     	//LINFO("Eyelink Time (FIXUPDATE) : %i (%i, %i)\n", (int)evt.fe.time, x, y);
			break;
		}
	}

	return Point2D<int>(x, y);
#endif 
}

// ######################################################################
CalibrationTransform::Data EyeTrackerEyeLink::getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const
{
    LINFO("\n getting calibration set...");
    CalibrationTransform::Data dummy;
    dummy.addData(Point2D<double>(-1.0,-1.0),Point2D<double>(-1.0,-1.0));
    return dummy;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
