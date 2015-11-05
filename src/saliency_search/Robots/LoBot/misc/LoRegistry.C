/**
   \file Robots/LoBot/misc/LoRegistry.C

   \brief This file acts as a central repository for the registration of
   the factories used to create the different locust models, integration
   algorithms, etc. supported by the Lobot/Robolocust project.
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoRegistry.C $
// $Id: LoRegistry.C 14305 2010-12-08 21:17:33Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// Robot behaviours
#include "Robots/LoBot/control/LoForward.H"
#include "Robots/LoBot/control/LoEmergencyStop.H"
#include "Robots/LoBot/control/LoExtricate.H"
#include "Robots/LoBot/control/LoLGMDExtricateSimple.H"
#include "Robots/LoBot/control/LoLGMDExtricateEMD.H"
#include "Robots/LoBot/control/LoLGMDExtricateVFF.H"
#include "Robots/LoBot/control/LoLGMDExtricateTTI.H"
#include "Robots/LoBot/control/LoCalibrateLET.H"
#include "Robots/LoBot/control/LoOpenPath.H"
#include "Robots/LoBot/control/LoSurvey.H"
#include "Robots/LoBot/control/LoGoal.H"
#include "Robots/LoBot/control/LoTrack.H"
#include "Robots/LoBot/control/LoBumpCounter.H"
#include "Robots/LoBot/control/LoMonitorDZone.H"
#include "Robots/LoBot/control/LoCountdown.H"
#include "Robots/LoBot/control/LoMetrics.H"
#include "Robots/LoBot/control/LoRenderResults.H"
#include "Robots/LoBot/control/LoTestScanMatching.H"
#include "Robots/LoBot/control/LoVFH.H"
#include "Robots/LoBot/control/LoRemoteControl.H"

// Robot platforms
#include "Robots/LoBot/io/LoRCCar.H"
#include "Robots/LoBot/io/LoRoombaCM.H"

// Locust LGMD models
#include "Robots/LoBot/lgmd/gabbiani/LoGabbiani.H"
#include "Robots/LoBot/lgmd/rind/LoStafford.H"

// Other lobot headers
#include "Robots/LoBot/misc/LoRegistry.H"

//------------------------------ MACROS ---------------------------------

/*
   To make things uniform/easy/nice/etc., each Robolocust class that is
   created using a factory defines a private type called my_factory and a
   static data member of type my_factory called register_me. To register
   the class's factory, we will have to define that class's register_me
   static data member in this central registry using a statement of the
   form:

      class_name::my_factory class_name::register_me(ID) ;

   where ID is the string ID used to create instances of the class. All
   the string IDs are #defined in LoRegistry.H.

   The following macro takes a class name and the string ID for that
   class and spits out a statement like the one shown above to define the
   register_me static data member of the given class.
*/
#define LOBOT_REGISTER(class_name, string_id) \
           class_name::my_factory class_name::register_me(string_id)

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCUST MODELS -----------------------------

LOBOT_REGISTER(GabbianiModel, LOLM_GABBIANI) ;
LOBOT_REGISTER(StaffordModel, LOLM_STAFFORD) ;

//-------------------------- ROBOT PLATFORMS ----------------------------

LOBOT_REGISTER(RCCar,    LORP_RC_CAR) ;
LOBOT_REGISTER(RoombaCM, LORP_ROOMBA_CM) ;

//------------------------- ROBOT BEHAVIOURS ----------------------------

LOBOT_REGISTER(Forward,             LOBE_FORWARD) ;
LOBOT_REGISTER(EmergencyStop,       LOBE_EMERGENCY_STOP) ;
LOBOT_REGISTER(Extricate,           LOBE_EXTRICATE) ;
LOBOT_REGISTER(LGMDExtricateSimple, LOBE_LGMD_EXTRICATE_SIMPLE) ;
LOBOT_REGISTER(LGMDExtricateEMD,    LOBE_LGMD_EXTRICATE_EMD) ;
LOBOT_REGISTER(LGMDExtricateVFF,    LOBE_LGMD_EXTRICATE_VFF) ;
LOBOT_REGISTER(LGMDExtricateTTI,    LOBE_LGMD_EXTRICATE_TTI) ;
LOBOT_REGISTER(CalibrateLET,        LOBE_CALIBRATE_LET) ;
LOBOT_REGISTER(OpenPath,            LOBE_OPEN_PATH) ;
LOBOT_REGISTER(Survey,              LOBE_SURVEY) ;
LOBOT_REGISTER(Goal,                LOBE_GOAL) ;
LOBOT_REGISTER(Track,               LOBE_TRACK) ;
LOBOT_REGISTER(BumpCounter,         LOBE_BUMP_COUNTER) ;
LOBOT_REGISTER(MonitorDZone,        LOBE_MONITOR_DZONE) ;
LOBOT_REGISTER(Countdown,           LOBE_COUNTDOWN) ;
LOBOT_REGISTER(Metrics,             LOBE_METRICS) ;
LOBOT_REGISTER(RenderResults,       LOBE_RENDER_RESULTS) ;
LOBOT_REGISTER(TestScanMatching,    LOBE_TEST_SCAN_MATCHING) ;
LOBOT_REGISTER(VFH,                 LOBE_VFH) ;
LOBOT_REGISTER(RemoteControl,       LOBE_REMOTE_CONTROL) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
