/*!@file Psycho/testGetPos2.C test if we can get the instanteous eye position from the eyeTracker */

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
// Primary maintainer for this file: Farhan Baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/testGetPos2.C $
// $Id:
//

#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Psycho/PsychoDisplay.H"
#include "Psycho/EyeTrackerConfigurator.H"
#include "Psycho/EyeTracker.H"
#include "Psycho/PsychoOpts.H"
#include "Component/EventLog.H"
#include "Component/ComponentOpts.H"
#include "Raster/Raster.H"
#include "Util/MathFunctions.H"
#include "Util/Types.H"
#include "Image/DrawOps.H"
#include "Image/CalibrationTransform.H"
#include "Image/AffineTransform.H"
#include "Image/IO.H"
// ######################################################################
static int submain(const int argc, char** argv)
{

        MYLOGVERB = LOG_INFO;  // suppress debug messages



// Instantiate a ModelManager:
 ModelManager manager("test get eye position");

 nub::soft_ref<EyeTrackerConfigurator>
   etc(new EyeTrackerConfigurator(manager));
 manager.addSubComponent(etc);

 nub::soft_ref<EventLog> el(new EventLog(manager));
 manager.addSubComponent(el);

 nub::soft_ref<PsychoDisplay> d(new PsychoDisplay(manager));
 manager.addSubComponent(d);


 manager.setOptionValString(&OPT_EventLogFileName, "psychodata.psy");
 manager.setOptionValString(&OPT_EyeTrackerType, "ISCAN");

 // Parse command-line:
 if (manager.parseCommandLine(argc, argv, "", 0, 0) == false)
   return(1);


 // hook our various babies up and do post-command-line configs:
 nub::soft_ref<EyeTracker> et = etc->getET();
 et->setEventLog(el);
 et->requestQuickEyeS();

 // let's get all our ModelComponent instances started:
 manager.start();

 d->setEyeTracker(et);
 d->setEventLog(el);

 el->pushEvent(std::string("===== Trial 1:  ====="));

 //lets time stamp event log and start the eyetracker

 //here we will start testing the get pos

 LINFO("eyescanner started going to start reading stuff");
//  int i=0;

 d->clearScreen();
 d->displayISCANcalib();
 d->waitForKey();

 Image< PixRGB<byte> > displayImage (1920,1080, ZEROS);
 drawCircle(displayImage, Point2D<int>(1920/2,1080/2), 5, PixRGB<byte>(255,0,0));

 SDL_Surface *surf = d->makeBlittableSurface(displayImage, true);
 d->displaySurface(surf, -2);
 et->calibrateOnline(d);
 Image<double> txf;
 Point2D<int> testpoint;
 Point2D<int> testpointCalib,testpointD;

     /*pts = et->getCalibrationSet(d);
     txf = a.computeTransform(pts);
     LINFO("transform is...");
     std::cerr << txf << std::endl;
     testpoint =  et->getEyePos();
     testpointD = Point2D<double>(testpoint);
     LINFO("\n testpoint %d,%d",testpoint.i,testpoint.j);
     testpointCalib = a.getCalibrated(testpointD);*/
     testpoint = et->getEyePos();
     testpointCalib = et->getCalibEyePos();
     LINFO("\n testpoint %d,%d, calibrated %d,%d",testpoint.i,testpoint.j,testpointCalib.i,testpointCalib.j);

   d->clearScreen();
   char tmp[40];
   SDL_Surface *surf2;
   LINFO("ready to set tracker to true");
   et->track(true);
   while(d->checkForKey() < 0)
   {
     displayImage.clear();
     testpointCalib =  et->getCalibEyePos();
     LINFO("\n testpoint %d,%d, calibrated %d,%d",testpoint.i,testpoint.j,testpointCalib.i,testpointCalib.j);

     if(displayImage.coordsOk(testpointCalib))
     {
             drawCircle(displayImage, Point2D<int>(testpointCalib), 2, PixRGB<byte>(255,0,0));
             sprintf(tmp,"(%d,%d)",(int)testpointCalib.i,(int)testpointCalib.j);
             writeText(displayImage,Point2D<int>(testpointCalib),tmp,PixRGB<byte>(255,0,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
             surf2 = d->makeBlittableSurface(displayImage, true);
             d->displaySurface(surf2, -2);
        SDL_FreeSurface(surf2);
     }
//     i++;

   }


 // stop the eye tracker:
 usleep(50000);
 et->track(false);
 // stop all our ModelComponents
 manager.stop();

 // all done!
 return 0;


}

extern "C" int main(const int argc, char** argv)
{
        //incase we abort dont want X to die
        try
        {
                return submain(argc,argv);
        }
        catch (...)
        {
                REPORT_CURRENT_EXCEPTION;
        }
        return 1;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


