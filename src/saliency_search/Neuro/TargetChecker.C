/*!@file Neuro/TargetChecker.C check+count targets hit during a neural simulation */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/TargetChecker.C $
// $Id: TargetChecker.C 13065 2010-03-28 00:01:00Z itti $
//

#ifndef NEURO_TARGETCHECKER_C_DEFINED
#define NEURO_TARGETCHECKER_C_DEFINED

#include "Neuro/TargetChecker.H"

#include "Channels/InputFrame.H"
#include "Component/GlobalOpts.H"
#include "Component/OptionManager.H"
#include "Image/DrawOps.H"
#include "Image/MathOps.H"
#include "Image/Transforms.H"
#include "Neuro/NeuroOpts.H"
#include "Raster/Raster.H"
#include "Util/TextLog.H"
#include "Util/sformat.H"
#include "Simulation/SimEvents.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEventQueue.H"

// ######################################################################
TargetChecker::TargetChecker(OptionManager& mgr, const std::string& descrName,
                             const std::string& tagName)
  :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventWTAwinner),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventTargetMask),
  itsFOAradius(&OPT_FOAradius, this),
  itsLogFile(&OPT_TextLogFile, this),
  itsTargetMaskFname(&OPT_TargetMaskFname, this),
  itsVisualField(),
  itsRawTargetMask(),
  itsTargetMask(),
  itsNumTargetsRemaining(0)
{ }

// ######################################################################
TargetChecker::~TargetChecker()
{ }

// ######################################################################
void TargetChecker::reset1()
{
  itsVisualField.freeMem();
  SimModule::reset1();
}

// ######################################################################
void TargetChecker::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  // is there a new attention shift? If so, we want to check whether
  // one or more of our targets were hit:
  const WTAwinner winner = e->winner();

  // draw in the visual field, and count the new targets hit if we
  // have a target mask:
  if (itsTargetMask.initialized())
    {
      // draw a disk anc count how many targets we hit:
      const int numhit =
        drawDiskCheckTarget(itsVisualField, itsTargetMask, winner.p,
                            itsFOAradius.getVal(), byte(255), byte(255), byte(250));

      // did we hit any targets, as defined by our itsTargetMask?
      if (numhit > 0)
        {
          LINFO("###### %d new target(s) hit at t=%fms ######", numhit, winner.t.msecs());
          itsNumTargetsRemaining -= numhit;

          textLog(itsLogFile.getVal(), "HitTargets", sformat("%d", numhit), winner.t);

          q.post(rutz::shared_ptr<SimEventTargetsHit> (new SimEventTargetsHit(this, numhit)));
        }

      // are we done with all targets?
      if (itsTargetMask.initialized() && itsNumTargetsRemaining <= 0)
        {
          LINFO("##### All targets found -- EXIT #####");
          textLog(itsLogFile.getVal(), "AllTargetsFound", "", winner.t);

          q.post(rutz::shared_ptr<SimEventBreak>(new SimEventBreak(this)));
        }
    }

  // if no target mask, just draw in the visual field:
  else if (itsVisualField.initialized())
    drawDisk(itsVisualField, winner.p, itsFOAradius.getVal(), byte(255));
}

// ######################################################################
void TargetChecker::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // ok, here is a new input frame! Let's initialize our guts accordingly:
  const InputFrame inframe = e->frame();

  if (itsRawTargetMask.initialized())
    {
      itsTargetMask = itsRawTargetMask;
      inplaceNormalize(itsTargetMask, byte(0), byte(255));
      itsNumTargetsRemaining = countParticles(itsTargetMask, byte(255));
      LINFO("Counting targets... %d detected", itsNumTargetsRemaining);

      textLog(itsLogFile.getVal(), "TargetsRemaining",
              sformat("%d", itsNumTargetsRemaining), q.now());
    }
  else
    itsTargetMask.freeMem();

  // initialize visual field if necessary:
  if (itsVisualField.isSameSize(inframe.colorByte()) == false)
    itsVisualField = Image<byte>(inframe.colorByte().getDims(), ZEROS);
}

// ######################################################################
void TargetChecker::
onSimEventTargetMask(SimEventQueue& q, rutz::shared_ptr<SimEventTargetMask>& e)
{
  LINFO("Loading new target mask at time %.2fms...", q.now().msecs());
  itsRawTargetMask = e->mask();
}

// ######################################################################
void TargetChecker::start1()
{
  if (!itsTargetMaskFname.getVal().empty())
    {
      itsRawTargetMask = Raster::ReadGray(itsTargetMaskFname.getVal());
      LINFO("Using targetmask from image file %s", itsTargetMaskFname.getVal().c_str());
    }

  SimModule::start1();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_TARGETCHECKER_C_DEFINED
