/*!@file Neuro/InferoTemporal.C Object recognition module */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporal.C $
// $Id: InferoTemporal.C 14390 2011-01-13 20:17:22Z pez $
//

#include "Neuro/InferoTemporal.H"

#include "Component/OptionManager.H"
#include "Image/CutPaste.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/Brain.H"
#include "Neuro/VisualCortex.H"
#include "SIFT/VisualObjectDB.H"
#include "SIFT/VisualObject.H"

#include <cstdlib>
#include <iostream>

// ######################################################################
namespace
{
  Image<PixRGB<byte> > getCroppedObject(const Image<PixRGB<byte> >& scene,
                                        const Image<float>& smoothMask)
  {
    if (!scene.initialized())
      return Image<PixRGB<byte> >();

    if (!smoothMask.initialized())
      return Image<PixRGB<byte> >();

    const float threshold = 1.0f;

    const Rectangle r = findBoundingRect(smoothMask, threshold);
    return crop(scene, r);
  }
}

// ######################################################################
InferoTemporal::InferoTemporal(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventWTAwinner)
{ }

// ######################################################################
InferoTemporal::~InferoTemporal()
{ }

// ######################################################################
void InferoTemporal::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  this->attentionShift(q, e->winner().p);
}

// ######################################################################
InferoTemporalStub::InferoTemporalStub(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName)
  :
  InferoTemporal(mgr, descrName, tagName)
{}

// ######################################################################
InferoTemporalStub::~InferoTemporalStub()
{}

// ######################################################################
void InferoTemporalStub::attentionShift(SimEventQueue& q,
                                        const Point2D<int>& location)
{}

// ######################################################################
InferoTemporalStd::InferoTemporalStd(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName) :
  InferoTemporal(mgr, descrName, tagName),
  itsUseAttention(&OPT_AttentionObjRecog, this),
  itsObjectDatabaseFile(&OPT_ObjectDatabaseFileName, this),
  itsTrainObjectDB(&OPT_TrainObjectDB, this),
  itsPromptUserTrainDB(&OPT_PromptUserTrainDB, this),
  itsMatchObjects(&OPT_MatchObjects, this),
  itsRecogMinMatch(&OPT_RecognitionMinMatch, this),
  itsMatchingAlg(&OPT_MatchingAlgorithm, this),
  itsObjectDB(new VisualObjectDB())
{ }

// ######################################################################
void InferoTemporalStd::start1()
{
  // if no filename given for our object DB, start empty, otherwise load it:
  if (itsObjectDatabaseFile.getVal().empty())
    LINFO("Starting with empty VisualObjectDB.");
  else
    itsObjectDB->loadFrom(itsObjectDatabaseFile.getVal());

  InferoTemporal::start1();
}

// ######################################################################
void InferoTemporalStd::stop1()
{
  // save database if we have a filename for it:
  if (itsObjectDatabaseFile.getVal().empty() == false)
    itsObjectDB->saveTo(itsObjectDatabaseFile.getVal());
}

// ######################################################################
InferoTemporalStd::~InferoTemporalStd()
{}

// ######################################################################
void InferoTemporalStd::attentionShift(SimEventQueue& q,
                                       const Point2D<int>& location)
{
  Image<PixRGB<float> > objImg;

  // get the lastest input frame from the retina:
  if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this))
    objImg = e->frame().colorByte();
  else
    LFATAL("Oooops, no input frame in the event queue?");

  // get the latest smooth mask from the shape estimator:
  Image<float> smoothMask;
  if (SeC<SimEventShapeEstimatorOutput>
      e = q.check<SimEventShapeEstimatorOutput>(this))
    smoothMask = e->smoothMask();

  // crop around object using mask?
  if (itsUseAttention.getVal())
    objImg = getCroppedObject(objImg, smoothMask);

  if (!objImg.initialized()) return; // no object image, so just do nothing

  rutz::shared_ptr<SimReqVCXfeatures> ef(new SimReqVCXfeatures(this, location));
  q.request(ef); // VisualCortex is now filling-in the features into ef->features()

  // create a new VisualObject. Since we give it no keypoints, they
  // will be automatically computed:
  rutz::shared_ptr<VisualObject>
    obj(new VisualObject("NewObject", "NewObject", objImg, location, ef->features()));

  // Try to match this to the objects in our database:
  if (itsMatchObjects.getVal())
    {
      // we need to have at least 3 keypoints to consider this a
      // serious object candidate:
      if (obj->numKeypoints() < 3)
        { LINFO("Not enough Keypoints -- NO RECOGNITION"); return; }

      LINFO("Attempting object recognition...");
      std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;

      const uint nm =
        itsObjectDB->getObjectMatches(obj, matches, VOMA_KDTREEBBF,
                                      100U, 0.5F, 0.5F, 1.0F,
                                      uint(itsRecogMinMatch.getVal()),
                                      6U, false);
      // If an object was identified
      if (nm > 0)
        {
          LINFO("***** %u object recognition match(es) *****", nm);
          for (uint i = 0 ; i < nm; i ++)
            LINFO("   Match with '%s' [score = %f]",
                  matches[i]->getVoTest()->getName().c_str(),
                  matches[i]->getScore());
        }
      else
        LINFO("***** Could not identify attended object! *****");
    }

  // do we want to train the database?
  if (itsTrainObjectDB.getVal())
    {
      std::string objname;

      // if interactive, ask the user for a name:
      if (itsPromptUserTrainDB.getVal())
        {
          LINFO("Enter name for new object or [RETURN] to skip training:");
          std::getline(std::cin, objname, '\n');
        }
      else
        {
          // get a unique random name for the object:
          char tmpn[14]; strcpy(tmpn, "Object-XXXXXX");
          if(mkstemp(tmpn) == -1)
	    LFATAL("mkstemp failed");
	  objname = tmpn;
        }

      // train the database:
      if (objname.length() > 0)
        {
          LINFO("Adding new object '%s' to database.", objname.c_str());
          obj->setName(objname);
          obj->setImageFname(objname + ".png");
          itsObjectDB->addObject(obj);
        }

    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
