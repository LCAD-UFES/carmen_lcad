/*!@file Neuro/SimulationViewerRecStats.C View/save a bunch of recognition stats */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerRecStats.C $
// $Id: SimulationViewerRecStats.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Neuro/SimulationViewerRecStats.H"
#include "Image/colorDefs.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/ImageSet.H"
#include "Image/MathOps.H"
#include "Neuro/Brain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/VisualCortex.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Media/MediaSimEvents.H"
#include <fcntl.h> // for open()
#include <cerrno> // for errno
#include <unistd.h>


// ######################################################################
SimulationViewerRecStats::SimulationViewerRecStats(OptionManager& mgr,
                                         const std::string& descrName,
                                         const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventWTAwinner),
  itsStatsFname(&OPT_SVstatsFname, this),
  itsUseStatsFile(false),
  itsStatsFile(NULL),
  totalRec(0), //the total number we processed
  totalRecScenes(0), //the total scenes recognized
  totalRecObj(0) //the total obj recognized
{ }

// ######################################################################
SimulationViewerRecStats::~SimulationViewerRecStats()
{ }

// ######################################################################
void SimulationViewerRecStats::start2()
{
  // initialize our stats dump file if desired:
  if (itsStatsFname.getVal().size())
  {
    itsStatsFile = new std::ofstream(itsStatsFname.getVal().c_str());
    if (itsStatsFile->is_open() == false)
      LFATAL("Failed to open '%s' for writing.",
             itsStatsFname.getVal().c_str());
    itsUseStatsFile = true;
  }
}

// ######################################################################
void SimulationViewerRecStats::stop2()
{
  // close our stats file if we have one:
  if (itsStatsFile)
  {
    itsStatsFile->flush();
    itsStatsFile->close();
    delete itsStatsFile;
    itsStatsFile = NULL;
  }
}

// #####################################################################
void SimulationViewerRecStats::lockFile(const std::string fileName,
                                        int &fd,
                                        struct flock &fl) const
{
  // lock file
  fd = open(fileName.c_str(), O_RDWR);
  if (fd < 0)
  {
        LINFO("lockFile: Open failure on file %s",fileName.c_str());
  }
  else
  {
    fl.l_type   = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start  = 0;
    fl.l_len    = 0;
    if (fcntl(fd, F_SETLK, &fl) == -1)
    {
      if (errno == EACCES || errno == EAGAIN)
        LINFO("'%s' Already locked by another process",fileName.c_str());
      else if(errno == EBADF)
        LINFO("'%s' not a valid open file descriptor",fileName.c_str());
      else if(errno == EINVAL)
        LINFO("'%s In a locking operation, fildes refers to a file with a type that does not support locking, or the struct flock pointed to by the third argument has an incorrect form",fileName.c_str());
      else if(errno == EMFILE)
        LINFO("'%s' process has already reached its maximum number of file descriptors",fileName.c_str());
      else
      LINFO("Cannot lock file '%s' Error code '%d' \(Is this an NFS mount?)",fileName.c_str(),errno);
    }
  }
}

// #####################################################################
void SimulationViewerRecStats::unlockFile(const std::string fileName,
                                          const  int fd,
                                          struct flock &fl) const
{
  // unlockfile
  fl.l_type   = F_UNLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start  = 0;
  fl.l_len    = 0;
  if (fcntl(fd, F_SETLK, &fl) == -1)
  {
    LINFO("Cannot unlock file '%s'",fileName.c_str());
  }
  close(fd);
}

// ######################################################################
void SimulationViewerRecStats::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  // Lock the file. We put this here to support multi-process in the
  // future. However, this will not work on NFS mounts.
  struct flock fl; int fd;

  if(itsUseStatsFile) lockFile(itsStatsFname.getVal().c_str(),fd,fl);

  double maxProb = 0.0F, normProb = 0.0F;

  totalRec++;
  //Get the predicted results
  std::string predSceneDescription;
  std::string predObjName;
  // any new input about the scene type
  if (SeC<SimEventSceneDescription> ee = q.check<SimEventSceneDescription>(this))
    predSceneDescription = ee->getSceneData()->description;

  // any new input about the object type
  if (SeC<SimEventObjectDescription> ee = q.check<SimEventObjectDescription>(this))
  {
    predObjName  = ee->getObjData()->name;
    maxProb      = ee->getObjData()->maxProb;
    normProb     = ee->getObjData()->normProb;
  }

  LINFO("Predicted: scene: %s object: %s",
      predSceneDescription.c_str(),
      predObjName.c_str());


  //Get the ground truth
  rutz::shared_ptr<TestImages::SceneData> sceneData;
  //Get the scene data, but dont mark it so we will get it on the next saccade
  if (SeC<SimEventInputFrame> ee = q.check<SimEventInputFrame>(this,SEQ_UNMARKED,0))
  {
    GenericFrame gf = ee->frame();
    rutz::shared_ptr<GenericFrame::MetaData> metaData = gf.getMetaData(std::string("SceneData"));
    if (metaData.get() != 0)
      sceneData.dyn_cast_from(metaData);
  }

  if (sceneData.get() != 0)
  {
    std::string objName = getObjNameAtLoc(sceneData->objects, e->winner().p);
    LINFO("Labeled: scene: %s object: %s",
        sceneData->description.c_str(),
        objName.c_str());

    if (sceneData->description == predSceneDescription)
      totalRecScenes++;

    if (objName == predObjName)
      totalRecObj++;

    LINFO("Recognition Rate: Scene %lu/%lu=%0.2f   Object %lu/%lu=%0.2f",
        totalRecScenes, totalRec, (float)totalRecScenes/(float)totalRec,
        totalRecObj, totalRec, (float)totalRecObj/(float)totalRec);

    if(itsUseStatsFile)
    {
      // print a header
      if(totalRec == 1)
        (*itsStatsFile) << "Total\tCorrect\tPredicted\tLabeled\tPercent\tmaxProb\tnormProb\n";

      (*itsStatsFile) << sceneData->filename << "\t"
                      << totalRec            << "\t"
                      << totalRecObj         << "\t"
                      << predObjName.c_str() << "\t"
                      << objName.c_str()     << "\t"
                      << (float)totalRecObj/(float)totalRec << "\t"
                      << maxProb             << "\t"
                      << normProb            << "\n";
    }
  }

  if (SeC<SimEventObjectToBias> e = q.check<SimEventObjectToBias>(this,SEQ_UNMARKED,0))
  {
    LINFO("Object To Bias: %s", e->name().c_str());
  }

  if(itsUseStatsFile) unlockFile(itsStatsFname.getVal().c_str(),fd,fl);
}

// ######################################################################
std::string SimulationViewerRecStats::
getObjNameAtLoc(const std::vector<TestImages::ObjData> &objects, const Point2D<int>& loc)
{

  for(uint obj=0; obj<objects.size(); obj++)
  {
    TestImages::ObjData objData = objects[obj];

    LINFO("Checking Object %s in file %s",objData.name.c_str(),objData.filename.c_str());

    //find the object dimention from the polygon
    if (objData.polygon.size() > 0)
    {
      Point2D<int> upperLeft = objData.polygon[0];
      Point2D<int> lowerRight = objData.polygon[0];

      for(uint i=0; i<objData.polygon.size(); i++)
      {
        //find the bounds for the crop
        if (objData.polygon[i].i < upperLeft.i) upperLeft.i = objData.polygon[i].i;
        if (objData.polygon[i].j < upperLeft.j) upperLeft.j = objData.polygon[i].j;

        if (objData.polygon[i].i > lowerRight.i) lowerRight.i = objData.polygon[i].i;
        if (objData.polygon[i].j > lowerRight.j) lowerRight.j = objData.polygon[i].j;
      }

      //check if point is within the polygon
      for(int y=upperLeft.j; y<lowerRight.j; y++)
        for(int x=upperLeft.i; x<lowerRight.i; x++)
        {
          if (pnpoly(objData.polygon, loc))
            return objData.name;
        }
    }

  }
  return std::string("Unknown");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
