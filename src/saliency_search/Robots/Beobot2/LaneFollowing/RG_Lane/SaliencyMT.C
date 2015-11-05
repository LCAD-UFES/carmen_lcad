/*!@file /Robots/Beobot2/LaneFollowing/RG_Lane/SaliencyMT.C 
 * A class for quick-and-dirty saliency mapping */

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
// Primary maintainer for this file: Zack Gossman <gossman@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/LaneFollowing/RG_Lane/SaliencyMT.C $
// $Id: SaliencyMT.C 8095 2007-03-12 01:14:23Z kai $
//

#include "Robots/Beobot2/LaneFollowing/RG_Lane/SaliencyMT.H"
#include "Demo/DemoOpts.H"

// ######################################################################
// ##### Global options:
// ######################################################################

#define sml        2
#define delta_min  3
#define delta_max  4
#define level_min  0
#define level_max  2
#define maxdepth   (level_max + delta_max + 1)
#define normtyp    (VCXNORM_MAXNORM)

// relative feature weights:
#define IWEIGHT 0.7
#define CWEIGHT 1.0
#define OWEIGHT 1.0
#define FWEIGHT 1.0
#define SWEIGHT 0.7

// image size vars
#define IMAGEWIDTH 320 
#define IMAGEHEIGHT 240

// action definitions CHANGE TO ENUM? FIX?
#define RETINA     1
#define WINNER     2
#define LUMINANCE  3
#define REDGREEN   4
#define BLUEYELLOW 5
#define ORI0       6
#define ORI45      7
#define ORI90      8
#define ORI135     9
#define CMAP       10
#define FLICKER    11
#define INTENSITY  12
#define SKINHUE    13

#define numthreads 1

// ######################################################################
void *SaliencyMT_CMAP(void *c)
{
  SaliencyMT *d = (SaliencyMT *)c;
  d->computeCMAP();
  return NULL;
}

// ######################################################################
SaliencyMT::SaliencyMT(OptionManager& mgr,
           const std::string& descrName,
           const std::string& tagName):
  ModelComponent(mgr, descrName, tagName),
  itsNumThreads(&OPT_SMTnumThreads, this)  // see Demo/DemoOpts.{H,C}

{

  numWorkers = 0U;

}

// ######################################################################
void SaliencyMT::start1()
{
  // start threads. They should go to sleep on the condition since no
  // jobs have ben queued up yet:
  pthread_mutex_init(&jobLock, NULL);
  pthread_mutex_init(&mapLock, NULL);
  pthread_cond_init(&jobCond, NULL);

  LINFO("Starting with %u threads...", itsNumThreads.getVal());

  // get our processing threads started:
  worker = new pthread_t[itsNumThreads.getVal()];
  for (uint i = 0; i < itsNumThreads.getVal(); i ++)
    {
      pthread_create(&worker[i], NULL, SaliencyMT_CMAP, (void *)this);

      // all threads should go and lock against our job condition. Sleep a
      // bit to make sure this really happens:
      usleep(100000);
    }
}

// ######################################################################
void SaliencyMT::stop2()
{
  // should cleanup the threads, mutexes, etc...
  pthread_cond_destroy(&jobCond);

  //for (uint i = 0; i < numthreads; i ++)
  //  pthread_delete(&worker[i].....

  delete [] worker;
}

// ######################################################################
SaliencyMT::~SaliencyMT()
{ }

// ######################################################################
void SaliencyMT::newInput(Image< PixRGB<byte> > img, bool procFlicker)
{
  //LINFO("new input.....");
  // store current color image:
  pthread_mutex_lock(&mapLock);
  colima = img;

  // also kill any old output and internals:
  outmap.freeMem();
  gotLum = false; gotRGBY = false; gotSkin = false;
  pthread_mutex_unlock(&mapLock);

  // setup job queue:
  pthread_mutex_lock(&jobLock);
  jobQueue.clear();

  jobQueue.push_back(jobData(INTENSITY, Gaussian5, IWEIGHT, 0.0F));

  jobQueue.push_back(jobData(REDGREEN, Gaussian5, CWEIGHT, 0.0F));

 // jobQueue.push_back(jobData(SKINHUE, Gaussian5, SWEIGHT, 0.0F));

  jobQueue.push_back(jobData(ORI0, Oriented5, OWEIGHT, 0.0F));
  jobQueue.push_back(jobData(ORI45, Oriented5, OWEIGHT, 45.0F));
  jobQueue.push_back(jobData(ORI90, Oriented5, OWEIGHT, 90.0F));
  jobQueue.push_back(jobData(ORI135, Oriented5, OWEIGHT, 135.0F));

  if (procFlicker)
    jobQueue.push_back(jobData(FLICKER, Gaussian5, FWEIGHT, 0.0F));

  jobQueue.push_back(jobData(BLUEYELLOW, Gaussian5, CWEIGHT, 0.0F));

  jobsTodo = jobQueue.size();
  pthread_mutex_unlock(&jobLock);

  // broadcast on job queue condition to wake up worker threads:
  pthread_cond_broadcast(&jobCond);
  //LINFO("new input ok.....");
}

// ######################################################################
bool SaliencyMT::outputReady()
{
  bool ret = false;

  pthread_mutex_lock(&jobLock);
  if (jobsTodo == 0U) ret = true;
  pthread_mutex_unlock(&jobLock);

  return ret;
}

// ######################################################################
Image<float> SaliencyMT::getOutput()
{
  Image<float> ret;

  pthread_mutex_lock(&mapLock);
  ret = outmap;
  pthread_mutex_unlock(&mapLock);

  return ret;
}

// ######################################################################
Image<float> SaliencyMT::getRGOutput()
{
  Image<float> ret;

  pthread_mutex_lock(&mapLock);
  ret = cmap_rg;
  pthread_mutex_unlock(&mapLock);

  return ret;
}
// ######################################################################
Image<float> SaliencyMT::getBYOutput()
{
  Image<float> ret;

  pthread_mutex_lock(&mapLock);
  ret = cmap_by;
  pthread_mutex_unlock(&mapLock);

  return ret;
}
// ######################################################################
Image<float> SaliencyMT::getIntensityOutput()
{
  Image<float> ret;

  pthread_mutex_lock(&mapLock);
  ret = cmap_intensity;
  pthread_mutex_unlock(&mapLock);

  return ret;
}
// ######################################################################
//The threaded function
void SaliencyMT::computeCMAP()
{
  pthread_mutex_lock(&mapLock);
  uint myNum = numWorkers ++;
  pthread_mutex_unlock(&mapLock);
  LINFO("  ... worker %u ready.", myNum);

  while(true)
    {
      // wait until there are jobs in the queue that we can process:
      pthread_mutex_lock(&jobLock);
      jobData current(0, Gaussian5, 0.0F, 0.0F); bool nojobs = true;
      if (jobQueue.empty() == false)
        {
          current = jobQueue.front();
          jobQueue.pop_front();
          nojobs = false;
        }
      else
        pthread_cond_wait(&jobCond, &jobLock);
      pthread_mutex_unlock(&jobLock);

      // if we don't have a job to do, just wait more:
      if (nojobs) continue;
      //LINFO("[%u] GOT: job %d", myNum, int(current.jobType));

      // read next entry in job queue and perform desired action on
      // current image and record result in output image
      // (accumulative)
      Image<float> curImage;

      // The case statement on this end parses the desired action from
      // the job queue and performs the needed image pre-processing
      pthread_mutex_lock(&mapLock);
      switch(current.jobType)
        {
          // While shared resources are used here, they are only read,
          // so they should not need to be protected by mutexers

          // ##################################################
        case REDGREEN:
          if (gotRGBY == false)
            { getRGBY(colima, r, g, b, y, byte(25)); gotRGBY = true; }
          curImage = r - g;
          break;

          // ##################################################
        case BLUEYELLOW:
          if (gotRGBY == false)
            { getRGBY(colima, r, g, b, y, byte(25)); gotRGBY = true; }
          curImage = b - y;
          break;

          // ##################################################
        case SKINHUE:
          if (gotSkin == false)
            {
              skinima = hueDistance(colima, COL_SKIN_MUR, COL_SKIN_MUG,
                                    COL_SKIN_SIGR, COL_SKIN_SIGG,
                                    COL_SKIN_RHO);
              gotSkin = true;
            }
          curImage = skinima;
          break;

          // ##################################################
        case ORI0:
        case ORI45:
        case ORI90:
        case ORI135:
        case INTENSITY:
          if (gotLum == false)
            { lum = Image<float>(luminance(colima)); gotLum = true; }
          curImage = lum;
          break;

          // ##################################################
        case FLICKER:
          if (gotLum == false)
            { lum = Image<float>(luminance(colima)); gotLum = true; }
          // compute flicker consp map and send to collector:
          if (prev.initialized() == false)
            {
              prev = lum;
              curImage.resize(lum.getDims(), true); // clear
            }
          else
            {
              curImage = lum - prev;
              prev = lum;
            }
          break;

          // ##################################################
        default:
          LERROR("What is going on around here?");
          curImage = lum;
        }
      pthread_mutex_unlock(&mapLock);

      // compute pyramid:
      ImageSet<float> pyr =
        buildPyrGeneric(curImage, 0, maxdepth,
                        current.ptyp, current.orientation);

      // alloc conspicuity map and clear it:
      Image<float> cmap(pyr[sml].getDims(), ZEROS);

      // intensities is the max-normalized weighted sum of IntensCS:
      for (int delta = delta_min; delta <= delta_max; delta ++)
        for (int lev = level_min; lev <= level_max; lev ++)
          {
            Image<float> tmp = centerSurround(pyr, lev, lev + delta, true);
            tmp = downSize(tmp, cmap.getWidth(), cmap.getHeight());
            tmp = maxNormalize(tmp, MAXNORMMIN, MAXNORMMAX, normtyp);
            cmap += tmp;
          }

      inplaceAddBGnoise(cmap, 25.0F);

      if (normtyp == VCXNORM_MAXNORM)
        cmap = maxNormalize(cmap, MAXNORMMIN, MAXNORMMAX, normtyp);
      else
        cmap = maxNormalize(cmap, 0.0f, 0.0f, normtyp);

      // multiply by conspicuity coefficient:
      if (current.weight != 1.0F) cmap *= current.weight;

      // Add to saliency map:
      pthread_mutex_lock(&mapLock);
      if (outmap.initialized()) outmap += cmap;
      else outmap = cmap;
			if(current.jobType == REDGREEN)
				cmap_rg = cmap;
			else if(current.jobType == BLUEYELLOW)
				cmap_by = cmap;
			else if(current.jobType == INTENSITY)
				cmap_intensity = cmap;
      pthread_mutex_unlock(&mapLock);
      pthread_mutex_lock(&jobLock);
      -- jobsTodo;
      //LINFO("done with job %d, %u todo...", int(current.jobType),jobsTodo);
      pthread_mutex_unlock(&jobLock);
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
