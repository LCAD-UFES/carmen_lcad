/*!@file Neuro/SimulationViewerStats.C View/save a bunch of stats */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerStats.C $
// $Id: SimulationViewerStats.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Neuro/SimulationViewerStats.H"
#include "Channels/ChannelMaps.H"
#include "Channels/ChannelOpts.H"
#include "Image/colorDefs.H"
#include "Image/CutPaste.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/fancynorm.H"
#include "Image/FFTWWrapper.H"
#include "Image/ImageSet.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/NamedImage.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/Brain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/VisualCortex.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include <fcntl.h> // for open()
#include <cerrno> // for errno
#include <unistd.h>


// ######################################################################
SimulationViewerStats::SimulationViewerStats(OptionManager& mgr,
                                         const std::string& descrName,
                                         const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsStatsFname(&OPT_SVstatsFname, this),
  itsSaveXcombo(&OPT_SVsaveXcombo, this),
  itsSaveYcombo(&OPT_SVsaveYcombo, this),
  itsComputeAGStats(&OPT_ComputeAGStats, this),
  itsAGTargetFrame(&OPT_AGTargetFrame, this),
  itsAGMaskFile(&OPT_AGMaskFile, this),
  itsAGStatsSaveFile(&OPT_AGStatsSaveFile, this),
  itsGetSingleChannelStats(&OPT_GetSingleChannelStats, this),
  itsGetSingleChannelStatsFile(&OPT_GetSingleChannelStatsFile, this),
  itsSaveStatsPerChannelFreq(&OPT_SaveStatsPerChannelFreq, this),
  itsStatsFile(NULL)
{ }

// ######################################################################
SimulationViewerStats::~SimulationViewerStats()
{ }

// ######################################################################
void SimulationViewerStats::start2()
{
  // Used by saveCompat for frame counts
  itsFrameIdx = 0;

  // Reset counters
  itsMaskCount = 0; itsLamCount = 0; itsOverlapCount = 0;
}

// ######################################################################
void SimulationViewerStats::stop2()
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
void SimulationViewerStats::lockFile(const std::string fileName,
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
void SimulationViewerStats::unlockFile(const std::string fileName,
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
void SimulationViewerStats::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void SimulationViewerStats::save1(const ModelComponentSaveInfo& sinfo)
{
  // Use a LINFO here since we are already slowed down by writing
  // stats, we should at least have a short debug on this fact
  LINFO("SAVING STATS TO %s",itsStatsFname.getVal().c_str());

  // Lock the file. We put this here to support multi-process in the
  // future. However, this will not work on NFS mounts.
  struct flock fl; int fd;
  lockFile(itsStatsFname.getVal().c_str(),fd,fl);

  // Since this is more or less debug info we open and flush every iteration.
  // rather than once each run.
  std::ofstream statsFile;
  statsFile.open(itsStatsFname.getVal().c_str(),std::ios_base::app);

  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs =
    dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

  // also get the SimEventQueue:
  SimEventQueue *q    = dynamic_cast<const SimModuleSaveInfo&>(sinfo).q;

  // initialize our stats dump file if desired:
  if(itsFrameIdx == 0)
  {
    rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
    q->request(vcxm); // VisualCortex is now filling-in the maps...

    if (itsStatsFname.getVal().size())
    {
      itsStatsFile = new std::ofstream(itsStatsFname.getVal().c_str());
      if (itsStatsFile->is_open() == false)
        LFATAL("Failed to open '%s' for writing.",
               itsStatsFname.getVal().c_str());

      // dump the settings of the model:
      getRootObject()->printout(*itsStatsFile, "# ");

      // list all our channels:
      //LFATAL("FIXME");
      // also get the SimEventQueue:

      rutz::shared_ptr<ChannelMaps> chm        = vcxm->channelmaps();
      uint                          numSubmaps = chm->numSubmaps();

      *itsStatsFile << "# CHANNELS: ";

      for(uint i = 0; i < numSubmaps; i++)
      {
        NamedImage<float> smap = chm->getRawCSmap(i);
        *itsStatsFile << smap.name() << ", ";
      }
      *itsStatsFile << std::endl;
    }
  }

  // We flush frequently since this output is debuggy in nature or it's being used
  // to collect information which needs assurance of accuracy for instance in
  // RSVP analysis. It is better to err on the side of caution.
  (*itsStatsFile).flush();
  (*itsStatsFile).close();

  // get the basic input frame info
  if (SeC<SimEventInputFrame> e = q->check<SimEventInputFrame>(this))
  {
    itsSizeX       = e->frame().getWidth();
    itsSizeY       = e->frame().getHeight();
    itsFrameNumber = (unsigned int)e->frameNum();
    itsFrameIdx    = itsFrameNumber;
  }
  else
  {
    itsFrameNumber = itsFrameIdx;
    itsFrameIdx++;
  }

  // get the latest input frame:
  // Since we are only using it's basic statistics (Height / Width) , we don't care about it's
  // blackboard status. Use SEQ_ANY then. Otherwise, this will not fetch at any rate.
  Image< PixRGB<byte> > input;
  if (SeC<SimEventRetinaImage> e = q->check<SimEventRetinaImage>(this,SEQ_ANY))
    input = e->frame().colorByte();
  else
    LINFO("No input? Check the SimEventCue.");

  // Get the current frame number or keep track on your own
  /*
  if (SeC<SimEventInputFrame> e = q->check<SimEventInputFrame>(this))
    itsFrameIdx = e->frameNum();
  else
    itsFrameIdx++;
  */

  // get the latest raw AGM:
  Image<float> agm;
  if (SeC<SimEventAttentionGuidanceMapOutput> e =
      q->check<SimEventAttentionGuidanceMapOutput>(this))
    agm = e->agm(1.0F);
  else
    LINFO("No AGM? Check the SimEventCue.");

  // if we are missing input or agm, return:
  // We also need to warn so that we know why the stats file may be empty
  bool quit = false;
  if (input.initialized() == false)
    {
      LINFO("WARNING!!! Input seems not to be initialized, so detailed stats cannot be saved.");
      quit = true;
    }
  if(agm.initialized() == false)
    {
      LINFO("WARNING!!! NO Attention Guidance MAP \"AGM\" so detailed stats cannot be saved.");
      quit = true;
    }

  if(quit == true) return;

  // update the trajectory:
  Image< PixRGB<byte> > res;
  const int w = input.getWidth();

  // save image results? if so let's prepare it
  if (itsSaveXcombo.getVal() || itsSaveYcombo.getVal())
    {
      Image<float> nagm = getMap(*q);
      res = colGreyCombo(input, nagm, itsSaveXcombo.getVal(),
                         itsDisplayInterp.getVal());
    }

  // if we are saving single channel stats save saliency stats using a compatable format
  // SEE: SingleChannel.C / saveStats(..) for more info on format
  if (itsGetSingleChannelStats.getVal())
    saveCompat(agm);

  // Save a bunch of stats?
  if (statsFile)
    {
      // start with the current simulation time:
       statsFile <<std::endl<<"= "<<q->now().msecs()
                     <<" # time of frame in ms"<<std::endl;

      // get min/max/avg and stdev and number of peaks in AGM:
      float mi, ma, av; getMinMaxAvg(agm, mi, ma, av);
      double sdev = stdev(agm);
      double peaksum; int npeaks = findPeaks(agm, 0.0f, 255.0f, peaksum);

      // find the location of max in the AGM, at scale of original input:
      float maxval; Point2D<int> maxloc;
      findMax(agm, maxloc, maxval);
      float scale = float(w) / float(agm.getWidth());
      maxloc.i = int(maxloc.i * scale + 0.4999F);
      maxloc.j = int(maxloc.j * scale + 0.4999F);
      if (res.initialized())
        {
          drawPatch(res, maxloc, 4, COL_YELLOW);
          drawPatch(res, maxloc + Point2D<int>(w, 0), 4, COL_YELLOW);
        }

      // find the location of min in the AGM, at scale of original input:
      float minval; Point2D<int> minloc;
      findMin(agm, minloc, minval);
      minloc.i = int(minloc.i * scale + 0.4999F);
      minloc.j = int(minloc.j * scale + 0.4999F);
      if (res.initialized())
        {
          drawPatch(res, minloc, 4, COL_GREEN);
          drawPatch(res, minloc + Point2D<int>(w, 0), 4, COL_GREEN);
        }

      // save some stats for that location:
       statsFile  <<maxloc.i<<' '<<maxloc.j<<' '<<minloc.i<<' '
                  <<minloc.j<<' '<<ma<<' '<<mi<<' '<<av<<' '<<sdev
                  <<' '<<npeaks<<' '<<peaksum
                  <<" # Xmax Ymax Xmin Ymin max min avg std npeaks peaksum"
                  <<std::endl;

      // build a vector of points where we will save samples. First is
      // the max, second the min, then a bunch of random locations:
      std::vector<Point2D<int> > loc;
      loc.push_back(maxloc);
      loc.push_back(minloc);
      for (uint n = 0; n < 100; n ++)
        loc.push_back(Point2D<int>(randomUpToNotIncluding(input.getWidth()),
                              randomUpToNotIncluding(input.getHeight())));

      // Get all the conspicuity maps:
      ImageSet<float> cmap;
      //LFATAL("FIXME");
      rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
      q->request(vcxm); // VisualCortex is now filling-in the maps...
      rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();
      uint numSubmaps = chm->numSubmaps();
      for(uint i=0;i < numSubmaps; i++)
      {
        NamedImage<float> tempMap = chm->getRawCSmap(i);
        Image<float> m = tempMap;
        cmap.push_back(m);

        // also store sample points at the min/max locations:
        Point2D<int> p; float v;
        findMax(m, p, v); loc.push_back(p);
        findMin(m, p, v); loc.push_back(p);
      }
      /*
      for (uint i = 0; i < itsBrain->getVC()->numChans(); i ++)
        {
          Image<float> m = itsBrain->getVC()->subChan(i)->getOutput();
          cmap.push_back(m);

          // also store sample points at the min/max locations:
          Point2D<int> p; float v;
          findMax(m, p, v); loc.push_back(p);
          findMin(m, p, v); loc.push_back(p);
        }
      */

      // Go over all sample points and save feature map and
      // conspicuity map values at those locations:
      for (uint i = 0; i < loc.size(); i ++)
        {
          Point2D<int> p = loc[i];
          Point2D<int> pp(int(p.i / scale), int(p.j / scale));

           statsFile <<p.i<<' '<<p.j<<"     ";

          // do the conspicuity maps first. Since they are all at the
          // scale of the AGM, we use pp:
          for (uint j = 0; j < cmap.size(); j ++)
          {
            if((int(p.i / scale) < agm.getWidth()) &&
               (int(p.j / scale) < agm.getHeight()))
            {
              (statsFile)<<cmap[j].getVal(pp)<<' ';
              (statsFile)<<"    ";
            }
            else
            {
              (statsFile)<<"-1"<<' ';
              (statsFile)<<"    ";
            }
          }

          // now the feature maps, we use coordinates p:
          /* TOO BOGUS - disabled for now
          std::vector<double> f;
          itsBrain->getVC()->getFeatures(p, f);
          for (uint j = 0; j < f.size(); j ++) (*statsFile)<<f[j]<<' ';
          */

           statsFile  <<"# features "<<i<<" at ("<<p.i
                         <<", "<<p.j<<')'<<std::endl;
        }
  }

  statsFile.flush();
  statsFile.close();
  unlockFile(itsStatsFname.getVal().c_str(),fd,fl);
  // save results?
  if (res.initialized())
    ofs->writeRGB(res, "T",
                  FrameInfo("SimulationViewerStats trajectory", SRC_POS));

  // Should we compute attention gate stats
  // If we have AG stats we will save the basic LAM stats anyways
  if(itsComputeAGStats.getVal())
    computeAGStats(*q);
  else if (SeC<SimEventAttentionGateOutput> ag =
           q->check<SimEventAttentionGateOutput>(this))
    computeLAMStats(ag->lam());

  //! Save the overlap image
  if(itsOverlap.initialized())
    ofs->writeRGB(itsOverlap, "AG-STAT-MASK",
                  FrameInfo("Stats mask overlap", SRC_POS));

  if(itsVisualSegments.initialized())
    ofs->writeRGB(itsVisualSegments, "AG-STAT-SEGS",
                  FrameInfo("Stats segments", SRC_POS));

  if(itsVisualCandidates.initialized())
    ofs->writeGray(itsVisualCandidates, "AG-STAT-CAND",
                   FrameInfo("Stats candidates", SRC_POS));

}

// ######################################################################
void SimulationViewerStats::computeLAMStats(const Image<float> &img)
{

  saveCompat(img,".final-lam.txt",-1);

  Image<float> lamr = rescale(img,itsSizeX,itsSizeY);

  itsMaskCount  = 0; itsLamCount = 0; itsOverlapCount = 0;
  itsTotalCount = 0;

  // Threshold the attention mask at 12.5%
  for(Image<float>::iterator litr = lamr.beginw();
      litr != lamr.endw(); litr++)
  {
    if(*litr > 32) itsLamCount++;
    itsTotalCount++;
  }
  saveAGMaskStats(lamr,"LAM");

}

// ######################################################################
void SimulationViewerStats::computeAGStats(SimEventQueue& q)
{
  // Do we have attention gate output
  if (SeC<SimEventAttentionGateOutput> ag =
      q.check<SimEventAttentionGateOutput>(this))
  {
    Image<float> lam = ag->lam();
    saveCompat(lam,".final-lam.txt",-1);
    // Do we have a frame?

    //LINFO("Checking Frame");
    // get the mask if we havn't already
    if(!itsMask.initialized())
    {
      // Open the mask file
      const Image< PixRGB<byte> > cmask =
        Raster::ReadRGB(itsAGMaskFile.getVal());

      LINFO("Target Mask File Read %s",itsAGMaskFile.getVal().c_str());

      //Image<float> fmask = luminance(cmask);
      itsMask.resize(cmask.getWidth(),cmask.getHeight());
      Image<bool>::iterator mitr = itsMask.beginw();
      // Set mask usging a stepper to 1 or 0;
      for(Image<PixRGB<byte> >::const_iterator citr = cmask.begin();
          citr != cmask.end(); citr++)
      {
        // Above 90% gray
        if(citr->p[0] < 230) *mitr++ = true;
        else                 *mitr++ = false;
      }

      itsLamMask.resize(itsMask.getWidth(),itsMask.getHeight(),true);
      itsOverlap.resize(itsMask.getWidth(),itsMask.getHeight(),true);
    }


    // We are offset by one frame back in the AG
    // so we need to make sure we stay in sync
    if(itsMask.initialized())
    {
      LINFO("FRAME %d",itsFrameNumber);
      // Are we at the target frame + 1? or
      // are we outputing all frame overlaps?
      if((itsFrameNumber == (unsigned int)(itsAGTargetFrame.getVal() + 1)) ||
         ((itsAGTargetFrame.getVal() == 0) && (itsFrameIdx > 0)))
      {

        LINFO("Checking Mask file to target Attention Gate Frame %d",
              (itsFrameNumber + 1));
        // We need to compute basically how much of the mask and
        // the last attention gate overlap.

        // If the mask and attention gate never overlap, then the
        // target should never be seen. HOWEVER, if there is some
        // overlap, then whether the target is detected or not
        // may be more ambiguous.

        // resize lam to the mask size
        Image<float> lamr          = rescale(lam,itsMask.getWidth(),
                                             itsMask.getHeight());
        Image<bool>::iterator mitr = itsLamMask.beginw();

        // Threshold the attention mask at 25%
        for(Image<float>::iterator litr = lamr.beginw();
            litr != lamr.endw();
            litr++)
        {
          // threshold at 12.5%
          if(*litr > 32) *mitr++ = true;
          else           *mitr++ = false;
        }

        itsMaskCount  = 0; itsLamCount = 0; itsOverlapCount = 0;
        itsTotalCount = 0;

        Image<bool>::iterator aitr          = itsMask.beginw();
        Image<bool>::iterator litr          = itsLamMask.beginw();
        Image<PixRGB<byte> >::iterator oitr = itsOverlap.beginw();

        // count overlapped pixels as well as total pixels active in
        // each of the two maps
        while(litr != itsLamMask.endw())
        {
          if(*aitr++)
          {
            itsMaskCount++;
            if(*litr++)
            {
              itsLamCount++;
              itsOverlapCount++;
              *oitr++ = PixRGB<byte>(0,0,0);      // Union         = Black
            }
            else *oitr++ = PixRGB<byte>(255,0,0); // Mask Only     = Red
          }
          else if(*litr++)
          {
            itsLamCount++;
            *oitr++ = PixRGB<byte>(0,0,255);      // Att Gate Only = Blue
          }
          else
          {
            *oitr++ = PixRGB<byte>(255,255,255); // None           = White
          }
          itsTotalCount++;
        }

        //char filename[128];
        //sprintf(filename,"mask-AG%0'6d-union.png",(itsFrameIdx - 1));
        //Raster::WriteRGB(itsOverlap,filename);

        // Check constistancy of mask. Transparency in PNG can mess it up
        if(itsMaskCount      == itsTotalCount)
          LFATAL("Constancy Check failed for mask - Too Large!");
        else if(itsMaskCount == 0)
          LFATAL("Constancy Check failed for mask - Too Small!");

        saveAGMaskStats(lamr,"AG-MASK");
      }
    }

    // do we have segments from stage two?
    if (SeC<SimEventAttentionGateStageTwoSegments> agst =
        q.check<SimEventAttentionGateStageTwoSegments>(this))
    {
      LINFO("Getting Segments Image");
      const Image<int> segments = agst->obj().segments;
      Image<float> newSegments  = static_cast<Image<float> >(segments);

      float min,max,avg;

      getMinMaxAvg(newSegments,min,max,avg);

      LINFO("Segments min %f max %f",min,max);

      Image<PixHSV<float> > classImage;

      classImage.resize(segments.getWidth(),segments.getHeight());

      if((max-min) > 0.0f)
      {
        // Create a rainbow to show the classes  using HSV color space
        // 330 since 360 is red
        newSegments = ((newSegments - min) / (max - min)) * 300.0f;

        Image<PixHSV<float> >::iterator classImageItr = classImage.beginw();

        for(Image<float>::iterator segmentsItr = newSegments.beginw();
            segmentsItr != newSegments.endw(); ++segmentsItr)
       {
         classImageItr->p[0] = *segmentsItr;
         //LINFO("%f",*segmentsItr);
         classImageItr->p[1] = 100.0f;
         classImageItr->p[2] = 255.0f;
         ++classImageItr;
       }

        // convert back to RGB
        Image<PixRGB<float> > temp =
          static_cast<Image<PixRGB<float> > >(classImage);

        itsVisualSegments = temp;
      }

      const Image<bool> cand = agst->candidates();
      Image<float> newCand   = static_cast<Image<float> >(cand);

      newCand = newCand * 255.0f;

      itsVisualCandidates = newCand;

    }
  }
}


// ######################################################################
void SimulationViewerStats::saveCompat(const Image<float>& img,
                                       const std::string suffix,
                                       const int frameOffset)
{


  unsigned int frameIdx = (unsigned int)((int)itsFrameNumber + frameOffset);

  std::string fileName;
  std::string txt = suffix;
  fileName = itsGetSingleChannelStatsFile.getVal() + txt;

  ushort minx = 0, miny = 0, maxx = 0, maxy = 0;
  float  min,  max,  avg,  std;
  uint   N;
  // get a whole bunch of stats about this output image
  getMinMaxAvgEtc(img, min, max, avg, std, minx, miny, maxx, maxy, N);

  LINFO("SAVING COMPAT STATS TO %s",fileName.c_str());

  std::ofstream statsFile(fileName.c_str(), std::ios::app);

  statsFile << "final" << "\t";

  statsFile << frameIdx << "\t";

  // differentiate between scale image stats and the combined max norm
  // for compatability with single channel stats

  statsFile << "COMBINED\t-1\t";
  //itsFrameIdx++;

  statsFile << "final" << "\t" << "FINAL" << "\t";
  statsFile << min  << "\t" << max  << "\t" << avg  << "\t" << std  << "\t"
            << minx << "\t" << miny << "\t" << maxx << "\t" << maxy << "\t"
            << N    << "\n";
  statsFile.close();

  if(itsSaveStatsPerChannelFreq.getVal())
    {
      std::string txt = ".final.freq.txt";
      fileName = itsGetSingleChannelStatsFile.getVal() + txt;

      FFTWWrapper fft(img.getWidth(), img.getHeight());
      double dimg[img.getHeight() * img.getWidth()];
      Image<float>::const_iterator itr = img.begin();
      double *ditr = &dimg[0];
      while(itr != img.end()) *ditr++ = double(*itr++);
      fft.init(dimg);
      double mag[img.getHeight() * (img.getWidth()/2 + 1)];
      fft.compute(mag);

      std::ofstream freqFile(fileName.c_str(), std::ios::app);
      freqFile << "final" << "\t";
      freqFile << frameIdx << "\t";

      freqFile << "COMBINED\t-1\t";

      freqFile << "SIZE\t" << img.getWidth() <<"\t"<< img.getHeight() << "\n";

      for(int i = 0; i < img.getHeight(); i++)
        {
          for(int j = 0; j < (img.getWidth()/2 + 1); j++)
            freqFile << mag[i * (img.getWidth()/2 + 1) + j] << "\t";
          freqFile << "\n";
        }
      freqFile.close();
    }
}

// ######################################################################
void SimulationViewerStats::saveAGMaskStats(const Image<float> &img,
                                            const std::string caller,
                                            const std::string suffix)
{

  ushort minx = 0, miny = 0, maxx = 0, maxy = 0;
  float  min,  max,  avg,  std;
  uint   N;
  // get a whole bunch of stats about this output image
  getMinMaxAvgEtc(img, min, max, avg, std, minx, miny, maxx, maxy, N);

  std::string fileName;
  std::string txt = suffix;
  fileName = itsGetSingleChannelStatsFile.getVal() + txt;

  LINFO("SAVING AG STATS TO %s",fileName.c_str());

  std::ofstream statsFile(fileName.c_str(), std::ios::app);

  statsFile << "AGstats" << "\t";

  statsFile << (itsFrameNumber - 1) << "\t";

  // differentiate between scale image stats and the combined max norm
  // for compatability with single channel stats

  statsFile << "COMBINED\t" << caller << "\t";
  //itsFrameIdx++;

  statsFile << "final" << "\t" << "FINAL" << "\t";
  statsFile << itsTotalCount   << "\t"
            << itsMaskCount    << "\t"
            << itsLamCount     << "\t"
            << itsOverlapCount << "\t";
  statsFile << min  << "\t" << max  << "\t" << avg  << "\t" << std  << "\t"
            << minx << "\t" << miny << "\t" << maxx << "\t" << maxy << "\t"
            << N    << "\n";

  statsFile.close();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
