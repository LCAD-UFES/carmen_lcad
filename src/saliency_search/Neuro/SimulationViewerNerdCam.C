/*!@file Neuro/SimulationViewerNerdCam.C entry interface between INVT and ASAC */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerNerdCam.C $
// $Id: SimulationViewerNerdCam.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Neuro/SimulationViewerNerdCam.H"

#include "Channels/BlueYellowChannel.H"
#include "Channels/ChannelBase.H"
#include "Channels/ColorChannel.H"
#include "Channels/DirectionChannel.H"
#include "Channels/FlickerChannel.H"
#include "Channels/GaborChannel.H"
#include "Channels/IntensityChannel.H"
#include "Channels/MotionChannel.H"
#include "Channels/OrientationChannel.H"
#include "Channels/RedGreenChannel.H"
#include "Image/ColorOps.H"    // for normalizeC()
#include "Image/CutPaste.H"    // for concatX()
#include "Image/DrawOps.H"
#include "Image/MathOps.H"   // for takeMax()
#include "Image/PyramidOps.H"
#include "Image/ShapeOps.H"  // for crop()
#include "Image/Transforms.H"  // for contour2D()
#include "Image/LevelSpec.H"
#include "Neuro/AttentionGuidanceMap.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/TaskRelevanceMap.H"
#include "Raster/Raster.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/readConfig.H"
#include <time.h>
#include <stdio.h>
#include <unistd.h>


// ######################################################################
SimulationViewerNerdCam::SimulationViewerNerdCam(OptionManager& mgr,
                                           const std::string& descrName,
                                           const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventWTAwinner),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsMetrics(new SpatialMetrics(mgr)),
  itsColorBoring("SVcolorBoring", this, PixRGB<byte>(127, 0, 0)),
  itsColorLink("SVcolorLink", this, PixRGB<byte>(255, 0, 0)),
  itsColorNormal("SVcolorNormal", this, PixRGB<byte>(255, 255, 0)),
  itsConfigFile(&OPT_NerdCamConfigFile, this),
  itsCropFOA(&OPT_SVcropFOA, this),
  itsDisplayAdditive(&OPT_SVdisplayAdditive, this),
  itsDisplayBoring(&OPT_SVdisplayBoring, this),
  itsDisplayFOA(&OPT_SVdisplayFOA, this),
  itsDisplayFOALinks(&OPT_SVdisplayFOALinks, this),
  itsDisplayHighlights(&OPT_SVdisplayHighlights, this),
  itsDisplayPatch(&OPT_SVdisplayPatch, this),
  itsDisplaySMmodulate(&OPT_SVdisplaySMmodulate, this),
  itsDisplayTime(&OPT_SVdisplayTime, this),
  itsFOApsiz("SVfoapsiz", this, 3),
  itsFOAthick("SVfoathick", this, 3),
  itsFOAlinkThick("SVfoalinkthick", this, 2),
  itsMegaCombo(&OPT_SVmegaCombo, this),
  itsWarp3Dpitch("SVwarp3DInitialPitch", this, -25.0F),
  itsWarp3Dyaw("SVwarp3DInitialYaw", this, -15.0F),
  itsWarp3DpitchRate("SVwarp3DpitchRate", this, 0.0F),
  itsWarp3DyawRate("SVwarp3DyawRate", this, 15.0F),
  itsWarp3DpitchMax("SVwarp3DpitchMax", this, 0.0F),
  itsWarp3DyawMax("SVwarp3DyawMax", this, 20.0F),
  itsUseLargerDrawings(&OPT_SVuseLargerDrawings, this),
  itsCumFOAmask(),
  itsCurrFOA(WTAwinner::NONE()),
  itsCurrFOAmask(),
  itsCurrTime(),
  itsDims3D(),
  itsMultiTraj(),
  itsPrevFOA(WTAwinner::NONE()),
  itsTraj(),
  itsPitch3D(-1.0e10F),
  itsYaw3D(-1.0e10F)
{
  this->addSubComponent(itsMetrics);
  itsInit = false;
  nlog("INIT - Normal startup recognized");
}

// ######################################################################
SimulationViewerNerdCam::~SimulationViewerNerdCam()
{
  nlog("SHUTDOWN - normal shutdown recognized");
}

// ######################################################################
void SimulationViewerNerdCam::init(const ushort baseSizeX,
                                   const ushort baseSizeY)
{
  char        base[100];
  std::string logstr;

  itsBaseSizeX = baseSizeX;
  itsBaseSizeY = baseSizeY;

  const std::string configFile = itsConfigFile.getVal();
  readConfig config(25);
  config.openFile(configFile.c_str(),false);

  itsLogFile           = config.getItemValueS("LogFile");
  sprintf(base,"INIT - Config File %s",configFile.c_str());
  logstr = base; nlog(logstr);

  itsOutputMotionImage = config.getItemValueS("OutputMotionImage");
  itsOutputSalMapImage = config.getItemValueS("OutputSalMapImage");
  itsOutput3DImage     = config.getItemValueS("Output3DImage");
  itsOutputMegaImage   = config.getItemValueS("OutputMegaImage");
  itsOutputTrajImage   = config.getItemValueS("OutputTrajImage");
  itsWebPageFile       = config.getItemValueS("WebPageFile");
  itsStatusFile        = config.getItemValueS("StatusFile");
  itsStatusHeader      = config.getItemValueS("StatusHeader");
  itsStatusFooter      = config.getItemValueS("StatusFooter");
  itsChannelFile       = config.getItemValueS("ChannelFile");
  itsBaseURL           = config.getItemValueS("BaseURL");
  itsBaseName          = config.getItemValueS("BaseName");
  itsMotionThreshold   = config.getItemValueF("MotionThreshold");

  sprintf(base,"INIT - Image %d x %d",itsBaseSizeX,itsBaseSizeY);
  logstr = base; nlog(logstr);
  nlog("INIT - Init complete");
  itsInit = true;

  // Get server Start Time
  time_t rawtime; struct tm * timeinfo;
  time ( &rawtime ); timeinfo = localtime ( &rawtime );
  itsStartTime   =  asctime (timeinfo);
  itsTotalFrames = 0;
}

// ######################################################################
void SimulationViewerNerdCam::reset1()
{
  itsCurrFOA     = WTAwinner::NONE();
  itsCurrFOAmask.freeMem();
  itsCurrTime    = SimTime::ZERO();
  itsCumFOAmask.freeMem();
  itsDims3D      = Dims();
  itsHasNewInput = false;
  itsMultiTraj.reset();
  itsPitch3D     = -1.0e10F;
  itsPrevFOA     = WTAwinner::NONE();
  itsTraj.freeMem();
  itsYaw3D       = -1.0e10F;

  SimulationViewer::reset1();
  nlog("INIT - Model reset complete");
}

// ######################################################################
void SimulationViewerNerdCam::start2()
{
  // do we wnat larger drawings?
  if (itsUseLargerDrawings.getVal())
  {
    itsFOApsiz.setVal(5);
  }
}

// ######################################################################
void SimulationViewerNerdCam::
onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  LINFO("Inputing to Nerd-cam");
  // keep a copy of the image
  itsInput = e->frame().asRgb();

  if (!itsInit) init(itsInput.getWidth(), itsInput.getHeight());

  // erase old trajectory and replace it by fresh new input:
  itsTraj = itsInput;
  // we have a new input; this will force redrawing various things on
  // the trajectory in case people request it before a new shift of
  // attention or other event occurrs:
  itsHasNewInput = true;
  itsTotalFrames++;
}

// ######################################################################
void SimulationViewerNerdCam::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
  const SimTime t = q.now();

  // do the 3D rotation:
  if (itsPitch3D == -1.0e10F)
    itsPitch3D = itsWarp3Dpitch.getVal();
  if (itsYaw3D == -1.0e10F)
    itsYaw3D = itsWarp3Dyaw.getVal();

  itsPitch3D += itsWarp3DpitchRate.getVal() * (t - itsCurrTime).secs();
  itsYaw3D += itsWarp3DyawRate.getVal() * (t - itsCurrTime).secs();

  if (itsYaw3D >= itsWarp3DyawMax.getVal() ||
      itsYaw3D <= -itsWarp3DyawMax.getVal())
    itsWarp3DyawRate.setVal(- itsWarp3DyawRate.getVal());
  if (itsPitch3D >= itsWarp3DpitchMax.getVal() ||
      itsPitch3D <= -itsWarp3DpitchMax.getVal())
    itsWarp3DpitchRate.setVal(- itsWarp3DpitchRate.getVal());

  itsCurrTime = t;
}

// ######################################################################
void SimulationViewerNerdCam::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  // Any output from the ShapeEstimator?
  Image<byte> foaMask;
  if (SeC<SimEventShapeEstimatorOutput> ee = q.check<SimEventShapeEstimatorOutput>(this))
    {
      foaMask = Image<byte>(ee->smoothMask() * 255.0F);
      if (foaMask.isSameSize(itsInput) == false)
        LFATAL("Dimensions of FOAmask must match those of input");
    }

  // check dims of mask:
  if (foaMask.initialized() && foaMask.isSameSize(itsInput) == false)
    LFATAL("Dimensions of FOAmask must match those of input");

  // update our internals:
  itsPrevFOA = itsCurrFOA; itsCurrFOA = e->winner();
  if (foaMask.initialized())
    itsCurrFOAmask = foaMask;  // keep a copy of the FOA mask
  else
    {
      // draw a disk at current foa position:
      itsCurrFOAmask.resize(itsInput.getDims(), true);
      if (itsCurrFOA.isValid())
        drawDisk(itsCurrFOAmask, itsCurrFOA.p,
                 itsMetrics->getFOAradius(), byte(255));
    }

  // update cumulative FOA mask:
  if (itsDisplayAdditive.getVal() &&            // cumulative display?
      itsCumFOAmask.initialized() &&            // not first frame?
      itsCumFOAmask.isSameSize(itsCurrFOAmask)) // not changing frame dims?
    itsCumFOAmask = takeMax(itsCumFOAmask, itsCurrFOAmask);
  else
    itsCumFOAmask = itsCurrFOAmask;

  // forget it if we don't have a traj yet:
  if (itsTraj.initialized() == false) return;

  // draw the FOA:
  if (itsDisplayFOA.getVal() || itsDisplayPatch.getVal()) drawFOA();

  // link the last two FOAs:
  if (itsDisplayFOALinks.getVal()) linkFOAs();
}

// ######################################################################
Image< PixRGB<byte> > SimulationViewerNerdCam::getTraj(SimEventQueue& q)
{
  //LINFO("Get Traj");
  itsCurrTime = q.now(); bool redraw = false;
  if (itsTraj.initialized() == false) return itsTraj;

  // ##### if not doing additive displays, clear traj and redraw only
  // ##### current foa/eye/head data:
  if (itsDisplayAdditive.getVal() == false)
    { itsTraj = itsInput; redraw = true; }

  // ##### re-do a bunch of drawings if we destroyed the traj
  // ##### (foveation, highlights, etc). We only care about current
  // ##### attention/eye/head position and not about any previous ones
  // ##### or links:
  if (redraw || itsHasNewInput)
  {
    if (itsDisplayFOA.getVal() || itsDisplayPatch.getVal()) drawFOA();
    itsHasNewInput = false;
  }

  // ##### let's get ready to return the traj, but we may reshape it
  // ##### just before that, or return something that involves the
  // ##### traj plus some other maps in some combined display:
  Image< PixRGB<byte> > ret;

  // if cropping let's crop now:
  // resizing and other normalizations are taken care of by FrameOstream
  if (itsCropFOA.getVal().isNonEmpty())
  {
    Dims crop_dims = itsCropFOA.getVal();
    Rectangle crect =
      Rectangle::tlbrI(itsCurrFOA.p.j - crop_dims.h() / 2,
                      itsCurrFOA.p.i - crop_dims.w() / 2,
                      itsCurrFOA.p.j + crop_dims.h() / 2 - 1,
                      itsCurrFOA.p.i + crop_dims.w() / 2 - 1);
    ret = crop(itsTraj, crect, true);
  }
  else ret = itsTraj;

  // ##### do a bunch of last-minute drawings:
  if (itsDisplayTime.getVal()) drawTime(ret);

  return ret;
}

// ######################################################################
void SimulationViewerNerdCam::saveResults(const nub::ref<FrameOstream>& ofs,
                                          SimEventQueue& q)
{
  // update the trajectory:
  const Image< PixRGB<byte> > traj = getTraj(q);

  const double msecs = itsCurrTime.msecs();

  LINFO("Running Nerd-cam on Sample Input time %f ms",msecs);
  Image<float> salMap             = getMap(q);

  // write out saliency map
  itsSalMap = rescale(salMap,itsBaseSizeX,itsBaseSizeY);

  // write web status page

  const Image<byte> bsal = itsSalMap;
  struct flock fl_smap;
  int fd_smap;
  lockFile(itsOutputSalMapImage,fd_smap,fl_smap);
  Raster::WriteRGB(bsal,itsOutputSalMapImage);


  nub::soft_ref<MotionChannel>      mc;

  LFATAL("FIXME I should derive from SimulationViewerStd");
  /*
  if(vc->hasSubChan("motion"))
    dynCastWeakToFrom(mc, vc->subChan("motion"));
  else
  {
    nlog("FATAL - No motion channel supplied by brain");
    LFATAL("No Motion Channel Supplied by Brain");
  }
  */
  if (mc.isInvalid())
  {
    nlog("FATAL - Channel named 'motion' not a MotionChannel");
    LFATAL("Channel named 'motion' not a MotionChannel");
  }

  int fd_traj, fd_mega;
  struct flock fl_traj;
  struct flock fl_mega;
  if(itsTraj.initialized())
  {
    // draw simple trajectory image
    lockFile(itsOutputTrajImage,fd_traj,fl_traj);
    Raster::WriteRGB(traj,itsOutputTrajImage);
    // Draw a megacombo
    if(itsTotalFrames > 1)
    {
      const Image< PixRGB<byte> > meg = drawMegaCombo(q);
      lockFile(itsOutputMegaImage,fd_mega,fl_mega);
      Raster::WriteRGB(meg,itsOutputMegaImage);
      itsComboSizeX = meg.getWidth();
      itsComboSizeY = meg.getHeight();
    }
  }

  float min = 0, max = 0, avg = 0;
  float div = 0;

  bool doMotion = false;

  for(ushort i = 0; i < mc->numChans(); i++)
  {
    float mi,ma,av;
    const Image<float> ftmp = mc->dirChan(i).getOutput();
    getMinMaxAvg(ftmp,mi,ma,av);
    min += mi; max += ma; avg += av;
    div++;
    if(ma > itsMotionThreshold)
      doMotion = true;
  }
  min = min/div; max = max/div; avg = avg/div;


  // write an image, something is moving
  int fd_mot;
  struct flock fl_mot;
  if(doMotion)
  {
    LINFO("Drawing Motion Image");
    const Image<PixRGB<byte> > bimg = itsInput;
    lockFile(itsOutputMotionImage,fd_mot,fl_mot);
    Raster::WriteRGB(bimg,itsOutputMotionImage);
  }

  // draw nifty 3D saliency map
  Image< PixRGB<byte> > ret = itsInput;
  drawGrid(ret, ret.getWidth() / 6, ret.getHeight() / 6, 3, 3,
           itsColorNormal.getVal());
  drawRect(ret, Rectangle::tlbrI(1, 1, ret.getHeight()-2, ret.getWidth()-2),
           itsColorNormal.getVal(), 3);
  ret = warp3Dmap(ret, getMap(q), itsPitch3D, itsYaw3D, itsDims3D);
  int fd_3d;
  struct flock fl_3d;
  lockFile(itsOutput3DImage,fd_3d,fl_3d);
  Raster::WriteRGB(ret,itsOutput3DImage);
  its3DSizeX = ret.getWidth();
  its3DSizeY = ret.getHeight();

  writeStatusPage();

  if(itsTotalFrames > 1)
  {
    writeChannelPage();
  }

  // unlock all our image files
  if(avg > itsMotionThreshold)
  {
    unlockFile(itsOutputMotionImage,fd_mot,fl_mot);
  }
  if(itsTraj.initialized())
  {
    unlockFile(itsOutputTrajImage,fd_traj,fl_traj);
    if(itsTotalFrames > 1)
    {
      unlockFile(itsOutputMegaImage,fd_mega,fl_mega);
    }
  }
  unlockFile(itsOutputSalMapImage,fd_smap,fl_smap);
  unlockFile(itsOutput3DImage,fd_3d,fl_3d);
}

// ######################################################################
void SimulationViewerNerdCam::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void SimulationViewerNerdCam::save1(const ModelComponentSaveInfo& sinfo)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs =
    dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

  // also get the SimEventQueue:
  SimEventQueue *q = dynamic_cast<const SimModuleSaveInfo&>(sinfo).q;

  saveResults(ofs, *q);
}

// #####################################################################
void SimulationViewerNerdCam::writeStatusPage() const
{
  std::ifstream headFile(itsStatusHeader.c_str(),std::ios::in);
  std::ifstream footFile(itsStatusFooter.c_str(),std::ios::in);

  // get min/max/avg and stdev and number of peaks:
  double peaksum;
  const double sdev = stdev(itsSalMap);
  float mi, ma, av; getMinMaxAvg(itsSalMap, mi, ma, av);
  const int npeaks = findPeaks(itsSalMap, 0.0f, 255.0f, peaksum);

  // find the location of max in the salmap, at scale of original input:
  float maxval; Point2D<int> maxloc;
  findMax(itsSalMap, maxloc, maxval);
  const float scale = float(itsInput.getWidth()) / float(itsSalMap.getWidth());
  maxloc.i = int(maxloc.i * scale + 0.4999F);
  maxloc.j = int(maxloc.j * scale + 0.4999F);

  // find the location of min in the salmap, at scale of original input:
  float minval; Point2D<int> minloc;
  findMin(itsSalMap, minloc, minval);
  minloc.i = int(minloc.i * scale + 0.4999F);
  minloc.j = int(minloc.j * scale + 0.4999F);

  int fd; struct flock fl;
  lockFile(itsStatusFile,fd,fl);
  std::ofstream statFile(itsStatusFile.c_str(),std::ios::out);

  std::string in;
  while (headFile >> in)
  {
    if(!in.compare("\\"))
      statFile << "\n";
    else
      statFile << in << " ";
  }
  statFile.flush();
  statFile << "\n<!-- End Header File -->\n"
           << "<!--\n"
           << "Nerd Cam Daemon\n"
           << "T. Nathan Mundhenk and Laurent Itti\n"
           << "http://www.nerd-cam.com\n"
           << "-->\n"
           << "<a href=\"" << itsBaseURL  << "\">"
           << "<H1>"       << itsBaseName << "</a> "
           << "Detailed Server Status</H1>\n";

  time_t rawtime; struct tm * timeinfo;
  time ( &rawtime ); timeinfo = localtime ( &rawtime );

  // general server stats
  statFile << "<H2>Nerd-Cam Daemon Saliency Server Statistics </H2>\n"
           << "<table border=\"0\" width=\"800\">\n"
           << "<tr>\n"
           << "\t<td width=\"275\">\n"
           << "\t\t<H3>Current Time </H3>"
           << asctime (timeinfo) << "<br>\n"
           << "\t\t<H3>Server Start Time </H3>"
           << itsStartTime << "<br>\n"
           << "\t\t<H3>Total Frames Processed </H3>"
           << itsTotalFrames << "<br>\n"
           << "\t\t<p>&nbsp;</td>\n"
           << "\t<td width=\"525\" align=\"center\">\n"
           << "\t\t<h1>Last Nerd Spotted by Nerd-Cam</h1>\n"
           << "\t\t<p><img border=\"0\" src=\"nerd-cam.motion.png\" "
           << "width=\"320\" height=\"240\"></p>\n"
           << "\t</td>\n"
           << "</tr>\n"
           << "</table>\n";

  statFile.flush();
  // save some stats for that location:
  statFile << "<H2>Current Saliency Map Statistics </H2>\n"
           << "<table border=0 cellpadding=0 width=800>\n"
           << "<tr>\n"
           << "\t<td width=275 valign=top>\n"
           << "\t\t<H3>Point of Maximum Saliency </H3>"
           << "(" << maxloc.i << "," << maxloc.j << ")<br>\n"
           << "\t\t<H3>Point of Minimum Saliency </H3>"
           << "(" << minloc.i << "," << minloc.j << ")<br>\n"
           << "\t\t<H3>Maximum Saliency Value </H3>"
           << ma      << "<br>\n"
           << "\t\t<H3>Minimum Saliency Value </H3>"
           << mi      << "<br>\n"
           << "\t\t<H3>Average Saliency Value </H3>"
           << av      << "<br>\n"
           << "\t\t<H3>Standard Deviation </H3>"
           << sdev    << "<br>\n"
           << "\t\t<H3>Number of Peaks </H3>"
           << npeaks  << "<br>\n"
           << "\t\t<H3>Peak Sum </H3>"
           << peaksum << "<br>\n"
           << "\t</td>\n"
           << "\t<td width=525 valign=top align=\"center\">\n"
           << "\t\t<H1>Superimposed 3D Saliency Image</H1>\n"
           << "\t\t<img border=0 width="
           << its3DSizeX
           << " height="
           << its3DSizeY
           << " src=\"awesome.3d.image.png\" "
           << " alt=\"Awesome 3D Saliency Map "
           << "to shock and awe\"><br>\n"
           << "\t</td>\n"
           << "</tr>\n"
           << "</table>\n";
  statFile.flush();
  statFile << "<!-- Start Footer File -->\n";
  while (footFile >> in)
  {
    if(!in.compare("\\"))
      statFile << "\n";
    else
      statFile << in << " ";
  }

  statFile.flush();
  statFile.close();
  while(statFile.is_open() != 0){}
  unlockFile(itsStatusFile,fd,fl);
}

// #####################################################################
void SimulationViewerNerdCam::writeChannelPage() const
{
  std::ifstream headFile(itsStatusHeader.c_str(),std::ios::in);
  std::ifstream footFile(itsStatusFooter.c_str(),std::ios::in);

  int fd; struct flock fl;
  lockFile(itsChannelFile,fd,fl);
  std::ofstream statFile(itsChannelFile.c_str(),std::ios::out);

  std::string in;
  while (headFile >> in)
  {
    if(!in.compare("\\"))
      statFile << "\n";
    else
      statFile << in << " ";
  }
  statFile.flush();
  statFile << "\n<!-- End Header File -->\n"
           << "<!--\n"
           << "Nerd Cam Daemon\n"
           << "T. Nathan Mundhenk and Laurent Itti\n"
           << "http://www.nerd-cam.com\n"
           << "-->\n"
           << "<a href=\"" << itsBaseURL  << "\">"
           << "<H1>"       << itsBaseName << "</a> "
           << "Detailed Channel Status</H1>\n"
           << "<H2>Mega Combo Saliency Image</H2>\n"
           << "<img border=0 width="
           << itsComboSizeX
           << " height="
           << itsComboSizeY
           << " src=\"mega.surprise.combo.png\" "
           << " alt=\"Mega Combo Saliency Map with all the topings "
           << "for your viewing pleasure\"><br>\n";

  statFile << "<H2>Current Saliency Map Statistics Per Channel</H2>\n"
           << "<table border=0 cellpadding=0 width=900>\n"
           << "<tr>\n"
           << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
           << "\t\t<H2>Channel Type</H2>\n"
           << "\t</td>\n"
           << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
           << "\t\t<H2>Maximum Saliency</H2>\n"
           << "\t</td>\n"
           << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
           << "\t\t<H2>Minimum Saliency</H2>\n"
           << "\t</td>\n"
           << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
           << "\t\t<H2>Average Saliency</H2>\n"
           << "\t</td>\n"
           << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
           << "\t\t<H2>Saliency Peaks</H2>\n"
           << "\t</td>\n"
           << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
           << "\t\t<H2>Saliency Peaks Sum</H2>\n"
           << "\t</td>\n"
           << "</tr>\n";
  statFile.flush();

  LFATAL("FIXME I should derive from SimulationViewerStd");
  /*

  const nub::ref<VisualCortex> vc = itsBrain->getVC();
  if (vc->hasSubChan("color"))
  {
    const Image<float> ftmp = vc->subChan("color")->getOutput();
    double peaksum;
    float mi, ma, av; getMinMaxAvg(ftmp, mi, ma, av);
    const int npeaks = findPeaks(ftmp, 0.0f, 255.0f, peaksum);
    statFile << "<tr>\n"
             << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
             << "\t\t<H2>Color Channel</H2>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << ma      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << mi      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << av      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << npeaks  << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << peaksum << "</H3>\n"
             << "\t</td>\n"
             << "</tr>\n";
  }
  statFile.flush();
  if (vc->hasSubChan("intensity"))
  {
    const Image<float> ftmp = vc->subChan("intensity")->getOutput();
    double peaksum;
    float mi, ma, av; getMinMaxAvg(ftmp, mi, ma, av);
    const int npeaks = findPeaks(ftmp, 0.0f, 255.0f, peaksum);
    statFile << "<tr>\n"
             << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
             << "\t\t<H2>Intensity Channel</H2>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << ma      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << mi      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << av      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << npeaks  << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << peaksum << "</H3>\n"
             << "\t</td>\n"
             << "</tr>\n";

  }
  statFile.flush();
  if (vc->hasSubChan("orientation"))
  {
    const Image<float> ftmp = vc->subChan("orientation")->getOutput();
    double peaksum;
    float mi, ma, av; getMinMaxAvg(ftmp, mi, ma, av);
    const int npeaks = findPeaks(ftmp, 0.0f, 255.0f, peaksum);
    statFile << "<tr>\n"
             << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
             << "\t\t<H2>Orientation Channel</H2>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << ma      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << mi      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << av      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << npeaks  << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n"
             << "\t\t<H3>" << peaksum << "</H3>\n"
             << "\t</td>\n"
             << "</tr>\n";
  }
  statFile.flush();
  if (vc->hasSubChan("flicker"))
  {
    const Image<float> ftmp = vc->subChan("flicker")->getOutput();
    double peaksum;
    float mi, ma, av; getMinMaxAvg(ftmp, mi, ma, av);
    const int npeaks = findPeaks(ftmp, 0.0f, 255.0f, peaksum);
    statFile << "<tr>\n"
             << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
             << "\t\t<H2>Flicker Channel</H2>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << ma      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << mi      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << av      << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << npeaks  << "</H3>\n"
             << "\t</td>\n"
             << "\t<td width=\"150\">\n"
             << "\t\t<H3>" << peaksum << "</H3>\n"
             << "\t</td>\n"
             << "</tr>\n";
  }
  statFile.flush();
  if (vc->hasSubChan("motion"))
  {
    nub::soft_ref<MotionChannel> mc; dynCastWeakToFrom(mc, vc->subChan("motion"));
    if (mc.isInvalid())
    {
      nlog(" FATAL- In writing channel page Channel named 'motion' not a MotionChannel");
      LFATAL("Channel named 'motion' not a MotionChannel");
    }
    for(uint i = 0; i < mc->numChans(); i++)
    {
      Image<float> ftmp = mc->dirChan(i).getOutput();
      double peaksum;
      float mi, ma, av; getMinMaxAvg(ftmp, mi, ma, av);
      const int npeaks = findPeaks(ftmp, 0.0f, 255.0f, peaksum);
      statFile << "<tr>\n"
               << "\t<td width=\"150\" bgcolor=\"#AAD0F0\">\n"
               << "\t\t<H2>";

      if(i == 0)
        statFile << "MotionRight";
      else if(i == 1)
        statFile << "MotionDown";
      else if(i == 2)
        statFile << "MotionLeft";
      else if(i == 3)
        statFile << "MotionUp";
      else
        statFile << "Motion " << i;

      statFile << "</H2>\n"
               << "\t</td>\n";

      if(i%2 == 0)
        statFile << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n";
      else
        statFile << "\t<td width=\"150\">\n";

      statFile << "\t\t<H3>" << ma      << "</H3>\n"
               << "\t</td>\n";

      if(i%2 == 0)
        statFile << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n";
      else
        statFile << "\t<td width=\"150\">\n";

      statFile << "\t\t<H3>" << mi      << "</H3>\n"
               << "\t</td>\n";

      if(i%2 == 0)
        statFile << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n";
      else
        statFile << "\t<td width=\"150\">\n";

      statFile << "\t\t<H3>" << av      << "</H3>\n"
               << "\t</td>\n";

      if(i%2 == 0)
        statFile << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n";
      else
        statFile << "\t<td width=\"150\">\n";

      statFile << "\t\t<H3>" << npeaks  << "</H3>\n"
               << "\t</td>\n";

      if(i%2 == 0)
        statFile << "\t<td width=\"150\" bgcolor=\"#C0C0C0\">\n";
      else
        statFile << "\t<td width=\"150\">\n";

      statFile << "\t\t<H3>" << peaksum << "</H3>\n"
               << "\t</td>\n"
               << "</tr>\n";
    }
  }
  statFile.flush();
  */

  statFile << "</table>\n"
           << "<!-- Start Footer File -->\n";
  while (footFile >> in)
  {
    if(!in.compare("\\"))
      statFile << "\n";
    else
      statFile << in << " ";
  }

  statFile.flush();
  statFile.close();
  while(statFile.is_open() != 0){}
  unlockFile(itsChannelFile,fd,fl);
}

// #####################################################################
void SimulationViewerNerdCam::drawTime(Image<PixRGB<byte> >& image) const
{
  char txt[20]; sprintf(txt, " %dms ", int(itsCurrTime.msecs() + 0.4999));
  writeText(image, Point2D<int>(0, 0), txt);
}

// #####################################################################
void SimulationViewerNerdCam::lockFile(const std::string fileName,
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
      LINFO("Cannot lock file '%s' Error code '%d'",fileName.c_str(),errno);
    }
  }
}

// #####################################################################
void SimulationViewerNerdCam::unlockFile(const std::string fileName,
                                         const int fd,
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

// #####################################################################
void SimulationViewerNerdCam::nlog(const std::string logData) const
{
  time_t rawtime; struct tm * timeinfo;
  time ( &rawtime ); timeinfo = localtime ( &rawtime );

  int fd; struct flock fl;
  lockFile(itsLogFile,fd,fl);
  std::ofstream logFile(itsLogFile.c_str(),std::ios::app);

  logFile << asctime (timeinfo)  << " - " << logData << "\n";

  logFile.close();
  unlockFile(itsLogFile,fd,fl);
}

// #####################################################################
void SimulationViewerNerdCam::drawDateTime(Image<PixRGB<byte> >& image) const
{
  time_t rawtime; struct tm * timeinfo;
  time ( &rawtime ); timeinfo = localtime ( &rawtime );

  writeText(image, Point2D<int>(0, 0), asctime (timeinfo));
}

// ######################################################################
void SimulationViewerNerdCam::drawFOA()
{
  if (itsCurrFOA.isValid() == false) return;

  // select a drawing color:
  PixRGB<byte> col(itsColorNormal.getVal());
  if (itsCurrFOA.boring) col -= itsColorBoring.getVal();

  // draw patch at current eye position:
  drawPatch(itsTraj, itsCurrFOA.p, itsFOApsiz.getVal(), col);

  // display focus of attention, possibly object-shaped:
  if (itsDisplayFOA.getVal())
    drawMaskOutline(itsTraj, itsCurrFOAmask, col, itsFOAthick.getVal(),
                    itsCurrFOA.p, itsMetrics->getFOAradius());
}

// ######################################################################
void SimulationViewerNerdCam::linkFOAs()
{
  if (itsCurrFOA.isValid() == false) return;

  const PixRGB<byte> col = itsColorLink.getVal();
  if (itsPrevFOA.isValid())
    {
      int d = int(itsPrevFOA.p.distance(itsCurrFOA.p));
      if (d > 0) drawArrow(itsTraj, itsPrevFOA.p, itsCurrFOA.p,
                           col, itsFOAlinkThick.getVal());
    }
}

// #####################################################################
Image< PixRGB<byte> > SimulationViewerNerdCam::
drawMegaCombo(SimEventQueue& q) const
{
  LINFO("Drawing Mega Combo");

  const PixRGB<byte> bg(128, 128, 255);
  const short XWW = itsTraj.getWidth() * 2;
  const short XWH = itsTraj.getHeight() * 2;
  Image< PixRGB<byte> > xwi(XWW/2, XWH/2, ZEROS);
  Image< PixRGB<byte> > xwin(XWW, XWH, ZEROS);

  Rectangle r(Point2D<int>(0,0), Dims(XWW/2, XWH/2));
  inplaceEmbed(xwi, itsTraj, r, bg, true);
  inplacePaste(xwin, xwi, Point2D<int>(0, 0));

  // get the normalized SM:
  const Image<float> agm = getMap(q);
  Image< PixRGB<byte> > xwi2(XWW/2, XWH/2, NO_INIT);

  // get the non-normalized TRM (neutral is 1.0):
  Image<float> trm;
  if (SeC<SimEventTaskRelevanceMapOutput> e =
      q.check<SimEventTaskRelevanceMapOutput>(this, SEQ_ANY))
    trm = e->trm(1.0F);
  else LFATAL("Cannot find a TRM!");

  // useful range is 0.0 .. 3.0; let's show values smaller
  // than 1 as red and those larger that 1 as green:
  Image<byte> rr, gg, bb;
  Image<float> trmfac(trm);
  inplaceClamp(trmfac, 0.0F, 1.0F);
  trmfac *= -1.0F; trmfac += 1.0F;
  rr = trmfac * 255.0F; // the redder, the lower the relevance
  trmfac = trm;
  inplaceClamp(trmfac, 1.0F, 3.0F);
  trmfac -= 1.0F;
  gg = trmfac * 127.5F; // the greener, the higher the relevance
  bb = agm;  // in blue is the AGM

  // place the agm into our result image:
  Image<PixRGB<byte> > cbtmp = rescale(makeRGB(rr, gg, bb), XWW/2, XWH/2);
  inplaceEmbed(xwi2, cbtmp, r, PixRGB<byte>(192, 192, 192), true);
  writeText(xwi2, Point2D<int>(0,0), " Attention Map ");
  inplacePaste(xwin, xwi2, Point2D<int>(XWW/2, 0));

  // now do the conspicuity maps:
  r = Rectangle::tlbrI(0, 0, XWH/4 - 1, XWW/4 - 1);
  xwi2.resize(XWW/4, XWH/4);

  LFATAL("FIXME I should derive from SimulationViewerStd");
  /*
  const nub::ref<VisualCortex> vc = itsBrain->getVC();

  if (vc->hasSubChan("color")) {
    Image<float> ftmp = vc->subChan("color")->getOutput();
    ftmp = rescale(ftmp, XWW / 4, XWH / 4);
    inplaceNormalize(ftmp, 0.0f, 255.0f);
    Image<byte> btmp = ftmp; writeText(btmp, Point2D<int>(0,0), " Color ");
    inplaceEmbed(xwi2, Image<PixRGB<byte> >(btmp), r, PixRGB<byte>(192), true);
    drawRect(xwi2, r, PixRGB<byte>(255), 1);
    inplacePaste(xwin, xwi2, Point2D<int>(0, XWH/2));
  }

  if (vc->hasSubChan("intensity")) {
    Image<float> ftmp = vc->subChan("intensity")->getOutput();
    ftmp = rescale(ftmp, XWW/4, XWH/4);
    inplaceNormalize(ftmp, 0.0f, 255.0f);
    Image<byte> btmp = ftmp; writeText(btmp, Point2D<int>(0,0), " Intensity ");
    inplaceEmbed(xwi2, Image<PixRGB<byte> >(btmp), r, PixRGB<byte>(192), true);
    drawRect(xwi2, r, PixRGB<byte>(255), 1);
    inplacePaste(xwin, xwi2, Point2D<int>(XWW/4, XWH/2));
  }

  if (vc->hasSubChan("orientation")) {
    Image<float> ftmp = vc->subChan("orientation")->getOutput();
    ftmp = rescale(ftmp, XWW/4, XWH/4);
    inplaceNormalize(ftmp, 0.0f, 255.0f);
    Image<byte> btmp = ftmp; writeText(btmp, Point2D<int>(0,0), " Orientation ");
    inplaceEmbed(xwi2, Image<PixRGB<byte> >(btmp), r, PixRGB<byte>(192), true);
    drawRect(xwi2, r, PixRGB<byte>(255), 1);
    inplacePaste(xwin, xwi2, Point2D<int>(XWW/2, XWH/2));
  }

  if (vc->hasSubChan("flicker")) {
    Image<float> ftmp = vc->subChan("flicker")->getOutput();
    ftmp = rescale(ftmp, XWW/4, XWH/4);
    inplaceNormalize(ftmp, 0.0f, 255.0f);
    Image<byte> btmp = ftmp; writeText(btmp, Point2D<int>(0,0), " Flicker ");
    inplaceEmbed(xwi2, Image<PixRGB<byte> >(btmp), r, PixRGB<byte>(192), true);
    drawRect(xwi2, r, PixRGB<byte>(255), 1);
    inplacePaste(xwin, xwi2, Point2D<int>(XWW/4+XWW/2, XWH/2));
  }

  if (vc->hasSubChan("motion")) {
    nub::soft_ref<MotionChannel> mc; dynCastWeakToFrom(mc, vc->subChan("motion"));
    if (mc.isInvalid()) LFATAL("Channel named 'motion' not a MotionChannel");
    if (mc->numChans() == 4) {
      Image<float> ftmp = mc->dirChan(2).getOutput();
      ftmp = rescale(ftmp, XWW/4, XWH/4);
      inplaceNormalize(ftmp, 0.0f, 255.0f);
      Image<byte> btmp = ftmp; writeText(btmp, Point2D<int>(0,0), " MotionLeft ");
      inplaceEmbed(xwi2, Image<PixRGB<byte> >(btmp), r, PixRGB<byte>(192), true);
      drawRect(xwi2, r, PixRGB<byte>(255), 1);
      inplacePaste(xwin, xwi2, Point2D<int>(0, XWH/4 + XWH/2));

      ftmp = mc->dirChan(1).getOutput();
      ftmp = rescale(ftmp, XWW/4, XWH/4);
      inplaceNormalize(ftmp, 0.0f, 255.0f);
      btmp = ftmp; writeText(btmp, Point2D<int>(0,0), " MotionDown ");
      inplaceEmbed(xwi2, Image<PixRGB<byte> >(btmp), r, PixRGB<byte>(192), true);
      drawRect(xwi2, r, PixRGB<byte>(255), 1);
      inplacePaste(xwin, xwi2, Point2D<int>(XWW/4, XWH/4 + XWH/2));

      ftmp = mc->dirChan(3).getOutput();
      ftmp = rescale(ftmp, XWW/4, XWH/4);
      inplaceNormalize(ftmp, 0.0f, 255.0f);
      btmp = ftmp; writeText(btmp, Point2D<int>(0,0), " MotionUp ");
      inplaceEmbed(xwi2, Image<PixRGB<byte> >(btmp), r, PixRGB<byte>(192), true);
      drawRect(xwi2, r, PixRGB<byte>(255), 1);
      inplacePaste(xwin, xwi2, Point2D<int>(XWW/2, XWH/4 + XWH/2));

      ftmp = mc->dirChan(0).getOutput();
      ftmp = rescale(ftmp, XWW/4, XWH/4);
      inplaceNormalize(ftmp, 0.0f, 255.0f);
      btmp = ftmp; writeText(btmp, Point2D<int>(0,0), " MotionRight ");
      inplaceEmbed(xwi2, Image<PixRGB<byte> >(btmp), r, PixRGB<byte>(192), true);
      drawRect(xwi2, r, PixRGB<byte>(255), 1);
      inplacePaste(xwin, xwi2, Point2D<int>(XWW/2 + XWW/4, XWH/4 + XWH/2));
    }
  }
  */
  return xwin;
}

// ######################################################################
void SimulationViewerNerdCam::drawMaskOutline(Image< PixRGB<byte> >& traj,
                                              const Image<byte> mask,
                                              const PixRGB<byte>& col,
                                              const int thick,
                                              const Point2D<int>& pos,
                                              const int radius) const
{
  if (traj.initialized() == false) return; // can't draw...

  // object-shaped drawing
  Image<byte> om(mask);
  inplaceLowThresh(om, byte(128)); // cut off fuzzy (interpolated) object boundaries
  om = contour2D(om);       // compute binary contour image
  const int w = traj.getWidth();
  const int h = traj.getHeight();
  Point2D<int> ppp;
  for (ppp.j = 0; ppp.j < h; ppp.j ++)
    for (ppp.i = 0; ppp.i < w; ppp.i ++)
      if (om.getVal(ppp.i, ppp.j))  // got a contour point -> draw here
        drawDisk(traj, ppp, thick, col);  // small disk for each point
  // OBSOLETE: drawCircle(traj, pos, radius, col, thick);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
