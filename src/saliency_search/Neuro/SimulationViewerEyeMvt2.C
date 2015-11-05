/*!@file Neuro/SimulationViewerEyeMvt2.C comparison between saliency and
  human eye movements */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeMvt2.C $
// $Id: SimulationViewerEyeMvt2.C 14376 2011-01-11 02:44:34Z pez $
//

#include "Neuro/SimulationViewerEyeMvt2.H"

#include "Channels/ChannelBase.H"
#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"    // for contrastModulate()
#include "Image/CutPaste.H"    // for concatX()
#include "Image/DrawOps.H"     // for colGreyCombo()
#include "Image/MathOps.H"     // for takeMax()
#include "Image/ShapeOps.H"    // for rescale()
#include "Image/Transforms.H"  // for segmentObjectClean(), contour2D()
#include "Neuro/AttentionGuidanceMap.H"
#include "Neuro/Brain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/Retina.H"
#include "Neuro/SaccadeControllers.H"
#include "Neuro/EyeHeadController.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/TaskRelevanceMap.H"
#include "Neuro/VisualBuffer.H"
#include "Neuro/VisualCortex.H"
#include "Psycho/EyeData.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"

#include <stdio.h>

// ######################################################################
SimulationViewerEyeMvt2::
SimulationViewerEyeMvt2(OptionManager& mgr,
                        const std::string& descrName,
                        const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventWTAwinner),
  SIMCALLBACK_INIT(SimEventSaccadeStatusEye),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsMetrics(new SpatialMetrics(mgr)),
  itsSaveMegaCombo(&OPT_SVEMsaveMegaCombo, this),
  itsColorEye("SVcolorHumanEye", this, PixRGB<byte>(128, 255, 255)),
  itsOutFname(&OPT_SVEMoutFname, this),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsUseIOR(&OPT_SVEMuseIOR, this),
  itsTraj(), itsRawTraj(),
  itsOutFile(NULL), itsLastSample(-1, -1), itsLastRadius(0),
  itsIORmask(),
  itsDidAttentionShift(false)
{
  this->addSubComponent(itsMetrics);

  LINFO("NOTE: Selecting EyeHeadController of type EyeTrack");
  getManager().setOptionValString(&OPT_EyeHeadControllerType, "EyeTrack");
}

// ######################################################################
SimulationViewerEyeMvt2::~SimulationViewerEyeMvt2()
{ }

// ######################################################################
void SimulationViewerEyeMvt2::start1()
{
  // abort if no output file:
  if (itsOutFname.getVal().empty()) LFATAL("I need an output file!");

  // open output file:
  itsOutFile = fopen(itsOutFname.getVal().c_str(), "w");
  if (itsOutFile == NULL) PLFATAL("Cannot write '%s'", itsOutFname.getVal().c_str());

  SimulationViewer::start1();
}

// ######################################################################
void SimulationViewerEyeMvt2::stop1()
{
  if (itsOutFile) { fclose(itsOutFile); itsOutFile = NULL; }

  SimulationViewer::stop1();
}

// ######################################################################
void SimulationViewerEyeMvt2::
onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  // reset our drawings:
  itsRawTraj = e->frame().asRgb();
}


// ######################################################################
void SimulationViewerEyeMvt2::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // Reset our drawings
  itsTraj = e->frame().colorByte();
}

// ######################################################################
void SimulationViewerEyeMvt2::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  const WTAwinner& win = e->winner();

  // do some drawings:
  drawCircle(itsTraj, win.p, itsMetrics->getFOAradius(), PixRGB<byte>(0, 127 + (win.boring?0:128), 0), 2);
  itsDidAttentionShift = true;
}

// ######################################################################
void SimulationViewerEyeMvt2::
onSimEventSaccadeStatusEye(SimEventQueue& q, rutz::shared_ptr<SimEventSaccadeStatusEye>& e)
{
  // grab the latest visual buffer:
  if (SeC<SimEventVisualBufferOutput> ebuf = q.check<SimEventVisualBufferOutput>(this, SEQ_ANY)) {
    Image<float> buf = ebuf->buffer();
    const int maplevel = ebuf->smlev();

    // also grab the latest retina output:
    if (SeC<SimEventRetinaImage> eret = q.check<SimEventRetinaImage>(this, SEQ_ANY)) {
      const Dims& dims = buf.getDims();
      const Point2D<int>& p = e->position();

      // select a drawing color & size:
      PixRGB<byte> col(itsColorEye.getVal()); int psiz = 5;
      if (transientStatusIsOn(e->saccadeStatus())) { col.setGreen(0); col.setRed(255); }
      if (transientStatusIsOn(e->blinkStatus())) { col.setBlue(0); }
      PixRGB<byte> bk(0, 0, 0);
      drawPatch(itsTraj, p, psiz + 2, bk);
      drawPatch(itsTraj, p, psiz, col);
      Point2D<int> rawp = eret->retinalToRaw(p);
      drawPatch(itsRawTraj, rawp, psiz + 2, bk);
      drawPatch(itsRawTraj, rawp, psiz, col);

      // get the latest eye movement data:
      Point2D<int> nextTarg; double ampl = 0.0;
      if (SeC<SimEventEyeTrackerData> etrac = q.check<SimEventEyeTrackerData>(this, SEQ_ANY)) {
        rutz::shared_ptr<EyeData> data = etrac->data();
        if (data->hasSaccadeTargetData()) {
          nextTarg = data->saccadeTarget();
          ampl = data->saccadeAmplitude();
          data->saccadeDuration();
        } else return;
      } else return;

      // get buffer-centered coords of our current eye position:
      Point2D<int> curr = ebuf->retinalToBuffer(p);

      // are we starting a saccade?
      if (ampl > 0.0)
        {
          // the coords we have from the SC are retinotopic; transform
          // into buffer-centered:
          itsLastSample = ebuf->retinalToBuffer(nextTarg);
          itsLastRadius = (itsMetrics->getFoveaRadius() + (1 << maplevel) - 1) >> maplevel;

          // check that saccade target is okay:
          if (buf.coordsOk(itsLastSample) == false)
            {
              LERROR("Hum, saccade target at (%d, %d)? -- CLAMPING", itsLastSample.i, itsLastSample.j);
              itsLastSample.clampToDims(dims);
            }
          float mi, ma, av; getMinMaxAvg(buf, mi, ma, av);

          // mask the buffer by a disk of radius FOAradius+stdev around
          // the saccade target location and compute the max inside:
          Image<float> fov(dims, ZEROS);
          drawCircle(itsTraj, nextTarg, itsMetrics->getFoveaRadius(), PixRGB<byte>(255, 255, 0), 2);
          float salience = getLocalMax(buf, itsLastSample, itsLastRadius);

          if (itsUseIOR.getVal())
            {
              // the human may or may not have turned on IOR, we don't really
              // know. Here let's just estimate what would happen if we
              // trigger IOR at our current location. If it improves, we'll
              // assume it was done. Improvement means that our measured
              // salience at the target location does not change, but the
              // average (and possibly max) over the entire image will go down
              // since we have eliminated our potentially very salient current
              // location. We first cut off anything less salient than 1/4 the
              // max salience, to make sure the segmentation will not spread
              // too far.
              Image<float> buf2(buf);
              inplaceLowThresh(buf2, ma * 0.25F, 0.0F);
              if (buf2.getVal(curr) > ma * 0.05F)
                {
                  itsIORmask = segmentObjectClean(buf2, curr);
                  Image<float> iormaskf(itsIORmask);
                  inplaceNormalize(iormaskf, 0.0F, 1.0F);
                  buf2 = buf * binaryReverse(iormaskf, 1.0F);
                  float iorsal = getLocalMax(buf2, itsLastSample, itsLastRadius);

                  if (fabs(iorsal - salience) < ma * 1.0e-4F)
                    {
                      LINFO("### Triggering IOR in Visual Buffer ###");
                      /*
                      // update min/max/avg/salience computation
                      itsBuffer->inhibit(iormaskf);
                      buf = itsBuffer->getBuffer();
                      getMinMaxAvg(buf, mi, ma, av);
                      salience = getLocalMax(buf, itsLastSample, itsLastRadius);
                      */
                    }
                }
            }

          // get salience we would obtain with a random
          // saccade of random amplitude:
          Point2D<int> p(randomUpToNotIncluding(dims.w()), randomUpToNotIncluding(dims.h()));
          drawCircle(itsTraj, ebuf->bufferToRetinal(p), itsMetrics->getFoveaRadius(),
                     PixRGB<byte>(255, 255, 255), 2);
          /*
         float rndsal2 = getLocalMax(buf, p, itsLastRadius);
          float rnddist = p.distance(itsLastSample) * (1<<maplevel);

          // get location that our buffer would predict is the best target
          // for a saccade right now:

          p = itsBuffer->findMostInterestingTargetLocMax(curr);
          drawPatch(itsTraj, ebuf->bufferToRetinal(p), psiz + 2, bk);
          drawPatch(itsTraj, ebuf->bufferToRetinal(p), psiz, PixRGB<byte>(0, 0, 255));
          float dist = p.distance(itsLastSample) * (1<<maplevel);

          // save vbuf contents at eye:
          fprintf(itsOutFile, "%g %g %g %g  %g %g %g  %g %g\n",
                  salience, mi, ma, av, ampl, durs, rndsal2, dist, rnddist);
          fflush(itsOutFile);
          */
        }
    }
  }
}

// ######################################################################
Image< PixRGB<byte> > SimulationViewerEyeMvt2::getTraj(SimEventQueue& q)
{
  /*
  PixRGB<byte> bgcol(64, 128, 0); // background color
  PixRGB<byte> gridcol(255, 255, 255); // grid color

  Dims tdims = itsTraj.getDims();
  int maplevel = itsLevelSpec.getVal().mapLevel();
  Image<float> sm = getMap(q);
  Image< PixRGB<byte> > smc = toRGB(Image<byte>(rescaleOpt(sm, tdims, itsDisplayInterp.getVal())));

  // get the saliency mask (will show a blob depicting the area that
  // gets transferred from saliency map to buffer at each attention
  // shift):
  Image<byte> mask;
  if (itsBuffer->isObjectBased() && itsDidAttentionShift)
    mask = rescaleOpt(itsBuffer->getSaliencyMask(), tdims, itsDisplayInterp.getVal());
  else
    mask.resize(tdims, true);
  itsDidAttentionShift = false;

  // get the buffer:
  Image<float> buf = itsBuffer->getBuffer();
  inplaceNormalize(buf, 0.0F, 255.0F);
  Dims bdims(buf.getWidth() << maplevel, buf.getHeight() << maplevel);

  // get a color, full-scale version of the buffer:
  Image< PixRGB<byte> > bufc = toRGB(Image<byte>(rescaleOpt(buf, bdims, itsDisplayInterp.getVal())));

  // draw circle around saliency sample if we just took one:
  if (itsLastRadius != 0)
    {
      Point2D<int> p(itsLastSample); p.i <<= maplevel; p.j <<= maplevel;
      int r(itsLastRadius); r <<= maplevel;
      drawCircle(bufc, p, r, PixRGB<byte>(255, 255, 0), 2);
      drawCircle(smc, itsBuffer->bufferToRetinal(p), r, PixRGB<byte>(255, 255, 0), 2);
      itsLastRadius = 0;
    }

  // draw IOR contours if any:
  if (itsIORmask.initialized())
    {
      Image<byte> contou = contour2D(rescale(itsIORmask, bdims));

      Point2D<int> ppp;
      for (ppp.j = 0; ppp.j < bdims.h(); ppp.j ++)
        for (ppp.i = 0; ppp.i < bdims.w(); ppp.i ++)
          if (contou.getVal(ppp.i, ppp.j))
            drawDisk(bufc, ppp, 2, PixRGB<byte>(255, 255, 0));
      itsIORmask.freeMem();
    }

  // return mega combo; we have 4 panels of the dims of the blown-up
  // buffer (bdims), and we will paste images of various dims in those
  // panels:
  Image< PixRGB<byte> > ret;

  // start with the unshifted raw input:
  Image< PixRGB<byte> > rawinp(bdims, NO_INIT); rawinp.clear(bgcol);
  Point2D<int> rawinpoff((rawinp.getWidth()-itsRawTraj.getWidth())/2, (rawinp.getHeight()-itsRawTraj.getHeight())/2);
  inplacePaste(rawinp, itsRawTraj, rawinpoff);

  // now the foveal/shifted image:
  Image< PixRGB<byte> > rtraj(bdims, NO_INIT); rtraj.clear(bgcol);
  Point2D<int> rtrajoff((rtraj.getWidth() - itsTraj.getWidth())/2, (rtraj.getHeight() - itsTraj.getHeight())/2);
  inplacePaste(rtraj, itsTraj, rtrajoff);

  // now the saliency map:
  Image< PixRGB<byte> > smc2(bdims, NO_INIT); smc2.clear(bgcol);
  inplacePaste(smc2, smc, rtrajoff);

  // ready for action:
  ret = concatY(concatX(rawinp, rtraj), concatX(bufc, smc2));

  // draw separating borders:
  drawLine(ret, Point2D<int>(0, bdims.h()), Point2D<int>(bdims.w()*2-1, bdims.h()), gridcol, 1);
  drawLine(ret, Point2D<int>(bdims.w(), 0), Point2D<int>(bdims.w(), bdims.h()*2-1), gridcol, 1);

  return ret;

  */
  return Image< PixRGB<byte> >();
}

// ######################################################################
void SimulationViewerEyeMvt2::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  // update the trajectory:
  Image< PixRGB<byte> > res = getTraj(q);

  // save results?
  if (itsSaveMegaCombo.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

      ofs->writeRGB(res, "T", FrameInfo("SimulationViewerEyeMvt2 trajectory", SRC_POS));
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
