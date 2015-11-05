/*!@file Neuro/SimulationViewerStd.C visualize various model simulations */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerStd.C $
// $Id: SimulationViewerStd.C 14503 2011-02-14 00:47:49Z dberg $
//

#include "Neuro/SimulationViewerStd.H"

#include "Channels/ChannelMaps.H"
#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"    // for normalizeC()
#include "Image/DrawOps.H"
#include "Image/MathOps.H"   // for takeMax()
#include "Image/PyramidOps.H" // for foveate()
#include "Image/ShapeOps.H"  // for crop()
#include "Image/Transforms.H"  // for contour2D()
#include "Neuro/AttentionGuidanceMap.H"
#include "Neuro/Brain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/TaskRelevanceMap.H"
#include "Neuro/VisualCortex.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimulationOpts.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/StringUtil.H"
#include "Util/TransientStatus.H"
#include "Util/sformat.H"

// ######################################################################
SimulationViewerStd::SimulationViewerStd(OptionManager& mgr,
                                         const std::string& descrName,
                                         const std::string& tagName) :
  SimulationViewerAdapter(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventSaccadeStatusEye),
  SIMCALLBACK_INIT(SimEventSaccadeStatusHead),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  SIMCALLBACK_INIT(SimEventITOutput),
  itsTimeStep(&OPT_SimulationTimeStep, this),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsMetrics(new SpatialMetrics(mgr)),
  itsFontSize(&OPT_SVfontSize, this),
  itsSaveTraj(&OPT_SVsaveTraj, this),
  itsSaveXcombo(&OPT_SVsaveXcombo, this),
  itsSaveYcombo(&OPT_SVsaveYcombo, this),
  itsSaveTRMXcombo(&OPT_SVsaveTRMXcombo, this),
  itsSaveTRMYcombo(&OPT_SVsaveTRMYcombo, this),
  itsSaveTRMmegaCombo(&OPT_SVsaveTRMmegaCombo, this),
  itsWarp3D(&OPT_SVwarp3D, this),
  itsMegaCombo(&OPT_SVmegaCombo, this),
  itsMegaComboZoom(&OPT_SVmegaComboZoom, this),
  itsMegaComboTopCMapsOnly(&OPT_SVmegaComboTopCMapsOnly, this),
  itsCropFOA(&OPT_SVcropFOA, this),
  itsFoveateTraj(&OPT_SVfoveateTraj, this),
  itsDisplayFOA(&OPT_SVdisplayFOA, this),
  itsDisplayPatch(&OPT_SVdisplayPatch, this),
  itsDisplayFOAnum(&OPT_SVdisplayFOAnum, this),
  itsDisplayFOALinks(&OPT_SVdisplayFOALinks, this),
  itsDisplayEye(&OPT_SVdisplayEye, this),
  itsDisplayEyeLinks(&OPT_SVdisplayEyeLinks, this),
  itsDisplayHead(&OPT_SVdisplayHead, this),
  itsDisplayHeadLinks(&OPT_SVdisplayHeadLinks, this),
  itsDisplayTime(&OPT_SVdisplayTime, this),
  itsDisplayAdditive(&OPT_SVdisplayAdditive, this),
  itsDisplayHighlights(&OPT_SVdisplayHighlights, this),
  itsDisplaySMmodulate(&OPT_SVdisplaySMmodulate, this),
  itsDisplayBoring(&OPT_SVdisplayBoring, this),
  itsDisplayShapeEstimator("SVdisplayShapeEstimator", this, false),
  itsColorNormal("SVcolorNormal", this, PixRGB<byte>(255, 255, 0)),
  itsColorBoring("SVcolorBoring", this, PixRGB<byte>(127, 0, 0)),
  itsColorBlink("SVcolorBlink", this, PixRGB<byte>(0, 128, 255)),
  itsColorSaccade("SVcolorSaccade", this, PixRGB<byte>(255, 128, 255)),
  itsColorSmoothPursuit("SVcolorSmoothPursuit", this,
                        PixRGB<byte>(128, 255, 255)),
  itsColorLink("SVcolorLink", this, PixRGB<byte>(255, 0, 0)),
  itsColorText("SVcolorText", this, PixRGB<byte>(192, 255, 64)),
  itsHighlightMax("SVhighlightMax", this, 320),
  itsShapeEstimatorBaseContrast("SVShapeEstimatorBaseContrast", this, 0.1F),
  itsShapeEstimatorBaseBright("SVShapeEstimatorBaseBright", this, 128),
  itsWarp3Dpitch("SVwarp3DInitialPitch", this, -25.0F),
  itsWarp3Dyaw("SVwarp3DInitialYaw", this, -15.0F),
  itsWarp3DpitchRate("SVwarp3DpitchRate", this, 0.0F),
  itsWarp3DyawRate("SVwarp3DyawRate", this, 15.0F),
  itsWarp3DpitchMax("SVwarp3DpitchMax", this, 0.0F),
  itsWarp3DyawMax("SVwarp3DyawMax", this, 20.0F),
  itsHeadRadius(&OPT_HeadMarkerRadius, this),
  itsMultiRetinaDepth(&OPT_MultiRetinaDepth, this),
  itsFOApsiz("SVfoapsiz", this, 3),
  itsFOAthick("SVfoathick", this, 3),
  itsFOAlinkThick("SVfoalinkthick", this, 2),
  itsFOVpsiz("SVfovpsiz", this, 3),
  itsFOVpthick("SVfovpthick", this, 1),
  itsFOVthick("SVfovthick", this, 2),
  itsFOVlinkThick("SVfovlinkthick", this, 1),
  itsHEDpsiz("SVhedpsiz", this, 9),
  itsHEDpthick("SVhedpthick", this, 1),
  itsHEDthick("SVhedthick", this, 1),
  itsHEDlinkThick("SVhedlinkthick", this, 1),
  itsUseLargerDrawings(&OPT_SVuseLargerDrawings, this),
  itsObsolete1(&OPT_SVxwindow, this),
  itsTraj(),
  itsFOAshiftNum(0U),
  itsCurrFOAmask(),
  itsCumFOAmask(),
  itsCurrEye(-1, -1),
  itsCurrFoveaMask(),
  itsCumFoveaMask(),
  itsCurrHead(-1, -1),
  itsCurrHeadMask(),
  itsCumHeadMask(),
  itsPrevEye(-1, -1),
  itsPrevHead(-1, -1),
  itsCurrTime(),
  itsEyeSaccade(false),
  itsEyeSmoothPursuit(false),
  itsHeadSaccade(false),
  itsHeadSmoothPursuit(false),
  itsEyeBlink(false),
  itsMultiTraj(),
  itsDims3D(),
  itsPitch3D(-1.0e10F),
  itsYaw3D(-1.0e10F),
  itsHasNewInput(false),
  itsFont(SimpleFont::FIXED(10))
{
  this->addSubComponent(itsMetrics);
}

// ######################################################################
SimulationViewerStd::~SimulationViewerStd()
{ }

// ######################################################################
void SimulationViewerStd::reset1()
{
  // reset some stuff for SimulationViewerStd
  itsTraj.freeMem();
  itsFOAshiftNum = 0U;
  itsCurrFOAmask.freeMem();
  itsCumFOAmask.freeMem();
  itsCurrEye = Point2D<int>(-1, -1);
  itsCurrFoveaMask.freeMem();
  itsCumFoveaMask.freeMem();
  itsCurrHead = Point2D<int>(-1, -1);
  itsCurrHeadMask.freeMem();
  itsCumHeadMask.freeMem();
  itsPrevEye = Point2D<int>(-1, -1);
  itsPrevHead = Point2D<int>(-1, -1);
  itsCurrTime = SimTime::ZERO();
  itsEyeSaccade = false;
  itsEyeSmoothPursuit = false;
  itsHeadSaccade = false;
  itsHeadSmoothPursuit = false;
  itsEyeBlink = false;
  itsMultiTraj.reset();
  itsDims3D = Dims();
  itsPitch3D = -1.0e10F;
  itsYaw3D = -1.0e10F;
  itsHasNewInput = false;

  // propagate to our base class:
  SimulationViewerAdapter::reset1();

}

// ######################################################################
void SimulationViewerStd::start2()
{
  // do we wnat larger drawings?
  if (itsUseLargerDrawings.getVal())
    {
      itsFOApsiz.setVal(5);
      itsFOVpsiz.setVal(5);
      itsFOVpthick.setVal(2);
      itsHEDpsiz.setVal(12);
      itsHEDpthick.setVal(2);
    }

  itsFont = SimpleFont::fixedMaxWidth(itsFontSize.getVal());
}

// ######################################################################
void SimulationViewerStd::
doEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  //itsInput set by base class SimulationViewerAdapter

  // erase old trajectory:
  itsTraj.resize(itsInput.getDims(), true);

  // if foveating the traj, get a foveation pyramid ready:
  if (itsFoveateTraj.getVal())
    itsMultiTraj = buildPyrGaussian(itsInput, 0, itsMultiRetinaDepth.getVal(), 9);

  // we have a new input; this will force redrawing various things on
  // the trajectory in case people request it before a new shift of
  // attention or other event occurrs:
  itsHasNewInput = true;
}

// ######################################################################
void SimulationViewerStd::
doEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  // Any output from the ShapeEstimator?
  Image<byte> foaMask;
  if (SeC<SimEventShapeEstimatorOutput> ee = q.check<SimEventShapeEstimatorOutput>(this))
    {
      foaMask = Image<byte>(ee->smoothMask() * 255.0F);
      foaMask = inverseRetinal(foaMask);//will inverse map if necessary
      if (foaMask.isSameSize(itsInput) == false)
        LFATAL("Dimensions of FOAmask must match those of input");
    }
  
  // update our FOA shift number
  itsFOAshiftNum = e->shiftNum();
  
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
void SimulationViewerStd::
onSimEventSaccadeStatusEye(SimEventQueue& q, rutz::shared_ptr<SimEventSaccadeStatusEye>& e)
{
  // update our saccade/blink/etc status:
  itsEyeSaccade = transientStatusIsOn(e->saccadeStatus());
  itsEyeSmoothPursuit = transientStatusIsOn(e->smoothPursuitStatus());
  itsEyeBlink = transientStatusIsOn(e->blinkStatus());

  // do nothing else if eye did not move
  if (e->position() == itsCurrEye) return;

  // update our internals:
  itsPrevEye = itsCurrEye; itsCurrEye = e->position();

  // draw a disk at current eye position:
  itsCurrFoveaMask.resize(itsInput.getDims(), true);
  if (itsCurrEye.isValid())
    drawDisk(itsCurrFoveaMask, itsCurrEye,
             itsMetrics->getFoveaRadius(), byte(255));

  // update cumulative fovea mask:
  if (itsDisplayAdditive.getVal() &&
      itsCumFoveaMask.initialized() &&
      itsCumFoveaMask.isSameSize(itsCurrFoveaMask))
    itsCumFoveaMask = takeMax(itsCumFoveaMask, itsCurrFoveaMask);
  else
    itsCumFoveaMask = itsCurrFoveaMask;

  // forget it if we don't have a traj yet:
  if (itsTraj.initialized() == false) return;

  // draw the eye position:
  if (itsDisplayEye.getVal()) drawEye();

  // link the last two eye positions:
  if (itsDisplayEyeLinks.getVal()) linkEyes();
}

// ######################################################################
void SimulationViewerStd::
onSimEventSaccadeStatusHead(SimEventQueue& q, rutz::shared_ptr<SimEventSaccadeStatusHead>& e)
{
  // update our saccade/blink/etc status:
  itsHeadSaccade = transientStatusIsOn(e->saccadeStatus());
  itsHeadSmoothPursuit = transientStatusIsOn(e->smoothPursuitStatus());

  // do nothing else if head did not move
  if (e->position() == itsCurrHead) return;

  // update our internals:
  itsPrevHead = itsCurrHead; itsCurrHead = e->position();

  // draw a disk at current head position:
  itsCurrHeadMask.resize(itsInput.getDims(), true);
  if (itsCurrHead.isValid())
    drawDisk(itsCurrHeadMask, itsCurrHead, itsHeadRadius.getVal(), byte(255));

  // update cumulative head mask:
  if (itsDisplayAdditive.getVal() &&
      itsCumHeadMask.initialized() &&
      itsCumHeadMask.isSameSize(itsCurrHeadMask))
    itsCumHeadMask = takeMax(itsCumHeadMask, itsCurrHeadMask);
  else
    itsCumHeadMask = itsCurrHeadMask;

  // forget it if we don't have a traj yet:
  if (itsTraj.initialized() == false) return;

  // draw the head position:
  if (itsDisplayHead.getVal()) drawHead();

  // link the last two head positions:
  if (itsDisplayHeadLinks.getVal()) linkHeads();
}

// ######################################################################
void SimulationViewerStd::
onSimEventITOutput(SimEventQueue& q, rutz::shared_ptr<SimEventITOutput>& e)
{
  LINFO("The match found is: %s", e->getObjData()->name.c_str());
// forget it if we don't have a traj yet:
  if (itsTraj.initialized() == false) return;
  else
    {
     drawOutlinedPolygon(itsTraj, e->getObjData()->polygon, PixRGB<byte>(255,128,128), Point2D<int>(0, 0), 0 , 1, 0, 0, 5 );
     Point2D<int> postxtobj (e->getObjData()->polygon[3].i, e->getObjData()->polygon[3].j+1);
     writeText(itsTraj, postxtobj, e->getObjData()->name.c_str(), PixRGB<byte>(1,1,1),
               PixRGB<byte>(255,255,255), SimpleFont::FIXED(10), false, ANCHOR_BOTTOM_LEFT);
    }
 }

// ######################################################################
Image< PixRGB<byte> > SimulationViewerStd::getTraj(SimEventQueue& q)
{
  // update our 3D rotations if necessary:
  if (itsWarp3D.getVal()) {
    const SimTime t = q.now();
    const SimTime dt = SimTime::computeDeltaT((t - itsCurrTime), itsTimeStep.getVal());
    const double dts = dt.secs();

    for (SimTime tt = itsCurrTime; tt < t; tt += dt) {
      if (itsPitch3D == -1.0e10F) itsPitch3D = itsWarp3Dpitch.getVal();
      if (itsYaw3D == -1.0e10F) itsYaw3D = itsWarp3Dyaw.getVal();

      itsPitch3D += itsWarp3DpitchRate.getVal() * dts;
      itsYaw3D += itsWarp3DyawRate.getVal() * dts;

      if (itsYaw3D >= itsWarp3DyawMax.getVal() || itsYaw3D <= -itsWarp3DyawMax.getVal())
        itsWarp3DyawRate.setVal(- itsWarp3DyawRate.getVal());
      if (itsPitch3D >= itsWarp3DpitchMax.getVal() || itsPitch3D <= -itsWarp3DpitchMax.getVal())
        itsWarp3DpitchRate.setVal(- itsWarp3DpitchRate.getVal());
    }
  }

  itsCurrTime = q.now();
  if (itsTraj.initialized() == false) return itsTraj;

  // ##### if not doing additive displays, clear traj and redraw only
  // ##### current foa/eye/head data:
  if (itsDisplayAdditive.getVal() == false)
    itsTraj.resize(itsInput.getDims(), true);

  // ##### What do we want to use as backdrop image under our
  // ##### trajectory drawings? Is it just the original input image,
  // ##### or something fancier?
  Image< PixRGB<byte> > ret;

  // show ShapeEstimator masking (at the location(s) of covert attention):
  if (itsDisplayShapeEstimator.getVal())
    ret = contrastModulate(itsInput, itsCumFOAmask,
                           itsShapeEstimatorBaseContrast.getVal(),
                           itsShapeEstimatorBaseBright.getVal());

  // display FOA highlights using mask of eye positions:
  if (itsDisplayHighlights.getVal() && itsCumFoveaMask.initialized())
    ret = highlightRegions(itsInput, itsCumFoveaMask,
                           itsHighlightMax.getVal());

  // modulate contrast by saliency map:
  if (itsDisplaySMmodulate.getVal())
    {
      const float fac = 0.001F;
      Image<float> sm = rescaleOpt(getMap(q) * fac,
                                itsTraj.getDims(),
                                itsDisplayInterp.getVal());
      float mi, ma; getMinMax(sm, mi, ma);
      LINFO("SM-based contrast modulation range: [%.3f .. %.3f]", mi, ma);

      ret = contrastModulate(itsInput, sm, 0.5F, byte(127));
    }

  // show foveated masking, at the location of overt attention:
  if (itsFoveateTraj.getVal())
    ret = foveate(itsCumFoveaMask, itsMultiTraj);

  // ##### at this point, if we have something in ret, then let's use
  // ##### that as backdrop image under our drawings, otherwise let's
  // ##### just use the original input:
  if (ret.initialized() == false) ret = itsInput;

  // ##### Update our trajectory drawings. We only care about current
  // ##### attention/eye/head position and not about any previous ones
  // ##### or links:
  if (itsHasNewInput)
    {
      if (itsDisplayFOA.getVal() || itsDisplayPatch.getVal() ||
          itsDisplayFOAnum.getVal()) drawFOA();
      if (itsDisplayEye.getVal()) drawEye();
      if (itsDisplayHead.getVal()) drawHead();
      itsHasNewInput = false;
    }

  // ##### now slap our FOA, FOA links, etc drawings on top of the backdrop:
  ret = composite(itsTraj, ret, PixRGB<byte>(0, 0, 0)); // black is transparent

  // if we want a megacombo, let's return it now:
  if (itsMegaCombo.getVal()) ret = drawMegaCombo(q, ret);

  // if cropping let's crop now:
  // resizing and other normalizations are taken care of by FrameOstream
  else if (itsCropFOA.getVal().isNonEmpty())
    {
      Dims crop_dims = itsCropFOA.getVal();
      Rectangle crect =
        Rectangle::tlbrI(itsCurrFOA.p.j - crop_dims.h() / 2,
                         itsCurrFOA.p.i - crop_dims.w() / 2,
                         itsCurrFOA.p.j + crop_dims.h() / 2 - 1,
                         itsCurrFOA.p.i + crop_dims.w() / 2 - 1);
      ret = crop(ret, crect, true);
    }

  // return a 3D traj warped onto saliency map?
  else if (itsWarp3D.getVal()) {
    drawGrid(ret, ret.getWidth() / 6, ret.getHeight() / 6, 3, 3,
             itsColorNormal.getVal());
    drawRect(ret, Rectangle::tlbrI(1, 1, ret.getHeight()-2, ret.getWidth()-2),
             itsColorNormal.getVal(), 3);
    ret = warp3Dmap(ret, getMap(q), itsPitch3D, itsYaw3D, itsDims3D);
  }

  // display combo traj + salmap?
  else if ((itsSaveXcombo.getVal() || itsSaveYcombo.getVal()))
    ret = colGreyCombo(ret, getMap(q),
                       itsSaveXcombo.getVal(), itsDisplayInterp.getVal());

  // display combo traj + salmap + highlighted trm?
  else if ((itsSaveTRMXcombo.getVal() || itsSaveTRMYcombo.getVal()))
    {
      ret = colGreyCombo(ret, getMap(q),
                         itsSaveTRMXcombo.getVal(), itsDisplayInterp.getVal());

      Image<float> trm;
      if (SeC<SimEventTaskRelevanceMapOutput> e =
          q.check<SimEventTaskRelevanceMapOutput>(this, SEQ_ANY))
        trm = e->trm(1.0F);
      else LFATAL("Cannot find a TRM!");
      //inplaceNormalize(trm, 0.0f, 255.0f);

      trm = inverseMap(trm);//pu map back in cartesian coords if necessary
      Image<PixRGB<float> > highlight_trm = toRGB(trm);
      highlight_trm = rescaleOpt(trm, itsInput.getDims(), itsDisplayInterp.getVal());
      // highlight_trm = (Image<PixRGB<float> >)itsInput *
      // rescaleOpt(trm, itsInput.getDims(), itsDisplayInterp.getVal());
      ret = colColCombo(ret, (Image<PixRGB<byte> >)highlight_trm,
                        itsSaveTRMXcombo.getVal(), itsDisplayInterp.getVal());
    }

  // display combo traj + salmap + trm + agm?
  else if (itsSaveTRMmegaCombo.getVal())
    {
      Image<float> sm;
      if (SeC<SimEventSaliencyMapOutput> e =
          q.check<SimEventSaliencyMapOutput>(this, SEQ_ANY))
        sm = e->sm(0.0F);
      else LFATAL("Cannot find a SM!");
      sm = inverseMap(sm);//pu map back in cartesian coords if necessary
      Image<PixRGB<float> > highlight_sm = toRGB(sm);
      highlight_sm = rescaleOpt(highlight_sm,
                                 itsInput.getDims(), itsDisplayInterp.getVal());
      Image<PixRGB<float> > xcombo1 = colColCombo(ret, highlight_sm,
                                                  1, itsDisplayInterp.getVal());

      Image<float> trm;
      if (SeC<SimEventTaskRelevanceMapOutput> e =
          q.check<SimEventTaskRelevanceMapOutput>(this, SEQ_ANY))
        trm = e->trm(1.0F);
      else LFATAL("Cannot find a TRM!");
      trm = inverseMap(trm);//pu map back in cartesian coords if necessary
      Image<PixRGB<float> > highlight_trm = toRGB(trm);
      highlight_trm = rescaleOpt(highlight_trm,
                                  itsInput.getDims(), itsDisplayInterp.getVal());

      Image<float> agm;
      if (SeC<SimEventAttentionGuidanceMapOutput> e =
          q.check<SimEventAttentionGuidanceMapOutput>(this, SEQ_ANY))
        agm = e->agm(0.0F);
      else LFATAL("Cannot find a AGM!");
      agm = inverseMap(agm);//pu map back in cartesian coords if necessary
      Image<PixRGB<float> > highlight_agm = toRGB(agm);
      highlight_agm = rescaleOpt(highlight_agm,
                                 itsInput.getDims(), itsDisplayInterp.getVal());

      Image<PixRGB<float> > xcombo2 = colColCombo(highlight_trm, highlight_agm,
                                         1, itsDisplayInterp.getVal());
      ret = colColCombo(xcombo1, xcombo2, 0, itsDisplayInterp.getVal());
    }

  // ##### do a bunch of last-minute drawings:
  if (itsDisplayTime.getVal()) drawTime(ret);

  // ##### return final transformed traj:
  return ret;
}

// ######################################################################
void SimulationViewerStd::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void SimulationViewerStd::save1(const ModelComponentSaveInfo& sinfo)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs =
    dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

  // also get the SimEventQueue:
  SimEventQueue *q = dynamic_cast<const SimModuleSaveInfo&>(sinfo).q;

  // update the trajectory:
  Image< PixRGB<byte> > res = getTraj(*q);

  // save results?
  if (itsSaveTraj.getVal() || itsSaveXcombo.getVal() ||
      itsSaveYcombo.getVal() || itsSaveTRMXcombo.getVal() ||
      itsSaveTRMYcombo.getVal() || itsSaveTRMmegaCombo.getVal() ||
      itsWarp3D.getVal() || itsMegaCombo.getVal())
    {
      ofs->writeRGB(res, "T",
                    FrameInfo("SimulationViewerStd trajectory", SRC_POS));
    }
}

// ######################################################################
void SimulationViewerStd::drawFOA()
{
  if (itsCurrFOA.isValid() == false) return;

  // select a drawing color:
  PixRGB<byte> col(itsColorNormal.getVal());
  if (itsCurrFOA.boring) col -= itsColorBoring.getVal();

  // display focus of attention, possibly object-shaped:
  if (itsDisplayFOA.getVal())
    drawMaskOutline(itsTraj, itsCurrFOAmask, col, itsFOAthick.getVal(), itsCurrFOA.p, itsMetrics->getFOAradius());

  // display attention shift number:
  if (itsDisplayFOAnum.getVal())
    {
      const PixRGB<byte> col2(itsColorText.getVal());
      const int rad = 2 + int(itsMetrics->getFOAradius() / M_SQRT2 + 0.5);
      const std::string txt = sformat("%d", itsFOAshiftNum);
      Point2D<int> pp = itsCurrFOA.p + Point2D<int>(rad + 3 + itsFOAthick.getVal(), rad + 3 + itsFOAthick.getVal());
      drawLine(itsTraj, itsCurrFOA.p, pp, col2);
      drawLine(itsTraj, pp, pp + Point2D<int>(txt.size() * itsFont.w() + 2, 0), col2);

      writeText(itsTraj, pp + Point2D<int>(1, 2), txt.c_str(), col2, PixRGB<byte>(0), itsFont, true);
    }

  // draw patch at current eye position:
  if (itsDisplayPatch.getVal()) drawPatch(itsTraj, itsCurrFOA.p, itsFOApsiz.getVal(), col);
}

// ######################################################################
void SimulationViewerStd::linkFOAs()
{
  if (itsCurrFOA.isValid() == false) return;

  PixRGB<byte> col = itsColorLink.getVal();
  if (itsPrevFOA.isValid())
    {
      int d = int(itsPrevFOA.p.distance(itsCurrFOA.p));
      if (d > 0) drawArrow(itsTraj, itsPrevFOA.p, itsCurrFOA.p,
                           col, itsFOAlinkThick.getVal());
    }
}

// ######################################################################
void SimulationViewerStd::drawEye()
{
  if (itsCurrEye.isValid() == false) return;

  int psiz = itsFOVpsiz.getVal();    // half-size of patch

  // select a drawing color:
  PixRGB<byte> col(itsColorNormal.getVal());
  if (itsEyeSmoothPursuit) col = itsColorSmoothPursuit.getVal();
  if (itsEyeSaccade) { col = itsColorSaccade.getVal(); psiz += 2; }
  if (itsEyeBlink) { col = itsColorBlink.getVal(); psiz += 4; }
  if (itsCurrFOA.boring) col -= itsColorBoring.getVal();

  // draw patch (thin hollow square):
  Rectangle r =
    Rectangle::tlbrI(itsCurrEye.j - psiz, itsCurrEye.i - psiz,
                    itsCurrEye.j + psiz, itsCurrEye.i + psiz);
  r = r.getOverlap(itsTraj.getBounds());
  drawRect(itsTraj, r, col, itsFOVpthick.getVal());

  // draw fovea, possibly mask-shaped:
  if (itsDisplayFOA.getVal())
    drawMaskOutline(itsTraj, itsCurrFoveaMask, col,
                    itsFOVthick.getVal(), itsCurrEye,
                    itsMetrics->getFoveaRadius());
}

// ######################################################################
void SimulationViewerStd::linkEyes()
{
  if (itsCurrEye.isValid() == false) return;

  PixRGB<byte> col = itsColorLink.getVal();

  // draw a line segment:
  if (itsPrevEye.isValid())
    drawLine(itsTraj, itsPrevEye, itsCurrEye, col,
             itsFOVlinkThick.getVal());
}

// ######################################################################
void SimulationViewerStd::drawHead()
{
  if (itsCurrHead.isValid() == false) return;

  int psiz = itsHEDpsiz.getVal();   // half-size of patch

  // select a drawing color:
  PixRGB<byte> col(itsColorNormal.getVal());
  if (itsHeadSmoothPursuit) col = itsColorSmoothPursuit.getVal();
  if (itsHeadSaccade) { col = itsColorSaccade.getVal(); psiz += 3; }
  if (itsCurrFOA.boring) col -= itsColorBoring.getVal();

  // draw square at current head position:
  Rectangle r =
    Rectangle::tlbrI(itsCurrHead.j - psiz, itsCurrHead.i - psiz,
                    itsCurrHead.j + psiz, itsCurrHead.i + psiz);
  r = r.getOverlap(itsTraj.getBounds());
  drawRect(itsTraj, r, col, itsHEDpthick.getVal());

  // draw head marker, possibly object-shaped:
  if (itsDisplayFOA.getVal())
    drawMaskOutline(itsTraj, itsCurrHeadMask, col, itsHEDthick.getVal(),
                    itsCurrHead, itsHeadRadius.getVal());
}

// ######################################################################
void SimulationViewerStd::linkHeads()
{
  if (itsCurrHead.isValid() == false) return;

  PixRGB<byte> col = itsColorLink.getVal();

  // draw a line segment:
  if (itsPrevHead.isValid())
    drawLine(itsTraj, itsPrevHead, itsCurrHead, col,
             itsHEDlinkThick.getVal());
}

// ######################################################################
void SimulationViewerStd::drawTime(Image<PixRGB<byte> >& image)
{
  const std::string txt = sformat(" %dms ", int(itsCurrTime.msecs() + 0.4999));
  writeText(image, Point2D<int>(0, 0), txt.c_str(),
            PixRGB<byte>(0), PixRGB<byte>(255), itsFont);
}

// ######################################################################
Image< PixRGB<byte> > SimulationViewerStd::drawMegaCombo(SimEventQueue& q,
                                                         const Image< PixRGB<byte> >& in)
{
  Image<byte> rr, gg, bb;
  std::string rng;
  if (itsMapType.getVal().compare("AGM") == 0)
    {
      // get the non-normalized AGM:
      Image<float> agm;
      if (SeC<SimEventAttentionGuidanceMapOutput> e = q.check<SimEventAttentionGuidanceMapOutput>(this, SEQ_ANY))
        agm = e->agm(1.0F);
      else LFATAL("Cannot find an AGM!");
      agm = inverseMap(agm);//pu map back in cartesian coords if necessary

      // create a string with the map's range of values, then normalize the map:
      float mini, maxi; getMinMax(agm, mini, maxi);
      rng = sformat("AGM=[%.03f .. %.03f]mV", mini * 1000.0F, maxi * 1000.0F);
      if (itsMapFactor.getVal() == 0.0F) inplaceNormalize(agm, 0.0F, 255.0F);
      else if (itsMapFactor.getVal() != 1.0F) agm *= itsMapFactor.getVal();

      // get the non-normalized TRM (neutral is 1.0):
      Image<float> trm;
      if (SeC<SimEventTaskRelevanceMapOutput> ee = q.check<SimEventTaskRelevanceMapOutput>(this, SEQ_ANY))
        trm = ee->trm(1.0F);
      else
        {
          LINFO("WARNING: Cannot find a TRM, assuming blank.");
          trm.resize(agm.getDims(), false); trm.clear(1.0F);
        }

      trm = inverseMap(trm);//pu map back in cartesian coords if necessary

      // create a string with the map's range of values:
      getMinMax(trm, mini, maxi);
      rng += sformat(", TRM=[%.03f .. %.03f]", mini, maxi);

      // useful range is 0.0 .. 3.0; let's show values smaller
      // than 1 as red and those larger that 1 as green:
      Image<float> trmfac(trm);
      inplaceClamp(trmfac, 0.0F, 1.0F);
      trmfac *= -1.0F; trmfac += 1.0F;
      rr = trmfac * 255.0F; // the redder, the lower the relevance
      trmfac = trm;
      inplaceClamp(trmfac, 1.0F, 3.0F);
      trmfac -= 1.0F;
      gg = trmfac * 127.5F; // the greener, the higher the relevance
      bb = agm;  // in blue is the AGM
    }
  else
    {
      // just get whatever map and display in greyscale:
      Image<float> m = getMap(q);
      rr = m; gg = m; bb = m;
    }

  // interpolate the attention map:
  Image<PixRGB<byte> > cbtmp = rescaleOpt(makeRGB(rr, gg, bb), in.getDims(), itsDisplayInterp.getVal());

  // add our drawings on top of it, a rectangle, and text labels:
  cbtmp = composite(itsTraj, cbtmp, PixRGB<byte>(0)); // pure black is transparent
  drawRect(cbtmp, Rectangle(Point2D<int>(0, 0), cbtmp.getDims()), PixRGB<byte>(128, 128, 255));
  writeText(cbtmp, Point2D<int>(1,1), " Attention Map ", itsColorText.getVal(), PixRGB<byte>(0), itsFont, true);
  writeText(cbtmp, Point2D<int>(3, cbtmp.getHeight() - 3), rng.c_str(), itsColorText.getVal(),
            PixRGB<byte>(0), SimpleFont::FIXED(6), true, ANCHOR_BOTTOM_LEFT);

  // get our output Layout started:
  Layout< PixRGB<byte> > output = hcat(in, cbtmp);

  // grab all the VisualCortex maps:
  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  // Display all the conspicuity maps in a nice array:
  ImageSet< PixRGB<byte> > cmaps;
  getMegaComboMaps(chm, cmaps);

  // all right, assemble the final output:
  if (cmaps.size() > 0)
    {
      Layout< PixRGB<byte> > lmaps = arrcat(cmaps, output.getWidth() / cmaps[0].getWidth());
      return vcat(output, lmaps).render();
    }
  else
    return output.render();
}

// ######################################################################
void SimulationViewerStd::getMegaComboMaps(const rutz::shared_ptr<ChannelMaps>& chm,
                                           ImageSet< PixRGB<byte> >& cmaps, const uint depth)
{
  for (uint i = 0; i < chm->numSubchans(); ++i)
    {
      rutz::shared_ptr<ChannelMaps> subchm = chm->subChanMaps(i);

      NamedImage<float> m = subchm->getMap();
      if (m.initialized() == false) m.resize(chm->getMap().getDims(), true); // some channels may not have outputs yet
      m = inverseMap(m);//pu map back in cartesian coords if necessary
      Image< PixRGB<byte> > mm = prepMapForDisplay(m);
      cmaps.push_back(mm);

      if (itsMegaComboTopCMapsOnly.getVal() == false && subchm->numSubchans() > 0)
        getMegaComboMaps(subchm, cmaps, depth + 1);
    }
}

// ######################################################################
Image< PixRGB<byte> > SimulationViewerStd::prepMapForDisplay(const NamedImage<float>& m) const
{
  const uint zoomfac = itsMegaComboZoom.getVal();

  // create a string with the map's range of values:
  float mini, maxi; getMinMax(m, mini, maxi);
  const std::string rng = sformat("[%.03f .. %.03f]", mini, maxi);

  // split the name into several tokens so that we can write them on
  // different lines:
  std::vector<std::string> tokens;
  split(m.name(), ":", std::back_inserter(tokens));

  // rescale, normalize and colorize the map:
  Image<float> ftmp = rescaleOpt(m, m.getWidth() * zoomfac, m.getHeight() * zoomfac, itsDisplayInterp.getVal());
  inplaceNormalize(ftmp, 0.0f, 255.0f); Image<byte> btmp = ftmp; // convert to byte
  Image< PixRGB<byte> > cbtmp = btmp; // convert to color

  // add a rectangle and the text labels:
  drawRect(cbtmp, Rectangle(Point2D<int>(0, 0), cbtmp.getDims()), PixRGB<byte>(128, 128, 255));
  for (size_t i = 1; i < tokens.size(); ++i) // note that we start at 1 to ignore "VisualCortex:" label
    writeText(cbtmp, Point2D<int>(1, 1+(i-1) * itsFont.h()), tokens[i].c_str(), itsColorText.getVal(),
              PixRGB<byte>(0), itsFont, true, ANCHOR_TOP_LEFT);
  writeText(cbtmp, Point2D<int>(1, cbtmp.getHeight() - 2), rng.c_str(), itsColorText.getVal(),
            PixRGB<byte>(0), SimpleFont::FIXED(6), true, ANCHOR_BOTTOM_LEFT);

  return cbtmp;
}

// ######################################################################
void SimulationViewerStd::drawMaskOutline(Image< PixRGB<byte> >& traj,
                                          const Image<byte> mask,
                                          const PixRGB<byte>& col,
                                          const int thick,
                                          const Point2D<int>& pos,
                                          const int radius)
{
  if (traj.initialized() == false) return; // can't draw...


  // object-shaped drawing
  Image<byte> om(mask);
  inplaceLowThresh(om, byte(128)); // cut off fuzzy (interpolated) object boundaries
  om = contour2D(om);       // compute binary contour image
  int w = traj.getWidth(), h = traj.getHeight();
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
