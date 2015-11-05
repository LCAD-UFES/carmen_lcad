/*!@file Neuro/SimulationViewerCompress.C multi-foveated saliency-based
  compression */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerCompress.C $
// $Id: SimulationViewerCompress.C 13343 2010-04-30 22:37:42Z lior $
//

#include "Neuro/SimulationViewerCompress.H"

#include "Channels/IntensityChannel.H"
#include "Channels/ChannelOpts.H" // for OPT_LevelSpec
#include "Channels/ChannelMaps.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"   // for toRGB() etc.
#include "Image/CutPaste.H"   // for concatX(), inplacePaste() etc.
#include "Image/DrawOps.H"    // for drawDisk(), drawPatch() etc.
#include "Image/FilterOps.H"  // for lowPass9()
#include "Image/MathOps.H"    // for binaryReverse(), thresholdedMix()
#include "Image/PyramidOps.H" // for buildPyrGaussian(), weightedBlur()
#include "Image/ShapeOps.H"   // for rescale()
#include "Image/Transforms.H" // for chamfer34()
#include "Neuro/AttentionGuidanceMap.H"
#include "Neuro/Brain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/Retina.H"
#include "Neuro/SaccadeControllers.H"
#include "Neuro/SaccadeControllerConfigurator.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/ShapeEstimator.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/TaskRelevanceMap.H"
#include "Neuro/VisualCortex.H"
#include "Simulation/SimulationOpts.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/MathFunctions.H"
#include "Neuro/NeuroSimEvents.H"
#include "Media/MediaSimEvents.H"

#include <stdio.h>
// ######################################################################
SimulationViewerCompress::
SimulationViewerCompress(OptionManager& mgr,
                         const std::string& descrName,
                         const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  SIMCALLBACK_INIT(SimEventSaccadeStatusEye),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsFOAradius(&OPT_FOAradius, this),
  itsNumFoveas(&OPT_SVCOMPnumFoveas, this),
  itsSaveTraj(&OPT_SVsaveTraj, this),
  itsSaveMegaCombo(&OPT_SVmegaCombo, this),
  itsSaveMask(&OPT_SVCOMPsaveMask, this),
  itsSaveFoveatedImage(&OPT_SVCOMPsaveFoveatedImage, this),
  itsDistanceFactor(&OPT_SVCOMPDistanceFactor, this),
  itsSaveEyeCombo(&OPT_SVCOMPsaveEyeCombo, this),
  itsDisplayPatch(&OPT_SVdisplayPatch, this),
  itsDisplayFOA(&OPT_SVdisplayFOA, this),
  itsDisplayEye(&OPT_SVCOMPdisplayHumanEye, this),
  itsColorNormal("SVcolorNormal", this, PixRGB<byte>(255, 255, 0)),
  itsColorEye("SVcolorHumanEye", this, PixRGB<byte>(128, 255, 255)),
  itsHeadRadius(&OPT_HeadMarkerRadius, this),
  itsMultiRetinaDepth(&OPT_SVCOMPMultiRetinaDepth, this),
  itsCacheSize(&OPT_SVCOMPcacheSize, this),
  itsUseTRMmax(&OPT_SVCOMPuseTRMmax, this),
  itsFoveaSCtype(&OPT_SVCOMPfoveaSCtype, this),
  itsOutFname(&OPT_SVEMoutFname, this),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsNumRandomSamples(&OPT_SVEMnumRandomSamples, this),
  itsEyeCompare(&OPT_SVCOMPeyeCompare, this),
  itsIFramePeriod(&OPT_SVCOMPiframePeriod, this),
  itsMultiTraj(),
  itsSC(), itsInputTime(), itsFrame(-1), itsMask(),
  itsIgnoreSC(), itsCurrentMask(), itsOutFile(NULL), itsBlurMask(),
  itsEyeData()
{
  LINFO("NOTE: disabling IOR");
  getManager().setOptionValString(&OPT_IORtype, "None");

  // select an eyetrack EyeHeadController:
  if(itsEyeCompare.getVal())
    getManager().setOptionValString(&OPT_EyeHeadControllerType, "EyeTrack");
}

// ######################################################################
SimulationViewerCompress::~SimulationViewerCompress()
{ }

// ######################################################################
void SimulationViewerCompress::paramChanged(ModelParamBase* const param,
                                            const bool valueChanged,
                                            ParamClient::ChangeStatus* status)
{
  SimulationViewer::paramChanged(param, valueChanged, status);
  if (valueChanged && (param == &itsNumFoveas || param == &itsFoveaSCtype)) buildSCC();
}

// ######################################################################
void SimulationViewerCompress::buildSCC()
{
  // drop any old SCCs:
  removeAllSubComponents();

  LINFO("Using %d SaccadeControllers of type %s", itsNumFoveas.getVal(), itsFoveaSCtype.getVal().c_str());

  // build an array of SCCs and export their options:
  for (int i = 0; i < itsNumFoveas.getVal(); i ++)
    {
      nub::soft_ref<SaccadeControllerEyeConfigurator> scc(new SaccadeControllerEyeConfigurator(getManager()));
      // we need to change the tag name so that we won't get confused
      // among our various SCCs:
      char num[10]; sprintf(num, "%d", i);
      scc->setTagName(scc->tagName() + num);
      scc->setDescriptiveName(scc->descriptiveName() + " " + std::string(num));
      scc->exportOptions(MC_RECURSE);
      addSubComponent(scc);

      // let's change its SC type to not what the command-line says,
      // but what we say:
      scc->setModelParamString("SaccadeControllerEyeType", itsFoveaSCtype.getVal());
    }
}

// ######################################################################
void SimulationViewerCompress::start1()
{
  itsSC.clear(); itsEyeData.clear();
  itsMask.setMaxSize(itsCacheSize.getVal());

  // setup shortcuts to our configured SCs:
  for (uint i = 0; i < numSubComp(); i ++) {
    nub::soft_ref<SaccadeControllerEyeConfigurator> scc = dynCast<SaccadeControllerEyeConfigurator>(subComponent(i));
    itsSC.push_back(scc->getSC());
    itsIgnoreSC.push_back(false);
  }

  // open output file if any:
  if (itsOutFname.getVal().empty() == false) {
    itsOutFile = fopen(itsOutFname.getVal().c_str(), "w");
    if (itsOutFile == NULL) PLFATAL("Cannot write '%s'", itsOutFname.getVal().c_str());
  }

  SimulationViewer::start1();
}

// ######################################################################
void SimulationViewerCompress::stop1()
{
  if (itsOutFile) { fclose(itsOutFile); itsOutFile = NULL; }
}

// ######################################################################
void SimulationViewerCompress::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  itsInputTime = q.now(); // keep track of time of last input
  ++ itsFrame; // keep track of frame number, to decide whether I-frame
  itsRawInputRectangle = e->rawInputRectangle();

  //  get a foveation pyramid ready:
  itsMultiTraj = buildPyrGaussian(e->frame().colorByte(), 0, itsMultiRetinaDepth.getVal(), 9);
}

// ######################################################################
void SimulationViewerCompress::
onSimEventSaccadeStatusEye(SimEventQueue& q, rutz::shared_ptr<SimEventSaccadeStatusEye>& e)
{
  // all the analysis will be done in getTraj(), so that we don't have
  // to recompute the whole blur mask at every eye movement
  // sample. Hence, here we just queue up the eye positions received:
  itsEyeData.push_back(e->position());
}

// ######################################################################
Image< PixRGB<byte> > SimulationViewerCompress::getTraj(SimEventQueue& q)
{
  Dims dims = itsMultiTraj[0].getDims(); // input image dims

  // let's get the current normalized (values in 0..255) saliency map:
  Image<float> sm = getMap(q, true);

  // find the top itsNumFoveas salient locations; to this end, we will
  // find the max in sm, then use the FOAradius to draw a disk at the
  // top location, then loop as many times as we have foveas:
  Image<float> smf = rescaleOpt(sm, dims, itsDisplayInterp.getVal());

  // in this function, we merge two behaviors: if we have
  // subcomponents (i.e., SaccadeControllers), then we will work in a
  // mode where we have a bunch of foveas moving around. Otherwise, we
  // will use the saliency map as a continuous modulator of blur:
  Image<byte> msk;
  if (itsSC.size()) msk = getMaskSC(smf, q); else msk = getMaskSM(smf);

  // add this mask to our sliding average:
  if (itsCacheSize.getVal() > 0)  // using a sliding average cache
    {
      itsMask.push_back(msk);

      // are we on an I-frame? If so, update our itsCurrentMask, and use
      // it. Otherwise, use the current contents of itsCurrentMask:
      if (itsFrame % itsIFramePeriod.getVal() == 0) itsCurrentMask = itsMask.mean();
    }
  else
    itsCurrentMask = msk;  // just using instantaneous mask

  // update blur mask using TRM if needed, otherwise it's just itsCurrentMask:
  if (itsUseTRMmax.getVal() && itsBlurMask.initialized()) {
    // if a location is rapidly changing (high TRM value), we use the
    // value of itsCurrentMask for our blur; otherwise, we take the
    // min between itsCurrentMask and our accumulated itsBlurMask:
    Image<byte> minMask = takeMin(itsBlurMask, itsCurrentMask);
    if (SeC<SimEventTaskRelevanceMapOutput> e = q.check<SimEventTaskRelevanceMapOutput>(this, SEQ_ANY)) {
      Image<float> trm = rescaleOpt(e->trm(1.0F), dims, itsDisplayInterp.getVal());
      itsBlurMask = thresholdedMix(trm, 0.99F, minMask, itsCurrentMask);
    } else LFATAL("Cannot find a TRM!");
  } else itsBlurMask = itsCurrentMask;

  // we use the mean of our mask sliding average for a weighted blur:
  Image< PixRGB<byte> > traj = weightedBlur(itsBlurMask, itsMultiTraj);  // weighted blur

  // draw a patch at center of each fovea?
  if (itsDisplayPatch.getVal())
    {
      // select a drawing color & size:
      PixRGB<byte> col(itsColorNormal.getVal()); int psiz = 4 + 2*itsSC.size();

      // draw a patch at current position of each fovea:
      for (uint i = 0; i < itsSC.size(); i ++)
        if (itsIgnoreSC[i] == false) drawPatchBB(traj, itsSC[i]->getPreviousDecision(0).p, psiz-i*2, col);
    }

  // draw FOA outlines?
  if (itsDisplayFOA.getVal())
    {
      // select a drawing color & size:
      PixRGB<byte> col(itsColorNormal.getVal()); int thick = 3;

      Image<byte> om(itsCurrentMask);
      om = binaryReverse(om, byte(255));
      inplaceLowThresh(om, byte(220), byte(0)); // get the objects
      om = contour2D(om);          // compute binary contour image
      int w = traj.getWidth(), h = traj.getHeight();
      Point2D<int> ppp;
      for (ppp.j = 0; ppp.j < h; ppp.j ++)
        for (ppp.i = 0; ppp.i < w; ppp.i ++)
          if (om.getVal(ppp.i, ppp.j))  // got a contour point -> draw here
            drawDisk(traj, ppp, thick, col);  // small disk for each point
    }

  // prepare a full-size color version of the SM for our various markings:
  Image< PixRGB<byte> > colorsm = toRGB(Image<byte>(rescaleOpt(sm, dims, itsDisplayInterp.getVal())));

  // get the raw, unfoveated input image and paste it into an image with our dims:
  Image< PixRGB<byte> > rawinp2;
  if (SeC<SimEventInputFrame> e = q.check<SimEventInputFrame>(this)) rawinp2 = e->frame().asRgb();
  Image< PixRGB<byte> > rawinp(dims, NO_INIT); rawinp.clear(PixRGB<byte>(64));
  Point2D<int> rawinpoff((rawinp.getWidth() - rawinp2.getWidth())/2, (rawinp.getHeight() - rawinp2.getHeight())/2);
  inplacePaste(rawinp, rawinp2, rawinpoff);

  // do we want to compare to human eye movement data?
  if (itsOutFile)
    {
      // compute average blur for this frame:
      byte mi, ma, avg; getMinMaxAvg(itsCurrentMask, mi, ma, avg);

      // get the raw SM:
      Image<float> rawsm;
      if (SeC<SimEventSaliencyMapOutput> e = q.check<SimEventSaliencyMapOutput>(this, SEQ_ANY))
        rawsm = e->sm(1.0F); else LFATAL("Cannot find a SM!");

      // get the map level to scale things down:
      int sml = itsLevelSpec.getVal().mapLevel();

      // let's get the raw saliency map and a vector of all our
      // conspicuity maps and of their min/max/avg:
      std::vector< Image<float> > cmap;
      std::vector<float> cmi, cma, cav;

      // grab all the VisualCortex maps:
      rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
      q.request(vcxm); // VisualCortex is now filling-in the maps...
      rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

      // find out a window to use for our random values, which is
      // important in case we are doing input shifting with
      // --shift-input and apply a field of view with
      // --input-fov. In these cases, we want to take random samples
      // only withing the actual display area:
      Rectangle r = itsRawInputRectangle;
      //drawRect(colorsm, r, PixRGB<byte>(0, 255, 0), 2);
      //drawRect(traj, r, PixRGB<byte>(0, 255, 0), 2);

      // get a version of the rectangle scaled to SM dims:
      Rectangle rsm = Rectangle::tlbrI(r.top() >> sml, r.left() >> sml, r.bottomO() >> sml, r.rightO() >> sml);
      rsm = rsm.getOverlap(rawsm.getBounds());

      // let's get the raw saliency map and a vector of all our
      // conspicuity maps and of their min/max/avg:
      Image<float> cropsm = crop(rawsm, rsm);
      float rawsmmi, rawsmma, rawsmav;
      getMinMaxAvg(cropsm, rawsmmi, rawsmma, rawsmav);

      // now for all the top-level channel conspicuity maps:
      for (uint ii = 0; ii < chm->numSubchans(); ii ++) {
        Image<float> cm = chm->subChanMaps(ii)->getMap();
        if (cm.initialized() == false) cm.resize(rawsm.getDims(), true); // some channels may not have maps yet
        Image<float> cropmap = crop(cm, rsm);
        float ccmi, ccma, ccav;
        getMinMaxAvg(cropmap, ccmi, ccma, ccav);
        cmi.push_back(ccmi); cma.push_back(ccma); cav.push_back(ccav);
        cmap.push_back(cropmap);
      }

      // loop over all fixations that happened during current frame:
      while(itsEyeData.size()) {
        // pick a random location to get blur there:
        Point2D<int> rnd(randomUpToNotIncluding(itsCurrentMask.getWidth()),
                         randomUpToNotIncluding(itsCurrentMask.getHeight()));

        // get next eye fixation:
        Point2D<int> eye = itsEyeData.front(); itsEyeData.pop_front();
        eye.clampToDims(itsCurrentMask.getDims());

        // also scale down eye coords to sm level:
        Point2D<int> eyesm(eye.i >> sml, eye.j >> sml);

        // finally shift eyesm to reflect our crops of the sm and cmaps:
        eyesm.i -= rsm.left(); eyesm.j -= rsm.top();
        eyesm.clampToDims(cropsm.getDims());

        // do we want to draw it?
        if (itsDisplayEye.getVal())
          {
            // select a drawing color & size:
            PixRGB<byte> col(itsColorEye.getVal()); int psiz = 5;
            drawPatchBB(traj, eye, psiz, col);
            drawPatchBB(colorsm, eye, psiz, col);

            // grab the latest retina:
            Point2D<int> rieye;
            if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this, SEQ_ANY))
              rieye = e->retinalToRaw(eye);
            else LFATAL("ooops, no retina image in the queue?");
            rieye += rawinpoff;
            drawPatchBB(rawinp, rieye, psiz, col);
          }

        fprintf(itsOutFile, "%d %d %d %d %d %d",
                eye.i,                         // eye x position
                eye.j,                         // eye y position
                itsCurrentMask.getVal(eye),    // blur val at eye
                mi,                            // min val of mask
                ma,                            // max val of mask
                avg);                          // average val of mask

        LINFO("eye pos and blur val at eye, mi, ma, avg:%d %d %d %d %d %d",
              eye.i, eye.j, itsCurrentMask.getVal(eye),mi,ma, avg);
        for(int k=0; k<itsNumRandomSamples.getVal(); k++)
          {
            Point2D<int> randp(randomUpToNotIncluding(itsCurrentMask.getWidth()),
                               randomUpToNotIncluding(itsCurrentMask.getHeight()));
            fprintf(itsOutFile, " %d", itsCurrentMask.getVal(randp));
          }
        fprintf(itsOutFile, "\n");
      }
    }

  // do we want a mega combo instead of the plain blurred image?
  if (itsSaveMegaCombo.getVal())
    {
      Image< PixRGB<byte> > ret =
        concatX(colGreyCombo(itsMultiTraj[0], rescaleOpt(sm, dims, itsDisplayInterp.getVal()), false),
                colGreyCombo(traj, itsCurrentMask, false));
      drawGrid(ret, 2, 2, 2, PixRGB<byte>(128));
      return ret;
    }

  // do we want a mask only?
  if(itsSaveMask.getVal()) return itsCurrentMask;

  // do we want a foveated image  only?
  if(itsSaveFoveatedImage.getVal()) return traj;

  // do we want an eye combo?
  if (itsSaveEyeCombo.getVal())
    {
      Image< PixRGB<byte> > ret = concatX(concatX(rawinp, traj), colorsm);
      drawLine(ret, Point2D<int>(dims.w()-1, 0), Point2D<int>(dims.w()-1, dims.h()-1), PixRGB<byte>(255,255,0), 3);
      drawLine(ret, Point2D<int>(dims.w()*2-1, 0), Point2D<int>(dims.w()*2-1,dims.h()-1), PixRGB<byte>(255,255,0), 3);

      // make sure the size is reasonable...
      while(ret.getWidth() > 1024) ret = decXY(lowPass3(ret));

      return ret;
    }

  // otherwise return the blurred image:
  return traj;
}

// ######################################################################
Image<byte> SimulationViewerCompress::getMaskSM(const Image<float>& smf)
{
  // let's start by smoothing the interpolated salmap a bit:
  Image<float> maskf = lowPass9(smf);

  // let's squash the SM a bit. We downplay values below average and
  // give more range to those above average (with possible saturation):
  float mi, ma, av; getMinMaxAvg(maskf, mi, ma, av);
  maskf = squash(maskf, mi, mi, 0.5F*(av-mi), 0.55F*(av-mi), ma, ma);

  // make a blurring mask:
  Image<byte> mask = binaryReverse(maskf, 255.0F);  // will clamp to 0..255

  return mask;
}

// ######################################################################

namespace
{
  struct Point2DS
  {
    Point2DS(const Point2D<int>& pp, double ss) : p(pp), sal(ss) {}

    Point2D<int> p;
    double sal;
  };
}

Image<byte> SimulationViewerCompress::getMaskSC(const Image<float>& smf,
                                                SimEventQueue& q)
{
  ///////nub::ref<VisualCortex> vc = itsBrain->getVC();
  ////FIXME///nub::ref<ShapeEstimator> se = itsBrain->getSE();

  // violently reset the se:
  ////FIXME///se->reset(MC_RECURSE);

  // find the top salient locations; to this end, we will find the max
  // in sm, then use the FOAradius to draw a disk at the top location,
  // then loop. We will extract more locations than we have foveas, so
  // that we are robust to slight changes in saliency ordering:
  std::vector<Point2DS> topsal;  // will store saliency in 't' field
  Image<float> smff = smf;  // get a copy we can modify
  for (uint i = 0; i < itsSC.size() + 4; i ++) {
    // find max:
    Point2D<int> p; float sal; findMax(smff, p, sal);

    // store coords & saliency:
    topsal.push_back(Point2DS(p, double(sal)));

    // get object shape at that location, or revert to a disk of no object:
    ////FIXME///se->compute(p);
    Image<byte> objmask; ////FIXME/// = se->getSmoothMask() * 255.0F;
    if (objmask.initialized() == false) {
      objmask.resize(smff.getDims(), true);
      drawDisk(objmask, p, itsFOAradius.getVal(), byte(255));
    }
    // inhibit the sm by the object shape:
    inplaceSetValMask(smff, objmask, 0.0F);
  }

  // if this is our first time (itsFeatures is empty), just assign an
  // SC to each of the top salient locations:
  if (itsFeatures.empty())
    for (uint i = 0; i < itsSC.size(); i ++)
      {
        // feed the SC:
        itsSC[i]->setPercept(WTAwinner(topsal[i].p, q.now(), topsal[i].sal, false), q);

        // keep track of the features each SC is tracking:
        rutz::shared_ptr<SimReqVCXfeatures> ef(new SimReqVCXfeatures(this, topsal[i].p));
        q.request(ef); // VisualCortex is now filling-in the features into ef->features()
        itsFeatures.push_back(ef->features());

        LINFO("Initializing SC[%d] to (%d,%d)", i, topsal[i].p.i, topsal[i].p.j);
      }
  else
    {
      // let's get an idea of which features are important for
      // differentiating between our current foveas, and of what
      // their range is:
      std::vector<double> minf, maxf; uint nf = itsFeatures[0].size();
      for (uint i = 0; i < nf; i ++) { minf.push_back(1.0e50); maxf.push_back(-1.0e50); }

      for (uint fov = 0; fov < itsSC.size(); fov ++)
        for (uint i = 0; i < nf; i ++)
          {
            if (itsFeatures[fov][i] < minf[i]) minf[i] = itsFeatures[fov][i];
            if (itsFeatures[fov][i] > maxf[i]) maxf[i] = itsFeatures[fov][i];
          }

      // solve correspondence problem: Score each salient location with
      // respect to each SC, based on distance and feature similarity in a
      // neighborhood:
      Image<float> score(topsal.size(), itsSC.size(), NO_INIT);
      for (uint i = 0; i < topsal.size(); i ++)
        for (uint j = 0; j < itsSC.size(); j ++)
          {
            // how well does salient location 'i' score with fovea
            // 'j'? First let's look at distance; what counts then is
            // the distance to the target (=percept) of the SC, not to
            // where the SC currently is:
            Point2D<int> pi = topsal[i].p;
            Point2D<int> pj = itsSC[j]->getPreviousPercept(0).p;
            float dist = pi.distance(pj);

            // a distance of up to twice our FOA radius yields no penalty:
            if (dist < 2.0F * float(itsFOAradius.getVal())) dist = 0.0F;

            // normalize the distance to 0..1 range:
            dist /= sqrt(smf.getWidth() * smf.getWidth() + smf.getHeight() * smf.getHeight());

            // now look at feature similarity; to this end, we explore
            // a neighborhood of the salient location and see whether
            // we can find the features that the SC is interested in:
            std::vector<float> scf = itsFeatures[j];
            double fdist = 1.0e50;
            int ci = pi.i; if (ci & 1) ci --;
            int cj = pi.j; if (cj & 1) cj --;
            for (int jj = cj - 10; jj <= cj + 10; jj += 2)
              for (int ii = ci - 10; ii <= ci + 10; ii += 2)
                {
                  Point2D<int> p(ii, jj);
                  if (smf.coordsOk(p))
                    {
                      // get a vector of features:
                      rutz::shared_ptr<SimReqVCXfeatures> ef(new SimReqVCXfeatures(this, p));
                      q.request(ef); // VisualCortex is now filling-in the features into ef->features()
                      const std::vector<float>& ff = ef->features();

                      // compute feature distance: get feature
                      // difference and normalize by feature range
                      // if range not too small:
                      double d = 0.0; int numf = 0;
                      for (uint k = 0; k < nf; k ++)
                        if (maxf[k] > minf[k] + 1.0)
                          {
                            double dd = (ff[k]-scf[k]) / (maxf[k]-minf[k]);

                            // accumulate compound feature distance:
                            d += dd * dd; numf ++;
                          }

                      // compute normalized weighted feature distance:
                      if (numf) d = sqrt(d / double(numf));

                      // if distance better than what we had, update:
                      if (d < fdist) fdist = d;
                    }
                }

            // point distance and feature distance both contribute
            // to score. In addition, we add here a penalty for
            // changing the ordering of the SCs. Best possible score
            // is zero and all scores are negative, growing larger
            // negatively as they get worse:
            float sco = -
              float(dist * 100.0) -
              float(fdist * 0.5) -
              10.0F * fabs(float(i)-float(j)) +
              100.0F * topsal[i].sal / topsal[0].sal;

            score.setVal(i, j, sco);
            LINFO("[topsal(%d)(%d,%d), SC(%d)(%d,%d)]: dist=%f fdist=%f "
                  "score=%f", i, pi.i, pi.j, j, pj.i, pj.j, dist, fdist, sco);
          }

      // find the best score and assign the corresponding salient
      // location to the corresponding SC; then kill that SC in the
      // score map and loop until all SCs have been assigned:
      for (uint i = 0; i < itsSC.size(); i ++)
        {
          Point2D<int> best; float val; findMax(score, best, val);
          int bi = best.i;  // salient location
          int bj = best.j;  // saccade controller

          // set new percept to the winning SC:
          itsSC[bj]->setPercept(WTAwinner(topsal[bi].p, q.now(), topsal[bi].sal, false), q);
          // also assign new feature vector to that SC; even though
          // the best score may have been achieved for a neighbor of
          // the topsal location, we use as feature vector the one
          // from the topsal location:
          rutz::shared_ptr<SimReqVCXfeatures> ef(new SimReqVCXfeatures(this, topsal[bi].p));
          q.request(ef); // VisualCortex is now filling-in the features into ef->features()
          itsFeatures[bj] = ef->features();

          // done with this SC; let's make sure we will not pick it up again:
          for (int k = 0; k < score.getWidth(); k ++) score.setVal(k, bj, -1.0e30F);

          // also make sure we will not pick up that salient loc again:
          for (int k = 0; k < score.getHeight(); k ++) score.setVal(bi, k, -1.0e30F);

          LINFO("Assigned topsal[%d](%d,%d) to SC[%d](%d,%d)", bi, topsal[bi].p.i, topsal[bi].p.j, bj,
                itsSC[bj]->getPreviousPercept(1).p.i, itsSC[bj]->getPreviousPercept(1).p.j);
        }
    }

  // evolve our SCs:
  for (uint i = 0; i < itsSC.size(); ++i) itsSC[i]->evolve(q);

  // run getDecision() on our SaccadeControllers:
  for (uint i = 0; i < itsSC.size(); ++i) itsSC[i]->getDecision(q, false);

  // create a mask with the object shapes at each SC:
  ///FIXME///se->reset(MC_RECURSE);  // violently reset the SE
  Image<float> maskf(smf.getDims(), ZEROS);
  for (uint i = 0; i < itsSC.size(); ++i)
    {
      // if we terminate this iteration early we will ignore this SC:
      itsIgnoreSC[i] = true;

      // get the current fixation for this SC:
      Point2DT p = itsSC[i]->getPreviousDecision(0);
      if (p.p.isValid() == false)
        { LINFO("Ignoring SC[%d] because coords (%d,%d) invalid", i, p.p.i, p.p.j); continue; }

      // if salience was very low, don't bother using this SC:
      if (smf.getVal(itsSC[i]->getPreviousPercept(0).p) < topsal[0].sal*0.05)
        { LINFO("Ignoring SC[%d] because salience too low", i); continue; }

      // otherwise segment the object and mark it:
      ////FIXME///se->compute(p.p);
      Image<float> objf;////FIXME/// = se->getSmoothMask();
      if (objf.initialized()) maskf = takeMax(maskf, objf);
      else drawDisk(maskf, p.p, itsFOAradius.getVal(), 1.0F);

      // ok, we won't ignore this SC:
      itsIgnoreSC[i] = false;
    }

  // binarize the object mask:
  inplaceLowThresh(maskf, 0.5F, 0.0F);

  // create a distance map from the mask (this code similar to that in foveate()):
  float maxdist = std::max(smf.getWidth(), smf.getHeight()) * 2 * itsDistanceFactor.getVal();
  float scalefac = maxdist / 255.0f;
  maskf = chamfer34(maskf, maxdist) / scalefac;

  // if modulator does not contain any point at zero (inside object),
  // that means that the mask was empty, which is the case at the
  // beginning of a simulation. Set it to some intermediary value to
  // provide a uniform medium blur; otherwise, squash it:
  float mi, ma, av; getMinMaxAvg(maskf, mi, ma, av);
  if (mi > 0.0F) maskf /= 3.0F;
  else maskf = squash(maskf, mi, mi, 0.5F*(av-mi), 0.6F*(av-mi), ma, ma);

  // return byte version of the mask:
  Image<byte> mask = maskf;  // will clamp as necessary
  return mask;
}

// ######################################################################
void SimulationViewerCompress::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  // update the trajectory:
  Image< PixRGB<byte> > res = getTraj(q);

  // save results?
  if (itsSaveTraj.getVal() || itsSaveMegaCombo.getVal() ||
      itsSaveEyeCombo.getVal() || itsSaveMask.getVal() || itsSaveFoveatedImage.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

      ofs->writeRGB(res, "T", FrameInfo("SimulationViewerCompress trajectory", SRC_POS));
    }
}

// ######################################################################
float SimulationViewerCompress::getSample(const Image<float>& smap,
                                          const Point2D<int>& p,
                                          const int radius) const
{
  // ### disk version:
  Image<float> fov(smap.getDims(), ZEROS);
  drawDisk(fov, p, radius, 1.0F);
  fov *= smap;  // max smap by the disk
  float junk, salience;
  getMinMax(fov, junk, salience);
  return salience;

  // ### point version:
  //  return smap.getVal(p);
}

// ######################################################################
float SimulationViewerCompress::
getRandomSample(const Image<float>& smap, const int radius, const int n) const
{
  float rndval = 0.0f;
  for (int i = 0; i < n; i ++)
    {
      Point2D<int> rndsm(randomUpToNotIncluding(smap.getWidth()), randomUpToNotIncluding(smap.getHeight()));
      rndval += getSample(smap, rndsm, radius);
    }
  return rndval / n;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
