/*!@file Neuro/Retina.C a human retina */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/Retina.C $
// $Id: Retina.C 15195 2012-03-01 19:52:57Z dberg $
//

#include "Neuro/Retina.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"//for luminance()
#include "Image/MatrixOps.H"
#include "Image/Point2DT.H"
#include "Image/PyramidOps.H" // for buildPyrGaussian(), foveate()
#include "SpaceVariant/SpaceVariantOpts.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SpatialMetrics.H"
#include "Channels/ChannelOpts.H"
#include "Psycho/EyeData.H"
#include "Raster/Raster.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/sformat.H"  // for sformat()

#include <cstdlib>
#include <iostream>
#include <exception>
#include <vector>

// ######################################################################

static const ModelOptionDef OPT_ClipMaskFname =
  { MODOPT_ARG_STRING, "ClipMaskFname", &MOC_BRAIN, OPTEXP_CORE,
    "Name of a grayscale image file to be loaded and used as a "
    "clipmask for Brain",
    "clip-mask", '\0', "<filename>", "" };

static const ModelOptionDef OPT_RawInpRectBorder =
  { MODOPT_ARG(int), "RawInpRectBorder", &MOC_BRAIN, OPTEXP_CORE,
    "Border size to use for the Retina's raw input rectangle (used to "
    "select random samples in SimulationViewerCompress), in pixels.",
    "rawinput-rect-border", '\0', "<int>", "128" };

static const ModelOptionDef OPT_EnablePyramidCaches =
  { MODOPT_FLAG, "EnablePyramidCaches", &MOC_BRAIN, OPTEXP_CORE,
    "Whether to allow caching of commonly-needed image pyramids based "
    "on the current input image, such as the intensity pyramid shared "
    "between the intensity channel and the motion channels, or the "
    "laplacian pyramid shared among the oriented gabor channels. There "
    "should be no reason to disable pyramid caching except for "
    "debugging or profiling.",
    "enable-pyramid-caches", '\0', "", "true" };

// ######################################################################
// ######################################################################
// ########## Retina implementation
// ######################################################################
// ######################################################################
Retina::Retina(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
  SimModule(mgr, descrName, tagName)
{ }

Retina::~Retina()
{ }

// ######################################################################
// ######################################################################
// ########## RetinaAdapter implementation
// ######################################################################
// ######################################################################
RetinaAdapter::RetinaAdapter(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
  Retina(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  SIMCALLBACK_INIT(SimEventSaccadeStatusEye),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsClipMaskFname(&OPT_ClipMaskFname, this),
  itsRawInpRectBorder(&OPT_RawInpRectBorder, this),
  itsInitialEyePosition(&OPT_SCeyeInitialPosition, this),
  itsEnablePyramidCaches(&OPT_EnablePyramidCaches, this),
  itsFoveaRadius(&OPT_FoveaRadius, this),
  itsSaveInput(&OPT_RetinaSaveInput, this), // see Neuro/NeuroOpts.{H,C}
  itsSaveOutput(&OPT_RetinaSaveOutput, this),
  itsFramingImageName(&OPT_InputFramingImageName, this),
  itsFramingImagePos(&OPT_InputFramingImagePos, this),
  itsFoveateInputDepth(&OPT_FoveateInputDepth, this), // idem
  itsShiftInput(&OPT_ShiftInputToEye, this), // see Neuro/NeuroOpts.{H,C}
  itsShiftInputBGcol(&OPT_ShiftInputToEyeBGcol, this), // idem
  itsInputFOV(&OPT_InputFOV, this),  // see Neuro/NeuroOpts.{H,C}
  itsSavePyr(&OPT_RetinaStdSavePyramid, this),
  itsBlankBlink(&OPT_BlankBlink, this),
  itsRetMaskFname(&OPT_RetinaMaskFname, this),
  itsFlipHoriz(&OPT_RetinaFlipHoriz, this),
  itsFlipVertic(&OPT_RetinaFlipVertic, this),
  itsClipMask(),
  itsEyePos(-1, -1),
  itsEyeBlinkStatus(false),
  itsRawInput(),
  itsOutput(),
  itsFramingImage(),
  itsRetinalShift(0, 0),
  itsRetMask()
{ }

// ######################################################################
RetinaAdapter::~RetinaAdapter()
{ }

// ######################################################################
void RetinaAdapter::onSimEventSaccadeStatusEye(SimEventQueue& q, rutz::shared_ptr<SimEventSaccadeStatusEye>& e)
{
  // update our blink status and eye position:
  const TransientStatus bs = e->blinkStatus();
  if (transientStatusIsOn(bs)) itsEyeBlinkStatus = true;
  else if (transientStatusIsOff(bs)) itsEyeBlinkStatus = false;
  itsEyePos = e->position(); // in retinal coordinates

  if (itsEyePos.isValid()) LDEBUG("Using eye position (%d, %d).", itsEyePos.i, itsEyePos.j);

  // if we are shifting inputs according to eye position, then we here need to output a new retina every time the eye
  // moves; otherwise, we will do it only every time the input frame changes, in onSimEventInputFrame(). Of course if
  // there is no input image yet, we output nothing, also time 0 is handled by onSimEventInputFrame():
  if (itsShiftInput.getVal() && itsRawInput.initialized() && q.now() != SimTime::ZERO()) {
    // all right, let's process the frame!
    const Image<PixRGB<byte> > outimg = getOutput(itsRawInput, itsEyePos, itsEyeBlinkStatus);

    // post an event with our output:
    InputFrame ifr = InputFrame::fromRgb(&outimg, q.now(), &itsClipMask,
                                         InputFrame::emptyCache, !itsEnablePyramidCaches.getVal());
    postInputFrame(q, ifr);
  }
}

// ######################################################################
void RetinaAdapter::onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  GenericFrame fr = e->frame();
  InputFrame ifr;

  // here is the inputs image:
  Image<PixRGB<byte> > inimg;
  Image<uint16> dimg;

  switch(fr.nativeType()) {
      case GenericFrame::RGBD:
      	  {
			  inimg = fr.asRgb();
			  dimg = fr.asGrayU16();
      	  }
      	  break;
      default:
          {
			  inimg = fr.asRgb();
          }
          break;
      }

  // keep a copy of input in case we want to later save or use it:
  itsRawInput = inimg;

  // NOTE that when we are just starting a simulation, the saccade controller will not have had a chance to post an eye
  // position yet. In this case, we will get it from our command-line:
  if (q.now() == SimTime::ZERO()) {
    itsEyePos = itsInitialEyePosition.getVal();
    if (itsEyePos.i == -2 && itsEyePos.j == -2) {
      // we want to start at center:
      itsEyePos.i = inimg.getWidth() / 2; itsEyePos.j = inimg.getHeight() / 2;

      // convert to retinal coords:
      itsEyePos += getRawToRetinalOffset();
    }
    if (itsEyePos.isValid()) LDEBUG("Using eye position (%d, %d).", itsEyePos.i, itsEyePos.j);
  }

  // if we are shifting inputs accordint to eye position, then we need to output a new retina every time the eye moves,
  // so that's done in onSimEventSaccadeStatusEye; otherwise, here, we will do it only every time the input frame
  // changes (except at time 0):
  if (itsShiftInput.getVal() == false || q.now() == SimTime::ZERO()) {
    // all right, let's process the frame!
    const Image<PixRGB<byte> > outimg = getOutput(inimg, itsEyePos, itsEyeBlinkStatus);


    if(fr.nativeType() == GenericFrame::RGBD)
    {
    	ifr = InputFrame::fromRgbDepth(&outimg, &dimg, q.now(), &itsClipMask, InputFrame::emptyCache, !itsEnablePyramidCaches.getVal());
    }
    else
    {
		// post an event with our output:
		ifr = InputFrame::fromRgb(&outimg, q.now(), &itsClipMask, InputFrame::emptyCache, !itsEnablePyramidCaches.getVal());
    }

    postInputFrame(q, ifr);

  }
}

// ######################################################################
void RetinaAdapter::postInputFrame(SimEventQueue& q, InputFrame& ifr) 
{
  rutz::shared_ptr<SimEventRetinaImage>
    eri(new SimEventRetinaImage(this, ifr, getRawInputRectangle(itsRawInput.getDims(), ifr.getDims()),
                                getRawToRetinalOffset()));
  q.post(eri);
}

// ######################################################################
Rectangle RetinaAdapter::getRawInputRectangle(const Dims& indims, const Dims& outdims) const
{
  const int border = itsRawInpRectBorder.getVal();
  Point2D<int> fpos = itsFramingImagePos.getVal();
  Rectangle r = Rectangle::tlbrI(itsRetinalShift.j + fpos.j + border,
                                 itsRetinalShift.i + fpos.i + border,
                                 itsRetinalShift.j + fpos.j + indims.h()-1 - border,
                                 itsRetinalShift.i + fpos.i + indims.w()-1 - border);
  return r.getOverlap(Rectangle(Point2D<int>(0,0), outdims));
}

// ######################################################################
Point2D<int> RetinaAdapter::getRawToRetinalOffset() const
{ return itsFramingImagePos.getVal() + itsRetinalShift; }

// ######################################################################
void RetinaAdapter::start1()
{
  if (!itsClipMaskFname.getVal().empty()) {
    itsClipMask = Raster::ReadGray(itsClipMaskFname.getVal());
    LINFO("Using clipmask from image file %s", itsClipMaskFname.getVal().c_str());
  }

  // if doing framing, read the framing image:
  if (itsFramingImageName.getVal().empty() == false) 
  {
    if (itsFramingImageName.getVal().find("BlackImage") != std::string::npos)
    {
      std::string size = itsFramingImageName.getVal().substr(10);

      itsFramingImage = Image<PixRGB<byte> >(fromStr<Dims>(size), ZEROS);
      LINFO("Using %dx%d framing image %s", itsFramingImage.getWidth(),
            itsFramingImage.getHeight(), itsFramingImageName.getVal().c_str());
    } 
    else
    {
      itsFramingImage = Raster::ReadRGB(itsFramingImageName.getVal());
      LINFO("Using %dx%d framing image %s", itsFramingImage.getWidth(),
            itsFramingImage.getHeight(), itsFramingImageName.getVal().c_str());
    }
  }

  // if doing input masking, read the mask image:
  if (itsRetMaskFname.getVal().empty() == false) {
    itsRetMask = Raster::ReadGray(itsRetMaskFname.getVal());
    LINFO("Using %dx%d retinal mask %s", itsRetMask.getWidth(),
          itsRetMask.getHeight(), itsRetMaskFname.getVal().c_str());
  }

  Retina::start1();
}

// ######################################################################
void RetinaAdapter::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void RetinaAdapter::save1(const ModelComponentSaveInfo& sinfo)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

  // save input?
  if (itsSaveInput.getVal() && itsRawInput.initialized())
    ofs->writeRGB(itsRawInput, "RETIN-", FrameInfo("retinal input", SRC_POS));

  // save output?
  if (itsSaveOutput.getVal() && itsOutput.initialized())
    ofs->writeRGB(itsOutput, "RETOUT-", FrameInfo("retinal output", SRC_POS));

  // save pyramid?
  if (itsSavePyr.getVal() && itsOutput.initialized())
    for (uint i = 0; i < itsMultiRetina.size(); i ++)
      ofs->writeRGB(itsMultiRetina[i], sformat("RET%d-", i),
                    FrameInfo(sformat("pyramid level %d in RetinaStd", i), SRC_POS));
}

// ######################################################################
Image< PixRGB<byte> > RetinaAdapter::getOutput(const Image<PixRGB<byte> >& inp,
                                               const Point2D<int>& eye, const bool inBlink)
{
  // start with our latest input:
  Image< PixRGB<byte> > ret = inp;

  // if we have a retinal mask image, apply it:
  if (itsRetMask.initialized())
    {
      if (ret.isSameSize(itsRetMask) == false)
        LFATAL("Retina (%dx%d) and retinal mask (%dx%d) dim mismatch",
               ret.getWidth(), ret.getHeight(), itsRetMask.getWidth(), itsRetMask.getHeight());

      // we rely on automatic promotions (ret * itsRetMask is Image< PixRGB<int> >) and demotion (assigment to result):
      ret = (ret * itsRetMask) / 255;
    }

  // any flipping?
  if (itsFlipHoriz.getVal()) ret = flipHoriz(ret);
  if (itsFlipVertic.getVal()) ret = flipVertic(ret);

  // embed our latest raw input within a larger framing image if any:
  if (itsFramingImage.initialized())
    {
      ret = itsFramingImage;
      inplacePaste(ret, inp, itsFramingImagePos.getVal());
    }

  // do we want to shift the input so that it is centered at the current eye position?
  if (itsShiftInput.getVal())
    {
      // do we have a valid eye position yet?
      if (eye.isValid())
        {
          LINFO("Shifting input to eye position (%d, %d)", eye.i, eye.j);

          itsRetinalShift.i += ret.getWidth() / 2 - eye.i;
          itsRetinalShift.j += ret.getHeight() / 2 - eye.j;

          ret = shiftClean(ret, itsRetinalShift.i, itsRetinalShift.j, itsShiftInputBGcol.getVal());
        }

      // in addition, do we want to crop the shifted input to a central field of view?
      Dims fovd(itsInputFOV.getVal());
      if (fovd.isEmpty() == false)
        {
          Point2D<int> fovoff( (ret.getWidth() - fovd.w()) / 2, (ret.getHeight() - fovd.h()) / 2);
          if (eye.i != -1 || eye.j != -1) itsRetinalShift -= fovoff;
          ret = crop(ret, fovoff, fovd);
        }
    }

  // if doing foveation, create color pyramid for itsMultiRetina, and foveate the input image, storing the result into
  // itsRetina; otherwise, itsMultiRetina remains unitialized and itsRetina is a straight copy of the input frame:
  const uint fid = itsFoveateInputDepth.getVal();
  if (fid > 0)
    {
      LINFO("Initializing multiretina (depth %d)", fid);
      itsMultiRetina = buildPyrGaussian(ret, 0, fid, 9);

      // create a mask with a disk at current overt fixation:
      Image<byte> mask(ret.getDims(), ZEROS);

      // do we have a valid eye position yet?
      if (eye.isValid())
        {
          drawDisk(mask, eye, itsFoveaRadius.getVal(), byte(255));
          LINFO("Foveating input at (%d, %d)", eye.i, eye.j);
        }
      else
        LINFO("Foveating input with uniform medium blur");

      // use the mask to foveate; if we don't have a SaccadeController but have nevertheless requested
      // itsFoveateInputDepth > 0, the behavior of foveate() is to apply a uniform medium blur to the entire image. If
      // we do have a SaccadeController, this will only happen as long as no overt fixation has been made yet:
      ret = foveate(mask, itsMultiRetina);
    }

  // if we are in a blink, assume that inputs are blanked out:
  if (inBlink && itsBlankBlink.getVal()) {
      LINFO("#### Visual input blanked out while in blink ####");
      ret.clear(PixRGB<byte>(0));
    }

  // derived classes may want to transform the retinal image at this point:
  ret = transform(ret);

  // keep a copy of output in case we want to later save or use it:
  itsOutput = ret;

  // ready to return:
  return ret;
}

// ######################################################################
// ######################################################################
// ########## RetinaConfigurator implementation
// ######################################################################
// ######################################################################
RetinaConfigurator::RetinaConfigurator(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsType(&OPT_RetinaType, this),
  itsRET(new RetinaStub(mgr))
{
  addSubComponent(itsRET);
}

// ######################################################################
RetinaConfigurator::~RetinaConfigurator()
{  }

// ######################################################################
nub::ref<Retina> RetinaConfigurator::getRET() const
{ return itsRET; }

// ######################################################################
void RetinaConfigurator::paramChanged(ModelParamBase* const param,
                                      const bool valueChanged,
                                      ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsType) {
    // let's unregister our existing Retina:
    removeSubComponent(*itsRET);

    // instantiate a Retina of the appropriate type (when the old
    // Retina is destroyed, it will un-export its command-line
    // options):
    if (itsType.getVal().compare("Stub") == 0)          // stub
      itsRET.reset(new RetinaStub(getManager()));
    else if (itsType.getVal().compare("Std") == 0)      // standard
      itsRET.reset(new RetinaStd(getManager()));
    else if (itsType.getVal().compare("CT") == 0)      // Cortical or collicular - Transform 
      itsRET.reset(new RetinaCT(getManager()));
    else
      LFATAL("Unknown Retina type %s", itsType.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:
    addSubComponent(itsRET);

    // tell the controller to export its options:
    itsRET->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected RET of type %s", itsType.getVal().c_str());
  }
}

// ######################################################################
// ######################################################################
// ########## RetinaStub implementation
// ######################################################################
// ######################################################################
RetinaStub::RetinaStub(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
  Retina(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame)
{ }

// ######################################################################
RetinaStub::~RetinaStub()
{ }

// ######################################################################
void RetinaStub::onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  GenericFrame fr = e->frame();
  InputFrame ifr;

  LINFO("#### RETINA ####");
  printf("#### RETINA ####\n\n");

  switch(fr.nativeType()) {
  case GenericFrame::RGBD:
    {
      LINFO("#### RGBD TYPE SELECTED ####");
      const Image<PixRGB<byte> > inimg = fr.asRgb();
      const Image<uint16> dimg = fr.asGrayU16();
      ifr = InputFrame::fromRgbDepth(&inimg, &dimg, q.now());
    }
    break;
  default:
    {
      LINFO("#### RGB TYPE SELECTED ####");

      const Image<PixRGB<byte> > inimg = fr.asRgb();
      ifr = InputFrame::fromRgb(&inimg, q.now());
    }
    break;
  }

  // post an event with our output:
  Rectangle rect(Point2D<int>(0,0), ifr.getDims());
  rutz::shared_ptr<SimEventRetinaImage> eri(new SimEventRetinaImage(this, ifr, rect, Point2D<int>(0,0)));
  q.post(eri);
}

// ######################################################################
// ######################################################################
// ########## RetinaStd implementation
// ######################################################################
// ######################################################################
RetinaStd::RetinaStd(OptionManager& mgr, const std::string& descrName,
                     const std::string& tagName) :
  RetinaAdapter(mgr, descrName, tagName)
{ }

// ######################################################################
RetinaStd::~RetinaStd()
{ }

// ######################################################################
Image<PixRGB<byte> > RetinaStd::transform(const Image<PixRGB<byte> >& image)
{ return image; }

// ######################################################################
// ######################################################################
// ########## RetinaCT implementation
// ######################################################################
// ######################################################################
RetinaCT::RetinaCT(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
  RetinaAdapter(mgr, descrName, tagName), 
  itsSurrFac(&OPT_SpaceVariantDogSize, this),
  itsLevels(&OPT_SpaceVariantChanScales, this),
  itsTransform(new SpaceVariantModule(mgr)),
  itsRgbCache(new PyramidCache<PixRGB<float> >), itsFloatCache(new PyramidCache<float>)
{
  this->addSubComponent(itsTransform);
}

// ######################################################################
RetinaCT::~RetinaCT()
{ }

// ######################################################################
void RetinaCT::start1()
{ 
  getRootObject()->setModelParamVal("UseSpaceVariantBoundary", true, MC_RECURSE | MC_IGNORE_MISSING);
  RetinaAdapter::start1();
}

// ######################################################################
Rectangle RetinaCT::getRawInputRectangle(const Dims& indims, const Dims& outdims) const
{
  Rectangle r(Point2D<int>(0,0), indims);
  return r;
}

// ######################################################################
void RetinaCT::postInputFrame(SimEventQueue& q, InputFrame& ifr) 
{
  ifr.setPyrCacheRgb(itsRgbCache);
  ifr.setPyrCache(itsFloatCache);

  rutz::shared_ptr<SimEventRetinaImage>
    eri(new SimEventRetinaImage(this, ifr, getRawInputRectangle(itsRawInput.getDims(), ifr.getDims()), 
                                getRawToRetinalOffset(),itsTransform->getTransform(),itsTransform->getMapTransform()));
  q.post(eri);
}

// ######################################################################
Image<PixRGB<byte> > RetinaCT::transform(const Image<PixRGB<byte> >& inp)
{
  //transform image and cache
  ImageSet<PixRGB<float> > rgbcache;
  rutz::mutex_lock_class lock;
  if (itsRgbCache->gaussian5.beginSet(inp, &lock))
    {
      const float maxrf = itsTransform->getMaxRf(itsLevels.getVal().getMaxLevel(), itsSurrFac.getVal());
      rgbcache = itsTransform->getScaleSpace(inp, maxrf);
      itsRgbCache->gaussian5.endSet(inp, rgbcache, &lock);
    }

  Image<float> inpl = luminance(inp);
  if (itsFloatCache->gaussian5.beginSet(inpl, &lock))
    {
      ImageSet<float> floatcache(rgbcache.size());
      for (uint ii = 0; ii < floatcache.size(); ++ii)
        floatcache[ii] = luminance(rgbcache[ii]);
      
      itsFloatCache->gaussian5.endSet(inpl, floatcache, &lock);
    }
  
  Image<PixRGB<byte> > output = itsTransform->transform(inp, &rgbcache);
  return output;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
