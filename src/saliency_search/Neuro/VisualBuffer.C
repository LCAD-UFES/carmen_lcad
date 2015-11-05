/*!@file Neuro/VisualBuffer.C Grab ausio samples from /dev/dsp */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualBuffer.C $
// $Id: VisualBuffer.C 13065 2010-03-28 00:01:00Z itti $
//

#include "Neuro/VisualBuffer.H"

#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Image/fancynorm.H"
#include "Media/MediaOpts.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/Retina.H"
#include "Simulation/SimEventQueue.H"
#include "Util/log.H"

// ######################################################################
VisualBuffer::VisualBuffer(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  SimModule(mgr, descrName, tagName)
{ }

// ######################################################################
VisualBuffer::~VisualBuffer()
{  }

// ######################################################################
VisualBufferStub::VisualBufferStub(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  VisualBuffer(mgr, descrName, tagName)
{ }

// ######################################################################
VisualBufferStub::~VisualBufferStub()
{  }

// ######################################################################
VisualBufferConfigurator::
VisualBufferConfigurator(OptionManager& mgr,
                         const std::string& descrName,
                         const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsVBtype(&OPT_VisualBufferType, this),
  itsVB(new VisualBufferStub(mgr))
{
  addSubComponent(itsVB);
}

// ######################################################################
VisualBufferConfigurator::~VisualBufferConfigurator()
{ }

// ######################################################################
nub::ref<VisualBuffer> VisualBufferConfigurator::getVB() const
{
  return itsVB;
}

// ######################################################################
void VisualBufferConfigurator::paramChanged(ModelParamBase* const param,
                                            const bool valueChanged,
                                            ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsVBtype) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current VisualBuffer will unexport its
    // command-line options):
    removeSubComponent(*itsVB);

    // instantiate a SM of the appropriate type:
    if (itsVBtype.getVal().compare("None") == 0 ||
        itsVBtype.getVal().compare("Stub") == 0) // no VB
      itsVB.reset(new VisualBufferStub(getManager()));
    else if (itsVBtype.getVal().compare("Std") == 0)      // standard
      itsVB.reset(new VisualBufferStd(getManager()));
    else
      LFATAL("Unknown VB type %s", itsVBtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:
    addSubComponent(itsVB);

    // tell the controller to export its options:
    itsVB->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected VB of type %s", itsVBtype.getVal().c_str());
  }
}

// ######################################################################
VisualBufferStd::VisualBufferStd(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  VisualBuffer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventAttentionGuidanceMapOutput),
  SIMCALLBACK_INIT(SimEventRetinaImage),
  itsFOAradius(&OPT_FOAradius, this),
  itsIgnoreBoring(&OPT_VBignoreBoring, this), // see Neuro/NeuroOpts.{H,C}
  itsObjectBased(&OPT_VBobjectBased, this), // see Neuro/NeuroOpts.{H,C}
  itsBufferDims(&OPT_VBdims, this),  // see Neuro/NeuroOpts.{H,C}
  itsLevelSpec(&OPT_LevelSpec, this), // see Channels/ChannelOpts.{H,C}
  itsInputDims(&OPT_InputFrameDims, this), // see Media/MediaOpts.{H,C}
  itsTimePeriod(&OPT_VBtimePeriod, this),  // see Neuro/NeuroOpts.{H,C}
  itsDecayFactor(&OPT_VBdecayFactor, this),
  itsNormType(&OPT_VBmaxNormType, this), // see Neuro/NeuroOpts.{H,C}
  itsBuffer(), itsSMdims(), itsSaliencyMask(), itsTime(),
  itsLastInteractTime(), itsRetinaOffset(0, 0)
{ }

// ######################################################################
void VisualBufferStd::start1()
{
  // initialize our buffer if we have valid dims for it:
  Dims d = itsBufferDims.getVal();
  if (d.isNonEmpty())
    {
      itsBuffer.resize(d, true);
      LINFO("Using buffer dims of (%d, %d)", d.w(), d.h());
    }
  LINFO("Using internal maxnorm of type %s, decay %f",
        itsNormType.getValString().c_str(), itsDecayFactor.getVal());
}

// ######################################################################
VisualBufferStd::~VisualBufferStd()
{  }

// ######################################################################
void VisualBufferStd::
onSimEventRetinaImage(SimEventQueue& q, rutz::shared_ptr<SimEventRetinaImage>& e)
{
  // just keep track of our retina offset
  itsRetinaOffset = e->offset();
}

// ######################################################################
void VisualBufferStd::
onSimEventAttentionGuidanceMapOutput(SimEventQueue& q, rutz::shared_ptr<SimEventAttentionGuidanceMapOutput>& e)
{
  // grab the agm::
  Image<float> agm = e->agm();

  // if our buffer is not object based, pass it the current saliency
  // map now (the winner location does not matter), otherwise, do it
  // only on every WTA winner:
  if (isObjectBased() == false)
    input(WTAwinner(Point2D<int>(0, 0), q.now(), 0.0, false), agm, Image<byte>());
  else if (SeC<SimEventWTAwinner> e = q.check<SimEventWTAwinner>(this))
    {
      const WTAwinner& win = e->winner();

      // Any output from the ShapeEstimator?
      Image<byte> foaMask;
      if (SeC<SimEventShapeEstimatorOutput> e = q.check<SimEventShapeEstimatorOutput>(this))
        foaMask = Image<byte>(e->smoothMask() * 255.0F);

      // all right, bufferize that stuff!
      input(win, agm, foaMask);
    }

  // evolve our internals one time step:
  this->evolve(q);
}

// ######################################################################
void VisualBufferStd::input(const WTAwinner& win, const Image<float>& sm,
                         const Image<byte>& objmask)
{
  // update our sm dims:
  itsSMdims = sm.getDims();

  // if our buffer size has not been set yet, do it now, and use the
  // size of the sm as our buffer size:
  if (itsBuffer.initialized() == false) {
    itsBuffer.resize(itsSMdims, true);
    LINFO("Using buffer dims of (%d, %d)", itsSMdims.w(), itsSMdims.h());
  }

  // ignore boring attention shifts:
  if (win.boring && itsIgnoreBoring.getVal())
    { LINFO("Ignoring boring attention shift"); return; }

  // Let's build a multiplicative mask that we will apply to the sm
  // and that will determine what saliency information gets
  // transferred into the buffer. We have two modes here, either
  // object-based (only transfer attended object) or map-based
  // (transfer whole map). In addition, we have two sub-cases
  // depending on whether objmask is initialized (then use it to
  // define the object) or not (then use a disk).

  Image<float> maskedsm; // the sm masked by our transfer mask
  Image<float> mask;     // the saliency mask

  if (itsObjectBased.getVal()) {
    // Object-based model. We start by building a float mask at retinal
    // resolution and coordinates:
    Image<byte> maskb;

    if (objmask.initialized()) {
      // build our mask using the object definition passed to us:
      maskb = objmask;              // get the (fuzzy-boundary) object shape
      inplaceLowThresh(maskb, byte(255)); // get tight object boundaries
    } else {
      // build our mask using a disk and distance-based decay:
      maskb.resize(itsInputDims.getVal(), true);  // create empty mask
      drawDisk(maskb, win.p, itsFOAradius.getVal(), byte(255));
    }

    maskb = chamfer34(maskb);  // introduce a distance-based spatial decay
    mask = maskb;              // convert to float; range is 0..255
    mask = rescale(mask, sm.getDims()); // downsize
    mask = binaryReverse(mask, 255.0F);
    mask = squash(mask, 0.0F, 0.0F, 128.0F, 0.25F, 255.0F, 1.0F); // squash
    maskedsm = sm * mask;
  } else {
    // we are not object-based and want to transfer the whole map:
    maskedsm = sm;
    mask.resize(sm.getDims()); mask.clear(1.0F);
  }

  // now let's take the max between our current buffer and our masked
  // sm, after shifting the masked sm to world-centered coordinates.
  // We start by computing the world coords of the top-left corner of
  // the masked sm, at maplevel:
  Point2D<int> tl = retinalToBuffer(Point2D<int>(0, 0));
  Image<float> newbuf(itsBuffer.getDims(), ZEROS);
  pasteImage(newbuf, maskedsm, 0.0F, tl);

  itsBuffer = takeMax(itsBuffer, newbuf);

  // apply one iteration of our internal dynamics:
  internalDynamics();

  // for display purposes, keep a copy of the mask used to transfer saliency:
  itsSaliencyMask = Image<byte>(mask * 255.0F);

  // the internal dynamics of the buffer are taken care of in evolve()
}

// ######################################################################
Image<byte> VisualBufferStd::getSaliencyMask() const
{ return itsSaliencyMask; }

// ######################################################################
void VisualBufferStd::evolve(SimEventQueue& q)
{
  itsTime = q.now();
    // apply one iteration of our internal dynamics if the time has come:
  if (itsTime - itsLastInteractTime >= itsTimePeriod.getVal())
    internalDynamics();  // will update itsLastInteractTime

  // post our buffer

}

// ######################################################################
void VisualBufferStd::inhibit(const Point2D<int>& loc)
{
  Image<float> mask(itsBuffer.getDims(), ZEROS);
  drawDisk(mask, loc, itsFOAradius.getVal() >> itsLevelSpec.getVal().mapLevel(), 1.0F);
  inhibit(mask);
}

// ######################################################################
void VisualBufferStd::inhibit(const Image<float>& mask)
{
  Image<float> inhib = binaryReverse(mask, 1.0F);
  itsBuffer *= inhib;
}

// ######################################################################
Point2D<int> VisualBufferStd::findMostInterestingTarget(const Point2D<int>& p)
{
  // let's start by cutting-off locations below a given fraction of
  // the max activation over the buffer, and binarize the rest:
  Image<float> buf = itsBuffer; float mi, ma; getMinMax(buf, mi, ma);
  Image<byte> bin = makeBinary(buf, ma * 0.25F, 0, 1);

  // now let's find the cluster that is closest to our current eye
  // position. For that, we compute a distance map from a single pixel
  // at current eye position, multiply it by our binary mask, and look
  // for the smallest non-zero value:
  Image<byte> dmap(bin.getDims(), ZEROS);
  dmap.setVal(p, 255); dmap = chamfer34(dmap);
  Image<byte> prod = bin * dmap;
  inplaceReplaceVal(prod, 0, 255); // eliminate zero distances outside our blobs

  Point2D<int> minp; byte minval;
  findMin(prod, minp, minval);

  // minp belongs to our closest cluster. Let's segment that cluster
  // out so that we can compute its center of gravity:
  Image<byte> obj; flood(bin, obj, minp, byte(1), byte(1));
  Image<float> objf = itsBuffer * obj;

  return centroid(objf);
}

// ######################################################################
Point2D<int> VisualBufferStd::findMostInterestingTargetLocMax(const Point2D<int>& p)
{
  // let's start by getting an idea of the range of values in our
  // buffer after we blur it a bit:
  Image<float> buf = lowPass9(itsBuffer);
  float mi, ma; getMinMax(buf, mi, ma);
  float thresh = ma * 0.25F;

  // let's go over the image and find the local max that is above a
  // threshold and closest to our current fixation. The code here is
  // similar to what we have in maxNormalizeStd() to find local maxes,
  // except that here we want to enforce a true local max, not a ridge
  // point (hence we use strict inequalities):
  int w = buf.getWidth(), h = buf.getHeight();
  Point2D<int> best(-1, -1); float bestdist(1.0e10);
  for (int j = 1; j < h - 1; j ++)
    for (int i = 1; i < w - 1; i ++)
      {
        int index = i + w * j;
        float val = buf.getVal(index);
        if (val >= thresh &&                      // strong enough activity
            val > buf.getVal(index - w) &&        // local max
            val > buf.getVal(index + w) &&        // local max
            val > buf.getVal(index - 1) &&        // local max
            val > buf.getVal(index + 1) &&        // local max
            p.distance(Point2D<int>(i, j)) < bestdist) // closest to eye
          { best.i = i; best.j = j; bestdist = p.distance(best); }
      }
  return best;
}

// ######################################################################
Image<float> VisualBufferStd::getBuffer() const
{ return itsBuffer; }

// ######################################################################
Point2D<int> VisualBufferStd::retinalToBuffer(const Point2D<int>& p) const
{
  return retinalToVisualBuffer(p, itsRetinaOffset, itsLevelSpec.getVal().mapLevel(), itsSMdims, itsBuffer.getDims());
}

// ######################################################################
Point2D<int> VisualBufferStd::bufferToRetinal(const Point2D<int>& p) const
{
  return visualBufferToRetinal(p, itsRetinaOffset, itsLevelSpec.getVal().mapLevel(), itsSMdims, itsBuffer.getDims());
}

// ######################################################################
bool VisualBufferStd::isObjectBased() const
{ return itsObjectBased.getVal(); }

// ######################################################################
void VisualBufferStd::internalDynamics()
{
  float mi, ma; getMinMax(itsBuffer, mi, ma);
  itsBuffer = maxNormalize(itsBuffer, 0.0F, 0.0F, itsNormType.getVal());
  inplaceNormalize(itsBuffer, mi, ma);
  if (itsDecayFactor.getVal() != 1.0F) itsBuffer *= itsDecayFactor.getVal();
  itsLastInteractTime = itsTime;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
