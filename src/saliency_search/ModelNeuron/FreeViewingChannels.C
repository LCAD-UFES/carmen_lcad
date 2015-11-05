/*!@file ModelNeuron/FreeViewingChannels.C Implementation */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/FreeViewingChannels.C $

#include "ModelNeuron/FreeViewingChannels.H"
#include "Image/ImageSetOps.H"
#include "Image/MathOps.H"


// ######################################################################
// Channel base implementations
// ######################################################################

// ######################################################################
FreeViewingChannel::FreeViewingChannel(OptionManager& mgr, const std::string& descrName, const std::string& tagName)
    : SCTransformModule(mgr, descrName, tagName), 
      takeAbs("FreeViewingChannelTakeABS", this, true),
      takeRectify("FreeViewingChannelTakeRectify", this, false),
      itsSubChanNames({descrName})
{
}

// ######################################################################
FreeViewingChannel::~FreeViewingChannel()
{
}

// ######################################################################
ImageSet<float> FreeViewingChannel::getOutput(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg)
{
  //pre-process
  ImageSet<float> imgset = doPreProcess(lumimg, rgimg, byimg);
  doTransform(imgset);
  doPostProcess(imgset);
  
  return imgset;
}

// ######################################################################
ImageSet<float> FreeViewingChannel::doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg)
{
  ImageSet<float> imgset;
  imgset.push_back(lumimg);
  return imgset;
}

// ######################################################################
void FreeViewingChannel::doTransform(ImageSet<float> & imgset)
{
  //spatially transform the image set
  for (uint ii = 0; ii < imgset.size(); ++ii)
    imgset[ii] = transformDoG(imgset[ii]);
  
  // rectify?
  if (takeRectify.getVal())
    doRectify(imgset);
  
  //take the absolute value?
  else if (takeAbs.getVal())
  {
    for (uint ii = 0; ii < imgset.size(); ++ii)
      imgset[ii] = abs(imgset[ii]);
  }
}

// ######################################################################
void FreeViewingChannel::doTransform(ImageSet<float> & imgset, uint const ii)
{
  imgset[ii] = transformDoG(imgset[ii]);
  
  // rectify?
  if (takeRectify.getVal())
    inplaceRectify(imgset[ii]);
  
  //take the absolute value?
  else if (takeAbs.getVal())
    imgset[ii] = abs(imgset[ii]);
}

// ######################################################################
void FreeViewingChannel::doPostProcess(ImageSet<float> & imgset)
{
}

// ######################################################################
std::vector<std::string> FreeViewingChannel::getSubChanNames() const
{
  return itsSubChanNames;
}

// ######################################################################
uint const FreeViewingChannel::numSubChans() const
{
  return itsSubChanNames.size();
}

// ######################################################################
uint const FreeViewingChannel::getFactor() const
{
  return 1;
}

// ######################################################################
// Spatiotemporal channel base
// ######################################################################
SCSpeChannelBase::SCSpeChannelBase(OptionManager& mgr, uint const scale, const std::string& descrName, const std::string& tagName) 
    : FreeViewingChannel(mgr, descrName, tagName), itsFilters(), itsEngine(), itsScale(scale)
    {
      itsSvDoGSize.setVal(3.2);
      itsRfSlope.setVal(0.1062);
      itsRfExp.setVal(1.0);
      itsRfOffset.setVal(0.75);

      takeAbs.setVal(false);
      takeRectify.setVal(true);

      itsSubChanNames.clear();
    }
  
// ######################################################################
SCSpeChannelBase::~SCSpeChannelBase() 
{ }

// ######################################################################
void SCSpeChannelBase::start1()
{
  FreeViewingChannel::start1();

  uint factor = getFactor();

  if (factor > 1)
  {
    PixPerDeg ppd = getPPD();
    PixPerDeg newppd(ppd.ppdx() / factor, ppd.ppdy() / factor);
    setPPD(newppd);
    LINFO("reducing ppd by factor of %d to %s", factor, toStr(newppd).c_str());
  }
}

// ######################################################################
void SCSpeChannelBase::setEngine(rutz::shared_ptr<SpatioTemporalEnergyEngine> const & engine) 
{
  itsEngine = engine;
}

// ######################################################################
uint const SCSpeChannelBase::getScale() const
{
  return itsScale;
}

// ######################################################################
uint const SCSpeChannelBase::getFactor() const
{
  return (int)pow(2.0F, (float)itsScale);
}
 
// ######################################################################
ImageSet<float> SCSpeChannelBase::doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg)
{
  ImageSet<float> imgset;
  if (itsEngine.is_valid())    
    imgset = itsFilters.getNextOutput(itsEngine->getFilters()[itsEngine->scaleToIndex(itsScale)]);
  
  return imgset;
}

// ######################################################################
// channel implementations
// ######################################################################

// ######################################################################
// luminance
// ######################################################################
SCLuminanceChannel::SCLuminanceChannel(OptionManager& mgr, 
                                       const std::string& descrName, 
                                       const std::string& tagName) :
    FreeViewingChannel(mgr, descrName, tagName)
{ 
  itsSvDoGSize.setVal(6.7);
  itsRfSlope.setVal(0.1062);
  itsRfExp.setVal(1.0);
  itsRfOffset.setVal(0.75);

  takeAbs.setVal(true);
  takeRectify.setVal(false);

  //magnocellular
  //itsRfSlope.setVal(0.0042);
  //itsRfExp.setVal(1.0);
  //itsRfOffset.setVal(0.0519);
}

// ######################################################################
SCLuminanceChannel::~SCLuminanceChannel()
{ }

// ######################################################################
ImageSet<float> SCLuminanceChannel::doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg)
{
  ImageSet<float> imgset;
  imgset.push_back(lumimg);
  return imgset;
}
 
// ######################################################################
// R-G
// ######################################################################
SCRGChannel::SCRGChannel(OptionManager& mgr, 
                         const std::string& descrName, 
                         const std::string& tagName) :
    FreeViewingChannel(mgr, descrName, tagName)
{ 
  itsSvDoGSize.setVal(6.7);
  itsRfSlope.setVal(0.1062);
  itsRfExp.setVal(1.0);
  itsRfOffset.setVal(0.75);

  takeAbs.setVal(true);
  takeRectify.setVal(false);

  //parvocellular
  //itsRfSlope.setVal(0.0002);
  //itsRfExp.setVal(1.7689);
  //itsRfOffset.setVal(0.0252);
}

// ######################################################################
SCRGChannel::~SCRGChannel()
{ }

// ######################################################################
ImageSet<float> SCRGChannel::doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg)
{
  ImageSet<float> imgset;
  imgset.push_back(rgimg);
  return imgset;
}

// ######################################################################
// B-Y
// ######################################################################
SCBYChannel::SCBYChannel(OptionManager& mgr, 
                         const std::string& descrName, 
                         const std::string& tagName) :
    FreeViewingChannel(mgr, descrName, tagName)
{ 
  itsSvDoGSize.setVal(6.7);
  itsRfSlope.setVal(0.1062);
  itsRfExp.setVal(1.0);
  itsRfOffset.setVal(0.75);

  takeAbs.setVal(true);
  takeRectify.setVal(false);

  //parvocellular
  //itsRfSlope.setVal(0.0002);
  //itsRfExp.setVal(1.7689);
  //itsRfOffset.setVal(0.0252);
}

// ######################################################################
SCBYChannel::~SCBYChannel()
{ }

// ######################################################################
ImageSet<float> SCBYChannel::doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg)
{
  ImageSet<float> imgset;
  imgset.push_back(byimg);
  return imgset;
}

// ######################################################################
// edge
// ######################################################################
SCEdgeChannel::SCEdgeChannel(OptionManager& mgr, 
                             uint const scale,
                             const std::string& descrName, 
                             const std::string& tagName) :
    SCSpeChannelBase(mgr, scale, descrName, tagName), 
    itsNumOrientations("EdgeChannelNumOrientations",this, 4)
{   
  takeAbs.setVal(false);
  takeRectify.setVal(true);

}

// ######################################################################
void SCEdgeChannel::start1()
{
  SCSpeChannelBase::start1();

  uint const numThetas = itsNumOrientations.getVal();

  //static all orientations 0-180
  std::vector<Gauss2ndDerivParams> params;
  for (float theta = 0.0F; theta < 180.0F; theta += 180.0F / numThetas)
  {
    params.push_back(Gauss2ndDerivParams(theta, 90.0F));
    itsSubChanNames.push_back(descriptiveName() + "-" + toStr(itsScale) + "-" + toStr(theta));
  }
  itsFilters.setupFilters(params);
}

// ######################################################################
// flicker
// ######################################################################
SCFlickerChannel::SCFlickerChannel(OptionManager& mgr, 
                                   uint const scale,
                                   const std::string& descrName, 
                                   const std::string& tagName) :
    SCSpeChannelBase(mgr, scale, descrName, tagName)
{
  takeAbs.setVal(true);
  takeRectify.setVal(false);   
}

// ######################################################################
void SCFlickerChannel::start1()
{
  SCSpeChannelBase::start1();

  //flicker
  std::vector<Gauss2ndDerivParams> params;
  params.push_back(Gauss2ndDerivParams(0.0F, 0.0F));
  itsSubChanNames.push_back(descriptiveName() + "-" + toStr(itsScale));
  itsFilters.setupFilters(params);
}

// ######################################################################
// motion
// ######################################################################
SCMotionChannel::SCMotionChannel(OptionManager& mgr, 
                                 uint const scale,
                                 const std::string& descrName, 
                                 const std::string& tagName) :
    SCSpeChannelBase(mgr, scale, descrName, tagName),
    itsNumTemporalOrientations("MotionChannelNumTemporalOrientations",this, 3),
    itsNumOrientations("MotionChannelNumOrientations",this, 4),
    itsRelativeMotion("MotionChannelRelativeMotion", this, true)
{   
  takeAbs.setVal(false);
  takeRectify.setVal(true);
}

// ######################################################################
void SCMotionChannel::start1()
{
  SCSpeChannelBase::start1();
  
  const uint numPhis = itsNumTemporalOrientations.getVal()+1;
  const uint numThetas = itsNumOrientations.getVal();
  std::string name = (itsRelativeMotion.getVal()) ? "Rel"+descriptiveName() : descriptiveName();
  
  //motion slow to fast, 0-360 orientations
  std::vector<Gauss2ndDerivParams> params;
  for (float phi = 90.0F - 90.0F/numPhis; phi > 0.0F; phi -= 90.0F/numPhis)
    for (float theta = 0.0F; theta < 360.0F; theta += 180.0F/numThetas)
    {
      params.push_back(Gauss2ndDerivParams(theta, phi));
      itsSubChanNames.push_back(name + "-" + toStr(itsScale) + "-" + toStr(theta) + "-" + toStr(phi));
    }
  itsFilters.setupFilters(params);
}

// ######################################################################
ImageSet<float> SCMotionChannel::doPreProcess(const Image<float> & lumimg, const Image<float> & rgimg, const Image<float> & byimg)
{
  ImageSet<float> ret = SCSpeChannelBase::doPreProcess(lumimg, rgimg, byimg);

  if (itsRelativeMotion.getVal())
  {
    uint const o = itsNumOrientations.getVal();
    uint const t = itsNumTemporalOrientations.getVal();

    for (uint tori = 0; tori < t; ++tori)
      for (uint ori = 0; ori < o; ++ori)
      {
        ret[(tori * o * 2) + ori] -= ret[(tori * o * 2) + ori + o];
        ret[(tori * o * 2) + ori + o] -= ret[(tori * o * 2) + ori];
      }
  }

  return ret;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
