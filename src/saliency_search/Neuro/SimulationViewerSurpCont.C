/*!@file Neuro/SimulationViewerSurpCont.C entry interface between INVT and ASAC */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerSurpCont.C $
// $Id: SimulationViewerSurpCont.C 13065 2010-03-28 00:01:00Z itti $
//

#include "Neuro/SimulationViewerSurpCont.H"

#include "Neuro/SimulationViewerStd.H"
#include "Channels/ChannelOpts.H"
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
#include "Neuro/SaliencyMap.H"
#include "Neuro/TaskRelevanceMap.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"



// ######################################################################
SimulationViewerSurpCont::SimulationViewerSurpCont(OptionManager& mgr,
                                           const std::string& descrName,
                                           const std::string& tagName) :
  SimulationViewer(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  SIMCALLBACK_INIT(SimEventClockTick),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsMetrics(new SpatialMetrics(mgr)),
  itsDrawDiffParts(&OPT_ASACdrawDiffParts, this),
  itsDrawBetaParts(&OPT_ASACdrawBetaParts, this),
  itsDrawBiasParts(&OPT_ASACdrawBiasParts, this),
  itsDrawSeperableParts(&OPT_ASACdrawSeperableParts, this),
  itsConfigFile(&OPT_ASACconfigFile, this),
  itsLevelSpec(&OPT_LevelSpec, this)
{
  this->addSubComponent(itsMetrics);
  itsInit = false;
}

// ######################################################################
SimulationViewerSurpCont::~SimulationViewerSurpCont()
{  }

// ######################################################################
void SimulationViewerSurpCont::init(const ushort baseSizeX,
                                    const ushort baseSizeY)
{
  std::string configFile = itsConfigFile.getVal();

  itsScaleSurpriseControl.SSCreadConfig(configFile);
  itsScaleSurpriseControl.SSCinit(baseSizeX,baseSizeY);

  itsInit = true;
}

// ######################################################################
void SimulationViewerSurpCont::setBrain(Brain* brain)
{
  LevelSpec              ls = itsLevelSpec.getVal();

  const uint lmin = ls.levMin();
  const uint lmax = ls.levMax();
  const uint dmin = ls.delMin();
  const uint dmax = ls.delMax();
  const uint mlev = ls.mapLevel();
  const uint midx = ls.maxIndex();
  const uint mdep = ls.maxDepth();

  itsScaleSurpriseControl.SSCsetLevelSpecInfo(lmin,lmax,dmin,dmax,
                                              mlev,midx,mdep);

}

// ######################################################################
void SimulationViewerSurpCont::
onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  LINFO("Inputing to Surprise Control raw image");
  // keep a copy of the image
  itsInput = e->frame().asRgb();

  if (!itsInit) init(itsInput.getWidth(), itsInput.getHeight());

  // we have a new input; this will force redrawing various things on
  // the trajectory in case people request it before a new shift of
  // attention or other event occurrs:
  itsHasNewInput = true;
  itsCurrTime    = q.now();
  itsScaleSurpriseControl.SSCinputRawImage(itsInput);
}

// ######################################################################
void SimulationViewerSurpCont::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
  // this seems bogus... is it really necessary?
  itsCurrTime = q.now();
}

// ######################################################################
void SimulationViewerSurpCont::saveResults(const nub::ref<FrameOstream>& ofs)
{
  // update our internal time:
  double msecs = itsCurrTime.msecs();

  LINFO("Running Surprise Control on Sample Input time %f ms",msecs);


  LFATAL("FIXME");
  ////  itsScaleSurpriseControl.SSCprocessFrame(itsBrain);

  LINFO("Saving Surprise Control Output");
  Image<PixRGB<byte> > bimage;

  Image<PixRGB<float> > outImage = itsScaleSurpriseControl.SSCgetFrame();
  bimage = outImage;


  ofs->writeRGB(bimage, "SSC", FrameInfo("ScaleSurpriseControl final image",
                                       SRC_POS));

  Image<PixRGB<float> > diffImage =
    itsScaleSurpriseControl.SSCgetDiffImage(false);
  bimage = diffImage;

  ofs->writeRGB(bimage, "SSC-diff",
                FrameInfo("ScaleSurpriseControl diff image",SRC_POS));

  diffImage = itsScaleSurpriseControl.SSCgetDiffImage(true);
  bimage    = diffImage;

  ofs->writeRGB(bimage, "SSC-diff-norm",
              FrameInfo("ScaleSurpriseControl diff image normalized",SRC_POS));

  if(itsDrawDiffParts.getVal())
  {
    std::vector<Image<PixRGB<float> > > diffParts =
      itsScaleSurpriseControl.SSCgetDiffParts();
    std::vector<Image<PixRGB<float> > >::const_iterator diffPartsItr =
      diffParts.begin();
    ushort type = 0;
    while(diffPartsItr != diffParts.end())
    {
      bimage = *diffPartsItr;
      char name[100];
      if(type == 0)
        sprintf(name,"SSC-diffParts-H1-");
      else if(type == 1)
        sprintf(name,"SSC-diffParts-H2-");
      else if(type == 2)
        sprintf(name,"SSC-diffParts-S-");
      else if(type == 3)
        sprintf(name,"SSC-diffParts-V-");
      else
        sprintf(name,"SSC-diffParts-%d-",type);
      std::string prefix    = name;
      std::string frameInfo = "ScaleSurpriseControl difference ";
      frameInfo             = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));
      ++diffPartsItr; type++;
    }
  }

  if(itsDrawBetaParts.getVal())
  {
    std::vector<Image<float> > betaParts =
      itsScaleSurpriseControl.SSCgetBetaParts(false);
    std::vector<Image<float> >::const_iterator betaPartsItr =
      betaParts.begin();
    ushort type = 0;
    while(betaPartsItr != betaParts.end())
    {
      bimage = *betaPartsItr;
      char name[100];
      sprintf(name,"SSC-betaParts-%s-",sc_channel_name_abv[type].c_str());
      std::string prefix    = name;
      std::string frameInfo = "ScaleSurpriseControl beta ";
      frameInfo             = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));
      ++betaPartsItr; type++;
    }

    betaParts = itsScaleSurpriseControl.SSCgetBetaParts(true);
    betaPartsItr = betaParts.begin();
    type = 0;
    while(betaPartsItr != betaParts.end())
    {
      bimage = *betaPartsItr;
      char name[100];
      sprintf(name,"SSC-betaParts-norm-%s-",sc_channel_name_abv[type].c_str());
      std::string prefix    = name;
      std::string frameInfo = "ScaleSurpriseControl beta norm";
      frameInfo             = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));
      ++betaPartsItr; type++;
    }
  }

  if(itsDrawBiasParts.getVal())
  {
    std::vector<Image<PixRGB<float> > > biasH1;
    std::vector<Image<PixRGB<float> > > biasH2;
    std::vector<Image<PixRGB<float> > > biasS;
    std::vector<Image<PixRGB<float> > > biasV;

    itsScaleSurpriseControl.SSCgetBiasParts(biasH1,biasH2,biasS,biasV);

    std::vector<Image<PixRGB<float> > >::const_iterator biasH1Itr =
      biasH1.begin();
    std::vector<Image<PixRGB<float> > >::const_iterator biasH2Itr =
      biasH2.begin();
    std::vector<Image<PixRGB<float> > >::const_iterator biasSItr  =
      biasS.begin();
    std::vector<Image<PixRGB<float> > >::const_iterator biasVItr  =
      biasV.begin();

    ushort scale = 0;

    while(biasH1Itr != biasH1.end())
    {
      char name[100];

      bimage = *biasH1Itr;
      sprintf(name,"SSC-biasParts-H1-%d-",scale);
      std::string prefix    = name;
      std::string frameInfo = "ScaleSurpriseControl biasH1 ";
      frameInfo             = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));

      bimage = *biasH2Itr;
      sprintf(name,"SSC-biasParts-H2-%d-",scale);
      prefix    = name;
      frameInfo = "ScaleSurpriseControl biasH2 ";
      frameInfo = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));

      bimage = *biasSItr;
      sprintf(name,"SSC-biasParts-S-%d-",scale);
      prefix    = name;
      frameInfo = "ScaleSurpriseControl biasS ";
      frameInfo = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));

      bimage = *biasVItr;
      sprintf(name,"SSC-biasParts-V-%d-",scale);
      prefix    = name;
      frameInfo = "ScaleSurpriseControl biasV ";
      frameInfo = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));
      ++biasH1Itr; ++biasH2Itr; ++biasSItr; ++biasVItr; scale++;
    }
  }

  if(itsDrawSeperableParts.getVal())
  {
    std::vector<Image<PixRGB<float> > > Zimgs;
    std::vector<Image<PixRGB<float> > > Yimgs;

    itsScaleSurpriseControl.SSCgetSeperableParts(Zimgs,Yimgs,false);

    std::vector<Image<PixRGB<float> > >::const_iterator Zitr = Zimgs.begin();
    std::vector<Image<PixRGB<float> > >::const_iterator Yitr = Yimgs.begin();

    ushort scale = 0;

    while(Zitr != Zimgs.end())
    {

      char name[100];
      bimage = *Zitr;
      sprintf(name,"SSC-seperable-parts-Z-%d-",scale);
      std::string prefix    = name;
      std::string frameInfo = "Seperable Parts Z";
      frameInfo             = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));

      bimage = *Yitr;
      sprintf(name,"SSC-seperable-parts-Y-%d-",scale);
      prefix    = name;
      frameInfo = "Seperable Parts Y";
      frameInfo             = frameInfo + prefix;
      ofs->writeRGB(bimage, prefix, FrameInfo(frameInfo,SRC_POS));

      ++Zitr; ++Yitr; scale++;
    }
  }
}

// ######################################################################
void SimulationViewerSurpCont::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  this->save1(e->sinfo());
}

// ######################################################################
void SimulationViewerSurpCont::save1(const ModelComponentSaveInfo& sinfo)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs =
    dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

  saveResults(ofs);
}

// #####################################################################
Image<PixRGB<float> > SimulationViewerSurpCont::getResult()
{
  return itsScaleSurpriseControl.SSCgetFrame();
}

// #####################################################################
std::vector<Image<PixRGB<float> > > SimulationViewerSurpCont::getDiffImages()
{
  return itsScaleSurpriseControl.SSCgetDiffParts();
}

// #####################################################################
std::vector<Image<float> > SimulationViewerSurpCont::getBetaImages()
{
  return itsScaleSurpriseControl.SSCgetBetaParts();
}

// #####################################################################
void SimulationViewerSurpCont::drawTime(Image<PixRGB<byte> >& image)
{
  char txt[20]; sprintf(txt, " %dms ", int(itsCurrTime.msecs() + 0.4999));
  writeText(image, Point2D<int>(0, 0), txt);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
