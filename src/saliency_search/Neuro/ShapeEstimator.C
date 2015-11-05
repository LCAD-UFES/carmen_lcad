/*!@file Neuro/ShapeEstimator.C Estimate the shape/size 
  of an attended object */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/ShapeEstimator.C $
// $Id: ShapeEstimator.C 14857 2011-07-26 01:14:21Z siagian $
//

#include "Neuro/ShapeEstimator.H"

#include "Component/ModelOptionDef.H"
#include "Component/GlobalOpts.H"
#include "Channels/ChannelMaps.H"
#include "Component/OptionManager.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ImageSet.H"
#include "Image/MathOps.H"
#include "Image/MorphOps.H"     // for openImg()
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "Image/Transforms.H"
#include "Media/MediaOpts.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/VisualCortex.H"
#include "Neuro/SpatialMetrics.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "Util/TextLog.H"

#include "Raster/Raster.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Util/Timer.H"

#include <cstdio>

#define SIGMA 20
#define CLAMP 0.4f

const ModelOptionDef OPT_ShapeEstimatorUseLargeNeigh =
  { MODOPT_FLAG, "ShapeEstimatorUseLargeNeigh", &MOC_BRAIN, OPTEXP_CORE,
    "Use a larger 3x3 neighborhood to track down a local maximum across "
    "scales when true, otherwise use a 2x2 neighborhood.",
    "shape-estim-largeneigh", '\0', "", "true" };

// ######################################################################
ShapeEstimator::ShapeEstimator(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName,
                               const nub::soft_ref<VisualCortex> vcx) :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventWTAwinner),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsMetrics(new SpatialMetrics(mgr)),
  itsLogFile(&OPT_TextLogFile, this),
  itsMode(&OPT_ShapeEstimatorMode, this),
  itsSmMethod(&OPT_ShapeEstimatorSmoothMethod, this),
  itsSaveObjMask(&OPT_BrainSaveObjMask, this), // Neuro/NeuroOpts.{H,C}
  itsUseLargeNeigh(&OPT_ShapeEstimatorUseLargeNeigh, this),
  structEl(), itsSmoothMask(), itsCumMask(),
  itsSalientRegionSegmenter(new SalientRegionSegmenter()),
  itsCenterSurroundHistogramSegmenter(new CenterSurroundHistogramSegmenter())
{
  this->addSubComponent(itsMetrics);

  // itsInputImageStem = mgr.getOptionValString(&OPT_InputFrameSource);
  // Raster::waitForKey();
}


// ######################################################################
void ShapeEstimator::reset1()
{
  // reset some stuff for ShapeEstimator
  itsSmoothMask.freeMem();
  itsCumMask.freeMem();

  // propagate to our base class:
  ModelComponent::reset1();
}

// ######################################################################
void ShapeEstimator::onSimEventWTAwinner
(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  // nothing to do?
  if (itsMode.getVal() == SEMnone) return;

  // if we use MT features map
  if(itsMode.getVal() == SEMMTfeatureMap)
    postMotionSimEventShapeEstimatorOutput(q,e);

  // if we use the Contour Boundary map
  else if(itsMode.getVal() == SEMContourBoundaryMap)
    postContourBoundarySimEventShapeEstimatorOutput(q,e);

  // if we use the variable center surround approach
  else if(itsMode.getVal() == SEMCSHistogram)
    postCSHistogramSimEventShapeEstimatorOutput(q,e);

  // else we either use saliency, conspicuity, or feature map
  else postSaliencyFeatureSimEventShapeEstimatorOutput(q,e);
}

// ######################################################################
void ShapeEstimator::postSaliencyFeatureSimEventShapeEstimatorOutput
(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  // any new covert attention shift?
  const Point2D<int> winner = e->winner().p; // in input coordinates

  // in the following, we are going to try to find good values for
  // winmap (the winning map or submap), and winloc (location of
  // winner in winmap's coordinates):
  NamedImage<float> winmap; Point2D<int> winloc(-1, -1);

  // grab the latest retina image, just to know its dims:
  Dims indims;
  if (SeC<SimEventRetinaImage> ee = 
      q.check<SimEventRetinaImage>(this, SEQ_ANY)) 
    indims = ee->frame().getDims();
  else 
    LFATAL("Oooops, we have a WTA winner "
           "but no retina image in the SimEventQueue");

  // invalidate any previous object mask:
  itsSmoothMask.freeMem();

  // grab all the VisualCortex maps:
  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  // in case we use saliency map, just store the SM as the winning map:
  if (itsMode.getVal() == SEMsaliencyMap)
    {
      winmap = chm->getMap();
      locateLocalMax(winmap, winner, indims, winloc);
    }
  else
    {
      // loop over the channels:
      float mx = 0.0F; rutz::shared_ptr<ChannelMaps> chan;

      for (uint i = 0; i < chm->numSubchans(); ++i)
        {
          rutz::shared_ptr<ChannelMaps> ch = chm->subChanMaps(i);
          NamedImage<float> output = ch->getMap();

          if (output.initialized())
            {
              Point2D<int> w;
              const float val = locateLocalMax(output, winner, indims, w);
              LDEBUG("Examining: %s, val[%3d %3d] = %g", 
                     output.name().c_str(), w.i, w.j, val);

              // do we have a new max?
              if (val > mx) 
                { winmap = output; mx = val; chan = ch; winloc = w; }
            }
        }

      // we did not find a max here? -> resort back to saliency map:
      if (mx == 0.0F)
        {
          winmap = chm->getMap();
          locateLocalMax(winmap, winner, indims, winloc);
        }
      else
        {
          // ok, we found a max in one of our channel conspicuity
          // maps.  If we are using the conspicuity map mode, we
          // are done. Otherwise, let's go one level further,
          // looking at individual feature maps:

          // NOTE: this code should be truly recursive, instead here
          // we are going to flatten the channel hierarchy...

          if (itsMode.getVal() != SEMconspicuityMap)
            {
              // loop over the submaps of the winning channel:
              mx = 0.0F; Point2D<int> winloc2(0, 0); NamedImage<float> winmap2;
              for (uint i = 0; i < chan->numSubmaps(); ++i)
                {
                  NamedImage<float> submap = chan->getSubmap(i);
                  if (submap.initialized())
                    {
                      Point2D<int> w;
                      const float val = 
                        locateLocalMax(submap, winloc, winmap.getDims(), w);
                      LDEBUG("Examining: %s, val[%3d %3d] = %g", 
                             submap.name().c_str(), w.i, w.j, val);

                      // do we have a new max?
                      if (val > mx) { winmap2 = submap; mx = val; winloc2 = w; }
                    }
                }

              // did not find a max here? then just keep our old
              // values for winmap, winloc, and winlabel (which
              // are at the conspicuity map level). Otherwise,
              // update to the feature map level:
              if (mx > 0.0F) { winmap = winmap2; winloc = winloc2; }
            }
        }
    }

  // post the results
  postSimEventShapeEstimatorOutput(winmap, winloc, winner, indims, q);
}

// ######################################################################
float ShapeEstimator::locateLocalMax(const Image<float>& submap,
                                     const Point2D<int>& winner,
                                     const Dims& fulldims,
                                     Point2D<int>& winloc,
                                     int mini, int maxi)
{
  // Scale down the winner coordinates, from input image dims to the
  // dims of submap, rounding down:
  winloc.i = (winner.i * submap.getWidth()) / fulldims.w();
  winloc.j = (winner.j * submap.getHeight()) / fulldims.h();

  // Explore the local neighborhood. Note that here we are only
  // interested in positive values:
  float maxval = 0.0F; std::vector<Point2D<int> > surr; 

  if (itsUseLargeNeigh.getVal()) mini = -1;

  for (int j = mini; j <= maxi; ++j)
    for (int i = mini; i <= maxi; ++i)
      surr.push_back(winloc + Point2D<int>(i, j));

  for (uint i = 0; i < surr.size(); ++i)
    {
      surr[i].clampToDims(submap.getDims());
      const float val = submap.getVal(surr[i]);
      if (val > maxval) { winloc = surr[i]; maxval = val; }
    }
  return maxval;
}

// ######################################################################
void ShapeEstimator::postMotionSimEventShapeEstimatorOutput
(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  // any new covert attention shift?
  const Point2D<int> winner = e->winner().p; // in input coordinates
  LINFO("SEMMTfeatureMap: winner: %d %d", winner.i, winner.j);

  // in the following, we are going to try to find good values for
  // winmap (the winning map or submap), and winloc (location of
  // winner in winmap's coordinates):
  NamedImage<float> winmap; Point2D<int> winloc(-1, -1);

  // grab the latest retina image, just to know its dims:
  Dims indims;
  if(SeC<SimEventRetinaImage> ee = 
     q.check<SimEventRetinaImage>(this, SEQ_ANY)) 
    indims = ee->frame().getDims();
  else LFATAL("Oooops, we have a WTA winner "
              "but no retina image in the SimEventQueue");

  SeC<SimEventRetinaImage> ee = q.check<SimEventRetinaImage>(this, SEQ_ANY);
  Image<PixRGB<byte> > image = ee->frame().colorByte();

  //Raster::waitForKey();
  uint width  = image.getWidth();
  uint height = image.getHeight();
  //uint scale  = 16;
  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(width*2,height*1), 0, 0, "ShapeEstObj"));
  itsWin->setDims(Dims(width*2, height*1));

  Image<PixRGB<byte> > disp = image;
  drawCross(disp, winner, PixRGB<byte>(255,0,0), 10, 2);
  itsWin->drawImage(disp,0,0);
  LINFO("Input Image: %d %d", image.getWidth(), image.getHeight());
  Raster::waitForKey();

  // invalidate any previous object mask:
  itsSmoothMask.freeMem();

  // grab all the VisualCortex maps:
  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();


  // Image<float> dwinmap = zoomXY(winmap,scale);
  // inplaceNormalize(dwinmap, 0.0F, 1.0F);
  // itsWin->drawImage(dwinmap,0,0);
  // LINFO("Saliency Map");
  // Raster::waitForKey();












  // loop over the channels:
  float mx = 0.0F; rutz::shared_ptr<ChannelMaps> chan;
  uint jmax = 0;

  // get the MotionSpatioTemporalChannel
  for (uint i = 0; i < chm->numSubchans(); ++i)
    {
      rutz::shared_ptr<ChannelMaps> ch = chm->subChanMaps(i);
      NamedImage<float> output = ch->getMap();

      // Image<float> doutput = zoomXY(output,16);
      // inplaceNormalize(doutput, 0.0F, 1.0F);
      // itsWin->drawImage(doutput,0,0);
      // LINFO("CMAP[%3d]: %s", i, output.name().c_str());
      // Raster::waitForKey(); 

      if(output.name().find(std::string("motionSpatioTemporal")) != 
         std::string::npos)
        {    
          // go through each sub channels
          uint nSubchans = ch->numSubchans();
          LINFO("num subchans: %d", nSubchans);
      
          int cDir = -1;
          for(uint j = 0; j < nSubchans; j++)
            {
              rutz::shared_ptr<ChannelMaps> sch = ch->subChanMaps(j);
              NamedImage<float> rawcsmap = sch->getRawCSmap(0);    

              std::string rcsname = rawcsmap.name();
              int fupos = rcsname.find_first_of('_') +1;
              std::string rcsname2 = rcsname.substr(fupos);
              int supos = rcsname2.find_first_of('_')+ fupos;
              std::string number = rcsname.substr(fupos,supos-fupos).c_str();
              //LINFO("%s %s -> %s", 
              //      rcsname.c_str(), rcsname2.c_str(), number.c_str());
              int dir = atoi(number.c_str());
              
              if(cDir < dir) 
                {
                  cDir = dir;

                  Image<float> drawcsmap = zoomXY(rawcsmap, 4);
                  inplaceNormalize(drawcsmap, 0.0F, 1.0F);
                  itsWin->drawImage(drawcsmap, 0,0);
                  LINFO("RCSMAP[%3d][%3d]:[%d %d]:  %s", 
                        i,j, rawcsmap.getWidth(), rawcsmap.getHeight(),
                        rawcsmap.name().c_str());
                  Raster::waitForKey(); 

                  // check to see if it's a higher saliency value
                  if (output.initialized())
                    {
                      Point2D<int> w;
                      const float val = 
                        locateLocalMax(rawcsmap, winner, indims, w, -1, 1);
                      LINFO("Examining: %s, val = %g", 
                            rawcsmap.name().c_str(), val);
                      
                      // do we have a new max?
                      if (val > mx) 
                        {
                          LINFO("j: %d", j); jmax = j;
                          winmap = rawcsmap; mx = val; chan = ch; winloc = w; 
                        }
                    }
                }
            }
        }
    }

  // if we did not have a MotionSpatioTemporalChannel
  // we did not find a max here? -> resort back to saliency map:
  if (mx == 0.0F)
    {
      winmap = chm->getMap();
      locateLocalMax(winmap, winner, indims, winloc);
      LINFO("did not find a max in the map");
    }

  LINFO("jmax: %d mx: %f", jmax, mx);
  postSimEventShapeEstimatorOutput(winmap, winloc, winner, indims, q);

  Raster::waitForKey();
}

// ######################################################################
void ShapeEstimator::postContourBoundarySimEventShapeEstimatorOutput
(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  Timer t(1000000); t.reset();

  // any new covert attention shift?
  const Point2D<int> winner = e->winner().p; // in input coordinates

  // in the following, we are going to try to find good values for
  // winmap (the winning map or submap), and winloc (location of
  // winner in winmap's coordinates):
  NamedImage<float> winmap; Point2D<int> winloc(-1, -1);

  // grab the latest retina image, just to know its dims:
  Dims indims;
  if (SeC<SimEventRetinaImage> ee = 
      q.check<SimEventRetinaImage>(this, SEQ_ANY)) 
    indims = ee->frame().getDims();
  else LFATAL("Oooops, we have a WTA winner "
              "but no retina image in the SimEventQueue");

  // invalidate any previous object mask:
  itsSmoothMask.freeMem();

  // grab all the VisualCortex maps:
  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  // get the image and saliency map
  SeC<SimEventRetinaImage> ee = 
    q.check<SimEventRetinaImage>(this, SEQ_ANY);
  Image<PixRGB<byte> > image = ee->frame().colorByte();
  winmap = chm->getMap();

  uint width  = image.getWidth();
  uint height = image.getHeight();
  uint scale  = 16;
  if(itsWin.is_invalid())
    itsWin.reset
      (new XWinManaged(Dims(width*3,height*2), 0, 0, "CB_ShapeEstObj"));
  else
    itsWin->setDims(Dims(width*3, height*2));
  
  Image<PixRGB<byte> > disp(width*3, height*2, ZEROS);
  inplacePaste(disp, image, Point2D<int>(0,0));
  drawCross(disp, winner, PixRGB<byte>(255,0,0), 10, 1);

  Image<float> dwinmap = zoomXY(winmap,scale);  
  inplaceNormalize(dwinmap, 0.0F, 255.0F);
  Image<byte> dbwinmap(dwinmap);  
  Image<PixRGB<byte> > dcwinmap = toRGB(dbwinmap);
  inplacePaste(disp, dcwinmap, Point2D<int>(width,0));
  drawCross(disp, Point2D<int>(width,0)+winner, 
            PixRGB<byte>(255,0,0), 10, 1);

  LINFO("winner: %d %d in [%3d %3d]", 
        winner.i, winner.j, image.getWidth(), image.getHeight());
  itsWin->drawImage(disp,0,0);
  Raster::waitForKey();

  // FIXXX: need to figure out what to do in a video
  //        right now if a new image is passed in, 
  //        it's always ignored THIS IS BAD!!!!!!!!!!!!!!
  if(!itsSalientRegionSegmenter->getImage().initialized())
    {
      itsSalientRegionSegmenter->setImage(image, itsInputImageStem);
      LINFO("initial image");
    }
  else LINFO("still the same image");

  // get the winning feature map name
  std::string winMapName = 
    getWinningFeatureMapName(winner, indims, chm);

  Image<float> smap = winmap;
  inplaceNormalize(smap, 0.0F, 255.0F);

  // correct the salient point location if needed
  Point2D<int> cwinner = 
    correctSalientPointLocation(winner, winMapName, smap);  
  winloc.i = (cwinner.i * winmap.getWidth() ) / width;
  winloc.j = (cwinner.j * winmap.getHeight()) / height;

  // get the salient region 
  Image<float> segObjImage = 
    itsSalientRegionSegmenter->getSalientRegion(cwinner);
  if(!segObjImage.initialized()) return;

  // winning map is just the downsized object mask 
  uint smWidth  = winmap.getWidth();
  uint smHeight = winmap.getHeight();
  winmap = downSize(segObjImage, Dims(smWidth, smHeight));
  //winmap = rescale (segObjImage, Dims(smWidth, smHeight));
  winmap.setName(winMapName);  

  postSimEventShapeEstimatorOutput(winmap, winloc, cwinner, indims, q);

  uint64 t2 = t.get();
  LINFO("time: %f", t2/1000.0);
}

// ######################################################################
std::string ShapeEstimator::getWinningFeatureMapName
(Point2D<int> winner, Dims indims, rutz::shared_ptr<ChannelMaps> chm)
{
  NamedImage<float> winmap; Point2D<int> winloc(-1, -1);

  // loop over the channels:
  float mx = 0.0F; rutz::shared_ptr<ChannelMaps> chan;
  
  for (uint i = 0; i < chm->numSubchans(); ++i)
    {
      rutz::shared_ptr<ChannelMaps> ch = chm->subChanMaps(i);
      NamedImage<float> output = ch->getMap();
      if (output.initialized())
        {
          Point2D<int> w;
          const float val = locateLocalMax(output, winner, indims, w);
          LDEBUG("Examining: %s, val[%3d %3d] = %g", 
                 output.name().c_str(), w.i, w.j, val);

          // do we have a new max?
          if (val > mx) 
            { winmap = output; mx = val; chan = ch; winloc = w; }
        }
    }

  // we did not find a max here? -> resort back to saliency map:
  if (mx == 0.0F)
    {
      winmap = chm->getMap();
      locateLocalMax(winmap, winner, indims, winloc);
    }
  else
    {
      // loop over the submaps of the winning channel:
      mx = 0.0F; Point2D<int> winloc2(0, 0); NamedImage<float> winmap2;
      for (uint i = 0; i < chan->numSubmaps(); ++i)
        {
          NamedImage<float> submap = chan->getSubmap(i);
          if (submap.initialized())
            {
              Point2D<int> w;
              const float val = 
                locateLocalMax(submap, winloc, winmap.getDims(), w);
              LDEBUG("Examining: %s, val[%3d %3d] = %g", 
                     submap.name().c_str(), w.i, w.j, val);
              
              // do we have a new max?
              if (val > mx) { winmap2 = submap; mx = val; winloc2 = w; }
            }
        }

      // did not find a max here? then just keep our old
      // values for winmap, winloc, and winlabel (which
      // are at the conspicuity map level). Otherwise,
      // update to the feature map level:
      if (mx > 0.0F) { winmap = winmap2; winloc = winloc2; }
    }

  return winmap.name();
}

// ######################################################################
Point2D<int> ShapeEstimator::correctSalientPointLocation
(Point2D<int> pt, std::string name, Image<float> salMap)
{
  uint boundary_step_size =
    itsSalientRegionSegmenter->getBoundaryStepSize();
  uint hstep = boundary_step_size/2;
  Image<int> cRAM = 
    itsSalientRegionSegmenter->getCurrentRegionAssignmentMap(); 

  // Round it to the closest 8x8 grid location
  Point2D<int> ptCorrGrid, ptCorr;
  ptCorrGrid.i = ((pt.i+hstep)/boundary_step_size); 
  ptCorrGrid.j = ((pt.j+hstep)/boundary_step_size); 
  ptCorr = ptCorrGrid * boundary_step_size; 

  // get channel name
  LINFO("winmap name: %s", name.c_str());
  std::string::size_type fcpos = name.find_first_of(':');
  std::string chanName, sChanName, scale;
  if(fcpos != std::string::npos)
    {
      std::string temp = name.substr(fcpos+1);
      LINFO("temp: %s", temp.c_str());

      std::string::size_type fcpos2c = temp.find_first_of(':');
      std::string::size_type fcpos2p = temp.find_first_of('(');
      std::string::size_type fcpos2e = temp.find_first_of(')');

      // channel has multiple sub-channels
      if(fcpos2c != std::string::npos && fcpos2p != std::string::npos) 
        {
          chanName  = temp.substr(0, fcpos2c);
          sChanName = temp.substr(fcpos2c+1, fcpos2p - fcpos2c - 1);
          scale     = temp.substr(fcpos2p+1, fcpos2e - fcpos2p - 1);
        }

      // channel has single sub-channel
      else if(fcpos2p != std::string::npos)
        {
          chanName = temp.substr(0, fcpos2p);
          scale    = temp.substr(fcpos2p+1, fcpos2e - fcpos2p - 1);
        } 
      // if no ':', it's conspicuity map but no more info
      // : no adjustment
    }
  // if no ':', it's saliency map : no adjustment

  LINFO("chanName: %s, schanName: %s scale: %s", 
        chanName.c_str(), sChanName.c_str(), scale.c_str());

  Point2D<int> delta(0,0);

  // from salient point get most appropriate salient region 
  // for orientation saliency the border is salient, 
  // so we move the point perpendicular to the angle 
  if(!chanName.compare("orientation"))
    {
      // FIXXX: assume there's only 4 direction
      std::string::size_type fupos = sChanName.find_first_of('_');
      uint dir = atoi(sChanName.substr(fupos+1).c_str());

      Point2D<int> d1, d2;
      if(dir == 0)
        { d1 = Point2D<int>( 0, -1); d2 = Point2D<int>( 0,  1); }
      else if(dir == 1)
        { d1 = Point2D<int>( 1, -1); d2 = Point2D<int>(-1,  1); }
      else if(dir == 2)
        { d1 = Point2D<int>( 1,  0); d2 = Point2D<int>(-1,  0); }
      else if(dir == 3)
        { d1 = Point2D<int>( 1,  1); d2 = Point2D<int>(-1, -1); }
      Point2D<int> t1 = ptCorrGrid + d1;
      Point2D<int> t2 = ptCorrGrid + d2;

      // if points are in bounds and not yet assigned
      bool vt1 = (cRAM.coordsOk(t1) && (t1.i != 0 || t1.j != 0) &&
                  cRAM.getVal(t1) == -1);
      bool vt2 = (cRAM.coordsOk(t2) && (t2.i != 0 || t2.j != 0) &&
                  cRAM.getVal(t2) == -1);
      if(vt1)
        {
          if(!vt2) delta = d1;
          else
            {              
              // go to side with higher saliency
              // we might hit background and not object
              // either way we are reducing 
              // the number of unassigned regions
              float wCRAM = float(cRAM.getWidth());
              float wSMAP = float(salMap.getWidth());
              float tempN = wSMAP/wCRAM;
              float val1 = salMap.getValInterp(t1.i*tempN, t1.j*tempN);
              float val2 = salMap.getValInterp(t2.i*tempN, t2.j*tempN);
              if(val1 > val2) delta = d1; else delta = d2;

              LINFO("t1[%3d %3d] t2[%3d %3d] "
                    "w %f w %f --> %f [%f %f] [%f %f] "
                    "v1:%f v2:%f", 
                    t1.i, t1.j, t2.i, t2.j, wCRAM, wSMAP, tempN,
                    t1.i*tempN, t1.j*tempN, t2.i*tempN, t2.j*tempN,
                    val1, val2);

            }
        } else if(vt2) delta = d2;

      // if both sides are assigned, 
      // we are in the crack between two salient regions
      // should have been corrected 
    }

  LINFO("pt: %3d %3d: %3d %3d", 
        pt.i, pt.j, ptCorr.i, ptCorr.j);

  // FIXXX: need to incorporate scale???
  ptCorrGrid += delta;
  ptCorr      = ptCorrGrid * BOUNDARY_STEP_SIZE;  
  LINFO("delta: %3d %3d --> corrected point: %d %d", 
        delta.i, delta.j, ptCorr.i, ptCorr.j);

  return ptCorr;
}

// ######################################################################
void ShapeEstimator::postCSHistogramSimEventShapeEstimatorOutput
(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  Timer t(1000000); t.reset();

  // any new covert attention shift?
  const Point2D<int> winner = e->winner().p; // in input coordinates

  // in the following, we are going to try to find good values for
  // winmap (the winning map or submap), and winloc (location of
  // winner in winmap's coordinates):
  NamedImage<float> winmap; Point2D<int> winloc(-1, -1);

  // grab the latest retina image, just to know its dims:
  Dims indims;
  if (SeC<SimEventRetinaImage> ee = 
      q.check<SimEventRetinaImage>(this, SEQ_ANY)) 
    indims = ee->frame().getDims();
  else LFATAL("Oooops, we have a WTA winner "
              "but no retina image in the SimEventQueue");

  // invalidate any previous object mask:
  itsSmoothMask.freeMem();

  // grab all the VisualCortex maps:
  rutz::shared_ptr<SimReqVCXmaps> vcxm(new SimReqVCXmaps(this));
  q.request(vcxm); // VisualCortex is now filling-in the maps...
  rutz::shared_ptr<ChannelMaps> chm = vcxm->channelmaps();

  // get the image and saliency map
  SeC<SimEventRetinaImage> ee = 
    q.check<SimEventRetinaImage>(this, SEQ_ANY);
  Image<PixRGB<byte> > image = ee->frame().colorByte();
  winmap = chm->getMap();

  // get 600x480 cropped image
  int width  = image.getWidth();
  int height = image.getHeight();
  int cwidth  = 600;
  int cheight = 480; 

  Point2D<int> rwinner(winner.i/10 * 10, winner.j/10 * 10); 
  int tt = rwinner.j - cheight/2; 
  if(tt < 0) tt = 0;
  int bb = tt + cheight; 
  if (bb >= height) 
    { 
      bb = height; 
      tt = bb - cheight; if(tt < 0) tt = 0;
    }

  int ll = rwinner.i - cwidth/2; 
  if(ll < 0) ll = 0;
  int rr = ll + cwidth; 
  if (rr >= width) 
    { 
      rr = width; 
      ll = rr - cwidth; if(ll < 0) ll = 0;
    }

  // crop the image properly 
  Rectangle r = Rectangle::tlbrO(tt,ll,bb,rr);

  Image<PixRGB<byte> > cimage = crop(image,r);  
  Point2D<int> offset(ll,tt);
  Point2D<int> cwinner = winner - offset;

  winloc.i = (winner.i * winmap.getWidth() ) / width;
  winloc.j = (winner.j * winmap.getHeight()) / height;

//   if(itsWin.is_invalid())
//     itsWin.reset(new XWinManaged(Dims(cwidth*2, cheight*1), 0, 0, "CSH_SE"));
//   else itsWin->setDims(Dims(cwidth*2, cheight*1));
// 
//   Image<PixRGB<byte> > cdisp(cwidth*2, cheight*1, ZEROS);
//   inplacePaste(cdisp, cimage, Point2D<int>(0,0));
//   drawCross(cdisp, cwinner, PixRGB<byte>(255,0,0), 10, 1);
//   // uint scale  = 16;
//   // Image<float> dwinmap = zoomXY(winmap,scale);  
//   // inplaceNormalize(dwinmap, 0.0F, 255.0F);
//   // Image<byte> dbwinmap(dwinmap);  
//   // Image<PixRGB<byte> > dcwinmap = toRGB(dbwinmap);
//   // inplacePaste(disp, dcwinmap, Point2D<int>(width,0));
//   // drawCross(disp, Point2D<int>(width,0)+winner, 
//   //           PixRGB<byte>(255,0,0), 10, 1);
//   LINFO("cwinner: %d %d in [%3d %3d]", 
//         cwinner.i, cwinner.j, cimage.getWidth(), cimage.getHeight());
//   itsWin->drawImage(cdisp,0,0);
//   Raster::waitForKey();

  // get the salient region 
  t.reset();
  itsCenterSurroundHistogramSegmenter->setImage(cimage);
  Image<float> tSegObjImage = 
    itsCenterSurroundHistogramSegmenter->getSalientRegion(cwinner);
  LINFO("time: %f", t.get()/1000.0F);
  if(!tSegObjImage.initialized()) return;

  Image<float> segObjImage(width, height, ZEROS);
  inplacePaste(segObjImage, tSegObjImage, offset);

  // winning map is just the downsized object mask 
  uint smWidth  = winmap.getWidth();
  uint smHeight = winmap.getHeight();
  winmap = downSize(segObjImage, Dims(smWidth, smHeight));
  //winmap = rescale (segObjImage, Dims(smWidth, smHeight));
  winmap.setName(std::string("CSHistogramMap"));  

  postSimEventShapeEstimatorOutput(winmap, winloc, winner, indims, q);

}

// ######################################################################
void ShapeEstimator::postSimEventShapeEstimatorOutput
(NamedImage<float> winmap, Point2D<int> winloc, 
 Point2D<int> winner, Dims indims, SimEventQueue& q)
{
  // since we had a winner to start with, one of the detections
  // should have worked... So we have winmap, winlabel, and winloc
  // all ready to go:
  ASSERT(winmap.initialized());

  // Now compute the objectmask:
  Image<byte> objmask;
  Image<float> winmapnormalized = winmap;
  inplaceNormalize(winmapnormalized, 0.0F, 1.0F);

  // if we found a good seed point, use it to segment the
  // object. Otherwise, we failed, so let's just return a disk:
  const bool goodseed = (winmap.getVal(winloc) > 0.0F);
  if (goodseed)
    {
      LDEBUG("Segmenting object around (%d, %d) in %s.", 
             winner.i, winner.j, winmap.name().c_str());
      objmask = segmentObjectClean(winmapnormalized, winloc);
    }
  else
    {
      LDEBUG("Drawing disk object around (%d, %d) in %s.", 
             winner.i, winner.j, winmap.name().c_str());
      objmask.resize(winmap.getDims(), true);
      drawDisk(objmask, winloc, 
               (itsMetrics->getFOAradius() * objmask.getWidth()) / indims.w(), 
               byte(255));
    }

  // all right, compute the smooth mask:
  switch(itsSmMethod.getVal())
    {
    case SESMgaussian:
      {
        Image<float> temp = scaleBlock(objmask, indims);
        itsSmoothMask = convGauss<float>(temp, SIGMA, SIGMA, 5);
        inplaceNormalize(itsSmoothMask, 0.0F, 3.0F);
        inplaceClamp(itsSmoothMask, 0.0F, 1.0F);
        break;
      }

    case SESMchamfer:
      {
        if (structEl.initialized() == false)
          {
            //create the structuring element for the opening
            const int ss = 8;
            structEl = Image<byte>(ss+ss,ss+ss, ZEROS);
            drawDisk(structEl, Point2D<int>(ss,ss), ss, byte(1));
          }

        const byte cutoff = 100;
        Image<byte> temp = scaleBlock(objmask, indims);
        temp = chamfer34(openImg(temp, structEl), cutoff);
        itsSmoothMask = binaryReverse(Image<float>(temp), 255.0F);
        inplaceNormalize(itsSmoothMask, 0.0F, 1.0F);
        break;
      }

    case SESMnone:
      {
        itsSmoothMask = scaleBlock(objmask, indims) / 255.0F;
        break;
      }

    default: LFATAL("Unknown Smoothing Method");
    }

  // update the cumulative smooth mask:
  if (itsCumMask.initialized()) itsCumMask = takeMax(itsCumMask, itsSmoothMask);
  else itsCumMask = itsSmoothMask;

  // finally compute the IOR mask. Basically, we want to take the
  // dot product between the binary object mask and the winning
  // map, so that we will inhibit stronger the locations which are
  // more active. However, let's add some default level of
  // inhibition inside the whole object, so that every location
  // within the object gets suppressed at least some.  Also note
  // that we need to make sure the WTA winner will actually get
  // inhibited, so that attention can shift. Indeed, sometimes the
  // max of the saliency map (which is the WTAwinner) may not
  // belong to the object that has been segmented, e.g., from some
  // feature map; so we just force it into the objmask first.
  Image<byte> objmask2(objmask);
  objmask2.setVal((winner.i * objmask.getWidth()) / indims.w(),
                  (winner.j * objmask.getHeight()) / indims.h(),
                  byte(255));
  Image<byte> iormask = lowPass3(objmask2) * (winmapnormalized + 0.25F);

  // great, let's post an event with all our results:
  rutz::shared_ptr<SimEventShapeEstimatorOutput>
    e(new SimEventShapeEstimatorOutput
      (this, winmap, 
       objmask, iormask, itsSmoothMask, itsCumMask,
       winmap.name(), goodseed));
  q.post(e);

  // log it:
  const std::string msg =
    sformat("(%d,%d) %s %s", 
            winner.i, winner.j, winmap.name().c_str(), 
            goodseed ? "[Shape]" : "[Disk]");

  textLog(itsLogFile.getVal(), "ShapeEstimator", msg);
  LINFO("Shape estimated %s", msg.c_str());

  // winning map : 1/16x1/16 --> probably has no more use
  // objmask     : 1/16x1/16 --> coarse version
  // iormask     : 1/16x1/16 --> winmap * objmask 
  // its smooth mask     : org size
  // its cumulative mask : org size

  // difference between objmask & iormask is that
  // the value of different parts in the region is modulated by winmap

  // ///////////////////////////////////////////////////////////////
  // // DEBUG DISPLAY 
  // ///////////////////////////////////////////////////////////////
  // uint width  = itsSmoothMask.getWidth();
  // uint height = itsSmoothMask.getHeight();
  // uint scale  = 16; 
  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(width*3,height*2), 0, 0, "ShapeEst"));
  // itsWin->setDims(Dims(width*3, height*2));

  // // LINFO("%d %d", winmap.getWidth(), winmap.getHeight());
  // // for(uint y = 0; y < 20; y++)
  // //   {
  // //     for(uint x = 0; x < 30; x++)
  // //       printf("%7.3f ", winmap.getVal(x,y));
  // //     printf("\n");
  // //   }
 
  // // LINFO("%d %d", objmask.getWidth(), objmask.getHeight());
  // // for(uint y = 0; y < 20; y++)
  // //   {
  // //     for(uint x = 0; x < 30; x++)
  // //       printf("%6d ", objmask.getVal(x,y));
  // //     printf("\n");
  // //   }

  // // LINFO("%d %d", iormask.getWidth(), iormask.getHeight());
  // // for(uint y = 0; y < 20; y++)
  // //   {
  // //     for(uint x = 0; x < 30; x++)
  // //       printf("%6d ", iormask.getVal(x,y));
  // //     printf("\n");
  // //   }

  // // LINFO("%d %d", itsSmoothMask.getWidth(), itsSmoothMask.getHeight());
  // // LINFO("%d %d", itsCumMask.getWidth(), itsCumMask.getHeight());


  // Image<float> disp(width*3, height*2, ZEROS);

  // Image<float> dwinmap = zoomXY(winmap,scale);
  // inplaceNormalize(dwinmap, 0.0F, 1.0F);
  // inplacePaste(disp, dwinmap, Point2D<int>(0,0));

  // Image<float> dobjmask = zoomXY(objmask,scale);
  // inplaceNormalize(dobjmask, 0.0F, 1.0F);
  // inplacePaste(disp, dobjmask, Point2D<int>(width,0));

  // Image<float> diormask = zoomXY(iormask,scale);
  // inplaceNormalize(diormask, 0.0F, 1.0F);
  // inplacePaste(disp, diormask, Point2D<int>(2*width,0));

  // Image<float> dsmmask = itsSmoothMask;
  // inplaceNormalize(dsmmask, 0.0F, 1.0F);
  // inplacePaste(disp, dsmmask, Point2D<int>(0,height));

  // Image<float> dcummask = itsCumMask;
  // inplaceNormalize(dcummask, 0.0F, 1.0F);
  // inplacePaste(disp, dcummask, Point2D<int>(width, height));

  // itsWin->drawImage(disp,0,0);

  // Raster::waitForKey();
}

// ######################################################################
void ShapeEstimator::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  if (itsSaveObjMask.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs = 
        dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

      if (itsSmoothMask.initialized())
        {
          const Image<byte> om = itsSmoothMask * 255.0F;
          ofs->writeGray
            (om, "OBJ", FrameInfo("ShapeEstimator object mask", SRC_POS));
        }
      else
        {
          if (SeC<SimEventRetinaImage> ee = 
              q.check<SimEventRetinaImage>(this, SEQ_ANY))
            ofs->writeGray(Image<byte>(ee->frame().getDims(), ZEROS), "OBJ",
                           FrameInfo("ShapeEstimator object mask", SRC_POS));
        }
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
