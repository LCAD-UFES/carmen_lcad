/*!@file Channels/MotionSpatioTemporalChannel.C */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MotionSpatioTemporalChannel.C $
// $Id: $
//

#ifndef MOTIONSPATIOTEMPORALCHANNEL_C_DEFINED
#define MOTIONSPATIOTEMPORALCHANNEL_C_DEFINED

#include "Channels/MotionSpatioTemporalChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/DirectionSpatioTemporalChannel.H"
#include "Component/OptionManager.H"
#include "rutz/trace.h"
#include "Util/Timer.H"
#include "Image/ShapeOps.H"
#include "Image/FilterOps.H"
#include "Image/DrawOps.H"

#define SELF_MOTION_WEIGHT     1.0F
#define OBJECT_MOTION_WEIGHT   1.0F
#define MAX_FIRING_RATE        100.0F

// ######################################################################
// MotionSpatioTemporalChannel member definitions:
// ######################################################################

// ######################################################################
MotionSpatioTemporalChannel::MotionSpatioTemporalChannel(OptionManager& mgr) :
  ComplexChannel(mgr, 
                 "MotionSpatioTemporal", 
                 "motionSpatioTemporal", 
                 MOTIONSPATIOTEMPORAL),
  itsPyrType("MotionChannelPyramidType", this, Gaussian5),
  itsNumDirs(&OPT_NumSpatioTemporalDirections, this), // see Channels/ChannelOpts.{H,C}
  itsNumSpeeds(&OPT_NumSpatioTemporalSpeeds, this), // see Channels/ChannelOpts.{H,C}
  itsFoeDetector(new FoeDetector(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // let's create our subchannels (may be reconfigured later if our
  // number of directions changes):
  buildSubChans();

  itsMT.reset(new MiddleTemporal());
  addSubComponent(itsFoeDetector);

  itsCurrentFoeMapIndex = -1;
  itsWin.reset();
}

// ######################################################################
MotionSpatioTemporalChannel::~MotionSpatioTemporalChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
DirectionSpatioTemporalChannel& MotionSpatioTemporalChannel::dirChan
(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *(dynCast<DirectionSpatioTemporalChannel>(subChan(idx)));
}

// ######################################################################
void MotionSpatioTemporalChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our subchannels now that we know how many we
  // want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  LINFO("Using %d directions spanning [0..360]deg", itsNumDirs.getVal());

  // the various directional pyrbuilders
  itsDirectionSpatioTemporalChannels.clear();
  itsDirectionSpatioTemporalChannels.resize(itsNumDirs.getVal());

  // go through the different directions and displacement/time
  for (uint i = 0; i < itsNumDirs.getVal(); i++)
    {
      for (uint j = 0; j < itsNumSpeeds.getVal(); j++)
        {
          float speed = pow(2.0, j);

          nub::ref<DirectionSpatioTemporalChannel> chan =
            makeSharedComp(new DirectionSpatioTemporalChannel
                           (getManager(), i, j,
                            360.0 * double(i) /
                            double(itsNumDirs.getVal()),
                            speed,
                            itsPyrType.getVal()));

          itsDirectionSpatioTemporalChannels[i].push_back(chan);

          this->addSubChan(chan);
          
          chan->exportOptions(MC_RECURSE);
        }
    }

  // Spatiotemporal features and MT features
  itsRawSpatioTemporalEnergy.clear();
  itsRawSpatioTemporalEnergy.resize(itsNumDirs.getVal());
  for(uint i = 0; i < itsNumDirs.getVal(); i++)
    itsRawSpatioTemporalEnergy[i].resize(itsNumSpeeds.getVal());
}

// ######################################################################
void MotionSpatioTemporalChannel::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumDirs &&
      numChans() != itsNumDirs.getVal())
    buildSubChans();
}

// ######################################################################
void MotionSpatioTemporalChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.grayFloat().initialized());

  Timer tim1(1000000);

  Image<byte> image(inframe.grayFloat());
  itsCurrentImage = image;

  // compute spatiotemporal motion detection 
  // into several directions and speeds
  for (uint i = 0; i < numChans(); i++)
    {
      subChan(i)->input(inframe);
      LINFO("Motion pyramid (%d/%d) ok.", i+1, numChans());
    }

  // get the spatiotemporal energy 
  uint index = 0;
  for (uint i = 0; i < itsNumDirs.getVal(); i++)
    {
      for (uint j = 0; j < itsNumSpeeds.getVal(); j++)
        {          
          itsRawSpatioTemporalEnergy[i][j] = 
            itsDirectionSpatioTemporalChannels[i][j]->getSpatioTemporalEnergy();
            //itsSpatioTemporalPyrBuilders[i][j]->getSpatioTemporalEnergy();
            //dynamic_cast<DirectionSpatioTemporalChannel*>(subChan(index))
            //      ->getSpatioTemporalEnergy();
            //subChan(index)->getSpatioTemporalEnergy();
          index++;
        }
    }

  LINFO("     time: %f \n", tim1.get()/1000.0);

  // if there is enough frames to compute the spatiotemporal energy
  if(itsRawSpatioTemporalEnergy[0][0].size() != 0)
    {
      Timer tim2(1000000);

      // compute the Middle Temporal features 
      itsMT->computeMTfeatures(itsRawSpatioTemporalEnergy);
      std::vector<Image<float> > mtFeatures = itsMT->getMTfeatures();
      for (uint i = 0; i < itsNumDirs.getVal(); i++)
        for (uint j = 0; j < itsNumSpeeds.getVal(); j++)
          itsDirectionSpatioTemporalChannels[i][j]->setMTfeatureMap(mtFeatures[i]);      

      LINFO("\n computeMTfeatures time: %f \n", tim2.get()/1000.0);
      
      // compute motion conspicuity map
      computeConspicuityMap();

      LINFO("\n computeConspicuityMap time: %f \n", tim2.get()/1000.0);
    }
}

// ######################################################################
void MotionSpatioTemporalChannel::computeConspicuityMap()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // if the MT features has not been computed just return blank map
  std::vector<Image<float> > mtFeatures = itsMT->getMTfeatures();
  uint mtWidth  = mtFeatures[0].getWidth();
  uint mtHeight = mtFeatures[0].getHeight();  
  if(mtWidth == 0 || mtHeight == 0) return;

  uint cmWidth  = subChan(0)->getMapDims().w();
  uint cmHeight = subChan(0)->getMapDims().h();

  // go through all the V1 features 
  // Max Normalize each direction&speed and combine
  //Image<float> result = getV1ObjectMotionMap();
  Image<float> result = getMTObjectMotionMap();

  Image<float> tsubmap = maxNormalize(result, MAXNORMMIN, MAXNORMMAX,
                                      itsNormType.getVal());
  result = tsubmap * numChans();
  Image<float> tres = result;
  result = rescale(tres, Dims(mtWidth, mtHeight));

  // May add a map that comes from higher level Motion areas
  //  : MST: FOE, planar motion  
  //  : STS: Biological motion  

  // NOTE:  FOE_METHOD_TEMPLATE is fooled by Planar movement!!! 
  Image<float> foeMap = 
    itsFoeDetector->getFoeMap(itsMT->getMTfeatures(),                            
                              itsMT->getMToptimalShift(), 
                              FOE_METHOD_TEMPLATE, true);// FOE_METHOD_AVERAGE);

 // if(itsWin.is_invalid())
 //   itsWin.reset(new XWinManaged(Dims(mtWidth*4, mtHeight*4), 
 //                                10, 0, "MotSpch: conspicuity map"));
 // else itsWin->setDims(Dims(mtWidth*4, mtHeight*4));
 // itsWin->drawImage(zoomXY(foeMap,4),0,0); Raster::waitForKey();

  // crazy normalizer  
  float mn,mx; getMinMax(foeMap,mn,mx);
  inplaceNormalize(foeMap,    0.0F, 1.0F);
  foeMap = toPower(foeMap, 40.0F);
  foeMap *= mx;

  // weight the firing rate to the maximum possible firing rate
  foeMap *= (SELF_MOTION_WEIGHT *  MAX_FIRING_RATE * numChans());

  //getMinMax(result,mn,mx);
  //LINFO("FINAL MSTv : %f %f",mn,mx);

  // getMinMax(foeMap,mn,mx);
  // LINFO("FINAL MSTd : %f %f",mn,mx);

  // itsWin->drawImage(zoomXY(foeMap,4),0,0); Raster::waitForKey();
  // itsWin->drawImage(zoomXY(result,4),0,0); Raster::waitForKey();

  result += foeMap;

  // itsWin->drawImage(zoomXY(result,4),0,0); Raster::waitForKey();

  // resize submap to fixed scale if necessary:
  //float mn, mx;
  getMinMax(result,mn,mx);
  if (mtWidth > cmWidth)
    result = downSize(result, Dims(cmWidth, cmHeight));
  else if (mtWidth < cmWidth)
    result = rescale(result, Dims(cmWidth, cmHeight));
  inplaceNormalize(result,0.0F,mx);

  itsConspicuityMap = result;
}

// ######################################################################
Image<float> MotionSpatioTemporalChannel::getV1ObjectMotionMap()
{
  uint cmWidth  = subChan(0)->getMapDims().w();
  uint cmHeight = subChan(0)->getMapDims().h();
  Image<float> result(cmWidth, cmHeight, ZEROS);

  for (uint i = 0; i < itsNumDirs.getVal(); i++)
    {
      for (uint j = 0; j < itsNumSpeeds.getVal(); j++)
        {
          for (uint k = 0; k < itsRawSpatioTemporalEnergy[i][j].size(); k++)
            {
              Image<float> tmap   = itsRawSpatioTemporalEnergy[i][j][k];
              Image<float> submap = downSizeClean(tmap, Dims(cmWidth, cmHeight));

              Image<float> psubmap;
              if (itsUseOlderVersion.getVal())
                {
                  LDEBUG("%s[%d]: applying %s(%f .. %f)", 
                         tagName().c_str(), i, 
                         maxNormTypeName(itsNormType.getVal()), MAXNORMMIN, MAXNORMMAX);
                  psubmap = maxNormalize(submap, MAXNORMMIN, MAXNORMMAX,
                                         itsNormType.getVal());
                }
              else
                {
                  LDEBUG("%s[%d]: applying %s(0.0 .. 0.0)", tagName().c_str(), i, 
                         maxNormTypeName(itsNormType.getVal()));
                  psubmap = maxNormalize(submap, 0.0f, 0.0f, itsNormType.getVal());
                }

              result += psubmap;




               // uint scale = pow(2.0, k); 	 
 
               // LINFO("mt[%d][%d][%d]", i,j,k); 	 
               // if(itsWin.is_invalid())
               //   itsWin.reset(new XWinManaged(Dims(mtWidth*8, mtHeight*8), 
               //                                10, 0, "MotSpch: conspicuity map"));
               //  Image<float> disp(Dims(8*mtWidth,8*mtHeight), NO_INIT); 	 
               // //itsWin->setDims(Dims(8*mtWidth,4*mtHeight)); 	 
 
 
               // Image<float> dtmap = zoomXY(tmap, scale); 	 
               // inplaceNormalize(dtmap,    0.0F, 255.0F); 	 
               // inplacePaste(disp, dtmap, Point2D<int>(0,0)); 	 
 
 
               // LINFO("mn"); 	 
               // Image<float> dsmap = zoomXY(submap,16); //16 	 
               // inplaceNormalize(dsmap,    0.0F, 255.0F); 	 
               // inplacePaste(disp, dsmap, Point2D<int>(4*mtWidth, 0)); 	 
 
               // LINFO("mn2"); 	 
               // Image<float> dpsmap = zoomXY(psubmap, 16); 	 
               // //Image<float> dpsmap = zoomXY(psubmap, 16); 	 
               // inplaceNormalize(dpsmap,    0.0F, 255.0F); 	 
               // inplacePaste(disp, dpsmap, Point2D<int>(0, 4*mtHeight)); 	 
 
               // LINFO("mn3"); 	 
               // Image<float> dres = zoomXY(result, 16); 	 
               // inplaceNormalize(dres,    0.0F, 255.0F); 	 
               // inplacePaste(disp, dres, Point2D<int>(4*mtWidth, 4*mtHeight)); 	 
 
               // //itsWin->drawImage(rescale(result, Dims(4*mtWidth, 4*mtHeight)),0,0); 	 
               // itsWin->drawImage(disp,0,0); 	 
               // Raster::waitForKey(); 	 
	  	 	
            }
        }
    }

  return result;
}

// ######################################################################
Image<float> MotionSpatioTemporalChannel::getMTObjectMotionMap()
{
  std::vector<Image<float> > mtFeatures = itsMT->getMTfeatures();
  uint mtWidth  = mtFeatures[0].getWidth();
  uint mtHeight = mtFeatures[0].getHeight();  
  if(mtWidth == 0 || mtHeight == 0) return Image<float>();

  uint cmWidth  = subChan(0)->getMapDims().w();
  uint cmHeight = subChan(0)->getMapDims().h();
  //Image<float> result(cmWidth, cmHeight, ZEROS);
  Image<float> result(mtWidth, mtHeight, ZEROS);

  LINFO("MT: %d %d CM: %d %d", mtWidth,mtHeight, cmWidth, cmHeight);



  // Image<float> img = itsCurrentImage;
  // uint imgWidth  = img.getWidth(); 
  // uint imgHeight = img.getHeight();
  // Image<float> disp2(Dims(imgWidth, imgHeight), NO_INIT); 	 
  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(Dims(imgWidth, imgHeight), 
  //                                10, 0, "MotSpch: MT conspicuity map"));



  for (uint i = 0; i < mtFeatures.size(); i++)
    {
      Image<float> tmap   = mtFeatures[i];

      
      

      // Image<float> im = img;
      // inplaceNormalize(im,    0.0F, 255.0F); 	 
      // Image<float> mt = zoomXY(tmap, 4);
      // inplaceNormalize(mt,    0.0F, 255.0F); 	 
      // disp2 = im + mt;
      // itsWin->drawImage(disp2,0,0); 	 
      // Raster::waitForKey();







      Image<float> submap = tmap; //downSizeClean(tmap, Dims(cmWidth, cmHeight));
      
      Image<float> psubmap;
      // if (itsUseOlderVersion.getVal())
      //   {
      //     LDEBUG("%s[%d]: applying %s(%f .. %f)", 
      //            tagName().c_str(), i, 
      //            maxNormTypeName(itsNormType.getVal()), MAXNORMMIN, MAXNORMMAX);
      //     psubmap = maxNormalize(submap, MAXNORMMIN, MAXNORMMAX,
      //                            itsNormType.getVal());
      //   }
      // else
      //   {
      //     LDEBUG("%s[%d]: applying %s(0.0 .. 0.0)", tagName().c_str(), i, 
      //            maxNormTypeName(itsNormType.getVal()));
      //     psubmap = maxNormalize(submap, 0.0f, 0.0f, itsNormType.getVal());
      //   }
      psubmap = submap;
      
      result += psubmap;

      //uint scale = 1; //4 for CM
      // LINFO("mt[%d]", i); 	 
      // if(itsWin.is_invalid())
      //   itsWin.reset(new XWinManaged(Dims(mtWidth*8, mtHeight*8), 
      //                                10, 0, "MotSpch: MT conspicuity map"));
      // Image<float> disp(Dims(8*mtWidth,8*mtHeight), NO_INIT); 	 
      // itsWin->setDims(Dims(8*mtWidth,8*mtHeight)); 
 
      // Image<float> dtmap = zoomXY(tmap, 4); 	 
      // inplaceNormalize(dtmap,    0.0F, 255.0F); 	 
      // inplacePaste(disp, dtmap, Point2D<int>(0,0));
      
      // LINFO("mn"); 	 
      // Image<float> dsmap = zoomXY(submap,4); //16 	 
      // inplaceNormalize(dsmap,    0.0F, 255.0F); 	 
      // inplacePaste(disp, dsmap, Point2D<int>(4*mtWidth, 0)); 	 
      
      // LINFO("mn2"); 	 
      // Image<float> dpsmap = zoomXY(psubmap, 4); 	 
      // inplaceNormalize(dpsmap,    0.0F, 255.0F); 	 
      // inplacePaste(disp, dpsmap, Point2D<int>(0, 4*mtHeight)); 	 
      
      // LINFO("mn3"); 	 
      // Image<float> dres = zoomXY(result, 4); 	 
      // inplaceNormalize(dres,    0.0F, 255.0F); 	 
      // inplacePaste(disp, dres, Point2D<int>(4*mtWidth, 4*mtHeight)); 	 
      
      // itsWin->drawImage(disp,0,0); 	 
      // Raster::waitForKey();
    }


  result = downSizeClean(result, Dims(cmWidth, cmHeight));
  return result;
}

// ######################################################################
Image<float> MotionSpatioTemporalChannel::getOutput()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 return itsConspicuityMap;
}

// ######################################################################
Image<float> MotionSpatioTemporalChannel::
downSizeMax(Image<float> img, uint scale)
{
  img = lowPassX(9,img);
  img = lowPassY(9,img);

  uint width  = img.getWidth();
  uint height = img.getHeight();
  
  uint oWidth  = width/scale;
  uint oHeight = height/scale;
  
  Image<float> result(oWidth, oHeight, NO_INIT);
  for(uint i = 0; i < oWidth; i++)
    for(uint j = 0; j < oHeight; j++)
      {
        float max = 0.0;
        for(uint di = 0; di < scale; di++)
          for(uint dj = 0; dj < scale; dj++)
            {
              uint ci = i*scale + di;
              uint cj = j*scale + dj;
              float val = img.getVal(ci,cj);              
              if(val > max) max = val;
            }
        result.setVal(i,j,max);        
      }

  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MOTION_SPATIOTEMPORALENERGY_CHANNEL_C_DEFINED
