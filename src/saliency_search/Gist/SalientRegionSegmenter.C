/*!@file Gist/SalientRegionSegmenter.C segment out object depicted by the
salient point. Here we use both region growing and boundary detection
 */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/SalientRegionSegmenter.C $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Gist/SalientRegionSegmenter.H"

#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"
#include "Image/ShapeOps.H"
#include "Image/Kernels.H"

#include "Util/Timer.H"

#include  <cstdio>

#define MINIMUM_CONTOUR_LENGTH        5
#define CONTOUR_CHAIN_NUM             2
#define DISTANCE_THRESHOLD_VALUE_2  0.2F   
#define SIMILARITY_THRESHOLD        1.0F  
#define RATIO_THRESHOLD             2.0F  

#define REGION_INCLUDE_THRESHOLD    0.6F

#define LEN_WEIGHT                 0.20F // contour boundary length weight
#define SIM_WEIGHT                 0.20F // similarity with closer side weight 
#define STR_WEIGHT                 0.20F // strength of boundary weight
#define RAT_WEIGHT                 0.20F // ratio of in:out weight
#define DIS_WEIGHT                 0.20F // distance between sal pt and segment

#define NEIGHBORHOOD_RADIUS          12
//#define NEIGHBORHOOD_RADIUS           8
#define MAX_FAR_TO_NEAR_RATIO       3.0F
#define DISTANCE_THRESHOLD_1       0.25F // quarter of the diagonal
#define DISTANCE_THRESHOLD_2       0.50F // quarter of the diagonal

#define NUM_CHANNELS                  3 // current number of channels   
#define NUM_L_BINS                   25 // # bins for L component of CIELab
#define NUM_A_BINS                   25 // # bins for a component of CIELab
#define NUM_B_BINS                   25 // # bins for b component of CIELab
#define NUM_HISTOGRAM_DIMS           75 // current total number of bins

#define UNVISITED_REGION              0 // unvisited mean, just looked at
                                        // for the first time 
#define SURROUND_REGION               1
//#define INCLUDED_EDGEL_REGION         2
#define CENTER_REGION                 3
#define CENTER_CORE_REGION            4
#define UNASSIGNED_REGION             5
#define OTHER_OBJECT_REGION           6 

#define CLOSER_SIDE                   1
#define FAR_SIDE                      2

// ######################################################################
// ######################################################################

// ######################################################################
SalientRegionSegmenter::SalientRegionSegmenter()
:
itsContourBoundaryDetector(new ContourBoundaryDetector())
{
  itsWin.reset();

  // These color features are inspired by Martin, PAMI 2004 pb paper
  float bg_smooth_sigma    = 0.1;       // bg histogram smoothing sigma
  float cg_smooth_sigma    = 0.05;      // cg histogram smoothing sigma

  itsLKernel = 
    gaussian<float>(1.0F, NUM_L_BINS*bg_smooth_sigma, 
                    int(3.0F*NUM_L_BINS*bg_smooth_sigma + 0.5F));
  itsAKernel = 
    gaussian<float>(1.0F, NUM_A_BINS*cg_smooth_sigma, 
                    int(3.0F*NUM_A_BINS*cg_smooth_sigma + 0.5F));
  itsBKernel = 
    gaussian<float>(1.0F, NUM_B_BINS*cg_smooth_sigma, 
                    int(3.0F*NUM_B_BINS*cg_smooth_sigma + 0.5F));

  float suml = sum(itsLKernel); itsLKernel = itsLKernel/suml;
  float suma = sum(itsAKernel); itsAKernel = itsAKernel/suma;
  float sumb = sum(itsBKernel); itsBKernel = itsBKernel/sumb;

  itsIteration = 0;
}

// ######################################################################
SalientRegionSegmenter::~SalientRegionSegmenter()
{ }

// ######################################################################
void SalientRegionSegmenter::setImage
(Image<PixRGB<byte> > image, std::string nameStem)
{ 
  // reset everything
  itsSalientPoints.clear();

  itsImage = image;
  if(nameStem.length() > 0)
    itsInputImageStem = nameStem;
  else
    itsInputImageStem = std::string("result");  

  // this is the feature histogram in Martin's pb PAMI 2004
  setImageFeatureHistogramValues();

  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();

  // calculate the contour boundary and region map
  computeBoundaryAndRegionInformation();

  // keep track of what histograms has been computed
  uint hstep = BOUNDARY_STEP_SIZE/2;
  uint wG = (w+hstep)/BOUNDARY_STEP_SIZE;
  uint hG = (h+hstep)/BOUNDARY_STEP_SIZE;

  itsImageFeatureHistograms = 
    Image<rutz::shared_ptr<Histogram> >(wG, hG, ZEROS);

  // set all grid region to -1 as unassigned
  // 0 would be for object 0
  itsCurrentRegionAssignmentMap  = Image<int>(wG, hG, ZEROS);
  itsCurrentRegionAssignmentMap -= 1;
  itsCurrentNumSalientRegions    = 0; 

  // set the final thin boundary image
  itsBoundaryImage = Image<float>(w,h,ZEROS);

  itsConsAssignedCount = 0;

  itsHistogramGenerationCountPt  = 0;
  itsHistogramGenerationCountPts = 0;

  itsTimePt  = 0;
  itsTimePts = 0;
  itsTimeCompare = 0;

  itsObjectMask.clear();
  itsSalientRegionMask.clear();
}

// ######################################################################
void SalientRegionSegmenter::setImageFeatureHistogramValues()
{
  // convert to CIElab color space
  Image<float> lImg; 
  Image<float> aImg;
  Image<float> bImg;
  getNormalizedLAB(itsImage, lImg, aImg, bImg);
  
  // convert to bin numbers
  itsImageHistogramEntries.clear();
  itsImageHistogramEntries.push_back(quantize_values(lImg, NUM_L_BINS));
  itsImageHistogramEntries.push_back(quantize_values(aImg, NUM_A_BINS));
  itsImageHistogramEntries.push_back(quantize_values(bImg, NUM_B_BINS));
}

// ######################################################################
Image<int> SalientRegionSegmenter::quantize_values
(Image<float> image, int num_bins, bool normalize)
{
  // FIXXX: May want to move to MathOps later
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();
  Image<int> result(w,h,NO_INIT);

  Image<float>::iterator iptr = image.beginw();  
  Image<int>::iterator aptr = result.beginw(), stop = result.endw();  
  float nbins = num_bins;
  while(aptr != stop)
    {
      float in = *iptr++;

      // bins will be 0 to num_bins-1
      int bin = int(in*nbins);
      if(bin == num_bins) bin = num_bins - 1;
      *aptr++ = bin;
    }

  return result;
}

// ######################################################################
void SalientRegionSegmenter::computeBoundaryAndRegionInformation()
{
  computeBoundaryInformation(); 
  computeRegionInformation(); 

  //displayBoundaryAndRegionInformation();
}

// ######################################################################
void SalientRegionSegmenter::computeBoundaryInformation()
{
  // compute contour boundary map
  Timer timer(1000000); timer.reset();
  itsContourBoundaryDetector->computeContourBoundary(itsImage);
  LINFO("Contour boundary detection time: %f ms", timer.get()/1000.0F);

  itsContourBoundaries = 
    itsContourBoundaryDetector->getContourBoundaries();

  itsEdgels = itsContourBoundaryDetector->getEdgels();

  float maxVal = 0.0; 
  for(uint i = 0; i < itsContourBoundaries.size(); i++)
    {
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[i];
      
      // go through each contour
      uint cLen = contour->edgels.size();
      for(uint j = 0; j < cLen; j++)
        {
          float val = contour->edgels[j]->val;
          if(maxVal < val) maxVal = val;
        }
    }

  itsMaximumEdgelStrength = maxVal;
}

// ######################################################################
void SalientRegionSegmenter::computeRegionInformation()
{
  // default parameters for the graph-based segmentation
  // NOTE: in original Felzenswalb-Huttenlocher paper: 
  // k = 500 --> here k = 25 create over-segmented object
  float sigma = .5; uint k = 25; uint min_size = 20; int num_ccs;

  itsInitialRegions.clear();

  itsInitialRegionImage = 
    SuperPixelSegment
    (itsImage, sigma, k, min_size, num_ccs, &itsInitialRegions);

  itsInitialRegionColorImage = 
    SuperPixelDebugImage(itsInitialRegions, itsImage);
 
  itsRegionAdjecencyList = getRegionAdjecencyList(); 
  //  displayRegionAdjecencyList();
}

// ######################################################################
std::vector<std::vector<uint> > 
SalientRegionSegmenter::getRegionAdjecencyList()
{
  uint nRegions = itsInitialRegions.size();
  std::vector<std::vector<uint> >  adjList(nRegions);
  for(uint i = 0; i < nRegions; i++)
    adjList[i] = std::vector<uint>();

  // find regions:
  // FIXXX: there may be a better way

  // check out all the 
  for(uint i = 0; i < nRegions; i++)
    {
      for(uint j = 0; j < itsInitialRegions[i].size(); j++)
        {          
          Point2D<int> pt = itsInitialRegions[i][j];          
          uint label = i;

          for(int di = -1; di <= 1; di++)
            {
              for(int dj = -1; dj <= -1; dj++)
                {
                  Point2D<int> dpt(di,dj);
                  Point2D<int> pt2 = pt + dpt;
                  if(!(di == 0 && dj == 0) &&  
                     itsInitialRegionImage.coordsOk(pt2))
                    {
                      //if the neighbor is different
                      uint label2 = itsInitialRegionImage.getVal(pt2);

                      // check if the label is already 
                      // in the adjecency list
                      if(label != label2)
                        {
                          bool in = false;
                          for(uint l = 0; l < adjList[i].size(); l++)
                            {
                              if(adjList[i][l] == label2) 
                                { in = true; l = adjList[i].size(); }
                            }
                          if(!in) adjList[i].push_back(label2);
                        }
                    }
                }
            }
        }
    }  
  return adjList;
}

// ######################################################################
void SalientRegionSegmenter::displayRegionAdjecencyList()
{
  // display stuff
  Image<PixRGB<byte> > ima = itsImage;

  Image<float> fIma(luminance(ima));
  Image<float> tempFIma = fIma;
  inplaceNormalize(tempFIma, 0.0f,255.0f);  
  Image<byte>  bIma(tempFIma); 

  // get non-max suppressed boundary map image
  float mVal = 32;
  float bVal = 255 - mVal;

  Image<byte> dImaR, dImaG, dImaB;
  getComponents(ima, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);

  uint w = ima.getWidth();
  uint h = ima.getHeight();
  Image<PixRGB<byte> > dispIma(4*w,2*h,ZEROS);
  inplacePaste (dispIma, ima, Point2D<int>(0,0));

  // go through each region
  uint nRegions = itsInitialRegions.size();
  for(uint i = 0; i < nRegions; i++)
    {     
      Image<float> regionMap(w,h, ZEROS);
      
      for(uint j = 0; j < itsInitialRegions[i].size(); j++)
        {
          Point2D<int> pt = itsInitialRegions[i][j]; 
          regionMap.setVal(pt, 1.0);
        }

      for(uint j = 0; j < itsRegionAdjecencyList[i].size(); j++)
        {
          uint label = itsRegionAdjecencyList[i][j];
          float randVal = rand()/(RAND_MAX+1.0);
          for(uint k = 0; k < itsInitialRegions[label].size(); k++)
            {
              Point2D<int> ptNeighbor = itsInitialRegions[label][k]; 
              regionMap.setVal(ptNeighbor, randVal);
            }
        }
      
      inplaceNormalize(regionMap, 0.0f,bVal);
      Image<byte> dBmapc(regionMap);
      Image<PixRGB<byte> > dBmap = toRGB(dBmapc);
  
      inplacePaste
        (dispIma, Image<PixRGB<byte> >(dIma+dBmap), Point2D<int>(w,0));

      if(itsWin.is_invalid())
        itsWin.reset(new XWinManaged(Dims(4*w,2*h), 0, 0, 
                                     "SalReg Segmenter"));
      else itsWin->setDims(Dims(4*w,2*h));
      itsWin->drawImage(dispIma, 0,0);

      Raster::waitForKey();
    }
}

// ######################################################################
void SalientRegionSegmenter::displayBoundaryAndRegionInformation()
{
  // display stuff
  Image<PixRGB<byte> > ima = itsImage;
  uint w = ima.getWidth();
  uint h = ima.getHeight();

  Image<float> fIma(luminance(ima));
  Image<float> tempFIma = fIma;
  inplaceNormalize(tempFIma, 0.0f,255.0f);  
  Image<byte>  bIma(tempFIma); 

  // get non-max suppressed boundary map image
  float mVal  = 64;
  float bVal  = 255 - mVal;
  Image<byte> dImaR, dImaG, dImaB;
  getComponents(ima, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);

  Image<float> tboundaryMap = 
    itsContourBoundaryDetector->getVarianceRidgeBoundaryMap();
  Image<float> boundaryMap = 
    clampedDiff(tboundaryMap, Image<float>(w,h,ZEROS));
  inplaceNormalize(boundaryMap, 0.0f,bVal);
  Image<byte> dBmapc(boundaryMap);
  Image<PixRGB<byte> > tdBmap = toRGB(dBmapc);
  Image<PixRGB<byte> > dBmap(dIma+tdBmap);

  // the non-max suppressed boundary map image
  Image<float> dBmapNMStemp = 
    itsContourBoundaryDetector->getNmsBoundaryMap();
  inplaceNormalize(dBmapNMStemp, 0.0F, 255.0F);
  Image<PixRGB<byte> > dBmapNMS = 
    toRGB(Image<byte>(dBmapNMStemp));

  // the contour boundary edgel map image
  Image<float> dCBEmapTemp = 
    itsContourBoundaryDetector->getEdgelBoundaryMap();
  inplaceNormalize(dCBEmapTemp, 0.0F, bVal);
  Image<PixRGB<byte> > tdCBEmap = 
    toRGB(Image<byte>(dCBEmapTemp));
  Image<PixRGB<byte> > dCBEmap(dIma+tdCBEmap);

  // get the contour boundary map
  Image<PixRGB<byte> > dCBmapTemp =
    itsContourBoundaryDetector->getContourBoundaryMap();
  Image<PixRGB<byte> > dCBmap(dIma+dCBmapTemp);

  std::string name = itsInputImageStem;
  Image<PixRGB<byte> > tIRCI = itsInitialRegionColorImage; 

  // setup the display map
  Image<PixRGB<byte> > dispIma(4*w,2*h,ZEROS);
  inplacePaste(dispIma, ima,      Point2D<int>(  0, 0));
  inplacePaste(dispIma, dBmap,    Point2D<int>(  w, 0));
  inplacePaste(dispIma, dBmapNMS, Point2D<int>(  0, h));
  inplacePaste(dispIma, dCBEmap,  Point2D<int>(  w, h));
  inplacePaste(dispIma, dCBmap,   Point2D<int>(2*w, 0));
  inplacePaste(dispIma, itsInitialRegionColorImage, Point2D<int>(2*w, h));

  // NOTE: to display contour boundary map
  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(4*w,2*h), 0, 0, 
                                 "SalReg Segmenter"));
  else itsWin->setDims(Dims(4*w,2*h));
  itsWin->drawImage(dispIma, 0,0);
  Raster::waitForKey();

  // VRD, NMS VRD, edgel, contour, FelzHutt k = 25
  // LINFO("saving");
  // Raster::WriteRGB(dBmap,    sformat("%s_vrd.png",        name.c_str()));
  // Raster::WriteRGB(dBmapNMS, sformat("%s_nms_vrd.png",    name.c_str()));
  // Raster::WriteRGB(dCBEmap,  sformat("%s_edgel.png",      name.c_str()));
  // Raster::WriteRGB(dCBmap,   sformat("%s_cb_image.png",   name.c_str()));
  // Raster::WriteRGB(tIRCI,    sformat("%s_FelzHutt25.png", name.c_str()));

  // Raster::waitForKey();
}

// ######################################################################
Image<float> SalientRegionSegmenter::getSalientRegion(Point2D<int> pt)
{ 
  int w = itsImage.getWidth();
  int h = itsImage.getHeight();

  // Round it to the closest 8x8 grid location
  Point2D<int> ptCorrGrid, ptCorr;
  //int step  = BOUNDARY_STEP_SIZE;
  int hstep = BOUNDARY_STEP_SIZE/2;
  ptCorrGrid.i = ((pt.i+hstep)/BOUNDARY_STEP_SIZE); 
  ptCorrGrid.j = ((pt.j+hstep)/BOUNDARY_STEP_SIZE); 

  // we cannot assign values on top and left boundaries
  if(ptCorrGrid.i == 0) ptCorrGrid.i = 1;
  if(ptCorrGrid.j == 0) ptCorrGrid.j = 1;

  int wG = (w+hstep)/BOUNDARY_STEP_SIZE;
  int hG = (h+hstep)/BOUNDARY_STEP_SIZE;

  if(ptCorrGrid.i > wG-1) ptCorrGrid.i = wG-1;
  if(ptCorrGrid.j > hG-1) ptCorrGrid.j = hG-1;

  ptCorr = ptCorrGrid *  BOUNDARY_STEP_SIZE; 

  LDEBUG("FINAL point: %3d %3d --> %3d %3d", 
         ptCorr.i, ptCorr.j, ptCorrGrid.i, ptCorrGrid.j);

  // THIS IS TO END SEGMENTATION IN A PICTURE
  if(itsCurrentRegionAssignmentMap.getVal(ptCorrGrid) >= 0)
    {    
      //itsConsAssignedCount++;
      //if(itsConsAssignedCount > 1)
      //  {
          LINFO("We're done segmenting this image "
                "save the result as a *.pnm file -> "
                "then convert to *.bmp separately\n");
          
          Image<float> tempBI = itsBoundaryImage;
          inplaceNormalize(tempBI, 0.0f,255.0f);  
          Image<byte> tempBIb(tempBI);
          Image<PixRGB<byte> > dmap = 
            makeRGB(tempBIb, tempBIb, tempBIb);
          std::string name = 
            sformat("%s_final_bres.pnm", itsInputImageStem.c_str());
          // Raster::WriteRGB(dmap, name.c_str());
          //Raster::waitForKey();
          //  }

          return Image<float>();
    } 
  //else itsConsAssignedCount = 0;

  Timer tim(1000000); tim.reset();

  // calculate the region first
  itsSalientPoints.push_back(ptCorr);
  Image<float> salientRegionMask = computeSalientRegion();

  LINFO("Time: %f: pt: %d: %f, pts: %d: %f comp: %f", tim.get()/1000.0F,   
        itsHistogramGenerationCountPt, itsTimePt/1000.0F,
        itsHistogramGenerationCountPts, itsTimePts/1000.0F,
        itsTimeCompare/1000.0F);

  //if(itsWin.is_invalid())
  //  itsWin.reset(new XWinManaged(Dims(4*w,2*h), 0, 0, 
  //                               "SalReg Segmenter"));
  //else itsWin->setDims(Dims(4*w,2*h));
  //itsWin->drawImage(salientRegionMask, 0,0);
  //Raster::waitForKey();

  return salientRegionMask;
}

// ######################################################################
Image<float> SalientRegionSegmenter::computeSalientRegion()
{ 
  // grow in hierarchical manner

  // --> may have GMM (Gaussian Mixture Model) 

  // try the PAMI 2011 center surround
  // GROWING SHOULD BE FAST: 100 - 200 ms target

  // does this spread too much ---> IOR but no Object
  // check out how that affect the 

  // what scale to spread???



  // we have original resolution
  // and an 8x8 grid overlay resolution
  // --> this can be refined later or make MULTISCALE
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();

  uint hstep = BOUNDARY_STEP_SIZE/2;
  uint wG = (w+hstep)/BOUNDARY_STEP_SIZE;
  uint hG = (h+hstep)/BOUNDARY_STEP_SIZE;

  Point2D<int> currentSalPoint = 
    itsSalientPoints[itsSalientPoints.size()-1];

  Point2D<int> currentSalPointGrid
    ((currentSalPoint.i+hstep)/BOUNDARY_STEP_SIZE, 
     (currentSalPoint.j+hstep)/BOUNDARY_STEP_SIZE);
 
  // assignment map on the grid overlay resolution
  Image<int> assignmentMap    (wG, hG, ZEROS);
  Image<int> coreAssignmentMap(wG, hG, ZEROS);

  // the image borders are out of bounds
  for(uint i = 0; i < wG; i++) 
    {
      assignmentMap.setVal    (i,0, OTHER_OBJECT_REGION);
      coreAssignmentMap.setVal(i,0, OTHER_OBJECT_REGION);
    }

  for(uint j = 0; j < hG; j++) 
    {
      assignmentMap.setVal    (0,j, OTHER_OBJECT_REGION);
      coreAssignmentMap.setVal(0,j, OTHER_OBJECT_REGION);
    }

  // set the core areas as assigned
  Image<int>::iterator 
    icramptr     = itsCurrentRegionAssignmentMap.beginw(),  
    stopicramptr = itsCurrentRegionAssignmentMap.endw();

  Image<int>::iterator 
    camptr = coreAssignmentMap.beginw(),  
    amptr  = assignmentMap.beginw();
  while(icramptr != stopicramptr)
    {
      if(*icramptr > -1)
        {
          *amptr  = OTHER_OBJECT_REGION; 
          *camptr = OTHER_OBJECT_REGION; 
        }

      amptr++; camptr++; icramptr++;
    }


  // list of grid nodes that are assigned 
  // to either object, background, or unassigned list
  std::vector<Point2D<int> > currentObjectLocations;
  std::vector<Point2D<int> > currentBackgroundLocations;
  std::vector<Point2D<int> > currentUnassignedLocations;
  std::vector<Point2D<int> > currentIncludedEdgelLocations;

  // oversegmented assignment map 
  Image<int> regionAssignmentMap(w, h, ZEROS);

  // list of initial oversegmented regions that are  
  // assigned to either object or background
  std::vector<int> currentObjectRegionList;
  std::vector<int> currentBackgroundRegionList;

  // create boundary fragments (also call segments): 
  //   divide contour to segments of 5 edgels: about 30 - 40 pix long
  //   --> only consider contours of 5 edgels or more.
  // sort on longest contour, most similar chi-sq, closest, most salient, 
  std::list<Segment> sortedSegments = fillSegmentQueue(currentSalPoint);
  //displaySegments(sortedSegments);

  // if there is no appropriate segment right away
  if(sortedSegments.size() == 0)
    {
      Point2D<int> fpt = 
        itsSalientPoints[itsSalientPoints.size()-1];
      itsSalientPoints.pop_back();
      LINFO("CAN'T GROW POINT: %3d %3d", fpt.i, fpt.j);
      return Image<float>();
    }

  // current starting point to grow from
  Point2D<int> currPoint     = currentSalPoint;
  Point2D<int> currPointGrid = currentSalPointGrid;
  LINFO("START PT: %3d %3d --> %3d %3d", 
        currPoint.i, currPoint.j, currPointGrid.i, currPointGrid.j);

  // to start, add the starting salient point to unassigned list 
  currentUnassignedLocations.push_back(currPointGrid); 
  assignmentMap.setVal(currPointGrid, UNASSIGNED_REGION);


  // while there are uncovered object cores
  bool allCoresCovered = false;
  while(!allCoresCovered)
    {
      // if no segments qualified -- we also break out 
      if(sortedSegments.size() == 0) break;

      // pick best segment (on metric above)
      Segment segment = sortedSegments.front();
      sortedSegments.pop_front();

      // compute histogram of distribution of points within 12pix
      rutz::shared_ptr<Histogram> hist1 = getHistogramDistribution(currPoint);
      
      // compute distribution of both side of the boundary 
      // closer is pts2, farther is pts3
      std::vector<Point2D<int> > pts2;
      std::vector<Point2D<int> > pts3;
      getSegmentPointDistributions(segment, currPoint, pts2, pts3);
      rutz::shared_ptr<Histogram> hist2 = getHistogramDistribution(pts2);
      rutz::shared_ptr<Histogram> hist3 = getHistogramDistribution(pts3);
      
      // compute the Chi-sq difference: this is the threshold for growing
      float threshDiff    = hist1->getChiSqDiff(*hist2);
      float borderDiff    = hist2->getChiSqDiff(*hist3);
      float oppThreshDiff = hist1->getChiSqDiff(*hist3);
      float ratio         = oppThreshDiff/threshDiff;
      //LINFO("THRESH Diff: %f, BorderDiff: %f oppThresh Diff: %f --> ratio: %f", 
      //      threshDiff, borderDiff, oppThreshDiff, ratio); // DISP

      //LINFO("{%3d} %10.3f, %10.3f, %10.3f, %10.3f, %10.3f) = %10.3f", // DISP
      //      segment.orgIndex,
      //      segment.lenVal, segment.simVal, segment.strVal, 
      //      segment.ratVal, segment.disVal, segment.pVal  );

      // grow CENTER to the direction of the boundary 
      // to connect the two 
      int index = segment.segStartIndex + MINIMUM_CONTOUR_LENGTH/2; 
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[segment.contourIndex];  
      rutz::shared_ptr<Edgel> edgel = contour->edgels[index];
      //Point2D<int> segCenterPoint = edgel->pt; 
      Point2D<int> segCenterPointGrid
        ((edgel->pt.i+hstep)/BOUNDARY_STEP_SIZE,
         (edgel->pt.j+hstep)/BOUNDARY_STEP_SIZE );
      std::vector<Point2D<int> > pointGrids = 
        getLine(currPointGrid, segCenterPointGrid);

      //LINFO("segment points"); // DISP
      // for(uint i = segment.segStartIndex; i <= segment.segEndIndex; i++) 
      //   {
      //     rutz::shared_ptr<Edgel> tedgel = contour->edgels[i];
      //     Point2D<int> tptGrid
      //       ((tedgel->pt.i+hstep)/BOUNDARY_STEP_SIZE,
      //        (tedgel->pt.j+hstep)/BOUNDARY_STEP_SIZE );
           
      //     LINFO("[<<%3d>>] %d %d --> %d %d", //DISP
      //           i, tptGrid.i, tptGrid.j,
      //           tedgel->pt.i,tedgel->pt.j);
      //   }

      // LINFO("LINE: %3d %3d to %3d %3d", // DISP
      //       currPointGrid.i, currPointGrid.j, 
      //       segCenterPointGrid.i, segCenterPointGrid.j);
      // for(uint i = 0; i < pointGrids.size(); i++)
      //   LINFO("[%3d]: %3d %3d", i, pointGrids[i].i, pointGrids[i].j);

      // check if all growing points 
      // are closer to center than surround
      bool clearPoints = false;
      for(uint i = 0; i < pointGrids.size(); i++)
        {
          // can't go through other objects
          if(assignmentMap.getVal(pointGrids[i]) == OTHER_OBJECT_REGION)
            {
              clearPoints = true; i = pointGrids.size();    
              continue;
            }

          // check if it is already included in the center
          if(assignmentMap.getVal( pointGrids[i]) == CENTER_REGION)
            continue;

          Point2D<int> pt = pointGrids[i]*BOUNDARY_STEP_SIZE; 
          rutz::shared_ptr<Histogram> ptHist = getHistogramDistribution(pt);
          float ptDiff     = ptHist->getChiSqDiff(*hist1);

          //LINFO("[%d %d] ptDiff: %f", 
          //      pointGrids[i].i, pointGrids[i].j, ptDiff);

          // check if point is part of segment
          bool isSegmentEdgel = false;
          for(uint j = segment.segStartIndex; j <= segment.segEndIndex; j++) 
            {
              rutz::shared_ptr<Edgel> tedgel = contour->edgels[j];
              Point2D<int> tptGrid
                ((tedgel->pt.i+hstep)/BOUNDARY_STEP_SIZE,
                 (tedgel->pt.j+hstep)/BOUNDARY_STEP_SIZE );
              if(tptGrid == pointGrids[i])
                {
                  isSegmentEdgel = true;
                  j = segment.segEndIndex + 1;
                  //LINFO("%d %d is an edgel pt",  
                  //      pointGrids[i].i,  pointGrids[i].j);
                }
            }

          // check if point is part of included edgels
          bool inCIEL = false;
          for(uint j = 0; j < currentIncludedEdgelLocations.size(); j++)
            if(currentIncludedEdgelLocations[j] == pointGrids[i])
              { 
                inCIEL = true; j = currentIncludedEdgelLocations.size(); 
                //LINFO("%d %d in included edgels", 
                //      pointGrids[i].i, pointGrids[i].j);
              }

          // if neither edgel or segment point
          if(!isSegmentEdgel && !inCIEL)
            {
              // check for ratio
              bool passRatio = checkBackgroundRatio
                (ptDiff, ptHist, pointGrids[i], assignmentMap,
                 currentBackgroundLocations,
                 currentIncludedEdgelLocations);

              if(!passRatio)
                { clearPoints = true; i = pointGrids.size(); }
            }
        }
      
      if(clearPoints) pointGrids.clear();

      // go through each starting grow location
      Image<bool> isComparedWith(wG, hG, ZEROS);
      for(uint i = 0; i < pointGrids.size(); i++)
        {
          Point2D<int> startPtGrid = pointGrids[i];          
          Point2D<int> startPt(startPtGrid.i*BOUNDARY_STEP_SIZE, 
                               startPtGrid.j*BOUNDARY_STEP_SIZE);

          //LINFO("start point: %3d %3d --> %3d %3d", 
          //      startPtGrid.i, startPtGrid.j, startPt.i, startPt.j);

          std::vector<Point2D<int> >::iterator culItr = 
            currentUnassignedLocations.begin();
          
          if(assignmentMap.getVal(startPtGrid) == UNVISITED_REGION) 
            {
              currentUnassignedLocations.insert(culItr, startPtGrid);
              assignmentMap.setVal(startPtGrid, UNASSIGNED_REGION);
            }
          //else LINFO("not adding grow pt -- already in unassigned list");

          // start from the front of the queue 
          uint currIndex = 0;

          // while there is still neighbor that we have not visited
          // as a result of the current salreg-salcontour combo
          while(currIndex < currentUnassignedLocations.size())
            {
              // get location
              Point2D<int> ptGrid = currentUnassignedLocations[currIndex];
              Point2D<int> pt     = ptGrid * BOUNDARY_STEP_SIZE;

              //LINFO("curr pt to check: %3d %3d --> %3d %3d", 
              //      ptGrid.i, ptGrid.j, pt.i, pt.j);

              // get the path to that point 
              // check if it goes through a surround grid pt
              bool throughSurround = 
                throughRegion(currPointGrid, ptGrid,
                              assignmentMap, SURROUND_REGION);
              //LINFO("throughSurround: %d", throughSurround);
              
              // if does not go through surround 
              // and has not yet checked
              if(!throughSurround && !isComparedWith.getVal(ptGrid)) 
                {
                  // check if the distribution in the location 
                  // goes past the threshold
                  rutz::shared_ptr<Histogram> thist = getHistogramDistribution(pt);
                  float diff = hist1->getChiSqDiff(*thist);

                  // if it is below threshold:
                  if(diff < threshDiff)
                    {
                      // LINFO("PASS: %3d difference:(%3d %3d) & (%3d %3d): %f", //DISP
                      //       currIndex, currPoint.i, currPoint.j, 
                      //       pt.i, pt.j, diff);

                      // assign current location to object list
                      currentObjectLocations.push_back(ptGrid);
                      assignmentMap.setVal(ptGrid, CENTER_REGION);
                      
                      // add its neighbors to unassigned locations list
                      uint bSize = currentUnassignedLocations.size();
                      addUnvisitedNeighbors
                        (currentUnassignedLocations, ptGrid, 
                         assignmentMap);
                      uint aSize = currentUnassignedLocations.size();
                      uint addN = aSize - bSize;

                      // take out the location from unassigned list
                      culItr = currentUnassignedLocations.begin();
                      currentUnassignedLocations.erase(culItr+currIndex+addN);

                      currIndex = 0;

                      // // display current situation
                      // displayGrowMap(assignmentMap, currPointGrid,
                      //                currentObjectLocations,
                      //                currentBackgroundLocations,
                      //                currentUnassignedLocations,
                      //                currentIncludedEdgelLocations,
                      //                isComparedWith, segment,
                      //                regionAssignmentMap,
                      //                coreAssignmentMap);

                      // PAMI12: get the unvisited neighbor's neighbor
                      //         imagine that the neighbor is a boundary 
                      //         check if we can add the neighbor's neighbor 
                      //         as a background
                      //bool addingUNN = 
                        checkUnvisitedNeighborsNeigbor
                        (ptGrid, assignmentMap, currentBackgroundLocations,
                         hist1, hist2, hist3, thist, borderDiff, ratio, diff);
                      
                      // display current situation
                      //if(addingUNN) 
                        // displayGrowMap(assignmentMap, currPointGrid,
                        //                currentObjectLocations,
                        //                currentBackgroundLocations,
                        //                currentUnassignedLocations,
                        //                currentIncludedEdgelLocations,
                        //                isComparedWith, segment,
                        //                regionAssignmentMap,
                        //                coreAssignmentMap);
                    }

                  // didn't pass just advance the index
                  else currIndex++;

                  // make note that this location is already compared
                  isComparedWith.setVal(ptGrid, true);
                }

              // already checked, just advance the index
              else currIndex++;
            }
        }

      // if we reach the segment
      if(pointGrids.size() != 0)
        {
          // display current situation
          // displayGrowMap(assignmentMap, currPointGrid,
          //                currentObjectLocations,
          //                currentBackgroundLocations,
          //                currentUnassignedLocations,
          //                currentIncludedEdgelLocations,
          //                isComparedWith, segment,
          //                regionAssignmentMap,
          //                coreAssignmentMap);
          
          // assign regions next to the segment
          assignRegionsAroundSegment
            (segment, pts2, pts3, 
             assignmentMap,
             coreAssignmentMap,
             currentObjectLocations,
             currentBackgroundLocations,
             currentUnassignedLocations,
             currentIncludedEdgelLocations,
             regionAssignmentMap,
             currentObjectRegionList, 
             currentBackgroundRegionList);

          // display current situation
          // displayGrowMap(assignmentMap, currPointGrid,
          //                currentObjectLocations,
          //                currentBackgroundLocations,
          //                currentUnassignedLocations,
          //                currentIncludedEdgelLocations,
          //                isComparedWith, segment,
          //                regionAssignmentMap,
          //                coreAssignmentMap);
        }

      //LINFO("Done with segment {%3d} --> %3d left", // DISP
      //      segment.orgIndex, int(sortedSegments.size()));

      // get the next core region
      // FIXXX: we can quit faster 
      //        like after we failed to reach an edge
      bool moveToNextCore = false;
      if(sortedSegments.size() == 0) moveToNextCore = true;

      if(moveToNextCore)  
        {
          //LINFO("MOVE TO NEXT CORE"); // DISP

          // set the current point as a covered center (a core)
          coreAssignmentMap.setVal(currPointGrid, 
                                   CENTER_CORE_REGION);

          uint index = 0;
          currPointGrid = 
            evaluateCoreAssignment
            (currentObjectLocations, assignmentMap, coreAssignmentMap, index);

          // // display current situation
          // LINFO("before GROW SURROUND");
          // displayGrowMap(assignmentMap, currPointGrid,
          //                currentObjectLocations,
          //                currentBackgroundLocations,
          //                currentUnassignedLocations,
          //                currentIncludedEdgelLocations,
          //                isComparedWith, segment,
          //                regionAssignmentMap,
          //                coreAssignmentMap);

          // GROW SURROUND 
          // if any unassigned neighbors can be added to BG list          
          growSurround
            (assignmentMap,
             currentObjectLocations,
             currentBackgroundLocations,
             currentUnassignedLocations,
             currentIncludedEdgelLocations,
             coreAssignmentMap);

          currPointGrid = 
            evaluateCoreAssignment
            (currentObjectLocations, assignmentMap, coreAssignmentMap, index);

          // LINFO("after  GROW SURROUND"); //DISP
          // // DISP IMAGE
          // displayGrowMap(assignmentMap, currPointGrid,
          //                currentObjectLocations,
          //                currentBackgroundLocations,
          //                currentUnassignedLocations,
          //                currentIncludedEdgelLocations,
          //                isComparedWith, segment,
          //                regionAssignmentMap,
          //                coreAssignmentMap);

          // // imagine new contours from the current contours
          // // then grow using them
          // // in PAMI 2012, NOT in CVPR 2012
          // growUsingImaginedContours
          //   (assignmentMap,
          //    currentObjectLocations,
          //    currentBackgroundLocations,
          //    currentUnassignedLocations,
          //    currentIncludedEdgelLocations,
          //    coreAssignmentMap);

          // LINFO("after   IMAGINED CONTOURS"); //DISP
          // // DISP IMAGE
          // displayGrowMap(assignmentMap, currPointGrid,
          //                currentObjectLocations,
          //                currentBackgroundLocations,
          //                currentUnassignedLocations,
          //                currentIncludedEdgelLocations,
          //                isComparedWith, segment,
          //                regionAssignmentMap,
          //                coreAssignmentMap);
         
          // clear sorted segment
          sortedSegments.clear();
          
          // keep going through the object location list
          // until there is a non-empty segment list
          // FIXXX: this maybe the point to stop growing
          //        for small isolated objects
          while(currPointGrid.isValid() &&                
                sortedSegments.size() == 0 && 
                index < currentObjectLocations.size())
            {
              // refill the segments 
              // with respect to the current point
              currPoint = currPointGrid * BOUNDARY_STEP_SIZE; 
              sortedSegments = fillSegmentQueue(currPoint);

              if(sortedSegments.size() == 0)
                {
                  index++;
                  currPointGrid = 
                    evaluateCoreAssignment
                    (currentObjectLocations, assignmentMap, 
                     coreAssignmentMap, index);
                }
            }

          if(sortedSegments.size() == 0) allCoresCovered = true;

          //LINFO("NEW CORE: %3d %3d", currPointGrid.i, currPointGrid.j); //DISP
        }
    }

  // DISP IMAGE
  // Segment segment(-1, 0, 0, 0);
  // displayGrowMap(assignmentMap, currPointGrid,
  //                currentObjectLocations,
  //                currentBackgroundLocations,
  //                currentUnassignedLocations,
  //                currentIncludedEdgelLocations,
  //                Image<bool>(),  Segment(),
  //                regionAssignmentMap,
  //                coreAssignmentMap);

  // create the current salient region mask
  Image<float> tempSRM(regionAssignmentMap-1);
  inplaceClamp(tempSRM, 0.0F, float(CENTER_REGION-1));
  tempSRM /= float(CENTER_REGION-1);
  Image<float> salientRegionMask = tempSRM;
  itsSalientRegionMask.push_back(Image<byte>(salientRegionMask*255.0));

  // set the core areas as assigned
  icramptr = itsCurrentRegionAssignmentMap.beginw();  
  camptr     = coreAssignmentMap.beginw();
  Image<int>::iterator stopcamptr = coreAssignmentMap.endw();
  while(camptr != stopcamptr)
    {
      if(*camptr == CENTER_CORE_REGION)
        *icramptr = itsCurrentNumSalientRegions; 

      icramptr++; camptr++;
    }
  itsCurrentNumSalientRegions++;

  // set the thin boundary image
  setBoundaryImage(assignmentMap, salientRegionMask);

  return salientRegionMask;
}

// ######################################################################
std::list<Segment> SalientRegionSegmenter::fillSegmentQueue
(Point2D<int> pt)
{
  std::list<Segment> sortedSegments;

  // for each contour
  int index = 0;
  for(uint i = 0; i < itsContourBoundaries.size(); i++)
    {
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[i];
      
      uint cLen = contour->edgels.size();

      // do we take out the short contours???
      if(cLen < MINIMUM_CONTOUR_LENGTH) continue;

      // go through each contour
      float step = float(MINIMUM_CONTOUR_LENGTH)/2.0F;
      uint numSegments = uint(floor(float(cLen)/step));
      if((cLen%MINIMUM_CONTOUR_LENGTH) == 0) numSegments--;
      //LINFO("Contour[%3d]: %3d: numSegments: %d", 
      //      i, cLen, numSegments);

      for(uint j = 0; j < numSegments; j++)
        {
          uint start = uint(j*step);
          uint end   = start + MINIMUM_CONTOUR_LENGTH - 1;

          if(end >= cLen)
            {
              end   = cLen - 1;
              start = end + 1 - MINIMUM_CONTOUR_LENGTH;
            }
         
          Segment segment(index, i, start, end);
          
          bool passThreshold = setPriority(segment, pt);
          if(passThreshold)
            {
              //LINFO("[i: %3d c: %3d s: %3d e: %3d]", index, i, start, end); //DISP
              sortedSegments.push_back(segment);
              index++;
            }
        }
    }

  // sort the segments
  sortedSegments.sort();

  // std::list<Segment>::iterator 
  //   itr  = sortedSegments.begin(), stop = sortedSegments.end();
  // while (itr != stop)
  //   {
  //     LINFO("[%3d] %6.3f*%6.3f + %6.3f*%6.3f + %6.3f*%6.3f + " //DISP
  //           "%6.3f*%6.3f + %6.3f*%6.3f = %6.3f", (*itr).orgIndex,
  //           LEN_WEIGHT, (*itr).lenVal,
  //           SIM_WEIGHT, (*itr).simVal,
  //           STR_WEIGHT, (*itr).strVal,
  //           RAT_WEIGHT, (*itr).ratVal,
  //           DIS_WEIGHT, (*itr).disVal, (*itr).pVal);
  //     itr++;
  //   }

  return sortedSegments;
}

// ######################################################################
// FIXXX: what happen in the second reprioritization
bool SalientRegionSegmenter::setPriority
(Segment &segment, Point2D<int> pt)
{
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();
  float diag = pow(w*w + h*h, 0.5);

  // selection priority is:
  // LEN_WEIGHT * length of the contour boundary +
  // SIM_WEIGHT * similarity with the closer side +
  // STR_WEIGHT * strength of boundary +
  // RAT_WEIGHT * ratio of in:out +
  // DIS_WEIGHT * proximity to the point

  // each factor has a max of 1.0

  // we have a few criteria that has to pass
  // minimum contour length has to be above MINIMUM_CONTOUR_LENGTH
  // that ratio of difference closer/farther side
  bool passThreshold = true;

  rutz::shared_ptr<Contour> contour = 
    itsContourBoundaries[segment.contourIndex];  
  uint cLen = contour->edgels.size();

  // length of the contour boundary 
  float tempCLen = float(cLen);
  float maxLen = float(MINIMUM_CONTOUR_LENGTH * CONTOUR_CHAIN_NUM); 
  if(tempCLen > maxLen) tempCLen = maxLen;
  float lenVal = tempCLen/maxLen;
  segment.lenVal = lenVal; 
  if(tempCLen < MINIMUM_CONTOUR_LENGTH) passThreshold = false;

  // similarity with the closer side 
  rutz::shared_ptr<Histogram> hist1 = getHistogramDistribution(pt);
  std::vector<Point2D<int> > pts2;
  std::vector<Point2D<int> > pts3;
  getSegmentPointDistributions(segment, pt, pts2, pts3);

  rutz::shared_ptr<Histogram> hist2 = getHistogramDistribution(pts2);
  rutz::shared_ptr<Histogram> hist3 = getHistogramDistribution(pts3);
  float diff12 = hist1->getChiSqDiff(*hist2);

  float tempDiff =  diff12;
  if(tempDiff > SIMILARITY_THRESHOLD) tempDiff = SIMILARITY_THRESHOLD; 
  float simVal = 1.0F - tempDiff;
  segment.simVal = simVal;
  if(diff12 > SIMILARITY_THRESHOLD) passThreshold = false;

  // strength of boundary
  float val = 0.0; 
  uint start = segment.segStartIndex;
  uint end   = segment.segEndIndex;
  for(uint i = start; i <= end; i++)
    {
      val += contour->edgels[i]->val;
    }
  val /= float(end - start + 1);
  float maxStr = .5 * itsMaximumEdgelStrength; 
  float strVal = val/maxStr; if(strVal > 1.0F) strVal = 1.0;
  strVal = 1.0F; // <---------- basically cancels the factor
  segment.strVal = strVal;
    
  // ratio of near:far side
  float diff13  =  hist1->getChiSqDiff(*hist3);
  float ratio  =  MAX_FAR_TO_NEAR_RATIO; 
  if(diff12 > 0.0) ratio = diff13/diff12;
  float nratio = MAX_FAR_TO_NEAR_RATIO;
  if(ratio < MAX_FAR_TO_NEAR_RATIO) nratio = ratio; 
  float ratVal = nratio/MAX_FAR_TO_NEAR_RATIO;
  segment.ratVal = ratVal;
  if(ratio < RATIO_THRESHOLD) passThreshold = false;

  // distance between segment and salient point
  int index   = segment.segStartIndex + MINIMUM_CONTOUR_LENGTH/2; 
  float dist  = pt.distance(contour->edgels[index]->pt);
  float tdist1 = DISTANCE_THRESHOLD_1 * diag;
  float tdist2 = DISTANCE_THRESHOLD_2 * diag;
  float disVal  = 1.0; 
  if(dist > tdist1 && dist <= tdist2) 
    disVal = 1.0F - (dist - tdist1)/(tdist2 - tdist1) * 
      (1.0F - DISTANCE_THRESHOLD_VALUE_2);
  else if(dist > tdist2)
    disVal = DISTANCE_THRESHOLD_VALUE_2;
  segment.disVal = disVal;

  segment.pVal =  
    LEN_WEIGHT * lenVal +
    SIM_WEIGHT * simVal +
    STR_WEIGHT * strVal +
    RAT_WEIGHT * ratVal +
    DIS_WEIGHT * disVal;

  if(passThreshold)
    {
      // LINFO("1. clen: %d --> lenVal: %f", cLen, lenVal); //DISP
      // LINFO("2. diff12: %f --> simVal: %f", diff12, simVal);
      // LINFO("3. strength of boundary: %f / %f = strVal: %f", 
      //       val, itsMaximumEdgelStrength, strVal);
      // LINFO("4. ratio in&out: %f/%f = %f --> ratVal: %f", 
      //       diff13, diff12, ratio, ratVal);
      // LINFO("5. distance: dist: %f diag: %f --> disVal: %f", 
      //       dist, diag, disVal);
      // LINFO("Total: %f --> %d\n", segment.pVal, passThreshold);
      
      // displayDistributionSetup(segment, pt, pts2, pts3);
    }
  return passThreshold;
}

// ######################################################################
rutz::shared_ptr<Histogram> 
SalientRegionSegmenter::getHistogramDistribution
(Point2D<int> pt)
{
  Point2D<int> ptGrid
    (pt.i/BOUNDARY_STEP_SIZE, pt.j/BOUNDARY_STEP_SIZE);
  rutz::shared_ptr<Histogram> histogram = 
    itsImageFeatureHistograms.getVal(ptGrid);  
  if(histogram.is_valid()) return histogram;

  itsHistogramGenerationCountPt++;
  Timer tim(1000000); tim.reset();

  Histogram lHist(NUM_L_BINS);
  Histogram aHist(NUM_A_BINS);
  Histogram bHist(NUM_L_BINS);

  int rad = NEIGHBORHOOD_RADIUS;
  int numPoints = 0;
  for(int i = -rad; i <= rad; i++) 
    for(int j = -rad; j <= rad; j++) 
      {
        bool isWithinCircle = ((i*i+j*j) <= (rad*rad));

        Point2D<int> pt2 = pt + Point2D<int>(i,j);
        if(itsImage.coordsOk(pt2) && isWithinCircle)
          {
            // L component
            int l_val = itsImageHistogramEntries[0].getVal(pt2);
            lHist.addValue(l_val, 1.0F);

            // a component
            int a_val = itsImageHistogramEntries[1].getVal(pt2);
            aHist.addValue(a_val, 1.0F);

            // b component
            int b_val = itsImageHistogramEntries[2].getVal(pt2);
            bHist.addValue(b_val, 1.0F);

            numPoints++;
          }
      }

  // maybe later want to add the weight for each channel
  //(Point2D<int> pt, std::vector<float> &dist, float weight)

  // smooth the histogram individually
  lHist.smooth(itsLKernel);
  aHist.smooth(itsAKernel);
  bHist.smooth(itsBKernel);

  // combine the histograms
  std::vector<Histogram> histograms;
  histograms.push_back(lHist);
  histograms.push_back(aHist);
  histograms.push_back(bHist);
  histogram.reset(new Histogram(histograms));
  
  // normalize with the number of points
  histogram->divide(numPoints);
  itsImageFeatureHistograms.setVal(ptGrid, histogram);

  itsTimePt += tim.get();

  return histogram;
}

// ######################################################################
rutz::shared_ptr<Histogram> 
SalientRegionSegmenter::getHistogramDistribution
(std::vector<Point2D<int> > pts)
{
  itsHistogramGenerationCountPts++;
  Timer tim(1000000); tim.reset();

  Histogram lHist(NUM_L_BINS);
  Histogram aHist(NUM_A_BINS);
  Histogram bHist(NUM_L_BINS);

  int numPoints = 0;
  for(uint i = 0; i < pts.size(); i++) 
      {
        Point2D<int> pt = pts[i];
        if(itsImage.coordsOk(pt))
          {
            // L component
            int l_val = itsImageHistogramEntries[0].getVal(pt);
            lHist.addValue(l_val, 1.0F);

            // a component
            int a_val = itsImageHistogramEntries[1].getVal(pt);
            aHist.addValue(a_val, 1.0F);

            // b component
            int b_val = itsImageHistogramEntries[2].getVal(pt);
            bHist.addValue(b_val, 1.0F);

            numPoints++;
          }
      }

  //LINFO("num points: %d", numPoints);
  
  // maybe later want to add the weight for each channel
  //(Point2D<int> pt, std::vector<float> &dist, float weight)

  // smooth the histogram individually
  lHist.smooth(itsLKernel);
  aHist.smooth(itsAKernel);
  bHist.smooth(itsBKernel);

  // combine the histograms
  std::vector<Histogram> histograms;
  histograms.push_back(lHist);
  histograms.push_back(aHist);
  histograms.push_back(bHist);
  rutz::shared_ptr<Histogram> histogram(new Histogram(histograms));
  
  // normalize with the number of points
  histogram->divide(numPoints);

  itsTimePts += tim.get();

  return histogram;
}

// ######################################################################
void SalientRegionSegmenter::getSegmentPointDistributions
(Segment segment, Point2D<int> pt, 
 std::vector<Point2D<int> > &pts1, std::vector<Point2D<int> > &pts2)
{
  // get the contour divider end points 
  uint start = segment.segStartIndex;
  uint end   = segment.segEndIndex;

  rutz::shared_ptr<Contour> contour = 
    itsContourBoundaries[segment.contourIndex];  

  rutz::shared_ptr<Edgel> edgels = contour->edgels[start];
  rutz::shared_ptr<Edgel> edgele = contour->edgels[end  ];
           
  uint  ainds  = edgels->angleIndex; 
  float bainds = 
    fmod((ainds+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);
  uint  ainde  = edgele->angleIndex; 
  float bainde = 
    fmod((ainde+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);

  int step  = BOUNDARY_STEP_SIZE;
  int hstep = step/2;
  float dxs = cos(bainds * M_PI/4.0) * hstep; 
  float dys = sin(bainds * M_PI/4.0) * hstep;

  float dxe = cos(bainde * M_PI/4.0) * hstep; 
  float dye = sin(bainde * M_PI/4.0) * hstep;

  Point2D<float> p1 = edgels->pt + Point2D<float>(-dxs-.5, -dys-.5);
  Point2D<float> p2 = edgele->pt + Point2D<float>( dxe+.5,  dye+.5); 
  
  // estimate the contour average angle
  float ang  = atan2(p2.j-p1.j, p2.i-p1.i);

  // the corrected angle would be to move it to PI/2
  // FIXXX: this could be incorrect 
  //        if we branch the contour to both left and right
  float cAng = M_PI/2.0F - ang;
  if(ang < 0.0F && ang > M_PI) 
    { LFATAL("out of Bounds Angle -- Fix it"); }

  if(cAng == 0.0F)
    {
      LINFO("Deal with ZERO corrected angle");
      Raster::waitForKey();
    }

  float cCAng = cos(cAng);
  float sCAng = sin(cAng);

  // get the midpoint of the angle line estimator
  Point2D<float> mid((p1.i+p2.i)/2.0, (p1.j+p2.j)/2.0);  

  // LINFO("c[%3d]:(%3d:a: %f)-(%3d:a: %3f) ", 
  //       segment.contourIndex, start, bainds, end, bainde);
  // LINFO("hstep: %d [%f %f] [%f %f]", 
  //       hstep, dxs, dys, dxe, dye);
  
  // LINFO("p1:(%f %f) p2:(%f %f) ", 
  //       p1.i, p1.j, p2.i, p2.j);
  // LINFO("ang: %f --> cAng: %f (c: %f s: %f) mid[%f %f] ", 
  //       ang, cAng, cCAng, sCAng, mid.i, mid.j);

  // rotate about that point to vertical
  float tbR = 0.0; float bbR = 0.0; 
  float lbR = 0.0; float rbR = 0.0;
  std::vector<Point2D<float> > sPt;
  std::vector<Point2D<float> > ePt;
  std::vector<Point2D<float> > srPt;
  std::vector<Point2D<float> > erPt;
  std::vector<Point2D<float> > mrPt;
  for(uint k = start; k <= end; k++)
    {
      Point2D<int> pt = contour->edgels[k]->pt;
      float i = pt.i - mid.i;
      float j = pt.j - mid.j; 
      float iR =  i*cCAng - j*sCAng;
      float jR =  i*sCAng + j*cCAng;

      uint aind   = contour->edgels[k]->angleIndex; 
      float baind = 
        fmod((aind+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);
      float dx = cos(baind * M_PI/4.0) * hstep; 
      float dy = sin(baind * M_PI/4.0) * hstep;

      Point2D<float> ps = pt + Point2D<float>(-dx-.5, -dy-.5);
      Point2D<float> pe = pt + Point2D<float>( dx+.5,  dy+.5); 
      
      float si = ps.i - mid.i;
      float sj = ps.j - mid.j; 
      float siR =  si*cCAng - sj*sCAng;
      float sjR =  si*sCAng + sj*cCAng;
      
      float ei = pe.i - mid.i;
      float ej = pe.j - mid.j; 
      float eiR =  ei*cCAng - ej*sCAng;
      float ejR =  ei*sCAng + ej*cCAng;

      sPt.push_back(ps);
      ePt.push_back(pe);
      srPt.push_back(Point2D<float>(siR,sjR));
      erPt.push_back(Point2D<float>(eiR,ejR));
      mrPt.push_back(Point2D<float>( iR, jR));

      // initialize everything on the first iteration
      if(k == start)
        {
          tbR = sjR;
          bbR = sjR;
          lbR = siR;
          rbR = siR;
        }

      if(tbR > sjR) tbR = sjR;
      if(tbR > ejR) tbR = ejR;

      if(bbR < sjR) bbR = sjR;
      if(bbR < ejR) bbR = ejR;

      if(lbR > siR) lbR = siR;
      if(lbR > eiR) lbR = eiR;

      if(rbR < siR) rbR = siR;
      if(rbR < eiR) rbR = eiR;

      // LINFO("[[%d]]", k);
      // LINFO("M[%3d %3d] --> offset[%f %f] --> [%f %f]", 
      //       pt.i, pt.j, i ,j, iR, jR);

      // LINFO("S[%f %f] --> offset[%f %f] --> [%f %f]", 
      //       ps.i, ps.j, si ,sj, siR, sjR);
      // LINFO("E[%f %f] --> offset[%f %f] --> [%f %f] \n", 
      //       pe.i, pe.j, ei ,ej, eiR, ejR);
    }

  //LINFO("[t: %f b: %f l: %f r: %f] ", tbR, bbR, lbR, rbR);

  // count the pixels in the left and right within the bounding box  
  float areaL = 0; float areaR = 0;
  float w = rbR - lbR;
  float h = bbR - tbR;
  if(w > 0.0F && h > 0.0F)
    {
      for(uint k = start; k <= end; k++)
        {
          int i = k-start;
          // we will use a trapezoid area 
          // to approximate number of pixels
          
          float tl = rbR - srPt[i].i;
          float bl = rbR - erPt[i].i;
          
          float h = erPt[i].j - srPt[i].j;
          
          float trapArea = (tl+bl)*h/2.0F;
          float boxArea  = w*h;
          areaL += (boxArea-trapArea);
          areaR += trapArea;
          
          // LINFO("tl = %f = %f - %f||| bl = %f = %f - %f ",
          //       tl, rbR, srPt[i].i, bl, rbR, erPt[i].i);
          // LINFO("h = %f = %f - %f | w = %f = %f - %f",
          //       h, erPt[i].j, srPt[i].j, w, rbR, lbR);

          // LINFO("[%d] b: %f b-t: %f, t: %f", 
          //       k, boxArea, boxArea-trapArea, trapArea);
        }
    }
  // LINFO("areaL: %f, areaR: %f ==> h: %f", areaL, areaR, h);

  // extend the box so that the number of pixels on both sides 
  // are close to the area circle with radius = NEIGHBORHOOD_RADIUS
  uint nrad = NEIGHBORHOOD_RADIUS;
  float minArea = nrad * nrad * M_PI; 
  float extL = 0.0; float extR = 0.0;

  //LINFO("lbR: %f areaL: %f", lbR, areaL);
  //LINFO("rbR: %f areaR: %f", rbR, areaR);
  if(areaL < minArea)
    {
      extL = lbR - (minArea - areaL)/h;
    }

  if(areaR < minArea)
    {
      extR = rbR + (minArea - areaR)/h;
    }

  //LINFO("add L:%f ->  %f add R: %f -> %f", 
  //      (minArea - areaL), (minArea - areaL)/h,
  //      (minArea - areaR), (minArea - areaR)/h );

  //LINFO("Sanity Check: min Area: %f, Area: ((%f) - (%f))*%f = %f", 
  //      minArea, extR, extL, h, (extR - extL)*h);

  // figure out which side the pt is
  // FIXXX: may want to be more careful here
  int pti = pt.i - mid.i;
  int ptj = pt.j - mid.j; 
  float ptiR =  pti*cCAng - ptj*sCAng;
  //float ptjR =  pti*sCAng + ptj*cCAng;
  float isLeft = true; if(ptiR > 0.0) isLeft = false;
  //LINFO("isLeft: %d", int(isLeft));

  // create a box
  uint finalW = extR-extL;
  uint finalH = h;
  uint maxDim = finalW;
  if(finalH > finalW) maxDim = finalH;
  Image<byte> temp1(maxDim*2, maxDim*2, ZEROS);
  Image<byte> temp2(maxDim*2, maxDim*2, ZEROS);

  // LINFO("finalW: %d finalH: %d ==> maxDim: %d", finalW, finalH, maxDim);
  //uint ttt1 = 0; uint ttt2 = 0;
  int prevJ = -1;
  for(uint k = start; k <= end; k++)
    {
      uint ind = k - start;

      float t  = srPt[ind].j; float b  =  erPt[ind].j;      
      float h  = b - t;

      float tH = srPt[ind].i; float bH =  erPt[ind].i;
      float w  = bH - tH;

      //LINFO("ind: %d h: %f = %f - %f w: %f = %f - %f", 
      //      ind, h, b, t, w, bH, tH);

      // get the pixels on the left and right side
      // LINFO("VERT range: %d to %d", int(t), int(b));
      //uint tt1 = 0; uint tt2 = 0;

      if(t > prevJ+1) t = prevJ+1;
      for(int j = int(t); j <= int(b + .5F); j++)
        {
          // compute the dividing edge location
          float div = (w/h*(j - b)) + tH;
          if(w < 0.0F) div = (w/h*(j - t)) + tH;

          float leftE  = extL+maxDim;
          float rightE = extR+maxDim;

          // LINFO("horizontal divider: %f, range: %f to %f", 
          //      div+maxDim, leftE, rightE);

          //uint t1 = 0; uint t2 = 0;
          uint jShift = j + maxDim;


          for(int i = int(leftE); i <= int(rightE+.5F); i++) 
            {
          // LINFO("[%3d %3d] [%3d %3d] = i %d jshift %d", 
          //       temp1.getWidth(), temp1.getHeight(),
          //       temp2.getWidth(), temp2.getHeight(), i, jShift);

              if(!temp1.coordsOk(i,jShift)) continue;

              if(i < (div+maxDim)) 
                {
                  if(isLeft) temp1.setVal(i,jShift, 128); //t1++; }
                  else       temp2.setVal(i,jShift, 128); //t2++; }
                }
              else
                {
                  if(isLeft) temp2.setVal(i,jShift, 128); //t2++; }
                  else       temp1.setVal(i,jShift, 128); //t1++; }
                }

              //LINFO("i:%d %d: div: %f --> %d == %3d %3d", 
              //      i, jShift, div+maxDim, int(i<(div+maxDim)), t1, t2);
            }
          //LINFO("[%3d %3d] t1: %d t2: %d", k, j, t1, t2);
          //tt1 += t1;
          //tt2 += t2;

          prevJ = j;
        }
      //LINFO("[%3d] t1: %d t2: %d", k, tt1, tt2);
      //ttt1 += tt1;
      //ttt2 += tt2;
    }
  //LINFO("ttt1: %d ttt2: %d", ttt1, ttt2);

  //LINFO("sum1: %f sum2: %f", 
  //      float(sum(temp1))/128.0F, float(sum(temp2))/128.0F);

  // re-rotate the bounding box
  Image<byte> temp1R = rotate(temp1, maxDim, maxDim, -cAng);
  Image<byte> temp2R = rotate(temp2, maxDim, maxDim, -cAng);

  //LINFO("md[%d] mid: %f %f", maxDim, mid.i, mid.j);

  // figure out which side is next to the point
  int tw = temp1.getWidth();
  int th = temp1.getHeight();
  for(int i = 0; i < tw; i++)
    for(int j = 0; j < th; j++)
      {
        if(temp1R.getVal(i,j) > 64)
          { 
            Point2D<int> pt(i+int(mid.i)-maxDim,
                            j+int(mid.j)-maxDim );
            if(itsImage.coordsOk(pt)) pts1.push_back(pt); 
            //LINFO("1 %d %d --> %d %d", i,j,pt.i, pt.j);
            //Raster::waitForKey();
          }

        if(temp2R.getVal(i,j) > 64) 
          {
            Point2D<int> pt(i+int(mid.i)-maxDim,
                            j+int(mid.j)-maxDim );
            if(itsImage.coordsOk(pt)) pts2.push_back(pt); 
            //LINFO("2 %d %d --> %d %d", i,j,pt.i, pt.j);
            //Raster::waitForKey();
          }
      }

  //LINFO("Final Size: %d %d", int(pts1.size()), int(pts2.size()));
}

// ######################################################################
void SalientRegionSegmenter::filterSegments
(std::list<Segment> &segments, Point2D<int> ptGrid, Image<int> assignmentMap)
{
  uint hstep = BOUNDARY_STEP_SIZE/2;
  
  // go through each point
  std::list<Segment>::iterator itr  = segments.begin(), stop = segments.end();
  while(itr != stop)
    {
      Segment segment = (*itr);
      bool erasing = true;
 
      // get the segment center 
      int index = segment.segStartIndex + MINIMUM_CONTOUR_LENGTH/2; 
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[segment.contourIndex];  
      rutz::shared_ptr<Edgel> edgel = contour->edgels[index];
      Point2D<int> segCenterPointGrid
        ((edgel->pt.i+hstep)/BOUNDARY_STEP_SIZE,
         (edgel->pt.j+hstep)/BOUNDARY_STEP_SIZE );
      std::vector<Point2D<int> > points = 
        getLine(ptGrid, segCenterPointGrid);

      //LINFO("segment points");
      for(uint i = segment.segStartIndex; i <= segment.segEndIndex; i++) 
        {
          rutz::shared_ptr<Edgel> tedgel = contour->edgels[i];
          Point2D<int> tptGrid
            ((tedgel->pt.i+hstep)/BOUNDARY_STEP_SIZE,
             (tedgel->pt.j+hstep)/BOUNDARY_STEP_SIZE );
           
          // LINFO("[<<%3d>>] %d %d --> %d %d", //DISP
          //       i, tptGrid.i, tptGrid.j,
          //       tedgel->pt.i,tedgel->pt.j);
        }

      // FIXXX: while NOT going through a surround area
      bool isOnSurround = false;

      // LINFO("LINE: %3d %3d to %3d %3d", //DISP
      //       ptGrid.i, ptGrid.j, 
      //       segCenterPointGrid.i, segCenterPointGrid.j);
      for(uint i = 0; i < points.size(); i++)
        {
          // LINFO("[%3d]: %3d %3d", i, points[i].i, points[i].j); //DISP
          if(assignmentMap.getVal(points[i]) == SURROUND_REGION)
            {
              isOnSurround = false;
              i = points.size();
            }
        }

      if(isOnSurround)
        {
          // LINFO("erasing segment:%d", (*itr).orgIndex); //DISP
          displaySegment(segment); 
          itr  = segments.erase(itr); 
          stop = segments.end();
          erasing = true;

          // LINFO("now at:%d", (*itr).orgIndex); //DISP
        }

      if(!erasing) itr++;
      //LINFO("then move to:%d", (*itr).orgIndex); // DISP
    }
}

// ######################################################################
void SalientRegionSegmenter::addUnvisitedNeighbors
(std::vector<Point2D<int> > &currentUnassignedLocations, 
 Point2D<int> currPointGrid, Image<int> &assignmentMap )
{
  // add to the front of the list

  for(int j = 1; j >= -1; j--)
    for(int i = 1; i >= -1; i--)
      if(!(i == 0 && j == 0))
        {
          Point2D<int> pt(currPointGrid.i+i, currPointGrid.j+j);
          
          // if the location is not yet visited
          if(assignmentMap.coordsOk(pt) && 
             assignmentMap.getVal(pt) == UNVISITED_REGION) 
            {
              std::vector<Point2D<int> >::iterator itr = 
                currentUnassignedLocations.begin();
              currentUnassignedLocations.insert(itr, pt);

              assignmentMap.setVal(pt, UNASSIGNED_REGION);
            }
        }
}

// ######################################################################
bool SalientRegionSegmenter::checkUnvisitedNeighborsNeigbor
(Point2D<int> currPointGrid, Image<int> &assignmentMap,
 std::vector<Point2D<int> > &currentBackgroundLocations,
 rutz::shared_ptr<Histogram> histA,
 rutz::shared_ptr<Histogram> histBobj,
 rutz::shared_ptr<Histogram> histBbg,
 rutz::shared_ptr<Histogram> histCobj,
 float borderDiff, float ratio, float threshDiff2)
{
  if(threshDiff2 == 0.0F) return false;

  // get the neighbor's neighbor
  std::vector<Point2D<int> > nn;
  for(int j = -2; j <= 2; j++)
    for(int i = -2; i <= 2; i++)
      {
        if((i >= -1 && i <= 1) && (j >= -1 && j <= 1)) continue;
          
          Point2D<int> pt(currPointGrid.i+i, currPointGrid.j+j);
          
          // if the location is not yet visited
          if(assignmentMap.coordsOk(pt) && 
             assignmentMap.getVal(pt) == UNVISITED_REGION) 
            nn.push_back(pt);
      }

  //LINFO("borderDiff: %f ratio: %f", borderDiff, ratio);

  // go through each neighbor's neighbor (nn)
  bool addingUNN = false;
  for(uint i = 0; i < nn.size(); i++)
    {
      rutz::shared_ptr<Histogram> histCbg = 
        getHistogramDistribution(nn[i]*BOUNDARY_STEP_SIZE);

      // for checking Bobj-Bbg < Cobj-Cbg
      float bDiff  = histCobj->getChiSqDiff(*histCbg);      

      // for checking ratio: A-Bbg / A-Bobj < A-Cbg / A-Cobj
      float bDiff2 = histA   ->getChiSqDiff(*histCbg);
      float cratio = bDiff2/threshDiff2;

      // // passes 
      if(borderDiff < bDiff && ratio < cratio)
        {
          assignmentMap.setVal(nn[i], SURROUND_REGION);
          currentBackgroundLocations.push_back(nn[i]);
          //LINFO("[%d %d] borderDiff %f < bDiff: %f && "
          //      "ratio: %f < cratio: %f",   
          //      nn[i].i, nn[i].j, borderDiff, bDiff, ratio, cratio);
          addingUNN = true;
        }
    }
  return addingUNN;
}

// ######################################################################
std::vector<Point2D<int> > SalientRegionSegmenter::getLine
(Point2D<int> p1, Point2D<int> p2)
{
  std::vector<Point2D<int> > points;

  // ray tracing algorithm
  // from Graphics Gems / Paul Heckbert
  int dx = p2.i - p1.i, ax = abs(dx) << 1, sx = signOf(dx);
  int dy = p2.j - p1.j, ay = abs(dy) << 1, sy = signOf(dy);
  int x = p1.i, y = p1.j;

  const int w = itsImage.getWidth();
  const int h = itsImage.getHeight();

  if (ax > ay)
    {
      int d = ay - (ax >> 1);
      for (;;)
        {
          // assignment
          if (x >= 0 && x < w && y >= 0 && y < h)
            points.push_back(Point2D<int>(x,y));

          if (x == p2.i) return points;
          if (d >= 0) { y += sy; d -= ax; }
          x += sx; d += ay;
        }
    }
  else
    {
      int d = ax - (ay >> 1);
      for (;;)
        {
          // assignment
          if (x >= 0 && x < w && y >= 0 && y < h)
            points.push_back(Point2D<int>(x,y));

          if (y == p2.j) return points;
          if (d >= 0) { x += sx; d -= ay; }
          y += sy; d += ax;
        }
    }
  
  return points;
}

// ######################################################################
bool SalientRegionSegmenter::throughRegion
(Point2D<int> ptGrid1, Point2D<int> ptGrid2, 
 Image<int> assigmentMap, int regionLabel)
{
  std::vector<Point2D<int> > ptGrids = getLine(ptGrid1, ptGrid2);
  for(uint i = 0; i < ptGrids.size(); i++)
    {
      if(assigmentMap.getVal(ptGrids[i]) == regionLabel) 
        return true; 
    }

  return false;
}

// ######################################################################
void SalientRegionSegmenter::assignRegionsAroundSegment
(Segment segment, 
 std::vector<Point2D<int> > pts2, 
 std::vector<Point2D<int> > pts3, 
 Image<int>                 &assignmentMap,
 Image<int>                 &coreAssignmentMap,
 std::vector<Point2D<int> > &currentObjectLocations,
 std::vector<Point2D<int> > &currentBackgroundLocations,
 std::vector<Point2D<int> > &currentUnassignedLocations,
 std::vector<Point2D<int> > &currentIncludedEdgelLocations,
 Image<int>       &regionAssignmentMap,
 std::vector<int> &currentObjectRegionList, 
 std::vector<int> &currentBackgroundRegionList)
{
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();

  int step  = BOUNDARY_STEP_SIZE;
  int hstep = BOUNDARY_STEP_SIZE/2;
  int wG = (w+hstep)/BOUNDARY_STEP_SIZE;
  int hG = (h+hstep)/BOUNDARY_STEP_SIZE;

  Image<int> bimage(w,h, ZEROS);
  for(uint i = 0; i < pts2.size(); i++) 
    {
      if(bimage.coordsOk(pts2[i]))
        bimage.setVal(pts2[i], CLOSER_SIDE);
    }
  for(uint i = 0; i < pts3.size(); i++) 
    {
      if(bimage.coordsOk(pts3[i]))
        bimage.setVal(pts3[i], FAR_SIDE);
    }
  std::vector<std::vector<Point2D<int> > > 
    neighbor(NUM_RIDGE_DIRECTIONS);

  // neighbor for assignment of center/surround
  for(uint k = 0; k < NUM_RIDGE_DIRECTIONS; k++)
    {      
      neighbor[k] = std::vector<Point2D<int> >();

      if(k == 0)
        {
          neighbor[k].push_back(Point2D<int>( 1,  0));
          neighbor[k].push_back(Point2D<int>(-1,  0));
        }
      else if(k == 1)
        {
          neighbor[k].push_back(Point2D<int>(-1, -1));
          neighbor[k].push_back(Point2D<int>( 0, -1));
          neighbor[k].push_back(Point2D<int>(-1,  0));
          neighbor[k].push_back(Point2D<int>( 1,  0));
          neighbor[k].push_back(Point2D<int>( 0,  1));
          neighbor[k].push_back(Point2D<int>( 1,  1));
        }
      else if(k == 2)
        {
          neighbor[k].push_back(Point2D<int>( 0,  1));
          neighbor[k].push_back(Point2D<int>( 0, -1));
        }
      else if(k == 3)
        {
          neighbor[k].push_back(Point2D<int>( 0, -1));
          neighbor[k].push_back(Point2D<int>( 1, -1));
          neighbor[k].push_back(Point2D<int>( 1,  0));
          neighbor[k].push_back(Point2D<int>(-1,  0));
          neighbor[k].push_back(Point2D<int>(-1,  1));
          neighbor[k].push_back(Point2D<int>( 0,  1));
        }
    }

  std::vector<Point2D<int> > currCenterPts;
  std::vector<Point2D<int> > currSurroundPts;

  // add edgel locations to the image
  for(uint e = segment.segStartIndex; e <= segment.segEndIndex; e++) 
    {
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[segment.contourIndex];  
      rutz::shared_ptr<Edgel> edgel = contour->edgels[e];           
      Point2D<int> pt = edgel->pt;
       Point2D<int> ptGrid
        ((pt.i+hstep)/BOUNDARY_STEP_SIZE,
         (pt.j+hstep)/BOUNDARY_STEP_SIZE );      

      bool inCIEL = false;
      for(uint i = 0; i < currentIncludedEdgelLocations.size(); i++)
          if(currentIncludedEdgelLocations[i] == ptGrid)
            { inCIEL = true; i = currentIncludedEdgelLocations.size(); }

      if(!inCIEL && 
         assignmentMap.coordsOk(ptGrid) &&
         (assignmentMap.getVal(ptGrid) != CENTER_REGION))
        currentIncludedEdgelLocations.push_back(ptGrid);
    }
  
  // go through all the edgels' grid location first
  for(uint e = segment.segStartIndex; e <= segment.segEndIndex; e++) 
    {
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[segment.contourIndex];  
      rutz::shared_ptr<Edgel> edgel = contour->edgels[e];           
      Point2D<int> pt = edgel->pt;
      Point2D<int> ptGrid
        ((pt.i+hstep)/BOUNDARY_STEP_SIZE,
         (pt.j+hstep)/BOUNDARY_STEP_SIZE );      
      //LINFO("[[[%3d]]] %d %d --> %d %d", e, pt.i, pt.j, ptGrid.i, ptGrid.j);
      
      if(!assignmentMap.coordsOk(ptGrid)) continue;      
      uint label = assignmentMap.getVal(ptGrid);

      // do nothing on center, surround, other objects 
      // if(label == CENTER_REGION)         LINFO(" C_REGION  : do nothing");
      // if(label == SURROUND_REGION)       LINFO(" S_REGION  : do nothing");
      // if(label == OTHER_OBJECT_REGION)   LINFO(" OO_REGION : do nothing");
      
      if(label == UNVISITED_REGION)
        {
          // add to background
          currentBackgroundLocations.push_back(ptGrid);
          assignmentMap.setVal(ptGrid, SURROUND_REGION);         
          currSurroundPts.push_back(ptGrid);

          //LINFO("UNVISITED adding %3d %3d to the background", 
          //      ptGrid.i, ptGrid.j);
        }

      if(label == UNASSIGNED_REGION)
        {
          // find it is in the unassigned list
          uint pos = 0;
          for(uint i = 0; i < currentUnassignedLocations.size(); i++)
            if(ptGrid == currentUnassignedLocations[i]) 
              { pos = i; i = currentUnassignedLocations.size(); }

          // delete it
          std::vector<Point2D<int> >::iterator itr = 
            currentUnassignedLocations.begin();
          currentUnassignedLocations.erase(itr+pos); 
          
          // add to background list
          currentBackgroundLocations.push_back(ptGrid);
          assignmentMap.setVal(ptGrid, SURROUND_REGION);    
          currSurroundPts.push_back(ptGrid);
        }
    }
 
  // then go through its neighbors
  for(uint e = segment.segStartIndex; e <= segment.segEndIndex; e++) 
    {
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[segment.contourIndex];  
      rutz::shared_ptr<Edgel> edgel = contour->edgels[e];           
      Point2D<int> pt = edgel->pt;
      Point2D<int> ptGrid
        ((pt.i+hstep)/BOUNDARY_STEP_SIZE,
         (pt.j+hstep)/BOUNDARY_STEP_SIZE );
      
      uint dir = edgel->angleIndex; 

      //LINFO("[<<%3d>>] %d %d --> %d %d (%d)", 
      //      e,  pt.i, pt.j, ptGrid.i, ptGrid.j, dir);

      // classify the neighbor location
      for(uint i = 0; i < neighbor[dir].size(); i++)
        {
          Point2D<int> ptGrid2 = ptGrid  + neighbor[dir][i];
          Point2D<int> pt2     = ptGrid2 * step;

          //LINFO("    N[%3d]: %d %d --> %d %d", 
          //      i, ptGrid2.i, ptGrid2.j, pt2.i, pt2.j);

          if(!assignmentMap.coordsOk(ptGrid2)) continue;

          uint label2 = assignmentMap.getVal(ptGrid2);
          //LINFO("LABEL2: %d", label2);

          // do nothing on center, surround, other objects 
          // if(label2 == CENTER_REGION)       LINFO(" C_REGION : do nothing");
          // if(label2 == SURROUND_REGION)     LINFO(" S_REGION : do nothing");
          // if(label2 == OTHER_OBJECT_REGION) LINFO("OO_REGION : do nothing");

          // check what side of the boundary the point is on
          // NOTE: its grid center has to be on either 
          //       closer (pts2) or far (pts3) side list
          int side = bimage.getVal(pt2);

          // NOTE: other way of getting side for a grid point
          // int cclose = 0, cfar = 0;
          // for(int ii = -hstep; ii < hstep; ii++)
          //   for(int jj = -hstep; jj < hstep; jj++)
          //     {
          //       Point2D<int> dpt2(pt2.i+ii, pt2.j+jj);
          //       if (!bimage.coordsOk(dpt2)) continue;
          //       int slabel = bimage.getVal(dpt2);
          //       if(slabel == CLOSER_SIDE) cclose++; 
          //       else if(slabel == FAR_SIDE) cfar++;
          //     }

          // if(cclose < cfar) side = FAR_SIDE;
          // else side = CLOSER_SIDE;

          //LINFO("UN Side: %d", side);
          
          if(label2 == UNVISITED_REGION)
            {
              if(side == CLOSER_SIDE)
                {
                  // add to center/object
                  currentObjectLocations.push_back(ptGrid2);
                  assignmentMap.setVal(ptGrid2, CENTER_REGION);
                  currCenterPts.push_back(ptGrid);

                  // LINFO("  adding %3d %3d to the center", //DISP
                  //       ptGrid2.i, ptGrid2.j);
                }
              else if(side == FAR_SIDE)
                {
                  // add to background
                  currentBackgroundLocations.push_back(ptGrid2);
                  assignmentMap.setVal(ptGrid2, SURROUND_REGION);
                  currSurroundPts.push_back(ptGrid);

                  // LINFO("  adding %3d %3d to the background", //DISP
                  //       ptGrid2.i, ptGrid2.j);
                }
            }
          
          if(label2 == UNASSIGNED_REGION)
            {
              if(side == CLOSER_SIDE || side == FAR_SIDE)
                {                
                  // find the location in the unassigned list
                  uint pos = 0;
                  for(uint i = 0; i < currentUnassignedLocations.size(); i++)
                    if(ptGrid2 == currentUnassignedLocations[i]) 
                      { pos = i; i = currentUnassignedLocations.size(); }
                  
                  // delete it
                  std::vector<Point2D<int> >::iterator itr = 
                    currentUnassignedLocations.begin();
                  currentUnassignedLocations.erase(itr+pos); 

                  if(side == CLOSER_SIDE)
                    {
                      // add to center/object
                      currentObjectLocations.push_back(ptGrid2);
                      assignmentMap.setVal(ptGrid2, CENTER_REGION);
                      currCenterPts.push_back(ptGrid);
                      
                      // LINFO("  adding %3d %3d to the center", //DISP
                      //       ptGrid2.i, ptGrid2.j);
                    }
                  else if(side == FAR_SIDE)
                    {
                      // add to background
                      currentBackgroundLocations.push_back(ptGrid2);
                      assignmentMap.setVal(ptGrid2, SURROUND_REGION);
                      currSurroundPts.push_back(ptGrid);
                      
                      // LINFO("  adding %3d %3d to the background", //DISP
                      //       ptGrid2.i, ptGrid2.j);
                    }                  
                }
              //else LINFO("not on the close or far side");
           }
        }
    }

  // ==================================================================
  // get all the oversegmented regions on the borders

  // for object/center
  float cSize = float(pts2.size());
  std::vector<int> tempC;
  for(uint i = 0; i < pts2.size(); i++)
    {
      int label = itsInitialRegionImage.getVal(pts2[i]);

      // if already included in the map
      if(regionAssignmentMap.getVal(pts2[i]) != UNVISITED_REGION) 
        continue;

      // check if the region is already added
      // in previous iteration
      bool in = false;
      for(uint j = 0; j < tempC.size(); j++)
        if(tempC[j] == label)
          { in = true; j = tempC.size(); }

      if(!in) 
        {
          // calculate the overlap on the region
          float rSize = float(itsInitialRegions[label].size());
          float nO = 0;
          for(uint j = 0; j < pts2.size(); j++) 
            {
              Point2D<int> pt = pts2[j];
              int tlabel = itsInitialRegionImage.getVal(pt);

              if(tlabel == label) nO+= 1.0F;
            }

          // anything over REGION_INCLUDE_THRESHOLD 
          // of total bounding box is good
          // --> ok because the region is similar appearance-wise 
          if((nO/cSize) > REGION_INCLUDE_THRESHOLD || 
             (nO/rSize) > REGION_INCLUDE_THRESHOLD   ) 
            tempC.push_back(label);
        }
    }

  // for background/surround 
  float sSize = float(pts3.size());
  std::vector<int> tempS;
  for(uint i = 0; i < pts3.size(); i++)
    {
      int label = itsInitialRegionImage.getVal(pts3[i]);

      // if already included in the map
      if(regionAssignmentMap.getVal(pts3[i]) != UNVISITED_REGION) 
        continue;

      // check if the region is already added
      // in previous iteration
      bool in = false;
      for(uint j = 0; j < tempS.size(); j++)
        if(tempS[j] == label)
          { in = true; j = tempS.size(); }

      if(!in) 
        {
          // calculate the overlap on the region
          float rSize = float(itsInitialRegions[label].size());
          float nO = 0;
          for(uint j = 0; j < pts3.size(); j++) 
            {
              Point2D<int> pt = pts3[j];
              int tlabel = itsInitialRegionImage.getVal(pt);


              if(tlabel == label) nO+= 1.0F;
            }

          // anything over 50% of total bounding box is good
          // --> ok because the region is similar appearance-wise 
          if((nO/sSize) > REGION_INCLUDE_THRESHOLD || 
             (nO/rSize) > REGION_INCLUDE_THRESHOLD   ) 
            tempS.push_back(label);
        }
    }

  // // take out object region from surround if needed            
  // std::vector<int>::iterator 
  //   itr  = currentBackgroundRegionList.begin(), 
  //   stop = currentBackgroundRegionList.end();
  // while(itr != stop)
  //   {
  //     bool erasing = false;
  //     for(uint i = 0; i < tempC.size(); i++)
  //       {  
  //         if(tempC[i] == (*itr))
  //           {
  //             // take out the region from the background list
  //             itr  = currentBackgroundRegionList.erase(itr);
  //             stop = currentBackgroundRegionList.end();
  //             erasing = true;
              
  //             // and break out
  //             i = tempC.size();
  //           }
  //       }
  //     if(!erasing) ++itr; 
  //   }

  // itr  = currentObjectRegionList.begin(); 
  // stop = currentObjectRegionList.end();
  // while(itr != stop)
  //   {
  //     bool erasing = false;
  //     for(uint i = 0; i < tempS.size(); i++)
  //       {  
  //         if(tempS[i] == (*itr))
  //           {
  //             // take out the point from the object list
  //             itr  = currentObjectRegionList.erase(itr);
  //             stop = currentObjectRegionList.end();
  //             erasing = true;
             
  //             // and break out
  //             i = tempS.size();
  //           }
  //       }
  //     if(!erasing) ++itr; 
  //   }

  // insert to the proper list
  for(uint i = 0; i < tempC.size(); i++)
    currentObjectRegionList.push_back(tempC[i]);

  for(uint i = 0; i < tempS.size(); i++)
    currentBackgroundRegionList.push_back(tempS[i]);

  // set the image 
  for(uint i = 0; i < tempS.size(); i++)
    for(uint j = 0; j < itsInitialRegions[tempS[i]].size(); j++)
      regionAssignmentMap.setVal
        (itsInitialRegions[tempS[i]][j], SURROUND_REGION);
  for(uint i = 0; i < tempC.size(); i++)
    for(uint j = 0; j < itsInitialRegions[tempC[i]].size(); j++)
      regionAssignmentMap.setVal
        (itsInitialRegions[tempC[i]][j], CENTER_REGION);

  // =============================================

  // print all the percentages for each region  
  // maybe compare % for center and surround 
  //   have a minimum count/percentage so that won't add outside the area

  // seems that bad boundary is the culprit --> need tensor voting
  
  // =============================================
  // NOTE: CAN JUST DO THIS AT THE END OF THE SEGMENTATION

  // for each center grid point
  std::vector<int> tList;
  for(uint i = 0; i < currentObjectLocations.size(); i++)
    {
      Point2D<int> oPtGrid = currentObjectLocations[i];
      Point2D<int> oPt = oPtGrid*BOUNDARY_STEP_SIZE;
      // LINFO("optGrid: %d %d --> %3d %3d", 
      //       oPtGrid.i, oPtGrid.j, oPt.i, oPt.j);

      // get all region within this grid point
      for(int gi = -hstep; gi < hstep; gi++)
        for(int gj = -hstep; gj < hstep; gj++)
          {
            Point2D<int> pt(oPt.i+gi, oPt.j+gj);
            if(!itsInitialRegionImage.coordsOk(pt)) continue;
            //LINFO("  pt: %d %d", pt.i, pt.j);
            int rIndex = itsInitialRegionImage.getVal(pt);
            float rsize = float(itsInitialRegions[rIndex].size());
            //LINFO("  %d %f", rIndex, rsize);

            // check if the region has been looked at
            bool lookedAt = false;
            for(uint l = 0; l < tList.size(); l++)
              if(tList[l] == rIndex) 
                { lookedAt = true; l = tList.size(); } 

            // if not yet looked at and not an object region
            if(!lookedAt &&
               regionAssignmentMap.getVal(pt) == UNVISITED_REGION)
              {
                // get the percentage of overlap
                // wrt all the center points
                float nOverlap2 = 0.0;

                // for each pt in the region list
                std::vector<Point2D<int> > tGList;
                for(uint j = 0; j < itsInitialRegions[rIndex].size(); j++)
                  {
                    // get the grid loc location
                    Point2D<int> pt2 = itsInitialRegions[rIndex][j];
                    Point2D<int> ptG2
                      ((pt2.i+hstep)/BOUNDARY_STEP_SIZE,
                       (pt2.j+hstep)/BOUNDARY_STEP_SIZE );
                    //LINFO("  ==> %3d %3d %3d %3d", 
                    //      pt2.i, pt2.j, ptG2.i, ptG2.j);

                    // have to skip out of bounds pixels
                    if(ptG2.i <  0  || ptG2.j <  0  || 
                       ptG2.i >= wG || ptG2.j >= hG   )
                      continue;
                    
                    if(assignmentMap.getVal(ptG2) != CENTER_REGION)
                      continue;

                    bool gLookedAt = false;
                    for(uint g = 0; g < tGList.size(); g++)
                      if(tGList[g] == ptG2) 
                        { gLookedAt = true; g = tGList.size(); } 

                    if(!gLookedAt)
                      {
                        // calculate the overlap
                        //      Note the grid loc has been looked at
                        for(int g2i = -hstep; g2i < hstep; g2i++)
                          for(int g2j = -hstep; g2j < hstep; g2j++)
                            {
                              Point2D<int> pt3(ptG2.i*BOUNDARY_STEP_SIZE+g2i, 
                                               ptG2.j*BOUNDARY_STEP_SIZE+g2j );
                              // LINFO("%d %d %d %d",
                              //       itsInitialRegionImage.getWidth(),
                              //       itsInitialRegionImage.getHeight(),
                              //       pt3.i, pt3.j);

                              if(itsInitialRegionImage.coordsOk(pt3) &&
                                 itsInitialRegionImage.getVal(pt3) == rIndex)
                                {
                                  nOverlap2 += 1.0F;
                                }                              
                            }
                        tGList.push_back(ptG2);
                      }                    
                  }

                // if sum > threshold add to the object
                if(nOverlap2/rsize > REGION_INCLUDE_THRESHOLD)
                  {
                    // take out if in the background
                    std::vector<int>::iterator 
                      itr2  = currentBackgroundRegionList.begin(), 
                      stop2 = currentBackgroundRegionList.end();
                    bool breakout = false;
                    while(!breakout && itr2 != stop2)
                      {
                        if(rIndex == (*itr2))
                          {
                            // take out the point from the unassigned list
                            itr2  = currentBackgroundRegionList.erase(itr2);
                            stop2 = currentBackgroundRegionList.end();
                            breakout = true;
                          }
                        itr2++;
                      }

                    // LINFO("RINDEX: %d, %f %f %f", 
                    //       rIndex, nOverlap2, rsize, nOverlap2/rsize);
                    currentObjectRegionList.push_back(rIndex);
                    
                    for(uint jj = 0; jj < itsInitialRegions[rIndex].size(); 
                        jj++)
                      regionAssignmentMap.setVal
                        (itsInitialRegions[rIndex][jj], CENTER_REGION);
                  }

                tList.push_back(rIndex);
              }
          }
    }

  // for each background grid point
  std::vector<int> tSurrList;
  for(uint i = 0; i < currentBackgroundLocations.size(); i++)
    {
      Point2D<int> bPtGrid = currentBackgroundLocations[i];
      Point2D<int> bPt = bPtGrid*BOUNDARY_STEP_SIZE;
      // LINFO("bptGrid: %d %d --> %3d %3d", 
      //       bPtGrid.i, bPtGrid.j, bPt.i, bPt.j);

      // get all region within this grid point
      for(int gi = -hstep; gi < hstep; gi++)
        for(int gj = -hstep; gj < hstep; gj++)
          {
            Point2D<int> pt(bPt.i+gi, bPt.j+gj);
            //LINFO("  pt: %d %d", pt.i, pt.j);
            int rIndex = itsInitialRegionImage.getVal(pt);
            float rsize = float(itsInitialRegions[rIndex].size());
            //LINFO("  %d %f", rIndex, rsize);

            // check if the region has been looked at
            bool lookedAt = false;
            for(uint l = 0; l < tSurrList.size(); l++)
              if(tSurrList[l] == rIndex) 
                { lookedAt = true; l = tSurrList.size(); } 

            // if not yet looked at and not an object region
            if(!lookedAt &&
               regionAssignmentMap.getVal(pt) == UNVISITED_REGION)// ||
              // regionAssignmentMap.getVal(pt) != SURROUND_REGION  ))
              {
                // get the percentage of overlap
                // wrt all the center points
                float nOverlap2 = 0.0;

                // for each pt in the region list
                std::vector<Point2D<int> > tGList;
                for(uint j = 0; j < itsInitialRegions[rIndex].size(); j++)
                  {
                    // get the grid loc location
                    Point2D<int> pt2 = itsInitialRegions[rIndex][j];
                    Point2D<int> ptG2
                      ((pt2.i+hstep)/BOUNDARY_STEP_SIZE,
                       (pt2.j+hstep)/BOUNDARY_STEP_SIZE );
                    //LINFO("  ==> %3d %3d %3d %3d", 
                    //      pt2.i, pt2.j, ptG2.i, ptG2.j);

                    // have to skip out of bounds pixels
                    if(ptG2.i <  0  || ptG2.j <  0  || 
                       ptG2.i >= wG || ptG2.j >= hG   )
                      continue;
                    
                    if(assignmentMap.getVal(ptG2) != SURROUND_REGION)
                      continue;

                    bool gLookedAt = false;
                    for(uint g = 0; g < tGList.size(); g++)
                      if(tGList[g] == ptG2) 
                        { gLookedAt = true; g = tGList.size(); } 

                    if(!gLookedAt)
                      {
                        // calculate the overlap
                        //      Note the grid loc has been looked at
                        for(int g2i = -hstep; g2i < hstep; g2i++)
                          for(int g2j = -hstep; g2j < hstep; g2j++)
                            {
                              Point2D<int> pt3(ptG2.i*BOUNDARY_STEP_SIZE+g2i, 
                                               ptG2.j*BOUNDARY_STEP_SIZE+g2j );
                              // LINFO("%d %d %d %d",
                              //       itsInitialRegionImage.getWidth(),
                              //       itsInitialRegionImage.getHeight(),
                              //       pt3.i, pt3.j);

                              if(itsInitialRegionImage.coordsOk(pt3) &&
                                 itsInitialRegionImage.getVal(pt3) == rIndex)
                                {
                                  nOverlap2 += 1.0F;
                                }                              
                            }
                        tGList.push_back(ptG2);
                      }                    
                  }

                // if sum > threshold add to the object
                if(nOverlap2/rsize > REGION_INCLUDE_THRESHOLD)
                  {
                    // LINFO("RINDEX: %d, %f %f %f", 
                    //       rIndex, nOverlap2, rsize, nOverlap2/rsize);

                    //check if the rIndex is already in object
                    bool inObjectRegionList = false;
                    for(uint jj = 0; jj < currentObjectRegionList.size(); jj++)
                      if(currentObjectRegionList[jj] == rIndex)
                        {  
                          inObjectRegionList = true;
                          jj = currentObjectRegionList.size();
                        }
               
                    if(!inObjectRegionList)
                      {
                        currentBackgroundRegionList.push_back(rIndex);
                        
                        for(uint jj = 0; jj < itsInitialRegions[rIndex].size(); 
                            jj++)
                          regionAssignmentMap.setVal
                            (itsInitialRegions[rIndex][jj], SURROUND_REGION);
                      }
                  }

                tSurrList.push_back(rIndex);
              }
          }
    }

  // // if the other side of the edgel is assigned to other objects 
  // // assign that region to the other object as well
  // for(uint e = segment.segStartIndex; e <= segment.segEndIndex; e++) 
  //   {
  //     rutz::shared_ptr<Contour> contour = 
  //       itsContourBoundaries[segment.contourIndex];  
  //     rutz::shared_ptr<Edgel> edgel = contour->edgels[e];           
  //     Point2D<int> pt = edgel->pt;
  //     Point2D<int> ptGrid
  //       ((pt.i+hstep)/BOUNDARY_STEP_SIZE,
  //        (pt.j+hstep)/BOUNDARY_STEP_SIZE );
      
  //     uint dir = edgel->angleIndex; 

  //     //LINFO("[<<%3d>>] %d %d --> %d %d (%d)", 
  //     //      e,  pt.i, pt.j, ptGrid.i, ptGrid.j, dir);

  //     // if the edgel's neighbors are assigned to other objects
  //     bool allOtherObject = true; int objNum = -1;
  //     for(uint i = 0; i < neighbor[dir].size(); i++)
  //       {
  //         Point2D<int> ptGrid2 = ptGrid + neighbor[dir][i];
  //         Point2D<int> pt2(ptGrid2.i*BOUNDARY_STEP_SIZE,
  //                          ptGrid2.j*BOUNDARY_STEP_SIZE );

  //         //LINFO("    N[%3d]: %d %d --> %d %d", 
  //         //      i, ptGrid2.i, ptGrid2.j, pt2.i, pt2.j);

  //         if(!assignmentMap.coordsOk(ptGrid2)) continue;

  //         uint label2 = assignmentMap.getVal(ptGrid2);
  //         LINFO("LABEL2: %d", label2);

  //         // check for far side
  //         int side = bimage.getVal(pt2);
  //         if(side == FAR_SIDE)
  //           {
  //             if(label2 != OTHER_OBJECT_REGION)
  //               {
  //                 //LINFO("not OO_REGION");
  //                 allOtherObject = false;
  //               }
  //             else if(objNum == -1)
  //               objNum = regionAssignmentMap.getVal(ptGrid2);
  //           }
  //       }

  //     // if all of the far side neighbor belong to another object
  //     // assign the grid point to that object
  //     if(allOtherObject)
  //       {
  //         assignmentMap.setVal(ptGrid, OTHER_OBJECT_REGION);
  //         coreAssignmentMap.setVal(ptGrid, OTHER_OBJECT_REGION);

  //         if(objNum != -1)
  //           {
  //             regionAssignmentMap.setVal(ptGrid, objNum);
  //             itsCurrentRegionAssignmentMap.setVal(ptGrid, objNum);
  //           }
  //       }
  //   }
}

// ######################################################################
Point2D<int> SalientRegionSegmenter::evaluateCoreAssignment
(std::vector<Point2D<int> > currentObjectLocations, 
 Image<int> assignmentMap, Image<int> &coreAssignmentMap, uint &index)
{ 
  // set the next start to be invalid temporarily
  Point2D<int> currPointGrid(-1,-1);

  // go through each center point
  for(uint c = 0; c < currentObjectLocations.size(); c++)
    {
      Point2D<int> coLoc = currentObjectLocations[c];
      
      // skip if it's already part of core
      if(coreAssignmentMap.getVal(coLoc) == CENTER_CORE_REGION)
        continue;

      // check if the point is covered
      bool isCovered = true;
      for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
          if(!(i == 0 && j == 0))
            {
              Point2D<int> pt(coLoc.i+i, coLoc.j+j);
              if(assignmentMap.coordsOk(pt) &&
                 assignmentMap.getVal(pt) == UNASSIGNED_REGION) 
                {
                  isCovered = false;
                }
            }
              
      // assign appropriately
      if(!isCovered)
        {
          // the first one AFTER the index
          // is the next start point
          // because it is the one 
          // that goes in under the lowest threshold 
          if(!currPointGrid.isValid() && c >= index) 
            {            
              currPointGrid = coLoc;
              index = c;
            }

          coreAssignmentMap.setVal(coLoc, CENTER_REGION);
        }
      else
        coreAssignmentMap.setVal(coLoc, CENTER_CORE_REGION);
    }

  return currPointGrid;
}

// ######################################################################
void SalientRegionSegmenter::growSurround
(Image<int>                 &assignmentMap,
 std::vector<Point2D<int> >  currentObjectLocations,
 std::vector<Point2D<int> > &currentBackgroundLocations,
 std::vector<Point2D<int> > &currentUnassignedLocations,
 std::vector<Point2D<int> >  currentIncludedEdgelLocations,
 Image<int>                  coreAssignmentMap)
{
  // FIXXXXXXXXXX: need to flip the way we look for 
  //               the closest surround points

  // for each center point
  for(uint c = 0; c < currentObjectLocations.size(); c++)
    {
      Point2D<int> coLocGrid = currentObjectLocations[c];
      Point2D<int> coLoc     = coLocGrid * BOUNDARY_STEP_SIZE;

      //LINFO("colocG: %3d %3d", coLocGrid.i, coLocGrid.j);

      // check non-core centers only
      if(coreAssignmentMap.getVal(coLocGrid) == CENTER_CORE_REGION)  
        continue;
      
      // get histogram of center
      rutz::shared_ptr<Histogram> histC = getHistogramDistribution(coLoc);

      // for each unassigned point around core
      for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
          if(!(i == 0 && j == 0))
            {
              Point2D<int> uPtGrid(coLocGrid.i+i, coLocGrid.j+j);
              Point2D<int> uPt = uPtGrid * BOUNDARY_STEP_SIZE;

              //LINFO("  uPtGrid: %3d %3d", uPtGrid.i, uPtGrid.j);

              if(assignmentMap.coordsOk(uPtGrid) &&
                 assignmentMap.getVal(uPtGrid) == UNASSIGNED_REGION) 
                {
                  //LINFO("    Neigh: %3d %3d", uPtGrid.i, uPtGrid.j);

                  // get histogram of the unassigned neighbor
                  rutz::shared_ptr<Histogram> histCN = getHistogramDistribution(uPt);

                  // for each background point
                  for(uint ii = 0; ii < currentBackgroundLocations.size(); ii++)
                    {
                      Point2D<int> bgPtGrid = currentBackgroundLocations[ii];
                      Point2D<int> bgPt     = bgPtGrid * BOUNDARY_STEP_SIZE;

                      // check if point is part of included edgels
                      bool inCIEL = false;
                      for(uint jj = 0; 
                          jj < currentIncludedEdgelLocations.size(); jj++)
                        if(currentIncludedEdgelLocations[jj] == bgPtGrid)
                          { 
                            inCIEL = true; 
                            jj = currentIncludedEdgelLocations.size(); 
                          }

                      // check if the line connecting the two points 
                      // go through region labeled CENTER_REGION
                      bool throughObject =
                        throughRegion(uPtGrid, bgPtGrid, 
                                      assignmentMap, CENTER_REGION); 

                      // if point is not edgel and is in line of sight
                      if(!inCIEL && !throughObject)
                        {
                          //LINFO("   line of sight: %3d %3d", 
                          //      bgPtGrid.i, bgPtGrid.j);

                          // get histogram of point
                          rutz::shared_ptr<Histogram> histS = 
                            getHistogramDistribution(bgPt); 
                          
                          // check if ratio > 2.0
                          float diffCtoCN = histC->getChiSqDiff(*histCN);
                          float diffCNtoS = histCN->getChiSqDiff(*histS);
                          if(diffCNtoS == 0.0F || diffCtoCN/diffCNtoS > 2.0F)
                            {
                              // delete it from the unassigned list
                              std::vector<Point2D<int> >::iterator 
                                itr  = currentUnassignedLocations.begin(), 
                                stop = currentUnassignedLocations.end();
                              bool erasing = false;
                              while(itr != stop)
                                {
                                  Point2D<int> ptGrid = (*itr);
                                  
                                  if(ptGrid == uPtGrid)
                                    {
                                      // LINFO("PASS:C[%3d %3d] - N[%3d %3d] - " //DISP
                                      //       "S[%3d %3d] - %f / %f = %f", 
                                      //       coLocGrid.i, coLocGrid.j, 
                                      //       uPtGrid.i, uPtGrid.j,
                                      //       bgPtGrid.i, bgPtGrid.j,
                                      //       diffCtoCN, diffCNtoS, 
                                      //       diffCtoCN/diffCNtoS );

                                      // take out the point 
                                      // from the unassigned list
                                      itr  = 
                                        currentUnassignedLocations.erase(itr); 
                                      stop = currentUnassignedLocations.end();
                                      erasing = true;
                                  
                                      // put to the background list
                                      currentBackgroundLocations.push_back
                                        (uPtGrid);
                                      assignmentMap.setVal
                                        (uPtGrid, SURROUND_REGION);

                                      // and break out
                                      itr = stop;
                                      ii = currentBackgroundLocations.size(); 
                                    }                                  
                                  if(!erasing) itr++;
                                }
                            }
                        }
                    }
                }
            }
    }
}

// ######################################################################
void SalientRegionSegmenter::growUsingImaginedContours
(Image<int>                 &assignmentMap,
 std::vector<Point2D<int> >  currentObjectLocations,
 std::vector<Point2D<int> > &currentBackgroundLocations,
 std::vector<Point2D<int> > &currentUnassignedLocations,
 std::vector<Point2D<int> >  currentIncludedEdgelLocations,
 Image<int>                  coreAssignmentMap)
{
  LINFO("NOTHING HERE -- Maybe try later");
}

// ######################################################################
bool SalientRegionSegmenter::checkBackgroundRatio
(float ptDiff, rutz::shared_ptr<Histogram> ptHist, Point2D<int> ptGrid,
 Image<int> assignmentMap,
 std::vector<Point2D<int> > currentBackgroundLocations,
 std::vector<Point2D<int> > currentIncludedEdgelLocations)
{
  // for each point in the background list
  for(uint j = 0; j < currentBackgroundLocations.size(); j++)
    {
      bool isAnEdgel = false;
      for(uint i = 0; i < currentIncludedEdgelLocations.size(); i++)
        {
          if(currentIncludedEdgelLocations[i] == 
             currentBackgroundLocations[j])
            {
              //LINFO("%3d %3d is an edgel",
              //      currentBackgroundLocations[j].i,
              //      currentBackgroundLocations[j].j );
              isAnEdgel = true;
              i = currentIncludedEdgelLocations.size();
            }
        }
      if(isAnEdgel) continue;

      Point2D<int> bPtGrid = currentBackgroundLocations[j];
      Point2D<int> bPt(bPtGrid.i*BOUNDARY_STEP_SIZE,
                       bPtGrid.j*BOUNDARY_STEP_SIZE );
      rutz::shared_ptr<Histogram> bHist = getHistogramDistribution(bPt);
      
      // check if the line connecting the two points 
      // go through region labeled CENTER_REGION
      bool throughObject =
        throughRegion(ptGrid, bPtGrid, assignmentMap, CENTER_REGION);

      // calculate the histogram difference between the two points
      float diff = ptHist->getChiSqDiff(*bHist);

      // if it's below threshold
      if(!throughObject && ptDiff > diff)
        {
          // LINFO("%3d %3d - %3d %3d tobj: %d diff: %f : %f : ? %d " DISP
          //       "CLOSER TO BACKGROUND", 
          //       ptGrid.i, ptGrid.j, bPtGrid.i, bPtGrid.j, 
          //       throughObject, diff, ptDiff, ptDiff > diff);
          return false;
        }
    }
  return true;
}


// ######################################################################
void SalientRegionSegmenter::setBoundaryImage
(Image<int> assignmentMap, Image<float> mask)
{
  // get the current salient object: cores, list of edgels used 
  int wG   = itsCurrentRegionAssignmentMap.getWidth();
  int hG   = itsCurrentRegionAssignmentMap.getHeight();
  int cNum = itsCurrentNumSalientRegions - 1;

  int step  = BOUNDARY_STEP_SIZE;
  int hstep = BOUNDARY_STEP_SIZE/2;

  Image<bool> tempDrawn(wG, hG, ZEROS);

  // for each point in the core
  for(int gi = 0; gi < wG; gi++)
    for(int gj = 0; gj < hG; gj++)
      {
        // skip if not core
        if(itsCurrentRegionAssignmentMap.getVal(gi,gj) != cNum) 
          continue; 

        // check the neighbors
        std::vector<Point2D<int> > tempGpts;
        //bool onExposedEdge = false;
        bool innerCore = true;
        for(int dgi = -1; dgi <= 1; dgi++)
          for(int dgj = -1; dgj <= 1; dgj++) 
            {
              if(dgi == 0 && dgj == 0) continue;

              int gii = gi + dgi;
              int gjj = gj + dgj;

              // only within the image
              if(!assignmentMap.coordsOk(gii, gjj)) 
                continue;

              int label = assignmentMap.getVal(gii,gjj);

              // ignore cores that are outside 
              // usually means it's on edges ????
              //if(label == UNVISITED_REGION) 
              //  onExposedEdge = true; 

              if(label == SURROUND_REGION)
                {
                  tempGpts.push_back(Point2D<int>(gii,gjj));
                  innerCore = false;
                }
            }

        // if core is has boundary around it
        if(!innerCore)
          {
            bool allEdgels = true;
            for(uint i = 0; i < tempGpts.size(); i++) 
              {
                Point2D<int> ptGrid = tempGpts[i];
                if(itsEdgels.getVal(ptGrid).is_invalid()) 
                  allEdgels = false; 
              }
            
            // also check the outer core
            if(!allEdgels)
              tempGpts.push_back(Point2D<int>(gi, gj));
            
            for(uint i = 0; i < tempGpts.size(); i++) 
              {
                Point2D<int> ptGrid = tempGpts[i];
                if(tempDrawn.getVal(ptGrid)) continue;
                
                // if the point is not a core 
                // and the surround has an edgel
                if(itsEdgels.getVal(ptGrid).is_valid())
                  {
                    rutz::shared_ptr<Edgel> edgel = 
                      itsEdgels.getVal(ptGrid);
                    
                    Point2D<int> pt = edgel->pt; 

                    float dir = edgel->angleIndex;
                    dir = fmod((dir +(NUM_RIDGE_DIRECTIONS/2)),
                               NUM_RIDGE_DIRECTIONS);

                    float dx = cos(dir * M_PI/4.0) * hstep; 
                    float dy = sin(dir * M_PI/4.0) * hstep;
              
                    Point2D<int> p1 = pt + Point2D<int>( dx+.5,  dy+.5); 
                    Point2D<int> p2 = pt + Point2D<int>(-dx-.5, -dy-.5);

                    // pt = ptGrid * step;
                    // for(int di = -hstep; di < hstep; di++)
                    //   for(int dj = -hstep; dj < hstep; dj++)
                    //     itsBoundaryImage.setVal(pt.i+di,pt.j+dj, 0.15F);

                    // draw the straightline contour in the image 
                    // for visualization
                    drawLine(itsBoundaryImage, p1, p2, 1.0F);
                  }
                
                // if not we have to go to the borders of the center region
                else
                  {
                    Point2D<int> pt = ptGrid * step; 

                    for(int di = -hstep; di < hstep; di++)
                      for(int dj = -hstep; dj < hstep; dj++)
                        {
                          int i = pt.i + di;
                          int j = pt.j + dj;

                          if(mask.getVal(i,j) == 0) continue;
                          
                          bool isBorder = false;
                          for(int dii = -1; dii <= 1; dii++)
                            for(int djj = -1; djj <= 1; djj++)
                              {
                                if(dii == 0 && djj == 0) continue;

                                int ii = i + dii;
                                int jj = j + djj;
                                if(!mask.coordsOk(ii, jj)) continue;
                                
                                if(mask.getVal(ii, jj) == 0) isBorder = true;
                              }

                          if(isBorder)
                            itsBoundaryImage.setVal(i,j, 1.0F);
                        }
                  }
                tempDrawn.setVal(ptGrid, true);
              }
          }
      }
}

// ######################################################################
void SalientRegionSegmenter::displayDistributionSetup
(Segment segment, Point2D<int> pt, 
 std::vector<Point2D<int> > pts2, 
 std::vector<Point2D<int> > pts3)
{
  // display stuff
  Image<PixRGB<byte> > ima = itsImage;
  uint w = ima.getWidth();
  uint h = ima.getHeight();

  Image<float> fIma(luminance(ima));
  Image<float> tempFIma = fIma;
  inplaceNormalize(tempFIma, 0.0f,255.0f);  
  Image<byte>  bIma(tempFIma); 

  // get the image
  float mVal = 96;
  float bVal = 255 - mVal;
  Image<byte> dImaR, dImaG, dImaB;
  getComponents(ima, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);

  Image<PixRGB<byte> > dispIma(4*w,2*h,ZEROS);

  inplacePaste (dispIma, ima, Point2D<int>(0,0));

  Image<float> stateMapR(w,h, ZEROS);
  Image<float> stateMapG(w,h, ZEROS);
  Image<float> stateMapB(w,h, ZEROS);

  // draw grid
  int grid = 8;
  for(uint i = grid; i < w; i++)
    for(uint j = grid; j < h; j++)
      {
        if(i%grid == 0 && j%grid == 0)
          {
            drawCross(stateMapR, Point2D<int>(i,j), 0.5F, 1);
            drawCross(stateMapG, Point2D<int>(i,j), 0.5F, 1);
            drawCross(stateMapB, Point2D<int>(i,j), 0.5F, 1);
          }
      }

  // draw closer population
  for(uint i = 0; i < pts2.size(); i++)
    {
      if(stateMapR.coordsOk(pts2[i]))
        {
          stateMapR.setVal(pts2[i], 0.25F);
        }
    }

  // draw farther population
  for(uint i = 0; i < pts3.size(); i++)
    {
      if(stateMapR.coordsOk(pts3[i]))
        {
          stateMapG.setVal(pts3[i], 0.25F);
        }
    } 
  
  // draw the point 
  int rad = NEIGHBORHOOD_RADIUS;
  for(int i = -rad; i <= rad; i++) 
    for(int j = -rad; j <= rad; j++) 
      {  
        bool isWithinCircle = ((i*i+j*j) <= (rad*rad));

        Point2D<int> pt2 = pt + Point2D<int>(i,j);
        if(itsImage.coordsOk(pt2) && isWithinCircle)
          {
            stateMapR.setVal(pt2, 0.5F);
          }
      }

  drawDisk(stateMapR, pt, 2,  1.0F);
  drawDisk(stateMapG, pt, 2,  1.0F);
  drawDisk(stateMapB, pt, 2,  1.0F);

  // drawLine(stateMapR, pt, Point2D<int>(90,14),  0.5F);
  // drawLine(stateMapG, pt, Point2D<int>(90,14),  0.5F);
  // drawLine(stateMapB, pt, Point2D<int>(90,14),  0.5F);
  
  // draw the contour 
  rutz::shared_ptr<Contour> contour = 
    itsContourBoundaries[segment.contourIndex];  

  int hstep = BOUNDARY_STEP_SIZE/2;
  uint start = segment.segStartIndex;
  uint end   = segment.segEndIndex;
  for(uint i = 0; i < contour->edgels.size(); i++)
    {
      rutz::shared_ptr<Edgel> edgel = contour->edgels[i];
          
      uint aind = edgel->angleIndex; 
      float baind = 
        fmod((aind+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);
      
      float dx = cos(baind * M_PI/4.0) * hstep; 
      float dy = sin(baind * M_PI/4.0) * hstep;
      
      Point2D<int> pt = edgel->pt;
      Point2D<int> p1 = pt + Point2D<int>( dx+.5,  dy+.5); 
      Point2D<int> p2 = pt + Point2D<int>(-dx-.5, -dy-.5);
      
      if(i >= start && i <= end) 
        {
          drawLine(stateMapR, p1, p2, 1.0F);
          drawLine(stateMapG, p1, p2, 1.0F);
          drawLine(stateMapB, p1, p2, 1.0F);
        }
      else
        {
          drawLine(stateMapR, p1, p2, 0.5F);
          drawLine(stateMapG, p1, p2, 0.5F);
          drawLine(stateMapB, p1, p2, 0.5F);
        }
      //drawDisk(stateMap, pt, 2,  1.0F);
    }

  inplaceNormalize(stateMapR, 0.0f,bVal);
  inplaceNormalize(stateMapG, 0.0f,bVal);
  inplaceNormalize(stateMapB, 0.0f,bVal);
  Image<byte> dSMapR(stateMapR);
  Image<byte> dSMapG(stateMapG);
  Image<byte> dSMapB(stateMapB);
  Image<PixRGB<byte> > dSmap = makeRGB(dSMapR, dSMapG, dSMapB);
  inplacePaste
    (dispIma, Image<PixRGB<byte> >(dIma+dSmap), Point2D<int>(w,0));
  itsWin->drawImage(dispIma, 0,0);
  Raster::waitForKey();
}

// ######################################################################
void SalientRegionSegmenter::displaySegments(std::list<Segment> segments)
{
  // display stuff
  Image<PixRGB<byte> > ima = itsImage;
  uint w = ima.getWidth();
  uint h = ima.getHeight();

  Image<float> fIma(luminance(ima));
  Image<float> tempFIma = fIma;
  inplaceNormalize(tempFIma, 0.0f,255.0f);  
  Image<byte>  bIma(tempFIma); 

  // get the image
  float mVal = 32;
  float bVal = 255 - mVal;
  Image<byte> dImaR, dImaG, dImaB;
  getComponents(ima, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);

  Image<PixRGB<byte> > dispIma(4*w,2*h,ZEROS);

  inplacePaste (dispIma, ima, Point2D<int>(0,0));

  int hstep = BOUNDARY_STEP_SIZE/2;

  // draw each segment
  uint index = 0;
  std::list<Segment>::iterator itr  = segments.begin(), stop = segments.end();
  while(itr != stop)
    {
      Image<float> stateMapR(w,h, ZEROS);
      Image<float> stateMapG(w,h, ZEROS);
      Image<float> stateMapB(w,h, ZEROS);
      
      // draw grid
      int grid = 8;
      for(uint i = grid; i < w; i++)
        for(uint j = grid; j < h; j++)
          {
            if(i%grid == 0 && j%grid == 0)
              {
                drawCross(stateMapR, Point2D<int>(i,j), 0.5F, 1);
                drawCross(stateMapG, Point2D<int>(i,j), 0.5F, 1);
                drawCross(stateMapB, Point2D<int>(i,j), 0.5F, 1);
              }
          }
      
      Segment segment = (*itr);

      // draw the contour 
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[segment.contourIndex];  

      uint start = segment.segStartIndex;
      uint end   = segment.segEndIndex;
      for(uint i = 0; i < contour->edgels.size(); i++)
        {
          rutz::shared_ptr<Edgel> edgel = contour->edgels[i];
          
          uint aind = edgel->angleIndex; 
          float baind = 
            fmod((aind+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);
          
          float dx = cos(baind * M_PI/4.0) * hstep; 
          float dy = sin(baind * M_PI/4.0) * hstep;
          
          Point2D<int> pt = edgel->pt;
          Point2D<int> p1 = pt + Point2D<int>( dx+.5,  dy+.5); 
          Point2D<int> p2 = pt + Point2D<int>(-dx-.5, -dy-.5);
      
          if(i >= start && i <= end) 
            {
              drawLine(stateMapR, p1, p2, 1.0F);
              drawLine(stateMapG, p1, p2, 1.0F);
              drawLine(stateMapB, p1, p2, 1.0F);
            }
          else
            {
              drawLine(stateMapR, p1, p2, 0.5F);
              drawLine(stateMapG, p1, p2, 0.5F);
              drawLine(stateMapB, p1, p2, 0.5F);
            }
          //drawDisk(stateMap, pt, 2,  1.0F);
        }

      inplaceNormalize(stateMapR, 0.0f,bVal);
      inplaceNormalize(stateMapG, 0.0f,bVal);
      inplaceNormalize(stateMapB, 0.0f,bVal);
      Image<byte> dSMapR(stateMapR);
      Image<byte> dSMapG(stateMapG);
      Image<byte> dSMapB(stateMapB);
      Image<PixRGB<byte> > dSmap = makeRGB(dSMapR, dSMapG, dSMapB);
      inplacePaste
        (dispIma, Image<PixRGB<byte> >(dIma+dSmap), Point2D<int>(w,0));      
      if(itsWin.is_invalid())
        itsWin.reset(new XWinManaged(Dims(4*w,2*h), 0, 0, 
                                     "SalReg Segmenter"));
      else itsWin->setDims(Dims(4*w,2*h));
      itsWin->drawImage(dispIma, 0,0);

      // LINFO("[%3d]:(%3d) %6.3f*%6.3f + %6.3f*%6.3f + %6.3f*%6.3f + " // DISP
      //       "%6.3f*%6.3f + %6.3f*%6.3f = %6.3f", index, (*itr).orgIndex,
      //       LEN_WEIGHT, (*itr).lenVal,
      //       SIM_WEIGHT, (*itr).simVal,
      //       STR_WEIGHT, (*itr).strVal,
      //       RAT_WEIGHT, (*itr).ratVal,
      //       DIS_WEIGHT, (*itr).disVal, (*itr).pVal);

      itr++; index++;
      Raster::waitForKey();
    }
}

// ######################################################################
void SalientRegionSegmenter::displaySegment(Segment segment)
{
  // display stuff
  Image<PixRGB<byte> > ima = itsImage;
  uint w = ima.getWidth();
  uint h = ima.getHeight();

  Image<float> fIma(luminance(ima));
  Image<float> tempFIma = fIma;
  inplaceNormalize(tempFIma, 0.0f,255.0f);  
  Image<byte>  bIma(tempFIma); 

  // get the image
  float mVal = 32;
  float bVal = 255 - mVal;
  Image<byte> dImaR, dImaG, dImaB;
  getComponents(ima, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);

  Image<PixRGB<byte> > dispIma(4*w,2*h,ZEROS);

  inplacePaste (dispIma, ima, Point2D<int>(0,0));

  int hstep = BOUNDARY_STEP_SIZE/2;

  Image<float> stateMapR(w,h, ZEROS);
  Image<float> stateMapG(w,h, ZEROS);
  Image<float> stateMapB(w,h, ZEROS);
  
  // draw grid
  int grid = 8;
  for(uint i = grid; i < w; i++)
    for(uint j = grid; j < h; j++)
      {
        if(i%grid == 0 && j%grid == 0)
          {
            drawCross(stateMapR, Point2D<int>(i,j), 0.5F, 1);
            drawCross(stateMapG, Point2D<int>(i,j), 0.5F, 1);
            drawCross(stateMapB, Point2D<int>(i,j), 0.5F, 1);
          }
      }
  
  // draw segment

  // draw the contour 
  rutz::shared_ptr<Contour> contour = 
    itsContourBoundaries[segment.contourIndex];  

  uint start = segment.segStartIndex;
  uint end   = segment.segEndIndex;
  for(uint i = 0; i < contour->edgels.size(); i++)
    {
      rutz::shared_ptr<Edgel> edgel = contour->edgels[i];
      
      uint aind = edgel->angleIndex; 
      float baind = 
        fmod((aind+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);
      
      float dx = cos(baind * M_PI/4.0) * hstep; 
      float dy = sin(baind * M_PI/4.0) * hstep;
      
      Point2D<int> pt = edgel->pt;
      Point2D<int> p1 = pt + Point2D<int>( dx+.5,  dy+.5); 
      Point2D<int> p2 = pt + Point2D<int>(-dx-.5, -dy-.5);
      
      if(i >= start && i <= end) 
        {
          drawLine(stateMapR, p1, p2, 1.0F);
          drawLine(stateMapG, p1, p2, 1.0F);
          drawLine(stateMapB, p1, p2, 1.0F);
        }
      else
        {
          drawLine(stateMapR, p1, p2, 0.5F);
          drawLine(stateMapG, p1, p2, 0.5F);
          drawLine(stateMapB, p1, p2, 0.5F);
        }
      //drawDisk(stateMap, pt, 2,  1.0F);
    }

  inplaceNormalize(stateMapR, 0.0f,bVal);
  inplaceNormalize(stateMapG, 0.0f,bVal);
  inplaceNormalize(stateMapB, 0.0f,bVal);
  Image<byte> dSMapR(stateMapR);
  Image<byte> dSMapG(stateMapG);
  Image<byte> dSMapB(stateMapB);
  Image<PixRGB<byte> > dSmap = makeRGB(dSMapR, dSMapG, dSMapB);
  inplacePaste
    (dispIma, Image<PixRGB<byte> >(dIma+dSmap), Point2D<int>(w,0));      

  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(4*w,2*h), 0, 0, 
                                 "SalReg Segmenter"));
  else itsWin->setDims(Dims(4*w,2*h));
  itsWin->drawImage(dispIma, 0,0);
  
  // LINFO("   :(%3d) %6.3f*%6.3f + %6.3f*%6.3f + %6.3f*%6.3f + " // DISP
  //       "%6.3f*%6.3f + %6.3f*%6.3f = %6.3f", segment.orgIndex,
  //       LEN_WEIGHT, segment.lenVal,
  //       SIM_WEIGHT, segment.simVal,
  //       STR_WEIGHT, segment.strVal,
  //       RAT_WEIGHT, segment.ratVal,
  //       DIS_WEIGHT, segment.disVal, segment.pVal);
  Raster::waitForKey();
}

// ######################################################################
void SalientRegionSegmenter::displayGrowMap
  (Image<int> assignmentMap, Point2D<int> currPointGrid,
   std::vector<Point2D<int> > currentObjectLocations,
   std::vector<Point2D<int> > currentBackgroundLocations,
   std::vector<Point2D<int> > currentUnassignedLocations,
   std::vector<Point2D<int> > currentIncludedEdgelLocations,
   Image<bool> isComparedWith, Segment segment,
   Image<int>  regionAssignmentMap,
   Image<int>  coreAssignmentMap) 
{
  // display stuff
  Image<PixRGB<byte> > ima = itsImage;
  int w = ima.getWidth();
  int h = ima.getHeight();

  Image<float> fIma(luminance(ima));
  Image<float> tempFIma = fIma;
  inplaceNormalize(tempFIma, 0.0f,255.0f);  
  Image<byte>  bIma(tempFIma); 

  // get the image
  float mVal = 96;
  float bVal = 255 - mVal;
  Image<byte> dImaR, dImaG, dImaB;
  getComponents(ima, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);

  Image<PixRGB<byte> > dispIma(4*w,2*h,ZEROS);
  inplacePaste (dispIma, ima, Point2D<int>(0,0));

  Image<float> stateMapR(w,h, ZEROS);
  Image<float> stateMapG(w,h, ZEROS);
  Image<float> stateMapB(w,h, ZEROS);

  std::vector<Point2D<int> >::iterator oitr = 
    currentObjectLocations.begin();
  std::vector<Point2D<int> >::iterator sitr = 
    currentBackgroundLocations.begin();
  std::vector<Point2D<int> >::iterator uitr = 
    currentUnassignedLocations.begin();
  std::vector<Point2D<int> >::iterator eitr =   
    currentIncludedEdgelLocations.begin();

  for(uint i = 0; i < currentObjectLocations.size(); i++)
    {
      Point2D<int> pt = (*oitr) * BOUNDARY_STEP_SIZE;
      Rectangle r = Rectangle::tlbrO(pt.j-4, pt.i-4, pt.j+4, pt.i+4);
      drawFilledRect(stateMapR, r, 0.5F);
      oitr++;
    }

  if(currPointGrid.isValid())
    {
      Point2D<int> cpg = currPointGrid * BOUNDARY_STEP_SIZE;
      Rectangle r = Rectangle::tlbrO(cpg.j-4, cpg.i-4, cpg.j+4, cpg.i+4);
      drawFilledRect(stateMapR, r, 0.5F);
      drawFilledRect(stateMapG, r, 0.0F);
      drawFilledRect(stateMapB, r, 0.5F);
    }

  for(uint i = 0; i < currentUnassignedLocations.size(); i++)
    {
      Point2D<int> pt = (*uitr) * BOUNDARY_STEP_SIZE;
      Rectangle r = Rectangle::tlbrO(pt.j-4, pt.i-4, pt.j+4, pt.i+4);
      drawFilledRect(stateMapR, r, 0.5F);
      drawFilledRect(stateMapG, r, 0.5F);
      uitr++;
    }

  for(uint i = 0; i < currentBackgroundLocations.size(); i++)
    {
      Point2D<int> pt = (*sitr) * BOUNDARY_STEP_SIZE;
      Rectangle r = Rectangle::tlbrO(pt.j-4, pt.i-4, pt.j+4, pt.i+4);

      drawFilledRect(stateMapG, r, 0.5F);
      sitr++;
    }

 for(uint i = 0; i < currentIncludedEdgelLocations.size(); i++)
   {
     Point2D<int> pt = (*eitr) * BOUNDARY_STEP_SIZE;
     Rectangle r = Rectangle::tlbrO(pt.j-4, pt.i-4, pt.j+4, pt.i+4);

     drawFilledRect(stateMapR, r, 0.0F);
     drawFilledRect(stateMapG, r, 0.0F);
     drawFilledRect(stateMapB, r, 0.5F);
     eitr++;
   }

  for(int i = 0; i < assignmentMap.getWidth(); i++)
    for(int j = 0; j < assignmentMap.getHeight(); j++)
      {
        if(assignmentMap.getVal(i,j) == OTHER_OBJECT_REGION)
          {
            Point2D<int> pt = Point2D<int>(i,j)*BOUNDARY_STEP_SIZE;
            int tr = pt.j-4; if(tr < 0) tr = 0;
            int br = pt.j+4; if(br > h) br = h;
            int lr = pt.i-4; if(lr < 0) lr = 0;
            int rr = pt.i+4; if(rr > w) rr = w;
            Rectangle r = Rectangle::tlbrO(tr, lr, br, rr);

            drawFilledRect(stateMapR, r, 0.25F);
            drawFilledRect(stateMapG, r, 0.25F);
            drawFilledRect(stateMapB, r, 0.25F);
          }
      }


  // draw grid
  int grad = 8;
  for(int i = grad; i < w; i++)
    for(int j = grad; j < h; j++)
      {
        if(i%grad == 0 && j%grad == 0)
          {
            drawCross(stateMapR, Point2D<int>(i,j), 0.5F, 1);
            drawCross(stateMapG, Point2D<int>(i,j), 0.5F, 1);
            drawCross(stateMapB, Point2D<int>(i,j), 0.5F, 1);
          }
      }
  
  // draw the contour 
  if(segment.orgIndex != -1)
    { 
      rutz::shared_ptr<Contour> contour = 
        itsContourBoundaries[segment.contourIndex];  

      int hstep = BOUNDARY_STEP_SIZE/2;
      uint start = segment.segStartIndex;
      uint end   = segment.segEndIndex;
      for(uint i = 0; i < contour->edgels.size(); i++)
        {
          rutz::shared_ptr<Edgel> edgel = contour->edgels[i];
          
          uint aind = edgel->angleIndex; 
          float baind = 
            fmod((aind+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);
      
          float dx = cos(baind * M_PI/4.0) * hstep; 
          float dy = sin(baind * M_PI/4.0) * hstep;
      
          Point2D<int> pt = edgel->pt;
          Point2D<int> p1 = pt + Point2D<int>( dx+.5,  dy+.5); 
          Point2D<int> p2 = pt + Point2D<int>(-dx-.5, -dy-.5);
          LDEBUG("%d %d",p1.i,p2.i);
      
          if(i >= start && i <= end) 
            {
              drawLine(stateMapR, p1, p2, 1.0F);
              drawLine(stateMapG, p1, p2, 1.0F);
              drawLine(stateMapB, p1, p2, 1.0F);
            }
          else
            {
              drawLine(stateMapR, p1, p2, 0.5F);
              drawLine(stateMapG, p1, p2, 0.5F);
              drawLine(stateMapB, p1, p2, 0.5F);
            }
          //drawDisk(stateMap, pt, 2,  1.0F);
        }
    }

  inplaceNormalize(stateMapR, 0.0f,bVal);
  inplaceNormalize(stateMapG, 0.0f,bVal);
  inplaceNormalize(stateMapB, 0.0f,bVal);
  Image<byte> dSMapR(stateMapR);
  Image<byte> dSMapG(stateMapG);
  Image<byte> dSMapB(stateMapB);
  Image<PixRGB<byte> > dSmap = makeRGB(dSMapR, dSMapG, dSMapB);
  inplacePaste
    (dispIma, Image<PixRGB<byte> >(dIma+dSmap), Point2D<int>(w,0));
  // Raster::WriteRGB(Image<PixRGB<byte> >(dIma+dSmap), 
  //                  sformat("%s_gridLabels_%03d.png", 
  //                          itsInputImageStem.c_str(),
  //                          itsIteration));

  inplaceNormalize(stateMapR, 0.0f,bVal * .25F);
  inplaceNormalize(stateMapG, 0.0f,bVal * .25F);
  inplaceNormalize(stateMapB, 0.0f,bVal * .25F);
  Image<byte> dSMapR2(stateMapR);
  Image<byte> dSMapG2(stateMapG);
  Image<byte> dSMapB2(stateMapB);

  Image<float> tRAM(regionAssignmentMap);
  inplaceNormalize(tRAM, 0.0f, bVal * .75F);
  Image<byte> tRAMb(tRAM);

  Image<PixRGB<byte> > dRmap = makeRGB
    ( Image<byte> (tRAMb+dSMapR2), 
      Image<byte> (tRAMb+dSMapG2), 
      Image<byte> (tRAMb+dSMapB2)  );

  //LINFO("its input stem: %s", itsInputImageStem.c_str());
  inplacePaste
    (dispIma, Image<PixRGB<byte> >(dIma+dRmap), Point2D<int>(2*w,0));  
  // Raster::WriteRGB(Image<PixRGB<byte> >(dIma+dRmap), 
  //                  sformat("%s_regAssMap_%03d.png", 
  //                          itsInputImageStem.c_str(),
  //                          itsIteration));

  // draw the core
  Image<float> stateMapR3(w,h, ZEROS);
  Image<float> stateMapG3(w,h, ZEROS);
  Image<float> stateMapB3(w,h, ZEROS);
  for(int i = 0; i < coreAssignmentMap.getWidth(); i++)
    for(int j = 0; j < coreAssignmentMap.getHeight(); j++)
      {        
        if(coreAssignmentMap.getVal(i,j) == CENTER_REGION)
          {
            Point2D<int> pt = Point2D<int>(i,j)*BOUNDARY_STEP_SIZE;
            Rectangle r = Rectangle::tlbrO(pt.j-4, pt.i-4, pt.j+4, pt.i+4);
            drawFilledRect(stateMapB3, r, 1.0F);
          }

        if(coreAssignmentMap.getVal(i,j) == CENTER_CORE_REGION)
          {
            Point2D<int> pt = Point2D<int>(i,j)*BOUNDARY_STEP_SIZE;
            Rectangle r = Rectangle::tlbrO(pt.j-4, pt.i-4, pt.j+4, pt.i+4);
            drawFilledRect(stateMapR3, r, 0.5F);
            drawFilledRect(stateMapG3, r, 0.5F);
            drawFilledRect(stateMapB3, r, 0.5F);
          }

        if(coreAssignmentMap.getVal(i,j) == OTHER_OBJECT_REGION)
          {
            Point2D<int> pt = Point2D<int>(i,j)*BOUNDARY_STEP_SIZE;
            int tr = pt.j-4; if(tr < 0) tr = 0;
            int br = pt.j+4; if(br > h) br = h;
            int lr = pt.i-4; if(lr < 0) lr = 0;
            int rr = pt.i+4; if(rr > w) rr = w;
            Rectangle r = Rectangle::tlbrO(tr, lr, br, rr);

            drawFilledRect(stateMapR3, r, 0.25F);
            drawFilledRect(stateMapG3, r, 0.25F);
            drawFilledRect(stateMapB3, r, 0.25F);
          }
      }

  inplaceNormalize(stateMapR3, 0.0f,bVal);
  inplaceNormalize(stateMapG3, 0.0f,bVal);
  inplaceNormalize(stateMapB3, 0.0f,bVal);
  Image<byte> dSMapR3(stateMapR3);
  Image<byte> dSMapG3(stateMapG3);
  Image<byte> dSMapB3(stateMapB3);
  Image<PixRGB<byte> > dRmap3 = makeRGB
    ( Image<byte> (dSMapR3), 
      Image<byte> (dSMapG3), 
      Image<byte> (dSMapB3)  );
  inplacePaste
    (dispIma, Image<PixRGB<byte> >(dIma+dRmap3), Point2D<int>(3*w, h));  
  // Raster::WriteRGB(Image<PixRGB<byte> >(dIma+dRmap3), 
  //                  sformat("%s_coreMap_%03d.png", 
  //                          itsInputImageStem.c_str(),
  //                          itsIteration));

  // draw the current object
  Image<float> dImaRf(dImaR);
  Image<float> dImaGf(dImaG);
  Image<float> dImaBf(dImaB);
  inplaceNormalize(dImaRf, 0.0f,255.0f);
  inplaceNormalize(dImaGf, 0.0f,255.0f);
  inplaceNormalize(dImaBf, 0.0f,255.0f);
  Image<float> weight(regionAssignmentMap-1);
  inplaceClamp(weight, 0.0F, float(CENTER_REGION-1));
  weight /= float(CENTER_REGION-1);

  Image<PixRGB<byte> > dRmap4 = makeRGB
    ( Image<byte> (dImaRf*weight), 
      Image<byte> (dImaGf*weight), 
      Image<byte> (dImaBf*weight) );

  inplacePaste(dispIma, dRmap4, Point2D<int>(3*w,0));  
  itsObjectMask.push_back(dRmap4);

  // Raster::WriteRGB(Image<PixRGB<byte> >(dRmap4), 
  //                  sformat("%s_object.png", itsInputImageStem.c_str()));

  Image<float> tempBI = itsBoundaryImage;
  inplaceNormalize(tempBI, 0.0f, 255.0f);  
  Image<byte> tempBIb(tempBI);
  Image<PixRGB<byte> > dRmap5 = makeRGB(tempBIb, tempBIb, tempBIb);
  inplacePaste(dispIma, dRmap5, Point2D<int>(2*w, h));  
  // Raster::WriteRGB(Image<PixRGB<byte> >(dRmap4), 
  //                  sformat("%s_boundary_%03d.png", 
  //                          itsInputImageStem.c_str(),
  //                          itsIteration));

  // NOTE: to display segmentation results
  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(4*w,2*h), 0, 0, 
                                 "SalReg Segmenter"));
  else itsWin->setDims(Dims(4*w,2*h));
  itsWin->drawImage(dispIma, 0,0);
  Raster::waitForKey();

  itsIteration++;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
