/*!@file Gist/benchmark-ShapeEstimator.C benchmark the various 
  shape estimator algorithm                                             */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/benchmark-ShapeEstimator.C $
// $Id: test-Gist-Sal-Nav.C 14762 2011-05-03 01:13:16Z siagian $
//
////////////////////////////////////////////////////////

#include "Component/ModelManager.H"
#include "Beobot/BeobotBrainMT.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "GUI/XWinManaged.H"

#include "Image/DrawOps.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/FilterOps.H"
#include "Image/MathOps.H"

#include "Neuro/ShapeEstimator.H" 

#include <iostream>
#include <list>
#include <fstream>
#include <algorithm>

#define ALG_L_COL_RED      PixRGB<byte>(255,  0,  0)
#define ALG_L_COL_ORANGE   PixRGB<byte>(255,128,  0)
#define ALG_L_COL_YELLOW   PixRGB<byte>(255,255,  0)
#define ALG_L_COL_GREEN    PixRGB<byte>(  0,255,  0)
#define ALG_L_COL_CYAN     PixRGB<byte>(  0,255,255)
#define ALG_L_COL_MAGENTA  PixRGB<byte>(255,  0,255)
#define ALG_L_COL_BLUE     PixRGB<byte>(  0,  0,255)

struct BenchmarkItem
{
  BenchmarkItem() { }; 

  std::string                fileName;
  Image<PixRGB<byte> >       image;
  std::vector<Rectangle>     groundTruth;
};

struct BenchmarkItemResult
{
  BenchmarkItemResult() 
  { 
    mindex = -1;
    sortValueInitialized = false;
  }; 

  std::vector<float> oVal;
  std::vector<float> aVal;
  std::vector<float> gVal;

  std::vector<float> precision;
  std::vector<float> recall;
  std::vector<float> fscore;

  int mindex;

  // this allows us to sort
  // sort with many different variables
  bool  sortValueInitialized;
  float sortValue;

  std::vector<Image<float> > algoOutputMask;
  std::vector<Rectangle>     algoOutputRect;

  void setSortValue(float val)
  {
    sortValueInitialized = true;
    sortValue = val;
  }

  // we flip the sign because want non-ascending order
  bool operator < (const BenchmarkItemResult& other)
  {
    if(sortValueInitialized && sortValueInitialized)
      return sortValue > other.sortValue;
    else
      return fscore[mindex] > other.fscore[mindex];
  }
};

// benchmark with the dataset and results
struct Benchmark
{
  Benchmark() { };

  // the name of the dataset 
  // and their images and ground truths
  std::string name;
  std::vector<BenchmarkItem> items;

  // the names of algorithms and their results
  std::vector<std::string> algorithmNames;
  std::vector<std::string> algorithmTagNames;
  std::vector<std::vector<BenchmarkItemResult> > results;
  std::vector<BenchmarkItemResult> summaryResults;
};

// fill the benchmark data
void fill_Liu_CVPR_2007_benchmark
(std::vector<Benchmark> &benchmarks, std::string gtFolder, 
 std::string dataFolder, uint numFiles);

void fill_Achanta_CVPR_2009_benchmark
(std::vector<Benchmark> &benchmarks, 
 std::string fname, std::string dataFolder, std::string gtFolder);

void computeMiddleRectangle
(std::vector<Benchmark> &benchmarks, 
 std::string outputFolder);

// compute the salient region
// using Siagian & Koch 2011 CBSE
void computeCBSE
(std::vector<Benchmark> &benchmarks, 
 nub::ref<BeobotBrainMT> bbmt,
 rutz::shared_ptr<SalientRegionSegmenter> srs,
 std::string outputFolder);

// compute the salient region
// using Walther, etal. 2004 SE
void computeSE
(std::vector<Benchmark> &benchmarks, 
 nub::ref<BeobotBrainMT> bbmt,
 std::string outputFolder);

// we precompute the region masks
void computeFromStoredMaps
(std::vector<Benchmark> &benchmarks, std::string outputFolder,
 std::string algoName, std::string algoTagName);

// // Achanta, etal 2009 Frequency Tuned Salient Region Segmenter
// void computeFTSRS
// (std::vector<Benchmark> &benchmarks, 
//  std::string outputFolder);

// // Achanta, etal 2007 Rectangular Center Surround Salient Region Segmenter
// void computeRCSSRS
// (std::vector<Benchmark> &benchmarks, 
//  std::string outputFolder);


// correct salient point if on boundary
Point2D<int> correctSalientPointLocation
(Point2D<int> pt, Point2D<int> wci, Image<float> salMap);

//
Rectangle getBoundingBox(Image<float> image);

// display the results in HTML format
void displayResults
(std::string fName, Benchmark benchmark,
 std::string outputFolder, std::string webOutputFolder);

//! debug window
rutz::shared_ptr<XWinManaged> win;

std::vector<PixRGB<byte> > algColorLabels;


// ######################################################################
// Main function
int main(const int argc, const char **argv)
{
  // instantiate a model manager:
  ModelManager manager("benchmark ShapeEstimator");

  // Instantiate our various ModelComponents:

  // get saliency
  nub::ref<BeobotBrainMT> bbmt(new BeobotBrainMT(manager));
  manager.addSubComponent(bbmt);

  // get shape estimator
  //nub::ref<ShapeEstimator> se(new ShapeEstimator(manager));
  //manager.addSubComponent(se);

  rutz::shared_ptr<SalientRegionSegmenter> 
    srs(new SalientRegionSegmenter());

  manager.exportOptions(MC_RECURSE);

  // Parse command-line:
  // benchmark name
  if (manager.parseCommandLine(argc, argv, "<image name>", 0, 1) == false) 
    return(1);

  // do post-command-line configs:
  std::string benchmarkName = manager.getExtraArg(0);
 
  std::vector<Benchmark> benchmarks;
  
  std::string benchmarkGtFolder1  ("../Objects/UserDataB");
  std::string benchmarkDataFolder1("../Objects/Image");
  std::string outputFolder        
    ("../public_html/Results/SalientRegionSegmentation");
  //uint numFiles1 = 1; //FIXXX: 1 0;

  // fill with the DatasetB of Liu, etal. IEEE T-PAMI 2011 
  //fill_Liu_CVPR_2007_benchmark
  //  (benchmarks, benchmarkGtFolder1, benchmarkDataFolder1, numFiles1);

  // fill with the DatasetB of Liu, etal. IEEE T-PAMI 2011 
  std::string achantaDBfname("../Objects/Acantha_CVPR_2009.txt");
  std::string benchmarkGtFolder2  ("../Objects/binarymasks");
  fill_Achanta_CVPR_2009_benchmark
    (benchmarks, achantaDBfname, 
     benchmarkDataFolder1, benchmarkGtFolder2);

  // let's do it!
  manager.start();

  // // PREV
  // Image<PixRGB<byte> > image = Raster::ReadRGB(benchmarkName.c_str());
  // uint width  = image.getWidth();
  // uint height = image.getHeight();
  // win.reset(new XWinManaged(Dims(width*3,height*2), 0, 0, "bm"));
  // bbmt->input(image); 
  // while(!bbmt->outputReady()) usleep(1000);
  // Image<float> currSalMap = bbmt->getSalMap();
  // Point2D<int> currwin; float maxval;
  // uint sw = currSalMap.getWidth();
  // uint sh = currSalMap.getHeight();
  // findMax(currSalMap, currwin, maxval);  
  // LINFO("%d %d %d %d", currwin.i, currwin.j, sw, width);
  // currwin.i = int(currwin.i * width  / float(sw));
  // currwin.j = int(currwin.j * height / float(sh));
  // LINFO("%d %d", currwin.i, currwin.j);
  // drawCross(image, currwin, PixRGB<byte>(255, 0 , 0));
  // win->drawImage(image,0,0);
  // Raster::waitForKey();

  // color labels for display
  algColorLabels.push_back(ALG_L_COL_RED);
  algColorLabels.push_back(ALG_L_COL_ORANGE);   
  algColorLabels.push_back(ALG_L_COL_YELLOW);   
  algColorLabels.push_back(ALG_L_COL_GREEN);    
  algColorLabels.push_back(ALG_L_COL_CYAN);
  algColorLabels.push_back(ALG_L_COL_MAGENTA);  
  algColorLabels.push_back(ALG_L_COL_BLUE);   

  // Compute the salient regions
  // using various techniques:
  
  // Siagian&Koch 2011 Contour Boundary Shape Estimator
  computeCBSE(benchmarks, bbmt, srs, outputFolder);

  // just compute the middle quarter rectangle
  computeMiddleRectangle(benchmarks, outputFolder);

  // Walther, etal 2004 Shape Estimator
  computeSE(benchmarks, bbmt, outputFolder);

  // Achanta, etal 2007 Rectangular Center Surround Salient Region Segmenter
  computeFromStoredMaps
    (benchmarks, outputFolder,               
     std::string("Rectangular Center Surround Salient Region Segmenter"),
     std::string("RCSSRS"));

  // Paul Rosin, 2008 Edge Density Salient Region Segmenter 
  computeFromStoredMaps
    (benchmarks, outputFolder,               
     std::string("Edge Density Salient Region Segmenter"),
     std::string("EDSRS"));

  // Achanta, etal 2009 Frequency Tuned Salient Region Segmenter
  computeFromStoredMaps
    (benchmarks, outputFolder,               
     std::string("Frequency Tuned Salient Region Segmenter"),
     std::string("FTSRS"));
  
  // evaluate results from Liu 
  // and put it to a web accessible page
  displayResults
    (std::string("../public_html/result.html"), benchmarks[0],
     outputFolder, std::string("Results/SalientRegionSegmentation"));

  // custom display rankings
}

// ######################################################################
void fill_Liu_CVPR_2007_benchmark
(std::vector<Benchmark> &benchmarks, std::string gtFolder, 
 std::string dataFolder, uint numFiles)
{
  Benchmark b; b.name = std::string("Liu_CVPR_2007");

  // go through each file
  for(uint i = 0; i < numFiles; i++)
    {
      // open file
      std::string fname = sformat("%s/%d_data.txt", gtFolder.c_str(), i);
      std::ifstream inf(fname.c_str());
      if (inf.is_open() == false) 
        LFATAL("Cannot open '%s'", fname.c_str());
      else
        {
          std::string line;
          std::getline(inf, line);
          std::getline(inf, line);

          // go through each example
          uint count = 0;
          while(count < 10 && std::getline(inf, line))
            {
              // file name, size, ground truth
              std::string filename = line.substr(0, line.length()-1);
              std::string::size_type fbspos = line.find_first_of('\\');
              filename[fbspos] = '/';
              Image<PixRGB<byte> > image = 
                Raster::ReadRGB(sformat("%s/%s", dataFolder.c_str(), 
                                        filename.c_str()).c_str());

              std::getline(inf, line);
              std::string size = line.substr(0, line.length()-1); 
              LINFO("[%3d][%3d]:'%s' -- %s", 
                    i, count, filename.c_str(), size.c_str());

              std::getline(inf, line);
              std::string gt = line.substr(0, line.length()-1); 
              std::vector<Rectangle>     groundTruth;      
              std::string::size_type fspos = line.find_first_of(';');
              while(fspos != std::string::npos)
                {
                  std::string r = line.substr(0,fspos);
                  line = line.substr(fspos+2);
                  std::string::size_type fsspos = r.find_first_of(' ');
                  int left = atoi(r.substr(0, fsspos).c_str());
                  r = r.substr(fsspos+1);
                  fsspos = r.find_first_of(' ');
                  int top = atoi(r.substr(0, fsspos).c_str());
                  r = r.substr(fsspos+1);
                  fsspos = r.find_first_of(' ');
                  int right = atoi(r.substr(0, fsspos).c_str());
                  r = r.substr(fsspos+1);
                  int bottom = atoi(r.c_str());

                  LDEBUG("%3d %3d %3d %3d", top, left, bottom, right);
                  Rectangle  rect = 
                    Rectangle::tlbrO(top, left, bottom, right);

                  groundTruth.push_back(rect);
                  fspos = line.find_first_of(';');
                }

              std::getline(inf, line); // blank
                            
              BenchmarkItem bi;
              bi.fileName    = filename;
              bi.image       = image;              
              bi.groundTruth = groundTruth;
              b.items.push_back(bi);
              count++;
            }
        } 
    }

  benchmarks.push_back(b);
}

// ######################################################################
void fill_Achanta_CVPR_2009_benchmark
(std::vector<Benchmark> &benchmarks, 
 std::string fname, std::string dataFolder, std::string gtFolder)
{
  Benchmark b; b.name = std::string("Achanta_CVPR_2009");

  // open file
  std::ifstream inf(fname.c_str());
  if (inf.is_open() == false) 
    { LINFO("Cannot open '%s'", fname.c_str()); return; }

  // go through each example
  std::string line; uint count = 0;
  while(std::getline(inf, line))
    {
      // file name, size, ground truth
      std::string::size_type ldpos = line.find_last_of('.');
      std::string filename = 
        line.substr(0, ldpos)+std::string(".jpg");
      Image<PixRGB<byte> > image = 
        Raster::ReadRGB(sformat("%s/%s", dataFolder.c_str(), 
                                filename.c_str()).c_str());
      LINFO("[%3d]:'%s'", count, filename.c_str());

      ldpos = filename.find_last_of('.');
      std::string::size_type fspos = filename.find_first_of('/');
      std::string gtfilename = 
        filename.substr(fspos+1, ldpos - fspos -1) + std::string(".png");      
      //gtfilename = gtfilename.substr(0, ldpos)+std::string(".png");
      LINFO("gt: %s", gtfilename.c_str());

      Image<float> gt = 
        Raster::ReadGray((gtFolder + '/' + gtfilename).c_str());

      std::vector<Rectangle>     groundTruth;      
      Rectangle r = getBoundingBox(gt);
      groundTruth.push_back(r);

      BenchmarkItem bi;
      bi.fileName    = filename;
      bi.image       = image;              
      bi.groundTruth = groundTruth;
      b.items.push_back(bi);
      
      // // display
      // uint width  = image.getWidth(); 
      // uint height = image.getHeight();
      // win.reset(new XWinManaged(Dims(width,height), 0, 0, "bmCBSE"));
      // drawRect(image, bi.groundTruth[0], PixRGB<byte>(255,0,0));
      // win->drawImage(image,0,0);
      // Raster::waitForKey();
      
      count++;      
    }

  benchmarks.push_back(b);
}

// ######################################################################
void computeCBSE
(std::vector<Benchmark> &benchmarks, 
 nub::ref<BeobotBrainMT> bbmt,
 rutz::shared_ptr<SalientRegionSegmenter> srs,
 std::string outputFolder)
{
  // just in case we want to average results from all benchmarks
  float sumO = 0.0F; float sumA = 0.0F; float sumG = 0.0F; 
  float sumP = 0.0F; float sumR = 0.0F; float sumF = 0.0F;
  uint  count = 0;
  for(uint i = 0; i < benchmarks.size(); i++)
    {
      std::string bname = benchmarks[i].name;

      // insert information about CBSE algorithm
      BenchmarkItemResult avgRes;
      benchmarks[i].algorithmNames.push_back
        (std::string("Contour Boundary Shape Estimator"));
      std::string tag("CBSE");
      benchmarks[i].algorithmTagNames.push_back(tag);

      std::vector<BenchmarkItemResult> birs;
      BenchmarkItemResult sbir;

      float isumO = 0.0F; float isumA = 0.0F; float isumG = 0.0F; 
      float isumP = 0.0F; float isumR = 0.0F; float isumF = 0.0F;
      uint  icount = 0;
      for(uint j = 0; j < benchmarks[i].items.size(); j++)
        {
          BenchmarkItemResult bir; 

          // get images & ground truth  
          BenchmarkItem bi = benchmarks[i].items[j];
          Image<PixRGB<byte> > image = bi.image;
          uint width  = image.getWidth();
          uint height = image.getHeight();

          // get saliency map, compute if needed
          std::string tfname = bi.fileName;
          std::replace(tfname.begin(), tfname.end(), '/', '_');
          std::string::size_type ldpos = tfname.find_last_of('.');
          tfname = tfname.substr(0, ldpos);
          std::string fstem = 
            outputFolder + '/' + bname + '/' + tfname;
          std::string afstem = fstem + '_' + tag;

          std::string ressmfname = fstem + std::string("_salmap.ppm");
          LDEBUG("resfname: %s", ressmfname.c_str());

          // if saliency map was not precomputed, compute now
          std::ifstream smf(ressmfname.c_str());
          Image<float> salMap;
          if (smf.is_open() == false) 
            {
              bbmt->input(image); 
              while(!bbmt->outputReady()) usleep(1000);
              salMap = bbmt->getSalMap();

              // save the saliency map
              Image<float> tsmap = salMap;
              inplaceNormalize(tsmap, 0.0F, 255.0F);
              Raster::WriteGray(Image<byte>(tsmap), ressmfname.c_str());
            }
          else salMap = Raster::ReadGray(ressmfname.c_str());

          // also check for the salient region mask
          Rectangle srRect; Image<float> srImage; uint nSalReg = 0;
          std::string resmaskfname = afstem + sformat("_sreg_0.ppm");
          std::ifstream srf; srf.open(resmaskfname.c_str());
          std::vector<Image<float> > maskMaps; 
          LDEBUG("sreg[%3d][%5d]: %s",i,j, resmaskfname.c_str());

          if (srf.is_open())
            {
              // while there is another mask stored
              while(srf.is_open())
                {
                  maskMaps.push_back(Raster::ReadGray(resmaskfname.c_str()));
                  nSalReg++;
                  resmaskfname = afstem + sformat("_sreg_%d.ppm", nSalReg);
                  srf.close();
                  srf.open(resmaskfname.c_str());
                }
            }
          else 
            {
              bbmt->input(image); 
              while(!bbmt->outputReady()) usleep(1000);
              salMap = bbmt->getSalMap();

              // get and correct location of salient point
              uint sw = salMap.getWidth();
              uint sh = salMap.getHeight();
            
              Image<float> currSalMap = salMap;
              Point2D<int> currwin; float maxval;
              findMax(currSalMap, currwin, maxval);  
              // get the winning channel information
              // RG: 0, BY: 1, I: 2, O0: 3, O45: 4, O90: 5, O135: 6
              Point2D<int> wci =  
                bbmt->getWinningChannelIndex(currwin);
              LINFO("wchan: %d, wschan: %d", wci.i, wci.j);

              // correct for it
              currwin = correctSalientPointLocation(currwin, wci, currSalMap);
              currwin.i = int(currwin.i * width  / float(sw));
              currwin.j = int(currwin.j * height / float(sh));            

              srs->setImage(image);

              // while salient region 
              // does not land on already occupied location
              float cmval = maxval;
              Image<float> sreg = srs->getSalientRegion(currwin);
              while(sreg.initialized() && nSalReg < 5)
                {
                  LINFO("[%3d] %d %d", nSalReg, currwin.i, currwin.j);
                  maskMaps.push_back(sreg);

                  // save the salient region
                  Image<float> tsreg = sreg;
                  inplaceNormalize(tsreg, 0.0F, 255.0F);
                  resmaskfname = afstem + sformat("_sreg_%d.ppm", nSalReg);    
                  LINFO("[][%s", resmaskfname.c_str());
            
                  Raster::WriteGray(Image<byte>(tsreg), resmaskfname.c_str());
                  nSalReg++;

                  // suppress the previous salient region
                  Image<float> sregMask = downSize(sreg, salMap.getDims());
                  sregMask = convGauss<float>(sregMask,4,4,2);
                  Image<float> supMask(sregMask);
                  inplaceNormalize(supMask, 0.0F, 3.0F);
                  inplaceClamp(supMask, 0.0F, 1.0F);
                  supMask = (supMask * -1.0F) + 1.0F;
                  currSalMap *= supMask;
                
                  // // display ////////////////////////////////////////////
                  //win.reset
                  //  (new XWinManaged(Dims(width*3,height*2), 0, 0, "bmCBSE"));
                  // Image<PixRGB<byte> > disp(width*3, height*2,ZEROS);
                  // inplacePaste(disp, image, Point2D<int>(0,0));
                  // Image<byte> tbSreg(tsreg);
                  // inplacePaste(disp, makeRGB(tbSreg,tbSreg,tbSreg), 
                  //              Point2D<int>(1*width,0));

                  // sregMask = zoomXY(sregMask, 4);
                  // inplaceNormalize(sregMask, 0.0F, 255.0F);
                  // Image<byte> timask(sregMask);
                  // inplacePaste(disp, makeRGB(timask,timask,timask), 
                  //              Point2D<int>(2*width,0));    

                  // supMask = zoomXY(supMask, 4);
                  // inplaceNormalize(supMask, 0.0F, 255.0F);
                  // Image<byte> tsmask(supMask);
                  // inplacePaste(disp, makeRGB(tsmask,tsmask,tsmask), 
                  //              Point2D<int>(0,height));

                  // Image<float> tsmap = zoomXY(salMap,4);
                  // inplaceNormalize(tsmap, 0.0F, 255.0F);                
                  // Image<byte> tbsmap(tsmap);
                  // inplacePaste(disp, makeRGB(tbsmap,tbsmap,tbsmap), 
                  //              Point2D<int>(width,height));    
                
                  // Image<float> tcsmap = zoomXY(currSalMap,4);
                  // inplaceNormalize(tcsmap, 0.0F, 255.0F);                
                  // Image<byte> tbcsmap(tcsmap);
                  // inplacePaste(disp, makeRGB(tbcsmap,tbcsmap,tbcsmap), 
                  //              Point2D<int>(2*width,height));    

                  // win->drawImage(disp,0,0);
                  // Raster::waitForKey();
                  // /////////////////////////////////////////////////////////

                  // find next salient point
                  findMax(currSalMap, currwin, cmval);

                  // get the winning channel information
                  // RG: 0, BY: 1, I: 2, O0: 3, O45: 4, O90: 5, O135: 6
                  Point2D<int> wci =  
                    bbmt->getWinningChannelIndex(currwin);
                  LINFO("wchan: %d, wschan: %d", wci.i, wci.j);
                
                  // correct for it
                  currwin = correctSalientPointLocation
                    (currwin, wci, currSalMap);
                
                  currwin.i = int(currwin.i * width  / float(sw));
                  currwin.j = int(currwin.j * height / float(sh));

                  sreg = srs->getSalientRegion(currwin);
                }
            }

          // calculate precision recall, fscore
          float maxO = 0.0F; float maxA = 0.0F; float maxG = 0.0F;
          float maxP = 0.0F; float maxR = 0.0F; float maxF = 0.0F;
          for(uint k = 0; k < maskMaps.size(); k++)
            {
              // get salient region mask and bounding box                
              Rectangle r = getBoundingBox(maskMaps[k]);
            
              // get overlap
              Image<float> tempGT(width, height, ZEROS);
              for(uint l = 0; l < bi.groundTruth.size(); l++)
                {
                  Image<float> tempGTk(width, height, ZEROS);
                  drawFilledRect(tempGTk, bi.groundTruth[l], 1.0F);
                  tempGT += tempGTk;
                }
              tempGT /= float(bi.groundTruth.size());

              Image<float> overlap;
              if(r.top() != -1) overlap = crop(tempGT, r);
              float oVal = float(sum(overlap));                
              float aVal = float(r.area());
              float gVal = float(sum(tempGT));
                
              // compute Precision, Recall, F-Score
              float precision = 0.0F;
              if(aVal != 0.0F) precision = oVal/aVal;
              float recall    = oVal/gVal; 
              float alpha     = 0.5F;
              float fscore    = 0.0F;
              if(oVal != 0.0F) 
                fscore =
                  ((1.0F + alpha)*precision*recall)/
                  (alpha*precision + recall);
              LDEBUG("g: %10.3f a:%10.3f o: %10.3f :: "
                     "precision: %10.3f recall: %10.3f fscore: %10.3f",
                     gVal, aVal, oVal, precision, recall, fscore);

              // FIXXX: maybe also compute boundary based measurement BDE [45]
            
              // check for max value
              // we initially pick the first one as best guess
              if(k == 0 || maxF < fscore)
                { 
                  maxO = oVal; maxA = aVal; maxG = gVal;
                  maxP = precision; maxR = recall; maxF = fscore; 
                  srRect = r; srImage = maskMaps[k];

                  // store the results
                  bir.mindex = k;
                }

              bir.oVal.push_back(oVal);   
              bir.aVal.push_back(aVal);
              bir.gVal.push_back(gVal);
              bir.precision.push_back(precision);
              bir.recall.push_back(recall);
              bir.fscore.push_back(fscore);
              bir.algoOutputMask.push_back(maskMaps[k]);
              bir.algoOutputRect.push_back(r);
            }
      
          LINFO("[%3d] Fprecision: %10.3f recall: %10.3f fscore: %10.3f", 
                bir.mindex, maxP, maxR, maxF);

          sumO += maxO; sumA += maxA; sumG += maxG; 
          sumP += maxP; sumR += maxR; sumF += maxF;
          count++;

          isumO += maxO; isumA += maxA; isumG += maxG; 
          isumP += maxP; isumR += maxR; isumF += maxF;
          icount++;

          // this is the running average
          float avgP = sumO/sumA;
          float avgR = sumO/sumG;
          float alpha = 0.5F;
          float avgF = ((1.0F + alpha)*avgP*avgR)/(alpha*avgP + avgR);

          LINFO("%s[%5d] P: %f R: %f F: %f", tag.c_str(), count-1, avgP, avgR, avgF);
          LINFO("            P: %f R: %f F: %f", sumP/count, sumR/count, sumF/count);

          // store results        
          bir.setSortValue(maxF);
          birs.push_back(bir);
            
          // // display results
          // //drawCross(image, currwin, PixRGB<byte>(0, 0, 255), 2);
          // for(uint k = 0; k < bi.groundTruth.size();k++)
          //   drawRect(image, bi.groundTruth[k], PixRGB<byte>(255,0,0));
          // drawRect(image, srRect, PixRGB<byte>(0,255,0));
        
          // Image<PixRGB<byte> > dispImage(width*2, height*1,ZEROS);
          // inplacePaste(dispImage, image, Point2D<int>(0,0));
        
          // Image<byte> tbsr(srImage);
          // inplaceNormalize(tbsr, byte(0), byte(255));
          // inplacePaste(dispImage, makeRGB(tbsr,tbsr,tbsr), 
          //              Point2D<int>(width,0));

          // std::string dresfname =           
          //   tfname + std::string("_dres_") + tag + std::string(".png");
          // //Raster::WriteRGB(dispImage, outputFolder + '/' + dresfname);
          // Raster::WriteRGB(makeRGB(tbsr,tbsr,tbsr), outputFolder + '/' + dresfname);

          // win.reset
          //   (new XWinManaged(Dims(width*2,height*1), 0, 0, "bm SE"));
          // win->drawImage(dispImage,0,0);
          // Raster::waitForKey();
        }

      // FIXXX: MAYBE MOVE THE ACROSS BENCHMARK SUM HERE
      
      // store the summary results
      sbir.mindex = 0;
      sbir.oVal.push_back(isumO);   
      sbir.aVal.push_back(isumA);
      sbir.gVal.push_back(isumG);

      float bP = isumO/isumA;
      float bR = isumO/isumG;
      float alpha = 0.5F;
      float bF = ((1.0F + alpha)*bP*bR)/(alpha*bP + bR);

      sbir.precision.push_back(bP);
      sbir.recall.push_back(bR);
      sbir.fscore.push_back(bF);

      LINFO("B[%5d] P: %f R: %f F: %f", i, bP, bR, bF);

      benchmarks[i].results.push_back(birs);
      benchmarks[i].summaryResults.push_back(sbir);
    }
}

// ######################################################################
Rectangle getBoundingBox(Image<float> img)
{
  int top = -1, left = -1, bottom = -1, right = -1;
  int width  = img.getWidth();
  int height = img.getHeight();

  for(int i = 0; i < width; i++)
    for(int j = 0; j < height; j++)
      {
        if(img.getVal(i,j) > 0.0F)
          {
            if(top == -1) 
              { top = j; left = i; bottom = j; right = i; }
            else
              {              
                if(top    > j) top    = j;
                if(bottom < j) bottom = j;
                if(left   > i) left   = i;
                if(right  < i) right  = i;
              }
          }
      }

  // if (bb1 - tt < 0) { t = bb1; b1 = tt; } else { t = tt; b1 = bb1; }
  // if (rr1 - ll < 0) { l = rr1; r1 = ll; } else { l = ll; r1 = rr1; }


  LINFO("t: %d l: %d b: %d r: %d", top, left, bottom, right);
  return Rectangle::tlbrO(top, left, bottom, right);
}

// ######################################################################
Point2D<int> correctSalientPointLocation
(Point2D<int> pt, Point2D<int> wci, Image<float> salMap)
{
  Point2D<int> ptCorr;

  // get channel name
  int chanName = wci.i;
  int scale    = wci.j;
  LINFO("chanName: %d, scale: %d", chanName, scale);

  Point2D<int> delta(0,0);

  // from salient point get most appropriate salient region 
  // for orientation saliency the border is salient, 
  // so we move the point perpendicular to the angle 
  if(chanName >= 3 && chanName <= 6)
    {
      // FIXXX: assume there's only 4 direction
      int dir = chanName - 3;

      Point2D<int> d1, d2;
      if(dir == 0)
        { d1 = Point2D<int>( 0, -1); d2 = Point2D<int>( 0,  1); }
      else if(dir == 1)
        { d1 = Point2D<int>( 1, -1); d2 = Point2D<int>(-1,  1); }
      else if(dir == 2)
        { d1 = Point2D<int>( 1,  0); d2 = Point2D<int>(-1,  0); }
      else if(dir == 3)
        { d1 = Point2D<int>( 1,  1); d2 = Point2D<int>(-1, -1); }
      d1 = d1*2;
      d2 = d2*2;
      Point2D<int> t1 = pt + d1;
      Point2D<int> t2 = pt + d2;

      // if points are in bounds and not yet assigned
      bool vt1 = (salMap.coordsOk(t1) && (t1.i != 0 || t1.j != 0) &&
                  salMap.getVal(t1) != 0);
      bool vt2 = (salMap.coordsOk(t2) && (t2.i != 0 || t2.j != 0) &&
                  salMap.getVal(t2) != 0);
      if(vt1)
        {
          if(!vt2) delta = d1;
          else
            {              
              // go to side with higher saliency
              // we might hit background and not object
              // either way we are reducing 
              // the number of unassigned regions
              float val1 = salMap.getVal(t1.i, t1.j);
              float val2 = salMap.getVal(t2.i, t2.j);
              if(val1 > val2) delta = d1; else delta = d2;

              LINFO("t1[%3d %3d] t2[%3d %3d] "
                    "v1:%f v2:%f", 
                    t1.i, t1.j, t2.i, t2.j, val1, val2);

            }
        } else if(vt2) delta = d2;

      // if both sides are assigned, 
      // we are in the crack between two salient regions
      // should have been corrected 
    }

  // FIXXX: need to incorporate scale???
  ptCorr = pt + delta;
  LINFO("pt: %3d %3d delta: %3d %3d --> corrected point: %d %d", 
        pt.i, pt.j, delta.i, delta.j, ptCorr.i, ptCorr.j);

  return ptCorr;
}

// ######################################################################
void computeSE
(std::vector<Benchmark> &benchmarks, 
 nub::ref<BeobotBrainMT> bbmt,
 std::string outputFolder)
{
  // just in case we want to average results from all benchmarks
  float sumO = 0.0F; float sumA = 0.0F; float sumG = 0.0F; 
  float sumP = 0.0F; float sumR = 0.0F; float sumF = 0.0F;
  uint  count = 0;
  for(uint i = 0; i < benchmarks.size(); i++)
    {
      std::string bname = benchmarks[i].name;

      // insert information about CBSE algorithm
      BenchmarkItemResult avgRes;
      benchmarks[i].algorithmNames.push_back
        (std::string("Shape Estimator"));
      std::string tag("SE");
      benchmarks[i].algorithmTagNames.push_back(tag);

      std::vector<BenchmarkItemResult> birs;
      BenchmarkItemResult sbir;

      float isumO = 0.0F; float isumA = 0.0F; float isumG = 0.0F; 
      float isumP = 0.0F; float isumR = 0.0F; float isumF = 0.0F;
      uint  icount = 0;
      for(uint j = 0; j < benchmarks[i].items.size(); j++)
        {
          BenchmarkItemResult bir; 

          // get images & ground truth  
          BenchmarkItem bi = benchmarks[i].items[j];
          Image<PixRGB<byte> > image = bi.image;
          uint width  = image.getWidth();
          uint height = image.getHeight();

          // get saliency map, compute if needed
          std::string tfname = bi.fileName;
          std::replace(tfname.begin(), tfname.end(), '/', '_');
          std::string::size_type ldpos = tfname.find_last_of('.');
          tfname = tfname.substr(0, ldpos);
          std::string fstem = 
            outputFolder + '/' + bname + '/' + tfname;
          std::string afstem = fstem + '_' + tag;

          std::string ressmfname = fstem + std::string("_salmap.ppm");
          LDEBUG("resfname: %s", ressmfname.c_str());

          // if saliency map was not precomputed, compute now
          std::ifstream smf(ressmfname.c_str());
          Image<float> salMap;
          if (smf.is_open() == false) 
            {
              bbmt->input(image); 
              while(!bbmt->outputReady()) usleep(1000);
              salMap = bbmt->getSalMap();

              // save the saliency map
              Image<float> tsmap = salMap;
              inplaceNormalize(tsmap, 0.0F, 255.0F);
              Raster::WriteGray(Image<byte>(tsmap), ressmfname.c_str());
            }
          else salMap = Raster::ReadGray(ressmfname.c_str());

          // also check for the salient region mask
          Rectangle srRect; Image<float> srImage; uint nSalReg = 0;
          std::string resmaskfname = afstem + sformat("_sreg_0.ppm");
          std::ifstream srf; srf.open(resmaskfname.c_str());
          std::vector<Image<byte> > maskMaps; 
          LDEBUG("sreg[%3d][%5d]: %s",i,j, resmaskfname.c_str());

          if (srf.is_open())
            {
              // while there is another mask stored
              while(srf.is_open())
                {
                  maskMaps.push_back(Raster::ReadGray(resmaskfname.c_str()));
                  nSalReg++;
                  resmaskfname = afstem + sformat("_sreg_%d.ppm", nSalReg);
                  srf.close();
                  srf.open(resmaskfname.c_str());
                }
            }
          else 
            {
              bbmt->input(image); 
              while(!bbmt->outputReady()) usleep(1000);
              salMap = bbmt->getSalMap();

              uint numpt = bbmt->getNumSalPoint();
              uint scale = image.getWidth()/salMap.getWidth();
              LINFO("scale: %d ", scale);
              
              for(uint k = 0; k < numpt; k++)
                {
                  maskMaps.push_back(zoomXY(bbmt->getObjectMask(k), scale));

                  Point2D<int> spt = bbmt->getSalPoint(k);
                  LINFO("pt: %d %d", spt.i, spt.j);

                  // save the salient region
                  Image<float> tsreg(maskMaps[k]);
                  inplaceNormalize(tsreg, 0.0F, 255.0F);
                  resmaskfname = afstem + sformat("_sreg_%d.ppm", nSalReg);          
                  Raster::WriteGray(Image<byte>(tsreg), resmaskfname.c_str());
                  nSalReg++;

                  // // display ////////////////////////////////////////////
                  // win.reset
                  //   (new XWinManaged(Dims(width*2,height*1), 0, 0, "bm SE"));
                  // Image<PixRGB<byte> > disp(width*2, height*1,ZEROS);
                  // inplacePaste(disp, image, Point2D<int>(0,0));
                  // Image<byte> tbSreg(tsreg);
                  // inplacePaste(disp, makeRGB(tbSreg,tbSreg,tbSreg), 
                  //              Point2D<int>(1*width,0));

                  // win->drawImage(disp,0,0);
                  // Raster::waitForKey();
                  // /////////////////////////////////////////////////////////
                }
            }

          // even if there is no overlap 
          // we select the first one as best guess
          bir.mindex = 0;

          // calculate precision recall, fscore
          float maxO = 0.0F; float maxA = 0.0F; float maxG = 0.0F;
          float maxP = 0.0F; float maxR = 0.0F; float maxF = 0.0F;
          for(uint k = 0; k < maskMaps.size(); k++)
            {
              // get salient region mask and bounding box                
              Rectangle r = getBoundingBox(maskMaps[k]);
            
              // get overlap
              Image<float> tempGT(width, height, ZEROS);
              for(uint l = 0; l < bi.groundTruth.size(); l++)
                {
                  Image<float> tempGTk(width, height, ZEROS);
                  drawFilledRect(tempGTk, bi.groundTruth[l], 1.0F);
                  tempGT += tempGTk;
                }
              tempGT /= float(bi.groundTruth.size());

              Image<float> overlap;
              if(r.top() != -1) overlap = crop(tempGT, r);
              float oVal = float(sum(overlap));                
              float aVal = float(r.area());
              float gVal = float(sum(tempGT));
                
              // compute Precision, Recall, F-Score
              float precision = 0.0F;
              if(aVal != 0.0F) precision = oVal/aVal;
              float recall    = oVal/gVal; 
              float alpha     = 0.5F;
              float fscore    = 0.0F;
              if(oVal != 0.0F) 
                fscore =
                  ((1.0F + alpha)*precision*recall)/
                  (alpha*precision + recall);
              LDEBUG("g: %10.3f a:%10.3f o: %10.3f :: "
                     "precision: %10.3f recall: %10.3f fscore: %10.3f",
                     gVal, aVal, oVal, precision, recall, fscore);
                
              // FIXXX: maybe also compute boundary based measurement BDE [45]
            
              // check for max value
              // we initially pick the first one as best guess
              if(k == 0 || maxF < fscore)
                { 
                  maxO = oVal; maxA = aVal; maxG = gVal;
                  maxP = precision; maxR = recall; maxF = fscore; 
                  srRect = r; srImage = maskMaps[k];

                  bir.mindex = k;
                }

              // store the results
              bir.oVal.push_back(oVal);   
              bir.aVal.push_back(aVal);
              bir.gVal.push_back(gVal);
              bir.precision.push_back(precision);
              bir.recall.push_back(recall);
              bir.fscore.push_back(fscore);
              bir.algoOutputMask.push_back(maskMaps[k]);
              bir.algoOutputRect.push_back(r);
            }
      
          LINFO("[%5d] Fprecision: %10.3f recall: %10.3f fscore: %10.3f", 
                bir.mindex, maxP, maxR, maxF);

          sumO += maxO; sumA += maxA; sumG += maxG; 
          sumP += maxP; sumR += maxR; sumF += maxF;
          count++;

          isumO += maxO; isumA += maxA; isumG += maxG; 
          isumP += maxP; isumR += maxR; isumF += maxF;
          icount++;

          // this is the running average
          float avgP = sumO/sumA;
          float avgR = sumO/sumG;
          float alpha = 0.5F;
          float avgF = ((1.0F + alpha)*avgP*avgR)/(alpha*avgP + avgR);

          LINFO("%s[%5d] P: %f R: %f F: %f", tag.c_str(), count-1, avgP, avgR, avgF);
          LINFO("          P: %f R: %f F: %f", sumP/count, sumR/count, sumF/count);

          bir.setSortValue(maxF);
          birs.push_back(bir);
            
          // // display results
          // for(uint k = 0; k < bi.groundTruth.size();k++)
          //   drawRect(image, bi.groundTruth[k], PixRGB<byte>(255,0,0));
          // //if(maxF > 0.0F)
          //   drawRect(image, srRect, PixRGB<byte>(0,255,0));
        
          // Image<PixRGB<byte> > dispImage(width*2, height*1,ZEROS);
          // inplacePaste(dispImage, image, Point2D<int>(0,0));
        
          // Image<byte> tbsr(srImage);
          // inplaceNormalize(tbsr, byte(0), byte(255));
          // inplacePaste(dispImage, makeRGB(tbsr,tbsr,tbsr), 
          //              Point2D<int>(width,0));

          // std::string dresfname =           
          //   tfname + std::string("_dres_") + tag + std::string(".png");
          // //Raster::WriteRGB(dispImage, outputFolder + '/' + dresfname);
          // Raster::WriteRGB(makeRGB(tbsr,tbsr,tbsr), outputFolder + '/' + dresfname);

          // win.reset
          //   (new XWinManaged(Dims(width*2,height*1), 0, 0, "bm SE"));
          // win->drawImage(dispImage,0,0);
          //Raster::waitForKey();
        }

      // FIXXX: MAYBE MOVE THE ACROSS BENCHMARK SUM HERE
      
      // store the summary results
      sbir.mindex = 0;
      sbir.oVal.push_back(isumO);   
      sbir.aVal.push_back(isumA);
      sbir.gVal.push_back(isumG);

      float bP = isumO/isumA;
      float bR = isumO/isumG;
      float alpha = 0.5F;
      float bF = ((1.0F + alpha)*bP*bR)/(alpha*bP + bR);

      sbir.precision.push_back(bP);
      sbir.recall.push_back(bR);
      sbir.fscore.push_back(bF);

      LINFO("S[%5d] P: %f R: %f F: %f", i, bP, bR, bF);

      benchmarks[i].results.push_back(birs);
      benchmarks[i].summaryResults.push_back(sbir);
    }
}

// ######################################################################
void computeFromStoredMaps
(std::vector<Benchmark> &benchmarks, std::string outputFolder,
 std::string algoName, std::string algoTagName)
{
  // just in case we want to average results from all benchmarks
  float sumO = 0.0F; float sumA = 0.0F; float sumG = 0.0F; 
  float sumP = 0.0F; float sumR = 0.0F; float sumF = 0.0F;
  uint  count = 0;
  for(uint i = 0; i < benchmarks.size(); i++)
    {
      std::string bname = benchmarks[i].name;

      // insert information about CBSE algorithm
      BenchmarkItemResult avgRes;
      benchmarks[i].algorithmNames.push_back(algoName);
      std::string tag = algoTagName;
      benchmarks[i].algorithmTagNames.push_back(tag);

      std::vector<BenchmarkItemResult> birs;
      BenchmarkItemResult sbir;

      float isumO = 0.0F; float isumA = 0.0F; float isumG = 0.0F; 
      float isumP = 0.0F; float isumR = 0.0F; float isumF = 0.0F;
      uint  icount = 0;
      for(uint j = 0; j < benchmarks[i].items.size(); j++)
        {
          BenchmarkItemResult bir; 

          // get images & ground truth  
          BenchmarkItem bi = benchmarks[i].items[j];
          Image<PixRGB<byte> > image = bi.image;
          uint width  = image.getWidth();
          uint height = image.getHeight();

          std::string tfname = bi.fileName;
          std::replace(tfname.begin(), tfname.end(), '/', '_');
          std::string::size_type ldpos = tfname.find_last_of('.');
          tfname = tfname.substr(0, ldpos);
          std::string fstem = 
            outputFolder + '/' + bname + '/' + tfname;
          std::string afstem = fstem + '_' + tag;

          // also check for the salient region mask
          Rectangle srRect; Image<float> srImage; uint nSalReg = 0;
          std::string resmaskfname = afstem + sformat("_sreg_0.ppm");
          std::ifstream srf; srf.open(resmaskfname.c_str());
          std::vector<Image<byte> > maskMaps; 
          LDEBUG("sreg[%3d][%5d]: %s",i,j, resmaskfname.c_str());

          if (srf.is_open())
            {
              // while there is another mask stored
              while(srf.is_open())
                {
                  maskMaps.push_back(Raster::ReadGray(resmaskfname.c_str()));
                  nSalReg++;
                  resmaskfname = afstem + sformat("_sreg_%d.ppm", nSalReg);
                  srf.close();
                  srf.open(resmaskfname.c_str());

                  // // display ////////////////////////////////////////////
                  // win.reset
                  //   (new XWinManaged(Dims(width*2,height*1), 0, 0, "bm SE"));
                  // Image<PixRGB<byte> > disp(width*2, height*1,ZEROS);
                  // inplacePaste(disp, image, Point2D<int>(0,0));
                  // Image<byte> tbSreg(tsreg);
                  // inplacePaste(disp, makeRGB(tbSreg,tbSreg,tbSreg), 
                  //              Point2D<int>(1*width,0));

                  // win->drawImage(disp,0,0);
                  // Raster::waitForKey();
                  // /////////////////////////////////////////////////////////
                }
            }
          else LFATAL("NOT FOUND");

          // even if there is no overlap 
          // we select the first one as best guess
          bir.mindex = 0;

          // calculate precision recall, fscore
          float maxO = 0.0F; float maxA = 0.0F; float maxG = 0.0F;
          float maxP = 0.0F; float maxR = 0.0F; float maxF = 0.0F;
          for(uint k = 0; k < maskMaps.size(); k++)
            {
              // get salient region mask and bounding box                
              Rectangle r = getBoundingBox(maskMaps[k]);
            
              // get overlap
              Image<float> tempGT(width, height, ZEROS);
              for(uint l = 0; l < bi.groundTruth.size(); l++)
                {
                  Image<float> tempGTk(width, height, ZEROS);
                  drawFilledRect(tempGTk, bi.groundTruth[l], 1.0F);
                  tempGT += tempGTk;
                }
              tempGT /= float(bi.groundTruth.size());

              Image<float> overlap;
              if(r.top() != -1) overlap = crop(tempGT, r);
              float oVal = float(sum(overlap));                
              float aVal = float(r.area());
              float gVal = float(sum(tempGT));
                
              // compute Precision, Recall, F-Score
              float precision = 0.0F;
              if(aVal != 0.0F) precision = oVal/aVal;
              float recall    = oVal/gVal; 
              float alpha     = 0.5F;
              float fscore    = 0.0F;
              if(oVal != 0.0F) 
                fscore =
                  ((1.0F + alpha)*precision*recall)/
                  (alpha*precision + recall);
              LDEBUG("g: %10.3f a:%10.3f o: %10.3f :: "
                     "precision: %10.3f recall: %10.3f fscore: %10.3f",
                     gVal, aVal, oVal, precision, recall, fscore);
                
              // FIXXX: maybe also compute boundary based measurement BDE [45]
            
              // check for max value
              // we initially pick the first one as best guess
              if(k == 0 || maxF < fscore)
                { 
                  maxO = oVal; maxA = aVal; maxG = gVal;
                  maxP = precision; maxR = recall; maxF = fscore; 
                  srRect = r; srImage = maskMaps[k];

                  bir.mindex = k;
                }

              // store the results
              bir.oVal.push_back(oVal);   
              bir.aVal.push_back(aVal);
              bir.gVal.push_back(gVal);
              bir.precision.push_back(precision);
              bir.recall.push_back(recall);
              bir.fscore.push_back(fscore);
              bir.algoOutputMask.push_back(maskMaps[k]);
              bir.algoOutputRect.push_back(r);
            }
      
          LINFO("[%5d] Fprecision: %10.3f recall: %10.3f fscore: %10.3f", 
                bir.mindex, maxP, maxR, maxF);

          sumO += maxO; sumA += maxA; sumG += maxG; 
          sumP += maxP; sumR += maxR; sumF += maxF;
          count++;

          isumO += maxO; isumA += maxA; isumG += maxG; 
          isumP += maxP; isumR += maxR; isumF += maxF;
          icount++;

          // this is the running average
          float avgP = sumO/sumA;
          float avgR = sumO/sumG;
          float alpha = 0.5F;
          float avgF = ((1.0F + alpha)*avgP*avgR)/(alpha*avgP + avgR);

          LINFO("%s[%5d] P: %f R: %f F: %f", 
                tag.c_str(), count-1, avgP, avgR, avgF);
          LINFO("          P: %f R: %f F: %f", 
                sumP/count, sumR/count, sumF/count);

          bir.setSortValue(maxF);
          birs.push_back(bir);
            
          // // display results
          // for(uint k = 0; k < bi.groundTruth.size();k++)
          //   drawRect(image, bi.groundTruth[k], PixRGB<byte>(255,0,0));
          // //if(maxF > 0.0F)
          //   drawRect(image, srRect, PixRGB<byte>(0,255,0));
        
          // Image<PixRGB<byte> > dispImage(width*2, height*1,ZEROS);
          // inplacePaste(dispImage, image, Point2D<int>(0,0));
        
          // Image<byte> tbsr(srImage);
          // inplaceNormalize(tbsr, byte(0), byte(255));
          // inplacePaste(dispImage, makeRGB(tbsr,tbsr,tbsr), 
          //              Point2D<int>(width,0));

          // std::string dresfname =           
          //   tfname + std::string("_dres_") + tag + std::string(".png");
          // //Raster::WriteRGB(dispImage, outputFolder + '/' + dresfname);
          // Raster::WriteRGB(makeRGB(tbsr,tbsr,tbsr), outputFolder + '/' + dresfname);

          // win.reset
          //   (new XWinManaged(Dims(width*2,height*1), 0, 0, "bm SE"));
          // win->drawImage(dispImage,0,0);
          //Raster::waitForKey();
        }

      // FIXXX: MAYBE MOVE THE ACROSS BENCHMARK SUM HERE
      
      // store the summary results
      sbir.mindex = 0;
      sbir.oVal.push_back(isumO);   
      sbir.aVal.push_back(isumA);
      sbir.gVal.push_back(isumG);

      float bP = isumO/isumA;
      float bR = isumO/isumG;
      float alpha = 0.5F;
      float bF = ((1.0F + alpha)*bP*bR)/(alpha*bP + bR);

      sbir.precision.push_back(bP);
      sbir.recall.push_back(bR);
      sbir.fscore.push_back(bF);

      LINFO("S[%5d] P: %f R: %f F: %f", i, bP, bR, bF);

      benchmarks[i].results.push_back(birs);
      benchmarks[i].summaryResults.push_back(sbir);
    }
}

// ######################################################################
void computeMiddleRectangle
(std::vector<Benchmark> &benchmarks, std::string outputFolder)
{
  // just in case we want to average results from all benchmarks
  float sumO = 0.0F; float sumA = 0.0F; float sumG = 0.0F; 
  float sumP = 0.0F; float sumR = 0.0F; float sumF = 0.0F;
  uint  count = 0;
  for(uint i = 0; i < benchmarks.size(); i++)
    {
      std::string bname = benchmarks[i].name;

      // insert information about the algorithm
      BenchmarkItemResult avgRes;
      benchmarks[i].algorithmNames.push_back
        (std::string("Middle Rectangle"));
      std::string tag("MR");
      benchmarks[i].algorithmTagNames.push_back(tag);

      std::vector<BenchmarkItemResult> birs;
      BenchmarkItemResult sbir;

      float isumO = 0.0F; float isumA = 0.0F; float isumG = 0.0F; 
      float isumP = 0.0F; float isumR = 0.0F; float isumF = 0.0F;
      uint  icount = 0;
      for(uint j = 0; j < benchmarks[i].items.size(); j++)
        {
          BenchmarkItemResult bir; 

          // get images & ground truth  
          BenchmarkItem bi = benchmarks[i].items[j];
          Image<PixRGB<byte> > image = bi.image;
          uint width  = image.getWidth();
          uint height = image.getHeight();

          std::string tfname = bi.fileName;
          std::replace(tfname.begin(), tfname.end(), '/', '_');
          std::string::size_type ldpos = tfname.find_last_of('.');
          tfname = tfname.substr(0, ldpos);
          std::string fstem = 
            outputFolder + '/' + bname + '/' + tfname;
          std::string afstem = fstem + '_' + tag;

          // also check for the salient region mask
          Rectangle srRect; Image<float> srImage; uint nSalReg = 0;
          std::string resmaskfname = afstem + sformat("_sreg_0.ppm");
          std::ifstream srf; srf.open(resmaskfname.c_str());
          std::vector<Image<byte> > maskMaps; 
          LDEBUG("sreg[%3d][%5d]: %s",i,j, resmaskfname.c_str());

          if (srf.is_open())
            {
              // while there is another mask stored
              while(srf.is_open())
                {
                  maskMaps.push_back(Raster::ReadGray(resmaskfname.c_str()));
                  nSalReg++;
                  resmaskfname = afstem + sformat("_sreg_%d.ppm", nSalReg);
                  srf.close();
                  srf.open(resmaskfname.c_str());

                  // // display ////////////////////////////////////////////
                  // win.reset
                  //   (new XWinManaged(Dims(width*2,height*1), 0, 0, "bm SE"));
                  // Image<PixRGB<byte> > disp(width*2, height*1,ZEROS);
                  // inplacePaste(disp, image, Point2D<int>(0,0));
                  // Image<byte> tbSreg(tsreg);
                  // inplacePaste(disp, makeRGB(tbSreg,tbSreg,tbSreg), 
                  //              Point2D<int>(1*width,0));

                  // win->drawImage(disp,0,0);
                  // Raster::waitForKey();
                  // /////////////////////////////////////////////////////////
                }
            }
          else 
            {
              Image<float> rimg(width, height, ZEROS);
              Rectangle r
                (Point2D<int>(width/4, height/4), 
                 Dims(width/2, height/2));
              drawFilledRect(rimg, r, 255.0F);
              maskMaps.push_back(rimg);
            }

          // even if there is no overlap 
          // we select the first one as best guess
          bir.mindex = 0;

          // calculate precision recall, fscore
          float maxO = 0.0F; float maxA = 0.0F; float maxG = 0.0F;
          float maxP = 0.0F; float maxR = 0.0F; float maxF = 0.0F;
          for(uint k = 0; k < maskMaps.size(); k++)
            {
              // get salient region mask and bounding box                
              Rectangle r = getBoundingBox(maskMaps[k]);
            
              // get overlap
              Image<float> tempGT(width, height, ZEROS);
              for(uint l = 0; l < bi.groundTruth.size(); l++)
                {
                  Image<float> tempGTk(width, height, ZEROS);
                  drawFilledRect(tempGTk, bi.groundTruth[l], 1.0F);
                  tempGT += tempGTk;
                }
              tempGT /= float(bi.groundTruth.size());

              Image<float> overlap;
              if(r.top() != -1) overlap = crop(tempGT, r);
              float oVal = float(sum(overlap));                
              float aVal = float(r.area());
              float gVal = float(sum(tempGT));
                
              // compute Precision, Recall, F-Score
              float precision = 0.0F;
              if(aVal != 0.0F) precision = oVal/aVal;
              float recall    = oVal/gVal; 
              float alpha     = 0.5F;
              float fscore    = 0.0F;
              if(oVal != 0.0F) 
                fscore =
                  ((1.0F + alpha)*precision*recall)/
                  (alpha*precision + recall);
              LDEBUG("g: %10.3f a:%10.3f o: %10.3f :: "
                     "precision: %10.3f recall: %10.3f fscore: %10.3f",
                     gVal, aVal, oVal, precision, recall, fscore);
                
              // FIXXX: maybe also compute boundary based measurement BDE [45]
            
              // check for max value
              // we initially pick the first one as best guess
              if(k == 0 || maxF < fscore)
                { 
                  maxO = oVal; maxA = aVal; maxG = gVal;
                  maxP = precision; maxR = recall; maxF = fscore; 
                  srRect = r; srImage = maskMaps[k];

                  bir.mindex = k;
                }

              // store the results
              bir.oVal.push_back(oVal);   
              bir.aVal.push_back(aVal);
              bir.gVal.push_back(gVal);
              bir.precision.push_back(precision);
              bir.recall.push_back(recall);
              bir.fscore.push_back(fscore);
              bir.algoOutputMask.push_back(maskMaps[k]);
              bir.algoOutputRect.push_back(r);
            }
      
          LINFO("[%5d] Fprecision: %10.3f recall: %10.3f fscore: %10.3f", 
                bir.mindex, maxP, maxR, maxF);

          sumO += maxO; sumA += maxA; sumG += maxG; 
          sumP += maxP; sumR += maxR; sumF += maxF;
          count++;

          isumO += maxO; isumA += maxA; isumG += maxG; 
          isumP += maxP; isumR += maxR; isumF += maxF;
          icount++;

          // this is the running average
          float avgP = sumO/sumA;
          float avgR = sumO/sumG;
          float alpha = 0.5F;
          float avgF = ((1.0F + alpha)*avgP*avgR)/(alpha*avgP + avgR);

          LINFO("%s[%5d] P: %f R: %f F: %f", 
                tag.c_str(), count-1, avgP, avgR, avgF);
          LINFO("          P: %f R: %f F: %f", 
                sumP/count, sumR/count, sumF/count);

          bir.setSortValue(maxF);
          birs.push_back(bir);
            
          // // display results
          // for(uint k = 0; k < bi.groundTruth.size();k++)
          //   drawRect(image, bi.groundTruth[k], PixRGB<byte>(255,0,0));
          // //if(maxF > 0.0F)
          //   drawRect(image, srRect, PixRGB<byte>(0,255,0));
        
          // Image<PixRGB<byte> > dispImage(width*2, height*1,ZEROS);
          // inplacePaste(dispImage, image, Point2D<int>(0,0));
        
          // Image<byte> tbsr(srImage);
          // inplaceNormalize(tbsr, byte(0), byte(255));
          // inplacePaste(dispImage, makeRGB(tbsr,tbsr,tbsr), 
          //              Point2D<int>(width,0));

          // std::string dresfname =           
          //   tfname + std::string("_dres_") + tag + std::string(".png");
          // //Raster::WriteRGB(dispImage, outputFolder + '/' + dresfname);
          // Raster::WriteRGB(makeRGB(tbsr,tbsr,tbsr), outputFolder + '/' + dresfname);

          // win.reset
          //   (new XWinManaged(Dims(width*2,height*1), 0, 0, "bm SE"));
          // win->drawImage(dispImage,0,0);
          //Raster::waitForKey();
        }

      // FIXXX: MAYBE MOVE THE ACROSS BENCHMARK SUM HERE
      
      // store the summary results
      sbir.mindex = 0;
      sbir.oVal.push_back(isumO);   
      sbir.aVal.push_back(isumA);
      sbir.gVal.push_back(isumG);

      float bP = isumO/isumA;
      float bR = isumO/isumG;
      float alpha = 0.5F;
      float bF = ((1.0F + alpha)*bP*bR)/(alpha*bP + bR);

      sbir.precision.push_back(bP);
      sbir.recall.push_back(bR);
      sbir.fscore.push_back(bF);

      LINFO("S[%5d] P: %f R: %f F: %f", i, bP, bR, bF);

      benchmarks[i].results.push_back(birs);
      benchmarks[i].summaryResults.push_back(sbir);
    }
}

// ######################################################################
// void displayResults
// (std::string fName, Benchmark benchmark, 
//  std::string outputFolder, std::string webOutputFolder)
// {
//   // std::string bname = benchmark.name;
//   // uint numItems     = benchmark.items.size();

//   // we are going to devide the web page to 100 images/page

//   // fName

//   //   std::string tfname = bi.fileName;

//   // std::replace(tfname.begin(), tfname.end(), '/', '_');

//   // std::string::size_type ldpos = fname.find_last_of('.');



//   // std::string stem = fname.substr(0, ldpos);
//   // std::string fext = fname.substr(ldpos+1);



//   // std::string fstem = 
//   //   outputFolder + '/' + bname + '/' + tfname;
//   // std::string afstem = fstem + '_' + tag;

// }

// ######################################################################
void displayResults
(std::string fName, Benchmark benchmark,
 std::string outputFolder, std::string webOutputFolder)
//(std::string fName, Benchmark benchmark, uint sindex, uint eindex,
// std::string outputFolder, std::string webOutputFolder)
{
  std::string bname = benchmark.name;
  //uint numItems     = benchmark.items.size();

  // open file
  // open an environment file
  FILE * dfile = fopen(fName.c_str(), "wt");
  if (dfile ==NULL) LFATAL("can't create file: %s", fName.c_str());
  LINFO("creating result HTML file %s", fName.c_str());

  // header
  std::string header("<html>\n");
  fputs (header.c_str(), dfile);
  header = std::string("<head>\n");
  fputs (header.c_str(), dfile);
  header = std::string
    ("<meta http-equiv=\"Content-Language\" content=\"en-us\">\n");
  fputs (header.c_str(), dfile);
  header = std::string
    ("<meta http-equiv=\"Content-Type\" "
     "content=\"text/html; charset=gb2312\">\n");
  fputs (header.c_str(), dfile);
  header = std::string
    ("<title>Salient Region Detection Benchmark</title>\n");
  fputs (header.c_str(), dfile);
  header = std::string("</head>\n");
  fputs (header.c_str(), dfile);

  // body
  std::string body("<body>\n");
  fputs (body.c_str(), dfile);
  body = std::string("<p dir=\"ltr\"><b><font face=\"Arial\">"
                     "Salient Region Detection</font></b></p>\n");
  fputs (body.c_str(), dfile);

  // set the image size to 200x150, a 4:3 aspect ratio
  uint width  = 200; 
  uint height = 150;
  float ar = float(width)/float(height);
  uint numAlgorithms = benchmark.algorithmNames.size();

  uint nspaces = (numAlgorithms+1)*width + 12;

  // introduction
  body = 
    std::string("The following table the benchmark Results " 
                "for the Liu etal. 2007 dataset. <br><br><br>\n");
  fputs (body.c_str(), dfile);

  // start table
  std::string table =
    sformat("<table border=\"1\" width=\"%d"
            "\" id=\"table11\" cellspacing=\"3\">\n", nspaces);
  fputs (table.c_str(), dfile);

  table = std::string("<tr>\n");
  fputs (table.c_str(), dfile);

  table = sformat("<td width=\"%d\" align=\"center\">\n", width);
  fputs (table.c_str(), dfile);

  table = std::string("<b>Input Image</b></td>\n");
  fputs (table.c_str(), dfile);

  // put the algorithm names on the column
  for(uint i = 0; i < benchmark.algorithmNames.size(); i++)
    {
      PixRGB<byte> c = algColorLabels[(i+1)%algColorLabels.size()];
      std::string color = sformat("%02x%02x%02x", c.red(), c.green(), c.blue());

      table = 
        sformat("<td bgcolor=\"%s\" width=\"%d\" align=\"center\">\n", 
                color.c_str(), width);
      fputs (table.c_str(), dfile);

      table = 
        sformat("<b>%s</b>\n</td>\n",
                benchmark.algorithmNames[i].c_str());
      fputs (table.c_str(), dfile);
    }

  table = std::string("</tr>\n\n");
  fputs (table.c_str(), dfile);

  // table content 

  table = std::string("<tr>\n");
  fputs (table.c_str(), dfile);

  // overall benchmark results: precision, recall, and fscore

  // precision
  table = sformat("<td width=\"%d\" align=\"center\">" 
                  "Precision</td>\n", width);
  fputs (table.c_str(), dfile);
  for(uint i = 0; i < benchmark.algorithmNames.size(); i++)
    {
      BenchmarkItemResult sbir = benchmark.summaryResults[i];

      std::string tentry = 
        sformat("<td width=\"%d\" align=\"center\">" 
                "%4.3f</td>\n", width, sbir.precision[0]);
      fputs (tentry.c_str(), dfile);
    }

  table = std::string("</tr>\n\n");
  fputs (table.c_str(), dfile);

  // recall
  table = sformat("<td width=\"%d\" align=\"center\">" 
                  "Recall</td>\n", width);
  fputs (table.c_str(), dfile);
  for(uint i = 0; i < benchmark.algorithmNames.size(); i++)
    {
      BenchmarkItemResult sbir = benchmark.summaryResults[i];

      std::string tentry = 
        sformat("<td width=\"%d\" align=\"center\">"
                "%4.3f</td>\n", width, sbir.recall[0]);
      fputs (tentry.c_str(), dfile);
    }

  table = std::string("</tr>\n\n");
  fputs (table.c_str(), dfile);

  // precision
  table = sformat("<td width=\"%d\" align=\"center\">" 
                  "F-Score</td>\n", width);
  fputs (table.c_str(), dfile);
  for(uint i = 0; i < benchmark.algorithmNames.size(); i++)
    {
      BenchmarkItemResult sbir = benchmark.summaryResults[i];

      std::string tentry = 
        sformat("<td width=\"%d\" align=\"center\" >"
                "%4.3f</td>\n", width, sbir.fscore[0]);
      fputs (tentry.c_str(), dfile);
    }

  table = std::string("</tr>\n\n");
  fputs (table.c_str(), dfile);
                  
  uint nColor = algColorLabels.size();
  
  // for each entry in benchmark
  for(uint j = 0; j < benchmark.items.size(); j++)
    {
      std::string tentry = 
        sformat("<tr>\n<td width=\"%d\" align=\"center\" >\n",
                width);
      fputs (tentry.c_str(), dfile);
        
      // create the image
      Image<PixRGB<byte> > image = benchmark.items[j].image;          

      // draw the ground truth
      BenchmarkItem bi = benchmark.items[j];
      for(uint k = 0; k < bi.groundTruth.size();k++)
        drawRect(image, bi.groundTruth[k],  algColorLabels[0]);

      // draw the rectangles
      for(uint k = 0; k < benchmark.results.size(); k++)
        if(j < benchmark.results[k].size())
          {
            BenchmarkItemResult bir = benchmark.results[k][j];
            int mi = bir.mindex;
            PixRGB<byte> c = algColorLabels[(k+1)%nColor];
            drawRect(image, bir.algoOutputRect[mi], c);
          }
      // if this combination of algorithm and data exists
 

      // get the image size
      uint iw = image.getWidth();
      uint ih = image.getHeight();

      uint dw = iw;
      uint dh = uint(float(iw)/ar+0.5F);   
      if(dh < ih)
        {
          dw = uint(float(ih)*ar+0.5F);
          dh = ih;
        }

      // display already corrects for aspect ratio
      Image<PixRGB<byte> > disp(dw, dh, ZEROS);
      Point2D<int> org
        (int((float(dw) - float(iw))/2), 
         int((float(dh) - float(ih))/2) );
      inplacePaste(disp, image, org);

      // copy the image to the folder if needed
      std::string tfname = bi.fileName;
      std::replace(tfname.begin(), tfname.end(), '/', '_');
      std::string::size_type ldpos = tfname.find_last_of('.');
      tfname = tfname.substr(0, ldpos);

      std::string disprfname = 
        bname + '/' + tfname + std::string("_disp_res.png");
      LINFO("fn:: %s/%s", outputFolder.c_str(), disprfname.c_str());
      Raster::WriteRGB(disp, outputFolder + '/' + disprfname);

      // image file
      tentry = 
        sformat("<img border=\"0\" src=\""
                "%s/%s\" width=\"%d\" height=\"%d \">\n", 
                webOutputFolder.c_str(), disprfname.c_str(), width, height);
      fputs (tentry.c_str(), dfile);

      // image name
      tentry = sformat("[%d] %s\n</td>\n", 
                       j, benchmark.items[j].fileName.c_str());
      fputs (tentry.c_str(), dfile);
        
      // for each algorithm
      for(uint k = 0; k < benchmark.results.size(); k++)
        {
          std::string tag = benchmark.algorithmTagNames[k];

          PixRGB<byte> c = algColorLabels[(k+1)];
          std::string color = 
            sformat("%02x%02x%02x", c.red(), c.green(), c.blue());
          tentry = 
            sformat("<td bgcolor=\"%s\" width=\"%d\" align=\"center\" >\n",
                    color.c_str(), width);
          fputs (tentry.c_str(), dfile);            

          // if this combination of algorithm and data exists
          if(j < benchmark.results[k].size())
            {
              BenchmarkItemResult bir = benchmark.results[k][j];

              int mi = bir.mindex;
              // LINFO("mindex mi: %d", mi);
                
              Image<float> adisp(dw, dh, ZEROS);
              inplacePaste(adisp, bir.algoOutputMask[mi], org);
              inplaceNormalize(adisp, 0.0F, 255.0F);

              // copy the image to the folder if needed
              std::string disparfname = 
                bname + '/' + tfname + '_' + tag + std::string("_disp_res.png");
              LINFO("af::: %s/%s", outputFolder.c_str(), disparfname.c_str());
              Raster::WriteRGB(adisp, outputFolder + '/' + disparfname);

              // float mn,mx;
              // getMinMax(adisp, mn, mx);
              // LINFO("mn: %f mx:  %f",mn, mx);

              // win.reset(new XWinManaged(Dims(dw, dh), 0, 0, "bm SE"));
              // win->drawImage(adisp,0,0);
              // Raster::waitForKey();

              tentry = 
                sformat("<img border=\"0\" src=\""
                        "%s/%s\"width=\"%d\" height=\"%d \">\n", 
                        webOutputFolder.c_str(), disparfname.c_str(), width, height);

              fputs (tentry.c_str(), dfile);

              // display the precision, recall, and fscore
              tentry = 
                sformat("p: %4.3f r: %4.3f f: %4.3f\n",
                        bir.precision[mi], bir.recall[mi], bir.fscore[mi]);

              fputs (tentry.c_str(), dfile);
            }
          else
            {
              tentry = 
                sformat("<img border=\"0\" src=\""
                        "\"width=\"%d\" height=\"%d \">\n", 
                        width, height);
              fputs (tentry.c_str(), dfile);
            }

          tentry = std::string("</td>\n");
          fputs (tentry.c_str(), dfile);
        }

      tentry = std::string("</tr>\n\n");
      fputs (tentry.c_str(), dfile);
    }

  // ender
  body = std::string("</table>\n");
  fputs (body.c_str(), dfile);
  
  body = std::string("</body>\n");
  fputs (body.c_str(), dfile);
  
  body = std::string("</html>\n");
  fputs (body.c_str(), dfile);
  
  fclose (dfile);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
