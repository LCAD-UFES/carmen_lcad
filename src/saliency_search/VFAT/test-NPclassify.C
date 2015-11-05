/*!@file VFAT/test-NPclassify.C  Test the non-parametric classifier */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/test-NPclassify.C $
// $Id: test-NPclassify.C 14376 2011-01-11 02:44:34Z pez $
//

// ############################################################
// ############################################################
// ##### ---NPclassify---
// ##### non-parametric classifier:
// ##### T. Nathan Mundhenk nathan@mundhenk.com
// ##### Vidhya Navalpakkam - navalpak@usc.edu
// ##### partners full name - email
// ############################################################
// ############################################################

//This is the start of the execution path for the NPclassify test alg.

#include "VFAT/NPclassify2.H"
#include "Raster/Raster.H"
#include "VFAT/covEstimate.H"
#include "VFAT/findColorIndex.H"
#include "Util/Timer.H"

#define INVALS 31

//! This is the configFile name
char* configFile;
//! This is the configFile object
readConfig configIn(25);
readConfig polySet(25);
//! generic pixel
PixRGB<float> pix;
//! number of items if training
int itemNumber;
float inVals[INVALS];

int main(int argc, char* argv[])
{
  // start timer
  Timer tim;
  tim.reset();
  uint64 t0 = tim.get();  // to measure display time
  // get test image
  Image<byte> input = Raster::ReadGray(argv[1]);
  if(argc > 2)
    itemNumber = atoi(argv[2]);
  else
    itemNumber = -666;
  Image<float> finput = input;
  Image<float> outputDensity;
  Image<PixRGB<float> > outputLinks;
  outputDensity.resize(input.getWidth(),input.getHeight());
  outputLinks.resize((input.getWidth()*2),input.getHeight());
  // create operating objects
  configIn.openFile("NPclassify.conf");
  polySet.openFile("polySet.conf");

  std::vector<float> feature(2,0);
  std::vector<std::vector<float> > features(50,feature);
  std::vector<int*> roots;
  std::vector<int*> parents;
  std::vector<float> density;
  std::vector<int> priorItemVal(50,0);

  long featureCount = 0;

  // convert test image to vector format
  for(int x = 0; x < finput.getWidth(); x++)
  {
    for(int y = 0; y < finput.getHeight();y++)
    {
      if(finput.getVal(x,y) < 254.0F)
      {
        // resize vector if needed
        if((unsigned)(featureCount) == features.size())
        {
          features.resize((featureCount+10),feature);
          priorItemVal.resize((featureCount+10),0);
          //LINFO("RESIZED SPACE %d",featureCount+50);
        }
        // insert x and y into vector
        priorItemVal[featureCount] = (int)finput.getVal(x,y);
        features[featureCount][0] = x;
        features[featureCount][1] = y;
        featureCount++;
      }
    }
  }


  std::vector<int> priorItemMember(featureCount,0);
  std::vector<int> priorClassVal(featureCount,0);
  std::vector<int> priorClassSize(featureCount,0);
  int priorClasses = 0;

  for(int i = 0; i < featureCount; i++)
  {
    bool add = true;
    for(int j = 0; j < priorClasses; j++)
    {
      if(priorItemVal[i] == priorClassVal[j])
      {
        LINFO("ADDED %d to class %d, value %d",i,j,
              priorItemVal[i]);
        priorItemMember[i] = j;
        priorClassSize[j]++;
        add = false;
      }
    }
    if(add == true)
    {
      priorClassVal[priorClasses] = priorItemVal[i];
      priorItemMember[i] = priorClasses;
      priorClassSize[priorClasses]++;
      priorClasses++;
    }
  }




  LINFO("Number of feature vectors %ld",featureCount);

  // (1) create the NP classify object, input conf files, specify if settings
  // are from command line
  NPclassify2<float> NP(configIn,polySet,false);
  // (2) input any command line arguments if any
  if(argc > 3)
  {
    for(int i = 0; i < INVALS; i++)
    {
      inVals[i] = atof(argv[4+i]);
    }
    NP.NPinputCommandLineSettings(inVals);
  }


  //----------------------------------------------------------------//

  // classify space using density webs
  // (3) specify the size of your space in samples and dimensions
  NP.NPresizeSpace(featureCount,2);
  // (4) input the current vector into the NP clusterer
  NP.NPaddSpace(features);
  // (5) start the alg.
  NP.NPclassifySpaceNew(false);
  std::cerr << "DONE\n";

  uint64 t1 = tim.get();
  t0 = t1 - t0;
  //LINFO("classifySpaceNew took %dms for image", t0);
  LINFO("Get Return Info");
  roots = NP.NPgetStems();
  parents = NP.NPgetParents();
  density = NP.NPgetDensity();

  std::vector<std::vector<int*> > theReturn = NP.NPgetChildren();

  std::vector<std::vector<int> > BB = NP.NPgetBoundingBoxes(false);
  LINFO("Draw Links");
  // draw links
  for(int i = 0; i < featureCount; i++)
  {
    float setValue = (float)(density[i]/NP.NPgetMaxDensity())*255;
    outputDensity.setVal((int)features[i][0],(int)features[i][1],setValue);

    pix.setRed(255),pix.setGreen(0);pix.setBlue(0);
    outputLinks.setVal(((int)features[i][0]+input.getWidth())
                       ,((int)features[i][1]),pix);
    pix.setRed(0),pix.setGreen(255);pix.setBlue(0);

    if(*parents[i] != -1)
    {
      for(int j = 0; j < NP.NPgetStemNumber(); j++)
      {
        if(*roots[j] == i)
        {
          pix.setRed(255);
          pix.setGreen(0);
        }
      }
      drawLine(outputLinks, Point2D<int>((int)features[i][0],(int)features[i][1]),
               Point2D<int>((int)features[*parents[i]][0]
                       ,(int)features[*parents[i]][1]),pix,1);
    }
  }
  LINFO("Draw Bounding Box");
  // draw bounding boxes
  pix.setRed(0),pix.setGreen(0);pix.setBlue(255);
  NP.NPdrawBoundingBoxes(&BB,&outputLinks,0,0,pix,true);
  //draw each class in a seperate image
  if(NP.NP_doClassMap == 1)
  {
    LINFO("Draw Class Images");
    Image<PixRGB<float> > classMap;
    Image<PixRGB<float> > priorClassMap;

    PixRGB<float> mrPixel;
    classMap.resize(finput.getWidth(),finput.getHeight());
    priorClassMap.resize(finput.getWidth(),finput.getHeight());
    findColorIndex FAC;

    pix.setRed(255),pix.setGreen(0);pix.setBlue(0);
    std::vector<int> sortClassSize(NP.NPgetStemNumber()+1,0);
    std::vector<int> sortClassMember(NP.NPgetStemNumber()+1,0);
    int sortCount = 0;
    int realClasses = 0;

    //**********************************************************
    // draw cov matrix
    Image<float> final;
    final.resize(finput.getWidth(),finput.getWidth());
    for(int i = 0; i < NP.NPgetStemNumber(); i++)
    {
      int classSize = NP.NPgetClassSize(i);
      if(NP.NPgetMinClassSize() <= classSize)
      {
        realClasses++;
        float t = 0.0F;
        float* tfloat = &t;
        std::vector<float> _vinput(classSize,0);
        std::vector<float*> _vTinput(classSize,tfloat);
        std::vector<std::vector<float> > NPclass(2,_vinput);
        std::vector<std::vector<float*> > NPclassP(2,_vTinput);
        for(int cs = 0; cs < classSize; cs++)
        {
          long item = NP.NPgetClass(i,cs);
          NPclass[0][cs] = finput.getWidth() - NP.NPgetFeature(item,0);
          std::cerr << "INPUTING " << NPclass[0][cs] << "\n";
          NPclass[1][cs] = NP.NPgetFeature(item,1);
          std::cerr << "INPUTING " << NPclass[1][cs] << "\n";
          NPclassP[0][cs] = &NPclass[0][cs];
          NPclassP[1][cs] = &NPclass[1][cs];
        }
        covHolder<float> covh;
        covh.resize(2,classSize,0.0F);
        covEstimate<float> CE(NPclassP,covh);
        CE.run();
        final = CE.returnCovSlice(0,1,final);
      }
    }
    Raster::VisuFloat(final,0,sformat("%s.outputCov.pgm",argv[1]));


    for(int i = 0; i < featureCount; i++)
    {
      unsigned int foo = (unsigned)priorItemMember[i];
      FAC.FACgetColor12(&foo,&mrPixel);
      priorClassMap.setVal((int)features[i][0]
                           ,(int)features[i][1],mrPixel);
    }

    //*******************************************************
    // sort classes based upon size
    for(int i = 0; i < NP.NPgetStemNumber(); i++)
    {
      // if I am the first class, set me as number 1
      if(sortCount == 0)
      {
        //LINFO("SETTING first class at %d size %d",i,NP.NPgetClassSize(i));
        sortClassSize[0] = NP.NPgetClassSize(i);
        sortClassMember[0] = i;
        sortCount++;
      }
      else
      {
        bool setThis = false;
        // look through the entire list in order
        int initSC = sortCount;
        for(int j = 0; j < initSC; j++)
        {
          // if I am bigger than someone, bump him and
          // everyone else back one, insert me
          if(NP.NPgetClassSize(i) > sortClassSize[j])
          {
            //LINFO("SETTING CLASS %d as larger than %d",i,sortClassMember[j]);
            ///LINFO("...SIZE %d > %d",NP.NPgetClassSize(i),sortClassSize[j]);
            //LINFO("...COUNT %d",j);
            setThis = true;
            int tempClassSize;
            int tempClassNum;
            int newClassSize = NP.NPgetClassSize(i);
            int newClassNum = i;
            for(int k = j; k <= sortCount; k++)
            {
              tempClassSize = sortClassSize[k];
              tempClassNum = sortClassMember[k];
              //LINFO("SETTING %d as class %d",k,newClassNum);
              sortClassSize[k] = newClassSize;
              sortClassMember[k] = newClassNum;
              newClassSize = tempClassSize;
              newClassNum = tempClassNum;
            }
            break;
          }
        }
        if(setThis == false)
        {

          //LINFO("SETTING default at %d size %d",i,NP.NPgetClassSize(i));
          //LINFO("...NUMBER %d",sortCount);
          sortClassSize[sortCount] = NP.NPgetClassSize(i);
          sortClassMember[sortCount] = i;
        }
        sortCount++;
      }
    }

    //*******************************************************
    // draw output images
    for(int i = 0; i < sortCount; i++)
    {
      unsigned int number;
      int *thisClass = &sortClassMember[i];
      LINFO("%d : CLASS %d SIZE %d",i,*thisClass
            ,NP.NPgetClassSize(*thisClass));
      number = (unsigned)(i + 1);

      //if(NP.NPisStem(*thisClass) == true)
      //  number = 0;
      LINFO("SETTING PIXEL %d",number);
      FAC.FACgetColor12(&number,&mrPixel);
      Image<PixRGB<float> > outputClass;
      outputClass.resize(finput.getWidth(),finput.getHeight());
      std::cerr << "class size " <<  NP.NPgetClassSize(*thisClass) << "\n";
      for(int j = 0; j < NP.NPgetClassSize(*thisClass); j++)
      {
        long item = NP.NPgetClass(*thisClass,j);
        outputClass.setVal((int)NP.NPgetFeature(item,0)
                           ,(int)NP.NPgetFeature(item,1),pix);
        classMap.setVal((int)NP.NPgetFeature(item,0)
                           ,(int)NP.NPgetFeature(item,1),mrPixel);
        if(j == 0)
        {
          if(NP.NPgetMinClassSize() <= NP.NPgetClassSize(*thisClass))
          {
            char foo;
            sprintf(&foo,"%d",*thisClass);
            writeText(classMap, Point2D<int>((int)NP.NPgetFeature(item,0),
                                        (int)NP.NPgetFeature(item,1)),
                      &foo,PixRGB<float>(255),PixRGB<float>(0));
          }
        }
      }
      //Raster::VisuRGB(outputClass,sformat("CLASS_%d.ppm",i));
    }

    //*******************************************************
    // stats on variance
    std::vector<float> meanInterClassVar(priorClasses,0.0F);
    std::vector<float> stdInterClassVar(priorClasses,0.0F);
    std::vector<int> nInterClassVar(priorClasses,0);

    std::vector<float> meanInterClusterVar(sortCount,0.0F);
    std::vector<float> stdInterClusterVar(sortCount,0.0F);

    LINFO("DOING STATS");
    for(int i = 0; i < sortCount; i++)
    {
      int *thisClass = &sortClassMember[i];
      if(NP.NPgetClassSize(*thisClass) > NP.NPgetMinClassSize())
      {
        for(int j = 0; j < NP.NPgetClassSize(*thisClass); j++)
        {
          long item = NP.NPgetClass(*thisClass,j);
          for(int f = 0; f < featureCount; f++)
          {
            if(((int)features[f][0] == (int)NP.NPgetFeature(item,0)) &&
               ((int)features[f][1] == (int)NP.NPgetFeature(item,1)))
            {
              if((NP.NPgetClassSize(*thisClass) > 0) &&
                 (priorClassSize[priorItemMember[f]] > 0))
                {
                meanInterClassVar[priorItemMember[f]] += (float)i;
                stdInterClassVar[priorItemMember[f]] += pow((float)i,2);
                nInterClassVar[priorItemMember[f]]++;
                meanInterClusterVar[i] += (float)priorItemMember[f];
                stdInterClusterVar[i] += pow((float)
                                                      priorItemMember[f],2);
              }
            }
          }
        }
      }
    }
    float totalInterClassVar = 0.0F;
    float sumInterClassVar = 0.0F;
    float totalInterClusterVar = 0.0F;
    float sumInterClusterVar = 0.0F;
    for(int i = 0; i < priorClasses; i++)
    {
      if((nInterClassVar[i]-1) > 0)
      {
        meanInterClassVar[i] = meanInterClassVar[i]/nInterClassVar[i];
        stdInterClassVar[i] = (stdInterClassVar[i]/(nInterClassVar[i]))
          - pow(meanInterClassVar[i],2);
        totalInterClassVar += stdInterClassVar[i]*nInterClassVar[i];
        sumInterClassVar += nInterClassVar[i];
        LINFO("INTERCLASS MEAN for class %d is %f",i,meanInterClassVar[i]);
        LINFO("INTERCLASS VAR for class %d is %f",i,stdInterClassVar[i]);
        LINFO("N = %d",priorClassSize[i]);
      }
    }
    LINFO("STATS 1 done");
    float totalMeanInterClassVar;
    if(sumInterClassVar > 0)
      totalMeanInterClassVar = totalInterClassVar/sumInterClassVar;
    else
      totalMeanInterClassVar = 0;
    std::cout << "Total interclass variance is "
              << totalMeanInterClassVar << "\n";

    for(int i = 0; i < sortCount; i++)
    {
      int *thisClass = &sortClassMember[i];
      if((NP.NPgetClassSize(*thisClass)-1) > 0)
      {
        meanInterClusterVar[i] = meanInterClusterVar[i]
          /NP.NPgetClassSize(*thisClass);
        stdInterClusterVar[i] = (stdInterClusterVar[i]/
                                          (NP.NPgetClassSize(*thisClass)))
          - pow(meanInterClusterVar[i],2);
        totalInterClusterVar += stdInterClusterVar[i]
          *NP.NPgetClassSize(*thisClass);
        sumInterClusterVar += NP.NPgetClassSize(*thisClass);
        LINFO("INTERCLUSTER MEAN for class %d is %f",
              i,meanInterClusterVar[i]);
        LINFO("INTERCLUSTER VAR for class %d is %f",
              i,stdInterClusterVar[i]);
        LINFO("N = %d",NP.NPgetClassSize(*thisClass));
      }
    }
    float totalMeanInterClusterVar;
    if(sumInterClusterVar > 0)
      totalMeanInterClusterVar = totalInterClusterVar/
      sumInterClusterVar;
    else
      totalMeanInterClusterVar = 0;
    std::cout << "Total intercluster variance is "
              << totalMeanInterClusterVar << "\n";
    LINFO("STATS 2 done");
    //*******************************************************
    Raster::VisuRGB(classMap,
                    sformat("%s.CLASS_MAP.ppm",argv[1]));
    Raster::VisuRGB(priorClassMap,
                    sformat("%s.PRIOR_CLASS_MAP.ppm",argv[1]));
  }
  std::cerr << "a\n";
  uint64 t2 = tim.get();
  t0 = t2 - t0;
  if(NP.NP_doDensityMap == 1)
    Raster::VisuFloat(outputDensity,0,sformat("%s.outputDensity.pgm",argv[1]));
  std::cerr << "b\n";
  if(NP.NP_doLinkMap == 1)
    Raster::VisuRGB(outputLinks,
                    sformat("%s.outputLinks.ppm",argv[1]));
  LINFO("classify took %llums", t0);

  //----------------------------------------------------------------//

  //classify using Kmeans

  //----------------------------------------------------------------//


  NP.NPresetSpace(featureCount,2);
  NP.NPaddSpace(features);
  unsigned int K = 4;
  float minDiff = 0.001F;
  unsigned int stopIter = 5;
  NP.NPclassifySpaceKmeans(&K,&minDiff,&stopIter);
  LINFO("Fetching Bounding Boxes");
  BB = NP.NPgetBoundingBoxes(true);
  LINFO("Draw Bounding Box");
  pix.setRed(255),pix.setGreen(255);pix.setBlue(0);
  NP.NPdrawBoundingBoxes(&BB,&outputLinks,0,0,pix,true);
  if(NP.NP_doLinkMap == 1)
    Raster::VisuRGB(outputLinks,
                    sformat("%s.outputLinksK.ppm",argv[1]));
  // draw bounding boxes
  if(NP.NP_doClassMap == 1)
  {
    LINFO("Draw Class Images");
    Image<PixRGB<float> > classMap;
    PixRGB<float> mrPixel;
    classMap.resize(finput.getWidth(),finput.getHeight());
    findColorIndex FAC;

    pix.setRed(255),pix.setGreen(0);pix.setBlue(0);
    for(int i = 0; i < NP.NPgetStemNumber(); i++)
    {
      unsigned int number;
      number = (unsigned)i + 1;
      //if(NP.NPisStem(i) == true)
      //  number = 0;
      FAC.FACgetColor12(&number,&mrPixel);
      Image<PixRGB<float> > outputClass;
      outputClass.resize(finput.getWidth(),finput.getHeight());
      std::cerr << "class size " <<  NP.NPgetClassSize(i) << "\n";
      for(int j = 0; j < NP.NPgetClassSize(i); j++)
      {

        long item = NP.NPgetClass(i,j);
        outputClass.setVal((int)NP.NPgetFeature(item,0)
                           ,(int)NP.NPgetFeature(item,1),pix);
        classMap.setVal((int)NP.NPgetFeature(item,0)
                           ,(int)NP.NPgetFeature(item,1),mrPixel);
        if(j == 0)
        {
            char foo;
            sprintf(&foo,"%d",(int)item);
            writeText(classMap, Point2D<int>((int)NP.NPgetFeature(item,0),
                                        (int)NP.NPgetFeature(item,1)),
                      &foo,PixRGB<float>(255),PixRGB<float>(0));
        }
      }
      //Raster::VisuRGB(outputClass,sformat("CLASS_%d.ppm",i));
    }

    //**********************************************************
    // draw cov matrix
    Image<float> final;
    final.resize(finput.getWidth(),finput.getWidth());
    for(int i = 0; i < NP.NPgetStemNumber(); i++)
    {
      int classSize = NP.NPgetClassSize(i);
      if(NP.NPgetMinClassSize() <= classSize)
      {
        float t = 0.0F;
        float* tfloat = &t;
        std::vector<float> _vinput(classSize,0);
        std::vector<float*> _vTinput(classSize,tfloat);
        std::vector<std::vector<float> > NPclass(2,_vinput);
        std::vector<std::vector<float*> > NPclassP(2,_vTinput);
        for(int cs = 0; cs < classSize; cs++)
        {
          long item = NP.NPgetClass(i,cs);
          NPclass[0][cs] = finput.getWidth() - NP.NPgetFeature(item,0);
          std::cerr << "INPUTING " << NPclass[0][cs] << "\n";
          NPclass[1][cs] = NP.NPgetFeature(item,1);
          std::cerr << "INPUTING " << NPclass[1][cs] << "\n";
          NPclassP[0][cs] = &NPclass[0][cs];
          NPclassP[1][cs] = &NPclass[1][cs];
        }
        covHolder<float> covh;
        covh.resize(2,classSize,0.0F);
        covEstimate<float> CE(NPclassP,covh);
        CE.run();
        final = CE.returnCovSlice(0,1,final);
      }
    }
    Raster::VisuFloat(final,0,sformat("%s.outputCovK.pgm",argv[1]));

    //*******************************************************
    std::vector<float> meanInterClassVar(priorClasses,0.0F);
    std::vector<float> stdInterClassVar(priorClasses,0.0F);
    std::vector<int> nInterClassVar(priorClasses,0);

    std::vector<float> meanInterClusterVar(K,0.0F);
    std::vector<float> stdInterClusterVar(K,0.0F);


    for(int i = 0; i < (signed)K; i++)
    {
      for(int j = 0; j < NP.NPgetClassSize(i); j++)
      {
        long item = NP.NPgetClass(i,j);
        for(int f = 0; f < featureCount; f++)
        {
          if(((int)features[f][0] == (int)NP.NPgetFeature(item,0)) &&
             ((int)features[f][1] == (int)NP.NPgetFeature(item,1)))
          {
            if((NP.NPgetClassSize(i) > 0) &&
               (priorClassSize[priorItemMember[f]] > 0))
            {
              meanInterClassVar[priorItemMember[f]] += (float)i;
              stdInterClassVar[priorItemMember[f]] += pow((float)i,2);
              nInterClassVar[priorItemMember[f]]++;
              meanInterClusterVar[i] += (float)priorItemMember[f];
              stdInterClusterVar[i] += pow((float)
                                           priorItemMember[f],2);
            }
          }
        }
      }
    }
    float totalInterClassVar = 0.0F;
    float sumInterClassVar = 0.0F;
    float totalInterClusterVar = 0.0F;
    float sumInterClusterVar = 0.0F;
    for(int i = 0; i < priorClasses; i++)
    {
      if((nInterClassVar[i]-1) > 0)
      {
        meanInterClassVar[i] = meanInterClassVar[i]/nInterClassVar[i];
        stdInterClassVar[i] = (stdInterClassVar[i]/(nInterClassVar[i]))
          - pow(meanInterClassVar[i],2);
        totalInterClassVar += stdInterClassVar[i]*nInterClassVar[i];
        sumInterClassVar += nInterClassVar[i];
        LINFO("K INTERCLASS MEAN for class %d is %f",i,meanInterClassVar[i]);
        LINFO("K INTERCLASS VAR for class %d is %f",i,stdInterClassVar[i]);
        LINFO("K N = %d",priorClassSize[i]);
      }
    }
    float totalMeanInterClassVar = totalInterClassVar/sumInterClassVar;
    std::cout << "K Total interclass variance is "
              << totalMeanInterClassVar << "\n";

    for(int i = 0; i < (signed)K; i++)
    {
      if((NP.NPgetClassSize(i)-1) > 0)
      {
        meanInterClusterVar[i] = meanInterClusterVar[i]
          /NP.NPgetClassSize(i);
        stdInterClusterVar[i] = (stdInterClusterVar[i]/
                                          (NP.NPgetClassSize(i)))
          - pow(meanInterClusterVar[i],2);
        totalInterClusterVar += stdInterClusterVar[i]
          *NP.NPgetClassSize(i);
        sumInterClusterVar += NP.NPgetClassSize(i);
        LINFO("K INTERCLUSTER MEAN for class %d is %f",
              i,meanInterClusterVar[i]);
        LINFO("K INTERCLUSTER VAR for class %d is %f",
              i,stdInterClusterVar[i]);
        LINFO("K N = %d",NP.NPgetClassSize(i));
      }
    }
    float totalMeanInterClusterVar = totalInterClusterVar/
      sumInterClusterVar;
    std::cout << "K Total intercluster variance is "
              << totalMeanInterClusterVar << "\n";
    Raster::VisuRGB(classMap,
                    sformat("%s.CLASS_MAP_K.ppm",argv[1]));
  }
}





// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
