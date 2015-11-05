/*!@file Learn/test-gbcOnMSRC.C QuadTree Multi-Class Classifier */
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
// Primary maintainer for this file: John Shen <shenjohn@usc.edu>
// $HeadURL$
// $Id$
//

#include "Channels/RawVisualCortex.H"
#include "Component/ModelComponent.H"
#include "Component/ModelManager.H"
#include "GUI/DebugWin.H"
#include "Image/Dims.H"
#include "Image/Image.H"
#include "Image/ShapeOps.H"
#include "Learn/GentleBoostComponent.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Util/FileUtil.H"
#include "Util/Timer.H"
#include <iomanip>

std::vector<std::string> readDir(std::string inName)
{
  DIR *dp = opendir(inName.c_str());
  if(dp == NULL)
  {
    LFATAL("Directory does not exist %s",inName.c_str());
  }
  dirent *dirp;
  std::vector<std::string> fList;
  while ((dirp = readdir(dp)) != NULL ) {
    if (dirp->d_name[0] != '.')
      fList.push_back(inName + '/' + std::string(dirp->d_name));
  }
  // LINFO("%" ZU " files in the directory\n", fList.size());
  // LINFO("file list : \n");
  // for (unsigned int i=0; i<fList.size(); i++)
  //         LINFO("\t%s", fList[i].c_str());
  std::sort(fList.begin(),fList.end());
  return fList;
}

int submain(const int argc, char** argv)
{
  MYLOGVERB = LOG_INFO;

  ModelManager manager("Test SVM on MSRC database");
  
  nub::ref<RawVisualCortex> vc(new RawVisualCortex(manager));
  manager.addSubComponent(vc);

  nub::ref<GentleBoostComponent> gbc(new GentleBoostComponent(manager));
   manager.addSubComponent(gbc);

  // manager.exportOptions(MC_RECURSE);
  
  // step 1: extract features
  if (manager.parseCommandLine((const int)argc, (const char**)argv, "<image directory> <ground truth directory>", 2, 2) == false)
    return 1;
  
  std::string dir_in = manager.getExtraArg(0).c_str();
  std::string dir_GT = manager.getExtraArg(1).c_str();
  std::vector<std::string> files_in = readDir(dir_in.c_str());
  //  std::vector<std::string> files_GT = readDir(dir_GT.c_str());

  vc->setModelParamVal("RawVisualCortexChans",std::string("IQCOETL"));
  vc->setModelParamVal("LevelSpec",LevelSpec(0,0,3,3,0),MC_RECURSE); // bug in resetting pyramids, need to check this 
  //vc->setModelParamVal("LevelSpec",LevelSpec(2,4,3,4,4));

  // fix image size
  const Dims dims(320,240);
  const Dims dims_portrait(240,320);
  
  manager.start();
  
  // defs
  const uint Nfiles = files_in.size();
  const uint Nmaps = vc->numSubmaps();

  // Image<double> traindata(Dims(Nfiles*dims.sz(),Nmaps),ZEROS);
  std::vector<double> labels(Nfiles*dims.sz());
  Timer t(1000000);
  //  int id_ctr = 0;
  for(uint i = 0; i < Nfiles; i++) {   
    // get the ground truth file name 
    std::string fil = files_in[i].c_str();    
    std::string fil_nopath;
    splitPath(fil, dir_in, fil_nopath);
    
    std::string fil_GT = fil_nopath;
    fil_GT.insert(fil_GT.length()-4, "_GT");
    fil_GT = dir_GT + fil_GT;
    
    // import the images
    const GenericFrame input = Raster::ReadFrame(fil.c_str());
    const GenericFrame input_GT = Raster::ReadFrame(fil_GT.c_str()); 
    
    Image<PixRGB<byte> > im(dims,NO_INIT);
    Image<byte> im_GT(dims, NO_INIT);
    
    Dims currdims;
    //convert to images and resize with correct orientation
    if(im.getWidth() > im.getHeight()) 
      currdims = dims;
    else 
      currdims = dims_portrait;
    im = rescale(input.asRgb(), currdims);
    im_GT = rescale(input_GT.asGray(), currdims);
    
    // provide input to visual cortex
    vc->input(InputFrame::fromRgb(&im));

    Point2D<int> P(0,0);

    t.reset();
    std::vector<float> features;
    std::vector<Image<float> > feat_maps;
    for(uint j = 0; j < Nmaps; j++) feat_maps.push_back(vc->getSubmap(j));
    
    for(P.i = 0; P.i < currdims.w(); P.i++)
      for(P.j = 0; P.j < currdims.h(); P.j++) {
        for(uint j = 0; j < Nmaps; j++) 
          features.push_back(feat_maps[j][P]);
        
        gbc->addTrainVector(features,im_GT[P],0);
        // LINFO("memory allocation of features: %zu",sizeof(features));
        features.clear();
      }
    feat_maps.clear();
    t.pause();
    LINFO("redone with image %d/%d, %s elapsed",i+1,Nfiles,
           toStr(t.getSimTime()).c_str());
    // LINFO("memory allocation of gbc: %zu", sizeof(gbc));
  }
    
  LINFO("training gbc");
  gbc->train(0);
  LINFO("saving gbc");
  gbc->save(0);
  //   // double * prev_ptr = traindata.beginw();
  //   LINFO("running through %u features for file %s", vc->numSubmaps(), fil.c_str());
  //   LINFO("training data array is dims %s, size %d", 
  //         toStr(traindata.getDims()).c_str(),traindata.size());
  //   for(uint i = 0; i < Nmaps; i++) {
      
  //     // naming the files
  //     // char * fil_feature = new char[100];
  //     //      sprintf(fil_feature,"/lab/jshen/Pictures/MSRC/Features/%s-map%02d",fil_nopath.c_str(),i);

  //     LINFO("processing feature %s", toStr(vc->getSubmapName(i)).c_str());
  //     // push out the raw features  
  //     //  Image<double> im_feat = vc->getSubmap(i); 

  //     // normalize the features (could be slow)
  //     inplaceNormalize(im_feat,0.0,1.0);

      
  //     // add entries to table
  //     // double * data_ptr = traindata.beginw() +
  //     //   dims.sz() * sizeof(double) * h + // move to the correct subcolumn
  //     //   traindata.getWidth() * sizeof(double) * i; // move to the correct row
  //     // LINFO("range is %p - %p (len %zu), pointer @ %p (jump %zu)", traindata.begin(), traindata.end(), (size_t)(traindata.end()-traindata.begin()) ,data_ptr, (size_t)(data_ptr-prev_ptr));
  //     // prev_ptr = data_ptr;
     

  //     // memcpy(data_ptr, im_feat.begin(), sizeof(double)*im_feat.size());
      
  //     // LINFO("copied to memory");
  //     //Raster::WriteGray(im_out,fil_out,RASFMT_PNG);
  //     //      LINFO("saved file %s for map %s", fil_out, vc->getSubmapName(i).c_str());   
  //   }

  //   LINFO("writing the GT data");
  //   double * label_ptr = &labels[0] + h * sizeof(double) * im_GT.size();
  //   memcpy(label_ptr, im_GT.begin(),sizeof(double)*im_GT.size());
  // }
  // SVMClassifier svm;
  // LINFO("starting training");
  // svm.train(traindata,labels);
  // LINFO("training complete");
  manager.stop();
  return 0;
}

extern "C" int main(const int argc, char** argv)
{
  try
    {
      return submain(argc, argv);
    }
  catch (...)
    {
      REPORT_CURRENT_EXCEPTION;
    }

  return 1;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
