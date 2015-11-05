/*!@file Learn/test-QuadTree.C QuadTree Multi-Class Classifier */
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

#include "Component/ModelComponent.H"
#include "Component/ModelManager.H"
#include "Image/ColorMap.H" 
#include "Image/ColorOps.H" // for colorize() 
#include "Image/CutPaste.H"
#include "Image/Dims.H"
#include "Image/Image.H"
#include "Image/ShapeOps.H"
#include "Learn/QuadTree.H"
#include "Channels/RawVisualCortex.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"

#include "GUI/DebugWin.H"

std::vector<byte> perm3Number(int key)
{
  key = key % 6;
  std::vector<byte> ret;
  int rem = key % 3;
  ret.push_back(rem);
  if(key > 3) {
    ret.push_back((rem + 1) % 3); 
    ret.push_back((rem + 2) % 3); 
  } else {
    ret.push_back((rem + 2) % 3); 
    ret.push_back((rem + 1) % 3); 
  }
  return ret;
}

QuadNode::NodeState hashNodeState(int key) {
  key = key % (30 * 6);
  return QuadNode::NodeState(key % 30, perm3Number(key / 30));
}

int submain(const int argc, char** argv)
{
  MYLOGVERB = LOG_INFO;

  ModelManager manager("Test Quad Tree");
  
  nub::ref<RawVisualCortex> vc(new RawVisualCortex(manager));
  manager.addSubComponent(vc);
  //  vc->setModelParamVal("LevelSpec",LevelSpec(2,4,3,4,4));
  
  
  uint nArgs = 2;
    if (manager.parseCommandLine((const int)argc, (const char**)argv, "<image filename>", nArgs, nArgs) == false)
    return 1;
    vc->setModelParamVal("RawVisualCortexChans",std::string("CIO"));
    vc->setModelParamVal("LevelSpec",LevelSpec(0,1,2,3,0),MC_RECURSE); // bug in resetting pyramids, need to check this 
        //vc->setModelParamVal("LevelSpec",LevelSpec(2,4,3,4,4));

  // fix image size
    Dims dims(320,240);

  // import image
  const GenericFrame input = Raster::ReadFrame(manager.getExtraArg(0).c_str());
  Image<PixRGB<byte> > im = rescale(input.asRgb(), dims);
  Image<uint> cl_im(dims, ZEROS);
  // test GistPixelClassifier
 
  //  GistPixelClassifier gpc;
  //gpc.learnInput(im, cl_im);

  // create Quad Tree for a given image size
  QuadTree myQuadTree(1,im);

  // // create tester Classifier based on color alone
  // rutz::shared_ptr<ColorPixelClassifier> cpc(new ColorPixelClassifier);
  // cpc->addCategory(ColorPixelClassifier::ColorCat
  //                  (PixRGB<byte>(140,140,100), 25)); //grass
  // cpc->addCategory(ColorPixelClassifier::ColorCat
  //                  (PixRGB<byte>(20,200,20), 25));
  // cpc->addCategory(ColorPixelClassifier::ColorCat
  //                  (PixRGB<byte>(20,20,200), 25));
  // cpc->addCategory(ColorPixelClassifier::ColorCat
  //                  (PixRGB<byte>(32,32,32), 25)); //brown
  // cpc->addCategory(ColorPixelClassifier::ColorCat
  //                  (PixRGB<byte>(255,0,0), 25)); //red
  // cpc->addCategory(ColorPixelClassifier::ColorCat
  //                  (PixRGB<byte>(0,255,0), 25)); //green
  // cpc->addCategory(ColorPixelClassifier::ColorCat
  //                  (PixRGB<byte>(0,0,255), 25)); //blue
  // myQuadTree.setClassifier(cpc);

  manager.start();

  // input
  vc->input(InputFrame::fromRgb(&im));

  for(uint i = 0; i < vc->numSubmaps(); i++) {
    Image<float> im_fl = vc->getSubmap(i); 
  // output
    inplaceNormalize(im_fl,0.0f,255.0f);
    LINFO("%s", toStr(vc->getSubmapName(i)).c_str());
    Image<PixRGB<byte> > im_out = im_fl;  
  }

  rutz::shared_ptr<QuadNode> root = myQuadTree.getRootNode();
  LINFO("caching classifier result");
  myQuadTree.cacheClassifierResult();
  LINFO("generating proposals");
  std::vector<QuadNode::NodeState> pr = myQuadTree.generateProposalsAt(root, manager.getExtraArgAs<double>(1));
  LINFO("done; %zu proposals made", pr.size());
   
  uint i,i_max = 0;
  for(i = 0; i < pr.size(); i++) {
    LINFO("proposal %d: %s -> %f", i, toStr(pr[i]).c_str(),pr[i].E);
    if(pr[i].E < pr[i_max].E) i_max = i;
  }

  if(pr.size() > 0) {
    LINFO("proposal with least energy %f: %s", pr[i_max].E, toStr(pr[i_max]).c_str());
    root->setState(pr[i_max]);
  }
  else
    {
      LINFO("no proposals match!"); return 0;
    }

  // image reporting
  Image<PixRGB<byte> > im_L0, im_L1, im_tot;
  im_L0 = root->getColorizedSegImage();
  im_L1 = root->getColorizedChildSegImage();
  
  im_tot = concatX(im, concatX(im_L0, im_L1));
  drawLine(im_tot, Point2D<int>(dims.w(), 0), Point2D<int>(dims.w(),dims.h()-1),PixRGB<byte>(255,255,255));
  drawLine(im_tot, Point2D<int>(2*dims.w(), 0), Point2D<int>(2*dims.w(),dims.h()-1),PixRGB<byte>(255,255,255));

  // output
  Raster::WriteRGB(im_tot,"result.png");

  SHOWIMG(im_tot);  
  //myQuadTree.printTree();
    //    Raster::WriteRGB(root->getColorizedSegImage(),
    //		      sformat("seg%02d.png",i));

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
