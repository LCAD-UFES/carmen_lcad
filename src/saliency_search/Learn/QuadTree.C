/*!@file Learn/QuadTree.C QuadTree Multi-Class Classifier */
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
// Implementation of the segmentation algorithm described in:
//
// Recursive Segmentation and Recognition Templates for 2D Parsing
// Leo Zhu, Yuanhao Chen, Yuan Lin, Chenxi Lin, Alan Yuille
// Advances in Neural Information Processing Systems, 2008
// 

#include "Channels/IntensityChannel.H"
#include "Channels/InputFrame.H"
#include "Channels/ChannelOpts.H"
#include "Component/ModelManager.H"
#include "GUI/DebugWin.H"
#include "Image/ColorMap.H" // for colorize()
#include "Image/ColorOps.H" // for colorize()
#include "Image/CutPaste.H" // for inplaceEmbed()
#include "Image/Dims.H"
#include "Image/Image.H"
#include "Image/LevelSpec.H"
#include "Image/MathOps.H" // for absDiff()
#include "Image/Pixels.H"
#include "Image/Point3D.H"
#include "Learn/QuadTree.H"
#include "Util/log.H"
#include "Util/StringConversions.H"

#include <cmath>
#include <vector>
#include <queue> // for DP queue
#include <iostream>
#include <algorithm> // for std::swap 

// ######################################################################
QuadTree::QuadTree(int Nlevels, Dims d) : itsNumLevels(Nlevels)
{
  // initialize tree
  rutz::shared_ptr<QuadNode> root_ref(new QuadNode()); //this is the temporary top, no ptrs initialized
  Rectangle thisWindow(Point2D<int>(0,0), d);
  addTreeUnder(root_ref, Nlevels, thisWindow);
  
  itsRootNode = root_ref->getChild(0);
  initAlphas();
}

// ######################################################################
QuadTree::QuadTree(int Nlevels, Image<PixRGB<byte> > im) : itsNumLevels(Nlevels)
{
  // QuadTree::QuadTree(Nlevels, im.getDims());
  Dims d = im.getDims();

  // initialize tree
  rutz::shared_ptr<QuadNode> root_ref(new QuadNode()); //this is the temporary top, no ptrs initialized
  Rectangle thisWindow(Point2D<int>(0,0), d);
  addTreeUnder(root_ref, Nlevels, thisWindow);
  
  itsRootNode = root_ref->getChild(0);

  itsImage = im;
  initAlphas();
}

// ######################################################################
void QuadTree::addTreeUnder(rutz::shared_ptr<QuadNode> parent, int Nlevel, Rectangle r)
{
  // initialize node
  rutz::shared_ptr<QuadNode> myNewNode(new QuadNode(parent));
  myNewNode->setArea(r);
  myNewNode->setDepth(itsNumLevels-Nlevel);

  // add to tree
  parent->addChild(myNewNode);

  // add to internal deque
  itsNodes.push_back(myNewNode);

  // if there are levels below, add sub-trees to this node
  if (Nlevel > 0) {
    // find 4 smaller rectangles
    Point2D<int> middle = r.center();
    
    Rectangle tl = Rectangle::tlbrO(r.top(),r.left(),middle.j,middle.i);
    Rectangle tr = Rectangle::tlbrO(r.top(),middle.i,middle.j,r.rightO());
    Rectangle bl = Rectangle::tlbrO(middle.j,r.left(),r.bottomO(),middle.i);
    Rectangle br = Rectangle::tlbrO(middle.j,middle.i,r.bottomO(),r.rightO());

    addTreeUnder(myNewNode, Nlevel - 1, tl);
    addTreeUnder(myNewNode, Nlevel - 1, tr);
    addTreeUnder(myNewNode, Nlevel - 1, bl);
    addTreeUnder(myNewNode, Nlevel - 1, br);
  }
}

// ######################################################################
void QuadTree::cacheClassifierResult()
{
  uint NClasses = itsClassifier->getNumClasses();
  Image<double> output(itsImage.getDims(),ZEROS);
  Image<double> denom(itsImage.getDims(),ZEROS);
  Dims patch_size(5,5);
  Image<PixRGB<byte> > patch(patch_size,ZEROS);
  Rectangle im_rect(Point2D<int>(0,0), itsImage.getDims());

  itsClassifierOutput.clear();
  itsBestClassOutput.resize(itsImage.getDims());
  for(uint i = 0; i < NClasses; i++) 
    itsClassifierOutput.push_back(output);

  Point2D<int> P_l; // local coords
  for(P_l.i = 0; P_l.i < itsImage.getWidth(); P_l.i++) 
    for(P_l.j = 0; P_l.j < itsImage.getHeight(); P_l.j++) {
      
      Rectangle rect_patch = Rectangle::centerDims(P_l, patch_size);
      rect_patch = constrainRect(rect_patch, im_rect,0,itsImage.getWidth(),0,itsImage.getHeight());
      patch = crop(itsImage, rect_patch);
      
      byte best_class = 0;
      for(uint i = 0; i < NClasses; i++) {
        double res = itsClassifier->classifyAt(patch, i);
        if(res > itsClassifierOutput[best_class][P_l]) 
          best_class = i;
        
        itsClassifierOutput[i][P_l] = res;
        // get partition denominator
        denom[P_l] += exp(res);
      }    
      itsBestClassOutput[P_l] = best_class;
    }
  
  for(uint i = 0; i < NClasses; i++) 
    itsClassifierOutput[i] = exp(itsClassifierOutput[i])/denom; 
}

// ######################################################################
double QuadTree::evaluateClassifierAt(rutz::shared_ptr<QuadNode> q) const
{
  int Npts = itsImage.getDims().sz();
  double E = 0;
  Point2D<int> P_l, P_g; // local coords
  for(P_l.i = 0; P_l.i < itsImage.getWidth(); P_l.i++) 
    for(P_l.j = 0; P_l.j < itsImage.getHeight(); P_l.j++) {
      P_g = q->convertToGlobal(P_l);
      E -= log(itsClassifierOutput[q->getObjLabelAt(P_g)][P_l]);
    }
  
  return E/Npts;
}

// ######################################################################
double QuadTree::evaluateCohesionAt(rutz::shared_ptr<QuadNode> q) const
{
  // a negative energy term where pixels that belong to the same partitions have similar appearance
  // loop over all pairs of points in the image at that quadnode
  Neighborhood one_away;
  //  one_away.push_back(Point2D<int>(-1,-1));
  //one_away.push_back(Point2D<int>(-1,0));
  //one_away.push_back(Point2D<int>(-1,1));
  //one_away.push_back(Point2D<int>(0,-1));
  const float lambda = 1; // term weighting the importance of "cohesion" relative to the other energy terms, might depend on scale
  const float sigma_col = 75; // the gaussian st-dev in color space
  const Point2D<int> origin(0,0);
  float e_coh = 0;

  one_away.push_back(Point2D<int>(0,1));
  one_away.push_back(Point2D<int>(1,-1));
  one_away.push_back(Point2D<int>(1,0));
  one_away.push_back(Point2D<int>(1,1));


  Image<byte> segImage = q->getSegImage();

  int nedges = 0;
  //  int n_same = 0;
  //int N_bad = 0;

  Point2D<int> P_l,Q_l, P_g, Q_g; // local and global coords
  for(P_l.i = 0; P_l.i < segImage.getWidth(); P_l.i++) {
    for(P_l.j = 0; P_l.j < segImage.getHeight(); P_l.j++) {
      for(uint k = 0; k < one_away.size(); k++) {
	// calculate the points
	Q_l = P_l + one_away[k];
	P_g = q->convertToGlobal(P_l);
	Q_g = q->convertToGlobal(Q_l);

	if (!segImage.coordsOk(Q_l)) continue; // if Q is out of bounds, skip
	
	nedges++; // count the edge
	//if P and Q are not in the same segment, skip
	if(segImage[P_l] != segImage[Q_l]) continue; 
	
	// the computation of the energy term	
	double color_dist = colorDistance(itsImage[P_g], itsImage[Q_g]);
	double space_dist = one_away[k].distance(origin);
	e_coh -= lambda / space_dist * exp(- color_dist * color_dist / (2 * sigma_col * sigma_col));
      }
    }
  }
  return e_coh / nedges;
}

// ######################################################################
double QuadTree::evaluateCorrespondenceAt(rutz::shared_ptr<QuadNode> q) const
{
  if(q->isLeaf()) return 0;

  Point2D<int> P_l,Q_l, P_g, Q_g; // local and global coords
  Image<byte> parentImage = q->getSegImage();
  Image<byte> childImage = q->getChildSegImage();
  
  Image<byte> deltaImage = absDiff(parentImage, childImage);
  double ret = double(-emptyArea(deltaImage))/deltaImage.getDims().sz();
	
  return ret;
}

// ######################################################################
double QuadTree::evaluateTotalEnergyAt(rutz::shared_ptr<QuadNode> q) const
{
  double E = evaluateClassifierAt(q) * itsAlphas[0];
  E += evaluateCohesionAt(q) * itsAlphas[1];
  E += evaluateCorrespondenceAt(q) * itsAlphas[2];

  if(q->isLeaf()) return E;
  for(uint i = 0; i < 4; i++)
    E += evaluateTotalEnergyAt(q->getChild(i));

  return E;
}

// ######################################################################
void QuadTree::printTree() const 
{
  LINFO("Tree of depth %u, number of nodes %zu", itsNumLevels, itsNodes.size());
  // NB: not a traversal for now, just reading off the queue
  for(uint i = 0; i < itsNodes.size(); i++) 
    LINFO("%s", toStr(*itsNodes[i]).c_str());
  
}

// ######################################################################
std::string QuadTree::writeTree() const 
{
  std::string ret = "";
  ret += sformat("Tree of depth %u, number of nodes %zu\n", itsNumLevels, itsNodes.size());
  // NB: not a traversal for now, just reading off the queue
  for(uint i = 0; i < itsNodes.size(); i++) 
    ret += sformat("%s\n", toStr(*itsNodes[i]).c_str());
  
  return ret;
}

// ######################################################################
std::vector<QuadNode::NodeState> QuadTree::generateProposalsAt(rutz::shared_ptr<QuadNode> q, double thresh)
{
  // this code is just for the leaf nodes right now
  uint NClasses = itsClassifier->getNumClasses();

  if(!q->isLeaf()) { //combine proposals
    for(uint i = 0; i < 4; i++) { // children loop
      std::vector<QuadNode::NodeState> child_props = generateProposalsAt(q->getChild(i), thresh);
      if(child_props.size() == 0) LFATAL("no proposals made for node %s", toStr(q->getChild(i)).c_str());
      q->getChild(i)->setState(child_props[0]);
    }
  }
  
  QuadNode::NodeState probe(0,0,1,2), realstate = q->getState();
  std::vector<QuadNode::NodeState> ret;
  
  // try fitting each template first
  //  uint common[3][NClasses];

  byte top_class[3][NClasses];  
  uint prevalence[3][NClasses];
  Rectangle r = q->getArea();
  
  if(1) {
    for(; probe.segTemplate < 30; probe.segTemplate++) {

      //clear prevalence
      for(uint i = 0; i < 3; i++) 
        for(uint j= 0; j < NClasses; j++) {
          prevalence[i][j]=0;         
          top_class[i][j]=j;
        }
      
      // find the frequency of each label for each region in the classifier
      for(uint i = 0; i < 3; i++) probe.objLabels[i] = i;
      
      q->setState(probe); //initialize just for counting purposes
      
      Point2D<int> P_g;
      for(P_g.i = r.left(); P_g.i < r.rightO(); P_g.i++) 
        for(P_g.j = r.top(); P_g.j < r.bottomO(); P_g.j++) 
          prevalence[q->getObjLabelAt(P_g)][itsBestClassOutput[P_g]]++;  
      
      // sort each entry by the class prevalence 
      for(uint i = 0; i < 3; i ++)  {
        for(uint j = 0; j < NClasses; j++) {
          for(uint k = 0; k < NClasses-j-1; k++)
            if(prevalence[i][k] < prevalence[i][k+1])
              {
                std::swap(top_class[i][k], top_class[i][k+1]);
                std::swap(prevalence[i][k], prevalence[i][k+1]);
              }      
        } 
      }    
      
      // finding proposals - DP setup
      std::queue<Point3D<uint> > tryme;    
      bool tested[NClasses][NClasses][NClasses];
      for(uint i = 0; i < NClasses; i++) 
        for(uint j = 0; j < NClasses; j++) 
          for(uint k = 0; k < NClasses; k++) 
            tested[i][j][k] = false;
      
      uint area = r.area();
      const double occ_tol = 0.0001;
      for(uint i = 0; i < NClasses; i++) {
        if(prevalence[0][0] - prevalence[0][i] > occ_tol * area) break;
        for(uint j = 0; j < NClasses; j++) {
          if(prevalence[1][0] - prevalence[1][j] > occ_tol * area) break; 
          for(uint k = 0; k < NClasses; k++) {
            if(prevalence[2][0] - prevalence[2][k] > occ_tol * area) break;
            tryme.push(Point3D<uint>(i,j,k));
            if(probe.isDoubleton()) break; //the last label doesn't matter
          }
          if(probe.isSingleton()) break; // the 2nd to last label doesn't matter
        }
      }
      while(!tryme.empty()) {
        Point3D<uint> n = tryme.front();
        tryme.pop();
        
        if(tested[n.x][n.y][n.z]) continue;
        if(n.x >= NClasses || n.y >= NClasses || n.z >= NClasses) continue;
        probe.objLabels[0] = top_class[0][n.x];
        probe.objLabels[1] = top_class[1][n.y];
        probe.objLabels[2] = top_class[2][n.z];
        q->setState(probe);
        q->storeEnergy(evaluateTotalEnergyAt(q));
        //      probe.evaled = true;
        tested[n.x][n.y][n.z]=true;
        
        if(q->getEnergy() < thresh) {
          probe.E = q->getEnergy();
          ret.push_back(probe);
          tryme.push(Point3D<uint>(n.x+1,n.y,n.z));
          if(!probe.isSingleton()) tryme.push(Point3D<uint>(n.x,n.y+1,n.z));
          if(!probe.isDoubleton()) tryme.push(Point3D<uint>(n.x,n.y,n.z+1));
        }
      }
    } //end seg template loop    

    for(uint j = 0; j < ret.size(); j++) 
      for(uint k = 0; k < ret.size()-j-1; k++)
        if(ret[k].E > ret[k+1].E)
          std::swap(ret[k],ret[k+1]); 

    q->setState(realstate);
  }
  return ret;
}

// ######################################################################
QuadNode::QuadNode() : itsIsStale(true), itsState(0)
{
  for(uint i = 0; i < 3; i++) itsState.objLabels.push_back(i);
}

// ######################################################################
QuadNode::QuadNode(rutz::shared_ptr<QuadNode> q) 
  : itsIsLeaf(true), itsIsStale(true),
    itsState(0), 
    itsParent(q)
{
  for(uint i = 0; i < 3; i++) itsState.objLabels.push_back(i);
} 

// ######################################################################
QuadNode::QuadNode(rutz::shared_ptr<QuadNode> q, NodeState n) 
  : itsIsLeaf(true), itsIsStale(true),
    itsState(n), 
    itsParent(q)
{
} 

// ######################################################################
Image<byte> QuadNode::getChildSegImage() 
{
  if(isLeaf()) return getSegImage();
  Image<byte> ret(getArea().dims(),ZEROS);
  for(uint i = 0; i < 4; i++) {
    rutz::shared_ptr<QuadNode> child = getChild(i);
    inplaceEmbed(ret, child->getSegImage(), child->getArea(),byte(-1));
  }
  return ret;
}

// ######################################################################
Image<PixRGB<byte> > QuadNode::getColorizedSegImage()
{
  return colorLabels(getSegImage());
}

// ######################################################################
Image<PixRGB<byte> > QuadNode::getColorizedChildSegImage() 
{
  return colorLabels(getChildSegImage());
}

// ######################################################################
void QuadNode::refreshSegImage() 
{
  ASSERT(itsState.objLabels.size() > 0);

  Image<byte> ret(itsArea.dims(),NO_INIT);
  Point2D<int> P_local;
  for(P_local.i = 0; P_local.i < itsArea.dims().w(); P_local.i++) 
    for (P_local.j = 0; P_local.j < itsArea.dims().h(); P_local.j++) 
      ret[P_local] = getObjLabelAt(convertToGlobal(P_local));
      
  itsSegImage = ret;
  itsIsStale = false;
}

// ######################################################################
Image<PixRGB<byte> > QuadNode::colorLabels(Image<byte> im) const
{
  ColorMap cm(256);
  PixRGB<byte> col;
  for(uint i = 2; i <= 2; i--) { // NB: once we get a classifier, we will need a real colormap instead of an adhoc one, this 
    col = PixRGB<byte>(0,0,0);
    col[i] = 255;
    cm[i] = col;
  }
  return colorize(im, cm);
}

// ######################################################################
byte QuadNode::getObjLabelAt(Point2D<int> loc) const
{
  // TODO: move this logic to drawing the template all at once 

  // check if point resides in the area
  if(!itsArea.contains(loc)) {
    LINFO("Node at window (%s) does not contain point (%s)", 
	  toStr(itsArea).c_str(), toStr(loc).c_str());
    return -1;
  }

  // result is already memoized
  if(!itsIsStale) { 
    return itsSegImage[convertToLocal(loc)];
  }

  //convert point to [0,1] x [0,1] (scaled) coordinates;
  Point2D<double> intLoc(double(loc.i-itsArea.left())/(itsArea.width()),
			 double(loc.j-itsArea.top())/(itsArea.height()));
  
  uint iST = itsState.segTemplate;
  byte lvl = 0, lvl1, lvl2;
  double keydim1 = 0, keydim2 = 0;

  if (iST == 0) 
    lvl = 0;
  else if(iST == 1 || iST == 2) {
    // horizontal/vertical edges
    if(iST == 1) keydim1 = intLoc.i;
    else keydim1 = intLoc.j;
    lvl = keydim1 * 3;
  }
  else if(iST == 3 || iST == 4) {
    //diagonal edges
    if(iST == 3) keydim1 = intLoc.i+intLoc.j-1;
    else keydim1 = intLoc.j-intLoc.i;
    lvl = (keydim1 < 0) ? 0 : 1;
  }
  else if(iST == 5) {
    //box inside another box
    keydim1 = fabs(intLoc.i - 0.5); 
    keydim2 = fabs(intLoc.j - 0.5);
    lvl1 = keydim1 < 0.25 ? 1 : 0;
    lvl2 = keydim2 < 0.25 ? 1 : 0;
    lvl = lvl1 * lvl2;
  }
  else if (iST >= 6 && iST <= 9) {
    //V-junctions
    switch(iST) {
    case 6:
      keydim1 = intLoc.i;
      keydim2 = intLoc.j;
      break;
    case 7:
      keydim1 = 1-intLoc.i;
      keydim2 = intLoc.j;
      break;
    case 8:
      keydim1 = intLoc.j;
      keydim2 = intLoc.i;
      break;
    case 9:
      keydim1 = 1-intLoc.j;
      keydim2 = intLoc.i;
      break;
    }
    lvl1 = (2*keydim2 - keydim1) < 0 ? 0 : 1;
    lvl2 = (2*keydim2 - 2 + keydim1) < 0 ? 0 : 1;
    lvl = lvl1 + lvl2;
  }
  else if (iST >= 10 && iST <= 13) {
    //diagonal orientations
    if(iST == 10 || iST == 12) keydim1 = intLoc.i + intLoc.j - 1;
    else keydim1 = intLoc.j - intLoc.i;

    if(iST == 10 || iST == 11) {
      lvl1 = (keydim1 < -0.5) ? 0 : 1;
      lvl2 = (keydim1 < 0.5) ? 0 : 1;
    }
    else {
      lvl1 = (keydim1 < -0.25) ? 0 : 1;
      lvl2 = (keydim1 < 0.25) ? 0 : 1;
    }
    lvl = lvl1 + lvl2;
  }
  else if (iST >= 18 && iST <= 21) {
    //Y-junctions
    switch(iST) {
    case 18:
      keydim1 = intLoc.i;
      keydim2 = intLoc.j-fabs(intLoc.i-0.5);
      break;
    case 19:
      keydim1 = intLoc.j;
      keydim2 = intLoc.i-fabs(intLoc.j-0.5);
      break;
    case 20:
      keydim1 = intLoc.i;
      keydim2 = 1-intLoc.j-fabs(intLoc.i-0.5);
      break;
    case 21:
      keydim1 = intLoc.j;
      keydim2 = 1-intLoc.i-fabs(intLoc.j-0.5);
      break;
    }    
    lvl1 = (keydim2 > 0.5) ? 0 : 1;
    lvl2 = (keydim1 < 0.5) ? 1 : 2;
    lvl = lvl1*lvl2;
  }
  else {
    // T-junctions
    switch(iST) {
    case 14:
    case 22:
    case 26:
      keydim1 = intLoc.j;
      keydim2 = intLoc.i;
      break;
    case 15:
    case 25:
    case 29:
      keydim1 = intLoc.i;
      keydim2 = intLoc.j;
      break;
    case 16:
    case 24:
    case 28:
      keydim1 = 1-intLoc.j;
      keydim2 = intLoc.i;
      break;
    case 17:
    case 23:
    case 27:
      keydim1 = 1-intLoc.i;
      keydim2 = intLoc.j;
      break;
    }

    switch(iST) {
    case 14:
    case 15:
    case 16:
    case 17:
      lvl1 = (keydim1 < 0.5) ? 0 : 1;
      lvl2 = (keydim2 < 0.5) ? 1 : 2;
      lvl = lvl1 * lvl2;
      break;
    case 22:
    case 23:
    case 24:
    case 25:
      lvl1 = (3 * keydim1 < 2) ? 0 : 1;
      lvl2 = (keydim2 < 0.5) ? 1 : 2;
      lvl = lvl1 * lvl2;
      break;
    case 26:
    case 27:
    case 28:
    case 29:
      lvl1 = (keydim1 < 0.25) ? 0 : 1;
      lvl2 = (keydim2 < 0.5) ? 1 : 2;
      lvl = lvl1 * lvl2;
      break;
    }
  }		  

  return itsState.objLabels[lvl];
}

// ######################################################################
// #### ColorPixelClassifier
// ######################################################################

ColorPixelClassifier::ColorPixelClassifier() : PixelClassifier() 
{}

// ######################################################################
double ColorPixelClassifier::classifyAt(Image<PixRGB<byte> > im, uint C)
{
  ASSERT(C < itsNumClasses);
  ColorCat cc = itsCats[C];
  Point2D<int> center(im.getWidth()/2, im.getHeight()/2);
  double dist = colorL2Distance(im[center],cc.color);
  return -dist / cc.sig_cdist;
}

// ######################################################################
// #### GistPixelClassifier
// ######################################################################

GistPixelClassifier::GistPixelClassifier() : PixelClassifier()
{
  
}

// ######################################################################

void GistPixelClassifier::learnInput(Image<PixRGB<byte> > im, Image<uint> labels)
{
}

// ######################################################################
double GistPixelClassifier::classifyAt(Image<PixRGB<byte> > im, uint C)
{ 
  return 0;
}
// Free functions:
// ######################################################################

// ######################################################################
std::string convertToString(const QuadNode &q)
{
  std::string ret = "";
  for(uint i = 0; i < q.getDepth(); i++) ret+='\t';
  return ret + "(" + toStr(q.getArea()) + "): " + 
    "seg class " + toStr(q.getState());
}

// ######################################################################
std::string convertToString(const QuadNode::NodeState& n)
{
  return convertToString(n.segTemplate) + " :(" + 
    convertToString(n.objLabels[0]) + "," +
    convertToString(n.objLabels[1]) + "," +
    convertToString(n.objLabels[2]) + ")";
}

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
