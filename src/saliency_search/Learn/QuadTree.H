/*!@file Learn/QuadTree.H QuadTree Multi-Class Classifier */
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
//////////////////////////////////////////////////////////////////////////
//
// Implementation of the segmentation algorithm described in:
//
// Recursive Segmentation and Recognition Templates for 2D Parsing
// Leo Zhu, Yuanhao Chen, Yuan Lin, Chenxi Lin, Alan Yuille
// Advances in Neural Information Processing Systems, 2008
// 

#ifndef QUADTREE_H_DEFINED
#define QUADTREE_H_DEFINED

#include "Channels/RawVisualCortex.H"
#include "Image/Image.H"
#include "Image/PixelsTypes.H"
#include "Image/Point2D.H"
#include "Image/Rectangle.H"
#include "rutz/shared_ptr.h"
#include "Util/log.H"
#include "Util/StringConversions.H"
#include "nub/ref.h"
#include <limits>
#include <math.h>
#include <stdio.h>

//! QuadTree for segmentation and recognition from Leo Zhu's 2008 NIPS paper
class PixelClassifier;
class ColorPixelClassifier;

typedef std::vector<Point2D<int> > Neighborhood;
class QuadNode
{
public:
  struct NodeState {
    uint segTemplate; //region partition
    std::vector<byte> objLabels; //3 object labels
    double E;
    bool evaled;
    
    NodeState(uint st) : segTemplate(st) {}
    NodeState(uint st, std::vector<byte> oL) : segTemplate(st), objLabels(oL) {}
    NodeState(uint st, byte l1, byte l2, byte l3) : 
      segTemplate(st) {
      objLabels.push_back(l1); 
      objLabels.push_back(l2);
      objLabels.push_back(l3);
    }
    
    bool isSingleton() const {return segTemplate==0;}
    bool isDoubleton() const {return segTemplate==0 || (segTemplate >= 3 && segTemplate <= 5);}
  };

  QuadNode();
  QuadNode(rutz::shared_ptr<QuadNode> q);
  QuadNode(rutz::shared_ptr<QuadNode> q, NodeState n);
  Image<byte> getSegImage() {if(itsIsStale) refreshSegImage(); return itsSegImage;}
  Image<byte> getChildSegImage();
  Image<PixRGB<byte> > getColorizedSegImage();
  Image<PixRGB<byte> > getColorizedChildSegImage();
  byte getObjLabelAt(Point2D<int> loc) const;

  bool isLeaf() const {return itsIsLeaf;}

  void setArea(Rectangle r) {itsArea = r;}
  Rectangle getArea() const {return itsArea;}

  void setDepth(uint l) {itsDepth = l;}
  uint getDepth() const {return itsDepth;}

  void setLabels(std::vector<byte> v) {itsState.objLabels = v; itsIsStale = true;}
  void setLabel(uint i, byte v) {itsState.objLabels[i] = v; itsIsStale = true;}
  std::vector<byte> getLabels() const {return itsState.objLabels;}
  
  void setSegTemplate(byte c) {itsState.segTemplate = c; itsIsStale = true;}
  uint getSegTemplate() const {return itsState.segTemplate;}

  void setState(NodeState n) {itsState = n; itsIsStale = true;}
  NodeState getState() const {return itsState;}

  void storeEnergy(double e) {itsState.E = e; itsState.evaled = true;}
  double getEnergy() const {return itsState.E;}
  bool energySaved() const {return itsState.evaled;}

  void addChild(rutz::shared_ptr<QuadNode> n) {
    itsIsLeaf = false;
    if(itsChildren.size() < 4) itsChildren.push_back(n);
    else LINFO("Tried to add node in excess of 4");
  }

  Point2D<int> convertToGlobal(Point2D<int> p) const
  {return Point2D<int>(p.i + itsArea.left(), p.j + itsArea.top());}

  Point2D<int> convertToLocal(Point2D<int> p) const
  {return Point2D<int>(p.i - itsArea.left(), p.j - itsArea.top());}

  rutz::shared_ptr<QuadNode> getChild(uint i) const {
    return itsChildren[i];
  }

private:
  void refreshSegImage();
  Image<PixRGB<byte> > colorLabels(Image<byte> im) const;

  uint itsDepth;

  // NB: if the node is a leaf, 
  // then the shared_ptrs to children are uninitialized
  bool itsIsLeaf;
  bool itsIsStale;
  
  Rectangle itsArea;

  NodeState itsState;
  Image<byte> itsSegImage;
  rutz::shared_ptr<QuadNode> itsParent;
  
  // top left, top right, bottom left, bottom right
  // NB: the tree structure is static once initialized
  std::vector<rutz::shared_ptr<QuadNode> > itsChildren;   
};

class QuadTree
{
public:
  // construct a quad tree on a given depth and image size
  QuadTree(int Nlevels, Dims d); 
  QuadTree(int Nlevels, Image<PixRGB<byte> > im);

  void initAlphas() {double weights[] = {1, 2.5, 2.5}; itsAlphas.assign(weights,weights+3);}
  void addTreeUnder(rutz::shared_ptr<QuadNode> parent,int Nlevels, Rectangle r);

  void cacheClassifierResult();
  double evaluateClassifierAt(rutz::shared_ptr<QuadNode> q) const; // classifier accuracy

  double evaluateCohesionAt(rutz::shared_ptr<QuadNode> q) const; //cohesion - similar pixels in similar groups in one region

  double evaluateCorrespondenceAt(rutz::shared_ptr<QuadNode> q) const; //labeling is the same at both layers, between a parent and its children

  double evaluateTotalEnergyAt(rutz::shared_ptr<QuadNode> q) const;
  //  byte getClassOutputAt(Point2D<int> P) const;
  void printTree() const;
  std::string writeTree() const;
  
  void setClassifier(rutz::shared_ptr<PixelClassifier> cc) {itsClassifier = cc;}
  rutz::shared_ptr<PixelClassifier> getClassifier() const {return itsClassifier;}

  rutz::shared_ptr<QuadNode> getRootNode() const {return itsRootNode;}

  std::vector<QuadNode::NodeState> generateProposalsAt(rutz::shared_ptr<QuadNode> q, double thresh);
private:

  uint itsNumLevels;
  Dims itsDims;
  Image<PixRGB<byte> > itsImage;

  rutz::shared_ptr<PixelClassifier> itsClassifier;
  std::vector<Image<double> > itsClassifierOutput;
  Image<byte> itsBestClassOutput;
  rutz::shared_ptr<QuadNode> itsRootNode;
  std::deque<rutz::shared_ptr<QuadNode> > itsNodes;  
  std::vector<double> itsAlphas;
};


// base class for a pixel-based classifier
class PixelClassifier 
{
public:
  PixelClassifier() {itsNumClasses=0;}

  uint getNumClasses() const {return itsNumClasses;}

  virtual void learnInput(Image<PixRGB<byte> > im, Image<uint> labels) = 0;
  virtual double classifyAt(Image<PixRGB<byte> > im, uint C) = 0;
protected:
  uint itsNumClasses;
};

class ColorPixelClassifier : public PixelClassifier
{
public:
  struct ColorCat {
    PixRGB<byte> color;
    double sig_cdist;
    
    ColorCat() {}
    ColorCat(PixRGB<byte> c, double d) {color = c; sig_cdist = d;}
  };

  ColorPixelClassifier();

  void learnInput(Image<PixRGB<byte> > im, Image<uint> labels) {}
  void addCategory(ColorCat cc) {itsCats.push_back(cc);
    itsNumClasses++;}
  
  // just uses color of center pixel for now
  double classifyAt(Image<PixRGB<byte> > im, uint C); 
private:
  std::vector<ColorCat> itsCats;
};

class GistPixelClassifier : public PixelClassifier
{
public:  
  // calculates local gist in a sliding window around each pixel
  GistPixelClassifier();

  //  void setVC(nub::ref<RawVisualCortex> VC) {itsVC = VC;}
  void learnInput(Image<PixRGB<byte> > im, Image<uint> labels);
  double classifyAt(Image<PixRGB<byte> > im, uint C);
  
private:
  //  nub::ref<RawVisualCortex> itsVC;
};

  ////////////////////////////////////////////////////
// free functions 

// just the manhattan distance (L1)
uint inline colorDistance(PixRGB<byte> A, PixRGB<byte>B) {
  return abs(A[0]-B[0])+abs(A[1]-B[1])+abs(A[2]-B[2]);
}

// euclidean distance in (L2) space
double inline colorL2Distance(PixRGB<byte> A, PixRGB<byte>B) {
  return sqrt((A[0]-B[0])*(A[0]-B[0])+(A[1]-B[1])*(A[1]-B[1])+(A[2]-B[2])*(A[2]-B[2]));
}


std::string convertToString(const QuadNode &q);
std::string convertToString(const QuadNode::NodeState& n); 

#endif // QUADTREE_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
