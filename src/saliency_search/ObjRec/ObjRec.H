/*!@file ObjRec/ObjRec.H Obj Reconition class */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/ObjRec.H $
// $Id: ObjRec.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef ObjRec_H_DEFINED
#define ObjRec_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/Pixels.H"

class ObjRec : public ModelComponent
{
  struct EdgeState
  {
    Point2D<int> pos;
    float ori;
    PixRGB<byte> color;
    float prob;
  };
  struct LineState
  {
    Point2D<int> pos;
    float ori;
    float length;
    PixRGB<byte> color;
    float prob;
  };

  struct SquareState
  {
    Point2D<int> pos;
    float ori;
    float size;
    PixRGB<byte> color;
    float prob;
  };


  struct WorldState
  {
    Image<float> edgeProb;
    std::vector<EdgeState> edges;
    std::vector<LineState> lines;
    std::vector<SquareState> squares;
  };

public:
  //Constructor
  ObjRec(OptionManager& mgr,
          const std::string& descrName = "ObjRec",
          const std::string& tagName = "ObjRec");
  ~ObjRec();

  //###########################################################
  void start2();

  double predictWorld(const Image<PixRGB<byte> > &worldImg);
  Image<PixRGB<byte> > getWorldPredictImage();

  void setImageDims(const Dims &dims);


  void initialProposal ();
  void initialProposal(const Image<PixRGB<byte> > &worldImg);
  double getPosterior(WorldState &worldState);
  double getLikelihood(WorldState &worldState);
  double getEdgeLikelihood(WorldState &worldState);
  double getLineLikelihood(WorldState &worldState);
  double getSquareLikelihood(WorldState &worldState);


  void generateNewState(WorldState &worldState);
  void generateNewEdgeState(WorldState &worldState);
  void generateNewLineState(WorldState &worldState);
  void generateNewSquareState(WorldState &worldState);

  double sampleOriFromEdgeSpace(WorldState &worldState);
  Point2D<int> samplePosFromEdgeSpace(WorldState &worldState);

  double sampleOriFromLineSpace(WorldState &worldState);
  Point2D<int> samplePosFromLineSpace(WorldState &worldState);
  double sampleSizeFromLineSpace(WorldState &worldState);

  double sampleLengthFromSquareSpace(WorldState &worldState);

  double evalNewEdgesWorld(WorldState &oldWorldState, WorldState &newWorldState);
  double evalNewLinesWorld(WorldState &oldWorldState, WorldState &newWorldState);
  double evalNewSquaresWorld(WorldState &oldWorldState, WorldState &newWorldState);

  void normalizeWorld(WorldState &worldState);


  Image<PixRGB<byte> > showWorld(WorldState &worldState);
  Image<PixRGB<byte> > showEdgesWorld(WorldState &worldState);
  Image<PixRGB<byte> > showLinesWorld(WorldState &worldState);
  Image<PixRGB<byte> > showSquaresWorld(WorldState &worldState);


  ////// debuging utilities ///////////
  double evalLikelihood(const Image<PixRGB<byte> > &worldImg, const Point2D<int> &pos, double angle, double length);


  double edgesProb(const Image<PixRGB<byte> > &worldImg);
  double edgesLiklyProb(const Image<PixRGB<byte> > &worldImg);
  double edgesPriorProb();

  void houghLines();

  void samplePosterior(const Image<float> &posterior, Point2D<int> &loc, int stop=1000);

  void train(const Image<PixRGB<byte> > &img, const std::string label);
  std::string test(const Image<PixRGB<byte> > &img);


private:

  Dims itsImageDims;
  Image<PixRGB<byte> > itsPredictWorldImg;
  Image<PixRGB<byte> > itsCurrentWorldImg;

  std::vector<Image<float> > itsGabors;

  WorldState itsWorldState;
  bool itsInitProposal;

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
