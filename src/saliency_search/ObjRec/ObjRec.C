/*!@file ObjRec/ObjRec.C Obj Reconition class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/ObjRec.C $
// $Id: ObjRec.C 10794 2009-02-08 06:21:09Z itti $
//

#include "ObjRec/ObjRec.H"
#include "Component/OptionManager.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"
#include "Image/Kernels.H"   // for dogFilter()
#include "Image/Convolutions.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "GUI/DebugWin.H"
#include "Util/MathFunctions.H"
#include "SIFT/Histogram.H"
#include "SIFT/FeatureVector.H"


void findMinMax(const std::vector<double> &vec, double &min, double &max)
{
  max = vec[0];
  min = max;
  for (uint n = 1 ; n < vec.size() ; n++)
  {
    if (vec[n] > max) max = vec[n];
    if (vec[n] < min) min = vec[n];
  }
}

double normalizeHist(std::vector<double> &hist)
{

  double sum = 0;
  for(uint i=0; i<hist.size(); i++)
    sum += hist[i];

  for(uint i=0; i<hist.size(); i++)
    hist[i] /= sum;

  return sum;

}

// ######################################################################
ObjRec::ObjRec(OptionManager& mgr, const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsImageDims(256, 256),
  itsInitProposal(true)
{

}


ObjRec::~ObjRec()
{
}

void ObjRec::setImageDims(const Dims &dims)
{
  itsImageDims = dims;
}

void ObjRec::start2()
{
  initRandomNumbers();

  //Generate gabor patches for speed up
  itsGabors.resize(180);
  for (uint i=0; i<itsGabors.size(); i++)
  {
    //float filter_period = 5;
    //float elongation = 1.0;
    //float angle = i;
    //int size = -1;
    //const double major_stddev = filter_period / 3.0;
    //const double minor_stddev = major_stddev * elongation;

    //Image<float> gabor = gaborFilter3(major_stddev, minor_stddev,
    //    filter_period, 0.0f, 180 - angle + 90.0 , size);

    float ori = i;
    int scale = 1;
    Image<float> dog = dogFilter<float>(1.75F + 0.5F*scale, ori, 3 + 1*scale);

    // normalize to zero mean:
    dog -= mean(dog);

    // normalize to unit sum-of-squares:
    dog /= sum(squared(dog));

    itsGabors[i] = dog;

  }

  //initial proposal
  initialProposal();
}

void ObjRec::initialProposal()
{

  int size = 20;
  itsWorldState.edges.resize(size*size);
  for(uint i=0; i<itsWorldState.edges.size(); i++)
  {
    int y = i/size;
    int x = i%size;
    itsWorldState.edges[i].pos = Point2D<int>(x*itsImageDims.w()/size,y*itsImageDims.h()/size);
    itsWorldState.edges[i].ori = 0.0;
    itsWorldState.edges[i].color = PixRGB<byte>(0,255,0);
    itsWorldState.edges[i].prob = 0.1;
  }

  itsWorldState.lines.resize(10);

  for(uint i=0; i<itsWorldState.lines.size(); i++)
  {
    int y = i;
    int x = itsImageDims.w()/2;
    itsWorldState.lines[i].pos = Point2D<int>(x,y*itsImageDims.h()/10);
    itsWorldState.lines[i].ori = 90.0;
    itsWorldState.lines[i].length = 30.0;
    itsWorldState.lines[i].color = PixRGB<byte>(255,0,255);
    itsWorldState.lines[i].prob = 0.1;

  }

}

void ObjRec::initialProposal(const Image<PixRGB<byte> > &worldImg)
{

  itsWorldState.edges.clear();
  Image<float> mag, ori;
  Image<float> worldLum = luminance(worldImg);
  gradientSobel(worldLum, mag, ori, 3);

  itsWorldState.edges.clear();
  Image<float> edgeProb = mag / sum(mag); //normalized edge probability
  itsWorldState.edgeProb = edgeProb;

  float maxVal, minVal;
  getMinMax(mag, minVal, maxVal);

  for(int y=0; y<mag.getHeight(); y++)
    for(int x=0; x<mag.getWidth(); x++)
    {
      float angle = ori.getVal(x,y) + M_PI/2;
      while(angle > M_PI) angle-=M_PI;
      while (angle < 0) angle+=M_PI;
      float p = mag.getVal(x,y);

      if (p > maxVal*0.10)
      {
        EdgeState es;
        es.pos = Point2D<int>(x,y);
        es.ori = angle*180/M_PI;
        es.color = PixRGB<byte>(0,255,0);
        es.prob = p;
        itsWorldState.edges.push_back(es);
      }
    }

  itsWorldState.lines.resize(10);

  for(uint i=0; i<itsWorldState.lines.size(); i++)
  {
    itsWorldState.lines[i].pos = samplePosFromEdgeSpace(itsWorldState);
    itsWorldState.lines[i].ori =  sampleOriFromEdgeSpace(itsWorldState);
    itsWorldState.lines[i].length = 30.0;
    itsWorldState.lines[i].color = PixRGB<byte>(255,0,255);
    itsWorldState.lines[i].prob = 0;
  }

  itsWorldState.squares.resize(1);
  for(uint i=0; i<itsWorldState.squares.size(); i++)
  {
    itsWorldState.squares[i].pos = samplePosFromLineSpace(itsWorldState);
    itsWorldState.squares[i].ori =  sampleOriFromLineSpace(itsWorldState);
    itsWorldState.squares[i].size = sampleSizeFromLineSpace(itsWorldState);
    itsWorldState.squares[i].color = PixRGB<byte>(0,255,0);
    itsWorldState.squares[i].prob = 0;
  }

}

void ObjRec::generateNewState(WorldState &worldState)
{
  generateNewEdgeState(worldState);
  generateNewLineState(worldState);
}

void ObjRec::generateNewEdgeState(WorldState &worldState)
{
  int idum = getIdum();

  static int modParam = 1;

  for(uint i=0; i<worldState.edges.size(); i++)
  {
    if (modParam == 0)
    {
      worldState.edges[i].pos = Point2D<int>(
          int(itsWorldState.edges[i].pos.i + 1*gasdev(idum)),
          int(itsWorldState.edges[i].pos.j + 1*gasdev(idum)));

      if (worldState.edges[i].pos.i > itsImageDims.w())
        worldState.edges[i].pos.i = itsImageDims.w();
      if (worldState.edges[i].pos.j > itsImageDims.h())
        worldState.edges[i].pos.j = itsImageDims.h();
      if (worldState.edges[i].pos.i < 0)
        worldState.edges[i].pos.i = 0;
      if (worldState.edges[i].pos.j < 0)
        worldState.edges[i].pos.j = 0;
    }

    if (modParam == 1)
    {
      //Standard search
     worldState.edges[i].ori = itsWorldState.edges[i].ori + 5*gasdev(idum);

      //perceptual grouping search
      // Look at the neighbors and try the same position and orientation

      if (worldState.edges[i].prob != 0)
      {
        float avgOri = 0;
        int oriTotal = 0;
        for(uint j=0; j<worldState.edges.size(); j++)
        {
          if (j == i) continue;

          //Look at near neighbors
          float dist = worldState.edges[i].pos.distance(worldState.edges[j].pos);
          if (dist < 50)
          {
            avgOri += worldState.edges[j].ori;
            oriTotal++;
          }
        }
        avgOri /= oriTotal;
        worldState.edges[i].ori = avgOri;
      } else {
        worldState.edges[i].ori = itsWorldState.edges[i].ori + 5*gasdev(idum);
      }


      if (worldState.edges[i].ori >= 180)
        worldState.edges[i].ori -= 180;
      if (worldState.edges[i].ori < 0)
        worldState.edges[i].ori += 180;
    }

  }

  if (modParam++ > 1)
    modParam = 0;

}

void ObjRec::generateNewLineState(WorldState &worldState)
{
  int idum = getIdum();

  static int modParam = 0;

  for(uint i=0; i<worldState.lines.size(); i++)
  {
    if (modParam == 0)
    {
      Point2D<int> probablePos = samplePosFromEdgeSpace(itsWorldState);
      worldState.lines[i].pos = Point2D<int>(
          int(probablePos.i + 1*gasdev(idum)),
          int(probablePos.j + 1*gasdev(idum)));

      if (worldState.lines[i].pos.i > itsImageDims.w())
        worldState.lines[i].pos.i = itsImageDims.w();
      if (worldState.lines[i].pos.j > itsImageDims.h())
        worldState.lines[i].pos.j = itsImageDims.h();
      if (worldState.lines[i].pos.i < 0)
        worldState.lines[i].pos.i = 0;
      if (worldState.lines[i].pos.j < 0)
        worldState.lines[i].pos.j = 0;
    }


    if (modParam == 1)
    {
      //use the edges as prior
      worldState.lines[i].ori = sampleOriFromEdgeSpace(itsWorldState) + 1*gasdev(idum);

      if (worldState.lines[i].ori >= 180)
        worldState.lines[i].ori -= 180;
      if (worldState.lines[i].ori < 0)
        worldState.lines[i].ori += 180;
    }

    if (modParam == 2)
    {
      //Standard search
      worldState.lines[i].length = sampleLengthFromSquareSpace(itsWorldState) + 1*gasdev(idum);
      //itsWorldState.lines[i].length + 3*gasdev(idum);

      //if (worldState.lines[i].length >= itsImageDims.w())
      //  worldState.lines[i].length = itsImageDims.w();
      if (worldState.lines[i].length < 5)
        worldState.lines[i].length = 5;
    }

    worldState.lines[i].color = PixRGB<byte>(0,0,255);
    worldState.lines[i].prob = 0.1;

  }


  if (modParam++ > 2)
    modParam = 0;

}

/*void ObjRec::generateNewSquareState(WorldState &worldState)
{
  int idum = getIdum();

  static int modParam = 0;

  for(uint i=0; i<worldState.squares.size(); i++)
  {
    if (modParam == 0)
    {
      Point2D<int> probablePos = samplePosFromLineSpace(itsWorldState);
      worldState.squares[i].pos = Point2D<int>(
          int(probablePos.i + 1*gasdev(idum)),
          int(probablePos.j + 1*gasdev(idum)));

      if (worldState.squares[i].pos.i > itsImageDims.w())
        worldState.squares[i].pos.i = itsImageDims.w();
      if (worldState.squares[i].pos.j > itsImageDims.h())
        worldState.squares[i].pos.j = itsImageDims.h();
      if (worldState.squares[i].pos.i < 0)
        worldState.squares[i].pos.i = 0;
      if (worldState.squares[i].pos.j < 0)
        worldState.squares[i].pos.j = 0;
    }

    if (modParam == 1)
    {
      //use the edges as prior
      worldState.squares[i].ori = sampleOriFromLineSpace(itsWorldState) + 1*gasdev(idum);

      if (worldState.squares[i].ori >= 180)
        worldState.squares[i].ori -= 180;
      if (worldState.squares[i].ori < 0)
        worldState.squares[i].ori += 180;
    }

    if (modParam == 2)
    {
      //Standard search
      worldState.squares[i].size = sampleSizeFromLineSpace(itsWorldState) + 1*gasdev(idum);

      //if (worldState.squares[i].length >= itsImageDims.w())
      //  worldState.squares[i].length = itsImageDims.w();
      if (worldState.squares[i].size < 5)
        worldState.squares[i].size = 5;
    }

    worldState.squares[i].color = PixRGB<byte>(255,0,255);
    worldState.squares[i].prob = 0.1;

  }


  if (modParam++ > 2)
    modParam = 0;

}*/

void ObjRec::generateNewSquareState(WorldState &worldState)
{

  //TODO need improtance sampling
  Image<float> posVotes(itsImageDims, ZEROS);
  Image<float> oriSizeVotes(181, itsImageDims.h(), ZEROS);

  for(uint i=0; i<worldState.lines.size(); i++)
  {
      float ori = worldState.lines[i].ori;
      float size = worldState.lines[i].length;
      Point2D<int> pos = worldState.lines[i].pos;


      int x =  int(cos((ori+90)*M_PI/180)*size/2);
      int y =  int(sin((ori+90)*M_PI/180)*size/2);

      for(int j=y-5; j<y+5; j++)
        for(int i=x-5; i<x+5; i++)
        {
          if (posVotes.coordsOk(pos.i+i,pos.j-j))
          {
            float val = posVotes.getVal(pos.i+i, pos.j-j);
            posVotes.setVal(pos.i+i, pos.j-j, val+1);
          }

          if (posVotes.coordsOk(pos.i-i,pos.j+j))
          {
            float val = posVotes.getVal(pos.i-i, pos.j+j);
            posVotes.setVal(pos.i-i, pos.j+j, val+1);
          }
        }


      for(int j=(int)size-5; j<(int)size+5; j++)
        for(int i=(int)ori-5; i<(int)ori+5; i++)
        {
          if (oriSizeVotes.coordsOk(i,j))
          {
            float val = oriSizeVotes.getVal(i,j);
            oriSizeVotes.setVal(i,j, val+1);
          }

          if (oriSizeVotes.coordsOk(i,j))
          {
            ori += 90;
            if (ori > 180) ori -= 180;
            float val = oriSizeVotes.getVal(i,j);
            oriSizeVotes.setVal(i,j, val+1);
          }
        }

  }
  //SHOWIMG(posVotes);
  //SHOWIMG(oriSizeVotes);


  Point2D<int> maxPos; float maxVal;
  findMax(oriSizeVotes, maxPos, maxVal);
  LINFO("Max at %ix%i %f", maxPos.i, maxPos.j, maxVal);

  Point2D<int> maxPos1; float maxVal1;
  findMax(posVotes, maxPos1, maxVal1);

  worldState.squares[0].pos = maxPos1;
  worldState.squares[0].ori = maxPos.i;
  worldState.squares[0].size = maxPos.j;
  worldState.squares[0].color = PixRGB<byte>(255,0,255);
  worldState.squares[0].prob = 0.1;



}

double ObjRec::sampleOriFromEdgeSpace(WorldState &worldState)
{
  //Pick an edge at random and use that for orientation
  //Rejection sampleing
  for(int i=0; i<100; i++)
  {
    int j = randomUpToNotIncluding(worldState.edges.size());
    if (worldState.edges[j].prob != 0)
      return worldState.edges[j].ori;
  }

  return 0;
}

Point2D<int> ObjRec::samplePosFromEdgeSpace(WorldState &worldState)
{
  //Pick an edge at random and use that for orientation
  //Rejection sampleing
  for(int i=0; i<100; i++)
  {
    int j = randomUpToNotIncluding(worldState.edges.size());
    if (worldState.edges[j].prob > 0)
      return worldState.edges[j].pos;
  }
  return Point2D<int>(100,100);
}

double ObjRec::sampleLengthFromSquareSpace(WorldState &worldState)
{

  return worldState.squares[0].size;

}

double ObjRec::sampleOriFromLineSpace(WorldState &worldState)
{
  //Pick an edge at random and use that for orientation
  //Rejection sampleing
  for(int i=0; i<100; i++)
  {
    int j = randomUpToNotIncluding(worldState.lines.size());
    if (worldState.lines[j].prob != 0)
      return worldState.lines[j].ori;
  }

  return 0;
}

Point2D<int> ObjRec::samplePosFromLineSpace(WorldState &worldState)
{
  //Pick an edge at random and use that for orientation
  //Rejection sampleing
  for(int i=0; i<100; i++)
  {
    int j = randomUpToNotIncluding(worldState.lines.size());
    if (worldState.lines[j].prob > 0)
      return worldState.lines[j].pos;
  }
  return Point2D<int>(100,100);
}

double ObjRec::sampleSizeFromLineSpace(WorldState &worldState)
{
  //Pick an edge at random and use that for orientation
  //Rejection sampleing
  for(int i=0; i<100; i++)
  {
    int j = randomUpToNotIncluding(worldState.lines.size());
    if (worldState.lines[j].prob > 0)
      return worldState.lines[j].length;
  }
  return 10;
}


Image<PixRGB<byte> > ObjRec::showWorld(WorldState &worldState)
{
  Image<PixRGB<byte> > edgesWorld = showEdgesWorld(worldState);
  Image<PixRGB<byte> > linesWorld = showLinesWorld(worldState);

  Image<PixRGB<byte> > worldImg = edgesWorld + linesWorld;
 // Image<PixRGB<byte> > worldImg = linesWorld;
  return worldImg;
}

Image<PixRGB<byte> > ObjRec::showEdgesWorld(WorldState &worldState)
{

  Image<PixRGB<byte> > worldImg(itsImageDims, ZEROS);
  Image<float> worldProbImg(itsImageDims, ZEROS);

  //SHOW Edges
  for(uint i=0; i<worldState.edges.size(); i++)
  {
    if (worldState.edges[i].prob != 0)
    {

      //PixRGB<byte> color(0, int(1/worldState.edges[i].prob*255), 0);
      PixRGB<byte> color = worldState.edges[i].color;
      drawLine(worldImg,
          worldState.edges[i].pos,
          worldState.edges[i].ori*M_PI/180,
          10,
          color);
      worldProbImg.setVal(worldState.edges[i].pos, worldState.edges[i].prob);
    }
  }

  return worldImg;

}

Image<PixRGB<byte> > ObjRec::showLinesWorld(WorldState &worldState)
{

  Image<PixRGB<byte> > worldImg(itsImageDims, ZEROS);
  Image<float> worldProbImg(itsImageDims, ZEROS);

  for(uint i=0; i<worldState.lines.size(); i++)
  {
    if (worldState.lines[i].prob != 0)
    {

      //PixRGB<byte> color(0, int(worldState.lines[i].prob*255), 0);
      PixRGB<byte> color = worldState.lines[i].color;
      //color[0] = (int) ((float)color[0] * worldState.lines[i].prob);
      //color[1] = (int) ((float)color[1] * worldState.lines[i].prob);
      //color[2] = (int) ((float)color[2] * worldState.lines[i].prob);

      drawLine(worldImg,
          worldState.lines[i].pos,
          worldState.lines[i].ori*M_PI/180,
          worldState.lines[i].length,
          color);
      //worldProbImg.setVal(worldState.lines[i].pos, worldState.lines[i].prob);
    }
  }

  return worldImg;

}

Image<PixRGB<byte> > ObjRec::showSquaresWorld(WorldState &worldState)
{

  Image<PixRGB<byte> > worldImg(itsImageDims, ZEROS);
  Image<float> worldProbImg(itsImageDims, ZEROS);

  for(uint i=0; i<worldState.squares.size(); i++)
  {
    if (worldState.squares[i].prob > 0)
    {

      //PixRGB<byte> color(0, int(worldState.lines[i].prob*255), 0);
      //color[0] = (int) ((float)color[0] * worldState.lines[i].prob);
      //color[1] = (int) ((float)color[1] * worldState.lines[i].prob);
      //color[2] = (int) ((float)color[2] * worldState.lines[i].prob);


      PixRGB<byte> color = worldState.squares[i].color;
      Point2D<int> pos = worldState.squares[i].pos;
      float size = worldState.squares[i].size;
      float ori = worldState.squares[i].ori*M_PI/180;

      drawRectOR(worldImg,
          Rectangle(Point2D<int>(pos.i-(int)(size/2),pos.j-(int)(size/2)), Dims((int)size, (int)size)),
          color,
          2,
          ori);
      //worldProbImg.setVal(worldState.lines[i].pos, worldState.lines[i].prob);
    }
  }

  return worldImg;

}



double ObjRec::predictWorld(const Image<PixRGB<byte> > &worldImg)
{
  itsCurrentWorldImg = worldImg;

  //The Metoplis-Hastings algorithem

  //TODO: store and use value from storage as opposed to calculating
  // a new posterior each time
  double p_of_xi = getPosterior(itsWorldState);

  LINFO("Old Posterior %f", p_of_xi);

  //generate new proposal sample
  WorldState worldState = itsWorldState;
  generateNewState(worldState);
  LINFO("New world state");
  showWorld(worldState);

  double p_of_xnew = getPosterior(worldState);
  LINFO("New Posterior %f", p_of_xnew);

  //evaluate the new world

  double edgesWorldProb = evalNewEdgesWorld(itsWorldState, worldState);
  double linesWorldProb = evalNewLinesWorld(itsWorldState, worldState);
  LINFO("EdgesProb: %f, linesProb: %f", edgesWorldProb, linesWorldProb);


  //show the current world state
  itsPredictWorldImg = showWorld(itsWorldState);

  return edgesWorldProb + linesWorldProb;
  //return linesWorldProb;
}

double ObjRec::evalNewEdgesWorld(WorldState &oldWorldState, WorldState &newWorldState)
{
  double p_of_world = 0;
  for(uint i=0; i<oldWorldState.edges.size(); i++)
  {
    double p_of_xnew = newWorldState.edges[i].prob;
    double p_of_xi = oldWorldState.edges[i].prob;

    float A = std::min(double(1), p_of_xnew/p_of_xi);
    //if (randomDouble() < A) //accept the new proposal with some probability
    if (A == 1.0) //always accept the new proposal if its better
    {
      oldWorldState.edges[i].pos =    newWorldState.edges[i].pos;
      oldWorldState.edges[i].ori =    newWorldState.edges[i].ori;
      oldWorldState.edges[i].color =  newWorldState.edges[i].color;
      oldWorldState.edges[i].prob =   newWorldState.edges[i].prob;
    }
    p_of_world += oldWorldState.edges[i].prob;
  }

  return p_of_world;
}

double ObjRec::evalNewLinesWorld(WorldState &oldWorldState, WorldState &newWorldState)
{
  double p_of_world = 0;
  for(uint i=0; i<oldWorldState.lines.size(); i++)
  {
    double p_of_xnew = newWorldState.lines[i].prob;
    double p_of_xi = oldWorldState.lines[i].prob;

    float A = std::min(double(1), p_of_xnew/p_of_xi);
    //if (randomDouble() < A) //accept the new proposal with some probability
    if (A == 1.0) //always accept the new proposal if its better
    {
      oldWorldState.lines[i].pos =    newWorldState.lines[i].pos;
      oldWorldState.lines[i].ori =    newWorldState.lines[i].ori;
      oldWorldState.lines[i].length = newWorldState.lines[i].length;
      //oldWorldState.lines[i].color =  newWorldState.lines[i].color;
      oldWorldState.lines[i].prob =   newWorldState.lines[i].prob;
    }
    p_of_world += oldWorldState.lines[i].prob;
  }

  return p_of_world;
}

double ObjRec::evalNewSquaresWorld(WorldState &oldWorldState, WorldState &newWorldState)
{
  double p_of_world = 0;
  for(uint i=0; i<oldWorldState.squares.size(); i++)
  {
    double p_of_xnew = newWorldState.squares[i].prob;
    double p_of_xi = oldWorldState.squares[i].prob;

    float A = std::min(double(1), p_of_xnew/p_of_xi);
    //if (randomDouble() < A) //accept the new proposal with some probability
    if (A == 1.0) //always accept the new proposal if its better
    {
      oldWorldState.squares[i].pos =    newWorldState.squares[i].pos;
      oldWorldState.squares[i].ori =    newWorldState.squares[i].ori;
      oldWorldState.squares[i].size = newWorldState.squares[i].size;
      //oldWorldState.squares[i].color =  newWorldState.squares[i].color;
      oldWorldState.squares[i].prob =   newWorldState.squares[i].prob;
    }
    p_of_world += oldWorldState.squares[i].prob;
  }

  return p_of_world;
}

double ObjRec::getPosterior(WorldState &worldState)
{
  //For now just the likelihood

  return getLikelihood(worldState);
}

double ObjRec::getLikelihood(WorldState &worldState)
{

  return getEdgeLikelihood(worldState) + getLineLikelihood(worldState);
}


double ObjRec::getEdgeLikelihood(WorldState &worldState)
{

  Image<float> worldLum = luminance(itsCurrentWorldImg);

  //evaluate the likelihood of all the edges
  double totalProb = 0;
  for(uint i=0; i<worldState.edges.size(); i++)
  {
    int edgeOri = (int)worldState.edges[i].ori;
    if (edgeOri < 0) edgeOri += 180;
    ASSERT(edgeOri >= 0 && edgeOri < 180);

    Point2D<int> edgePos = worldState.edges[i].pos;


    Image<float> gabor = itsGabors[edgeOri];
    Point2D<int> gaborPos(edgePos.i - (gabor.getWidth()/2),
        edgePos.j - (gabor.getHeight()/2));

    double prob = 0;
    if (worldLum.coordsOk(
          gaborPos.i + gabor.getWidth() - 1,
          gaborPos.j + gabor.getHeight() -1 ) &&
        worldLum.coordsOk(gaborPos))
    {
      Image<float> worldPart = crop(worldLum,
          gaborPos,
          gabor.getDims());

      prob = sum(worldPart*gabor); //normalize to 1
      if (prob < 0) prob = 0.0;


      //Look at how many edges around this one have the same orientation
      //If so, the update the probability proprotianl to the number of edges
      //with the same orientation
      //Perceptual grouping
      if (prob != 0)
      {
        float oriTotal = 0;
        for(uint j=0; j<worldState.edges.size(); j++)
        {
          if (j == i) continue;

          //Look at near neighbors
          float dist = worldState.edges[i].pos.distance(worldState.edges[j].pos);
          if (dist < 50) //look at close neighbors
          {
            oriTotal += fabs(worldState.edges[j].ori-worldState.edges[i].ori);
          }
        }
        prob += 1/oriTotal;
      }

      //if (prob != 0)
      //{
      //  SHOWIMG(gabor);
      //  SHOWIMG(worldPart);
      //  LINFO("Prob %f", prob);
      //}
    }
    worldState.edges[i].prob = prob;
    totalProb += prob;
  }

  return totalProb;
}


/*double ObjRec::getLineLikelihood(WorldState &worldState)
{

  double totalProb = 0;
  for(uint line=0; line<worldState.lines.size(); line++)
  {

    float lineOri = (int)worldState.lines[line].ori;
    Point2D<int> linePos = worldState.lines[line].pos;
    double lineLen = worldState.lines[line].length;

    if (lineLen < 1)
    {
      worldState.lines[line].prob = 0;
      continue;
    }

    float period = 15;
    float elongation = lineLen/(2*period);
    float theta = 180 - lineOri + 90.0;
    const double major_stddev = period / 3.0;
    const double minor_stddev = major_stddev * elongation;

    // change the angles in degree to the those in radians:
    const float rtDeg = M_PI / 180.0F * theta;

    // calculate constants:
    //const float omega = (2.0F * M_PI) / period;
    const float co = cos(rtDeg), si = sin(rtDeg);
    const float major_sigq = 2.0F * major_stddev * major_stddev;
    const float minor_sigq = 2.0F * minor_stddev * minor_stddev;

    float muX = linePos.i, muY = linePos.j;

    double probLineOri = 0;
    double probNotLineOri1 = 0;
    double probNotLineOri2 = 0;

 //   Image<float> probImg(worldLum.getDims(), ZEROS);
    for(uint i=0; i<worldState.edges.size(); i++)
    {
      Point2D<int> edgePos = worldState.edges[i].pos;
      int x = edgePos.i;
      int y = edgePos.j;

      float edgeProb = worldState.edges[i].prob;
      float edgeOri =  worldState.edges[i].ori;

      float lkly = edgeProb*gauss(edgeOri, lineOri, 2.0F)/50;

      //p(lineOri)
      const float major = (x-muX)*co + (y-muY)*si;
      const float minor = (x-muX)*si - (y-muY)*co;
      float prior1 = (1.0F
          * exp(-(major*major) / major_sigq)
          * exp(-(minor*minor) / minor_sigq));
      probLineOri += prior1*lkly;

     // p(edge!=lineOri)
      float x1 = cos(lineOri*M_PI/180)*lineLen/2;
      float y1 = sin(lineOri*M_PI/180)*lineLen/2;

      float stddev = major_stddev;
      float prior2 = (1.0F
         * exp(-( (x-muX-x1)*(x-muX-x1) )/ major_sigq)
         * exp(-( (y-muY+y1)*(y-muY+y1) )/ major_sigq) );
      probNotLineOri1 += prior2*(1-lkly);

      float prior3 = (1.0F
         * exp(-( (x-muX+x1)*(x-muX+x1) )/ (stddev*stddev))
         * exp(-( (y-muY-y1)*(y-muY-y1) )/ (stddev*stddev)) );
      probNotLineOri2 += prior3*(1-lkly);

    }

    double sum =  probLineOri + probNotLineOri1 + probNotLineOri2;
    double prob = probLineOri/sum *
      probNotLineOri1/sum *
      probNotLineOri2/sum;

    if (prob < 0) prob = 0.0;

    worldState.lines[line].prob = prob;
    LINFO("Line %i prob %f", line, prob);
    totalProb += prob;
  }

  return totalProb;
}
*/

double ObjRec::getLineLikelihood(WorldState &worldState)
{

  double totalProb = 0;
  for(uint line=0; line<worldState.lines.size(); line++)
  {

    float lineOri = worldState.lines[line].ori;
    Point2D<int> linePos = worldState.lines[line].pos;
    double lineLen = worldState.lines[line].length*0.75; //only evaluate 75% of the length

    if (lineLen < 1)
    {
      worldState.lines[line].prob = 0;
      continue;
    }

    Image<float> lineModel(worldState.edgeProb.getDims(), ZEROS);

    drawLine(lineModel, linePos, lineOri*M_PI/180, lineLen, 1.0F, 1);


    Image<float> lineProb = lineModel*worldState.edgeProb;

    double sum = 0;
    double prob = 0;
    for(int i=0; i<lineModel.getSize(); i++)
    {
      if (lineModel[i] > 0)
      {
        sum++;
        prob += lineProb[i];
      }
    }

    worldState.lines[line].prob = prob;
    totalProb += prob;
  }

  return totalProb;
}

double ObjRec::getSquareLikelihood(WorldState &worldState)
{

  double totalProb = 0;
  for(uint square=0; square<worldState.squares.size(); square++)
  {

    float squareOri = worldState.squares[square].ori*M_PI/180;
    Point2D<int> squarePos = worldState.squares[square].pos;
    double squareSize = worldState.squares[square].size*0.75; //only evaluate 75% of the length

    if (squareSize < 1)
    {
      worldState.squares[square].prob = 0;
      continue;
    }

    Image<float> squareModel(worldState.edgeProb.getDims(), ZEROS);

    drawRectOR(squareModel,
        Rectangle(Point2D<int>(squarePos.i-(int)(squareSize/2),squarePos.j-(int)(squareSize/2)),
          Dims((int)squareSize, (int)squareSize)),
        1.0F,
        2,
        squareOri);


    Image<float> squareProb = squareModel*worldState.edgeProb;

    double sum = 0;
    double prob = 0;
    for(int i=0; i<squareModel.getSize(); i++)
    {
      if (squareModel[i] > 0)
      {
        sum++;
        prob += squareProb[i];
      }
    }

    worldState.squares[square].prob = prob;
    totalProb += prob;
  }

  return totalProb;
}





Image<PixRGB<byte> > ObjRec::getWorldPredictImage()
{
  return itsPredictWorldImg;
}



///////////////////////////// Debuging utility functions //////////////////////////////
Image<PixRGB<byte> > showHist(const std::vector<double> &hist)
{
  int w = 256, h = 256;
  if (hist.size() > (uint)w) w = hist.size();

  if (hist.size() == 0) return Image<PixRGB<byte> >();

  int dw = w / hist.size();
  Image<byte> res(w, h, ZEROS);

  // draw lines for 10% marks:
  for (int j = 0; j < 10; j++)
    drawLine(res, Point2D<int>(0, int(j * 0.1F * h)),
             Point2D<int>(w-1, int(j * 0.1F * h)), byte(64));
  drawLine(res, Point2D<int>(0, h-1), Point2D<int>(w-1, h-1), byte(64));

  double minii, maxii;
  findMinMax(hist, minii, maxii);

   // uniform histogram
  if (maxii == minii) minii = maxii - 1.0F;

  double range = maxii - minii;

  printf("Hist: ");
  for (uint i = 0; i < hist.size(); i++)
    {
      if (hist[i] != 0)
        printf("%i: %0.2f ", i, hist[i]);
      int t = abs(h - int((hist[i] - minii) / range * double(h)));

      // if we have at least 1 pixel worth to draw
      if (t < h-1)
        {
          for (int j = 0; j < dw; j++)
            drawLine(res,
                     Point2D<int>(dw * i + j, t),
                     Point2D<int>(dw * i + j, h - 1),
                     byte(255));
          //drawRect(res, Rectangle::tlbrI(t,dw*i,h-1,dw*i+dw-1), byte(255));
        }
    }
  printf("\n");
  return res;
}


#define ORIENTARRAY 36
void calculateOrientationVector(const float x, const float y, const float s,
                const Image<float>& gradmag, const Image<float>& gradorie, Histogram& OV) {


        // compute the effective blurring sigma corresponding to the
        // fractional scale s:
        const float sigma = s;

        const float sig = 1.5F * sigma, inv2sig2 = - 0.5F / (sig * sig);
        const int dimX = gradmag.getWidth(), dimY = gradmag.getHeight();

        const int xi = int(x + 0.5f);
        const int yi = int(y + 0.5f);

        const int rad = int(3.0F * sig);
        const int rad2 = rad * rad;


        // search bounds:
        int starty = yi - rad; if (starty < 0) starty = 0;
        int stopy = yi + rad; if (stopy >= dimY) stopy = dimY-1;

        // 1. Calculate orientation vector
        for (int ind_y = starty; ind_y <= stopy; ind_y ++)
        {
                // given that y, get the corresponding range of x values that
                // lie within the disk (and remain within the image):
                const int yoff = ind_y - yi;
                const int bound = int(sqrtf(float(rad2 - yoff*yoff)) + 0.5F);
                int startx = xi - bound; if (startx < 0) startx = 0;
                int stopx = xi + bound; if (stopx >= dimX) stopx = dimX-1;

                for (int ind_x = startx; ind_x <= stopx; ind_x ++)
                {
                        const float dx = float(ind_x) - x, dy = float(ind_y) - y;
                        const float distSq = dx * dx + dy * dy;

                        // get gradient:
                        const float gradVal = gradmag.getVal(ind_x, ind_y);

                        // compute the gaussian weight for this histogram entry:
                        const float gaussianWeight = expf(distSq * inv2sig2);

                        // add this orientation to the histogram
                        // [-pi ; pi] -> [0 ; 2pi]
                        float angle = gradorie.getVal(ind_x, ind_y) + M_PI;

                        // [0 ; 2pi] -> [0 ; 36]
                        angle = 0.5F * angle * ORIENTARRAY / M_PI;
                        while (angle < 0.0F) angle += ORIENTARRAY;
                        while (angle >= ORIENTARRAY) angle -= ORIENTARRAY;

                        OV.addValueInterp(angle, gaussianWeight * gradVal);
                }
        }


        // smooth the orientation histogram 3 times:
        for (int i = 0; i < 3; i++) OV.smooth();
}

// ######################################################################


uint createVectorsAndKeypoints(const float x, const float y, const float s,
    const Image<float>& gradmag, const Image<float>& gradorie, Histogram& OV)
{

  const float sigma = s; //itsSigma * powf(2.0F, s / float(itsLumBlur.size() - 3));

  // find the max in the histogram:
  float maxPeakValue = OV.findMax();
  LINFO("Max peak %f", maxPeakValue);

  const int xi = int(x + 0.5f);
  const int yi = int(y + 0.5f);

  uint numkp = 0;

  // 2. Create feature vector and keypoint for each significant
  // orientation peak:
  for (int bin = 0; bin < ORIENTARRAY; bin++)
  {
    LINFO("Looking for peaks");
    // consider the peak centered around 'bin':
    const float midval = OV.getValue(bin);

    // if current value much smaller than global peak, forget it:
    if (midval < 0.8F * maxPeakValue) continue;
    LINFO("Within 80 of  maximum");

    // get value to the left of current value
    const float leftval = OV.getValue((bin == 0) ? ORIENTARRAY-1 : bin-1);

    // get value to the right of current value
    const float rightval = OV.getValue((bin == ORIENTARRAY-1) ? 0 : bin+1);
    LINFO("%f %f %f", leftval, midval, rightval);

    // only consider local peaks:
    if (leftval > midval) continue;
    if (rightval > midval) continue;
    LINFO("Local Peak");

    // interpolate the values to get the orientation of the peak:
    //  with f(x) = ax^2 + bx + c
    //   f(-1) = x0 = leftval
    //   f( 0) = x1 = midval
    //   f(+1) = x2 = rightval
    //  => a = (x0+x2)/2 - x1
    //     b = (x2-x0)/2
    //     c = x1
    // f'(x) = 0 => x = -b/2a
    const float a  = 0.5f * (leftval + rightval) - midval;
    const float b  = 0.5f * (rightval - leftval);
    float realangle = float(bin) - 0.5F * b / a;

    realangle *= 2.0F * M_PI / ORIENTARRAY; // [0:36] to [0:2pi]
    realangle -= M_PI;                      // [0:2pi] to [-pi:pi]

    // ############ Create keypoint:

    // compute the feature vector:
    FeatureVector fv;

    const float sinAngle = sin(realangle), cosAngle = cos(realangle);

    // check this scale
    const int radius = int(5.0F * sigma + 0.5F); // NOTE: Lowe uses radius=8?
    const float gausssig = float(radius); // 1/2 width of descript window
    const float gaussfac = - 0.5F / (gausssig * gausssig);


    // Scan a window of diameter 2*radius+1 around the point of
    // interest, and we will cumulate local samples into a 4x4 grid
    // of bins, with interpolation. NOTE: rx and ry loop over a
    // square that is assumed centered around the point of interest
    // and rotated to the gradient orientation (realangle):

    int scale = abs(int(s));
    scale = scale > 5 ? 5 : scale;

    for (int ry = -radius; ry <= radius; ry++)
      for (int rx = -radius; rx <= radius; rx++)
      {
        // rotate the point:
        const float newX = rx * cosAngle - ry * sinAngle;
        const float newY = rx * sinAngle + ry * cosAngle;

        // get the coords in the image frame of reference:
        const float orgX = newX + float(xi);
        const float orgY = newY + float(yi);

        // if outside the image, forget it:
        if (gradmag.coordsOk(orgX, orgY) == false) continue;

        // find the fractional coords of the corresponding bin
        // (we subdivide our window into a 4x4 grid of bins):
        const float xf = 2.0F + 2.0F * float(rx) / float(radius);
        const float yf = 2.0F + 2.0F * float(ry) / float(radius);


        // find the Gaussian weight from distance to center and
        // get weighted gradient magnitude:
        const float gaussFactor = expf((newX*newX+newY*newY) * gaussfac);
        const float weightedMagnitude =
          gaussFactor * gradmag.getValInterp(orgX, orgY);

        // get the gradient orientation relative to the keypoint
        // orientation and scale it for 8 orientation bins:
        float gradAng = gradorie.getValInterp(orgX, orgY) - realangle;

        gradAng=fmod(gradAng, 2*M_PI); //bring the range from 0 to M_PI

        //convert from -M_PI to M_PI
        if (gradAng < 0.0) gradAng+=2*M_PI; //convert to -M_PI to M_PI
        if (gradAng >= M_PI) gradAng-=2*M_PI;
        //split to eight bins
        const float orient = (gradAng + M_PI) * 8 / (2 * M_PI);

        /*
        //reflect the angle to convert from 0 to M_PI
        if (gradAng >= M_PI) gradAng-=M_PI;
        //split to four bins
        const float orient = (gradAng + M_PI) * 4 / (2 * M_PI);
         */

        // will be interpolated into 2 x 2 x 2 bins:
        LINFO("%f %f %f %f", xf, yf, orient, weightedMagnitude);

        fv.addValue(xf, yf, orient, weightedMagnitude);

      }

    // normalize, clamp, scale and convert to byte:
    std::vector<byte> oriVec;
    fv.toByteKey(oriVec);
    //The key point

    ++ numkp;

  }
  return numkp;
}


/*double ObjRec::evalLikelihood(const Image<PixRGB<byte> > &worldImg, const Point2D<int> &pos, double angle, double length)
{
  Image<float> mag, ori;

  Image<float> worldLum = luminance(worldImg);
  gradientSobel(worldLum, mag, ori, 3);

  SHOWIMG(mag);
  SHOWIMG(ori);
  Histogram OV(36);

  // 1. Calculate main orientation vector
  calculateOrientationVector(pos.i, pos.j, 1, mag, ori, OV);

  Image<byte> histImg = OV.getHistogramImage(256, 256, 0, -1);
  SHOWIMG((Image<float>)histImg);

  // 2. Create feature vector and keypoint for each significant
  // orientation peak:
  createVectorsAndKeypoints(pos.i, pos.j, 1, mag, ori, OV);

  return 0;
}*/

/*
double ObjRec::evalLikelihood(const Image<PixRGB<byte> > &worldImg, const Point2D<int> &pos, double angle, double length)
{
  Image<float> mag, ori;

  Image<float> worldLum = luminance(worldImg);
  gradientSobel(worldLum, mag, ori, 3);

  SHOWIMG(mag);
  SHOWIMG(ori);

  Image<float> posOriImg(mag.getWidth(), 180, ZEROS);
  for(int y=0; y<mag.getHeight(); y++)
    for(int x=0; x<mag.getWidth(); x++)
    {
      float angle = ori.getVal(x,y) + M_PI/2;
      while(angle > M_PI) angle-=M_PI;
      while (angle < 0) angle+=M_PI;
      float p = mag.getVal(x,y);
      posOriImg.setVal(x, (int)(angle*180/M_PI), p);
      if (p>0)
        LINFO("%ix%i %f", x,(int)(angle*180/M_PI), p);
    }
  SHOWIMG(posOriImg);



  return 0;
}
*/

void ObjRec::normalizeWorld(WorldState &worldState)
{

  //normalize lines
  double sum = 0;
  for(uint i=0; i<itsWorldState.lines.size(); i++)
    sum += worldState.lines[i].prob;

  for(uint i=0; i<itsWorldState.lines.size(); i++)
     worldState.lines[i].prob /= sum;


}

//double ObjRec::evalLikelihood(const Image<PixRGB<byte> > &worldImg, const Point2D<int> &pos, double angle, double length)
//{
//
//  itsCurrentWorldImg = worldImg;
//  if (itsInitProposal)
//  {
//    initialProposal(worldImg);
//    itsInitProposal = false;
//  }
//
// // Image<PixRGB<byte> > edgeImage = showEdgesWorld(itsWorldState);
// // SHOWIMG(edgeImage);
//
//  //New proposal
//  WorldState worldState = itsWorldState;
//  //generateNewLineState(worldState);
//  for(uint i=0; i<itsWorldState.lines.size(); i++)
//  {
//    worldState.lines[i].pos = pos;
//    worldState.lines[i].ori = angle;
//    worldState.lines[i].length = length;
//    worldState.lines[i].color = PixRGB<byte>(255,0,0);
//    worldState.lines[i].prob = 0.1;
//  }
//  normalizeWorld(worldState);
//  itsPredictWorldImg = showLinesWorld(worldState);
//
//  double newProb =  getLineLikelihood(worldState);
//
//  evalNewLinesWorld(itsWorldState, worldState);
//
//  normalizeWorld(itsWorldState);
//
//  itsPredictWorldImg += showLinesWorld(itsWorldState);
//
//  return newProb;
//
//}

void ObjRec::samplePosterior(const Image<float> &posterior, Point2D<int> &loc, int stop)
{
  loc.i = -1; loc.j = -1;
  for (int i=0; i<stop; i++)
  {
    int xRand = randomUpToNotIncluding(posterior.getWidth());
    int yRand = randomUpToNotIncluding(posterior.getHeight());
    double p = randomDouble();

    //rejection sampling
    if (p < posterior.getVal(xRand,yRand))
    {
      loc.i = xRand;
      loc.j = yRand;
      return;
    }
  }
}

double ObjRec::edgesProb(const Image<PixRGB<byte> > &worldImg)
{

  return edgesLiklyProb(worldImg)*edgesPriorProb();
}

double ObjRec::edgesLiklyProb(const Image<PixRGB<byte> > &worldImg)
{
  Image<float> mag, ori;
  Image<float> worldLum = luminance(worldImg);
  gradientSobel(worldLum, mag, ori, 3);

  itsWorldState.edges.clear();

  for(int y=0; y<mag.getHeight(); y++)
    for(int x=0; x<mag.getWidth(); x++)
    {
      float angle = ori.getVal(x,y) + M_PI/2;
      while(angle > M_PI) angle-=M_PI;
      while (angle < 0) angle+=M_PI;
      float p = 1.0F/(1.0F+10.0F*exp((-1.0F/100.0F)*mag.getVal(x,y)));
      if (p<0.1) p = 0;


      if (p > 0)
      {
        EdgeState es;
        es.pos = Point2D<int>(x,y);
        es.ori = angle*180/M_PI;
        es.color = PixRGB<byte>(0,255,0);
        es.prob = p;
        itsWorldState.edges.push_back(es);
      }
    }

  return 0;
}

double ObjRec::edgesPriorProb()
{

  //uniform prior
  for(uint i=0; i<itsWorldState.edges.size(); i++)
  {
    float edgeProb = itsWorldState.edges[i].prob;
    float edgePrior = 0.5; //uniform over edge=on and edge=off
    itsWorldState.edges[i].prob = edgeProb*edgePrior;
  }

  return 0;
}

void ObjRec::houghLines()
{

  //get the hough voting
  Image<float> posVotes(itsImageDims, ZEROS);

  for(uint i=0; i<itsWorldState.edges.size(); i++)
  {
    //float edgeProb = itsWorldState.edges[i].prob;
    Point2D<int> edgeLoc = itsWorldState.edges[i].pos;

    float prob = posVotes.getVal(edgeLoc) + 1;
    for(int j=edgeLoc.j-3; j<edgeLoc.j+3; j++)
      for(int i=edgeLoc.i-3; i<edgeLoc.i+3; i++)
      {
        if (posVotes.coordsOk(i,j))
        {
          float val = posVotes.getVal(i,j);
          posVotes.setVal(i,j, val+prob);
        }
      }

  }
  inplaceNormalize(posVotes, 0.0F, 1.0F);


  SHOWIMG(posVotes);

  Image<float> sampleSpace(posVotes.getDims(), ZEROS);
  for(int i=0; i<100; i++)
  {
    Point2D<int> pos;
    {
      samplePosterior(posVotes, pos);
      if(sampleSpace.coordsOk(pos))
        sampleSpace.setVal(pos, posVotes.getVal(pos));
    }
  }
  SHOWIMG(sampleSpace);
  Point2D<int> maxPos; float maxVal;
  findMax(posVotes, maxPos, maxVal);
  LINFO("Max at %ix%i %f", maxPos.i, maxPos.j, maxVal);

}


double ObjRec::evalLikelihood(const Image<PixRGB<byte> > &worldImg, const Point2D<int> &pos, double angle, double length)
{
  itsCurrentWorldImg = worldImg;

  double prob = edgesProb(worldImg);

  houghLines();


  return prob;
}


/*double ObjRec::evalLikelihood(const Image<PixRGB<byte> > &worldImg, const Point2D<int> &pos, double angle, double length)
{
  itsCurrentWorldImg = worldImg;


  if (itsInitProposal)
  {
    initialProposal(worldImg);
    itsInitProposal = false;
  }

  WorldState worldState = itsWorldState;
  generateNewLineState(worldState);
  generateNewSquareState(worldState);
  //for(uint i=0; i<itsWorldState.lines.size(); i++)
  //{
  //  worldState.lines[i].pos = pos;
  //  worldState.lines[i].ori = angle;
  //  worldState.lines[i].length = length;
  //  worldState.lines[i].color = PixRGB<byte>(255,0,0);
  //  worldState.lines[i].prob = 0.1;
  //}

  //for(uint i=0; i<itsWorldState.squares.size(); i++)
  //{
  //  worldState.squares[i].pos = pos;
  //  worldState.squares[i].ori = angle;
  //  worldState.squares[i].size = length;
  //  worldState.squares[i].color = PixRGB<byte>(255,0,0);
  //  worldState.squares[i].prob = 0.1;
  //}

 // itsPredictWorldImg = showLinesWorld(worldState);
  itsPredictWorldImg = showSquaresWorld(worldState);
 // SHOWIMG(itsPredictWorldImg);


  double newProb =  getLineLikelihood(worldState);
  getSquareLikelihood(worldState);

  evalNewLinesWorld(itsWorldState, worldState);
  evalNewSquaresWorld(itsWorldState, worldState);

 // normalizeWorld(itsWorldState);

  itsPredictWorldImg += showLinesWorld(itsWorldState);
  itsPredictWorldImg += showSquaresWorld(itsWorldState);

  return newProb;

}*/


void ObjRec::train(const Image<PixRGB<byte> > &img, const std::string label)
{



}

std::string ObjRec::test(const Image<PixRGB<byte> > &img)
{


  return std::string("Test");
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
