/*!@file Learn/test-gistRec.C
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-GistRec.C $
// $Id: test-GistRec.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/ColorOps.H"
#include "Image/ShapeOps.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/Transforms.H"
#include "Media/FrameSeries.H"
#include "Raster/Raster.H"
#include "Util/log.H"
#include "Util/MathFunctions.H"
#include "Learn/SOFM.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Media/TestImages.H"

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

Image<PixRGB<byte> > showHist(const std::vector<double> &hist, int loc)
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

  for (uint i = 0; i < hist.size(); i++)
    {
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
  return res;
}

void smoothHist(std::vector<double> &hist)
{
  const uint siz = hist.size();
  float vect[siz];

  for (uint n = 0 ; n < siz ; n++)
  {
    float val0 = hist[ (n-1+siz) % siz ];
    float val1 = hist[ (n  +siz) % siz ];
    float val2 = hist[ (n+1+siz) % siz ];

    vect[n] = 0.25F * (val0 + 2.0F*val1 + val2);
  }

  for (uint n = 0 ; n < siz ; n++) hist[n] = vect[n];
}

void normalizeHist(std::vector<double> &hist, double high, double low)
{

  double oldmin, oldmax;
  findMinMax(hist, oldmin, oldmax);

   float scale = float(oldmax) - float(oldmin);
   //if (fabs(scale) < 1.0e-10F) scale = 1.0; // image is uniform
   const float nscale = (float(high) - float(low)) / scale;

   for(uint i=0; i<hist.size(); i++)
   {
     hist[i] = low + (float(hist[i]) - float(oldmin)) * nscale ;
   }


}

Point2D<int> processOriMap(Image<PixRGB<byte> > &inputImg,
    SOFM &sofm,
    int ii,
    nub::ref<OutputFrameSeries> &ofs)
{

    //get Orientations
    Image<byte> lum = luminance(inputImg);
    Image<float> mag, ori;
    gradientSobel(lum, mag, ori, 3);
    std::vector<double> oriHist(360,0);
    for(int i=0; i<mag.getSize(); i++)
    {
      int deg = int(ori[i]*180/M_PI);
      if (deg < 0) deg+=360;
      oriHist[deg] += mag[i];
    }
    normalizeHist(oriHist, 0.0F, 255.0F);
    smoothHist(oriHist);
    ofs->writeRGB(showHist(oriHist,0), "Ori Hist");
    sofm.setInput(oriHist);
    sofm.propagate();
    Point2D<int> winner = sofm.getWinner();
    LINFO("Winner at %ix%i", winner.i, winner.j);
    Image<float> sofmOut = sofm.getMap();
    inplaceNormalize(sofmOut, 0.0F, 255.0F);

    drawCircle(sofmOut, winner, 6, 255.0F);

    ofs->writeRGB(sofmOut, "OriSOFM_act_map");


 //   sofm.SetLearningRate(ii);
 //   sofm.organize(oriHist);
    //inplaceNormalize(SMap, 0.0F, 255.0F);
    //

    //save the info every 100 triels
//    if (!(ii%100))
//      sofm.WriteNet("oriSofm.net");
//
    return winner;
}

Point2D<int> processRGColMap(Image<PixRGB<byte> > &inputImg,
    SOFM &sofm,
    int ii,
    nub::ref<OutputFrameSeries> &ofs)
{

    Image<float> rg,by;
    getRGBY(inputImg, rg, by, byte(25));
    inplaceNormalize(rg, 0.0F, 255.0F);

    std::vector<double> colHist(256,0);
    for(int i=0; i<rg.getSize(); i++)
    {
      int col = (int)rg[i];
      colHist[col]++;
    }
    normalizeHist(colHist, 0.0F, 255.0F);
    smoothHist(colHist);
    ofs->writeRGB(showHist(colHist,0), "ColRG Hist");
    sofm.setInput(colHist);
    sofm.propagate();
    Point2D<int> winner = sofm.getWinner();
    Image<float> sofmOut = sofm.getMap();
    inplaceNormalize(sofmOut, 0.0F, 255.0F);

    drawCircle(sofmOut, winner, 6, 255.0F);

    ofs->writeRGB(sofmOut, "colRGSOFM_act_map");

    //sofm.SetLearningRate(ii);
    //sofm.organize(colHist);

    //save the info every 100 triels
   // if (!(ii%100))
   //   sofm.WriteNet("colRGSofm.net");

    return winner;
}

Point2D<int> processBYColMap(Image<PixRGB<byte> > &inputImg,
    SOFM &sofm,
    int ii,
    nub::ref<OutputFrameSeries> &ofs)
{

    Image<float> rg,by;
    getRGBY(inputImg, rg, by, byte(25));
    inplaceNormalize(by, 0.0F, 255.0F);

    std::vector<double> colHist(256,0);
    for(int i=0; i<by.getSize(); i++)
    {
      int col = (int)by[i];
      colHist[col]++;
    }
    normalizeHist(colHist, 0.0F, 255.0F);
    smoothHist(colHist);
    ofs->writeRGB(showHist(colHist,0), "ColBY Hist");
    sofm.setInput(colHist);
    sofm.propagate();
    Point2D<int> winner = sofm.getWinner();
    Image<float> sofmOut = sofm.getMap();
    inplaceNormalize(sofmOut, 0.0F, 255.0F);

    drawCircle(sofmOut, winner, 6, 255.0F);

    ofs->writeRGB(sofmOut, "ColBySOFM_act_map");

    //sofm.SetLearningRate(ii);
    //sofm.organize(colHist);

    //save the info every 100 triels
//    if (!(ii%100))
//      sofm.WriteNet("colBYSofm.net");
//
    return winner;
}

Point2D<int> processSceneMap(
    Point2D<int> oriWinner,
    Point2D<int> rgWinner,
    Point2D<int> byWinner,
    SOFM &sofm,
    int ii,
    nub::ref<OutputFrameSeries> &ofs)
{

    std::vector<double> input(6,0);
    input[0] = oriWinner.i;
    input[1] = oriWinner.j;
    input[2] = rgWinner.i;
    input[3] = rgWinner.j;
    input[4] = byWinner.i;
    input[5] = byWinner.j;


    sofm.setInput(input);
    sofm.propagate();
    Point2D<int> winner = sofm.getWinner();
    Image<float> sofmOut = sofm.getMap();
    inplaceNormalize(sofmOut, 0.0F, 255.0F);

    drawCircle(sofmOut, winner, 6, 255.0F);

    ofs->writeRGB(sofmOut, "SceneSOFM_act_map");

    //sofm.SetLearningRate(ii);
    //sofm.organize(input);

    //save the info every 100 triels
  //  if (!(ii%100))
  //    sofm.WriteNet("sceneSofm.net");

    return winner;
}



int main(int argc, char** argv)
{

  MYLOGVERB = LOG_INFO;  // suppress debug messages

  // Instantiate a ModelManager:
  ModelManager manager("Test SOFM");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  // Parse command-line:
  if (manager.parseCommandLine((const int)argc, (const char**)argv, "<path to mages>", 1, 1) == false)
    return(1);

  manager.start();

  TestImages testImages(manager.getExtraArg(0).c_str(),
      TestImages::MIT_LABELME);

#define SIZE 256
  SOFM oriSofm("oriSofm.net", 360, SIZE, SIZE);
  SOFM rgSofm("rgSofm.net", 256, SIZE, SIZE);
  SOFM bySofm("bySofm.net", 256, SIZE, SIZE);
  SOFM sceneSofm("sceneSofm.net", 6, SIZE, SIZE);

  //oriSofm.RandomWeights();
  //rgSofm.RandomWeights();
  //bySofm.RandomWeights();
  //sceneSofm.RandomWeights();

  oriSofm.ReadNet("oriSofm.net");
  rgSofm.ReadNet("colRGSofm.net");
  bySofm.ReadNet("colBYSofm.net");
  sceneSofm.ReadNet("sceneSofm.net");

  initRandomNumbers();


  // main loop:
  int ii=0;

  LINFO("Process input");
  while(1)
  {
    //choose a scene at random
    int scene = randomUpToIncluding(testImages.getNumScenes()-1);

    Image<PixRGB<byte> > inputImg = testImages.getScene(scene);

    ofs->writeRGB(inputImg, "Input");

    Point2D<int> oriWinner = processOriMap(inputImg, oriSofm, ii, ofs);
    LINFO("oriWinner %ix%i", oriWinner.i, oriWinner.j);

    Point2D<int> colRGWinner = processRGColMap(inputImg, rgSofm, ii, ofs);
    LINFO("colRGWinner %ix%i", colRGWinner.i, colRGWinner.j);

    Point2D<int> colBYWinner = processBYColMap(inputImg, bySofm, ii, ofs);
    LINFO("colBYWinner %ix%i", colBYWinner.i, colBYWinner.j);

    Point2D<int> sceneWinner = processSceneMap(oriWinner, colRGWinner, colBYWinner,
        sceneSofm, ii, ofs);


    ii++;

    getchar();

  }

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
