/*!@file AppMedia/test-viewport.C test the opengl viewport */

// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-vobj.C $
// $Id: test-vobj.C 10794 2009-02-08 06:21:09Z itti $


#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ShapeOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/ColorOps.H"
#include "Image/Transforms.H"
#include "Image/MathOps.H"
#include "Image/MatrixOps.H"
#include "Image/Kernels.H"
#include "Image/fancynorm.H"
#include "Image/Layout.H"
#include "Util/log.H"
#include <math.h>
#include <stdlib.h>

struct ObjState
{
  float pos[3];
  float side[3];
  float ori[3];
  float color[3];
  float lik;
};

Image<float> thresh(const Image<float>& src)
{

  Image<float> result = src;
  Image<float>::const_iterator sptr = src.begin();
  Image<float>::iterator dptr = result.beginw();
  int size = src.getSize();

  float min, max;
  getMinMax(src, min, max);

  //treshold
  for (int i = 0; i < size; i ++)
  {
    if (*sptr/max > 0.10)
      *dptr++ = 1.0;
    else
      *dptr++ = 0;
    *sptr++;
  }


  return result;
}

Image<float> edgeDist(const Image<float>& priorMag, const Image<float>& priorOri,
    const Image<float>& dataMag, const Image<float>& dataOri)
{

  Image<float> dataChamfer = chamfer34(thresh(dataMag));
  Image<float> priorThresh = thresh(priorMag);

  return dataChamfer*priorThresh;
}



Image<PixRGB<byte> > drawObject(ObjState &objState)
{
  Image<PixRGB<byte> > objImg(256,256,ZEROS);
  drawPatch(objImg, Point2D<int>((int)objState.pos[0], (int)objState.pos[1]), 40, PixRGB<byte>(255,0,0));

  return objImg;
}

Image<PixRGB<byte> > drawWorld()
{
  Image<PixRGB<byte> > objImg(256,256,ZEROS);

  int x = randomUpToIncluding(256);
  int y = randomUpToIncluding(256);
  LINFO("Object at %ix%i", x, y);
  int size = 40;
  PixRGB<byte> color = PixRGB<byte>(255,0,0);

  drawPatch(objImg, Point2D<int>(x,y), size, color);

  //x = randomUpToIncluding(256);
  //y = randomUpToIncluding(256);
  //LINFO("Object at %ix%i", x, y);

  //drawPatch(objImg, Point2D<int>(x,y), size, color);

  return objImg;
}

Image<float> likelyhood(const Image<PixRGB<byte> > &world, const Image<PixRGB<byte> > &model)
{

  Image<float> worldMag, worldOri;
  Image<byte> worldLum = luminance(world);
  gradientSobel(worldLum, worldMag, worldOri, 3);

  Image<float> modelMag, modelOri;
  Image<byte> modelLum = luminance(model);
  gradientSobel(modelLum, modelMag, modelOri, 3);

  //Image<float> diffImg = edgeDist(worldMag, worldOri, modelMag, modelOri);
  Image<float> diffImg = edgeDist(modelMag, modelOri, worldMag, worldOri);

  return diffImg;

}

void normalize(std::vector<ObjState> &particles)
{

  float sum = 0;
  for (uint i=0; i<particles.size(); i++)
    sum += particles[i].lik;

  for (uint i=0; i<particles.size(); i++)
     particles[i].lik /= sum;
}

int resample(std::vector<ObjState> &particles)
{

  float max = particles[0].lik;
  int maxi = 0;
  for (uint i=1; i<particles.size(); i++)
  {
    if (particles[i].lik > max)
    {
      maxi = i;
      max = particles[i].lik;
    }

  }

  return maxi;

}

Image<float> getLikImg(std::vector<ObjState> &particles)
{

  Image<float> retImg(256,256,ZEROS);

  for (uint i=0; i<particles.size(); i++)
    retImg.setVal((int)particles[i].pos[0], (int)particles[i].pos[1],particles[i].lik);

  inplaceNormalize(retImg, 0.0F, 255.0F);

  return retImg;
}


int main(const int argc, const char **argv)
{
  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test ObjRec");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*mgr));
  mgr->addSubComponent(ofs);

  mgr->exportOptions(MC_RECURSE);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "", 0, 0) == false)
    return 1;
  mgr->start();


  int run = 1;
  Image<PixRGB<byte> > worldImg = drawWorld();

  //generate particles as uniform dist
  std::vector<ObjState> particles(100);
  for(uint i=0; i<particles.size(); i++)
  {
    particles[i].pos[0] = randomUpToIncluding(255);
    particles[i].pos[1] = randomUpToIncluding(255);
    particles[i].lik = 1/(float)particles.size();
  }
  while(run)
  {

    LINFO("Move particles");
    for(uint i=0; i<particles.size(); i++)
    {
      particles[i].pos[0] += randomUpToIncluding(6)-3;
      particles[i].pos[1] += randomUpToIncluding(6)-3;

      //apply prior
      if (particles[i].pos[0] < 0) particles[i].pos[0] = 0;
      if (particles[i].pos[0] > 255) particles[i].pos[0] = 255;

      if (particles[i].pos[1] < 0) particles[i].pos[1] = 0;
      if (particles[i].pos[1] > 255) particles[i].pos[1] = 255;
    }


    LINFO("Evaluate importance weight");
    //Evaluate each particle
    for(uint i=0; i<particles.size(); i++)
    {
      Image<PixRGB<byte> > modelImg = drawObject(particles[i]);
      Image<float> diffImg = likelyhood(worldImg, modelImg);
      float diff = sum(diffImg);
      float lik = 1/diff;
      particles[i].lik *= lik;
    }

    //normalize the weight
    normalize(particles);


    float sum = 0;
    for(uint i=0; i<particles.size(); i++)
      sum += (particles[i].lik)*(particles[i].lik);

    LINFO("Effective N: %f ", 1/sum);
    for(uint i=0; i<particles.size(); i++)
      printf("%0.2f ", particles[i].lik);
    printf("\n");

    //selection step
    LINFO("resample particles");
    int maxParticle = resample(particles);

    LINFO("Show results");
    Image<float> likImg = getLikImg(particles);

    //Show the most probable object

    Image<PixRGB<byte> > probImg = drawObject(particles[maxParticle]);

    Layout<PixRGB<byte> > outDisp;
    outDisp = vcat(outDisp, hcat(worldImg, toRGB(Image<byte>(likImg))));
    outDisp = vcat(outDisp, hcat(worldImg, probImg));

    ofs->writeRgbLayout(outDisp, "Result", FrameInfo("Result", SRC_POS));
    getchar();
  }

  //while(run){


  //  Layout<PixRGB<byte> > outDisp;
  //  XEvent event = vp.initFrame();
  //  handleEvents(event, objState);

  //  drawObject(vp, objState);

  //  run = vp.updateFrame();
  //  Image<PixRGB<byte> > worldImg = flipVertic(vp.getFrame());
  //  inputImg = rescale(input.asRgb(), worldImg.getDims());

  //  Image<float> distmap;
  //  Image<byte> foamask = Image<byte>();

  //  Point2D<int> foa(145,65);
  //  Rectangle foaRec = segmentColor(inputImg, foa,
  //      0, 0.1, 10, 200, distmap, foamask);
  //  drawRect(inputImg, foaRec, PixRGB<byte>(255,0,0));


  //  //World
  //  Image<float> priorMag, priorOri;
  //  Image<byte> lum = luminance(worldImg);
  //  gradientSobel(lum, priorMag, priorOri, 3);

  //  outDisp = vcat(outDisp, hcat(worldImg, toRGB(Image<byte>(priorMag))));

  //  //Real
  //  lum = luminance(inputImg);
  //  Image<float> dataMag, dataOri;
  //  gradientSobel(lum, dataMag, dataOri, 3);
  //  dataMag = dataMag * foamask;

  //  outDisp = vcat(outDisp, hcat(inputImg, toRGB(Image<byte>(dataMag))));

  //  Image<float> comb = priorMag + priorMag;

  //  Image<float> diffImg = edgeDist(dataMag, dataOri, priorMag, priorOri);
  //      //dataMag, dataOri);
  //  LINFO("Diff %f \n", sum(diffImg));


  //  outDisp = vcat(outDisp, hcat(toRGB(Image<byte>(comb)), toRGB(Image<byte>(diffImg))));

  //  ofs->writeRgbLayout(outDisp, "Edges", FrameInfo("Edges", SRC_POS));
  //  usleep(1000);
  //}

  exit(0);

}


