/*!@file Surprise/runRemoveSurprise.C attempt to remove surprise from image */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/runRemoveSurprise.C $
// $Id: runRemoveSurprise.C 14376 2011-01-11 02:44:34Z pez $
//

#ifndef RUN_REMOVE_SURPRISE_C_DEFINED
#define RUN_REMOVE_SURPRISE_C_DEFINED

#include "Surprise/ScaleRemoveSurprise.H"
#include "GUI/XWindow.H"
#include "Raster/Raster.H"
#include "Image/Normalize.H"

#define USE_SCALES true

using std::string;

int main(const int argc, const char **argv)
{
  if(argc < 2)
    LFATAL("Usage: runRemoveSurprise frames rawImagePrefix");

  uint frames           = atoi(argv[1]);
  string rawImagePrefix = argv[2];
  string confName;
  if(argc >= 2)
  {
    confName = argv[3];
  }
  else
  {
    confName = "null";
  }

  Image<float> salmap;
  Image<float> COmap;
  Image<float> MOmap;
  Image<float> ORmap;
  Image<float> INmap;
  Image<byte>  tbyte;

  Image<PixRGB<float> > rawImage;
  Image<PixRGB<float> > outImage;
  Image<PixRGB<float> > diffImage;
  Image<PixRGB<byte> > tpbyte;

  string start = "000000.png";
  string raw   = rawImagePrefix + start;

  tpbyte = Raster::ReadRGB(raw);


  if(!USE_SCALES)
  {
    RemoveSurprise<PixH2SV1<float>,PixHyper<float,4>,float>
      rs(tpbyte.getWidth(),tpbyte.getHeight());
    rs.RSsetAxisBias(0.5F,0.5F,1.0F);
    rs.RScreateSepFilters(3,3,4.0);
    rs.RSfindConvolutionEndPoints();
    rs.RSsetConspicBias(1.5F, 1.5F, 1.5F, 0.0F);
    rs.RSuseTrueKalman(false);
    rs.RSsetLambda(0.80F);
    // set INmap to all 1's if we do not have it

    INmap.resize(tpbyte.getWidth(),tpbyte.getHeight());
    for(int i = 0; i < INmap.getWidth(); i++)
    {
      for(int j = 0; j < INmap.getHeight(); j++)
      {
        INmap.setVal(i,j,1.0F);
      }
    }
    LINFO("RUNNING FRAMES");
    for(uint i = 0; i < frames; i++)
    {
      char c[100];
      string a, Myname;
      string b = ".png";
      string p = ".";
      const uint frameNumber = i;


      if(frameNumber < 10)
        sprintf(c,"00000%d",frameNumber);
      else if(frameNumber < 100)
        sprintf(c,"0000%d",frameNumber);
      else if(frameNumber < 1000)
        sprintf(c,"000%d",frameNumber);
      else if(frameNumber < 10000)
        sprintf(c,"00%d",frameNumber);
      else if(frameNumber < 100000)
        sprintf(c,"0%d",frameNumber);
      else
        sprintf(c,"%d",frameNumber);

      // read raw image
      raw  = rawImagePrefix + c + b;
      tpbyte = Raster::ReadRGB(raw);
      rawImage = tpbyte;
      rs.RSinputRawImage(rawImage,i);

      // read salmap
      string d = ".pnm";
      b        = "#.ppm-SM";
      raw      = rawImagePrefix + b + c + d;
      tbyte    = Raster::ReadGray(raw);
      salmap   = tbyte;
      rs.RSinputSalMap(salmap);

      // read CO consp map
      b        = "#.ppm-COcolor-";
      raw      = rawImagePrefix + b + c + d;
      tbyte    = Raster::ReadGray(raw);
      COmap    = tbyte;
      rs.RSinputConspicCO(COmap);

      // read MO consp map
      b        = "#.ppm-COmotion-";
      raw      = rawImagePrefix + b + c + d;
      tbyte    = Raster::ReadGray(raw);
      MOmap    = tbyte;
      rs.RSinputConspicMO(MOmap);

      // read OR consp map
      b        = "#.ppm-COorientation-";
      raw      = rawImagePrefix + b + c + d;
      tbyte    = Raster::ReadGray(raw);
      ORmap    = tbyte;
      rs.RSinputConspicOR(ORmap);

      rs.RSinputConspicIN(INmap);

      // process this frame
      rs.RSprocessFrameSeperable();

      // get processed frame
      outImage = rs.RSgetFrame();
      tpbyte   = outImage;
      b        = ".ReduceSup2.";
      raw      =  rawImagePrefix + b + c + d;
      Raster::WriteRGB(tpbyte,raw);
    }
  }
  else
  {
    ScaleRemoveSurprise<float>
      srs(tpbyte.getWidth(),tpbyte.getHeight(),confName);

    // input set of weights from an image (or set?) as anit removal
    //srs.SRSsetAntiWeights();
    //srs.SRSsetAntiWeightsInteract(10,10);
    string bb              = "#.png";
    string basepre         = "org.";
    string antipre         = "plane.";
    string antiFileSetName = antipre + bb;
    string baseFileSetName = basepre + bb;
    string fileSetName     = rawImagePrefix + bb;
    //srs.SRScomputeBayesFeatureBias(10,baseFileSetName, antiFileSetName);
    //srs.SRSopenBayesFeatureBias(baseFileSetName, antiFileSetName);
    diffImage.resize(tpbyte.getWidth(),tpbyte.getHeight());
    LINFO("RUNNING FRAMES");
    for(uint i = 0; i < frames; i++)
    {
      char c[100];
      string a, Myname;
      string b = ".png";
      string p = ".";
      const uint frameNumber = i;

      if(frameNumber < 10)
        sprintf(c,"00000%d",frameNumber);
      else if(frameNumber < 100)
        sprintf(c,"0000%d",frameNumber);
      else if(frameNumber < 1000)
        sprintf(c,"000%d",frameNumber);
      else if(frameNumber < 10000)
        sprintf(c,"00%d",frameNumber);
      else if(frameNumber < 100000)
        sprintf(c,"0%d",frameNumber);
      else
        sprintf(c,"%d",frameNumber);

      // read raw image
      raw  = rawImagePrefix + c + b;
      tpbyte = Raster::ReadRGB(raw);
      rawImage = tpbyte;
      srs.SRSinputRawImage(rawImage,i);

      // find bayesian maps for current image
      //srs.SRScomputeBayesFeatureCurrent(i,fileSetName);

      // read salmap
      string d = ".pnm";
      b        = "#.png-SM";
      raw      = rawImagePrefix + b + c + d;
      tbyte    = Raster::ReadGray(raw);
      salmap   = tbyte;
      srs.SRSinputSalMap(salmap);


      // process this frame
      srs.SRSprocessFrame();

      d = ".png";

      // get processed frame
      outImage    = srs.SRSgetFrame();
      tpbyte      = outImage;
      b           = ".ReduceSupScale.";
      raw         =  rawImagePrefix + b + c + d;
      Raster::WriteRGB(tpbyte,raw);

      diffImage   = srs.SRSgetDiffImage();
      tpbyte      = diffImage;
      b           = ".ReduceSupScale.diff.";
      raw         = rawImagePrefix + b + c + d;
      Raster::WriteRGB(tpbyte,raw);
      /*
      std::vector<Image<PixRGB<float> > > diffParts = srs.SRSgetDiffParts();
      b           = ".ReduceSupScale.diffParts.";
      for(uint i = 2; i < diffParts.size(); i++)
      {
        char part[100];
        sprintf(part,"%d",i);
        string spart = part;
        raw          = rawImagePrefix + b + c + p + spart + d;
        Image<PixRGB<byte> > bimage = diffParts[i];
        Raster::WriteRGB(bimage,raw);
      }

      std::vector<Image<float> > betaParts = srs.SRSgetBetaParts();
      b           = ".ReduceSupScale.betaParts.";
      for(uint i = 0; i < betaParts.size(); i++)
      {
        char part[100];
        sprintf(part,"%d",i);
        string spart = part;
        raw          = rawImagePrefix + b + c + p + spart + d;
        betaParts[i] = normalizeFloat(betaParts[i],FLOAT_NORM_0_255);
        Image<byte> bimage = betaParts[i];
        Raster::WriteGray(bimage,raw);
      }
      */
    }
  }
  return 0;
}

#endif
