/*!@file Transport/World2DInput.C Simple 2D world */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/World2DInput.C $
// $Id: World2DInput.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Transport/World2DInput.H"
#include "Component/OptionManager.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"
#include "Raster/GenericFrame.H"
#include <unistd.h>

// ######################################################################
World2DInput::World2DInput(OptionManager& mgr) :
  FrameIstream(mgr, "World2DInput Input", "World2DInputInput"),
  itsImageDims(320,240)
{
  initRandomNumbers();

  itsCurrentPos.i = 0;
  itsCurrentPos.j = 128;
  itsFrame = 0;

}

World2DInput::~World2DInput()
{
}

// ######################################################################
void World2DInput::setConfigInfo(const std::string& dimsstring)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  if (dimsstring.size() == 0)
    return;

  Dims d; convertFromString(dimsstring, d);
  this->setImageDims(d);
}

// ######################################################################
GenericFrameSpec World2DInput::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = itsImageDims;
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame World2DInput::readFrame()
{
  generateWorld();

  Image<PixRGB<byte> > result = getImage();
  usleep(10000);
  return GenericFrame(result);
}

// ######################################################################
void World2DInput::setImageDims(const Dims& s)
{
  itsImageDims = s;
}



void World2DInput::generateWorld()
{

  itsWorldImg = Image<PixRGB<byte> >(itsImageDims,ZEROS);
  itsWorldImg.clear(PixRGB<byte>(128,128,128));


 // float p = 128+128*sin((float)itsFrame*0.05);
 // drawDisk(itsWorldImg, Point2D<int>((int)p,50), 10, PixRGB<byte>(0,155,100));

 // p = 128+128*sin((float)itsFrame*0.01);
 // drawDisk(itsWorldImg, Point2D<int>((int)p,128), 30, PixRGB<byte>(0,255,100));

 // p = 200+50*sin((float)itsFrame*0.008);
 // drawDisk(itsWorldImg, Point2D<int>(50, (int)p), 15, PixRGB<byte>(255,255,100));

  //static int col = 240; //itsFrame;
  //drawFilledRect(itsWorldImg,
  //    Rectangle(Point2D<int>(col,128), Dims(50,50)),
  //    PixRGB<byte>(col,col,col));
  //col  = (col +1)%255;

  //drawSuperquadric(itsWorldImg, Point2D<int>(320/2, 240/2),
  //    50, 50, 0.5, PixRGB<byte>(0,255,0));


  //Draw overlapping rectangles for amodal completion
  Dims size(100,100);
  drawFilledRect(itsWorldImg,
      Rectangle(Point2D<int>((320/2)-(size.w()/2),
          (240/2)-(size.h()/2)), size),
      PixRGB<byte>(0,0,0));

  drawFilledRect(itsWorldImg,
      Rectangle(Point2D<int>((320/2)-(size.w()/2)-50,(240/2)-(size.h()/2)-50),
        size),
      PixRGB<byte>(255,255,255));

  //placeLine(itsWorldImg);

  //placeSquare(itsWorldImg);
 // placeLine(itsWorldImg);

 // placeSquare(itsWorldImg);

 // inplaceColorSpeckleNoise(itsWorldImg, (int)(itsWorldImg.size()*0.70));
  itsFrame++;
}

double World2DInput::getLineOri()
{

  double ori;
  double horizStd = 15; //the probability of horizantal edges in the world
  double vertStd = 15; //the probability of vertical edges in the world

  double u = randomDouble();

  int idum = getIdum();
  if (u > 0.5)
    ori = 0 + horizStd*gasdev(idum);
  else
    ori = 90 + vertStd*gasdev(idum);

  if (ori < 0)
    ori += 180;

  return ori;
}

double World2DInput::getLineLength()
{

  double length = 10 + randomUpToNotIncluding(itsImageDims.w()/10);

  return length;
}

PixRGB<byte> World2DInput::getLineColor()
{
  return PixRGB<byte>(255,0,0);
}


void World2DInput::placeLine(Image<PixRGB<byte> > &img)
{

  //The line parameters
  double ori = 45; //getLineOri();
  double length = 50; //getLineLength();
  PixRGB<byte> col = getLineColor();
  int rad = 1;
  //Point2D<int> pos(randomUpToNotIncluding(itsImageDims.w()),
  //    randomUpToNotIncluding(itsImageDims.h()));
  Point2D<int> pos(150,100);

  LINFO("Line param ori=%f, length=%f, pos=%i,%i",
      ori, length, pos.i, pos.j);
  //getchar();
  //draw the line
  int x1 = int(cos(ori*M_PI/180)*length);
  int y1 = int(sin(ori*M_PI/180)*length);

  Point2D<int> p1 = pos;
  Point2D<int> p2(pos.i+x1, pos.j-y1);

  drawLine(img, p1, p2, col, rad);

}

void World2DInput::placeSquare(Image<PixRGB<byte> > &img)
{

  //The line parameters
  double ori = getLineOri();
  double size = getLineLength();
  PixRGB<byte> col = getLineColor();
  int rad = 1;
  Point2D<int> pos(randomUpToNotIncluding(itsImageDims.w()),
      randomUpToNotIncluding(itsImageDims.h()));
// Point2D<int> pos(100,100);

  LINFO("Square param ori=%f, size=%f, pos=%i,%i",
      ori, size, pos.i, pos.j);
  //getchar();
  //draw the line
  drawRectOR(img, Rectangle(pos, Dims((int)size,(int)size)), col, rad, ori);

}

Image<PixRGB<byte> > World2DInput::getImage()
{

  return itsWorldImg;

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
