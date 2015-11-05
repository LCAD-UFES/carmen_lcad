/*!@file Transport/World3DInput.C Simple 3D world */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/World3DInput.C $
// $Id: World3DInput.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Transport/World3DInput.H"
#include "Component/OptionManager.H"
#include "Image/DrawOps.H"
#include "Image/MatrixOps.H"
#include "Image/Transforms.H"
#include "Image/ColorOps.H"
#include "Raster/GenericFrame.H"
#include <unistd.h>

// ######################################################################
World3DInput::World3DInput(OptionManager& mgr) :
  FrameIstream(mgr, "World3DInput Input", "World3DInputInput"),
  itsImageDims(320,240)
{
  initRandomNumbersZero();

  itsViewPort = new ViewPort3D(320,240);
  double trans[3][4] = {
    {-0.996624, 0.070027, 0.042869, -16.907477},
    {-0.004359, 0.476245, -0.879302, 9.913470},
    {-0.081990, -0.876520, -0.474332, 276.648010}};
  itsViewPort->setCamera(trans);

  itsViewPort->initFrame(); //TODO: hack, this needs to be called because the lighting changes after the first call

  itsObjects.push_back (
      Object(Object::BOX,
        Point3D<float>(-90,50,15),
        Point3D<float>(0.0,0.00,0),
        Point3D<float>(1.0, 1.0, 1.0),
        Point3D<float>(30,30,30) ));

}

World3DInput::~World3DInput()
{
}

// ######################################################################
void World3DInput::setConfigInfo(const std::string& dimsstring)
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
GenericFrameSpec World3DInput::peekFrameSpec()
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
GenericFrame World3DInput::readFrame()
{

  itsObjects[0].rotation.z += 1;
  if (itsObjects[0].rotation.z > 90)
  {
    itsObjects[0].rotation.z = 0;

    itsObjects[0].pos.x += 10;
    if (itsObjects[0].pos.x > 90)
    {
      itsObjects[0].pos.x = -100;

      itsObjects[0].pos.y -= 10;
      LINFO("Pos %f", itsObjects[0].pos.y);
      if (itsObjects[0].pos.y < -100)
      {
        itsObjects[0].pos.y = 50;
      }
    }
  }

  //itsObjects[0].pos.x = -10.80;
  //itsObjects[0].pos.y = -90.15;
  //itsObjects[0].rotation.z = 52;


  //itsObjects[0].pos.x = 0;
  //itsObjects[0].pos.y = 0;
  //itsObjects[0].rotation.z = 52;

  generateWorld();

  Image<PixRGB<byte> > result = getImage();

  rutz::shared_ptr<ObjectsData> objectsData(new ObjectsData);
  for(uint i=0; i<itsObjects.size(); i++)
    objectsData->objects.push_back(itsObjects[i]);

  usleep(10000);
  GenericFrame frame(result);
  frame.addMetaData(std::string("ObjectsData"), objectsData);
  return frame;
}

// ######################################################################
void World3DInput::setImageDims(const Dims& s)
{
  itsImageDims = s;
}

void World3DInput::drawWheelAndBolts(bool rightSide)
{
  glColor3f(0.5,0.5,0.5);

  //Wheel
  glBegin(GL_TRIANGLE_FAN);
  for(int i=0; i<20; i++)
  {
    float ang = 2*M_PI*i/20;
    glVertex2f(cos(ang)*0.55, sin(ang)*0.55);
  }
  glEnd();

  for(int i=0; i<=4; i++)
  {
    glPushMatrix();
    glRotatef(72.0*i, 0.0, 0.0, 1.0);
    if (rightSide)
      glTranslatef(0.2, 0.0, 0.01);
    else
      glTranslatef(0.2, 0.0, -0.01);
    glColor3f(0.3, 0.3, 0.3);

    //Bolt
    glBegin(GL_TRIANGLE_FAN);
    for(int i=0; i<6; i++)
    {
      float ang = 2*M_PI*i/6;
      glVertex2f(cos(ang)*0.05, sin(ang)*0.05);
    }
    glEnd();

    glPopMatrix();
  }

}


void World3DInput::drawCar()
{
    glColor3f(1.0,0.0,0.0);

    glScalef(20.0,20.0,20.0);
    glRotatef(90.0,1.0,0.0, 0.0);
    glBegin(GL_QUADS);
      // Front Face
      glNormal3f( 0.0, 0.0, 1.0);
      glVertex3f(-2.0, -0.5,  1.0);
      glVertex3f( 2.0, -0.5,  1.0);
      glVertex3f( 2.0,  0.5,  1.0);
      glVertex3f(-2.0,  0.5,  1.0);
      // Back Face
      glNormal3f( 0.0, 0.0,-1.0);
      glVertex3f(-2.0, -0.5, -1.0);
      glVertex3f(-2.0,  0.5, -1.0);
      glVertex3f( 2.0,  0.5, -1.0);
      glVertex3f( 2.0, -0.5, -1.0);
      // Top Face
      glNormal3f( 0.0, 1.0, 0.0);
      glVertex3f(-2.0,  0.5, -1.0);
      glVertex3f(-2.0,  0.5,  1.0);
      glVertex3f( 2.0,  0.5,  1.0);
      glVertex3f( 2.0,  0.5, -1.0);
      // Bottom Face
      glNormal3f( 0.0,-1.0, 0.0);
      glVertex3f(-2.0, -0.5, -1.0);
      glVertex3f( 2.0, -0.5, -1.0);
      glVertex3f( 2.0, -0.5,  1.0);
      glVertex3f(-2.0, -0.5,  1.0);
      // Right face
      glNormal3f( 1.0, 0.0, 0.0);
      glVertex3f( 2.0, -0.5, -1.0);
      glVertex3f( 2.0,  0.5, -1.0);
      glVertex3f( 2.0,  0.5,  1.0);
      glVertex3f( 2.0, -0.5,  1.0);
      // Left Face
      glNormal3f(-2.0, 0.0, 0.0);
      glVertex3f(-2.0, -0.5, -1.0);
      glVertex3f(-2.0, -0.5,  1.0);
      glVertex3f(-2.0,  0.5,  1.0);
      glVertex3f(-2.0,  0.5, -1.0);

      // Cabin
      // Front Face
      glNormal3f( 0.0, 0.0, 1.0);
      glVertex3f(-0.5,  0.5,  1.0);
      glVertex3f( 0.5,  0.5,  1.0);
      glVertex3f( 0.5,  1.0,  1.0);
      glVertex3f(-0.5,  1.0,  1.0);
      // Back Face
      glNormal3f( 0.0, 0.0,-1.0);
      glVertex3f(-0.5,  0.5, -1.0);
      glVertex3f(-0.5,  1.0, -1.0);
      glVertex3f( 0.5,  1.0, -1.0);
      glVertex3f( 0.5,  0.5, -1.0);
      // Top Face
      glNormal3f( 0.0, 1.0, 0.0);
      glVertex3f(-0.5,  1.0, -1.0);
      glVertex3f(-0.5,  1.0,  1.0);
      glVertex3f( 0.5,  1.0,  1.0);
      glVertex3f( 0.5,  1.0, -1.0);

      // Right face
      glNormal3f( 1.0, 0.0, 0.0);
      glVertex3f( 0.5,  0.5, -1.0);
      glVertex3f( 0.5,  1.0, -1.0);
      glVertex3f( 0.5,  1.0,  1.0);
      glVertex3f( 0.5,  0.5,  1.0);
      // Left Face
      glNormal3f(-0.5, 0.0, 0.0);
      glVertex3f(-0.5,  0.5, -1.0);
      glVertex3f(-0.5,  0.5,  1.0);
      glVertex3f(-0.5,  1.0,  1.0);
      glVertex3f(-0.5,  1.0, -1.0);

    glEnd();

    glPushMatrix();
      glTranslatef(-1.0,-0.5,1.01);                    // move to first wheel position
      drawWheelAndBolts(true);
    glPopMatrix();

    glPushMatrix();
      glTranslatef(-1.0,-0.5,-1.01);                   // move to 2nd wheel position
      drawWheelAndBolts(false);
    glPopMatrix();

    glPushMatrix();
      glTranslatef(1.0,-0.5,-1.01);                    // move to 3nd wheel position
      drawWheelAndBolts(false);
    glPopMatrix();

    glPushMatrix();
      glTranslatef(1.0,-0.5,1.01);                     // move to 3nd wheel position
      drawWheelAndBolts(true);
    glPopMatrix();



}


void World3DInput::generateWorld()
{

  itsViewPort->initFrame();

  //drawCar();


  for(uint i=0; i<itsObjects.size(); i++)
  {
    switch (itsObjects[i].type)
    {
      case Object::BOX:
        itsViewPort->drawBox(
            itsObjects[i].pos,
            itsObjects[i].rotation,
            itsObjects[i].params,
            PixRGB<byte>(0,256,0));
        break;
      default:
        break;

    }
  }

  itsWorldImg = flipVertic(itsViewPort->getFrame());

  itsFrame++;
}

Image<PixRGB<byte> > World3DInput::getImage()
{

  return itsWorldImg;

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
