/*!@file Transport/World3DInput.H Simple 2d world  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/World3DInput.H $
// $Id: World3DInput.H 13115 2010-04-01 00:02:24Z lior $
//

#ifndef World3DInput_H_DEFINED
#define World3DInput_H_DEFINED

#include "Transport/FrameIstream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/Point3D.H"
#include "Image/Point2D.H"
#include "GUI/ViewPort3D.H"
#include "Raster/GenericFrame.H"

class World3DInput : public FrameIstream
{
public:


  struct Object
  {
    enum ObjectType {BOX, CYLINDER, SPHERE};
    ObjectType type;
    Point3D<float> pos;
    Point3D<float> rotation;
    Point3D<float> color;
    Point3D<float> params;
    Object() {}

    Object(const ObjectType t,
        const Point3D<float>& p,
        const Point3D<float>& r,
        const Point3D<float>& c,
        const Point3D<float>& s) :
      type(t), pos(p), rotation(r), color(c), params(s)
    {}
  };

  struct ObjectsData : public GenericFrame::MetaData
  {
    std::vector<Object> objects;
  };

  //Constructor
  World3DInput(OptionManager& mgr);
  ~World3DInput();


  //! Override from FrameIstream; just calls setDims()
  virtual void setConfigInfo(const std::string& dimsstring);

  //! Peek at the specifications of the next frame to be read
  virtual GenericFrameSpec peekFrameSpec();

  //! Generate the world
  virtual GenericFrame readFrame();

  //! Specify the dimensions of the image to the world
  void setImageDims(const Dims& s);

  void drawWheelAndBolts(bool rightSide);
  void drawCar();


  Image<PixRGB<byte> > getImage();
  void generateWorld();

private:

  ViewPort3D* itsViewPort;
  std::vector<Object> itsObjects;

  Dims itsImageDims;
  Image<PixRGB<byte> > itsWorldImg;
  Point2D<int> itsCurrentPos;
  int itsFrame;

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
