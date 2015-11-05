/*!@file Transport/World2DInput.H Simple 2d world  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/World2DInput.H $
// $Id: World2DInput.H 9952 2008-07-21 23:48:53Z lior $
//

#ifndef World2DInput_H_DEFINED
#define World2DInput_H_DEFINED

#include "Transport/FrameIstream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"

class World2DInput : public FrameIstream
{
public:
  //Constructor
  World2DInput(OptionManager& mgr);
  ~World2DInput();


  //! Override from FrameIstream; just calls setDims()
  virtual void setConfigInfo(const std::string& dimsstring);

  //! Peek at the specifications of the next frame to be read
  virtual GenericFrameSpec peekFrameSpec();

  //! Generate the world
  virtual GenericFrame readFrame();

  //! Specify the dimensions of the image to the world
  void setImageDims(const Dims& s);


  Image<PixRGB<byte> > getImage();
  void generateWorld();
  double  getLineOri();
  double  getLineLength();
  PixRGB<byte> getLineColor();
  void placeLine(Image<PixRGB<byte> > &img);
  void placeSquare(Image<PixRGB<byte> > &img);

private:

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
