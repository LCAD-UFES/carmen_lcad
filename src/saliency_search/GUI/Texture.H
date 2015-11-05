/*!@file GUI/Texture.H test opengl viewport */

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
// Originally obtained from ode
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/Texture.H $
// $Id: Texture.H 12951 2010-03-05 19:06:12Z lior $

#ifndef Texture_H
#define Texture_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <stdio.h>
#include "Util/log.H"

#ifdef INVT_HAVE_LIBGLUT
#include <GL/glu.h>
#endif

//***************************************************************************
// PPM image object

typedef unsigned char byte;

class VPImage {
  int image_width,image_height;
  byte *image_data;
public:
  VPImage (const char *filename);
  // load from PPM file
  ~VPImage();
  int width() { return image_width; }
  int height() { return image_height; }
  byte *data() { return image_data; }
  void skipWhiteSpace (const char *filename, FILE *f);
  int readNumber (const char *filename, FILE *f);
  bool itsInitialized;

};

//***************************************************************************
// Texture object.

class Texture {
  VPImage *image;
  GLuint name;
public:
  Texture (const char *filename);
  ~Texture();
  void bind (int modulate);
  bool initialized() { return itsInitialized; }
  GLuint getName() { return name; }
  bool itsInitialized;
};

#endif

