/*!@file GUI/Texture.C test opengl viewport */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/Texture.C $
// $Id: Texture.C 12951 2010-03-05 19:06:12Z lior $

#include "GUI/Texture.H"

Texture::Texture (const char *filename)
{
  image = new VPImage (filename);
  itsInitialized = false;
  if (image->itsInitialized)
  {
    glGenTextures (1,&name);
    glBindTexture (GL_TEXTURE_2D,name);

    // set pixel unpacking mode
    glPixelStorei (GL_UNPACK_SWAP_BYTES, 0);
    glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei (GL_UNPACK_SKIP_ROWS, 0);
    glPixelStorei (GL_UNPACK_SKIP_PIXELS, 0);

    // glTexImage2D (GL_TEXTURE_2D, 0, 3, image->width(), image->height(), 0,
    //                   GL_RGB, GL_UNSIGNED_BYTE, image->data());
#ifdef INVT_HAVE_LIBGLUT
    gluBuild2DMipmaps (GL_TEXTURE_2D, 3, image->width(), image->height(),
        GL_RGB, GL_UNSIGNED_BYTE, image->data());
#endif

    // set texture parameters - will these also be bound to the texture???
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
        GL_LINEAR_MIPMAP_LINEAR);

    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    itsInitialized = true;
  }
}

Texture::~Texture()
{
  delete image;
  glDeleteTextures (1,&name);
}


void Texture::bind (int modulate)
{
  glBindTexture (GL_TEXTURE_2D,name);
  glTexEnvi (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,
             modulate ? GL_MODULATE : GL_DECAL);
}

VPImage::VPImage (const char *filename)
{
  itsInitialized = false;
  FILE *f = fopen (filename,"rb");
  if (!f)
  {
    LINFO ("Can't open image file `%s'",filename);
    return;
  }

  // read in header
  if (fgetc(f) != 'P' || fgetc(f) != '6')
  {
    LINFO ("image file \"%s\" is not a binary PPM (no P6 header)",filename);
    return;
  }
  skipWhiteSpace (filename,f);

  // read in image parameters
  image_width = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  image_height = readNumber (filename,f);
  skipWhiteSpace (filename,f);
  int max_value = readNumber (filename,f);

  // check values
  if (image_width < 1 || image_height < 1)
    LFATAL ("bad image file \"%s\"",filename);
  if (max_value != 255)
    LFATAL ("image file \"%s\" must have color range of 255",filename);

  // read either nothing, LF (10), or CR,LF (13,10)
  int c = fgetc(f);
  if (c == 10) {
    // LF
  }
  else if (c == 13) {
    // CR
    c = fgetc(f);
    if (c != 10) ungetc (c,f);
  }
  else ungetc (c,f);

  // read in rest of data
  image_data = new byte [image_width*image_height*3];
  if (fread (image_data,image_width*image_height*3,1,f) != 1)
    LFATAL ("Can not read data from image file `%s'",filename);
  fclose (f);

  itsInitialized = true;
}


VPImage::~VPImage()
{
  delete[] image_data;
}

void VPImage::skipWhiteSpace (const char *filename, FILE *f)
{
  int c,d;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) LFATAL ("unexpected end of file in \"%s\"",filename);

    // skip comments
    if (c == '#') {
      do {
        d = fgetc(f);
        if (d==EOF) LFATAL ("unexpected end of file in \"%s\"",filename);
      } while (d != '\n');
      continue;
    }

    if (c > ' ') {
      ungetc (c,f);
      return;
    }
  }
}


// read a number from a stream, this return 0 if there is none (that's okay
// because 0 is a bad value for all PPM numbers anyway).

int VPImage::readNumber (const char *filename, FILE *f)
{
  int c,n=0;
  for(;;) {
    c = fgetc(f);
    if (c==EOF) LFATAL ("unexpected end of file in \"%s\"",filename);
    if (c >= '0' && c <= '9') n = n*10 + (c - '0');
    else {
      ungetc (c,f);
      return n;
    }
  }
}


