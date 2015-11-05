/*!@file Image/colorDefs.H definitions of a few colors
 */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
//
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/colorDefs.H $
// $Id: colorDefs.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef COLORDEFS_H_DEFINED
#define COLORDEFS_H_DEFINED

// a few colors by their names
#define COL_BLACK        PixRGB<byte>(  0,  0,  0)
#define COL_WHITE        PixRGB<byte>(255,255,255)
#define COL_YELLOW       PixRGB<byte>(255,255,  0)
#define COL_ORANGE       PixRGB<byte>(255,128,  0)
#define COL_BLUE         PixRGB<byte>(  0,  0,255)
#define COL_RED          PixRGB<byte>(255,  0,  0)
#define COL_GREEN        PixRGB<byte>(  0,255,  0)

/* these values come from a statistical analysis
   of ~8000 faces in color images
   Dirk Walther, 2004 */
#define COL_SKIN         PixRGB<byte>(176,122,107)
#define COL_SKIN_MUR     0.434904
#define COL_SKIN_MUG     0.301983
#define COL_SKIN_SIGR    0.053375
#define COL_SKIN_SIGG    0.024349
#define COL_SKIN_RHO     0.5852

// this value is defined as transparent by convention
#define COL_TRANSPARENT  PixRGB<byte>(  1,  1,  1)

// these values are set for MBARI stuff
#define COL_INTERESTING COL_YELLOW
#define COL_CANDIDATE   COL_ORANGE
#define COL_SKIP        COL_BLUE
#define COL_PREDICTION  COL_GREEN
#define COL_FOE         COL_RED

const double JPEGYUV_Y_OFFSET  = 0.0;
const int    JPEGYUV_Y_RANGE   = 255;
const double JPEGYUV_UV_OFFSET = 127.5;
const int    JPEGYUV_UV_RANGE  = 255;

// coeffs from PVRG-JPEG:
// also see http://www.ijg.org/files/jpegsrc.v6b.tar.gz (jdcolor.c)
const double JPEGYUV_RGB_Y =  1.0;
const double JPEGYUV_R_V   =  1.402;
const double JPEGYUV_G_U   = -0.34414;
const double JPEGYUV_G_V   = -0.71414;
const double JPEGYUV_B_U   =  1.772;

/*
  the rgb2yuv and yuv2rgb conversion matrics are mathematical inverses
  of each other:

  ._                          _.   ._                          _.(-1)
  |                            |   |                            |
  |  1.0000        0   1.4020  |   |  0.2990   0.5870   0.1140  |
  |  1.0000  -0.3441  -0.7141  | = | -0.1687  -0.3313   0.5000  |
  |  1.0000   1.7720        0  |   |  0.5000  -0.4187  -0.0813  |
  |_                          _|   |_                          _|
 */

const double JPEGYUV_Y_R =  0.29900072215374496; //  0.299
const double JPEGYUV_Y_G =  0.58699836510985282; //  0.587
const double JPEGYUV_Y_B =  0.11400091273640223; //  0.114
const double JPEGYUV_U_R = -0.16873629918382893; // -0.16874
const double JPEGYUV_U_G = -0.33126318572790797; // -0.33126
const double JPEGYUV_U_B =  0.5;                 //  0.5
const double JPEGYUV_V_R =  0.5;                 //  0.5
const double JPEGYUV_V_G = -0.41868642304554415; // -0.41869
const double JPEGYUV_V_B = -0.08131306186619276; // -0.08131

/*
  see http://www.fourcc.org/fccyvrgb.php
      http://www.fourcc.org/fccyvrgb.php#mikes_answer
      http://www.siggraph.org/education/materials/HyperGraph/video/mpeg/mpegfaq/ccir601.html
      http://www.siggraph.org/education/materials/HyperGraph/video/mpeg/mpegfaq/mpeggloss-y.html

  The "natural" range for the luminance (Y) component is [0.0,1.0]. In
  jpeg-yuv, that gets mapped to [0,255]. In video-yuv, it gets mapped
  to [16,235]. To restore, we subtract VIDEOYUV_Y_OFFSET=16, giving
  [0,219], then divide by VIDEOYUV_Y_RANGE=219, giving [0,1].

  The "natural" range for the chrominance (U,V) components is
  [-0.5,0.5]. In jpeg-yuv, that gets mapped to [0,255]. In video-yuv,
  it gets mapped to [16,240]. To restore, we subtract
  VIDEOYUV_UV_OFFSET=128, giving [-112,112], then divide by
  VIDEOYUV_UV_RANGE=224, giving [-0.5,0.5].
 */

const int VIDEOYUV_Y_OFFSET  = 16;
const int VIDEOYUV_Y_RANGE   = 219; // Y range = [16,235]
const int VIDEOYUV_UV_OFFSET = 128;
const int VIDEOYUV_UV_RANGE  = 224; // UV range = [16,240]

const double VIDEOYUV_RGB_Y = JPEGYUV_RGB_Y * (255.0/VIDEOYUV_Y_RANGE);
const double VIDEOYUV_R_V   = JPEGYUV_R_V   * (255.0/VIDEOYUV_UV_RANGE);
const double VIDEOYUV_G_U   = JPEGYUV_G_U   * (255.0/VIDEOYUV_UV_RANGE);
const double VIDEOYUV_G_V   = JPEGYUV_G_V   * (255.0/VIDEOYUV_UV_RANGE);
const double VIDEOYUV_B_U   = JPEGYUV_B_U   * (255.0/VIDEOYUV_UV_RANGE);

/*
  The above conversions should give something like the following:

  VIDEOYUV_RGB_Y: 1.16438356164383561
  VIDEOYUV_R_V:   1.59602678571428580
  VIDEOYUV_G_U:  -0.39176651785714289
  VIDEOYUV_G_V:  -0.81297187500000001
  VIDEOYUV_B_U:   2.01723214285714292
*/

const double VIDEOYUV_Y_R = JPEGYUV_Y_R * (VIDEOYUV_Y_RANGE/255.0);
const double VIDEOYUV_Y_G = JPEGYUV_Y_G * (VIDEOYUV_Y_RANGE/255.0);
const double VIDEOYUV_Y_B = JPEGYUV_Y_B * (VIDEOYUV_Y_RANGE/255.0);
const double VIDEOYUV_U_R = JPEGYUV_U_R * (VIDEOYUV_UV_RANGE/255.0);
const double VIDEOYUV_U_G = JPEGYUV_U_G * (VIDEOYUV_UV_RANGE/255.0);
const double VIDEOYUV_U_B = JPEGYUV_U_B * (VIDEOYUV_UV_RANGE/255.0);
const double VIDEOYUV_V_R = JPEGYUV_V_R * (VIDEOYUV_UV_RANGE/255.0);
const double VIDEOYUV_V_G = JPEGYUV_V_G * (VIDEOYUV_UV_RANGE/255.0);
const double VIDEOYUV_V_B = JPEGYUV_V_B * (VIDEOYUV_UV_RANGE/255.0);

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
