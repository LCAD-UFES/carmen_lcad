/*!@file Gist/ModelFace.H model face parameters */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/ModelFace.H $
// $Id: ModelFace.H 8409 2007-05-19 04:51:53Z siagian $
//

#ifndef MODEL_FACE_H_DEFINED
#define MODEL_FACE_H_DEFINED

#define NUM_FACE_PART          3
enum FacePart { EYE = 0, NOSE = 1, MOUTH = 2, L_EYE = 3, R_EYE = 4 };

#define NUM_HEADER_LINE        4

#define MODEL_FACE_WIDTH       40
#define MODEL_FACE_HEIGHT      50
#define MODEL_FACE_DIAGONAL    64

// NOTE: left is the person's on the image's left
//       your right

// NOTE: width and height should always be even numbers
//       diagonal smallest even integer bigger than diagonal
#define MODEL_EYE_WIDTH        12
#define MODEL_EYE_HEIGHT       6
#define MODEL_EYE_DIAGONAL     14
#define RE_INPUT               32*4
#define RE_HIDDEN_1            200
#define RE_HIDDEN_2            60
#define RE_TOP                 1
#define RE_LEFT                17
#define RE_BOTTOM              18
#define RE_RIGHT               19
#define LE_TOP                 0
#define LE_LEFT                14
#define LE_BOTTOM              15
#define LE_RIGHT               16

#define MODEL_NOSE_WIDTH       14
#define MODEL_NOSE_HEIGHT      20
#define MODEL_NOSE_DIAGONAL    26
#define N_INPUT                61*4
#define N_HIDDEN_1             300
#define N_HIDDEN_2             100
#define N_TOP                  6
#define N_LEFT                 7
#define N_BOTTOM               9
#define N_RIGHT                8

#define MODEL_MOUTH_WIDTH      20
#define MODEL_MOUTH_HEIGHT     10
#define MODEL_MOUTH_DIAGONAL   24
#define M_INPUT                48*4
#define M_HIDDEN_1             250
#define M_HIDDEN_2             60
#define M_TOP                  10
#define M_LEFT                 12
#define M_BOTTOM               11
#define M_RIGHT                13


#define GIST_IMAGE_FOLDER      "pics/face/"
#define GIST_TRAIN_FILE        "gistTrain.txt"
#define GIST_TEST_FILE         "gistTest.txt"
#define WEIGHT_DATA_FOLDER     "../weightData/"
#define IMAGE_FOLDER           "../images/awata0830_M_3/"
#define DATA_FOLDER            "../modelgraphs/awata0830_M_3/"
#define FILE_TAGNAME           "awata0830_M_3_"
#define IMAGE_FILE_EXT         ".png"
#define DATA_FILE_EXT          ".mdlgraph"

#define NUM_DIR                4  // the number of used orientations
#define NUM_G_LEV              5  // number of gabor levels
#define NUM_G_DIR              8  // number of gabor orientations
#define NUM_H_LEV              5  // number of output levels
#define PL_HIDDEN_1            60
#define PL_HIDDEN_2            30
#define PL2_INPUT              40
#define PL2_HIDDEN_1           60
#define PL2_HIDDEN_2           30

#endif // !MODEL_FACE_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
