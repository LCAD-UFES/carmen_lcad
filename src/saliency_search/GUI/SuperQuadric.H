/*!@file GUI/SuperQuadric.H  3D render SuperQuadric */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/SuperQuadric.H $
// $Id: SuperQuadric.H 13070 2010-03-28 15:59:21Z lior $
//

#ifndef SuperQuadric_H_DEFINED
#define SuperQuadric_H_DEFINED

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <GL/gl.h>
#undef APIENTRY // otherwise it gets redefined between gl.h and glut.h???
#include <GL/glut.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//This class is derived from examples from http://www.gamedev.net/reference/articles/article1172.asp
//
class SuperQuadric
{
public:

  SuperQuadric();

  ~SuperQuadric();

  /* sqEllipsoid(a1, a2, a3, u, v, n, e, *x, *y, *z, *nx, *ny, *nz)
   *
   * a1, a2, and a3 are the x, y, and z scaling factors, respecfully.
   * For proper generation of the solid, u should be >= -PI / 2 and <= PI / 2.
   * Similarly, v should be >= -PI and <= PI.
   */
  void ellipsoid ( float a1, float a2, float a3,
      float u, float v, float n, float e, 
      float *x, float  *y, float *z,
      float *nx, float *ny, float *nz );

  /* sqToroid(a1, a2, a3, u, v, n, e, alpha, *x, *y, *z, *nx, *ny, *nz)
   *
   * a1, a2, and a3 are the x, y, and z scaling factors, respecfully.
   * For proper generation of the solid, u should be >= -PI and <= PI.
   * Similarly, v should be >= -PI and <= PI.
   * Also, alpha should be > 1.
   */
  void toroid ( float a1, float a2, float a3,
      float u, float v, float n, float e, float alpha,
      float *x, float  *y, float *z,
      float *nx, float *ny, float *nz );


  /* sqSolidEllipsoid 
   * 
   * Generates a solid ellipsoid using the parameters from sq and optionally 
   * generates texture coordinates and a display list using the ID from sq.
   */
  void solidEllipsoid();

  /* sqSolidToroid ( sq, make_display_list, gen_texture_coordinates )
   * 
   * Generates a solid toroid using the parameters from sq and optionally 
   * generates texture coordinates and a display list using the ID from sq.
   */
  void solidToroid();

  /* sqEllipsoidInsideOut (x, y, z )
   *
   * Tests to see if point P is inside the SuperQuadric sq.
   * Returns 1 if on the surface, > 1 if outside the surface, or
   * < 1 if inside the surface
   */
  float ellipsoidInsideOut ( float x, float y, float z );

  /* sqToroidInsideOut ( x, y, z )
   *
   * Tests to see if point P is inside the SuperQuadric sq.
   * Returns 1 if on the surface, > 1 if outside the surface, or
   * < 1 if inside the surface
   */
  float toroidInsideOut (float x, float y, float z );

  //Some simple SuperQuadrics
  void solidSphere ( float radius );
  void solidCylinder ( float radius );
  void solidStar ( float radius );
  void solidDoublePyramid ( float radius );
  void solidTorus ( float radius1, float radius2 );
  void solidPineappleSlice ( float radius1, float radius2 );
  void solidPillow ( float radius );
  void solidSquareTorus ( float radius1, float radius2 );
  void solidPinchedTorus ( float radius1, float radius2 );
  void solidRoundCube ( float radius );

public:
  float its_a1, its_a2, its_a3;     /* Scaling factors for x, y, and z */
  float its_alpha;          /* For generating toroids. This is the inner radius */
  float its_n, its_e;           /* North-South/East-West Roundness/Squareness Factors */
  float its_u1, its_u2;         /* Initial and Final U values */
  float its_v1, its_v2;         /* Initial and Final V values */
  int its_u_segs;           /* Number of segments for U */
  int its_v_segs;           /* Number of segments for V */
  float its_s1, its_t1;         /* Initial s and t texture coordinates */
  float its_s2, its_t2;         /* Final S and T texture coordinates */
  int itsTextureFlag;     /* Flag determining texture coordinate specification */
  int itsGlListId;       /* OpenGL Display List ID */
  bool itsUseDisplayList;
  bool itsUseTexture;
};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif //
