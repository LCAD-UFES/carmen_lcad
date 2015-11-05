/*!@file GUI/SuperQuadric.C  3D rendering SuperQuadric  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/SuperQuadric.C $
// $Id: SuperQuadric.C 13070 2010-03-28 15:59:21Z lior $
//


#ifndef SuperQuadric_C_DEFINED
#define SuperQuadric_C_DEFINED

#include "GUI/SuperQuadric.H"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

namespace SuperQuadricFunc
{

  //Helper functions
 
  /* Returns the sign of x */
  float sgnf ( float x ) {
    if ( x < 0 )
      return -1;
    if ( x > 0 )
      return 1;
    return 0;
  }

  /* Returns the absolute value of x */
  float absf ( float x ) {
    if ( x < 0 )
      return -x;
    return x;
  }

  /* sqC (v, n)
   * This function implements the c(v,n) utility function
   *
   * c(v,n) = sgnf(cos(v)) * |cos(v)|^n
   */
  float sqC ( float v, float n ) {
    return sgnf((float)cos(v)) * (float)powf(absf((float)cos(v)),n);
  }

  /* sqCT (v, n, alpha)
   * This function implements the CT(v,n,alpha) utility function
   *
   * CT(v,n,alpha) = alpha + c(v,n)
   */
  float sqCT ( float v, float n, float alpha ) {
    return alpha + sqC(v,n);
  }

  /* sqS (v, n)
   * This function implements the s(v,n) utility function
   *
   * s(v,n) = sgnf(sin(v)) * |sin(v)|^n
   */
  float sqS ( float v, float n ) {
    return sgnf((float)sin(v)) * (float)powf(absf((float)sin(v)),n);
  }


  void taper(float *x, float *y, float *z, 
      float az, float Kx, float Ky)
  {

    *x = ( (Kx/az) * *z  + 1) * *x;
    *y = ( (Ky/az) * *z  + 1) * *y;

  }


}
// ######################################################################
SuperQuadric::SuperQuadric()
{
  itsUseDisplayList = false;
  itsUseTexture = false;

  its_u_segs = 20;
  its_v_segs = 20;
}

SuperQuadric::~SuperQuadric()
{
}



void SuperQuadric::ellipsoid ( float a1, float a2, float a3,
    float u, float v, float n, float e, 
    float *x, float  *y, float *z,
    float *nx, float *ny, float *nz )
{
  *x = a1 * SuperQuadricFunc::sqC (u, n) * SuperQuadricFunc::sqC (v, e);
  *y = a2 * SuperQuadricFunc::sqC (u, n) * SuperQuadricFunc::sqS (v, e);
  *z = a3 * SuperQuadricFunc::sqS (u, n);

  //taper(x, y, z, a3, 0, 0);

  *nx= SuperQuadricFunc::sqC (u, 2 - n) * SuperQuadricFunc::sqC (v, 2 - e) / a1;
  *ny= SuperQuadricFunc::sqC (u, 2 - n) * SuperQuadricFunc::sqS (v, 2 - e) / a2;
  *nz= SuperQuadricFunc::sqS (u, 2 - n) / a3;
}

void SuperQuadric::toroid ( float a1, float a2, float a3,
                             float u, float v,
                             float n, float e, float alpha,
    float *x, float  *y, float *z, float *nx, float *ny, float *nz )
{
  float A1, A2, A3;
  A1 = 1 / (a1 + alpha);
  A2 = 1 / (a2 + alpha);
  A3 = 1 / (a3 + alpha);
  *x = A1 * SuperQuadricFunc::sqCT (u, e, alpha) * SuperQuadricFunc::sqC (v, n);
  *y = A2 * SuperQuadricFunc::sqCT (u, e, alpha) * SuperQuadricFunc::sqS (v, n);
  *z = A3 * SuperQuadricFunc::sqS (u, e);

  *nx= SuperQuadricFunc::sqC (u, 2 - e) * SuperQuadricFunc::sqC (v, 2 - n) / A1;
  *ny= SuperQuadricFunc::sqC (u, 2 - e) * SuperQuadricFunc::sqS (v, 2 - n) / A2;
  *nz= SuperQuadricFunc::sqS (u, 2 - e) / A3;
}

void SuperQuadric::solidEllipsoid ()
{
  //TODO: This can be made more effficent by using lookup tables
  //Since some vetices share the same calculation (4 calc for the same vertex),
  //they can be stored //into a temporary lookuptable and puuled when needed.
  //Use CULLing
  //
  float U, dU, V, dV;
  float S, dS, T, dT;
  int X, Y;     /* for looping */
  float x, y, z;
  float nx, ny, nz;

  /* Calculate delta variables */
  dU = (float)(its_u2 - its_u1) / (float)its_u_segs;
  dV = (float)(its_v2 - its_v1) / (float)its_v_segs;
  dS = (float)(its_s2 - its_s1) / (float)its_u_segs;
  dT = (float)(its_t2 - its_t1) / (float)its_v_segs;

  glDisable (GL_CULL_FACE); //Disable cull for now, this will make things slower, and should eventually figured out
  glEnable (GL_NORMALIZE); //To normalize the normals for lightting conditions
  
  /* If we're going to make a display list then start it */
  if ( itsUseDisplayList )
    glNewList ( itsGlListId, GL_COMPILE );

  /* Initialize variables for loop */
  U = its_u1;
  S = its_s1;
  glBegin ( GL_QUADS );
  for ( Y = 0; Y < its_u_segs; Y++ ) {
    /* Initialize variables for loop */
    V = its_v1;
    T = its_t1;
    for ( X = 0; X < its_v_segs; X++ ) {
      /* VERTEX #1 */
      ellipsoid (its_a1, its_a2, its_a3,
          U, V,
          its_n, its_e,
          &x, &y, &z, &nx, &ny, &nz );
      glNormal3f ( nx, ny, nz );
      glTexCoord2f ( S, T );
      glVertex3f ( x, y, z );

      /* VERTEX #2 */     
      ellipsoid (its_a1, its_a2, its_a3,
          U + dU, V,
          its_n, its_e,
          &x, &y, &z, &nx, &ny, &nz );
      glNormal3f ( nx, ny, nz );
      glTexCoord2f ( S + dS, T );
      glVertex3f ( x, y, z );

      /* VERTEX #3 */
      ellipsoid (its_a1, its_a2, its_a3,
          U + dU, V + dV,
          its_n, its_e,
          &x, &y, &z, &nx, &ny, &nz );
      glNormal3f ( nx, ny, nz );
      glTexCoord2f ( S + dS, T + dT );
      glVertex3f ( x, y, z );

      /* VERTEX #4 */
      ellipsoid (its_a1, its_a2, its_a3,
          U, V + dV,
          its_n, its_e,
          &x, &y, &z, &nx, &ny, &nz );
      glNormal3f ( nx, ny, nz );
      glTexCoord2f ( S, T + dT );
      glVertex3f ( x, y, z );

      /* Update variables for next loop */
      V += dV;
      T += dT;
    }
    /* Update variables for next loop */
    S += dS;
    U += dU;
  }
  glEnd ( );

  /* If we're making a display list then stop */
  if ( itsUseDisplayList )
    glEndList ( );
  glEnable (GL_CULL_FACE);
  glDisable (GL_NORMALIZE);
  
}

void SuperQuadric::solidToroid ()
{
  float U, dU, V, dV;
  float S, dS, T, dT;
  int X, Y;     /* for looping */
  float x, y, z;
  float nx, ny, nz;

  /* Calculate delta variables */
  dU = (float)(its_u2 - its_u1) / its_u_segs;
  dV = (float)(its_v2 - its_v1) / its_v_segs;
  dS = (float)(its_s2 - its_s1) / its_u_segs;
  dT = (float)(its_t2 - its_t1) / its_v_segs;

  glDisable (GL_CULL_FACE); //Disable cull for now, this will make things slower, and should eventually figured out
  
  glEnable (GL_NORMALIZE); //To normalize the normals for lightting conditions

  /* If we're going to make a display list then start it */
  if (itsUseDisplayList )
    glNewList ( itsGlListId, GL_COMPILE );

  /* Initialize variables for loop */
  U = its_u1;
  S = its_s1;
  glBegin ( GL_QUADS );
  for ( Y = 0; Y < its_u_segs; Y++ ) {
    /* Initialize variables for loop */
    V = its_v1;
    T = its_t1;
    for ( X = 0; X < its_v_segs; X++ ) {
      /* VERTEX #1 */
      toroid ( its_a1, its_a2, its_a3,
          U, V,
          its_n, its_e, its_alpha, 
          &x, &y, &z,
          &nx, &ny, &nz );

      glNormal3f ( nx, ny, nz );
      glTexCoord2f ( S, T );
      glVertex3f ( x, y, z );

      /* VERTEX #2 */
      toroid ( its_a1, its_a2, its_a3,
          U + dU, V,
          its_n, its_e, its_alpha,
          &x, &y, &z, &nx, &ny, &nz );
      glNormal3f ( nx, ny, nz );
      glTexCoord2f ( S + dS, T );
      glVertex3f ( x, y, z );

      /* VERTEX #3 */
      toroid ( its_a1, its_a2, its_a3,
          U + dU, V + dV,
          its_n, its_e, its_alpha,
          &x, &y, &z, &nx, &ny, &nz );
      glNormal3f ( nx, ny, nz );
      glTexCoord2f ( S + dS, T + dT );
      glVertex3f ( x, y, z );

      /* VERTEX #4 */
      toroid ( its_a1, its_a2, its_a3,
          U, V + dV,
          its_n, its_e, its_alpha,
          &x, &y, &z, &nx, &ny, &nz );
      glNormal3f ( nx, ny, nz );
      glTexCoord2f ( S, T + dT);
      glVertex3f ( x, y, z );

      /* Update variables for next loop */
      V += dV;
      T += dT;
    }
    /* Update variables for next loop */
    S += dS;
    U += dU;
  }
  glEnd ( );

  /* If we're making a display list then stop */
  if ( itsUseDisplayList )
    glEndList ( );
  glEnable (GL_CULL_FACE); 
  glDisable (GL_NORMALIZE);
}

float SuperQuadric::ellipsoidInsideOut (float x, float y, float z )
{
  float result;
  result = powf ( powf ( x / its_a1, 2 / its_e ) + 
      powf ( y / its_a2, 2 / its_e ), its_e / its_n ) + 
    powf ( z / its_a3, 2 / its_n );
  return result;
}

float SuperQuadric::toroidInsideOut ( float x, float y, float z )
{
  float result;
  result = powf ( powf ( powf ( x / its_a1, 2 / its_e ) + powf ( y / its_a2, 2 / its_e ),
        its_e / 2 ) - its_alpha, 2 / its_n ) + powf ( z / its_a3, 2 / its_n );
  return result;
}

void SuperQuadric::solidSphere ( float radius ) {
  its_a1 = its_a2 = its_a3 = radius;
  its_n = 1.0f;
  its_e = 1.0f;
  its_u1 = -M_PI / 2;
  its_u2 = M_PI / 2;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidEllipsoid ();
}

void SuperQuadric::solidCylinder ( float radius ) {
  its_a1 = radius;
  its_a2 = radius;
  its_a3 = radius;
  its_n = 0.0f;
  its_e = 1.0f;
  its_u1 = -M_PI / 2;
  its_u2 = M_PI / 2;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidEllipsoid();
}

void SuperQuadric::solidStar ( float radius ) {

  its_a1 = its_a2 = its_a3 = radius;
  its_n = 4.0f;
  its_e = 4.0f;
  its_u1 = -M_PI / 2;
  its_u2 = M_PI / 2;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidEllipsoid();
}

void SuperQuadric::solidDoublePyramid ( float radius ) {
  its_a1 = its_a2 = its_a3 = radius;
  its_n = 2.0f;
  its_e = 2.0f;
  its_u1 = -M_PI / 2;
  its_u2 = M_PI / 2;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidEllipsoid();
}

void SuperQuadric::solidTorus ( float radius1, float radius2 )
{
  its_a1 = its_a2 = its_a3 = (radius1 + radius2)/2.0f;
  its_alpha = radius2;
  its_n = 1.0f;
  its_e = 1.0f;
  its_u1 = -M_PI;
  its_u2 = M_PI;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidToroid();
}

void SuperQuadric::solidPineappleSlice ( float radius1, float radius2 )
{
  its_a1 = its_a2 = its_a3 = (radius1 + radius2)/2.0f;
  its_alpha = radius2;
  its_n = 0.0f;
  its_e = 1.0f;
  its_u1 = -M_PI;
  its_u2 = M_PI;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidToroid();
}

void SuperQuadric::solidPillow ( float radius ) {
  its_a1 = its_a2 = its_a3 = radius;
  its_n = 1.0f;
  its_e = 0.0f;
  its_u1 = -M_PI / 2;
  its_u2 = M_PI / 2;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidEllipsoid();
}

void SuperQuadric::solidSquareTorus ( float radius1, float radius2 )
{
  its_a1 = its_a2 = its_a3 = (radius1 + radius2)/2.0f;
  its_alpha = radius2;
  its_n = 0.2f;
  its_e = 0.2f;
  its_u1 = -M_PI;
  its_u2 = M_PI;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidToroid();
}

void SuperQuadric::solidPinchedTorus ( float radius1, float radius2 )
{
  its_a1 = its_a2 = its_a3 = (radius1 + radius2)/2.0f;
  its_alpha = radius2;
  its_n = 1.0f;
  its_e = 4.0f;
  its_u1 = -M_PI;
  its_u2 = M_PI;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidToroid();
}

void SuperQuadric::solidRoundCube ( float radius ) {
  its_a1 = its_a2 = its_a3 = radius;
  its_n = 0.2f;
  its_e = 0.2f;
  its_u1 = -M_PI / 2;
  its_u2 = M_PI / 2;
  its_v1 = -M_PI;
  its_v2 = M_PI;
  its_s1 = 0.0f;
  its_t1 = 0.0f;
  its_s2 = 1.0f;
  its_t2 = 1.0f;
  solidEllipsoid();
}

#endif
