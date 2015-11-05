/*!@file GUI/ViewPort3D.C  3D rendering view port */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/ViewPort3D.C $
// $Id: ViewPort3D.C 14492 2011-02-11 22:53:49Z lior $
//


//TODO:
//http://www.opengl.org.ru/docs/pg/0204.html
//void glEdgeFlag(GLboolean flag);
//void glEdgeFlagv(const GLboolean *flag);
//
//Indicates whether a vertex should be considered as initializing a boundary edge of a polygon. If flag is GL_TRUE, the edge flag is set to TRUE (the default), and any vertices created are considered to precede boundary edges until this function is called again with flag being GL_FAL
// for when drawing wireframe mode

#ifndef ViewPort3D_C_DEFINED
#define ViewPort3D_C_DEFINED

#include "GUI/ViewPort3D.H"
#include "Image/DrawOps.H"
#include "GUI/DebugWin.H"
#include <stdio.h>
// ######################################################################
ViewPort3D::ViewPort3D(const int width,const int height, 
    bool wireframe, bool useLights, bool useFeedback)  :
  itsScreenWidth(width),
  itsScreenHeight(height),
  itsCameraPosition(Point3D<float>(0,0,0)),
  itsCameraRotation(Point3D<float>(0,0,0)),
  itsShowRenderWindow(false),
  itsWireframe(wireframe),
  itsUseLights(useLights),
  itsUseFeedback(useFeedback),
  itsSphereQuality(5),
  itsCylinderQuality(5),
  itsSphereListNum(0),
  itsInitGlew(false)

{

  //Initialize the window the the opengl

  itsDisplay = XOpenDisplay(0);
  if (!itsDisplay)
    LFATAL("Failed to open X display");

  // Get a matching FB config
  static int visual_attribs[] =
    {
      GLX_X_RENDERABLE    , True,
      GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
      GLX_RENDER_TYPE     , GLX_RGBA_BIT,
      GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
      GLX_RED_SIZE        , 8,
      GLX_GREEN_SIZE      , 8,
      GLX_BLUE_SIZE       , 8,
      GLX_ALPHA_SIZE      , 8,
      GLX_DEPTH_SIZE      , 24,
      GLX_STENCIL_SIZE    , 8,
      GLX_DOUBLEBUFFER    , True,
      //GLX_SAMPLE_BUFFERS  , 1,
      //GLX_SAMPLES         , 4,
      None
    };

  XVisualInfo *vi;
  LINFO( "Getting matching framebuffer configs" );
  int fbcount;
  GLXFBConfig *fbc = glXChooseFBConfig( itsDisplay, DefaultScreen( itsDisplay ),
                                        visual_attribs, &fbcount );
  if ( fbc )
  {
    LINFO( "Found %d matching FB configs.", fbcount );

    // Pick the FB config/visual with the most samples per pixel
    LINFO( "Getting XVisualInfos" );
    int best_fbc = -1, worst_fbc = -1, best_num_samp = -1, worst_num_samp = 999;

    for (int i = 0; i < fbcount; i++ )
    {
      XVisualInfo *tmpVi = glXGetVisualFromFBConfig( itsDisplay, fbc[i] );
      if ( tmpVi )
      {
        int samp_buf, samples;
        glXGetFBConfigAttrib( itsDisplay, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf );
        glXGetFBConfigAttrib( itsDisplay, fbc[i], GLX_SAMPLES       , &samples  );

        LINFO( "  Matching fbconfig %d, visual ID 0x%2x: SAMPLE_BUFFERS = %d,"
            " SAMPLES = %d",
            i, (uint)tmpVi -> visualid, samp_buf, samples );

        if ( best_fbc < 0 || (samp_buf && samples > best_num_samp) )
          best_fbc = i, best_num_samp = samples;
        if ( worst_fbc < 0 || !samp_buf || samples < worst_num_samp )
          worst_fbc = i, worst_num_samp = samples;
      }
      XFree( tmpVi );
    }
    // Get a visual
    //int fbc_id = best_fbc;
    int fbc_id = worst_fbc;

    vi = glXGetVisualFromFBConfig( itsDisplay, fbc[ fbc_id ]  );
    LINFO( "Chosen visual ID = 0x%x", (uint)vi->visualid );
  } else {

    int screen = DefaultScreen(itsDisplay);
    // get GL visual
    static int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE,16,
      GLX_RED_SIZE,4, GLX_GREEN_SIZE,4,
      GLX_BLUE_SIZE,4, None};
    vi = glXChooseVisual (itsDisplay,screen,attribList);
  }
  if (!vi) LFATAL("no good X11 visual found for OpenGL");

  LINFO( "Creating colormap" );
  XSetWindowAttributes swa;
  swa.colormap = XCreateColormap( itsDisplay, RootWindow( itsDisplay, vi->screen ),
                                  vi->visual, AllocNone );
  swa.background_pixmap = None ;
  swa.border_pixel      = 0;
  swa.event_mask        = StructureNotifyMask;

  LINFO( "Creating window");
  itsWin = XCreateWindow( itsDisplay, RootWindow( itsDisplay, vi->screen ),
                              0, 0, itsScreenWidth, itsScreenHeight, 0,
                              vi->depth, InputOutput,
                              vi->visual,
                              CWBorderPixel|CWColormap|CWEventMask, &swa );
  if ( !itsWin )
    LFATAL( "Failed to create window." );

  if (itsShowRenderWindow)
  {
    XStoreName( itsDisplay, itsWin, "Render Window");
    XMapWindow( itsDisplay, itsWin ); //Enable this to show the window
  }

  // See if GL driver supports glXCreateContextAttribsARB()
  //   Create an old-style GLX context first, to get the correct function ptr.
  // glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;

  GLXContext ctx_old = glXCreateContext( itsDisplay, vi, 0, True );
  //glXCreateContextAttribsARB = 
    (glXCreateContextAttribsARBProc)
           glXGetProcAddress( (const GLubyte *) "glXCreateContextAttribsARB" );

  itsCtx = ctx_old;
  XFree( fbc );

  // Verifying that context is a direct context
  //LINFO("Verifying that context is direct");
  //if ( ! glXIsDirect ( itsDisplay, itsCtx ) )
  //  LFATAL( "Indirect GLX rendering context obtained");


  //Set default camera projection matrix
  double cameraParam[3][4] = {
    {350.475735, 0, 158.250000, 0},
    {0.000000, -363.047091, 118.250000, 0.000000},
    {0.000000, 0.000000, 1.000000, 0.00000}};
  setProjectionMatrix(cameraParam);

  itsUseExParam = false;


  if (itsUseFeedback)
  {
    itsFeedbackBufferSize = 1024*1024;
    itsFeedbackBuffer = new float[itsFeedbackBufferSize];
  } else {
    itsFeedbackBufferSize = 0;
    itsFeedbackBuffer = NULL;
  }



}

ViewPort3D::~ViewPort3D()
{
  delete[] itsFeedbackBuffer;
  glXMakeCurrent( itsDisplay, 0, 0 );
  glXDestroyContext( itsDisplay, itsCtx );
}

void ViewPort3D::initCtx()
{
  glXMakeCurrent (itsDisplay,itsWin,itsCtx);
}


void ViewPort3D::initFrame()
{
  glXMakeCurrent (itsDisplay,itsWin,itsCtx);

  //Draw and read from the back buffer
  glDrawBuffer(GL_BACK);
  glReadBuffer(GL_BACK);

  glEnable(GL_TEXTURE_2D);
  glShadeModel(GL_SMOOTH);
  glClearColor(0, 0, 0, 0.5);
  glClearDepth(1.0);

  //glDisable (GL_TEXTURE_GEN_S);
  //glDisable (GL_TEXTURE_GEN_T);
  //glShadeModel (GL_FLAT);

  glEnable (GL_CULL_FACE);
  glCullFace (GL_BACK);
  glFrontFace (GL_CCW);

  glEnable(GL_DEPTH_TEST); //enable depth testing
  glDepthFunc (GL_LESS);

  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); //Goot prespective calculations

  if (itsWireframe)
  {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //wireframe mode
    glLineWidth(1);
  }
  else
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); 

  if (itsUseLights)
    setLights();
  else
    glDisable(GL_LIGHTING);

  //Setup camera
  // Reset The Current Viewport
  glViewport(0,0,itsScreenWidth,itsScreenHeight);

  // Select The Projection Matrix
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixd(itsProjMatrix);

  //Set the camera position and orientation
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();

  if (itsUseExParam)
  {
    glLoadMatrixd( itsCameraExParam );
  } else {
    glLoadIdentity();

    glRotatef (180, 1,0,0); //Have the camera initialy look stright down
    glRotatef (itsCameraRotation.x, 1,0,0);
    glRotatef (itsCameraRotation.y, 0,1,0);
    glRotatef (-itsCameraRotation.z, 0,0,1);
    glTranslatef (-itsCameraPosition.x,-itsCameraPosition.y,-itsCameraPosition.z);
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);        // Clear Screen And Depth Buffer

  if (itsUseFeedback)
  {
    glFeedbackBuffer(itsFeedbackBufferSize, GL_2D, itsFeedbackBuffer);
    (void) glRenderMode(GL_FEEDBACK);
  }


}

void ViewPort3D::initProjection()
{
  glXMakeCurrent (itsDisplay,itsWin,itsCtx);

  //Draw and read from the back buffer
  glDrawBuffer(GL_BACK);
  glReadBuffer(GL_BACK);

  glEnable(GL_TEXTURE_2D);

  //Setup camera
  // Reset The Current Viewport
  glViewport(0,0,itsScreenWidth,itsScreenHeight);

  //// Select The Projection Matrix
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0,320,0.0,240, -1.0, 1.0);
  

  ////Set the camera position and orientation
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();

  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);        // Clear Screen And Depth Buffer

}


void ViewPort3D::setLights()
{

  glEnable(GL_LIGHTING);

  //Configure overall ambient light
  GLfloat lModelAmbient[] = { 0.5, 0.5, 0.5, 1.0 };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lModelAmbient);


  //For more realistic lighting effects
  //glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);


  //Illuminate the back faces as well
  //Should only be used if we have objects that are cut
  //so that the lightting will iluminate the back faces as well
  //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  //Configure light 0
  GLfloat LightAmbient[]= { 0.0f, 0.0f, 0.0f, 1.0f }; //Overall lighting
  GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f }; //RGB bright Directional light
  GLfloat LightPosition[]= { 100.0f, 100.0f, 2.0f, 0.0f }; //Position from above

  glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
  glLightfv(GL_LIGHT0, GL_POSITION,LightPosition);
  glEnable(GL_LIGHT0);

}

void ViewPort3D::setCamera ( const Point3D<float> pos,
                             const Point3D<float> rot )
{

  itsCameraPosition = pos;
  itsCameraRotation = rot;
}

void ViewPort3D::setCamera ( const double trans[3][4])
{
    for(int j = 0; j < 3; j++ ) {
      for(int i = 0; i < 4; i++ ) {
        itsCameraExParam[i*4+j] = trans[j][i];
      }
    }
    itsCameraExParam[0*4+3] = itsCameraExParam[1*4+3] = itsCameraExParam[2*4+3] = 0.0;
    itsCameraExParam[3*4+3] = 1.0;

    itsUseExParam = true;

}

Image<PixRGB<byte> > ViewPort3D::getFrame(){

  //Get the frame from the back buffer

  glFlush();
  Image<PixRGB<byte> > retImg(itsScreenWidth, itsScreenHeight, NO_INIT);
  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glReadBuffer(GL_BACK_LEFT);
  glReadPixels (0, 0, itsScreenWidth, itsScreenHeight,
      GL_RGB, GL_UNSIGNED_BYTE,
      (unsigned char*)retImg.getArrayPtr());

  return retImg;
}

Image<PixRGB<float> > ViewPort3D::getFrameFloat(){

  //Get the frame from the back buffer

  glFlush();
  Image<PixRGB<float> > retImg(itsScreenWidth, itsScreenHeight, NO_INIT);
  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glReadBuffer(GL_BACK_LEFT);
  glReadPixels (0, 0, itsScreenWidth, itsScreenHeight,
      GL_RGB, GL_FLOAT, retImg.getArrayPtr());

  return retImg;
}

Image<float> ViewPort3D::getDepthFrame(){

  //Get the frame from the back buffer

  glFlush();
  Image<float> retImg(itsScreenWidth, itsScreenHeight, NO_INIT);
  glPixelStorei(GL_PACK_ALIGNMENT,1);
  //glReadBuffer(GL_BACK_LEFT);
  glReadPixels (0, 0, itsScreenWidth, itsScreenHeight,
      GL_DEPTH_COMPONENT, GL_FLOAT,
      (unsigned char*)retImg.getArrayPtr());
  return retImg;
}

std::vector<ViewPort3D::Line> ViewPort3D::getFrameLines()
{
  std::vector<Line> lines;

  if (itsUseFeedback)
  {
    int size = glRenderMode(GL_RENDER);

    if (size < 0)
      LFATAL("FeedbackBuffer too small, increase itsFeedbackSize");

    int count = size;
    while (count) {
      float token = itsFeedbackBuffer[size-count]; count--;

      if (token == GL_PASS_THROUGH_TOKEN) {
        LINFO("GL_PASS_THROUGH_TOKEN\n");
        LINFO("  %4.2f\n", itsFeedbackBuffer[size-count]);
        count--;
        if (count < 0) break;
      }
      else if (token == GL_POINT_TOKEN) {
        LINFO("GL_POINT_TOKEN\n");
        count -= 2;
        if (count < 0) break;
        //print3DcolorVertex (size, &count, buffer,x,y,z);
      }
      else if (token == GL_LINE_TOKEN) {
        Line l;
        l.p1.i = itsFeedbackBuffer[size-count+0];
        l.p1.j = itsScreenHeight-itsFeedbackBuffer[size-count+1];

        l.p2.i = itsFeedbackBuffer[size-count+2];
        l.p2.j = itsScreenHeight-itsFeedbackBuffer[size-count+3];
        count -= 4;
        if (count < 0) break;

        lines.push_back(l);

      }
      else if (token == GL_LINE_RESET_TOKEN) {
        Line l;
        l.p1.i = itsFeedbackBuffer[size-count+0];
        l.p1.j = itsScreenHeight-itsFeedbackBuffer[size-count+1];

        l.p2.i = itsFeedbackBuffer[size-count+2];
        l.p2.j = itsScreenHeight-itsFeedbackBuffer[size-count+3];
        count -= 4;
        if (count < 0) break;

        lines.push_back(l);
      }
    }
    

  } else {
    LFATAL("Need to enable feedback to use this function");
  }


  return lines;
}


Point3D<float> ViewPort3D::getPosition(Point3D<float> loc)
{
  GLint viewport[4];
  GLdouble modelview[16];
  GLdouble projection[16];
  GLfloat winX, winY, winZ;
  GLdouble posX=0, posY=0, posZ=0;

  glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
  glGetDoublev( GL_PROJECTION_MATRIX, projection );
  glGetIntegerv( GL_VIEWPORT, viewport );

  winX = loc.x;
  winY = (float)viewport[3] - loc.y; //Flip the y
  winZ = loc.z;
  //glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
  //LINFO("Win %f %f(%f) %f", winX, winY, (float)viewport[3], winZ);

#ifdef INVT_HAVE_LIBGLUT
  gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
#else
  LFATAL("Need lib Glut fro this function");
#endif

  return Point3D<float>(posX, posY, posZ);

}

void ViewPort3D::setProjectionMatrix(float fovy, float aspect, float zmin, float zmax)
{

 glXMakeCurrent (itsDisplay,itsWin,itsCtx);
 glViewport(0,0,itsScreenWidth,itsScreenHeight);
 glMatrixMode(GL_PROJECTION);
 glLoadIdentity();

 float xmin, xmax, ymin, ymax;
 ymax = zmin * tan(fovy * M_PI / 360.0);
 ymin = -ymax;
 xmin = ymin * aspect;
 xmax = ymax * aspect;
 glFrustum(xmin, xmax, ymin, ymax, zmin, zmax);

 //Save the projection matrix
 glGetDoublev(GL_PROJECTION_MATRIX, itsProjMatrix);

}


void ViewPort3D::setProjectionMatrix(const double cameraParam[3][4])
{
  //Borrowed from ARTToolkit

  double   icpara[3][4];
  double   trans[3][4];
  double   p[3][3], q[4][4];
  int      i, j;
  double gnear = 0.1; //50;
  double gfar = 5000;

  if( paramDecompMat(cameraParam, icpara, trans) < 0 )
    LFATAL("Camera Parameter error!!\n");

  for( i = 0; i < 3; i++ ) {
    for( j = 0; j < 3; j++ ) {
      p[i][j] = icpara[i][j] / icpara[2][2];
    }
  }
  q[0][0] = (2.0 * p[0][0] / itsScreenWidth);
  q[0][1] = (2.0 * p[0][1] / itsScreenWidth);
  q[0][2] = ((2.0 * p[0][2] / itsScreenWidth)  - 1.0);
  q[0][3] = 0.0;

  q[1][0] = 0.0;
  q[1][1] = (2.0 * p[1][1] / itsScreenHeight);
  q[1][2] = ((2.0 * p[1][2] / itsScreenHeight) - 1.0);
  q[1][3] = 0.0;

  q[2][0] = 0.0;
  q[2][1] = 0.0;
  q[2][2] = (gfar + gnear)/(gfar - gnear);
  q[2][3] = -2.0 * gfar * gnear / (gfar - gnear);

  q[3][0] = 0.0;
  q[3][1] = 0.0;
  q[3][2] = 1.0;
  q[3][3] = 0.0;

  for( i = 0; i < 4; i++ ) {
    for( j = 0; j < 3; j++ ) {
      itsProjMatrix[i+j*4] = q[i][0] * trans[0][j]
                           + q[i][1] * trans[1][j]
                           + q[i][2] * trans[2][j];
    }
    itsProjMatrix[i+3*4] = q[i][0] * trans[0][3]
                         + q[i][1] * trans[1][3]
                         + q[i][2] * trans[2][3]
                         + q[i][3];
  }

}

#define NORM(a,b,c) sqrt(a*a + b*b + c*c)
#define DOT(a1,a2,a3,b1,b2,b3) (a1*b1 + a2*b2 + a3*b3)

int  ViewPort3D::paramDecompMat(const double source[3][4],
                                double cpara[3][4],
                                double trans[3][4] )
{
  int       r, c;
  double    Cpara[3][4];
  double    rem1, rem2, rem3;

  if( source[2][3] >= 0 ) {
    for( r = 0; r < 3; r++ ){
      for( c = 0; c < 4; c++ ){
        Cpara[r][c] = source[r][c];
      }
    }
  }
  else {
    for( r = 0; r < 3; r++ ){
      for( c = 0; c < 4; c++ ){
        Cpara[r][c] = -(source[r][c]);
      }
    }
  }

  for( r = 0; r < 3; r++ ){
    for( c = 0; c < 4; c++ ){
      cpara[r][c] = 0.0;
    }
  }
  cpara[2][2] = NORM( Cpara[2][0], Cpara[2][1], Cpara[2][2] );
  trans[2][0] = Cpara[2][0] / cpara[2][2];
  trans[2][1] = Cpara[2][1] / cpara[2][2];
  trans[2][2] = Cpara[2][2] / cpara[2][2];
  trans[2][3] = Cpara[2][3] / cpara[2][2];

  cpara[1][2] = DOT( trans[2][0], trans[2][1], trans[2][2],
      Cpara[1][0], Cpara[1][1], Cpara[1][2] );
  rem1 = Cpara[1][0] - cpara[1][2] * trans[2][0];
  rem2 = Cpara[1][1] - cpara[1][2] * trans[2][1];
  rem3 = Cpara[1][2] - cpara[1][2] * trans[2][2];
  cpara[1][1] = NORM( rem1, rem2, rem3 );
  trans[1][0] = rem1 / cpara[1][1];
  trans[1][1] = rem2 / cpara[1][1];
  trans[1][2] = rem3 / cpara[1][1];

  cpara[0][2] = DOT( trans[2][0], trans[2][1], trans[2][2],
      Cpara[0][0], Cpara[0][1], Cpara[0][2] );
  cpara[0][1] = DOT( trans[1][0], trans[1][1], trans[1][2],
      Cpara[0][0], Cpara[0][1], Cpara[0][2] );
  rem1 = Cpara[0][0] - cpara[0][1]*trans[1][0] - cpara[0][2]*trans[2][0];
  rem2 = Cpara[0][1] - cpara[0][1]*trans[1][1] - cpara[0][2]*trans[2][1];
  rem3 = Cpara[0][2] - cpara[0][1]*trans[1][2] - cpara[0][2]*trans[2][2];
  cpara[0][0] = NORM( rem1, rem2, rem3 );
  trans[0][0] = rem1 / cpara[0][0];
  trans[0][1] = rem2 / cpara[0][0];
  trans[0][2] = rem3 / cpara[0][0];

  trans[1][3] = (Cpara[1][3] - cpara[1][2]*trans[2][3]) / cpara[1][1];
  trans[0][3] = (Cpara[0][3] - cpara[0][1]*trans[1][3]
      - cpara[0][2]*trans[2][3]) / cpara[0][0];

  for( r = 0; r < 3; r++ ){
    for( c = 0; c < 3; c++ ){
      cpara[r][c] /= cpara[2][2];
    }
  }

  return 0;
}


////////////////////////////////////////////////////////////////////////////

void ViewPort3D::setColor(const PixRGB<byte> color)
{

  float r = color[0]/256.0;
  float g = color[1]/256.0;
  float b = color[2]/256.0;

  if (itsUseLights)
  {
    GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
    light_ambient[0] = r*0.3f;
    light_ambient[1] = g*0.3f;
    light_ambient[2] = b*0.3f;
    light_ambient[3] = 1; //alpha
    light_diffuse[0] = r*0.7f;
    light_diffuse[1] = g*0.7f;
    light_diffuse[2] = b*0.7f;
    light_diffuse[3] = 1; //alpha
    light_specular[0] = r*0.2f;
    light_specular[1] = g*0.2f;
    light_specular[2] = b*0.2f;
    light_specular[3] = 1; //alpha

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
    glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 50.0f);
  } else {
    //Simple color
    glColor3f(r,g,b);
  }

}

void ViewPort3D::drawGrid(Dims size, Dims step, float height)
{

  glPushMatrix();

  glTranslatef(0, 0, 0);
  glRotatef(0, 1,0,0);
  glRotatef(0, 0,1,0);
  glRotatef(0, 0,0,1);

  glBegin(GL_LINES);
  //Horizontal lines
  for(float i=-size.w()/2; i<=size.w()/2; i+=step.w())
  {
    glVertex3f( -size.h()/2,i,height);
    glVertex3f( size.h()/2,i,height);
  }

  //Vertical lines
  for(float i=-size.h()/2; i<=size.h()/2; i+=step.h())
  {
    glVertex3f(i, -size.w()/2,height);
    glVertex3f(i, size.w()/2,height);
  }

  glEnd();                 

  glPopMatrix();

}

void ViewPort3D::drawLine(const Point3D<float>& p1, const Point3D<float>& p2)
{
  glPushMatrix();

  glTranslatef(0, 0, 0);
  glRotatef(0, 1,0,0);
  glRotatef(0, 0,1,0);
  glRotatef(0, 0,0,1);

  glBegin(GL_LINES);
    glVertex3f(p1.x, p1.y, p1.z);
    glVertex3f(p2.x, p2.y, p2.z);
  glEnd();                 

  glPopMatrix();

}


void ViewPort3D::drawRectangle(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const float width, const float height,
                       const PixRGB<byte> color)
{

  glPushMatrix();
  setColor(color);

  glShadeModel (GL_FLAT);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  double lx = width*0.5f;
  double ly = height*0.5f;

  glBegin(GL_QUADS);
  glNormal3f(0,0,1);
  glTexCoord2i(0, 0);  glVertex3f( lx, ly, 0);                        // Top Right Of The Quad (Front)
  glTexCoord2i(0, 1);  glVertex3f(-lx, ly, 0);                        // Top Left Of The Quad (Front)
  glTexCoord2i(1, 1);  glVertex3f(-lx,-ly, 0);                        // Bottom Left Of The Quad (Front)
  glTexCoord2i(1, 0);  glVertex3f( lx,-ly, 0);                        // Bottom Right Of The Quad (Front)
  glEnd();                                                          // Done Drawing The Quad

  glPopMatrix();

}


void ViewPort3D::drawDisk(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const float radius,
                       const PixRGB<byte> color)
{
  glPushMatrix();
  setColor(color);
  glShadeModel (GL_SMOOTH);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  //// number of sides to the cylinder (divisible by 4):
  const int n = itsCylinderQuality*4;

  double a = double(M_PI*2.0)/double(n);
  double sa = (double) sin(a);
  double ca = (double) cos(a);

  // draw circle
  glShadeModel (GL_FLAT);
  double ny=1;
  double nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,0);
  for (int i=0; i<=n; i++) {
    glVertex3d (ny*radius,nz*radius,0);

    // rotate ny,nz
    double tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  glPopMatrix();
}

void ViewPort3D::drawCircle(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const float radius,
                       const PixRGB<byte> color)
{
  glPushMatrix();
  setColor(color);
  glShadeModel (GL_SMOOTH);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  //// number of sides to the cylinder (divisible by 4):
  const int n = itsCylinderQuality*4;

  double a = double(M_PI*2.0)/double(n);
  double sa = (double) sin(a);
  double ca = (double) cos(a);

  // draw circle
  double ny=1;
  double nz=0;                  // normal vector = (0,ny,nz)
  glBegin(GL_LINE_LOOP);
  for(int i=0; i<=n; i++)
  {
    glVertex3d( ny*radius, nz*radius,0);
    // rotate ny,nz
    double tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }

  glEnd();                 

  glPopMatrix();
}

void ViewPort3D::drawEllipse(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const float radiusX, const float radiusY,
                       const PixRGB<byte> color)
{
  glPushMatrix();
  setColor(color);
  glShadeModel (GL_SMOOTH);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  //// number of sides to the cylinder (divisible by 4):
  const int n = itsCylinderQuality*4;

  double a = double(M_PI*2.0)/double(n);
  double sa = (double) sin(a);
  double ca = (double) cos(a);

  // draw circle
  double ny=1;
  double nz=0;                  // normal vector = (0,ny,nz)
  glBegin(GL_LINE_LOOP);
  for(int i=0; i<=n; i++)
  {
    glVertex3d( ny*radiusX, nz*radiusY,0);
    // rotate ny,nz
    double tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }

  glEnd();                 

  glPopMatrix();
}


void ViewPort3D::drawBox(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const Point3D<float> size,
                       const PixRGB<byte> color)
{

  glPushMatrix();
  setColor(color);

  glShadeModel (GL_FLAT);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  double lx = size.x*0.5f;
  double ly = size.y*0.5f;
  double lz = size.z*0.5f;

  glBegin(GL_QUADS);
  glNormal3f(0,1,0);
  glTexCoord2i(0, 0);  glVertex3f( lx, ly,-lz);                        // Top Right Of The Quad (Top)
  glTexCoord2i(0, 1);  glVertex3f(-lx, ly,-lz);                        // Top Left Of The Quad (Top)
  glTexCoord2i(1, 1);  glVertex3f(-lx, ly, lz);                        // Bottom Left Of The Quad (Top)
  glTexCoord2i(1, 0);  glVertex3f( lx, ly, lz);                        // Bottom Right Of The Quad (Top)

  glNormal3f(0,-1,0);
  glTexCoord2i(0, 0);  glVertex3f( lx,-ly, lz);                        // Top Right Of The Quad (Bottom)
  glTexCoord2i(0, 1);  glVertex3f(-lx,-ly, lz);                        // Top Left Of The Quad (Bottom)
  glTexCoord2i(1, 1);  glVertex3f(-lx,-ly,-lz);                        // Bottom Left Of The Quad (Bottom)
  glTexCoord2i(1, 0);  glVertex3f( lx,-ly,-lz);                        // Bottom Right Of The Quad (Bottom)

  glNormal3f(0,0,1);
  glTexCoord2i(0, 0);  glVertex3f( lx, ly, lz);                        // Top Right Of The Quad (Front)
  glTexCoord2i(0, 1);  glVertex3f(-lx, ly, lz);                        // Top Left Of The Quad (Front)
  glTexCoord2i(1, 1);  glVertex3f(-lx,-ly, lz);                        // Bottom Left Of The Quad (Front)
  glTexCoord2i(1, 0);  glVertex3f( lx,-ly, lz);                        // Bottom Right Of The Quad (Front)

  glNormal3f(0,0,-1);
  glTexCoord2i(0, 0);  glVertex3f( lx,-ly,-lz);                        // Bottom Left Of The Quad (Back)
  glTexCoord2i(0, 1);  glVertex3f(-lx,-ly,-lz);                        // Bottom Right Of The Quad (Back)
  glTexCoord2i(1, 1);  glVertex3f(-lx, ly,-lz);                        // Top Right Of The Quad (Back)
  glTexCoord2i(1, 0);  glVertex3f( lx, ly,-lz);                        // Top Left Of The Quad (Back)

  glNormal3f(-1,0,0);
  glTexCoord2i(0, 0);  glVertex3f(-lx, ly, lz);                        // Top Right Of The Quad (Left)
  glTexCoord2i(0, 1);  glVertex3f(-lx, ly,-lz);                        // Top Left Of The Quad (Left)
  glTexCoord2i(1, 1);  glVertex3f(-lx,-ly,-lz);                        // Bottom Left Of The Quad (Left)
  glTexCoord2i(1, 0);  glVertex3f(-lx,-ly, lz);                        // Bottom Right Of The Quad (Left)

  glNormal3f(1,0,0);
  glTexCoord2i(0, 0);  glVertex3f( lx, ly,-lz);                        // Top Right Of The Quad (Right)
  glTexCoord2i(0, 1);  glVertex3f( lx, ly, lz);                        // Top Left Of The Quad (Right)
  glTexCoord2i(1, 1);  glVertex3f( lx,-ly, lz);                        // Bottom Left Of The Quad (Right)
  glTexCoord2i(1, 0);  glVertex3f( lx,-ly,-lz);                        // Bottom Right Of The Quad (Right)
  glEnd();                                                          // Done Drawing The Quad

  glPopMatrix();

}

void ViewPort3D::drawSphere(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const Point3D<float> size,
                       const PixRGB<byte> color)
{

  glPushMatrix();

  setColor(color);

  //Enable normalize because we scale the sphere and dont want color and lights to
  //get scaled as well (they need to have a mag of 1)
  //However, normalize slows things down, so we only use it for sphere
  glEnable (GL_NORMALIZE);

  glShadeModel (GL_SMOOTH);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);
  glScaled (size.x,size.y,size.z);


  if (itsSphereListNum == 0)
    itsSphereListNum = generateSphere(itsSphereQuality);

  if (itsSphereListNum != 0)
    glCallList (itsSphereListNum);

  glDisable (GL_NORMALIZE);

  glPopMatrix();
}

uint ViewPort3D::generateSphere(int quality)
{

  // icosahedron data for an icosahedron of radius 1.0
# define ICX 0.525731112119133606f
# define ICZ 0.850650808352039932f
  static GLdouble idata[12][3] = {
    {-ICX, 0, ICZ},
    {ICX, 0, ICZ},
    {-ICX, 0, -ICZ},
    {ICX, 0, -ICZ},
    {0, ICZ, ICX},
    {0, ICZ, -ICX},
    {0, -ICZ, ICX},
    {0, -ICZ, -ICX},
    {ICZ, ICX, 0},
    {-ICZ, ICX, 0},
    {ICZ, -ICX, 0},
    {-ICZ, -ICX, 0}
  };

  static int index[20][3] = {
    {0, 4, 1},          {0, 9, 4},
    {9, 5, 4},          {4, 5, 8},
    {4, 8, 1},          {8, 10, 1},
    {8, 3, 10},   {5, 3, 8},
    {5, 2, 3},          {2, 7, 3},
    {7, 10, 3},   {7, 6, 10},
    {7, 11, 6},   {11, 0, 6},
    {0, 1, 6},          {6, 1, 10},
    {9, 0, 11},   {9, 11, 2},
    {9, 2, 5},          {7, 2, 11},
  };

  uint listnum = glGenLists (1);
  glNewList (listnum,GL_COMPILE);
  glBegin (GL_TRIANGLES);
  for (int i=0; i<20; i++) {
    drawPatch (&idata[index[i][2]][0],&idata[index[i][1]][0],
        &idata[index[i][0]][0],quality);
  }
  glEnd();
  glEndList();

  return listnum;

}

void ViewPort3D::drawPatch (double p1[3], double p2[3], double p3[3], int level)
{
  int i;
  if (level > 0) {
    double q1[3],q2[3],q3[3];                 // sub-vertices
    for (i=0; i<3; i++) {
      q1[i] = 0.5f*(p1[i]+p2[i]);
      q2[i] = 0.5f*(p2[i]+p3[i]);
      q3[i] = 0.5f*(p3[i]+p1[i]);
    }
    double length1 = (double)(1.0/sqrt(q1[0]*q1[0]+q1[1]*q1[1]+q1[2]*q1[2]));
    double length2 = (double)(1.0/sqrt(q2[0]*q2[0]+q2[1]*q2[1]+q2[2]*q2[2]));
    double length3 = (double)(1.0/sqrt(q3[0]*q3[0]+q3[1]*q3[1]+q3[2]*q3[2]));
    for (i=0; i<3; i++) {
      q1[i] *= length1;
      q2[i] *= length2;
      q3[i] *= length3;
    }
    drawPatch (p1,q1,q3,level-1);
    drawPatch (q1,p2,q2,level-1);
    drawPatch (q1,q2,q3,level-1);
    drawPatch (q3,q2,p3,level-1);
  }
  else {
    glNormal3f (p1[0],p1[1],p1[2]);
    glVertex3f (p1[0],p1[1],p1[2]);
    glNormal3f (p2[0],p2[1],p2[2]);
    glVertex3f (p2[0],p2[1],p2[2]);
    glNormal3f (p3[0],p3[1],p3[2]);
    glVertex3f (p3[0],p3[1],p3[2]);
  }
}

void ViewPort3D::drawCappedCylinder(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const float radius,
                       const float length,
                       const PixRGB<byte> color)
{

  glPushMatrix();

  setColor(color);

  glShadeModel (GL_SMOOTH);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  // number of sides to the cylinder (divisible by 4):
  const int n = itsCylinderQuality*4;

  float halfLength = length * 0.5;

  double a = double(M_PI*2.0)/double(n);
  double sa = (double) sin(a);
  double ca = (double) cos(a);

  // draw cylinder body
  double ny=1, nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (int i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius,nz*radius,halfLength);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius,nz*radius,-halfLength);
    // rotate ny,nz
    double tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw first cylinder cap
  double start_nx = 0;
  double start_ny = 1;
  for (int j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    double start_nx2 =  ca*start_nx + sa*start_ny;
    double start_ny2 = -sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    double nx = start_nx, ny = start_ny, nz = 0;
    double nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (int i=0; i<=n; i++) {
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*radius,nz2*radius,halfLength+nx2*radius);
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*radius,nz*radius,halfLength+nx*radius);
      // rotate n,n2
      double tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  // draw second cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (int j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    double start_nx2 = ca*start_nx - sa*start_ny;
    double start_ny2 = sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    double nx = start_nx, ny = start_ny, nz = 0;
    double nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (int i=0; i<=n; i++) {
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*radius,nz*radius,-halfLength+nx*radius);
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*radius,nz2*radius,-halfLength+nx2*radius);
      // rotate n,n2
      double tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  glPopMatrix();
}

void ViewPort3D::drawCylinder(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const float radius,
                       const float length,
                       const PixRGB<byte> color)
{

  glPushMatrix();

  setColor(color);

  glShadeModel (GL_SMOOTH);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  // number of sides to the cylinder (divisible by 4):
  const int n = itsCylinderQuality*4;

  float halfLength = length * 0.5;

  double a = double(M_PI*2.0)/double(n);
  double sa = (double) sin(a);
  double ca = (double) cos(a);

  // draw cylinder body
  double ny=1, nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (int i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius,nz*radius,halfLength);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius,nz*radius,-halfLength);
    // rotate ny,nz
    double tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw top cap
  glShadeModel (GL_FLAT);
  ny=1, nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,halfLength);
  for (int i=0; i<=n; i++) {
    //if (i==1 || i==n/2+1)
    //  setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,1);
    glVertex3d (ny*radius,nz*radius,halfLength);
    //if (i==1 || i==n/2+1)
    //  setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    double tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw bottom cap
  ny=1; nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,-1);
  glVertex3d (0,0,-halfLength);
  for (int i=0; i<=n; i++) {
    //if (i==1 || i==n/2+1)
    //  setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,-1);
    glVertex3d (ny*radius,nz*radius,-halfLength);
    //if (i==1 || i==n/2+1)
    //  setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    double tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();


  glPopMatrix();
}

void ViewPort3D::drawCone(const Point3D<float> pos,
                       const Point3D<float> rot, //Rotation
                       const float radius,
                       const float length,
                       const PixRGB<byte> color)
{

  glPushMatrix();

  setColor(color);

  glShadeModel (GL_SMOOTH);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  // number of sides to the cylinder (divisible by 4):
  const int n = itsCylinderQuality*4;

  float halfLength = length * 0.5;

  double a = double(M_PI*2.0)/double(n);
  double sa = (double) sin(a);
  double ca = (double) cos(a);

  glShadeModel (GL_FLAT);

  glFrontFace(GL_CW);
  // draw bottom cap
  double ny=1; double nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,-halfLength);
  for (int i=0; i<=n; i++) {
    glNormal3d (0,0,1);
    glVertex3d (ny*radius,nz*radius,-halfLength);
    // rotate ny,nz
    double tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  glFrontFace(GL_CCW);
  // draw body
  ny=1; nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,-1);
  glVertex3d (0,0,halfLength);
  for (int i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius,nz*radius,-halfLength);
    // rotate ny,nz
    double tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  glPopMatrix();
}


void ViewPort3D::drawGround(const Point2D<float> size, const PixRGB<byte> color)
{

  glPushMatrix();
  setColor(color);
  glShadeModel (GL_FLAT);
  glTranslatef(0, 0, 0);

  glBegin(GL_QUADS);

  glNormal3f(0,0,1);
  glVertex3f( size.i, size.j, 0);                        // Top Right Of The Quad (Front)
  glVertex3f(-size.i, size.j, 0);                        // Top Left Of The Quad (Front)
  glVertex3f(-size.i,-size.j, 0);                        // Bottom Left Of The Quad (Front)
  glVertex3f( size.i,-size.j, 0);                        // Bottom Right Of The Quad (Front)

  glEnd();                                                          // Done Drawing The Quad

  glPopMatrix();

}




void ViewPort3D::drawExtrudedContour(const std::vector<Point2D<float> >& contour,
    const Point3D<float> pos,
    const Point3D<float> rot,
    const float thickness,
    const PixRGB<byte> color)
{
  std::vector<Point2D<float> > triangles = triangulate(contour);

  glPushMatrix();
  setColor(color);

  glShadeModel (GL_SMOOTH);

  glTranslatef(pos.x, pos.y, pos.z);
  glRotatef(rot.x, 1,0,0);
  glRotatef(rot.y, 0,1,0);
  glRotatef(rot.z, 0,0,1);

  //Draw the first side
  glFrontFace(GL_CW);
  glBegin(GL_TRIANGLES);
  glNormal3f(0,0,-1);
  for(uint i=0; i<triangles.size(); i++)
  {
    glVertex3f(triangles[i].i, triangles[i].j,-thickness/2);                        // Top Right Of The Quad (Top)
  }
  glEnd();

  glBegin(GL_QUAD_STRIP);
  uint numPoints = contour.size();
  for(uint i=0; i<numPoints; i++)
  {
    glVertex3f(contour[i].i, contour[i].j,-thickness/2);                        // Top Right Of The Quad (Top)
    glVertex3f(contour[i].i, contour[i].j,thickness/2);                        // Top Right Of The Quad (Top)
    glVertex3f(contour[(i+1)%numPoints].i, contour[(i+1)%numPoints].j,-thickness/2);                        // Top Right Of The Quad (Top)
    glVertex3f(contour[(i+1)%numPoints].i, contour[(i+1)%numPoints].j,thickness/2);                        // Top Right Of The Quad (Top)

    double dx = contour[(i+1)%numPoints].j - contour[i].j;
    double dy = contour[i].i - contour[(i+1)%numPoints].i;
    double len = sqrt(dx * dx + dy * dy);
    glNormal3f(dx / len, dy / len, 0.0);
  }
  glEnd();

  glFrontFace(GL_CCW);
  ////Draw the next side
  glBegin(GL_TRIANGLES);
  glNormal3f(0,0,1);
  for(uint i=0; i<triangles.size(); i++)
  {
    //glNormal3f(0,0,1);
    glVertex3f(triangles[i].i, triangles[i].j, thickness/2);                        // Top Right Of The Quad (Top)
  }
  glEnd();

  glPopMatrix();
}


bool ViewPort3D::snip(const std::vector<Point2D<float> > &contour,
                            int u,int v,int w,int n,int *V)
{

  Point2D<float> A = contour[V[u]];
  Point2D<float> B = contour[V[v]];
  Point2D<float> C = contour[V[w]];

  const float EPSILON=0.0000000001f;

  if ( EPSILON > (((B.i-A.i)*(C.j-A.j)) - ((B.j-A.j)*(C.i-A.i))) ) return false;

  for (int p=0;p<n;p++)
  {
    if( (p == u) || (p == v) || (p == w) ) continue;
    Point2D<float> P = contour[V[p]];
    if (pnTriangle(A,B,C,P)) return false;
  }

  return true;
}

//Borowed from John W. Ratcliff [jratcliff@verant.com] and adapted to work with Point2D class
//http://www.flipcode.com/archives/Efficient_Polygon_Triangulation.shtml
std::vector<Point2D<float> > ViewPort3D::triangulate(const std::vector<Point2D<float> >& contour)
{
  std::vector<Point2D<float> > result;

  int n = contour.size();
  if ( n < 3 ) return result;

  int *V = new int[n];

  /* we want a counter-clockwise polygon in V */

  if ( 0.0f < area(contour) )
    for (int v=0; v<n; v++) V[v] = v;
  else
    for(int v=0; v<n; v++) V[v] = (n-1)-v;

  int nv = n;

  /*  remove nv-2 Vertices, creating 1 triangle every time */
  int count = 2*nv;   /* error detection */

  for(int m=0, v=nv-1; nv>2; )
  {
    /* if we loop, it is probably a non-simple polygon */
    if (0 >= (count--))
    {
      //** Triangulate: ERROR - probable bad polygon!
      return result;
    }

    /* three consecutive vertices in current polygon, <u,v,w> */
    int u = v  ; if (nv <= u) u = 0;     /* previous */
    v = u+1; if (nv <= v) v = 0;     /* new v    */
    int w = v+1; if (nv <= w) w = 0;     /* next     */

    if ( snip(contour,u,v,w,nv,V) )
    {
      int a,b,c,s,t;

      /* true names of the vertices */
      a = V[u]; b = V[v]; c = V[w];

      /* output Triangle */
      result.push_back( contour[a] );
      result.push_back( contour[b] );
      result.push_back( contour[c] );

      m++;

      /* remove v from remaining polygon */
      for(s=v,t=v+1;t<nv;s++,t++) V[s] = V[t]; nv--;

      /* resest error detection counter */
      count = 2*nv;
    }
  }

  delete V;

  return result;

}

uint ViewPort3D::addTexture(const Image<PixRGB<byte> >& textureImg)
{
  uint texId;
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glGenTextures(1, &texId);
  glBindTexture(GL_TEXTURE_2D, texId);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


  unsigned char* arrayPtr =  const_cast<unsigned char*>
    (reinterpret_cast<const unsigned char*> (textureImg.getArrayPtr()));
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
      textureImg.getWidth(), textureImg.getHeight(),
      0, GL_RGB, GL_UNSIGNED_BYTE, 
      arrayPtr);

  return texId;

}

uint ViewPort3D::addTexture(const Image<float>& textureImg)
{
  uint texId;
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glGenTextures(1, &texId);
  glBindTexture(GL_TEXTURE_2D, texId);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);


  unsigned char* arrayPtr =  const_cast<unsigned char*>
    (reinterpret_cast<const unsigned char*> (textureImg.getArrayPtr()));
  glTexImage2D(GL_TEXTURE_2D, 0, GL_FLOAT,
      textureImg.getWidth(), textureImg.getHeight(),
      0, GL_LUMINANCE, GL_UNSIGNED_BYTE, 
      arrayPtr);

  return texId;

}

void ViewPort3D::loadTexture(const Image<PixRGB<byte> >& textureImg, uint texId)
{
  glBindTexture(GL_TEXTURE_2D, texId);

  unsigned char* arrayPtr =  const_cast<unsigned char*>
    (reinterpret_cast<const unsigned char*> (textureImg.getArrayPtr()));
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
      textureImg.getWidth(), textureImg.getHeight(),
      0, GL_RGB, GL_UNSIGNED_BYTE, 
      arrayPtr);

}


void ViewPort3D::bindTexture(const uint texId)
{
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  glBindTexture(GL_TEXTURE_2D, texId);

}

#ifdef HAVE_GL_GLEW_H

void ViewPort3D::printInfoLog(GLhandleARB obj)
{
  int infologLength = 0;
  int charsWritten  = 0;
  char *infoLog;

  glGetObjectParameterivARB(obj, GL_OBJECT_INFO_LOG_LENGTH_ARB,
      &infologLength);

  if (infologLength > 0)
  {
    infoLog = (char *)malloc(infologLength);
    glGetInfoLogARB(obj, infologLength, &charsWritten, infoLog);
    LINFO("%s",infoLog);
    free(infoLog);
  }
  

}
//! Shader support
GLhandleARB ViewPort3D::createShader(const char *prog, int type)
{

  if (!itsInitGlew)
  {
    glewInit();
    if (GLEW_ARB_vertex_shader && GLEW_ARB_fragment_shader)
      LINFO("Ready for GLSL\n");
    else
      LFATAL("No GLSL support\n");
    itsInitGlew=true;
  }
  

  //Read the text file into memory

  FILE *fp = fopen(prog, "rt");
  if (fp == NULL)
    LFATAL("Can not find shader program %s", prog);

  //Find the size of the prog
  fseek(fp, 0, SEEK_END);
  int count = ftell(fp);
  rewind(fp);

  char* progContent = NULL;
  if (count > 0)
  {
    progContent = (char *)malloc(sizeof(char) * (count+1));
    count = fread(progContent, sizeof(char), count, fp);
    progContent[count] = '\0';
  }
  fclose(fp);

  if (progContent == NULL)
    LFATAL("Can not load program");

  GLhandleARB h;

  h = glCreateShaderObjectARB(type);

  LINFO("Add program\n");
  glShaderSourceARB(h, 1, (const char**)&progContent,NULL);
  LINFO("Done");

  glCompileShaderARB(h);

  LINFO("Compiling Shader");
  printInfoLog(h);

  GLhandleARB p = glCreateProgramObjectARB();
  glAttachObjectARB(p,h);
  glLinkProgramARB(p);
  printInfoLog(p);
  glUseProgramObjectARB(p);

  return p;
}

void ViewPort3D::progToTexture(const GLhandleARB prog, const int texId)

{
	glUseProgramObjectARB(prog);
	initProjection();
  glBegin(GL_QUADS);
  glTexCoord2f(0.0, 0.0); glVertex3f(0, 0, 0);
  glTexCoord2f(1.0, 0.0); glVertex3f(320, 0, 0);
  glTexCoord2f(1.0, 1.0); glVertex3f(320, 240 , 0);
  glTexCoord2f(0.0, 1.0); glVertex3f(0, 240, 0);
  glEnd();
  
	glFlush();
  if (texId != -1) //Render to a diffrent texture
		glBindTexture(GL_TEXTURE_2D, texId);
	glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 5, 5, 0, 0, 320-10,240-10);


}

#endif





// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif

