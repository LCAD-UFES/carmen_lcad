/*!@file GUI/Viewport.C test opengl viewport */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/ViewPort.C $
// $Id: ViewPort.C 14563 2011-03-02 01:09:49Z dberg $


#include "GUI/ViewPort.H"
#include <math.h>
#include <pthread.h>
#include <sys/stat.h>

//TODO: we need to only draw a new camera position of the current rendering. NOt rander the whole thing. Can Use glViewPort

ViewPort::ViewPort(const char *winname,
    const char* ground, const char* sky, bool useFog,
    bool drawWorld,
    int w, int h,
    const double cameraParam[3][4] ) :
  itsWidth(w),
  itsHeight(h),
  run(0),
  display(0),
  visual(0),
  colormap(0),
  win(0),
  glx_context(0),
  last_key_pressed(0),
  wm_protocols_atom(0),
  wm_delete_window_atom(0),
  screen(0),
  sky_texture(0),
  ground_texture(0),
  wood_texture(0),
  tree_texture(0),
  other_texture(0),
  listnum(0),
  itsDrawWorld(drawWorld),
  itsUseFog(useFog),
  itsWireframe(false),
  itsZoomFactor(0.8)
{
  createMainWindow (winname);

  glXMakeCurrent (display,win,glx_context);
  startGraphics("./etc/textures/", ground, sky);

  for(int i=0; i<4; i++)
    color[i] = 0;
  tnum = 0;                        // current texture number

  ground_scale = 1.0f/20.0f;        // ground texture scale (1/size)
  ground_ofsx = 0.5;                // offset of ground texture
  ground_ofsy = 0.5;
  sky_scale = 1.0f/4.0f;        // sky texture scale (1/size)
  sky_height = 1.0f;                // sky height above viewpoint

  sphere_quality = 1;
  capped_cylinder_quality = 3;

  if (itsDrawWorld)
  {
    use_textures=1;                // 1 if textures to be drawn
    use_shadows=1;                // 1 if shadows to be drawn
  } else {
    use_textures=0;                // 1 if textures to be drawn
    use_shadows=0;                // 1 if shadows to be drawn
  }


  initCamera();                //set the camera to initail position

  if (cameraParam != NULL)
    buildProjectionMatrix(cameraParam, itsProjMatrix);

  run = 1;
  //int rc = pthread_create(&win_thread, NULL, ViewPort::mainWindowThread, this);
  //if (rc){
  //        LFATAL("Error creating main window thread\n");
  //}
  //mainWindowThread(this);
}

ViewPort::~ViewPort(){
  stopGraphics();

  destroyMainWindow();
}

void ViewPort::setTextures(bool val)
{
  if (val)
    use_textures=1;
  else
    use_textures=0;
}

void ViewPort::setShadows(bool val)
{
  if (val)
    use_shadows=1;
  else
    use_shadows=0;
}


void ViewPort::createMainWindow (const char *winname){

  display = XOpenDisplay (NULL);
  if (!display) LFATAL("can not open X11 display");

  screen = DefaultScreen(display);


  // get GL visual
  static int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE,16,
    GLX_RED_SIZE,4, GLX_GREEN_SIZE,4,
    GLX_BLUE_SIZE,4, None};
  visual = glXChooseVisual (display,screen,attribList);
  if (!visual) LFATAL("no good X11 visual found for OpenGL");

  // create colormap
  colormap = XCreateColormap (display,RootWindow(display,screen),
      visual->visual,AllocNone);

  // initialize variables
  win = 0;
  //glx_context = 0;
  last_key_pressed = 0;

  if (itsWidth < 1 || itsHeight < 1) LDEBUG ("bad window width or height");

  // create the window
  XSetWindowAttributes attributes;
  attributes.background_pixel = BlackPixel(display,screen);
  attributes.colormap = colormap;
  attributes.event_mask = ButtonPressMask | ButtonReleaseMask |
    KeyPressMask | KeyReleaseMask | ButtonMotionMask | PointerMotionHintMask |
    StructureNotifyMask;
  win = XCreateWindow (display,RootWindow(display,screen),50,50,itsWidth,itsHeight,
      0,visual->depth, InputOutput,visual->visual,
      CWBackPixel | CWColormap | CWEventMask,&attributes);

  // associate a GLX context with the window
  glx_context = glXCreateContext (display,visual,0,GL_TRUE);
  if (!glx_context) LFATAL ("can't make an OpenGL context");

  // set the window title
  XTextProperty window_name;
  window_name.value = (unsigned char *)winname; //(unsigned char *) "Simulation";
  window_name.encoding = XA_STRING;
  window_name.format = 8;
  window_name.nitems = strlen((char *) window_name.value);
  XSetWMName (display,win,&window_name);

  // participate in the window manager 'delete yourself' protocol
  wm_protocols_atom = XInternAtom (display,"WM_PROTOCOLS",False);
  wm_delete_window_atom = XInternAtom (display,"WM_DELETE_WINDOW",False);
  if (XSetWMProtocols (display,win,&wm_delete_window_atom,1)==0)
    LFATAL ("XSetWMProtocols() call failed");

  // pop up the window
  XMapWindow (display,win);
  XSync (display,win);

}

void ViewPort::destroyMainWindow()
{
  glXDestroyContext (display,glx_context);
  XDestroyWindow (display,win);
  XSync (display,0);
  display = 0;
  win = 0;
  glx_context = NULL;
}


void ViewPort::startGraphics (const char *prefix, const char *ground, const char *sky){
  // All examples build into the same dir
  char *s =  new char[strlen(prefix) + 20];

  strcpy (s,prefix);
  strcat (s,sky);
  sky_texture = new Texture (s);

  strcpy (s,prefix);
  strcat (s,ground);
  ground_texture = new Texture (s);

  strcpy (s,prefix);
  strcat (s,"wood.ppm");
  wood_texture = new Texture (s);

  strcpy (s,prefix);
  strcat (s,"tree.ppm");
  tree_texture = new Texture (s);
}

void ViewPort::stopGraphics()
{
  if (sky_texture) delete sky_texture;
  if (ground_texture) delete ground_texture;
  if (wood_texture) delete wood_texture;
  if (tree_texture) delete tree_texture;
  if (other_texture) delete other_texture;
  sky_texture = 0;
  ground_texture = 0;
  tree_texture = 0;
  other_texture = 0;
}

void ViewPort::handleEvent (XEvent &event)
{
  static int mx=0,my=0;         // mouse position
  static int mode = 0;                // mouse button bits

  //LDEBUG("Handeling event %i\n", event.type);

  switch (event.type) {

    case ButtonPress: {
                        if (event.xbutton.button == Button1) mode |= 1;
                        if (event.xbutton.button == Button2) mode |= 2;
                        if (event.xbutton.button == Button3) mode |= 4;
                        mx = event.xbutton.x;
                        my = event.xbutton.y;
                      }
                      return;

    case ButtonRelease: {
                          if (event.xbutton.button == Button1) mode &= (~1);
                          if (event.xbutton.button == Button2) mode &= (~2);
                          if (event.xbutton.button == Button3) mode &= (~4);
                          mx = event.xbutton.x;
                          my = event.xbutton.x;
                        }
                        return;

    case MotionNotify: {
                         if (event.xmotion.is_hint) {
                           Window root,child;
                           unsigned int mask;
                           XQueryPointer (display,win,&root,&child,&event.xbutton.x_root,
                               &event.xbutton.y_root,&event.xbutton.x,&event.xbutton.y,
                               &mask);
                         }
                         dsMotion (mode, event.xmotion.x - mx, event.xmotion.y - my);
                         mx = event.xmotion.x;
                         my = event.xmotion.y;
                       }
                       return;

    case ClientMessage:
                       if (event.xclient.message_type == wm_protocols_atom &&
                           event.xclient.format == 32 &&
                           Atom(event.xclient.data.l[0]) == wm_delete_window_atom) {
                         run = 0;
                         return;
                       }
                       return;

    case ConfigureNotify:
                       itsWidth = event.xconfigure.width;
                       itsHeight = event.xconfigure.height;
                       return;
  }
}

void ViewPort::initCamera()
{
  view_xyz[0] = 0;
  view_xyz[1] = 5;
  view_xyz[2] = 5;
  view_hpr[0] = -100;
  view_hpr[1] = -30;
  view_hpr[2] = 0;
}


//!init the frame and process window events before drawing stuff
XEvent ViewPort::initFrame(const double* cameraParam){

  ////process enevent for the window
  XEvent event;
  while (run && XPending (display)) {
    XNextEvent (display,&event);
    handleEvent (event);
  }

  dsDrawFrame (cameraParam);

  return event;
}


//! update the frame and return if we are running or not
int ViewPort::updateFrame(){

  glFlush();
  glXSwapBuffers (display,win);
  XSync (display,0);

  return run;

}

void ViewPort::getFrame(unsigned char *img){
  glXMakeCurrent (display,win,glx_context);
  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glReadBuffer(GL_BACK_LEFT);
  glReadPixels (0, 0, itsWidth, itsHeight, GL_RGB, GL_UNSIGNED_BYTE, img);
}

Image<PixRGB<byte> > ViewPort::getFrame(){

  Image<PixRGB<byte> > retImg(itsWidth, itsHeight, NO_INIT);
  glXMakeCurrent (display,win,glx_context);
  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glReadBuffer(GL_BACK_LEFT);
  glReadPixels (0, 0, itsWidth, itsHeight, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)retImg.getArrayPtr());

  return retImg;
}

void ViewPort::mainWindowThread(void *data){
  ViewPort *vp = (ViewPort *)data;
  vp->run = 1;
  while(vp->run){
    XEvent event;
    while (vp->run && XPending (vp->display)) {
      XNextEvent (vp->display,&event);
      vp->handleEvent (event);
    }

    //vp->dsDrawFrame ();

    glFlush();
    glXSwapBuffers (vp->display,vp->win);
    XSync (vp->display,0);

    // capture frames if necessary
    if (vp->writeframes) {
      //captureFrame (frame);
      //frame++;
    }
  }

}

inline double ViewPort::dDOT (const double *a, const double *b)
{ return ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2]); }


void ViewPort::normalizeVector3 (float v[3])
{
  double len = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
  if (len <= 0.0f) {
    v[0] = 1;
    v[1] = 0;
    v[2] = 0;
  }
  else {
    len = 1.0f / (double)sqrt(len);
    v[0] *= len;
    v[1] *= len;
    v[2] *= len;
  }
}

//***************************************************************************
// OpenGL utility stuff

void ViewPort::buildProjectionMatrix(const double cameraParam[3][4], double projMatrix[16])
{
  //Borrowed from ARTToolkit

  double   icpara[3][4];
  double   trans[3][4];
  double   p[3][3], q[4][4];
  int      i, j;
  double gnear = 50;
  double gfar = 5000;

  if( paramDecompMat(cameraParam, icpara, trans) < 0 )
    LFATAL("Camera Parameter error!!\n");

  for( i = 0; i < 3; i++ ) {
    for( j = 0; j < 3; j++ ) {
      p[i][j] = icpara[i][j] / icpara[2][2];
    }
  }
  q[0][0] = (2.0 * p[0][0] / itsWidth);
  q[0][1] = (2.0 * p[0][1] / itsWidth);
  q[0][2] = ((2.0 * p[0][2] / itsWidth)  - 1.0);
  q[0][3] = 0.0;

  q[1][0] = 0.0;
  q[1][1] = (2.0 * p[1][1] / itsHeight);
  q[1][2] = ((2.0 * p[1][2] / itsHeight) - 1.0);
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
      projMatrix[i+j*4] = q[i][0] * trans[0][j]
        + q[i][1] * trans[1][j]
        + q[i][2] * trans[2][j];
    }
    projMatrix[i+3*4] = q[i][0] * trans[0][3]
      + q[i][1] * trans[1][3]
      + q[i][2] * trans[2][3]
      + q[i][3];
  }

}

#define NORM(a,b,c) sqrt(a*a + b*b + c*c)
#define DOT(a1,a2,a3,b1,b2,b3) (a1*b1 + a2*b2 + a3*b3)

int  ViewPort::paramDecompMat(const double source[3][4], double cpara[3][4], double trans[3][4] )
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



void ViewPort::setCamera (double x, double y, double z, double h, double p, double r)
{
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  glRotatef (90, 0,0,1);
  glRotatef (90, 0,1,0);
  glRotatef (r, 1,0,0);
  glRotatef (p, 0,1,0);
  glRotatef (-h, 0,0,1);
  glTranslatef (-x,-y,-z);
}


// sets the material color, not the light color

void ViewPort::setColor (double r, double g, double b, double alpha)
{
  GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
  light_ambient[0] = r*0.3f;
  light_ambient[1] = g*0.3f;
  light_ambient[2] = b*0.3f;
  light_ambient[3] = alpha;
  light_diffuse[0] = r*0.7f;
  light_diffuse[1] = g*0.7f;
  light_diffuse[2] = b*0.7f;
  light_diffuse[3] = alpha;
  light_specular[0] = r*0.2f;
  light_specular[1] = g*0.2f;
  light_specular[2] = b*0.2f;
  light_specular[3] = alpha;
  glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
  glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
  glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
  glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0f);
}


void ViewPort::setTransform (const double pos[3], const double R[12])
{
  GLfloat matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}

void ViewPort::unProjectPoint(const int x, const int y, double objLoc[3])
{

  //From http://nehe.gamedev.net/data/articles/article.asp?article=13
  GLint viewport[4];
  GLdouble modelview[16];
  GLdouble projection[16];
  GLfloat winX=0, winY=0, winZ=0;
  GLdouble posX=0, posY=0, posZ=0;

  glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
  glGetDoublev( GL_PROJECTION_MATRIX, projection );
  glGetIntegerv( GL_VIEWPORT, viewport );

  winX = (float)x;
  winY = (float)viewport[3] - (float)y;
  glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );


#ifdef INVT_HAVE_LIBGLUT
  gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
#endif

  objLoc[0] = posX;
  objLoc[1] = posY;
  objLoc[2] = posZ;

}

// set shadow projection transform

void ViewPort::setShadowTransform()
{
  GLfloat matrix[16];
  for (int i=0; i<16; i++) matrix[i] = 0;
  matrix[0]=1;
  matrix[5]=1;
  matrix[8]=-LIGHTX;
  matrix[9]=-LIGHTY;
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixf (matrix);
}


void ViewPort::drawBox (const double sides[3])
{
  double lx = sides[0]*0.5f;
  double ly = sides[1]*0.5f;
  double lz = sides[2]*0.5f;

  glBegin(GL_QUADS);
  glNormal3f(0,1,0);
  glVertex3f( lx, ly,-lz);                        // Top Right Of The Quad (Top)
  glVertex3f(-lx, ly,-lz);                        // Top Left Of The Quad (Top)
  glVertex3f(-lx, ly, lz);                        // Bottom Left Of The Quad (Top)
  glVertex3f( lx, ly, lz);                        // Bottom Right Of The Quad (Top)

  glNormal3f(0,-1,0);
  glVertex3f( lx,-ly, lz);                        // Top Right Of The Quad (Bottom)
  glVertex3f(-lx,-ly, lz);                        // Top Left Of The Quad (Bottom)
  glVertex3f(-lx,-ly,-lz);                        // Bottom Left Of The Quad (Bottom)
  glVertex3f( lx,-ly,-lz);                        // Bottom Right Of The Quad (Bottom)

  glNormal3f(0,0,1);
  glVertex3f( lx, ly, lz);                        // Top Right Of The Quad (Front)
  glVertex3f(-lx, ly, lz);                        // Top Left Of The Quad (Front)
  glVertex3f(-lx,-ly, lz);                        // Bottom Left Of The Quad (Front)
  glVertex3f( lx,-ly, lz);                        // Bottom Right Of The Quad (Front)

  glNormal3f(0,0,-1);
  glVertex3f( lx,-ly,-lz);                        // Bottom Left Of The Quad (Back)
  glVertex3f(-lx,-ly,-lz);                        // Bottom Right Of The Quad (Back)
  glVertex3f(-lx, ly,-lz);                        // Top Right Of The Quad (Back)
  glVertex3f( lx, ly,-lz);                        // Top Left Of The Quad (Back)

  glNormal3f(-1,0,0);
  glVertex3f(-lx, ly, lz);                        // Top Right Of The Quad (Left)
  glVertex3f(-lx, ly,-lz);                        // Top Left Of The Quad (Left)
  glVertex3f(-lx,-ly,-lz);                        // Bottom Left Of The Quad (Left)
  glVertex3f(-lx,-ly, lz);                        // Bottom Right Of The Quad (Left)

  glNormal3f(1,0,0);
  glVertex3f( lx, ly,-lz);                        // Top Right Of The Quad (Right)
  glVertex3f( lx, ly, lz);                        // Top Left Of The Quad (Right)
  glVertex3f( lx,-ly, lz);                        // Bottom Left Of The Quad (Right)
  glVertex3f( lx,-ly,-lz);                        // Bottom Right Of The Quad (Right)
  glEnd();                                                          // Done Drawing The Quad

}


// This is recursively subdivides a triangular area (vertices p1,p2,p3) into
// smaller triangles, and then draws the triangles. All triangle vertices are
// normalized to a distance of 1.0 from the origin (p1,p2,p3 are assumed
// to be already normalized). Note this is not super-fast because it draws
// triangles rather than triangle strips.

void ViewPort::drawPatch (double p1[3], double p2[3], double p3[3], int level)
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

// draw a sphere of radius 1

void ViewPort::drawSphere()
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

  if (listnum==0) {
    listnum = glGenLists (1);
    glNewList (listnum,GL_COMPILE);
    glBegin (GL_TRIANGLES);
    for (int i=0; i<20; i++) {
      drawPatch (&idata[index[i][2]][0],&idata[index[i][1]][0],
          &idata[index[i][0]][0],sphere_quality);
    }
    glEnd();
    glEndList();
  }
  glCallList (listnum);
}



void ViewPort::drawSphereShadow (double px, double py, double pz, double radius)
{
  // calculate shadow constants based on light vector
  static int init=0;
  static double len2,len1,scale;
  if (!init) {
    len2 = LIGHTX*LIGHTX + LIGHTY*LIGHTY;
    len1 = 1.0f/(double)sqrt(len2);
    scale = (double) sqrt(len2 + 1);
    init = 1;
  }

  // map sphere center to ground plane based on light vector
  px -= LIGHTX*pz;
  py -= LIGHTY*pz;

  const double kx = 0.96592582628907f;
  const double ky = 0.25881904510252f;
  double x=radius, y=0;

  glBegin (GL_TRIANGLE_FAN);
  for (int i=0; i<24; i++) {
    // for all points on circle, scale to elongated rotated shadow and draw
    double x2 = (LIGHTX*x*scale - LIGHTY*y)*len1 + px;
    double y2 = (LIGHTY*x*scale + LIGHTX*y)*len1 + py;
    glTexCoord2f (x2*ground_scale+ground_ofsx,y2*ground_scale+ground_ofsy);
    glVertex3f (x2,y2,0);

    // rotate [x,y] vector
    double xtmp = kx*x - ky*y;
    y = ky*x + kx*y;
    x = xtmp;
  }
  glEnd();
}


void ViewPort::drawTriangle (const double *v0, const double *v1, const double *v2, int solid)
{
  float u[3],v[3],normal[3];
  u[0] = v1[0] - v0[0];
  u[1] = v1[1] - v0[1];
  u[2] = v1[2] - v0[2];
  v[0] = v2[0] - v0[0];
  v[1] = v2[1] - v0[1];
  v[2] = v2[2] - v0[2];

  normal[0] = (u[1]*v[2] - u[2]*v[1]);
  normal[1] = (u[2]*v[0] - u[0]*v[2]);
  normal[2] = (u[0]*v[1] - u[1]*v[0]);

  normalizeVector3 (normal);

  glBegin(solid ? GL_TRIANGLES : GL_LINE_STRIP);
  glNormal3fv (normal);
  glVertex3dv (v0);
  glVertex3dv (v1);
  glVertex3dv (v2);
  glEnd();
}

void ViewPort::drawTriangleD (const double *v0, const double *v1, const double *v2, int solid)
{
  float u[3],v[3],normal[3];
  u[0] = double( v1[0] - v0[0] );
  u[1] = double( v1[1] - v0[1] );
  u[2] = double( v1[2] - v0[2] );
  v[0] = double( v2[0] - v0[0] );
  v[1] = double( v2[1] - v0[1] );
  v[2] = double( v2[2] - v0[2] );

  normal[0] = (u[1]*v[2] - u[2]*v[1]);
  normal[1] = (u[2]*v[0] - u[0]*v[2]);
  normal[2] = (u[0]*v[1] - u[1]*v[0]);

  normalizeVector3 (normal);

  glBegin(solid ? GL_TRIANGLES : GL_LINE_STRIP);
  glNormal3fv (normal);
  glVertex3dv (v0);
  glVertex3dv (v1);
  glVertex3dv (v2);
  glEnd();
}


// draw a capped cylinder of length l and radius r, aligned along the x axis

void ViewPort::drawCappedCylinder (double l, double r)
{
  int i,j;
  double tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
  // number of sides to the cylinder (divisible by 4):
  const int n = capped_cylinder_quality*4;

  l *= 0.5;
  a = double(M_PI*2.0)/double(n);
  sa = (double) sin(a);
  ca = (double) cos(a);

  // draw cylinder body
  ny=1; nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw first cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    double start_nx2 =  ca*start_nx + sa*start_ny;
    double start_ny2 = -sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    double nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,l+nx2*r);
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,l+nx*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
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
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    double start_nx2 = ca*start_nx - sa*start_ny;
    double start_ny2 = sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    double nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,-l+nx*r);
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,-l+nx2*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
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


// draw a cylinder of length l and radius r, aligned along the z axis

void ViewPort::drawCylinder (double l, double r, double zoffset)
{
  int i;
  double tmp,ny,nz,a,ca,sa;
  const int n = 24;        // number of sides to the cylinder (divisible by 4)

  l *= 0.5;
  a = double(M_PI*2.0)/double(n);
  sa = (double) sin(a);
  ca = (double) cos(a);

  // draw cylinder body
  ny=1; nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l+zoffset);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l+zoffset);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw top cap
  glShadeModel (GL_FLAT);
  ny=1; nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,1);
  glVertex3d (0,0,l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,1);
    glVertex3d (ny*r,nz*r,l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw bottom cap
  ny=1; nz=0;                  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_FAN);
  glNormal3d (0,0,-1);
  glVertex3d (0,0,-l+zoffset);
  for (i=0; i<=n; i++) {
    if (i==1 || i==n/2+1)
      setColor (color[0]*0.75f,color[1]*0.75f,color[2]*0.75f,color[3]);
    glNormal3d (0,0,-1);
    glVertex3d (ny*r,nz*r,-l+zoffset);
    if (i==1 || i==n/2+1)
      setColor (color[0],color[1],color[2],color[3]);

    // rotate ny,nz
    tmp = ca*ny + sa*nz;
    nz = -sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();
}

//***************************************************************************
// motion model

// initialize the above variables

void ViewPort::initMotionModel()
{
  view_xyz[0] = 2;
  view_xyz[1] = 0;
  view_xyz[2] = 1;
  view_hpr[0] = 180;
  view_hpr[1] = 0;
  view_hpr[2] = 0;
}


void ViewPort::wrapCameraAngles()
{
  for (int i=0; i<3; i++) {
    while (view_hpr[i] > 180) view_hpr[i] -= 360;
    while (view_hpr[i] < -180) view_hpr[i] += 360;
  }
}


// call this to update the current camera position. the bits in `mode' say
// if the left (1), middle (2) or right (4) mouse button is pressed, and
// (deltax,deltay) is the amount by which the mouse pointer has moved.

void ViewPort::dsMotion (int mode, int deltax, int deltay)
{
  double side = 0.01f * double(deltax);
  double fwd = (mode==4) ? (0.01f * double(deltay)) : 0.0f;
  double s = (double) sin (view_hpr[0]*DEG_TO_RAD);
  double c = (double) cos (view_hpr[0]*DEG_TO_RAD);

  LINFO("Movde %i", mode);
  switch(mode)
  {
    case 1:
      view_hpr[0] += double (deltax) * 0.5f;
      view_hpr[1] += double (deltay) * 0.5f;
      break;
    case 2:
    case 5:
     view_xyz[2] += 0.01f * double(deltay);
     view_hpr[2] += double (deltax) * 0.5f;
     break;
    case 4:
      view_xyz[0] += -s*side + c*fwd;
      view_xyz[1] += c*side + s*fwd;
      break;

  }
  wrapCameraAngles();

  LINFO("Camera pos:(%f,%f,%f) Rot:(%f,%f,%f)",
      view_xyz[0], view_xyz[1], view_xyz[2],
      view_hpr[0], view_hpr[1], view_hpr[2]);

}

//***************************************************************************
// drawing loop stuff

void ViewPort::drawSky (double view_xyz[3])
{
  glDisable (GL_LIGHTING);
  if (use_textures && sky_texture->initialized()) {
    glEnable (GL_TEXTURE_2D);
    sky_texture->bind (0);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (0,0.5,1.0);
  }

  // make sure sky depth is as far back as possible
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LEQUAL);
  glDepthRange (1,1);

  const double ssize = 1000.0f;
  static double offset = 0.0f;

  double x = ssize*sky_scale;
  double z = view_xyz[2] + sky_height;

  glBegin (GL_QUADS);
  glNormal3f (0,0,-1);
  glTexCoord2f (-x+offset,-x+offset);
  glVertex3f (-ssize+view_xyz[0],-ssize+view_xyz[1],z);
  glTexCoord2f (-x+offset,x+offset);
  glVertex3f (-ssize+view_xyz[0],ssize+view_xyz[1],z);
  glTexCoord2f (x+offset,x+offset);
  glVertex3f (ssize+view_xyz[0],ssize+view_xyz[1],z);
  glTexCoord2f (x+offset,-x+offset);
  glVertex3f (ssize+view_xyz[0],-ssize+view_xyz[1],z);
  glEnd();

  offset = offset + 0.001f;
  if (offset > 1) offset -= 1;

  glDepthFunc (GL_LESS);
  glDepthRange (0,1);
}


void ViewPort::drawGround()
{
  glDisable (GL_LIGHTING);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  // glDepthRange (1,1);

  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (0);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R,GROUND_G,GROUND_B);
  }

  if (itsUseFog)
  {
    // ground fog seems to cause problems with TNT2 under windows

    GLfloat fogColor[4] = {0.0, 0.2, 0.1, 1};
    glEnable (GL_FOG);
    glFogi (GL_FOG_MODE, GL_LINEAR);
    glFogfv (GL_FOG_COLOR, fogColor);
    glFogf (GL_FOG_DENSITY, 0.07f);
    glHint (GL_FOG_HINT, GL_NICEST); // GL_DONT_CARE);
    glFogf (GL_FOG_START, 0.0);
    glFogf (GL_FOG_END, 10.0);
  }


  const double gsize = 100.0f;
  const double offset = 0; // -0.001f; ... polygon offsetting doesn't work well

  glBegin (GL_QUADS);
  glNormal3f (0,0,1);

  glTexCoord2f (-gsize*ground_scale + ground_ofsx,
      -gsize*ground_scale + ground_ofsy);
  glVertex3f (-gsize,-gsize,offset);

  glTexCoord2f (gsize*ground_scale + ground_ofsx,
      -gsize*ground_scale + ground_ofsy);
  glVertex3f (gsize,-gsize,offset);

  glTexCoord2f (gsize*ground_scale + ground_ofsx,
      gsize*ground_scale + ground_ofsy);
  glVertex3f (gsize,gsize,offset);

  glTexCoord2f (-gsize*ground_scale + ground_ofsx,
      gsize*ground_scale + ground_ofsy);
  glVertex3f (-gsize,gsize,offset);
  glEnd();

  //  if (itsUseFog)
  //    glDisable (GL_FOG);
}


void ViewPort::drawPyramidGrid()
{
  // setup stuff
  glEnable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);

  // draw the pyramid grid
  for (int i=-1; i<=1; i++) {
    for (int j=-1; j<=1; j++) {
      glPushMatrix();
      glTranslatef ((double)i,(double)j,(double)0);
      if (i==1 && j==0) setColor (1,0,0,1);
      else if (i==0 && j==1) setColor (0,0,1,1);
      else setColor (1,1,0,1);
      const double k = 0.03f;
      glBegin (GL_TRIANGLE_FAN);
      glNormal3f (0,-1,1);
      glVertex3f (0,0,k);
      glVertex3f (-k,-k,0);
      glVertex3f ( k,-k,0);
      glNormal3f (1,0,1);
      glVertex3f ( k, k,0);
      glNormal3f (0,1,1);
      glVertex3f (-k, k,0);
      glNormal3f (-1,0,1);
      glVertex3f (-k,-k,0);
      glEnd();
      glPopMatrix();
    }
  }
}


void ViewPort::dsDrawFrame (const double cameraParam[16])
{
  glXMakeCurrent (display,win,glx_context);
  // setup stuff
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glDisable (GL_TEXTURE_2D);
  glDisable (GL_TEXTURE_GEN_S);
  glDisable (GL_TEXTURE_GEN_T);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  glEnable (GL_CULL_FACE);
  glCullFace (GL_BACK);
  glFrontFace (GL_CCW);

  if (itsWireframe)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //wireframe mode

  // setup viewport
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glViewport (0,0,itsWidth,itsHeight);

  glMatrixMode (GL_PROJECTION);

  if (itsProjMatrix==NULL) //Load from current view
  {
    glLoadIdentity();
    const double vnear = 50.1f;
    const double vfar = 5000.0f;
    const double k = itsZoomFactor;     // view scale, 1 = +/- 45 degrees


    if (itsWidth >= itsHeight) {
      double k2 = double(itsHeight)/double(itsWidth);
      glFrustum (-vnear*k,vnear*k,-vnear*k*k2,vnear*k*k2,vnear,vfar);
    }
    else {
      double k2 = double(itsWidth)/double(itsHeight);
      glFrustum (-vnear*k*k2,vnear*k*k2,-vnear*k,vnear*k,vnear,vfar);
    }
  } else { //Load the camera params
   glLoadMatrixd(itsProjMatrix);
  }

  // setup lights. it makes a difference whether this is done in the
  // GL_PROJECTION matrix mode (lights are scene relative) or the
  // GL_MODELVIEW matrix mode (lights are camera relative, bad!).
  static GLfloat light_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
  static GLfloat light_diffuse[] = { 0.0, 0.0, 0.0, 0.0 };
  static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
  glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
  glColor3f (0.0, 0.0, 0.0);

  // clear the window
  glClearColor (0.0,0.0,0.0,0);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // go to GL_MODELVIEW matrix mode and set the camera
  glMatrixMode (GL_MODELVIEW);

  if (cameraParam==NULL)
  {
    glLoadIdentity();
    setCamera (view_xyz[0],view_xyz[1],view_xyz[2],
        view_hpr[0],view_hpr[1],view_hpr[2]);
  } else {
    glLoadMatrixd(cameraParam );
  }


  // set the light position (for some reason we have to do this in model view.
  static GLfloat light_position[] = { LIGHTX, LIGHTY, 5.0 };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_position);
  glLightfv (GL_LIGHT0, GL_POSITION, light_position);

  // draw the background (ground, sky etc)
  if (itsDrawWorld)
  {
    drawSky (view_xyz);
    drawGround();
  }

  // draw the little markers on the ground
  //drawPyramidGrid();

  // leave openGL in a known state - flat shaded white, no textures
  glEnable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);
  glShadeModel (GL_FLAT);
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LESS);
  glColor3f (1,1,1);
  setColor (1,1,1,1);

  // draw the rest of the objects. set drawing state first.
  color[0] = 1;
  color[1] = 1;
  color[2] = 1;
  color[3] = 1;
  tnum = 0;
}

int ViewPort::dsGetShadows()
{
  return use_shadows;
}


void ViewPort::dsSetShadows (int a)
{
  use_shadows = (a != 0);
}


int ViewPort::dsGetTextures()
{
  return use_textures;
}


void ViewPort::dsSetTextures (int a)
{
  use_textures = (a != 0);
}

//***************************************************************************
// C interface

// sets lighting and texture modes, sets current color
void ViewPort::setupDrawingMode()
{
  glXMakeCurrent (display,win,glx_context);

  glEnable (GL_LIGHTING);
  bool enableWrap = true;
  if (tnum) {
    if (use_textures) {
      glEnable (GL_TEXTURE_2D);

      switch (tnum)
      {
        case SKY:
          sky_texture->bind (1);
          enableWrap = true;
          break;
        case GROUND:
          ground_texture->bind (1);
          enableWrap = true;
          break;
        case WOOD:
          wood_texture->bind (1);
          enableWrap = true;
          break;
        case TREE:
          tree_texture->bind (1);
          enableWrap = true;
          break;
        case OTHER:
          if (other_texture)
            other_texture->bind (1);
          //enableWrap = true;
          //glBindTexture(GL_TEXTURE_2D, tnum);
          enableWrap = false;
          break;
        default:
          LINFO("Unknown Texture %i", tnum);
          break;
      }

      if (enableWrap)
      {
        glEnable (GL_TEXTURE_GEN_S);
        glEnable (GL_TEXTURE_GEN_T);
        glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
        glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
        static GLfloat s_params[4] = {1.0f,1.0f,0.0f,1};
        static GLfloat t_params[4] = {0.817f,-0.817f,0.817f,1};
        glTexGenfv (GL_S,GL_OBJECT_PLANE,s_params);
        glTexGenfv (GL_T,GL_OBJECT_PLANE,t_params);
      }
    }
    else {
      glDisable (GL_TEXTURE_2D);
    }
  }
  else {
    glDisable (GL_TEXTURE_2D);
  }
  setColor (color[0],color[1],color[2],color[3]);

  if (color[3] < 1) {
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  }
  else {
    glDisable (GL_BLEND);
  }
}


void ViewPort::setShadowDrawingMode()
{
  glDisable (GL_LIGHTING);
  if (use_textures) {
    glEnable (GL_TEXTURE_2D);
    ground_texture->bind (1);
    glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    glEnable (GL_TEXTURE_2D);
    glEnable (GL_TEXTURE_GEN_S);
    glEnable (GL_TEXTURE_GEN_T);
    glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_EYE_LINEAR);
    static GLfloat s_params[4] = {(GLfloat)ground_scale,0,0,(GLfloat)ground_ofsx};
    static GLfloat t_params[4] = {0,(GLfloat)ground_scale,0,(GLfloat)ground_ofsy};
    glTexGenfv (GL_S,GL_EYE_PLANE,s_params);
    glTexGenfv (GL_T,GL_EYE_PLANE,t_params);
  }
  else {
    glDisable (GL_TEXTURE_2D);
    glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
        GROUND_B*SHADOW_INTENSITY);
  }
  glDepthRange (0,0.9999);
}


void ViewPort::dsSetViewpoint (double xyz[3], double hpr[3])
{
  if (xyz) {
    view_xyz[0] = xyz[0];
    view_xyz[1] = xyz[1];
    view_xyz[2] = xyz[2];
  }
  if (hpr) {
    view_hpr[0] = hpr[0];
    view_hpr[1] = hpr[1];
    view_hpr[2] = hpr[2];
    wrapCameraAngles();
  }
}


void ViewPort::dsGetViewpoint (double xyz[3], double hpr[3])
{
  if (xyz) {
    xyz[0] = view_xyz[0];
    xyz[1] = view_xyz[1];
    xyz[2] = view_xyz[2];
  }
  if (hpr) {
    hpr[0] = view_hpr[0];
    hpr[1] = view_hpr[1];
    hpr[2] = view_hpr[2];
  }
}


void ViewPort::dsSetTexture (TEXTURES texture_number, Texture* texturePtr)
{
  tnum = texture_number;
  if (texturePtr)
    other_texture = texturePtr;
}


void ViewPort::dsSetColor (double red, double green, double blue)
{
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = 1;
}


void ViewPort::dsSetColorAlpha (double red, double green, double blue,
    double alpha)
{
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  color[3] = alpha;
}


void ViewPort::dsDrawBox (const double pos[3], const double R[12],
    const double sides[3])
{

  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos,R);
  drawBox (sides);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawBox (sides);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


void ViewPort::dsDrawSphere (const double pos[3], const double R[12],
    double radius)
{
  setupDrawingMode();
  glEnable (GL_NORMALIZE);
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  glScaled (radius,radius,radius);
  drawSphere();
  glPopMatrix();
  glDisable (GL_NORMALIZE);

  // draw shadows
  if (use_shadows) {
    glDisable (GL_LIGHTING);
    if (use_textures) {
      ground_texture->bind (1);
      glEnable (GL_TEXTURE_2D);
      glDisable (GL_TEXTURE_GEN_S);
      glDisable (GL_TEXTURE_GEN_T);
      glColor3f (SHADOW_INTENSITY,SHADOW_INTENSITY,SHADOW_INTENSITY);
    }
    else {
      glDisable (GL_TEXTURE_2D);
      glColor3f (GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
          GROUND_B*SHADOW_INTENSITY);
    }
    glShadeModel (GL_FLAT);
    glDepthRange (0,0.9999);
    drawSphereShadow (pos[0],pos[1],pos[2],radius);
    glDepthRange (0,1);
  }
}

void ViewPort::dsDrawTriangle (const double pos[3], const double R[12],
    const double *v0, const double *v1,
    const double *v2, int solid)
{
  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos,R);
  drawTriangle (v0, v1, v2, solid);
  glPopMatrix();
}


void ViewPort::dsDrawCylinder (const double pos[3], const double R[12],
    double length, double radius)
{
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCylinder (length,radius,0);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCylinder (length,radius,0);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


void ViewPort::dsDrawCappedCylinder (const double pos[3], const double R[12],
    double length, double radius)
{
  setupDrawingMode();
  glShadeModel (GL_SMOOTH);
  setTransform (pos,R);
  drawCappedCylinder (length,radius);
  glPopMatrix();

  if (use_shadows) {
    setShadowDrawingMode();
    setShadowTransform();
    setTransform (pos,R);
    drawCappedCylinder (length,radius);
    glPopMatrix();
    glPopMatrix();
    glDepthRange (0,1);
  }
}


void ViewPort::dsDrawLine (const double pos1[3], const double pos2[3])
{
  setupDrawingMode();
  glColor3f (color[0],color[1],color[2]);
  glDisable (GL_LIGHTING);
  glLineWidth (2);
  glShadeModel (GL_FLAT);
  glBegin (GL_LINES);
  glVertex3f (pos1[0],pos1[1],pos1[2]);
  glVertex3f (pos2[0],pos2[1],pos2[2]);
  glVertex3f (pos1[0],pos1[1],pos1[2]);
  glVertex3f (pos2[0],pos2[1],pos2[2]);
  glEnd();
}


void ViewPort::dsDrawBoxD (const double pos[3], const double R[12],
    const double sides[3])
{
  int i;
  double pos2[3],R2[12],fsides[3];
  for (i=0; i<3; i++) pos2[i]=(double)pos[i];
  for (i=0; i<12; i++) R2[i]=(double)R[i];
  for (i=0; i<3; i++) fsides[i]=(double)sides[i];
  dsDrawBox (pos2,R2,fsides);
}


void ViewPort::dsDrawSphereD (const double pos[3], const double R[12], double radius)
{
  int i;
  double pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(double)pos[i];
  for (i=0; i<12; i++) R2[i]=(double)R[i];
  dsDrawSphere (pos2,R2,radius);
}


void ViewPort::dsDrawTriangleD (const double pos[3], const double R[12],
    const double *v0, const double *v1,
    const double *v2, int solid)
{
  int i;
  double pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(double)pos[i];
  for (i=0; i<12; i++) R2[i]=(double)R[i];

  setupDrawingMode();
  glShadeModel (GL_FLAT);
  setTransform (pos2,R2);
  drawTriangleD (v0, v1, v2, solid);
  glPopMatrix();
}


void ViewPort::dsDrawCylinderD (const double pos[3], const double R[12],
    double length, double radius)
{
  int i;
  double pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(double)pos[i];
  for (i=0; i<12; i++) R2[i]=(double)R[i];
  dsDrawCylinder (pos2,R2,length,radius);
}


void ViewPort::dsDrawCappedCylinderD (const double pos[3], const double R[12],
    double length, double radius)
{
  int i;
  double pos2[3],R2[12];
  for (i=0; i<3; i++) pos2[i]=(double)pos[i];
  for (i=0; i<12; i++) R2[i]=(double)R[i];
  dsDrawCappedCylinder (pos2,R2,length,radius);
}


void ViewPort::dsDrawLineD (const double _pos1[3], const double _pos2[3])
{
  int i;
  double pos1[3],pos2[3];
  for (i=0; i<3; i++) pos1[i]=(double)_pos1[i];
  for (i=0; i<3; i++) pos2[i]=(double)_pos2[i];
  dsDrawLine (pos1,pos2);
}


void ViewPort::dsSetSphereQuality (int n)
{
  sphere_quality = n;
}


void ViewPort::dsSetCappedCylinderQuality (int n)
{
  capped_cylinder_quality = n;
}

//THis function was obtained from a sample online.
// http://www.spacesimulator.net/tut4_3dsloader.html
// TODO rewrite this function
ViewPort::DSObject ViewPort::load3DSObject(const char*filename, const char* textureFile)
{

  DSObject p_object;
  int i; //Index variable

  //load the texture
  if (textureFile)
  {
    p_object.texture = new Texture(textureFile);
  } else {
    p_object.texture = NULL;
  }



  FILE *l_file; //File pointer

  unsigned short l_chunk_id; //Chunk identifier
  unsigned int l_chunk_lenght; //Chunk lenght

  unsigned char l_char; //Char variable
  unsigned short l_qty; //Number of elements in each chunk

  unsigned short l_face_flags; //Flag that stores some face information

  if ((l_file=fopen (filename, "rb"))== NULL)
  {
    LINFO("Can not open file %s", filename);
    return p_object; //Open the file
  }

  //Get the file length
  struct stat buf;
  fstat(fileno(l_file), &buf);
  long filelength = buf.st_size;

  while (ftell (l_file) < filelength) //Loop to scan the whole file
  {
    //getch(); //Insert this command for debug (to wait for keypress for each chuck reading)

    if(fread (&l_chunk_id, 2, 1, l_file) != 1) LFATAL("fread failed"); //Read the chunk header
    //printf("ChunkID: %x\n",l_chunk_id);
    if(fread (&l_chunk_lenght, 4, 1, l_file) != 1) LFATAL("fread failed"); //Read the lenght of the chunk
    //printf("ChunkLenght: %x\n",l_chunk_lenght);

    switch (l_chunk_id)
    {
      //----------------- MAIN3DS -----------------
      // Description: Main chunk, contains all the other chunks
      // Chunk ID: 4d4d
      // Chunk Lenght: 0 + sub chunks
      //-------------------------------------------
      case 0x4d4d:
        break;

        //----------------- EDIT3DS -----------------
        // Description: 3D Editor chunk, objects layout info
        // Chunk ID: 3d3d (hex)
        // Chunk Lenght: 0 + sub chunks
        //-------------------------------------------
      case 0x3d3d:
        break;

        //--------------- EDIT_OBJECT ---------------
        // Description: Object block, info for each object
        // Chunk ID: 4000 (hex)
        // Chunk Lenght: len(object name) + sub chunks
        //-------------------------------------------
      case 0x4000:
        i=0;
        do
        {
          if(fread (&l_char, 1, 1, l_file) != 1) LFATAL("fread faile");
          p_object.name[i]=l_char;
          i++;
        }while(l_char != '\0' && i<20);
        break;

        //--------------- OBJ_TRIMESH ---------------
        // Description: Triangular mesh, contains chunks for 3d mesh info
        // Chunk ID: 4100 (hex)
        // Chunk Lenght: 0 + sub chunks
        //-------------------------------------------
      case 0x4100:
        break;

        //--------------- TRI_VERTEXL ---------------
        // Description: Vertices list
        // Chunk ID: 4110 (hex)
        // Chunk Lenght: 1 x unsigned short (number of vertices)
        //             + 3 x float (vertex coordinates) x (number of vertices)
        //             + sub chunks
        //-------------------------------------------
      case 0x4110:
        if(fread (&l_qty, sizeof (unsigned short), 1, l_file) != 1) LFATAL("fread failed");
        p_object.vertices_qty = l_qty;
        //printf("Number of vertices: %d\n",l_qty);
        p_object.vertex.resize(l_qty);
        for (i=0; i<l_qty; i++)
        {
          if(fread (&p_object.vertex[i].x, sizeof(float), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Vertices list x: %f\n",p_object.vertex[i].x);
          if(fread (&p_object.vertex[i].y, sizeof(float), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Vertices list y: %f\n",p_object.vertex[i].y);
          if(fread (&p_object.vertex[i].z, sizeof(float), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Vertices list z: %f\n",p_object.vertex[i].z);
        }
        break;

        //--------------- TRI_FACEL1 ----------------
        // Description: Polygons (faces) list
        // Chunk ID: 4120 (hex)
        // Chunk Lenght: 1 x unsigned short (number of polygons)
        //             + 3 x unsigned short (polygon points) x (number of polygons)
        //             + sub chunks
        //-------------------------------------------
      case 0x4120:
        if(fread (&l_qty, sizeof (unsigned short), 1, l_file) != 1) LFATAL("fread failed");
        p_object.polygons_qty = l_qty;
        p_object.polygon.resize(l_qty);
        printf("Number of polygons: %d\n",l_qty);
        for (i=0; i<l_qty; i++)
        {
          if(fread (&p_object.polygon[i].a, sizeof (unsigned short), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Polygon point a: %d\n",p_object.polygon[i].a);
          if(fread (&p_object.polygon[i].b, sizeof (unsigned short), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Polygon point b: %d\n",p_object.polygon[i].b);
          if(fread (&p_object.polygon[i].c, sizeof (unsigned short), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Polygon point c: %d\n",p_object.polygon[i].c);
          if(fread (&l_face_flags, sizeof (unsigned short), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Face flags: %x\n",l_face_flags);
        }
        break;

        //------------- TRI_MAPPINGCOORS ------------
        // Description: Vertices list
        // Chunk ID: 4140 (hex)
        // Chunk Lenght: 1 x unsigned short (number of mapping points)
        //             + 2 x float (mapping coordinates) x (number of mapping points)
        //             + sub chunks
        //-------------------------------------------
      case 0x4140:
        if(fread (&l_qty, sizeof (unsigned short), 1, l_file) != 1) LFATAL("fread failed");
        p_object.mapcoord.resize(l_qty);
        for (i=0; i<l_qty; i++)
        {
          if(fread (&p_object.mapcoord[i].u, sizeof (float), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Mapping list u: %f\n",p_object.mapcoord[i].u);
          if(fread (&p_object.mapcoord[i].v, sizeof (float), 1, l_file) != 1) LFATAL("fread failed");
          //printf("Mapping list v: %f\n",p_object.mapcoord[i].v);
        }
        break;

        //----------- Skip unknow chunks ------------
        //We need to skip all the chunks that currently we don't use
        //We use the chunk lenght information to set the file pointer
        //to the same level next chunk
        //-------------------------------------------
      default:
        fseek(l_file, l_chunk_lenght-6, SEEK_CUR);
    }
  }
  fclose (l_file); // Closes the file stream

  return p_object;
}


int ViewPort::loadBitmap(const char*filename)
{
  FILE * file;
  char temp;
  long i;
  int num_texture = 10;

  BITMAPINFOHEADER infoheader;

  if( (file = fopen(filename, "rb"))==NULL) return (-1); // Open the file for reading

  fseek(file, 18, SEEK_CUR);  /* start reading width & height */
  if(fread(&infoheader.biWidth, sizeof(int), 1, file) != 1) LFATAL("fread failed");

  if(fread(&infoheader.biHeight, sizeof(int), 1, file) != 1) LFATAL("fread failed");

  if(fread(&infoheader.biPlanes, sizeof(short int), 1, file) != 1) LFATAL("fread failed");
  if (infoheader.biPlanes != 1) {
    //printf("Planes from %s is not 1: %u\n", filename, infoheader.biPlanes);
    return 0;
  }

  // read the bpp
  if(fread(&infoheader.biBitCount, sizeof(unsigned short int), 1, file) != 1) LFATAL("fread failed");
  if (infoheader.biBitCount != 24) {
    //printf("Bpp from %s is not 24: %d\n", filename, infoheader.biBitCount);
    return 0;
  }

  fseek(file, 24, SEEK_CUR);

  // read the data.
  infoheader.data = (char *) malloc(infoheader.biWidth * infoheader.biHeight * 3);
  if (infoheader.data == NULL) {
    //printf("Error allocating memory for color-corrected image data\n");
    return 0;
  }

  if ((i = fread(infoheader.data, infoheader.biWidth * infoheader.biHeight * 3, 1, file)) != 1) {
    //printf("Error reading image data from %s.\n", filename);
    return 0;
  }

  for (i=0; i<(infoheader.biWidth * infoheader.biHeight * 3); i+=3) { // reverse all of the colors. (bgr -> rgb)
    temp = infoheader.data[i];
    infoheader.data[i] = infoheader.data[i+2];
    infoheader.data[i+2] = temp;
  }


  fclose(file); // Closes the file stream


  glBindTexture(GL_TEXTURE_2D, num_texture); // Bind the ID texture specified by the 2nd parameter

  // The next commands sets the texture parameters
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); // If the u,v coordinates overflow the range 0,1 the image is repeated
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // The magnification function ("linear" produces better results)
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST); //The minifying function

  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // We don't combine the color with the original surface color, use only the texture map.

  // Finally we define the 2d texture
  glTexImage2D(GL_TEXTURE_2D, 0, 3, infoheader.biWidth, infoheader.biHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, infoheader.data);

  // And create 2d mipmaps for the minifying function
#ifdef INVT_HAVE_LIBGLUT
  gluBuild2DMipmaps(GL_TEXTURE_2D, 3, infoheader.biWidth, infoheader.biHeight, GL_RGB, GL_UNSIGNED_BYTE, infoheader.data);
#endif

  free(infoheader.data); // Free the memory we used to load the texture

  return (num_texture); // Returns the current texture OpenGL ID
}


void ViewPort::dsDraw3DSObject(const double pos[3], const double R[12], DSObject& object)
{
  if (object.texture != NULL)
    dsSetTexture (ViewPort::OTHER, object.texture);

  setupDrawingMode();
  glEnable (GL_NORMALIZE);
  glShadeModel (GL_SMOOTH);

  setTransform (pos,R);
  glScaled (object.scale,object.scale,object.scale);
  draw3dsObject(object);
  glPopMatrix();
  glDisable (GL_NORMALIZE);

}

void ViewPort::draw3dsObject(DSObject& object)
{
  glBegin(GL_TRIANGLES); // glBegin and glEnd delimit the vertices that define a primitive (in our case triangles)
  //glBegin (GL_TRIANGLE_FAN);
  //glBegin (GL_TRIANGLE_STRIP);
  for (int l_index=0;l_index<object.polygons_qty;l_index++)
  {
    //----------------- FIRST VERTEX -----------------
    // Texture coordinates of the first vertex
    glTexCoord2f( object.mapcoord[ object.polygon[l_index].a ].u,
        object.mapcoord[ object.polygon[l_index].a ].v);
    // Coordinates of the first vertex
    glVertex3f( object.vertex[ object.polygon[l_index].a ].x,
        object.vertex[ object.polygon[l_index].a ].y,
        object.vertex[ object.polygon[l_index].a ].z); //Vertex definition

    //----------------- SECOND VERTEX -----------------
    // Texture coordinates of the second vertex
    glTexCoord2f( object.mapcoord[ object.polygon[l_index].b ].u,
        object.mapcoord[ object.polygon[l_index].b ].v);
    // Coordinates of the second vertex
    glVertex3f( object.vertex[ object.polygon[l_index].b ].x,
        object.vertex[ object.polygon[l_index].b ].y,
        object.vertex[ object.polygon[l_index].b ].z);

    //----------------- THIRD VERTEX -----------------
    // Texture coordinates of the third vertex
    glTexCoord2f( object.mapcoord[ object.polygon[l_index].c ].u,
        object.mapcoord[ object.polygon[l_index].c ].v);
    // Coordinates of the Third vertex
    glVertex3f( object.vertex[ object.polygon[l_index].c ].x,
        object.vertex[ object.polygon[l_index].c ].y,
        object.vertex[ object.polygon[l_index].c ].z);
  }
  glEnd();
  glFlush();

}
