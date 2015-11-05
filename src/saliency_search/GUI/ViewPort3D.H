/*!@file GUI/ViewPort3D.H  3D rendering view port */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/ViewPort3D.H $
// $Id: ViewPort3D.H 14542 2011-02-23 21:07:42Z dicky $
//

#ifndef ViewPort3D_H_DEFINED
#define ViewPort3D_H_DEFINED

#include "Util/Types.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/ImageSet.H"
#include "Image/Point2D.H"
#include "Image/Point3D.H"

#ifdef HAVE_GL_GLEW_H
#include <GL/glew.h>
#endif

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>


#ifdef INVT_HAVE_LIBGLUT
#include <GL/glut.h>
#endif

#include <vector>

typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

class ViewPort3D
{
public:


  struct Line
  {
    Point2D<float> p1;
    Point2D<float> p2;
  };


  ViewPort3D(int width, int height, 
      bool wireframe = false, bool useLights = true, bool useFeedback = false);

  ~ViewPort3D();

  Dims getDims() { return Dims(itsScreenWidth,  itsScreenHeight); }

  //! rendering options
  void setWireframeMode(bool val) { itsWireframe = val; }
  void setLightsMode(bool val) { itsUseLights = val; }
  void setFeedbackMode(bool val) { itsUseFeedback = val; }


  //! Initialize a rendering frame
  void initFrame();

  void initProjection();


  //! Initialize the opengl context 
  void initCtx();

  //! Render and get a the frame from the buffer
  Image<PixRGB<byte> > getFrame();
  Image<PixRGB<float> > getFrameFloat();

  //! Render the frame and return lines instread of a frame
  std::vector<ViewPort3D::Line> getFrameLines();
  

  //! Get the depth Frame
  Image<float> getDepthFrame();

  //! Get the position of an point in image space to world space
  Point3D<float> getPosition(Point3D<float> loc);

  //! Build a projection matrix based on real camera parameters
  void setProjectionMatrix(const double cameraParam[3][4]);

  //! Build a projection matrix based on precpective parameters
  //fovy -- field of view in the y direction
  //aspect -- ratio that determines the field of view in the x direction
  //zmin -- Distance from the viewer to the near clipping plane
  //zmax -- Distance from the viewer to the fax clipping plane
  void setProjectionMatrix(float fovy, float aspect, float zmin, float zmax);
      
  //! Decompose a matrix
  int  paramDecompMat(const double source[3][4],
      double cpara[3][4],
      double trans[3][4] );

  //! Setup the camera viewport
  void setCamera ( const Point3D<float> pos, const Point3D<float> rot );

  void setCamera( const double trans[3][4]);


  //! Setup the lights
  void setLights();

  //// Draw varius primities objects ////////

  void setColor(const PixRGB<byte> color);

  void drawGrid(Dims size, Dims step, float height=0);
  
  
  //Surfaces
  void drawLine(const Point3D<float>& p1, const Point3D<float>& p2);

  void drawRectangle(const Point3D<float> pos, //position
      const Point3D<float> rot, //Rotation
      const float width, const float height,
      const PixRGB<byte> color);

  void drawCircle(const Point3D<float> pos,
      const Point3D<float> rot, //Rotation
      const float radius,
      const PixRGB<byte> color);

  void drawDisk(const Point3D<float> pos,
      const Point3D<float> rot, //Rotation
      const float radius,
      const PixRGB<byte> color);

  void drawEllipse(const Point3D<float> pos,
      const Point3D<float> rot, //Rotation
      const float radiusX, const float radiusY,
      const PixRGB<byte> color);
  


  //Solids
  
  void drawBox(const Point3D<float> pos, //position
      const Point3D<float> rot, //Rotation
      const Point3D<float> size, //size
      const PixRGB<byte> color);

  //! Draw an extruded contour
  void drawExtrudedContour(const std::vector<Point2D<float> >& contour,
      const Point3D<float> pos,
      const Point3D<float> rot,
      const float thickness,
      const PixRGB<byte> color);


  //! Draw a sphere
  void drawSphere(const Point3D<float> pos,
      const Point3D<float> rot, //Rotation
      const Point3D<float> size,
      const PixRGB<byte> color);

  //! Draw a cylinder
  void drawCylinder(const Point3D<float> pos,
      const Point3D<float> rot, //Rotation
      const float radius,
      const float length,
      const PixRGB<byte> color);

  //! Draw a cone 
  void drawCone(const Point3D<float> pos,
      const Point3D<float> rot, //Rotation
      const float radius,
      const float length,
      const PixRGB<byte> color);

  //! Draw a Capped cylinder
  void drawCappedCylinder(const Point3D<float> pos,
      const Point3D<float> rot, //Rotation
      const float radius,
      const float length,
      const PixRGB<byte> color);

  //! Draw the ground
  void drawGround(const Point2D<float> size, const PixRGB<byte> color);


  //! Generate a unit sphere and save it in a list for faster processing
  uint generateSphere(int quality);


  //! Triangulate a polygon
  //Borowed from John W. Ratcliff [jratcliff@verant.com] and adapted to work with Point2D class
  //http://www.flipcode.com/archives/Efficient_Polygon_Triangulation.shtml
  std::vector<Point2D<float> > triangulate(const std::vector<Point2D<float> >& contour);
  bool snip(const std::vector<Point2D<float> > &contour,
      int u,int v,int w,int n,int *V);



  //! This is recursively subdivides a triangular area (vertices p1,p2,p3) into
  //! smaller triangles, and then draws the triangles. All triangle vertices are
  //! normalized to a distance of 1.0 from the origin (p1,p2,p3 are assumed
  //! to be already normalized). Note this is not super-fast because it draws
  //! triangles rather than triangle strips.

  void drawPatch (double p1[3], double p2[3], double p3[3], int level);

  
  //!Add a texture from Image and return its id in texture memory
  //Note, need to initframe first before adding
  uint addTexture(const Image<PixRGB<byte> >& textureImg);

  uint addTexture(const Image<float>& textureImg);

  //!Bind a texture id to current set
  void bindTexture(const uint texId);
 
  //!Load texture to an exsisting id
  void loadTexture(const Image<PixRGB<byte> >& textureImg, uint texId);

  
 
#ifdef HAVE_GL_GLEW_H
  //! Shader support
  void printInfoLog(GLhandleARB obj);
  //! Shader support
  GLhandleARB createShader(const char *prog, int type);

  //! Render a program into a texture (-1 use current texture)
  void progToTexture(const GLhandleARB prog, const int texId = -1);
  
#endif
  


private:
  int itsScreenWidth; //width of viewport
  int itsScreenHeight;  //height of viewport
  Point3D<float> itsCameraPosition;
  Point3D<float> itsCameraRotation;

  bool itsShowRenderWindow; //Wether to show the render window (for debuging);
  bool itsWireframe; //Show wireframe only
  bool itsUseLights;
  bool itsUseFeedback;
  int itsSphereQuality;
  int itsCylinderQuality;


  //Objects lists
  uint itsSphereListNum;

  bool itsInitGlew; //Used to keep track if glew is init

  Display* itsDisplay; //Out main display
  Window itsWin; //the window handle
  GLXContext itsCtx; //the display context


  double itsProjMatrix[16]; //The camera projection matrix
  double itsCameraExParam[16]; //The camera extrinsic parameter matrix
  bool itsUseExParam;

  //For feedback buffer;
  int itsFeedbackBufferSize;
  float* itsFeedbackBuffer;




};


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif //
