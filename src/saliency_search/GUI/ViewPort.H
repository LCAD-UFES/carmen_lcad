/*!@file GUI/Viewport.H test opengl viewport */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/GUI/ViewPort.H $
// $Id: ViewPort.H 12962 2010-03-06 02:13:53Z irock $


#ifndef ViewPort_H
#define ViewPort_H

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <pthread.h>
#include <GL/glx.h>
#include "GUI/Texture.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/log.H"

#include <vector>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// constants to convert degrees to radians and the reverse
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)

// light vector. LIGHTZ is implicitly 1
#define LIGHTX (0.0f)
#define LIGHTY (0.0f)

// ground and sky
#define SHADOW_INTENSITY (0.65f)
#define GROUND_R (1.0f)         // ground color for when there's no texture
#define GROUND_G (1.0f)
#define GROUND_B (1.0f)

//***************************************************************************
// misc mathematics stuff

/*
#ifndef dCROSS
#define dCROSS(a,op,b,c) \
(a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
(a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
(a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]);
#endif
*/


class ViewPort {


  public:
    enum TEXTURES {NONE,OTHER, SKY, GROUND, WOOD, TREE}; //NONE always need to be 0

    // Our vertex type
    struct vertex_type {

      vertex_type(float _x = 0, float _y = 0, float _z = 0):
        x(_x), y(_y), z(_z)
      {}

      float x,y,z;
    };

    // The polygon (triangle), 3 numbers that aim 3 vertices
    struct polygon_type {
      polygon_type(int _a = 0,int _b = 0, int _c=0) :
        a(_a), b(_b), c(_c)
      {}
      int a,b,c;
    };

    // The mapcoord type, 2 texture coordinates for each vertex
    struct mapcoord_type {

      mapcoord_type(float _u=0, float _v=0) :
        u(_u), v(_v)
      {}
      float u,v;
    };

    typedef struct {
      char name[20];
      int vertices_qty;
      int polygons_qty;
      std::vector<vertex_type> vertex;
      std::vector<polygon_type> polygon;
      std::vector<mapcoord_type> mapcoord;
      Texture* texture;
      float scale;
    } DSObject;

    ViewPort(const char *winname,
        const char* ground = "ground.ppm",
        const char*sky = "sky.ppm",
        bool useFog = false,
        bool drawWorld=true,
        int w=320, int h=240,
        const double cameraParam[3][4] = NULL);
    ~ViewPort();
    static void mainWindowThread(void *t);
    XEvent initFrame(const double* cameraParam=NULL);
    int updateFrame();
    void getFrame(unsigned char *img);
    Image<PixRGB<byte> > getFrame();

    void setTextures(bool val);

    void setShadows(bool val);

    Dims getDims(){return Dims(itsWidth,itsHeight);}

    void setWireframe(bool val) { itsWireframe = val; }
    void setDrawWorld(bool val) { itsDrawWorld = val; }
    void setZoom(float val) { itsZoomFactor = val; }

  public:
    int itsWidth;
    int itsHeight;
    int run;                                //is the window running
    int writeframes;         //do we need to write the frames to the disk?

    pthread_t win_thread;

    Display *display;
    XVisualInfo *visual;
    Colormap colormap;
    Window win;
    GLXContext glx_context;
    int last_key_pressed;        //the last key pressed in the window

    Atom wm_protocols_atom;
    Atom wm_delete_window_atom;

    int screen;
    void createMainWindow(const char *winname);
    void destroyMainWindow();
    void handleEvent (XEvent &event);

    void startGraphics (const char *prefix, const char *ground, const char *sky);
    void stopGraphics();

    //3d grapics stuff
    Texture *sky_texture;
    Texture *ground_texture;
    Texture *wood_texture;
    Texture *tree_texture;
    Texture *other_texture;

    GLuint listnum;   //store a list for processing spheres
    bool itsDrawWorld;

    //Build the projection matrix from a camera parameter
    void buildProjectionMatrix(const double cameraParam[3][4], double projMatrix[16]);
    int  paramDecompMat(const double source[3][4], double cpara[3][4], double trans[3][4] );


    // current camera position and orientation
    void initCamera();
    double view_xyz[3];        // position x,y,z
    double view_hpr[3];        // heading, pitch, roll (degrees)
    double itsProjMatrix[16]; //The camera projection matrix

    //***************************************************************************
    // the current drawing state (for when the user's step function is drawing)
    double color[4];        // current r,g,b,alpha color
    int tnum;                        // current texture number

    double ground_scale;        // ground texture scale (1/size)
    double ground_ofsx;                // offset of ground texture
    double ground_ofsy;
    double sky_scale;        // sky texture scale (1/size)
    double sky_height;                // sky height above viewpoint



    int sphere_quality;
    int capped_cylinder_quality;

    // textures and shadows
    int use_textures;                // 1 if textures to be drawn
    int use_shadows;                // 1 if shadows to be drawn

    /* set and get the camera position. xyz is the cameria position (x,y,z).
     * hpr contains heading, pitch and roll numbers in degrees. heading=0
     * points along the x axis, pitch=0 is looking towards the horizon, and
     * roll 0 is "unrotated".
     */
    void dsSetViewpoint (double xyz[3], double hpr[3]);
    void dsGetViewpoint (double xyz[3], double hpr[3]);

    //! get the 3D location of an x,y pos on the screen
    void unProjectPoint(const int x, const int y, double objLoc[3]);


    /* change the way objects are drawn. these changes will apply to all further
     * dsDrawXXX() functions. the texture number must be a DS_xxx texture
     * constant. the red, green, and blue number are between 0 and 1.
     * alpha is between 0 and 1 - if alpha is not specified it's assubed to be 1.
     * the current texture is colored according to the current color.
     * at the start of each frame, the texture is reset to none and the color is
     * reset to white.
     */
    void dsSetTexture (TEXTURES texture_number, Texture* texturePtr = NULL);
    void dsSetColor (double red, double green, double blue);
    void dsSetColorAlpha (double red, double green, double blue, double alpha);


    /* draw objects.
     *   - pos[] is the x,y,z of the center of the object.
     *   - R[] is a 3x3 rotation matrix for the object, stored by row like this:
     *        [ R11 R12 R13 0 ]
     *        [ R21 R22 R23 0 ]
     *        [ R31 R32 R33 0 ]
     *   - sides[] is an array of x,y,z side lengths.
     *   - all cylinders are aligned along the z axis.
     */
    void dsDrawBox (const double pos[3], const double R[12], const double sides[3]);
    void dsDrawSphere (const double pos[3], const double R[12], double radius);
    void dsDrawTriangle (const double pos[3], const double R[12],
        const double *v0, const double *v1, const double *v2, int solid);
    void dsDrawCylinder (const double pos[3], const double R[12],
        double length, double radius);
    void dsDrawCappedCylinder (const double pos[3], const double R[12],
        double length, double radius);
    void dsDrawLine (const double pos1[3], const double pos2[3]);

    /* these drawing functions are identical to the ones above, except they take
     * double arrays for `pos' and `R'.
     */
    void dsDrawBoxD (const double pos[3], const double R[12],
        const double sides[3]);
    void dsDrawSphereD (const double pos[3], const double R[12],
        const double radius);
    void dsDrawTriangleD (const double pos[3], const double R[12],
        const double *v0, const double *v1, const double *v2, int solid);
    void dsDrawCylinderD (const double pos[3], const double R[12],
        double length, double radius);
    void dsDrawCappedCylinderD (const double pos[3], const double R[12],
        double length, double radius);
    void dsDrawLineD (const double pos1[3], const double pos2[3]);

    /* Set the drawn quality of the objects. Higher numbers are higher quality,
     * but slower to draw. This must be set before the first objects are drawn to
     * be effective.
     */
    void dsSetSphereQuality (int n);                /* default = 1 */
    void dsSetCappedCylinderQuality (int n);        /* default = 3 */

    //util
    void normalizeVector3(float*);
    double dDOT(const double*, const double*);
    void setCamera(double, double, double, double, double, double);
    void setColor(double, double, double, double);
    void setTransform(const double*, const double*);
    void setShadowTransform();
    void drawBox(const double*);
    void drawPatch(double*, double*, double*,int);

    void drawSphere();
    void drawSphereShadow(double, double, double, double);
    void drawTriangle(const double*, const double*, const double*, int);
    void drawTriangleD(const double*, const double*, const double*, int);
    void drawCappedCylinder(double, double);
    void drawCylinder(double, double, double);
    void initMotionModel();
    void wrapCameraAngles();
    void dsMotion(int, int, int);
    void drawSky(double*);
    void drawGround();
    void drawPyramidGrid();

    void dsDrawFrame(const double cameraParam[16]=NULL);
    int dsGetShadows();
    void dsSetShadows(int);
    int dsGetTextures();
    void dsSetTextures(int);
    void setupDrawingMode();
    void setShadowDrawingMode();


    //3ds object support
    typedef struct                       /**** BMP file info structure ****/
    {
      unsigned int   biSize;           /* Size of info header */
      int            biWidth;          /* Width of image */
      int            biHeight;         /* Height of image */
      unsigned short biPlanes;         /* Number of color planes */
      unsigned short biBitCount;       /* Number of bits per pixel */
      unsigned int   biCompression;    /* Type of compression to use */
      unsigned int   biSizeImage;      /* Size of image data */
      int            biXPelsPerMeter;  /* X pixels per meter */
      int            biYPelsPerMeter;  /* Y pixels per meter */
      unsigned int   biClrUsed;        /* Number of colors used */
      unsigned int   biClrImportant;   /* Number of important colors */
      char *data;
    } BITMAPINFOHEADER;


    DSObject load3DSObject(const char* filename, const char* textureFile = NULL);
    int loadBitmap(const char*filename);
    void dsDraw3DSObject(const double pos[3], const double R[12], DSObject &object);
    void draw3dsObject(DSObject& object);


  private:
    bool itsUseFog;
    bool itsWireframe;
    float itsZoomFactor; //1 = +/- 45 degrees


};

#endif
