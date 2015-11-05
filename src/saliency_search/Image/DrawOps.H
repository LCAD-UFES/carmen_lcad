/*!@file Image/DrawOps.H functions for drawing on images
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
// pending; filed July 23, 2001, following provisional applications     //
// No. 60/274,674 filed March 8, 2001 and 60/288,724 filed May 4, 2001).//
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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/DrawOps.H $
// $Id: DrawOps.H 15383 2012-08-09 01:18:01Z kai $
//

#ifndef IMAGE_DRAWOPS_H_DEFINED
#define IMAGE_DRAWOPS_H_DEFINED

#include "Util/Types.H"
#include "Image/SimpleFont.H"
#include "Image/Pixels.H"

#include <string>
#include <vector>

template <class T> class Point2D;
class Dims;
class Rectangle;
template <class T> class Image;
template <class T> class PixRGB;
class ColorMap;

//! draw the trajectory for the points in foas onto img
/*! Yellow circles and red arrows are drawn, connecting the points in foas.
  @param foas a vector with the foci of attention ised for drawing*/
void drawTraj(Image< PixRGB<byte> >& img,
              const Point2D<int>* foas_begin, const Point2D<int>* foas_end);

//! build a combo between a color and greyscale image
/*! @param colimg the color image (e.g., trajectory)
   @param bwimg the greyscale image (e.g., saliency map)
   @param xcombo will combine horizontally if true, otherwise vertically
   @param interp use bilinear interpolation for any image rescaling */
Image< PixRGB<byte> > colGreyCombo(const Image< PixRGB<byte> >& colimg,
                                   const Image<float>& bwimg,
                                   const bool xcombo,
                                   const bool interp = true);

//! build a combo between a color and color image
/*! @param colimg1 the color image (e.g., trajectory + saliency map)
   @param colimg2 the second color image (e.g., the scene highlighted by TRM)
   @param xcombo will combine horizontally if true, otherwise vertically
   @param interp use bilinear interpolation for any image rescaling */
Image< PixRGB<byte> > colColCombo(const Image< PixRGB<byte> >& colimg1,
                                   const Image< PixRGB<byte> >& colimg2,
                                  const bool xcombo,
                                  const bool interp = true);

//! highlight an image in the regions given by a mask
Image< PixRGB<byte> > highlightRegions(const Image< PixRGB<byte> >& img,
                                       const Image<byte>& mask,
                                       const int maxval = 320);

//! warp image onto height map
Image< PixRGB<byte> > warp3Dmap(const Image< PixRGB<byte> >& img,
                                const Image<float>& hmap,
                                const float pitch, const float yaw,
                                Dims& imdims);

//! Set values at all locations where mask is non-zero
void inplaceSetValMask(Image<float>& dest,
                       const Image<byte>& mask, const float val);


//! Draws a disk with color "value" in image
template <class T_or_RGB>
void drawDisk(Image<T_or_RGB>& dst,
              const Point2D<int>& center, const int radius,
              const T_or_RGB value,
              const float alpha = 1.0);

//! Draws a half disk with color "value" in image
template <class T_or_RGB>
void drawHalfDisk(Image<T_or_RGB>& dst,
              const Point2D<int>& center, const int radius,
              const T_or_RGB value, float ori = 0);

//! Draw a line from p1 to p2 of color col and pixel radius rad
template <class T_or_RGB>
void drawLine(Image<T_or_RGB>& dst,
              const Point2D<int>& p1, const Point2D<int>& p2, const T_or_RGB col,
              const int rad = 1,							
              const float alpha = 1.0);
         
//! Draw a line at pos angle ori(rad) and length len
template <class T_or_RGB>
void drawLine(Image<T_or_RGB>& dst,
              const Point2D<int>& pos, float ori, float len, const T_or_RGB col,
              const int rad = 1);

//! Draw a corner at pos and ori with an ang
template <class T_or_RGB>
void drawCorner(Image<T_or_RGB>& dst,
              const Point2D<int>& pos, float ori, float ang, float len,
              const T_or_RGB col, const int rad = 1);


//! Draw a rectangle of color col and line width of rad*2, with rounded corners
template <class T_or_RGB>
void drawRect(Image<T_or_RGB>& dst,
              const Rectangle& rect, const T_or_RGB col, const int rad = 1);

//! Draw a rectangle of the given color and line width, with square corners
template <class T_or_RGB>
void drawRectSquareCorners(Image<T_or_RGB>& dst, const Rectangle &r,
                           const T_or_RGB color, const int linewidth);

//! Draw a rectangle of color col and pixel radius rad, rotate by ori
template <class T_or_RGB>
void drawRectOR(Image<T_or_RGB>& dst,
                const Rectangle& rect, const T_or_RGB col, const int rad = 1,
                const float ori = 0.0F);

//! Draw a rectangle of color col and pixel radius rad, no size assertions
template <class T_or_RGB>
void drawRectEZ(Image<T_or_RGB>& dst,
                const Rectangle& rect, const T_or_RGB col, const int rad = 1);

//! Draw a filled rectangle with the given fill value
template <class T_or_RGB>
void drawFilledRect(Image<T_or_RGB>& dst, const Rectangle& rect,
                    const T_or_RGB fillVal);

//! Draw a cross of color col, half-size siz and pixel radius rad
template <class T_or_RGB>
void drawCross(Image<T_or_RGB>& dst,
               const Point2D<int>& p, const T_or_RGB col, const int siz = 3,
               const int rad = 1);

//! Draw a cross of color col, half-size siz and pixel radius rad
template <class T_or_RGB>
void drawCrossOR(Image<T_or_RGB>& dst,
               const Point2D<int>& p, const T_or_RGB col, const int siz = 3,
               const int rad = 1, const float ori = 0.0F);

//! Draw a square patch of color col and size siz
template <class T_or_RGB>
void drawPatch(Image<T_or_RGB>& dst,
               const Point2D<int>& p, const int siz, const T_or_RGB col);

//! Draw a square patch of color col and size siz, with a black border
template <class T_or_RGB>
void drawPatchBB(Image<T_or_RGB>& dst,
                 const Point2D<int>& p, const int siz, const T_or_RGB col, const T_or_RGB bgcol = T_or_RGB());

//! Draw a circle of color col and radius radius, with pixel radius rad
template <class T_or_RGB>
void drawCircle(Image<T_or_RGB>& dst,
                const Point2D<int>& p, const int radius, const T_or_RGB col,
                const int rad = 1);

//! Draw a arc of color col and from point, with pixel radius rad
template <class T_or_RGB>
void drawArc(Image<T_or_RGB>& dst,
                const Point2D<int>& center, const int radius,
								const T_or_RGB col, 
								const float startAng = 0.0F,const float endAng = 2*M_PI,
								const int rad = 1);

//! Draw a ellipse of color col and radius radiusx, radiusy, with pixel radius rad
template <class T_or_RGB>
void drawEllipse(Image<T_or_RGB>& dst,
                const Point2D<int>& p, const int radiusx,
                const int radiusy, const T_or_RGB col, const int rad = 1);

//! Draw an arrow from p1 to p2 with pixel radius rad
template <class T_or_RGB>
void drawArrow(Image<T_or_RGB>& dst,
               const Point2D<int>& p1, const Point2D<int>& p2, const T_or_RGB col,
               const int rad = 1, const int len = -1);

//! Draw a grid
template <class T_or_RGB>
void drawGrid(Image<T_or_RGB>& dst,
              const int spacingX, const int spacingY,
              const int thickX, const int thickY, const T_or_RGB col);

//! Draw an nx * ny grid that exactly partitions the given image
template <class T_or_RGB>
void drawGrid(Image<T_or_RGB>& dst, const uint nx, const uint ny,
              const int thick, const T_or_RGB col);

//! Draw a superquadric shape. nSeg defines the quality of the shape. ie. how many line to draw
template <class T_or_RGB>
void drawSuperquadric(Image<T_or_RGB>& dst,
                const Point2D<int>& p,
                const float a, const float b, const float e, const T_or_RGB col,
                const float rot = 0, const float k1 = 0, const float k2 = 0,
                const float thetai = -M_PI,
                const float thetaf = M_PI,
                const int rad = 1,
                const int nSeg = 40);


//! Draw a 2D contour
/*! Will draw a disk of color col and radius rad in dst at every point where src is on a contour. dst should be
    initialized ahd have same dims as src. Nothing is drawn for point off contours. */
template <class T, class T_or_RGB>
void drawContour2D(const Image<T>& src, Image<T_or_RGB>& dst, const T_or_RGB &col, const byte rad);

enum TextAnchor
  {
    ANCHOR_TOP_LEFT,
    ANCHOR_TOP_RIGHT,
    ANCHOR_BOTTOM_LEFT,
    ANCHOR_BOTTOM_RIGHT,
    ANCHOR_CENTER
  };

//! Write some text in normal font; only ASCII chars 32 .. 126 are supported
/*! See Image/SimpleFont.H for font choices. */
template <class T_or_RGB>
void writeText(Image<T_or_RGB>& dst,
               const Point2D<int>& pt, const char* text,
               const T_or_RGB col = T_or_RGB(),
               const T_or_RGB bgcol = T_or_RGB(255),
               const SimpleFont& f = SimpleFont::FIXED(10),
               const bool transparent_bg = false,
               const TextAnchor anchor = ANCHOR_TOP_LEFT);

/// Make an image containing one or more lines of text
/** The function will choose a font width that fits the desired image
    size.

    @param w the desired width (in pixels) of the resulting image
    @param lines an array of strings to be written in the result, one
    line per string
    @param nlines the number of strings in the string array
    @param col the foreground color for the written text
    @param bg the background color for the written text
    @param max_chars_per_line if non-zero, then each line of text will
    be truncated at this many characters
    @param fontwidth the max font width unless it is zero, in which case
    it will be determined automatically to fit the text in the box.
 */
template <class T_or_RGB>
Image<T_or_RGB> makeMultilineTextBox(const int w,
                                     const std::string* lines,
                                     const size_t nlines,
                                     const T_or_RGB col,
                                     const T_or_RGB bg = T_or_RGB(),
                                     const size_t max_chars_per_line = 0,
                                     const int fontwidth = 0);

//! draw a dot which is a point with 4 connected points
/*! this is between the setVal method and drawCircle method for
  illustrating points
*/
template <class T_or_RGB>
void drawPoint(Image<T_or_RGB>& dst,
               int X,int Y, T_or_RGB pix);


//! draw a 6 point star 
template <class T_or_RGB>
void drawStar(Image<T_or_RGB>& dst,
               const Point2D<int>& p, const T_or_RGB col, const int siz = 5,
               const int rad = 1);
//! draw a 5 point star 
template <class T_or_RGB>
void drawStar2(Image<T_or_RGB>& dst,
               const Point2D<int>& p, const T_or_RGB col, const int siz = 5,
               const int rad = 1);

//! Draws a disk in image and check if it hits a target
/*! @param mask input binary target mask to draw into
  @param center coordinates of disk center
  @param radius radisu of disk, in pixels
  @param value pixel value used to draw the disk
  @param targetval value with which the targets are represented
  @param floodval value to flood target with if target hit
  @return number of targets hit and flooded */
template <class T>
int drawDiskCheckTarget(Image<T>& dst,
                        Image<T>& mask, const Point2D<int>& center,
                        const int radius, const T value,
                        const T targetval, const T floodval);

//! Warp image in 3D onto z-map
/*! @param ima Input image
  @param zmap Height map of same size as ima
  @param pitch 3D pitch angle, in degrees
  @param yaw 3D yaw angle, in degrees
  @param dims The result will have these dimensions, unless the dimensions are
  empty (i.e. 0-by-0), in which case the result will be the best size computed
  from ima's size and the pitch and yaw angles.
*/
template <class T>
Image<PixRGB<T> > warp3D(const Image<PixRGB<T> >& ima,
                         const Image<float>& zmap,
                         const float pitch, const float yaw,
                         const float zMax, Dims& dims);

//! Format a feature map for display
/*! Take an Image<float> feature map as input and convert it to
  PixRGB<byte> by either multiplying the values by a factor or by
  normalizing them to 0 .. 255 (if factor is 0.0), and then using a
  given color map (which will also be displayed if showColorScale is
  true). Then rescale to the specified dims, using interpolation if
  desired. Then write a descriptor label in the top-left corner using
  a large font, and finally show the original range of values in the
  bottom-right corner uing a tiny font. */
Image< PixRGB<byte> >
formatMapForDisplay(const Image<float>& img, const float factor,
                    const Dims& newdims, const bool useInterp,
                    const ColorMap& cmap, const bool showColorScale,
                    const char *label);

//! Generate a plot from a vector of points. Choose numticks < 0 to
//! disable x and y axes tick marks. class U should be a deque or vector
template <typename T, class U>
Image< PixRGB<byte> >
linePlot(const U& points, const int w, const int h, 
         const T& minVal = T(), const T& maxVal = T(), 
         const char *title = "",
         const char *yLabel = "", const char *xLabel = "",
         const PixRGB<byte>& linecol=PixRGB<byte>(0,0,0),
         const PixRGB<byte>& bckcol=PixRGB<byte>(255,255,255),
         const int numticks = 4, const bool axisonright = false);

//! Generate multiline plot from a vector of lines

template <typename T>
Image< PixRGB<byte> >
multilinePlot(const std::vector<std::vector<T> >& lines,
         const int w, const int h, T minVal = T(),
         T maxVal = T(), const char *title = "",
         const char *xLabel = "", const char *yLabel = "",
         const std::vector<PixRGB<byte> >& linescolor = std::vector<PixRGB<byte> >(0),
         const PixRGB<byte>& gridcolor=PixRGB<byte>(255,0,0),
         const PixRGB<byte>& bckcolor=PixRGB<byte>(255,255,255));

//! Draw a polygon with outlined polygon, no change outside
template <class T_or_RGB>
void drawOutlinedPolygon(Image<T_or_RGB>& img,
                       const std::vector<Point2D<int> >& polygon,
                         const T_or_RGB col,
                         const Point2D<int> trans = Point2D<int>(0,0),
                         const float rot = 0.0,
                         const float scale = 1.0,
                         const float k1 = 0, 
                         const float k2 = 0, 
                         const int rad = 1);

//! Draw a filled polygon with color col inside the polygon, no change outside
template <class T_or_RGB>
void drawFilledPolygon(Image<T_or_RGB>& img,
                       const std::vector<Point2D<int> >& polygon,
                       const T_or_RGB col);

//! Simple struct to define a bargraph meter
struct MeterInfo
{
  std::string label;
  double val;
  double valmax;
  double thresh;
  PixRGB<byte> color;
};

//! Draw some bargraph meters
Image<PixRGB<byte> > drawMeters(const MeterInfo* infos, const size_t ninfo,
                                const size_t nx, const Dims& meterdims);


template <class T>
Image<T> drawHistogram(std::vector<float> hist, int width, int height, T lineVal, T fillVal );

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


#endif // !IMAGE_DRAWOPS_H_DEFINED
