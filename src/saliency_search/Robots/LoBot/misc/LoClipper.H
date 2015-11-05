/**
   \file  Robots/LoBot/misc/LoClipper.H
   \brief Line clipping with the Cohen-Sutherland algorithm.

   This file defines a class that implements the Cohen-Sutherland line
   clipping algorithm for clipping a line segment to a rectangle
   aligned with the principal axes.
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoClipper.H $
// $Id: LoClipper.H 13445 2010-05-21 06:12:57Z mviswana $
//

#ifndef LOBOT_COHEN_SUTHERLAND_DOT_H
#define LOBOT_COHEN_SUTHERLAND_DOT_H

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//----------------------- CLASS DEFINITION -----------------------------

/**
   \class lobot::Clipper
   \brief A line clipper.

   This class implements the Cohen-Sutherland line clipping algorithm for
   clipping lines against rectangles aligned with the principal axes (aka
   upright rectangles).

   Typical usage of this class is as shown below:

      // Create clipper object and specify clipping boundary
      Clipper clipper(left, right, bottom, top) ;

      // Specify line to be clipped
      double line[4] ; // first two elements are (x0,y0); next two (x1,y1)
      double clipped_line[4] ;
      Clipper::ClipInfo clip_flag = clipper.clip(line, clipped_line) ;

      // Examine clip flag returned by line clipper and do whatever is
      // required with the clipped line...
*/
class Clipper {
   /// This line clipper only works with rectangular clipping boundaries.
   /// Furthermore, the clipping boundary must be an upright rectangle,
   /// i.e., be aligned with the principal axes. This array stores the
   /// clipping rectangle's coordinates like so:
   ///
   ///   m_clip_boundary[0] = left
   ///   m_clip_boundary[1] = right
   ///   m_clip_boundary[2] = bottom
   ///   m_clip_boundary[3] = top
   float m_clip_boundary[4] ;

public:
   /// When this Cohen-Sutherland line clipper is created, it should be
   /// told the clipping boundary to use.
   Clipper(float left, float right, float bottom, float top) ;

   /// This constructor allows clients to pass the clipping boundary in
   /// via an array. The array is expected to have four elements that
   /// supply the boundary like so:
   ///   clip_boundary[0] = left
   ///   clip_boundary[1] = right
   ///   clip_boundary[2] = bottom
   ///   clip_boundary[3] = top
   Clipper(const float clip_boundary[4]) ;

   /// It is possible to create an instance of this clipper with some
   /// clipping boundary and then later change that. These methods allow
   /// clients to reset the clipping boundary to new values.
   //@{
   void clip_boundary(float left, float right, float bottom, float top) {
      m_clip_boundary[0] = left ;
      m_clip_boundary[1] = right ;
      m_clip_boundary[2] = bottom ;
      m_clip_boundary[3] = top ;
   }

   void clip_boundary(const float clip_boundary[4]) {
      m_clip_boundary[0] = clip_boundary[0] ;
      m_clip_boundary[1] = clip_boundary[1] ;
      m_clip_boundary[2] = clip_boundary[2] ;
      m_clip_boundary[3] = clip_boundary[3] ;
   }
   //@}

   /// A line may lie completely or partially inside the clipping
   /// boundary. It may also be completely outside the clipping boundary.
   /// This implementation of the Cohen-Sutherland algorithm informs
   /// clients of what the original situation was using these bit flags.
   enum {
      COMPLETELY_INSIDE    =  1,
      COMPLETELY_OUTSIDE   =  2,
      FIRST_POINT_CLIPPED  =  4,
      SECOND_POINT_CLIPPED =  8,
      BOTH_POINTS_CLIPPED  = 12,
   } ;

   /// This method implements the Cohen-Sutherland line clipping
   /// algorithm. It expects its first argument to be an array specifying
   /// the input line's end points. The clipped line is passed back to
   /// the client via the second parameter. Additionally, the function's
   /// return value specifies whether the original line was completely or
   /// partially inside the clipping boundary or completely outside of
   /// it.
   ///
   /// The input and output line's end points are stored in arrays of
   /// four elements like so:
   ///               array[0] = x0     array[1] = y0
   ///               array[2] = x1     array[3] = y1
   ///
   /// The return value is a set of bit flags that work like so:
   ///     bit 0 = 1 ==> original line was completely inside
   ///     bit 1 = 1 ==> original line was completely outside
   ///     bit 2 = 1 ==> original line was partially inside and the
   ///                   first end-point was clipped
   ///     bit 3 = 1 ==> original line was partially inside and the
   ///                   second end-point was clipped
   ///
   /// Bits 0, 1, and the remaining two are mutually exclusive, i.e., if
   /// bit 0 is on, then none of the others can be on; if bit 1 is on,
   /// all the others will be off. Bits 2 and 3 can be on at the same
   /// time; but if either of them is on, then bit 0 and 1 will be off.
   ///
   /// Rather than check the return code with magic numbers, the caller
   /// can use the COMPLETELY_INSIDE, COMPLETELY_OUTSIDE,
   /// FIRST_POINT_CLIPPED, SECOND_POINT_CLIPPED and/or
   /// BOTH_POINTS_CLIPPED enums defined above.
   unsigned char clip(const float end_points[4],
                            float new_end_points[4]) const ;

private:
   /// The Cohen-Sutherland algorithm works by partitioning "space" into
   /// nine areas as shown below:
   ///
   ///                            |      |
   ///                       TTFF | FTFF | FTTF
   ///                            |      |
   ///                      ------+------+------
   ///                            |      |
   ///                       TFFF | FFFF | FFTF
   ///                            |      |
   ///                      ------+------+------
   ///                            |      |
   ///                       TFFT | FFFT | FFTT
   ///                            |      |
   ///
   /// The input line's end points are assigned a 4-bit code. Each bit in
   /// this code corresponds to a side of the clipping rectangle. This
   /// enum "de-magic-numbers" these code bits.
   enum {
      LEFT_BIT = 8, TOP_BIT = 4, RIGHT_BIT = 2, BOTTOM_BIT = 1,
   } ;

   /// This function examines the point passed in to it and returns a
   /// four bit code specifying where the point is w.r.t. the clipping
   /// boundary's sides. The point is specified with an array of two
   /// elements, wherein the first element is the point's x-coordinate
   /// and the second element its y-coordinate.
   unsigned char cs_code(const float point[2]) const ;

   /// A line lying completely inside the clipping rectangle will have
   /// both its end-points' codes zero. OR-ing the two codes will thus
   /// yield zero. Therefore, a line lying completely inside the clipping
   /// rectangle can be trivially accepted by OR-ing the Cohen-Sutherland
   /// codes of its end points and checking if the result is zero.
   bool trivial_accept(unsigned char code1, unsigned char code2) const {
      return (code1 | code2) == 0 ;
   }

   /// The Cohen-Sutherland codes for the end points of a line lying
   /// completely to one side of the clipping rectangle will have at
   /// least one bit on in the same position. Thus, such a line can be
   /// trivially accepted by AND-ing its end points' codes and checking
   /// if the result is non-zero.
   bool trivial_reject(unsigned char code1, unsigned char code2) const {
      return (code1 & code2) != 0 ;
   }

   /// This function checks if a point lies outside the clipping
   /// rectangle. The point is specified with an array of two elements;
   /// the first element is the point's x-coordinate and the second one
   /// its y-coordinate.
   bool outside(const float point[2]) const {
      return (   point[0] < m_clip_boundary[0]
              || point[0] > m_clip_boundary[1]
              || point[1] < m_clip_boundary[2]
              || point[1] > m_clip_boundary[3]) ;
   }

   /// If a line cannot be trivially accepted or rejected, we have to do
   /// a little geometry to find the line's intersection points with the
   /// clipping rectangle and chop it down to size. This method takes
   /// care of the math for doing this.
   void chop(float point[2], unsigned char code, float dx, float dy) const ;
} ;

//----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
