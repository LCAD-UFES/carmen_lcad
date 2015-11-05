/*!@file Image/IO.C I/O operations for Image's
 */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/IO.C $
// $Id: IO.C 7811 2007-01-26 23:38:43Z rjpeters $
//

#ifndef IMAGE_IO_C_DEFINED
#define IMAGE_IO_C_DEFINED

#include "Image/IO.H"

#include "Image/Image.H"
#include "Image/Pixels.H"

#include <iomanip>
#include <vector>

namespace
{
  // The point of these little functions is to convert numbers to a type
  // that will print as a number; basically the only problem is with byte,
  // which is likely a typedef for char, so if it is printed as-is, then we
  // get the characters themselves rather than their numeric
  // representation... so by overloading we can make printable(byte) return
  // an int.


  // The default printable() is just a template that returns the val unchanged.
  template <class T> T printable(T val) { return val; }

  // Overloads of printable() can specialize the behavior for specific
  // types, such as byte here:
  int printable(byte val) { return int(val); }

}

// ######################################################################
template <class T>
std::ostream& operator<<(std::ostream& os, const Image<T>& img)
{
  os << "[\n";
  for (int y = 0; y < img.getHeight(); ++y)
    {
      for (int x = 0; x < img.getWidth(); ++x)
        {
          os << std::setw(5)
             << std::showpoint << std::fixed << std::setprecision(3)
             << printable(img.getVal(x,y)) << " ";
        }
      os << "\n";
    }
  os << "]\n";

  return os;
}

// ######################################################################
template <class T>
void writeImageToStream(std::ostream& os, const Image<T>& img)
{
  Dims dims = img.getDims();
  os << dims.w() << ' ' << dims.h() << '\n';

  typename Image<T>::const_iterator ptr = img.begin();
  for (int y = 0; y < dims.h(); ++y)
    {
      for (int x = 0; x < dims.w(); ++x)
        os << *ptr++ << ' ';

      os << '\n';
    }
  os << '\n';
}

// ######################################################################
template <class T>
void readImageFromStream(std::istream& is, Image<T>& img)
{
  int w,h;
  is >> w; is >> h;
  img.resize(w,h,NO_INIT);
  typename Image<T>::iterator ptr;
  for (ptr = img.beginw(); ptr != img.endw(); ++ptr)
    is >> *ptr;
}

// ######################################################################
// Get the explicit instantiations:
#include "inst/Image/IO.I"

template std::ostream& operator<<(std::ostream&, const Image<int>&);
template std::ostream& operator<<(std::ostream&, const Image<double>&);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_IO_C_DEFINED
