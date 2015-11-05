/*!@file Image/calibrateFunctions.C [put description here] */

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
//
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/calibrateFunctions.C $
// $Id: calibrateFunctions.C 6182 2006-01-31 18:41:41Z rjpeters $
//

#include "Image/calibrateFunctions.H"

#include "Image/Image.H"
#include "Util/Assert.H"
#include <iostream>
#include <vector>

double getError (const Image<byte>& img1,
                 const Image<byte>& img2,
                 const int shiftx, const int shifty)
{
  ASSERT(img1.isSameSize(img2));

  int w = img1.getWidth();
  int h = img1.getHeight();

  int xb1, xe1, xb2, xe2;
  if (shiftx < 0)
    {
      xb1 = 0;       xe1 = w + shiftx;
      xb2 = -shiftx; xe2 = w;
    }
  else
    {
      xb1 = shiftx; xe1 = w;
      xb2 = 0;      xe2 = w - shiftx;
    }
  ASSERT((xb1 < xe1) && (xb2 < xe2));

  int yb1, ye1, yb2, ye2;
  if (shifty < 0)
    {
      yb1 = 0;       ye1 = h + shifty;
      yb2 = -shifty; ye2 = h;
    }
  else
    {
      yb1 = shifty; ye1 = h;
      yb2 = 0;      ye2 = h - shifty;
    }
  ASSERT((yb1 < ye1) && (yb2 < ye2));

  double sum = 0.0, diff;
  Image<byte>::const_iterator ptr1 = img1.begin() + (xb1 + yb1 * w);
  Image<byte>::const_iterator ptr2 = img2.begin() + (xb2 + yb2 * w);

  for (int y = yb1; y < ye1; y++)
    {
      Image<byte>::const_iterator ptr1b = ptr1;
      Image<byte>::const_iterator ptr2b = ptr2;
      for (int x = xb1; x < xe1; x++)
        {
          diff = double(*ptr1b++) - double(*ptr2b++);
          sum += diff * diff;
        }
      ptr1 += w; ptr2 += w;
    }

  return sum / (xe1 - xb1) / (ye1 - yb1);
}


int findAlign(const Image<byte>& img1,
              const Image<byte>& img2,
              const int align_xy,
              const int range_start,
              const int range_end)
{
  ASSERT((img1.isSameSize(img2)));
  ASSERT((align_xy == 0) || (align_xy == 1));
  int dim[2], shift[2];
  dim[0] = img1.getWidth();
  dim[1] = img1.getHeight();
  shift[0] = 0; shift[1] = 0;
  //std::cout << "range: " << range_start << " : " << range_end;
  //std::cout << " - dim[" << align_xy << "] = " << dim[align_xy] << "\n";
  ASSERT((range_start < range_end) && (range_start > -dim[align_xy]) &&
         (range_end < dim[align_xy]));

  int num_elem = range_end - range_start;

  std::vector<double> err(num_elem+1);
  for (int i=0; i<= num_elem; i++)
    err[i] = -1.0;

  int left, right, middle, mid_l, mid_r;

  left = 0; shift[align_xy] = left + range_start;
  err[left] = getError(img1,img2,shift[0],shift[1]);

  right = num_elem; shift[align_xy] = right + range_start;
  err[right] = getError(img1,img2,shift[0],shift[1]);

  middle = num_elem / 2; shift[align_xy] = middle + range_start;
  err[middle] = getError(img1,img2,shift[0],shift[1]);

  while ((left != middle) && (middle != right))
    {
      mid_l = (left + middle) / 2;
      if (err[mid_l] < 0)
        {
          shift[align_xy] = mid_l + range_start;
          err[mid_l] = getError(img1,img2,shift[0],shift[1]);
        }

      mid_r = (middle + right) / 2;
      if (err[mid_r] < 0)
        {
          shift[align_xy] = mid_r + range_start;
          err[mid_r] = getError(img1,img2,shift[0],shift[1]);
        }
      if ((err[middle] < err[mid_l]) && (err[middle] < err[mid_r]))
        {
          left = mid_l;
          right = mid_r;
        }
      else if (err[mid_l] < err[mid_r])
        {
          right = middle;
          middle = mid_l;
        }
      else
        {
          left = middle;
          middle = mid_r;
        }
    }
  return middle + range_start;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
