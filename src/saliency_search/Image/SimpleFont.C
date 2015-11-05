/*! @file Image/SimpleFont.C simple raster font definition */

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

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/SimpleFont.C $
// $Id: SimpleFont.C 15310 2012-06-01 02:29:24Z itti $


#include "Image/SimpleFont.H"

#include "Util/Assert.H"
#include "Util/log.H"

#include <sstream>

/* NOTE: To add a new font, you should

   (1) first generate a tightly cropped image of the 95 chars
       belonging in the font (see AppMedia/app-font2c.C for details);

   (2) add that image file to etc/fonts/ so that it can be used in the
       future to regenerate the font definition if needed;

   (3) add your font name to the list of fonts in
       devscripts/update-fonts.sh and then re-run that script;

   (4) add #include "Image/YOURFONTNAME.h" below;

   (5) add a SimpleFont wrapper for your new font to the g_std_fonts
       list below, so that it can be retrieved by the various
       SimpleFont helper functions like SimpleFont::FIXED() and
       SimpleFont::fixedMaxWidth().
*/
#include "Image/font5x7.h"
#include "Image/font6x10.h"
#include "Image/font7x13.h"
#include "Image/font8x13bold.h"
#include "Image/font9x15bold.h"
#include "Image/font10x20.h"
#include "Image/font11x22.h"
#include "Image/font12x22.h"
#include "Image/font14x26.h"
#include "Image/font15x28.h"
#include "Image/font16x29.h"
#include "Image/font20x38.h"

static const SimpleFont g_std_fonts[12] =
  {
    SimpleFont(5U, 7U, &font5x7[0][0]),
    SimpleFont(6U, 10U, &font6x10[0][0]),
    SimpleFont(7U, 13U, &font7x13[0][0]),
    SimpleFont(8U, 13U, &font8x13bold[0][0]),
    SimpleFont(9U, 15U, &font9x15bold[0][0]),
    SimpleFont(10U, 20U, &font10x20[0][0]),
    SimpleFont(11U, 22U, &font11x22[0][0]),
    SimpleFont(12U, 22U, &font12x22[0][0]),
    SimpleFont(14U, 26U, &font14x26[0][0]),
    SimpleFont(15U, 28U, &font15x28[0][0]),
    SimpleFont(16U, 29U, &font16x29[0][0]),
    SimpleFont(20U, 38U, &font20x38[0][0])
  };

// ######################################################################
SimpleFont::SimpleFont(const uint cw, const uint ch,
                       const unsigned char *cdata) :
  ww(cw), hh(ch), data(cdata)
{ }

// ######################################################################
SimpleFont::~SimpleFont()
{ }

// ######################################################################
const unsigned char * SimpleFont::charptr(const int c) const
{
  if (c < 32 || c > 126) return data; // return 'space' if out of range
  return data + (c - 32) * ww * hh;
}

// ######################################################################
SimpleFont SimpleFont::FIXED(const uint width)
{
  std::ostringstream oss;

  for (size_t i = 0; i < numStdFonts(); ++i)
    {
      if (g_std_fonts[i].ww == width)
        return g_std_fonts[i];

      if (i > 0)
        oss << ", ";
      oss << g_std_fonts[i].ww;
    }

  LFATAL("Invalid width %u - valid are: %s", width, oss.str().c_str());
  return SimpleFont(0U, 0U, NULL); // keep compiler happy
}

// ######################################################################
SimpleFont SimpleFont::fixedMaxWidth(const uint maxwidth)
{
  ASSERT(numStdFonts() > 0);

  size_t best_match = 0;

  for (size_t i = 0; i < numStdFonts(); ++i)
    {
      if (g_std_fonts[i].ww <= maxwidth ||
          g_std_fonts[i].ww < g_std_fonts[best_match].ww)
        best_match = i;
    }

  return g_std_fonts[best_match];
}

// ######################################################################
SimpleFont SimpleFont::fixedMaxHeight(const uint maxheight)
{
  ASSERT(numStdFonts() > 0);

  size_t best_match = 0;

  for (size_t i = 0; i < numStdFonts(); ++i)
    {
      if (g_std_fonts[i].hh <= maxheight ||
          g_std_fonts[i].hh < g_std_fonts[best_match].hh)
        best_match = i;
    }

  return g_std_fonts[best_match];
}

// ######################################################################
SimpleFont SimpleFont::stdFont(const size_t n)
{
  if (n >= numStdFonts())
    LFATAL("expected n < %" ZU ", but got n = %" ZU , numStdFonts(), n);

  return g_std_fonts[n];
}

// ######################################################################
size_t SimpleFont::numStdFonts()
{
  return sizeof(g_std_fonts) / sizeof(g_std_fonts[0]);
}
