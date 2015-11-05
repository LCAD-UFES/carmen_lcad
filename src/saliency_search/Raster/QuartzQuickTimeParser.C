/*!@file Raster/QuartzQuickTimeParser.C */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/QuartzQuickTimeParser.C $
// $Id: QuartzQuickTimeParser.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef RASTER_QUARTZQUICKTIMEPARSER_C_DEFINED
#define RASTER_QUARTZQUICKTIMEPARSER_C_DEFINED

#include "Raster/QuartzQuickTimeParser.H"

#ifdef HAVE_QUICKTIME_QUICKTIME_H

#include "Raster/GenericFrame.H"
#include "Util/Janitor.H"
#include "Util/log.H"

#include <QuickTime/QuickTime.h>

struct QuartzQuickTimeParser::Rep
{
  GenericFrame frame;
};

QuartzQuickTimeParser::QuartzQuickTimeParser(const char* filename)
  :
  rep(new Rep)
{
  // Convert image path to CFString
  CFStringRef inPath = CFStringCreateWithCString(NULL, filename,
                                                 CFStringGetSystemEncoding());
  if (!inPath)
    LFATAL("Could not get CFString from %s", filename);

  // create the data reference
  Handle myDataRef = NULL;
  OSType myDataRefType;
  OSErr result =
    QTNewDataReferenceFromFullPathCFString
    (inPath, (unsigned long) kQTNativeDefaultPathStyle,
     0, &myDataRef, &myDataRefType);

  if (result != noErr)
    LFATAL("Could not get DataRef for %s (error %d)", filename, result);

  Janitor<GraphicsImportComponent> importer(NULL, &CloseComponent);
  result = GetGraphicsImporterForDataRef(myDataRef, myDataRefType, &importer.it);

  if (result != noErr)
    LFATAL("Could not get GraphicsImportComponent for %s (error %d)",
           filename, result);

  unsigned long imageCount = 0;
  {
    ComponentResult res = GraphicsImportGetImageCount(importer.it, &imageCount);
    if (res != noErr)
      LFATAL("Could not get image count for %s (error %ld)",
             filename, res);
  }

  if (imageCount == 0)
    LFATAL("Oops! No images found in %s", filename);

  if (imageCount > 1)
    LFATAL("Oops! Found %lu images in %s, but can handle only 1",
           imageCount, filename);

  ASSERT(imageCount == 1);

  ImageDescriptionHandle desc;

  {
    ComponentResult res = GraphicsImportGetImageDescription(importer.it, &desc);
    if (res != noErr)
      LFATAL("Could not get image description for %s (error %ld)",
             filename, res);
  }

  LDEBUG("%s: dims are %dx%d, depth is %d, codec is '%4s'",
         filename, int((*desc)->width), int((*desc)->height),
         int((*desc)->depth),
         reinterpret_cast<const char*>(&(*desc)->cType));

  Rect bounds;
  {
    ComponentResult res = GraphicsImportGetNaturalBounds(importer.it, &bounds);
    if (res != noErr)
      LFATAL("Could not get image bounds for %s (error %ld)",
             filename, res);
  }

  CGImageRef imageRef = 0;
  if (noErr !=
      GraphicsImportCreateCGImage
      (importer.it, &imageRef,
       kGraphicsImportCreateCGImageUsingCurrentSettings))
    LFATAL("Could not create CGImage");

  CGRect rect;
  rect.origin.x = 0;
  rect.origin.y = 0;
  rect.size.width = CGImageGetWidth(imageRef);
  rect.size.height = CGImageGetHeight(imageRef);

  switch ((*desc)->depth)
    {
    case 1: case 2: case 4: case 8: case 16: case 24: case 32: // RGB
      {
        Janitor<CGColorSpaceRef> colorSpace
          (CGColorSpaceCreateWithName(kCGColorSpaceGenericRGB),
           &CGColorSpaceRelease);

        if (NULL == colorSpace.it)
          LFATAL("couldn't create rgb color space");

        LDEBUG("rgb colorspace has %" ZU " components",
               CGColorSpaceGetNumberOfComponents(colorSpace.it));

        // Yuck. CoreGraphics won't let us render directly into 24-bit
        // RGB pixels; instead we have to render into 32-bit ARGB
        // pixels with an ignored alpha field, and then copy the data
        // a second time from there into our desired 24-bit result.

        Image<byte> argb(4*CGImageGetWidth(imageRef),
                         CGImageGetHeight(imageRef),
                         NO_INIT);

        // see http://developer.apple.com/qa/qa2001/qa1037.html for a
        // list of the parameter combinations that are supported by
        // CGBitmapContextCreate()

        Janitor<CGContextRef> bitmapContext
          (CGBitmapContextCreate(argb.getArrayPtr(),
                                 argb.getWidth() / 4,
                                 argb.getHeight(),
                                 8,
                                 argb.getWidth(),
                                 colorSpace.it,
                                 kCGImageAlphaNoneSkipLast),
           &CFRelease);

        if (NULL == bitmapContext.it)
          LFATAL("couldn't create rgb bitmap context");

        CGContextDrawImage(bitmapContext.it, rect, imageRef);

        Image<PixRGB<byte> > rgb(CGImageGetWidth(imageRef),
                                 CGImageGetHeight(imageRef),
                                 NO_INIT);
        const byte* argbptr = argb.getArrayPtr();
        byte* rgbptr = reinterpret_cast<byte*>(rgb.getArrayPtr());
        const size_t n = rgb.getSize();

        for (size_t i = 0; i < n; ++i)
          {
            rgbptr[0] = argbptr[0];
            rgbptr[1] = argbptr[1];
            rgbptr[2] = argbptr[2];
            rgbptr += 3;
            argbptr += 4;
          }

        rep->frame = GenericFrame(rgb);
      }
      break;

    case 34: case 36: case 40: // grayscale
      {
        // 34=>2-bit, 36=>4-bit, 40=>8-bit
        const int depth = (*desc)->depth - 32;
        (void) depth;

        Janitor<CGColorSpaceRef> colorSpace
          (CGColorSpaceCreateWithName(kCGColorSpaceGenericGray),
           &CGColorSpaceRelease);

        if (NULL == colorSpace.it)
          LFATAL("couldn't create grayscale color space");

        LDEBUG("grayscale colorspace has %" ZU " components",
               CGColorSpaceGetNumberOfComponents(colorSpace.it));

        Image<byte> gray(CGImageGetWidth(imageRef),
                         CGImageGetHeight(imageRef),
                         NO_INIT);

        Janitor<CGContextRef> bitmapContext
          (CGBitmapContextCreate(gray.getArrayPtr(),
                                 gray.getWidth(),
                                 gray.getHeight(),
                                 8,
                                 gray.getWidth(),
                                 colorSpace.it,
                                 kCGImageAlphaNone),
           &CFRelease);

        if (NULL == bitmapContext.it)
          LFATAL("couldn't create grayscale bitmap context");

        CGContextDrawImage(bitmapContext.it, rect, imageRef);

        rep->frame = GenericFrame(gray);
        break;
      }
    }
}

QuartzQuickTimeParser::~QuartzQuickTimeParser()
{
  delete rep;
}

GenericFrameSpec QuartzQuickTimeParser::getFrameSpec() const
{
  return rep->frame.frameSpec();
}

std::string QuartzQuickTimeParser::getComments() const
{
  return std::string();
}

uint QuartzQuickTimeParser::getTagCount() const
{
  return 0;
}

bool QuartzQuickTimeParser::getTag(uint tag,
                             std::string &name,
                             std::string &value) const
{
  name = std::string();
  value = std::string();
  return false;
}

GenericFrame QuartzQuickTimeParser::getFrame()
{
  return rep->frame;
}

#endif // HAVE_QUICKTIME_QUICKTIME_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_QUARTZQUICKTIMEPARSER_C_DEFINED
