/*!@file Media/QuartzQuickTimeDecoder.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/QuartzQuickTimeDecoder.C $
// $Id: QuartzQuickTimeDecoder.C 13800 2010-08-18 20:58:25Z dberg $
//

// Portions of the source code in this file are derived from code in
// the QTPixelBufferVCToCGImage sample program, Copyright 2005-2006
// Apple Computer, Inc., in which the included license agreement
// allows redistribution with or without modifications as long as the
// Apple Computer name is not used to endorse the modified product.

#ifndef MEDIA_QUARTZQUICKTIMEDECODER_C_DEFINED
#define MEDIA_QUARTZQUICKTIMEDECODER_C_DEFINED

#include "Media/QuartzQuickTimeDecoder.H"

#ifdef HAVE_QUICKTIME_QUICKTIME_H

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/log.H"

// ######################################################################
static GenericFrame PixelBuffer2GenericFrame(CVImageBufferRef inImage)
{
  if (NULL == inImage) return GenericFrame();

  const byte* const baseAddress =
    static_cast<const byte*>(CVPixelBufferGetBaseAddress(inImage));
  const size_t bytesPerRow = CVPixelBufferGetBytesPerRow(inImage);
  const size_t width = CVPixelBufferGetWidth(inImage);
  const size_t height = CVPixelBufferGetHeight(inImage);

  const OSType inPixelFormat =
    CVPixelBufferGetPixelFormatType((CVPixelBufferRef)inImage);

  LDEBUG("PixelBuffer FormatType: %lx", inPixelFormat);

  switch(inPixelFormat)
    {
    case k32ARGBPixelFormat:

      LDEBUG("baseAddress=%p", baseAddress);
      LDEBUG("bytesPerRow=%u", (unsigned int) bytesPerRow);

      {
        Image<PixRGB<byte> > rgb(width, height, NO_INIT);
        byte* data = reinterpret_cast<byte*>(rgb.getArrayPtr());

        for (size_t y = 0; y < height; ++y)
          {
            const byte* row = baseAddress + y*bytesPerRow;
            for (size_t x = 0; x < width; ++x)
              {
                // alpha = row[0];
                data[0] = row[1];
                data[1] = row[2];
                data[2] = row[3];
                data += 3;
                row += 4;
              }
          }

        return GenericFrame(rgb);
      }

      break;
    default:
      LFATAL("I don't know what to do with this format!");
      break;
    }

  /* can't happen */ return GenericFrame();
}

// ######################################################################
static void SetNumberValue(CFMutableDictionaryRef inDict,
                           CFStringRef inKey, SInt32 inValue)
{
  CFNumberRef number = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &inValue);
  if (NULL == number)
    LFATAL("CFNumberCreate() failed");

  CFDictionarySetValue(inDict, inKey, number);

  CFRelease(number);
}

// ######################################################################
/* Create a QuickTime Pixel Buffer Context
   This function creates a QuickTime Visual Context which will produce CVPixelBuffers
*/
static QTVisualContextRef CreatePixelBufferContext(SInt32 inPixelFormat,
                                                   const CGRect* inBounds)
{
  if (0 == inPixelFormat)
    LFATAL("pixel format must be non-zero");

  if (CGRectIsNull(*inBounds))
    LFATAL("bounds rect must be non-empty");

  // Pixel Buffer attributes
  Janitor<CFMutableDictionaryRef> pixelBufferOptions
    (CFDictionaryCreateMutable(kCFAllocatorDefault, 0,
                               &kCFTypeDictionaryKeyCallBacks,
                               &kCFTypeDictionaryValueCallBacks),
     &CFRelease);

  if (NULL == pixelBufferOptions.it)
    LFATAL("couldn't create pixelBufferOptions");

  // the pixel format we want
  SetNumberValue(pixelBufferOptions.it, kCVPixelBufferPixelFormatTypeKey, inPixelFormat);

  // size
  SetNumberValue(pixelBufferOptions.it, kCVPixelBufferWidthKey, int(inBounds->size.width));
  SetNumberValue(pixelBufferOptions.it, kCVPixelBufferHeightKey, int(inBounds->size.height));

  // alignment
  SetNumberValue(pixelBufferOptions.it, kCVPixelBufferBytesPerRowAlignmentKey, 16);

  // QT Visual Context attributes
  Janitor<CFMutableDictionaryRef> visualContextOptions
    (CFDictionaryCreateMutable(kCFAllocatorDefault, 0,
                               &kCFTypeDictionaryKeyCallBacks,
                               &kCFTypeDictionaryValueCallBacks),
     &CFRelease);

  if (NULL == visualContextOptions.it)
    LFATAL("couldn't create visualContextOptions");

  // set the pixel buffer attributes for the visual context
  CFDictionarySetValue(visualContextOptions.it,
                       kQTVisualContextPixelBufferAttributesKey,
                       pixelBufferOptions.it);

  // create a Pixel Buffer visual context
  QTVisualContextRef outVisualContext;
  if (noErr != QTPixelBufferContextCreate(kCFAllocatorDefault,
                                          visualContextOptions.it,
                                          &outVisualContext))
    LFATAL("couldn't create visualContext");

  if (NULL == outVisualContext)
    LFATAL("newly created visualContext was null");

  return outVisualContext;
}

// ######################################################################
/* A Callback to receive notifications when a new image becomes
   available.  This callback is called from random threads and is best
   used as a notification that something has changed during playback
   to the visual context.
*/
void QuartzQuickTimeDecoder::
imageAvailableCallback(QTVisualContextRef visualContext,
                       const CVTimeStamp* timeStamp, void* refCon)
{
  // Print out some information about the timeStamp
  LDEBUG("CVTimeStamp Flags: %llu", timeStamp->flags);

  if (timeStamp->flags & kCVTimeStampVideoTimeValid)
    LDEBUG("CVTimeStamp VideoTime: %lld", timeStamp->videoTime);

  if (timeStamp->flags & kCVTimeStampHostTimeValid)
    LDEBUG("CVTimeStamp HostTime: %llu", timeStamp->hostTime);

  if (timeStamp->flags & kCVTimeStampRateScalarValid)
    LDEBUG("CVTimeStamp Rate: %1.0g", timeStamp->rateScalar);

  QuartzQuickTimeDecoder* qqd = (QuartzQuickTimeDecoder*) refCon;

  // Check to make sure we do have an image for this time and if so then grab it
  if (!QTVisualContextIsNewImageAvailable(qqd->itsVisualContext.it, timeStamp))
    {
      qqd->itsCallbackError = "no new image was available";
    }
  else
    {
      CVImageBufferRef newImage = NULL;
      if (noErr != QTVisualContextCopyImageForTime(qqd->itsVisualContext.it,
                                                   kCFAllocatorDefault, timeStamp,
                                                   &newImage))
        {
          qqd->itsCallbackError = "QTVisualContextCopyImageForTime() failed";
        }
      else if (NULL == newImage)
        {
          qqd->itsCallbackError =
            "QTVisualContextCopyImageForTime() gave a null image";
        }
      else if (noErr != (CVPixelBufferLockBaseAddress
                         ((CVPixelBufferRef)newImage, 0)))
        {
          qqd->itsCallbackError = "CVPixelBufferLockBaseAddress failed";
        }
      else
        {
          // Get a CGImage from the returned CV pixel buffer
          try
            {
              qqd->itsFrame = PixelBuffer2GenericFrame(newImage);
              ++qqd->itsFrameNumber;
            }
          catch (std::exception& e)
            {
              qqd->itsCallbackError = e.what();
            }
          catch (...)
            {
              qqd->itsCallbackError = "unknown error";
            }

          CVPixelBufferUnlockBaseAddress((CVPixelBufferRef)newImage, 0);
        }

      if (newImage)
        CVPixelBufferRelease(newImage);
    }

  QTVisualContextTask(qqd->itsVisualContext.it);
}

// ######################################################################
QuartzQuickTimeDecoder::QuartzQuickTimeDecoder(const char* fname)
  :
  itsMovie(NULL, &DisposeMovie),
  itsVisualContext(NULL, &QTVisualContextRelease),
  itsFrame(),
  itsFrameNumber(0),
  itsNextTime(0),
  itsNextFramePushback(false),
  itsFirstFrame(true)
{
  // Initialize QuickTime
  EnterMovies();

  // Convert movie path to CFString
  CFStringRef inPath = CFStringCreateWithCString(NULL, fname,
                                                 CFStringGetSystemEncoding());
  if (!inPath)
    LFATAL("Could not get CFString from %s", fname);

  // create the data reference
  Janitor<Handle> myDataRef(NULL, &DisposeHandle);
  OSType myDataRefType;
  if (noErr != QTNewDataReferenceFromFullPathCFString
      (inPath, (unsigned long) kQTNativeDefaultPathStyle,
       0, &myDataRef.it, &myDataRefType))
    LFATAL("Could not get DataRef for %s", fname);

  // get the Movie
  short actualResId = DoTheRightThing;
  if (noErr != NewMovieFromDataRef(&itsMovie.it, newMovieActive,
                                   &actualResId, myDataRef.it, myDataRefType))
    LFATAL("Could not get Movie from DataRef for %s", fname);

  // Create the QT Pixel Buffer Visual Context
  Rect bounds;
  GetMovieBox(itsMovie.it, &bounds);

  HIRect theBounds;
  theBounds.origin.x = bounds.left;
  theBounds.origin.y = bounds.top;
  theBounds.size.width = bounds.right - bounds.left;
  theBounds.size.height = bounds.bottom - bounds.top;

  itsDims = Dims(int(theBounds.size.width), int(theBounds.size.height));

  LDEBUG("theBounds2 = {%f, %f, %f, %f}\n",
         theBounds.origin.x, theBounds.origin.y,
         theBounds.size.width, theBounds.size.height);

  itsVisualContext.it =
    CreatePixelBufferContext(k32ARGBPixelFormat, &theBounds);

  if (noErr != SetMovieVisualContext(itsMovie.it, itsVisualContext.it))
    LFATAL("SetMovieVisualContext failed");

  // Install our visual context callback we'll use as a notification mechanism
  if (noErr !=
      QTVisualContextSetImageAvailableCallback
      (itsVisualContext.it, &QuartzQuickTimeDecoder::imageAvailableCallback,
       this))
    LFATAL("QTVisualContextSetImageAvailableCallback failed");
}

// ######################################################################
QuartzQuickTimeDecoder::~QuartzQuickTimeDecoder()
{}

// ######################################################################
int QuartzQuickTimeDecoder::apparentFrameNumber() const
{
  return
    itsNextFramePushback
    ? itsFrameNumber - 1
    : itsFrameNumber;
}

// ######################################################################
GenericFrameSpec QuartzQuickTimeDecoder::peekFrameSpec()
{
//   if (itsFirstFrame)
//     {
//       // if we're still waiting for the first frame, then we can't
//       // possibly have a pushback frame:
//       ASSERT(!itsNextFramePushback);

//       readRawFrame();
//       itsNextFramePushback = true;
//     }

//   ASSERT(!itsFirstFrame);

  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_AUTO;
  result.videoByteSwap = false;
  result.dims = itsDims;
  result.floatFlags = 0;

  return result;
}

// ######################################################################
VideoFrame QuartzQuickTimeDecoder::readVideoFrame()
{
  return this->readFrame().asVideo();
}

// ######################################################################
Image<PixRGB<byte> > QuartzQuickTimeDecoder::readRGB()
{
  return this->readFrame().asRgb();
}

// ######################################################################
bool QuartzQuickTimeDecoder::readAndDiscardFrame()
{
  return (this->readFrame().initialized());
}

// ######################################################################
GenericFrame QuartzQuickTimeDecoder::readFrame()
{
  if (itsNextTime < 0)
    return GenericFrame();

  // else...

  if (itsNextFramePushback)
    {
      itsNextFramePushback = false;
      ASSERT(itsFrame.initialized());
      const GenericFrame f = itsFrame;
      itsFrame = GenericFrame();
      return f;
    }

  const int oldFrameNumber = itsFrameNumber;

  if (itsFirstFrame)
    {
      PrerollMovie(itsMovie.it, 0, fixed1);

      MoviesTask(itsMovie.it, 0);
    }

  // It's a little tricky trying to get things started properly for
  // both mpeg-1 and mpeg-4 movies. Apparently with mpeg-1 movies, the
  // first MoviesTask() call (under if(itsFirstFrame)) does NOT
  // trigger a drawCompleteCallback(). We test whether a new frame has
  // been decoded by checking if itsFrameNumber has changed from
  // oldFrameNumber. We expect that the 'gotframe' variable here will
  // be true only on itsFirstFrame of mpeg-4 movies, and will be false
  // on other frames of mpeg-4 movies and will be false for all frames
  // (including itsFirstFrame) of mpeg-1 movies.

  const bool gotframe = (itsFrameNumber != oldFrameNumber);

  TimeValue current = GetMovieTime(itsMovie.it, NULL);
  GetMovieNextInterestingTime(itsMovie.it, nextTimeStep, 0, NULL,
                              current, fixed1, &itsNextTime, NULL);

  LDEBUG("current=%ld next=%ld", current, itsNextTime);

  // if we haven't yet gotten a frame, then we need to issue another
  // MoviesTask() call
  if (!gotframe)
    MoviesTask(itsMovie.it, 0);

  // wait for the drawCompleteCallback() to indicate the current frame
  // has been rendered (actually, I'm not sure if this is necessary or
  // not -- if MoviesTask() is single-threaded, then it's not
  // necessary, but I'm not positive that it's single-threaded)

  while (itsFrameNumber == oldFrameNumber && itsCallbackError.length() == 0)
    {
      LDEBUG("waiting for drawCompleteCallback");
      usleep(10000);
    }

  if (itsCallbackError.length() > 0)
    {
      const std::string s = itsCallbackError;
      itsCallbackError = std::string();
      LFATAL("error during callback: %s", s.c_str());
    }

  LDEBUG("frame #%d @ %ld", itsFrameNumber, current);

  if (itsNextTime >= 0)
    SetMovieTimeValue(itsMovie.it, itsNextTime);

  itsFirstFrame = false;

  ASSERT(itsFrame.initialized());
  const GenericFrame f = itsFrame;
  itsFrame = GenericFrame();
  return f;
}

#endif // HAVE_QUICKTIME_QUICKTIME_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_QUARTZQUICKTIMEDECODER_C_DEFINED
