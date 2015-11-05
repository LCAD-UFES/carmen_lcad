/*!@file Devices/QuickTimeGrabber.C Grab frames (e.g. from a camera) using QuickTime's SequenceGrabber APIs */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/QuickTimeGrabber.C $
// $Id: QuickTimeGrabber.C 9341 2008-02-28 00:13:24Z rjpeters $
//

// Much of the source code in this file is derived from code in the
// "BrideOfMungGrab" sample program, Copyright 2000-2005 Apple
// Computer, Inc., in which the included license agreement allows
// redistribution with or without modifications as long as the Apple
// Computer name is not used to endorse the modified product.

#ifndef DEVICES_QUICKTIMEGRABBER_C_DEFINED
#define DEVICES_QUICKTIMEGRABBER_C_DEFINED

#define MAC_OS_X_VERSION_MIN_REQUIRED MAC_OS_X_VERSION_10_3

#include "Devices/QuickTimeGrabber.H"

#include "Devices/DeviceOpts.H"
#include "Raster/GenericFrame.H"
#include "Util/Janitor.H"
#include "Util/log.H"
#include "Util/sformat.H"

#ifdef HAVE_QUICKTIME_QUICKTIME_H
#include <Carbon/Carbon.h>
#include <QuickTime/QuickTime.h>
#endif

#include <unistd.h>

#ifdef HAVE_QUICKTIME_QUICKTIME_H

// ######################################################################
static void setVideoChannelBounds(SGChannel videoChannel,
                                  const Rect* scaledSourceBounds,
                                  const Rect* scaledVideoBounds)
{
  // Notes: see Q&A 1250

  // calculate the matrix to transform the
  // scaledSourceBounds to the source bounds
  Rect sourceBounds;
  SGGetSrcVideoBounds(videoChannel, &sourceBounds);

  MatrixRecord scaledSourceBoundsToSourceBounds;
  RectMatrix(&scaledSourceBoundsToSourceBounds,
             scaledSourceBounds, &sourceBounds);

  // apply the same transform to the
  // scaledVideoBounds to get the video bounds
  Rect videoBounds = *scaledVideoBounds;
  TransformRect(&scaledSourceBoundsToSourceBounds, &videoBounds, 0);

  if (noErr != SGSetVideoRect(videoChannel, &videoBounds))
    {
      // some video digitizers may only be able to capture full frame
      // and will return qtParamErr or possibly digiUnimpErr if they
      // can't handle working with less than full frame
      SGSetVideoRect(videoChannel, &sourceBounds);
    }

  // the channel bounds is scaledVideoBounds offset to (0, 0)
  Rect channelBounds = *scaledVideoBounds;
  OffsetRect(&channelBounds, -channelBounds.left, -channelBounds.top);

  // Note: SGSetChannelBounds merely allows the client to specify it's
  // preferred bounds. The actual bounds returned by the vDig in the
  // image description may be different
  if (noErr != SGSetChannelBounds(videoChannel, &channelBounds))
    LFATAL("SGSetChannelBounds() failed");
}

// ######################################################################
struct QuickTimeGrabber::Impl
{
  Impl(const Dims& dims);

  ~Impl();

private:
  class SGChannelHolder
  {
  public:
    SGChannelHolder(SeqGrabComponent* owner) : it(0), itsOwner(owner) {}
    ~SGChannelHolder() { SGDisposeChannel(*itsOwner, it); }
    SGChannel it;
  private:
    SGChannelHolder(const SGChannelHolder&); // not implemented
    SGChannelHolder& operator=(const SGChannelHolder&); // not implemented
    SeqGrabComponent* itsOwner;
  };

  Janitor<SeqGrabComponent>  itsSeqGrab;        // sequence grabber
  SGChannelHolder            itsSGChanVideo;
  ImageSequence              itsDrawSeq;        // unique identifier for our draw sequence
  TimeScale                  itsTimeScale;
  TimeBase                   itsTimeBase;
  UInt8                      itsQueuedFrameCount;
  UInt8                      itsSkipFrameCount;
  unsigned int               itsSkipFrameCountTotal;
  TimeValue                  itsPrevTime;
  long                       itsFrameCount;
  GWorldPtr                  itsGWorld;
  bool                       itsGotFrame;
  Image<PixRGB<byte> >       itsCurrentImage;
  std::string                itsErrorMsg;
  bool                       itsStreamStarted;

  static pascal OSErr grabDataProc(SGChannel c, Ptr p, long len,
                                   long* /*offset*/,
                                   long /*chRefCon*/, TimeValue time,
                                   short /*writeType*/, long refCon);

  OSErr grabData(SGChannel c, Ptr p, long len, TimeValue time);

  static pascal ComponentResult
  grabCompressCompleteBottle(SGChannel c, UInt8* itsQueuedFrameCount,
                             SGCompressInfo* ci, TimeRecord* t,
                             long refCon);

public:
  void startStream();

  GenericFrame readFrame();

  std::string getSummary() const;
};

// ######################################################################
QuickTimeGrabber::Impl::Impl(const Dims& dims)
  :
  itsSeqGrab(0, &CloseComponent),
  itsSGChanVideo(&itsSeqGrab.it),
  itsDrawSeq(0),
  itsTimeScale(0),
  itsTimeBase(0),
  itsQueuedFrameCount(0),
  itsSkipFrameCount(0),
  itsSkipFrameCountTotal(0),
  itsPrevTime(0),
  itsFrameCount(0),
  itsGWorld(0),
  itsGotFrame(false),
  itsCurrentImage(),
  itsErrorMsg(),
  itsStreamStarted(false)
{
  OSErr err;

  EnterMovies();

  // open the default sequence grabber
  itsSeqGrab.it = OpenDefaultComponent(SeqGrabComponentType, 0);
  if (itsSeqGrab.it == NULL)
    LFATAL("OpenDefaultComponent() failed");

  // initialize the default sequence grabber component
  if (noErr != (err = SGInitialize(itsSeqGrab.it)))
    LFATAL("SGInitialize() failed (err=%ld)", (long) err);

  Rect scaleRect;
  MacSetRect(&scaleRect, 0, 0, dims.w(), dims.h());
  ASSERT(itsGWorld == 0);
  QTNewGWorld(&itsGWorld,
              k32ARGBPixelFormat, &scaleRect,
              NULL, NULL,
              kNativeEndianPixMap);

  // set its graphics world
  if (noErr != (err = SGSetGWorld(itsSeqGrab.it, itsGWorld, NULL)))
    LFATAL("SGSetGWorld() failed (err=%ld)", (long) err);

  // specify the destination data reference for a record operation
  // tell it we're not making a movie if the flag seqGrabDontMakeMovie
  // is used, the sequence grabber still calls your data function, but
  // does not write any data to the movie file writeType will always
  // be set to seqGrabWriteAppend
  if (noErr !=
      (err = SGSetDataRef(itsSeqGrab.it, 0, 0,
                          seqGrabDontMakeMovie | seqGrabDataProcIsInterruptSafe)))
    LFATAL("SGSetDataRef() failed (err=%ld)", (long) err);

  Impl::SGChannelHolder sgchanSound(&itsSeqGrab.it);

  if (noErr != (err = SGNewChannel(itsSeqGrab.it,
                                   VideoMediaType, &itsSGChanVideo.it)))
    LFATAL("SGNewChannel(video) failed (err=%ld)", (long) err);

  if (noErr != (err = SGNewChannel(itsSeqGrab.it,
                                   SoundMediaType, &sgchanSound.it)))
    {
      // don't care if we couldn't get a sound channel
      sgchanSound.it = NULL;
      LERROR("SGNewChannel(audio) failed (err=%ld)", (long) err);
    }

  // get the active rectangle
  Rect srcBounds;
  if (noErr != (err = SGGetSrcVideoBounds(itsSGChanVideo.it, &srcBounds)))
    LFATAL("SGGetSrcVideoBounds() failed (err=%ld)", (long) err);

  // we always want all the source
  setVideoChannelBounds(itsSGChanVideo.it, &srcBounds, &srcBounds);

  // set usage for new video channel to avoid playthrough
  // note we don't set seqGrabPlayDuringRecord
  if (noErr != (err = SGSetChannelUsage(itsSGChanVideo.it,
                                        seqGrabRecord |
                                        seqGrabLowLatencyCapture |
                                        seqGrabAlwaysUseTimeBase)))
    LFATAL("SGSetChannelUsage(video) failed (err=%ld)", (long) err);

  if (noErr != (err = SGSetChannelUsage(sgchanSound.it, seqGrabRecord |
                                        //seqGrabPlayDuringRecord |
                                        seqGrabLowLatencyCapture |
                                        seqGrabAlwaysUseTimeBase)))
    LERROR("SGSetChannelUsage(audio) failed (err=%ld)", (long) err);

  // specify a sequence grabber data function
  if (noErr != (err = SGSetDataProc(itsSeqGrab.it,
                                    NewSGDataUPP(Impl::grabDataProc),
                                    (long)(this))))
    LFATAL("SGSetDataProc() failed (err=%ld)", (long) err);

  SGSetChannelRefCon(itsSGChanVideo.it, (long)(this));

  // set up the video bottlenecks so we can get our queued frame count
  VideoBottles vb = { 0 };
  if (noErr != (err = SGGetVideoBottlenecks(itsSGChanVideo.it, &vb)))
    LFATAL("SGGetVideoBottlenecks() failed (err=%ld)", (long) err);

  vb.procCount = 9; // there are 9 bottleneck procs; this must be filled in
  vb.grabCompressCompleteProc =
    NewSGGrabCompressCompleteBottleUPP
    (Impl::grabCompressCompleteBottle);

  if (noErr != (err = SGSetVideoBottlenecks(itsSGChanVideo.it, &vb)))
    LFATAL("SGSetVideoBottlenecks() failed (err=%ld)", (long) err);

  SGSetFrameRate(itsSGChanVideo.it, FixRatio(30, 1));
}

// ######################################################################
QuickTimeGrabber::Impl::~Impl()
{
  const std::string summary = this->getSummary();
  if (summary.size() > 0)
    LINFO("%s", summary.c_str());

  if (itsSeqGrab.it != 0)
    SGStop(itsSeqGrab.it);

  // clean up the bits
  if (itsDrawSeq)
    CDSequenceEnd(itsDrawSeq);

  DisposeGWorld(itsGWorld);
}

// ######################################################################
/* grabDataProc

   Purpose: sequence grabber data procedure - this is where the work
   is done

   Notes:

   the sequence grabber calls the data function whenever any of the
   grabber's channels write digitized data to the destination movie
   file.

   NOTE: We really mean any, if you have an audio and video channel
         then the DataProc will be called for either channel whenever
         data has been captured. Be sure to check which channel is
         being passed in. In this example we never create an audio
         channel so we know we're always dealing with video.

   This data function does two things, it first decompresses captured
   video data into an offscreen GWorld, draws some status information
   onto the frame then transfers the frame to an onscreen window.

   For more information refer to Inside Macintosh: QuickTime
   Components, page 5-120

   c - the channel component that is writing the digitized data.

   p - a pointer to the digitized data.

   len - the number of bytes of digitized data.

   offset - a pointer to a field that may specify where you are to
   write the digitized data, and that is to receive a value indicating
   where you wrote the data.

   chRefCon - per channel reference constant specified using
   SGSetChannelRefCon.

   time - the starting time of the data, in the channel's time scale.

   writeType - the type of write operation being performed.

   seqGrabWriteAppend - Append new data.

   seqGrabWriteReserve - Do not write data. Instead, reserve space for
   the amount of data specified in the len parameter.

   seqGrabWriteFill - Write data into the location specified by
   offset. Used to fill the space previously reserved with
   seqGrabWriteReserve. The Sequence Grabber may call the DataProc
   several times to fill a single reserved location.

   refCon - the reference constant you specified when you assigned
   your data function to the sequence grabber.
*/
pascal OSErr QuickTimeGrabber::Impl::
grabDataProc(SGChannel c, Ptr p, long len,
             long* /*offset*/,
             long /*chRefCon*/, TimeValue time,
             short /*writeType*/, long refCon)
{
  QuickTimeGrabber::Impl* rep = (QuickTimeGrabber::Impl*)refCon;
  if (rep != NULL)
    try
      {
        return rep->grabData(c, p, len, time);
      }
    catch (...)
      {
        return -1;
      }

  return -1;
}

// ######################################################################
OSErr QuickTimeGrabber::Impl::grabData(SGChannel c, Ptr p,
                                       long len, TimeValue time)
{
  if (itsGotFrame)
    {
      LDEBUG("already got a frame on this iteration");
      return noErr;
    }

  // we only care about the video
  if (c != itsSGChanVideo.it)
    {
      return noErr;
    }

  // reset frame and time counters after a stop/start
  if (time < itsPrevTime)
    {
      LDEBUG("resetting frame/time counters (current=%ld, last=%ld)",
             (long) time, (long) itsPrevTime);
      itsPrevTime = 0;
      itsFrameCount = 0;
    }

  if (itsTimeScale == 0)
    {
      LDEBUG("setting up time scale & timebase");

      Fixed framesPerSecond;
      long  milliSecPerFrameIgnore, bytesPerSecondIgnore;

      // first time here so get the time scale & timebase
      if (noErr != SGGetChannelTimeScale(c, &itsTimeScale))
        {
          itsErrorMsg = "SGGetChannelTimeScale() failed";
          return OSErr(-1);
        }

      if (noErr != SGGetTimeBase(itsSeqGrab.it, &itsTimeBase))
        {
          itsErrorMsg = "SGGetTimeBase() failed";
          return OSErr(-1);
        }

      if (noErr != VDGetDataRate(SGGetVideoDigitizerComponent(c),
                                 &milliSecPerFrameIgnore,
                                 &framesPerSecond,
                                 &bytesPerSecondIgnore))
        {
          itsErrorMsg = "VDGetDataRate() failed";
          return OSErr(-1);
        }
    }

  if (itsDrawSeq == 0)
    {
      LDEBUG("setting up decompression sequence");

      // set up decompression sequence
      ImageDescriptionHandle imageDesc =
        (ImageDescriptionHandle)NewHandle(0);

      // retrieve a channel's current sample description, the channel
      // returns a sample description that is appropriate to the type
      // of data being captured
      if (noErr != SGGetChannelSampleDescription(c, (Handle)imageDesc))
        {
          itsErrorMsg = "SGGetChannelSampleDescription() failed";
          return OSErr(-1);
        }

      // make a scaling matrix for the sequence
      Rect sourceRect = { 0, 0 };
      sourceRect.right = (**imageDesc).width;
      sourceRect.bottom = (**imageDesc).height;

      Rect scaleRect;
      GetPixBounds(GetGWorldPixMap(itsGWorld), &scaleRect);

      // if DV do high quality 720x480 both fields
      CodecFlags cFlags =
        (kDVCNTSCCodecType == (**imageDesc).cType)
        ? codecHighQuality
        : codecNormalQuality;

      MatrixRecord scaleMatrix;
      RectMatrix(&scaleMatrix, &sourceRect, &scaleRect);

      LINFO("sourceRect = %dx%d, scaleRect = %dx%d",
            sourceRect.right - sourceRect.left,
            sourceRect.bottom - sourceRect.top,
            scaleRect.right - scaleRect.left,
            scaleRect.bottom - scaleRect.top);

      // begin the process of decompressing a sequence of frames this
      // is a set-up call and is only called once for the sequence -
      // the ICM will interrogate different codecs and construct a
      // suitable decompression chain, as this is a time consuming
      // process we don't want to do this once per frame (eg. by using
      // DecompressImage) for more information see Ice Floe #8
      // http://developer.apple.com/quicktime/icefloe/dispatch008.html
      // the destination is specified as the GWorld
      CGrafPtr dest = itsGWorld;
      if (noErr != DecompressSequenceBeginS
          (&itsDrawSeq,     // pointer to field to receive unique ID for sequence
           imageDesc,       // handle to image description structure
           p,               // points to the compressed image data
           len,             // size of the data buffer
           dest,            // port for the DESTINATION image
           NULL,            // graphics device handle, if port is set, set to NULL
           NULL,            // decompress the entire source image - no source extraction
           &scaleMatrix,    // transformation matrix
           srcCopy,         // transfer mode specifier
           (RgnHandle)NULL, // clipping region in dest. coordinate system to use as a mask
           0,               // flags
           cFlags,          // accuracy in decompression
           bestSpeedCodec)) // compressor identifier or special identifiers ie. bestSpeedCodec
        {
          itsErrorMsg = "DSeqBegin failed";
          return OSErr(-1);
        }

      DisposeHandle((Handle)imageDesc);

    } // itsDrawSeq == 0

  // get the TimeBase time and figure out the delta between that time
  // and this frame time
  const TimeValue timeBaseTime = GetTimeBaseTime(itsTimeBase,
                                                 itsTimeScale, NULL);
  const TimeValue timeBaseDelta = timeBaseTime - time;
  const TimeValue frameTimeDelta = time - itsPrevTime;

  if (timeBaseDelta < 0)
    {
      itsErrorMsg = "bogus timeBaseDelta";
      return OSErr(-1);
    }

  // if we have more than one queued frame and our capture rate drops
  // below 10 frames, skip the frame to try and catch up
  if ((itsQueuedFrameCount > 1)
      &&  ((itsTimeScale / frameTimeDelta) < 10)
      && (itsSkipFrameCount < 15))
    {
      LDEBUG("dropping frame");
      ++itsSkipFrameCount;
      ++itsSkipFrameCountTotal;
    }
  else
    {
      itsFrameCount++;

      CodecFlags ignore;

      // decompress a frame into the window - can queue a frame for async decompression when passed in a completion proc
      if (noErr != DecompressSequenceFrameS
          (itsDrawSeq, // sequence ID returned by DecompressSequenceBegin
           p,          // pointer to compressed image data
           len,        // size of the buffer
           0,          // in flags
           &ignore,    // out flags
           NULL))      // async completion proc
        {
          itsErrorMsg =  "DSeqFrameS failed";
          return OSErr(-1);
        }

      // get the information we need from the GWorld
      Rect pbound;
      GetPixBounds(GetGWorldPixMap(itsGWorld), &pbound);

      char* const baseAddr =
        GetPixBaseAddr(GetGWorldPixMap(itsGWorld));

      const long rowBytes =
        QTGetPixMapHandleRowBytes(GetGWorldPixMap(itsGWorld));

      itsCurrentImage.resize(Dims(pbound.right - pbound.left,
                                     pbound.bottom - pbound.top));

      Image<PixRGB<byte> >::iterator itr = itsCurrentImage.beginw();

      for (int y = pbound.top; y < pbound.bottom; ++y)
        {
          char* p = baseAddr + rowBytes * (y-pbound.top);

          for (int x = pbound.left; x < pbound.right; ++x)
            {
              const UInt32 color = *((UInt32*)(p) + x - pbound.left);
              const UInt32 R = (color & 0x00FF0000) >> 16;
              const UInt32 G = (color & 0x0000FF00) >> 8;
              const UInt32 B = (color & 0x000000FF) >> 0;

              *itr++ = PixRGB<byte>(R,G,B);
            }
        }

      itsSkipFrameCount = 0;
      itsPrevTime = time;
      itsGotFrame = true;
    }

  // status information
  const float fps = (float)itsTimeScale / (float)frameTimeDelta;
  const float averagefps = ((float)itsFrameCount * (float)itsTimeScale) / (float)time;
  const UInt8 minutes = (time / itsTimeScale) / 60;
  const UInt8 seconds = (time / itsTimeScale) % 60;
  const UInt8 frames = (time % itsTimeScale) / frameTimeDelta;
  LDEBUG("#%06ld t:%ld nq:%u, %02d:%02d.%02d, fps:%5.1f av:%5.1f",
         itsFrameCount, time, itsQueuedFrameCount,
         minutes, seconds, frames, fps, averagefps);

  return noErr;
}

// ######################################################################
/* grabCompressCompleteBottle

   Purpose: figure out how many frames are queued by the vDig

   Notes: the UInt8 *queuedFrameCount replaces Boolean *done. (0
   (==false) still means no frames, and 1 (==true) one, but if more
   than one are available, the number should be returned here - The
   value 2 previously meant more than one frame, so some VDIGs may
   return 2 even if more than 2 are available, and some will still
   return 1 as they are using the original definition.
*/
pascal ComponentResult QuickTimeGrabber::Impl::
grabCompressCompleteBottle(SGChannel c, UInt8* queuedFrameCount,
                           SGCompressInfo* ci, TimeRecord* t, long refCon)
{
  QuickTimeGrabber::Impl* rep = (QuickTimeGrabber::Impl*)refCon;
  if (NULL == rep) return -1;

  // call the original proc; you must do this
  const OSErr err = SGGrabCompressComplete(c, queuedFrameCount, ci, t);

  // save the queued frame count so we have it
  rep->itsQueuedFrameCount = *queuedFrameCount;

  return err;
}

// ######################################################################
void QuickTimeGrabber::Impl::startStream()
{
  // lights...camera...
  if (noErr != SGPrepare(itsSeqGrab.it, false, true))
    LFATAL("SGPrepare() failed");

  // ...action
  if (noErr != SGStartRecord(itsSeqGrab.it))
    LFATAL("SGStartRecord() failed");

  itsStreamStarted = true;
}

// ######################################################################
GenericFrame QuickTimeGrabber::Impl::readFrame()
{
  if (!itsStreamStarted)
    this->startStream();

  while (1)
    {
      itsGotFrame = false;
      itsErrorMsg = "";
      if (noErr != SGIdle(itsSeqGrab.it))
        LFATAL("SGIdle() failed");

      if (itsErrorMsg.length() > 0)
        {
          // some error specific to SGIdle occurred - any errors
          // returned from the data proc will also show up here and we
          // don't want to write over them

          // in QT 4 you would always encounter a cDepthErr error
          // after a user drags the window, this failure condition has
          // been greatly relaxed in QT 5 it may still occur but
          // should only apply to vDigs that really control the screen

          // you don't always know where these errors originate from,
          // some may come from the VDig...

          LFATAL("QuickTimeGrabber error during SGIdle (%s)",
                 itsErrorMsg.c_str());

          // ...to fix this we simply call SGStop and SGStartRecord
          // again calling stop allows the SG to release and
          // re-prepare for grabbing hopefully fixing any problems,
          // this is obviously a very relaxed approach
          SGStop(itsSeqGrab.it);
          SGStartRecord(itsSeqGrab.it);
        }

      if (itsGotFrame)
        return GenericFrame(itsCurrentImage);

      usleep(20000);
    }
}

// ######################################################################
std::string QuickTimeGrabber::Impl::getSummary() const
{
  if (itsPrevTime <= 0 || itsTimeScale <= 0)
    return std::string();

  const double averagefps = (double(itsFrameCount) * double(itsTimeScale))
    / double(itsPrevTime);
  const UInt8 minutes = (itsPrevTime / itsTimeScale) / 60;
  const UInt8 seconds = (itsPrevTime / itsTimeScale) % 60;
  return sformat("summary nframes:%ld, ndrop:%u, %02d:%02d, avg fps:%5.1f",
                 itsFrameCount, itsSkipFrameCountTotal,
                 minutes, seconds, averagefps);
}

#endif // HAVE_QUICKTIME_QUICKTIME_H

// ######################################################################
QuickTimeGrabber::QuickTimeGrabber(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName)
  :
  FrameIstream(mgr, descrName, tagName),
  itsDims(&OPT_FrameGrabberDims, this),
  rep(0)
{}

// ######################################################################
QuickTimeGrabber::~QuickTimeGrabber()
{
#ifdef HAVE_QUICKTIME_QUICKTIME_H
  if (rep)
    delete rep;
#endif
}

// ######################################################################
void QuickTimeGrabber::startStream()
{
  if (!this->started())
    LFATAL("start() must be called before startStream()");

#ifndef HAVE_QUICKTIME_QUICKTIME_H
  LFATAL("you must have QuickTime installed to use QuickTimeGrabber");
#else
  ASSERT(rep != 0);
  rep->startStream();
#endif
}

// ######################################################################
GenericFrameSpec QuickTimeGrabber::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGB_U8;
  result.videoFormat = VIDFMT_RGB24;
  result.videoByteSwap = false;
  result.dims = itsDims.getVal();
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame QuickTimeGrabber::readFrame()
{
  if (!this->started())
    LFATAL("start() must be called before readFrame()");

#ifndef HAVE_QUICKTIME_QUICKTIME_H
  LFATAL("you must have QuickTime installed to use QuickTimeGrabber");
  /* can't happen */ return GenericFrame();
#else
  ASSERT(rep != 0);
  return rep->readFrame();
#endif
}

// ######################################################################
void QuickTimeGrabber::start1()
{
  FrameIstream::start1();

#ifndef HAVE_QUICKTIME_QUICKTIME_H
  LFATAL("you must have QuickTime installed to use QuickTimeGrabber");
#else
  ASSERT(rep == 0);
  rep = new Impl(itsDims.getVal());
#endif
}

// ######################################################################
void QuickTimeGrabber::stop2()
{
  FrameIstream::stop2();

#ifndef HAVE_QUICKTIME_QUICKTIME_H
  LERROR("you must have QuickTime installed to use QuickTimeGrabber");
#else
  ASSERT(rep != 0);
  delete rep;
  rep = 0;
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_QUICKTIMEGRABBER_C_DEFINED
