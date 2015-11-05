/*!@file Devices/RTSPGrabber.C video stream grabber froman rtsp source using LiveMedia */


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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/RTSPGrabber.C $
// $Id: RTSPGrabber.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Devices/RTSPGrabber.H"

#include "Component/OptionManager.H" // for option alias requests
#include "Component/ModelOptionDef.H"
#include "Devices/DeviceOpts.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/MathOps.H"
#include "Raster/GenericFrame.H"
#include "Util/Assert.H"
#include "Util/SimTime.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"
#include "Video/FfmpegFrame.H"


#ifdef INVT_HAVE_LIVEMEDIA
#include <groupsock/GroupsockHelper.hh>
#endif

#include <unistd.h>

#ifdef INVT_HAVE_LIVEMEDIA
void sessionAfterPlaying(void* /*clientData*/) {
   LINFO("Sesstion after playing\n");
}

static void getFrame(void *clientData, unsigned frameSize,
    unsigned, struct timeval pTime, unsigned)
{
  RTSPGrabber* rtspGrabber = (RTSPGrabber*)clientData;

  if (frameSize > rtspGrabber->getFrameBufferSize())
    LINFO("Frame buffer size to small, please increase (framesize %i)",
        frameSize);

  rtspGrabber->itsFrameSize = frameSize;
  rtspGrabber->blockingFlag = ~0;
}

static void onSourceClosure(void* clientData)
{
  RTSPGrabber* rtspGrabber = (RTSPGrabber*)clientData;
  LINFO("On source closure\n");
  rtspGrabber->blockingFlag = ~0;
}
#endif



// ######################################################################
void* RTSPGrabber_run(void *r0)
{
  RTSPGrabber *r = (RTSPGrabber *)r0;
  r->run(); return NULL;
}

// ######################################################################
RTSPGrabber::RTSPGrabber(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName,
                                 const ParamFlag flags,
                                 unsigned int bufferSize) :
  FrameIstream(mgr, descrName, tagName),
  itsDims(&OPT_FrameGrabberDims, this, Dims(320, 240), USE_MY_VAL),
  itsGrabMode(&OPT_FrameGrabberMode, this, VIDFMT_RGB24, USE_MY_VAL),
  itsByteSwap(&OPT_FrameGrabberByteSwap, this, false, USE_MY_VAL),
  itsWhiteBalTarR(&OPT_FrameGrabberWhiteBalTargetR, this, false, USE_MY_VAL),
  itsWhiteBalTarG(&OPT_FrameGrabberWhiteBalTargetG, this, false, USE_MY_VAL),
  itsWhiteBalTarB(&OPT_FrameGrabberWhiteBalTargetB, this, false, USE_MY_VAL),
  itsWhiteBalRefR(&OPT_FrameGrabberWhiteBalReferenceR, this, false, USE_MY_VAL),
  itsWhiteBalRefG(&OPT_FrameGrabberWhiteBalReferenceG, this, false, USE_MY_VAL),
  itsWhiteBalRefB(&OPT_FrameGrabberWhiteBalReferenceB, this, false, USE_MY_VAL),
  itsFPS(&OPT_FrameGrabberFPS, this, 15.0, USE_MY_VAL),
  itsFrameBufferSize(bufferSize),
  itsFrameBuffer(new unsigned char[bufferSize]),
  itsThreadRunning(false)
{
  pthread_mutex_init(&itsLock, NULL);

#ifdef INVT_HAVE_AVCODEC

  // no need to guard these functions for being called multiple times;
  // they all have internal guards
  av_register_all();

  //init ffmpeg
  avcodec_init();

  // register all the codecs
  avcodec_register_all();

  itsCodecContext = avcodec_alloc_context();

  AVCodec* codec = avcodec_find_decoder(CODEC_ID_MPEG4 );

  if (codec == 0) LFATAL("no codec found for codec_id=%d", CODEC_ID_MPEG4);

  if (!codec || avcodec_open(itsCodecContext, codec) < 0) LFATAL("avcodec_open() failed");

#if defined(INVT_FFMPEG_HAS_DEFAULTS_FUNCTIONS)
  avcodec_get_frame_defaults(&itsPicture);
#else
  {
    AVFrame* tmp = avcodec_alloc_frame();
    memcpy(&itsPicture, tmp, sizeof(AVFrame));
    av_free(tmp);
  }
#endif

  Image<PixRGB<byte> > frame(itsDims.getVal(), NO_INIT);
  itsCurrentFrame = VideoFrame(frame);
#endif
}

// ######################################################################
void RTSPGrabber::start1()
{
#ifndef INVT_HAVE_LIVEMEDIA
  LFATAL("you must have liveMedia support in order to use RTSPGrabber");
#else
  LINFO("URL %s", itsURL.c_str());
  TaskScheduler* scheduler = BasicTaskScheduler::createNew();
  itsEnv = BasicUsageEnvironment::createNew(*scheduler);
  itsClient = RTSPClient::createNew(*itsEnv,
      0, //verbosity level
      "test", //prog name
      0);

  if (itsClient == NULL)
    LFATAL("Failed to create client\n");

  itsRTSPClient = (RTSPClient*)itsClient;
  char* optionsResponse = itsRTSPClient->sendOptionsCmd(itsURL.c_str(), NULL, NULL);
  LINFO("%s", optionsResponse);

  char* sdpDescription = itsRTSPClient->describeURL(itsURL.c_str());
  if (sdpDescription == NULL) {
    Medium::close(itsClient);
    LFATAL("Failed to get a SDP description from URL %s", itsEnv->getResultMsg());
  }
  LINFO("SDP desc: %s", sdpDescription);

  itsSession = MediaSession::createNew(*itsEnv, sdpDescription);
  delete[] sdpDescription;

  if (itsSession == NULL)
  {
    Medium::close(itsClient);
    LFATAL("Failed to create a MediaSession object from the SDP description: %s ", itsEnv->getResultMsg());
  } else if (!itsSession->hasSubsessions()) {
    Medium::close(itsClient);
    LFATAL("This session has no media subsessions (i.e., \"m=\" lines)");
  }

  //setup sub sessions
  MediaSubsessionIterator iter(*itsSession);
  MediaSubsession *subsession;
  while ((subsession = iter.next()) != NULL) {

    //get only video
    //TODO add audio as well
    if (strcmp(subsession->mediumName(), "video") != 0) {
      LINFO("Ignoring %s/%s", subsession->mediumName(), subsession->codecName());
      continue;
    }


    if (!subsession->initiate(-1)) {
      LINFO("Unable to create receiver for %s/%s %s",
          subsession->mediumName(),
          subsession->codecName(),
          itsEnv->getResultMsg());
    } else {
      LINFO("Created receiver for %s/%s port %i",
          subsession->mediumName(),
          subsession->codecName(),
          subsession->clientPortNum());

      if (subsession->readSource() != NULL)
        itsFramedSource = subsession->readSource();

      if (subsession->rtpSource() != NULL) {
        // Because we're saving the incoming data, rather than playing
        // it in real time, allow an especially large time threshold
        // (1 second) for reordering misordered incoming packets:
        unsigned const thresh = 1000000; // 1 second
        subsession->rtpSource()->setPacketReorderingThresholdTime(thresh);

        unsigned int socketInputBufferSize = 20000;
        if (socketInputBufferSize > 0) {
          // Set the RTP source's input buffer size as specified:
          int socketNum
            = subsession->rtpSource()->RTPgs()->socketNum();
          unsigned curBufferSize
            = getReceiveBufferSize(*itsEnv, socketNum);
          unsigned newBufferSize
            = setReceiveBufferTo(*itsEnv, socketNum, socketInputBufferSize);
          LINFO("Changed socket receive buffer size for the %s/%s cur=%i bytes new=%i bytes",
              subsession->mediumName(),
              subsession->codecName(),
              curBufferSize,  newBufferSize);
        }
      }

      Boolean streamUsingTCP = False;
      if (!itsRTSPClient->setupMediaSubsession(*subsession, false, streamUsingTCP)) {
        LINFO("Failed to setup %s/%s  %s",
            subsession->mediumName(), subsession->codecName(),
            itsEnv->getResultMsg());
      } else {
        LINFO("Setup %s/%s port %i",
            subsession->mediumName(),  subsession->codecName(),
            subsession->clientPortNum());
      }
    }
  }

  if(!itsRTSPClient->playMediaSession(*itsSession))
    LINFO("Can not play %s", itsEnv->getResultMsg());


  // start the capture thread
  // start thread for run():
  pthread_create(&itsRunThread, NULL, &RTSPGrabber_run, (void *)this);

#endif
}

// ######################################################################
void RTSPGrabber::run()
{
#ifndef INVT_HAVE_LIVEMEDIA
  LFATAL("you must have liveMedia support in order to use RTSPGrabber");
#else

  itsThreadRunning = true;

  while(itsThreadRunning)
  {

    //Grab the frame
    blockingFlag = 0;
    itsFrameSize = 0;
    itsFramedSource->getNextFrame(itsFrameBuffer, itsFrameBufferSize,
        getFrame, this,
        onSourceClosure,this);

    //block untill data becomes available
    TaskScheduler& scheduler = itsFramedSource->envir().taskScheduler();
    scheduler.doEventLoop(&blockingFlag);

    //convert the frame
    if (itsFrameSize > 0)
    {
      pthread_mutex_lock(&itsLock);

#if LIBAVCODEC_VERSION_MAJOR >= 53 && LIBAVCODEC_VERSION_MINOR >= 21
      //for new avcodec
      AVPacket avpkt;
      av_init_packet(&avpkt);
      avpkt.data = (uint8_t *)itsFrameBuffer;
      avpkt.size = itsFrameSize;
      avpkt.flags = AV_PKT_FLAG_KEY;
      int len = avcodec_decode_video2(itsCodecContext,
		      &itsPicture, &itsGotPicture, &avpkt);
#else
      //for old avcodec
      int len = avcodec_decode_video(itsCodecContext,
          &itsPicture, &itsGotPicture,
          itsFrameBuffer, itsFrameSize);
#endif

      (void)len; //to avoid compiler warning;

      itsCurrentFrame =  convertAVFrameToVideoFrame(&itsPicture,
          itsCodecContext->pix_fmt,
          itsDims.getVal());
      itsGotPicture = 0;

      pthread_mutex_unlock(&itsLock);
    }
    usleep(10000);
  }


  pthread_exit(0);
#endif
}



// ######################################################################
void RTSPGrabber::setConfigInfo(const std::string& url)
{
//  if (url[0] == '/') //skip the // if nedded
//    itsURL.setVal(url.substr(2));
//  else
//    itsURL.setVal(url);

  itsURL = std::string("rtsp:") + url;

}

// ######################################################################
void RTSPGrabber::shutdown()
{
  itsThreadRunning = false;

  if (0 != pthread_join(itsRunThread, NULL)) LERROR("Pthread__join() failed");
  LINFO("Disconnecting from stream");

#ifdef INVT_HAVE_LIVEMEDIA
  //close
  MediaSubsessionIterator iter(*itsSession);
  MediaSubsession* subsession;
  while ((subsession = iter.next()) != NULL) {
    Medium::close(subsession->sink);
    subsession->sink = NULL;
  }

  itsRTSPClient->teardownMediaSession(*itsSession);
  Medium::close(itsSession);
  Medium::close(itsClient);
#endif
}

// ######################################################################
void RTSPGrabber::stop2()
{
#ifndef INVT_HAVE_LIVEMEDIA
  LFATAL("you must have liveMedia support in order to use RTSPGrabber");
#else
  shutdown();
#endif

}

// ######################################################################
RTSPGrabber::~RTSPGrabber()
{
  delete itsFrameBuffer;

#ifdef HAVE_FFMPEG_AVCODEC_H
  avcodec_close(itsCodecContext);
  av_free(itsCodecContext);
  av_free(&itsPicture);
#endif
}

// ######################################################################
GenericFrameSpec RTSPGrabber::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::VIDEO;
  result.videoFormat = itsGrabMode.getVal();
  result.videoByteSwap = itsByteSwap.getVal();
  result.dims = itsDims.getVal();
  result.floatFlags = 0;

  return result;
}

// ######################################################################
SimTime RTSPGrabber::getNaturalFrameTime() const
{
  return SimTime::HERTZ(itsFPS.getVal());
}



// ######################################################################
GenericFrame RTSPGrabber::readFrame()
{
  return GenericFrame(this->grabRaw());
}

// ######################################################################
VideoFrame RTSPGrabber::grabRaw()
{
#ifndef INVT_HAVE_LIVEMEDIA
  LFATAL("you must have liveMedia support in order to use RTSPGrabber");
  return VideoFrame();  /* can't happen */
#else
  pthread_mutex_lock(&itsLock);
  VideoFrame videoFrame = itsCurrentFrame;
  pthread_mutex_unlock(&itsLock);
  usleep(10000);
  return videoFrame;
#endif
}

// ######################################################################
void RTSPGrabber::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{
#ifndef INVT_HAVE_LIVEMEDIA
  LFATAL("you must have liveMedia support in order to use RTSPGrabber");
#else
  FrameIstream::paramChanged(param, valueChanged, status);
#endif
}

// ######################################################################
void RTSPGrabber::createDecoder()
{

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

