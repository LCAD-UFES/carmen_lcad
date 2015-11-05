/*!@file Neuro/NeoBrainVss.C for the vss demos */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/NeoBrain.C $
// $Id: NeoBrain.C 13716 2010-07-28 22:07:03Z itti $
//

#include "Neuro/NeoBrain.H"

#include "GUI/DebugWin.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H" // for REQUEST_OPTIONALIAS_NEURO()
#include "Image/MathOps.H" // for findMax()
#include "Neuro/NeuroOpts.H"
#include "rutz/error_context.h"
#include "rutz/mutex.h"
#include "rutz/sfmt.h"

#include <algorithm>

static const ModelOptionDef OPT_NeobrainSpeakSaliency =
  { MODOPT_FLAG, "NeobrainSpeakSaliency", &MOC_BRAIN, OPTEXP_CORE,
    "Whether to use speech to speak about saliency in NeoBrain",
    "neobrain-speak-saliency", '\0', "", "false" };

static const ModelOptionDef OPT_NeobrainSpeakObjects =
  { MODOPT_FLAG, "NeobrainSpeakObjects", &MOC_BRAIN, OPTEXP_CORE,
    "Whether to use speech to speak about objects in NeoBrain",
    "neobrain-speak-objects", '\0', "", "false" };

static const ModelOptionDef OPT_NeobrainSpeechFile =
  { MODOPT_ARG_STRING, "NeobrainSpeechFile", &MOC_BRAIN, OPTEXP_CORE,
    "Speech utterances for various speech tokens",
    "neobrain-speech-file", '\0', "<filename>", "etc/speech.pmap" };

static const ModelOptionDef OPT_NeobrainBoringnessThresh =
  { MODOPT_ARG(int), "NeobrainBoringnessThresh", &MOC_BRAIN, OPTEXP_CORE,
    "Threshold for boringness beyond which we start a new track",
    "neobrain-boringness-thresh", '\0', "<int>", "80" };

static const ModelOptionDef OPT_NeobrainTrackDelayFrames =
  { MODOPT_ARG(unsigned long), "NeobrainTrackDelayFrames", &MOC_BRAIN, OPTEXP_CORE,
    "Number of frames to wait after a shift before starting tracking",
    "neobrain-track-delay-frames", '\0', "<ulong>", "20" };

static const ModelOptionDef OPT_NeobrainStopTrackDelayFrames =
  { MODOPT_ARG(int), "NeobrainStopTrackDelayFrames", &MOC_BRAIN, OPTEXP_CORE,
    "Number of frames to wait after deciding to stop tracking before\n"
    "the tracking is actually disengaged. A value of -1 will never stop. ",
    "neobrain-stop-track-delay-frames", '\0', "<int>", "10" };

static const ModelOptionDef OPT_TrackTarget =
  { MODOPT_FLAG, "TrackTarget", &MOC_BRAIN, OPTEXP_CORE,
    "Whether to start up in target tracking mode.",
    "track-target", '\0', ""
#ifdef HAVE_OPENCV
    , "true"
#else
    , "false" // we can't do tracking without OpenCV
#endif
  };

static const ModelOptionDef OPT_NeobrainKeepTracking =
  { MODOPT_FLAG, "NeobrainKeepTracking", &MOC_BRAIN, OPTEXP_CORE,
    "If this option is true, the the brain will try to keep tracking the\n"
    "object for as long as posible",
    "neobrain-keeptracking", '\0', "", "false" };


// ######################################################################
NeoBrain::NeoBrain(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName)
  :
  ModelComponent(mgr, descrName, tagName),
  itsAllowTracking(&OPT_TrackTarget, this, ALLOW_ONLINE_CHANGES),
  itsKeepTracking(&OPT_NeobrainKeepTracking, this, ALLOW_ONLINE_CHANGES),
  itsUseHead("NeobrainUseHead", this, true, ALLOW_ONLINE_CHANGES),
  itsRelaxNeck("NeobrainRelaxNeck", this, true, ALLOW_ONLINE_CHANGES),
  itsSleeping("Sleeping", this, false, ALLOW_ONLINE_CHANGES),
  itsBoringnessThresh(&OPT_NeobrainBoringnessThresh, this, ALLOW_ONLINE_CHANGES),
  itsErrTolerance("NeobrainErrTolerance", this, 1, ALLOW_ONLINE_CHANGES),
  itsDistTolerance("NeobrainDistTolerance", this, 2, ALLOW_ONLINE_CHANGES),
  itsTrackDelayFrames(&OPT_NeobrainTrackDelayFrames, this,
                      ALLOW_ONLINE_CHANGES),
  itsBigErrFramesThresh("NeobrainBigErrFramesThresh", this, 500,
                        ALLOW_ONLINE_CHANGES),
  itsTargetFramesThresh("NeobrainTargetFramesThresh", this, 300,
                        ALLOW_ONLINE_CHANGES),
  itsNoMoveFramesThresh("NeobrainNoMoveFramesThresh", this, 1000,
                        ALLOW_ONLINE_CHANGES),
  itsStopTrackDelayFrames(&OPT_NeobrainStopTrackDelayFrames, this,
                          ALLOW_ONLINE_CHANGES),
  itsHeadInfoEyeTiltPos("HeadInfoEyeTiltPos", this, -1.00,
                          ALLOW_ONLINE_CHANGES),
  itsHeadInfoEyePanPos("HeadInfoEyePanPos", this, 0.20,
                          ALLOW_ONLINE_CHANGES),
  itsHeadInfoHeadPanPos("HeadInfoHeadPanPos", this, -1.00,
                          ALLOW_ONLINE_CHANGES),
  itsSpeakSaliency(&OPT_NeobrainSpeakSaliency, this, ALLOW_ONLINE_CHANGES),
  itsSpeakObjects(&OPT_NeobrainSpeakObjects, this, ALLOW_ONLINE_CHANGES),
  itsSpeechFile(&OPT_NeobrainSpeechFile, this),
  itsRefreshSpeechFile("NeobrainRefreshSpeechFile", this, false,
                       ALLOW_ONLINE_CHANGES),
  itsExcitementThresh("NeobrainExcitementThresh", this, 220.f,
                      ALLOW_ONLINE_CHANGES),
  itsTargetFrames(0),
  itsBigErrFrames(0),
  itsNoMoveFrames(0),
  itsStopFrames(0),
  itsHeadInfoFrames(0),
  itsPrevTargetX(-1.0f),
  itsPrevTargetY(-1.0f),
  itsBoringness(0.0f),
  itsBoringCount(0),
  itsExcitementLevel(0.0f),
  itsSleep(1000.0f),
  itsPrepSleep(0),
  itsAlmostSinging(false)
{
  itsBeoHead = nub::soft_ref<BeoHead>(new BeoHead(mgr));
  addSubComponent(itsBeoHead);

  // Instantiate our various ModelComponents:
  itsSpeechSynth = nub::soft_ref<SpeechSynth>(new SpeechSynth(mgr));
  addSubComponent(itsSpeechSynth);

  if (0 != pthread_mutex_init(&itsSpeechTokenMapMutex, NULL))
    LFATAL("pthread_mutex_init() failed");

#ifdef HAVE_OPENCV
  this->points[0] = NULL;
  this->points[1] = NULL;
  this->status = NULL;
  this->pyramid = NULL;
  this->prev_pyramid = NULL;
#else
  // we can't do tracking without OpenCV, so no point in allowing the
  // user to try to turn it on:
  itsAllowTracking.setInactive(true);
#endif
}

// ######################################################################
NeoBrain::~NeoBrain()
{
  if (0 != pthread_mutex_destroy(&itsSpeechTokenMapMutex))
    LERROR("pthread_mutex_destroy() failed");

#ifdef HAVE_OPENCV
  //cvFree(&this->points[0]);
  //cvFree(&this->points[1]);
  //cvFree(&this->status);
  cvReleaseImage(&this->pyramid);
  cvReleaseImage(&this->prev_pyramid);
#endif
}

// ######################################################################
void NeoBrain::start2()
{
  itsBeoHead->relaxNeck();

  //if (itsSpeakSaliency.getVal())
  if (!readSpeechFile(itsSpeechTokenMap, itsSpeechFile.getVal()))
    itsSpeakSaliency.setVal(false);

  if (itsSpeakSaliency.getVal())
    saveSpeechFile(itsSpeechTokenMap, "backup.pmap");
}

// ######################################################################
void NeoBrain::init(Dims imageDims, int nPoints, int wz )
{
  win_size = wz;

#ifdef HAVE_OPENCV
  MAX_COUNT = nPoints;
  count = 0;
  points[0] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
  points[1] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));

  prev_grey = Image<byte>(imageDims, ZEROS);
  pyramid = cvCreateImage( cvSize(imageDims.w(), imageDims.h()), 8, 1 );
  prev_pyramid = cvCreateImage( cvSize(imageDims.w(), imageDims.h()), 8, 1 );
  status = (char*)cvAlloc(MAX_COUNT);
#endif

  flags = 0;
  itsState = CHECK_TARGET;
  itsImageDims = imageDims;
  itsTracking = false;

//  if (itsSpeakSaliency.getVal())
  {
    itsSpeechSynth->sendCommand("(lex.add.entry '(\"bigpause\" n (((pau) 1) ((pau) 1) ((pau) 1) ((pau) 1))))\n", -10, true);

    itsSpeechSynth->sendCommand("(set! daisy (wave.load \"daisy.wav\"))",
        -10, true);
    itsSpeechSynth->sendCommand("(set! headinfo (wave.load \"headInfo.wav\"))",
        -10, true);
    itsSpeechSynth->sendCommand("(voice_cmu_us_rms_arctic_clunits)",
        -10, true);
  }
}


// ######################################################################
bool NeoBrain::readSpeechFile(TokenMap& tokenMap,
                              const std::string& fname)
{
  rutz::shared_ptr<ParamMap> pmap;

  try
    {
      pmap = ParamMap::loadPmapFile(fname);
    }
  catch (std::exception& e)
    {
      REPORT_CURRENT_EXCEPTION;
      return false;
    }

  ASSERT(pmap.is_valid());

  GVX_ERR_CONTEXT(rutz::sfmt("unpacking pmap file %s", fname.c_str()));

  tokenMap.clear();

  const int numTokenTypes = pmap->getIntParam("NUM_TOKEN_TYPES");
  for (int i = 0; i < numTokenTypes; ++i)
    {
      rutz::shared_ptr<ParamMap> submap =
        pmap->getSubpmap(sformat("TOKEN_TYPE_%d", i));

      GVX_ERR_CONTEXT(rutz::sfmt("unpacking TOKEN_TYPE_%d", i));

      const std::string name = submap->getStringParam("NAME");

      TokenType toktype;

      const int numTokens = submap->getIntParam("NUM_TOKENS");
      for (int k = 0; k < numTokens; ++k)
        {
          rutz::shared_ptr<ParamMap> subsubmap =
            submap->getSubpmap(sformat("TOKEN_%d", k));

          GVX_ERR_CONTEXT(rutz::sfmt("unpacking TOKEN_%d", k));

          SpeechToken tok;
          tok.low = subsubmap->getIntParam("LOW");
          tok.high = subsubmap->getIntParam("HIGH");

          const int numTextItems = subsubmap->getIntParam("NUM_TEXT_ITEMS");

          for (int j = 0; j < numTextItems; ++j)
            {
              tok.textList.push_back(subsubmap->getStringParam
                                     (sformat("TEXT_ITEM_%d", j)));
            }

          toktype.tokens.push_back(tok);
        }

      tokenMap[name] = toktype;
    }

  return true;
}

// ######################################################################
void NeoBrain::saveSpeechFile(const TokenMap& tokenMap,
                              const std::string& fname)
{
  rutz::shared_ptr<ParamMap> pmap(new ParamMap);

  int numTokenTypes = 0;
  for (TokenMap::const_iterator
         itr = tokenMap.begin(), stop = tokenMap.end();
       itr != stop; ++itr)
    {
      rutz::shared_ptr<ParamMap> submap(new ParamMap);

      submap->putStringParam("NAME", (*itr).first);

      int numTokens = 0;
      for (size_t j = 0; j < (*itr).second.tokens.size(); ++j)
        {
          rutz::shared_ptr<ParamMap> subsubmap(new ParamMap);

          subsubmap->putIntParam("LOW", (*itr).second.tokens[j].low);
          subsubmap->putIntParam("HIGH", (*itr).second.tokens[j].high);
          int numTextItems = 0;
          for (size_t i = 0; i < (*itr).second.tokens[j].textList.size(); ++i)
            {
              subsubmap->putStringParam(sformat("TEXT_ITEM_%d", numTextItems),
                                        (*itr).second.tokens[j].textList[i]);
              ++numTextItems;
            }
          subsubmap->putIntParam("NUM_TEXT_ITEMS", numTextItems);

          submap->putSubpmap(sformat("TOKEN_%d", numTokens), subsubmap);

          ++numTokens;
        }

      submap->putIntParam("NUM_TOKENS", numTokens);

      pmap->putSubpmap(sformat("TOKEN_TYPE_%d", numTokenTypes), submap);

      ++numTokenTypes;
    }
  pmap->putIntParam("NUM_TOKEN_TYPES", numTokenTypes);

  pmap->format(fname);
}

// ######################################################################
std::string NeoBrain::getToken(const std::string& token, int val) const
{

  std::string result;

  {
    GVX_MUTEX_LOCK(&itsSpeechTokenMapMutex);

    TokenMap::const_iterator itr = itsSpeechTokenMap.find(token);
    if (itr == itsSpeechTokenMap.end())
    {
      LERROR("no such speech token: %s", token.c_str());
      TokenMap::const_iterator itr2;
      for(itr2=itsSpeechTokenMap.begin(); itr2 != itsSpeechTokenMap.end(); ++itr2)
      {
        std::string tmp = (*itr2).first; //.getTextItemForVal(2);
        LINFO("%s", tmp.c_str());
      }
    }
    else
      result = (*itr).second.getTextItemForVal(val);
  }

  return result;
}

// ######################################################################
bool NeoBrain::sayToken(const std::string& token,
                        int val, int priority) const
{
  const std::string text = this->getToken(token, val);

  if (!itsSpeakSaliency.getVal())
    return false;

  if (text.length() == 0)
    return false;

  return this->sayText(text, priority, false);
}

// ######################################################################
bool NeoBrain::sayObjectLabel(const std::string& label,
                              int confidence, bool forceLabel)
{
  if ( (label == "nomatch"
        && label == "none"
        && label == ""
        && label != itsLastSpokenLabel) ||
        forceLabel)
    {
      if (itsSpeakObjects.getVal())
      {

        const std::string intro = this->getToken("object_intro", confidence);
        if (!intro.empty()
            &&
            this->sayText(sformat("%s %s", intro.c_str(), label.c_str()),
              /* priority = */ -5,
              /* block = */ false))
        {
    //      setKeepTracking(false);
          itsLastSpokenLabel = label;
          return true;
        }

//      if (itsSpeakObjects.getVal()
//          &&
//          itsSpeechSynth->playWavFile(label + ".wav",
//                                      /* priority = */ -5,
//                                      /* block = */ false))
//        {
//          setKeepTracking(false);
//          itsLastSpokenLabel = label;
//          return true;
//        }
      }
    }

  itsLastSpokenLabel = "";

  return false;
}

// ######################################################################
void NeoBrain::setTarget(const Point2D<int> loc, const Image<byte>& grey,
                         const int saliencyval, bool changeState, bool forceNewLocation)
{
  if (forceNewLocation)
    itsTracking = false;

  //Dont set the target if we are tracking
  if (!itsAllowTracking.getVal() || itsTracking)
    return;

#ifdef HAVE_OPENCV

  count = MAX_COUNT;

  IplImage* tmp = img2ipl(grey);
  if (count > 1)
  {
    IplImage* eig = cvCreateImage(cvGetSize(tmp), 32, 1);
    IplImage* temp = cvCreateImage(cvGetSize(tmp), 32, 1);
    double quality = 0.01;
    double min_distance = 5;

    cvGoodFeaturesToTrack(tmp, eig, temp, points[1], &count,
        quality, min_distance, 0, 3, 0, 0.04);
    cvReleaseImage(&eig);
    cvReleaseImage(&temp);

  } else {
    //get from the saliency map
    points[1][0].x = loc.i;
    points[1][0].y = loc.j;

  }
  cvFindCornerSubPix(tmp, points[1], count,
      cvSize(win_size,win_size), cvSize(-1,-1),
      cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,
        20,0.03));
  cvReleaseImageHeader(&tmp);

  IplImage *swap_temp;
  prev_grey = grey;
  CV_SWAP( prev_pyramid, pyramid, swap_temp );
  CV_SWAP( points[0], points[1], swap_points );


  //LINFO("Init %i point (%f,%f)\n", count, points[1][0].x, points[1][0].y);

  if (changeState) {
    itsState = CHECK_TARGET;
  }
  itsTracking = true;
#endif

  if (saliencyval >= 0)
    this->saySaliencyVal(byte(saliencyval));
}

// ######################################################################
Point2D<int> NeoBrain::trackObject(const Image<byte>& grey)
{
  itsRefreshSpeechFile.setVal(false);

  Point2D<int> targetLoc(-1,-1);

#ifdef HAVE_OPENCV
  if (itsAllowTracking.getVal() && itsTracking)
  {
    if (count > 0)
    {
      IplImage* tmp1 = img2ipl(prev_grey);
      IplImage* tmp2 = img2ipl(grey);

      cvCalcOpticalFlowPyrLK(tmp1, tmp2, prev_pyramid, pyramid,
                             points[0], points[1], count,
                             cvSize(win_size,win_size), 3, status, 0,
                             cvTermCriteria(CV_TERMCRIT_ITER
                                            |CV_TERMCRIT_EPS,
                                            20,0.03), flags);

      cvReleaseImageHeader(&tmp1);
      cvReleaseImageHeader(&tmp2);

      flags |= CV_LKFLOW_PYR_A_READY;

      //show track points
      int k, i;
      for(i = k = 0; i<count; i++)
      {
        if (!status[i])
          continue;
        points[1][k++] = points[1][i];

        targetLoc.i = std::min(grey.getWidth()-1, std::max(0, (int)points[1][i].x));
        targetLoc.j = std::min(grey.getHeight()-1, std::max(0, (int)points[1][i].y));
        ASSERT(grey.coordsOk(targetLoc));
      }
      count = k;

    }

    IplImage *swap_temp;
    CV_SWAP( prev_pyramid, pyramid, swap_temp );
    CV_SWAP( points[0], points[1], swap_points );

    moveHeadToTarget();
  }
  prev_grey = grey;
#endif

  return targetLoc;
}

// ######################################################################
std::vector<Point2D<int> > NeoBrain::getTrackersLoc(const Image<byte>& grey)
{
  std::vector<Point2D<int> > trackersLoc;

#ifdef HAVE_OPENCV
  if (itsAllowTracking.getVal() && itsTracking)
  {
    if (count > 0)
    {
      IplImage* tmp1 = img2ipl(prev_grey);
      IplImage* tmp2 = img2ipl(grey);

      cvCalcOpticalFlowPyrLK(tmp1, tmp2, prev_pyramid, pyramid,
                             points[0], points[1], count,
                             cvSize(win_size,win_size), 3, status, 0,
                             cvTermCriteria(CV_TERMCRIT_ITER
                                            |CV_TERMCRIT_EPS,
                                            20,0.03), flags);

      cvReleaseImageHeader(&tmp1);
      cvReleaseImageHeader(&tmp2);

      flags |= CV_LKFLOW_PYR_A_READY;

      //show track points
      int k, i;
      for(i = k = 0; i<count; i++)
      {
        if (!status[i])
          continue;
        points[1][k++] = points[1][i];

        Point2D<int> tracker(std::min(grey.getWidth()-1, std::max(0, (int)points[1][i].x)),
            std::min(grey.getHeight()-1, std::max(0, (int)points[1][i].y)));
        trackersLoc.push_back(tracker);
      }
      count = k;

    }

    IplImage *swap_temp;
    CV_SWAP( prev_pyramid, pyramid, swap_temp );
    CV_SWAP( points[0], points[1], swap_points );
  }
  prev_grey = grey;
#endif

  return trackersLoc;
}


// ######################################################################
void NeoBrain::moveHeadToTarget()
{
#ifdef HAVE_OPENCV
  if (count > 0)
    {

      float targetX = points[1][0].x/itsImageDims.w();
      float targetY = points[1][0].y/itsImageDims.h();

      itsStats.bigerrframes        = itsBigErrFrames;
      itsStats.bigerrframes_thresh = itsBigErrFramesThresh.getVal();
      itsStats.targetframes        = itsTargetFrames;
      itsStats.targetframes_thresh = itsTargetFramesThresh.getVal();
      itsStats.nomoveframes        = itsNoMoveFrames;
      itsStats.nomoveframes_thresh = itsNoMoveFramesThresh.getVal();
      itsStats.stopframes          = itsStopFrames;
      itsStats.stopframes_thresh   = itsStopTrackDelayFrames.getVal();


      itsStats.leftEyePanPos   =  itsBeoHead->getLeftEyePanPos();
      itsStats.leftEyeTiltPos  =  itsBeoHead->getLeftEyeTiltPos();
      itsStats.rightEyePanPos  =  itsBeoHead->getRightEyePanPos();
      itsStats.rightEyeTiltPos =  itsBeoHead->getRightEyeTiltPos();
      itsStats.headPanPos      =  itsBeoHead->getHeadPanPos();
      itsStats.headTiltPos     =  itsBeoHead->getHeadTiltPos();
      itsStats.headYawPos      =  itsBeoHead->getHeadYawPos();


      switch(itsState)
        {
        case CHECK_TARGET:
          itsTargetFrames++;
          if (targetX > 1.0 || targetX < 0 ||
              targetY > 1.0 || targetY < 0)
            {
              enterCheckTargetState();
              itsTracking = false;
            }
          else if (itsTargetFrames > itsTrackDelayFrames.getVal())
            {
              itsPrevTargetX = targetX;
              itsPrevTargetY = targetY;
              itsState = TRACK_TARGET;
            }
          break;

        case TRACK_TARGET:
          itsTargetFrames++;

          if (!itsKeepTracking.getVal() &&
              itsStopFrames > 0)
            ++itsStopFrames;

          LDEBUG("itsBigErrFrames=%lu (thresh %lu), "
                 "itsTargetFrames=%lu (thresh %lu), "
                 "itsNoMoveFrames=%lu (thresh %lu), "
                 "itsStopFrames=%lu (thresh %i), "
                 "itsBoringness=%.2f (thresh %d)",
                 itsBigErrFrames, itsBigErrFramesThresh.getVal(),
                 itsTargetFrames, itsTargetFramesThresh.getVal(),
                 itsNoMoveFrames, itsNoMoveFramesThresh.getVal(),
                 itsStopFrames, itsStopTrackDelayFrames.getVal(),
                 itsBoringness, itsBoringnessThresh.getVal());

          if (itsStopTrackDelayFrames.getVal() != -1 &&
              (int)itsStopFrames >= itsStopTrackDelayFrames.getVal())
            {
              enterCheckTargetState();
              itsTracking = false;
            }
          else if (itsStopFrames == 0
              && itsBigErrFrames >= itsBigErrFramesThresh.getVal())
            {
              this->sayToken("notrack_target", 0, 10);
              ++itsStopFrames;
            }
          else if (!itsKeepTracking.getVal() &&
                   itsStopFrames == 0
                   && itsTargetFrames >= itsTargetFramesThresh.getVal())
            {
              this->sayToken("tiresome_target", 0, 10);
              ++itsStopFrames;
            }
          else if (!itsKeepTracking.getVal() &&
                   itsStopFrames == 0
                   && itsNoMoveFrames >= itsNoMoveFramesThresh.getVal())
            {
              this->sayToken("nomove_target", 0, 10);
              ++itsStopFrames;
            }
          else if (!itsKeepTracking.getVal() &&
                   itsStopFrames == 0
                   && itsBoringness >= itsBoringnessThresh.getVal())
            {
              this->sayToken("boring_target", 0, 10);
              ++itsStopFrames;
            }
          else
            {
              const float xerr =
                itsUseHead.getVal()
                ? fabs(0.5 - targetX)
                : fabs(itsPrevTargetX - targetX);

              const float yerr =
                itsUseHead.getVal()
                ? fabs(0.5 - targetY)
                : fabs(itsPrevTargetY - targetY);

              const float err =
                itsUseHead.getVal()
                ? itsBeoHead->trackTarget(0.5, 0.5, targetX, targetY)
                : (xerr + yerr);

              const float errtol = 0.01f * itsErrTolerance.getVal();
              const float disttol = 0.01f * itsDistTolerance.getVal();

              if (err > errtol)
                itsBigErrFrames++;

              if (err < errtol && xerr < disttol && yerr < disttol)
                itsNoMoveFrames++;
              else
                itsNoMoveFrames = 0;

              LDEBUG("usehead = %d, err = %f (%f+%f), target = %f,%f",
                     int(itsUseHead.getVal()),
                     err, xerr, yerr, targetX, targetY);

              itsStats.last_err = err;
              itsStats.last_xerr = xerr;
              itsStats.last_yerr = yerr;
              itsStats.err_tol = errtol;
              itsStats.dist_tol = disttol;

              itsPrevTargetX = targetX;
              itsPrevTargetY = targetY;

              itsState = TRACK_TARGET;
            }

            //get head position, if is matches a predefined position then utter a response
            if ( (itsBeoHead->getLeftEyeTiltPos() >= itsHeadInfoEyeTiltPos.getVal() - 0.01) &&
                 (itsBeoHead->getLeftEyeTiltPos() <= itsHeadInfoEyeTiltPos.getVal() + 0.01) &&

                 (itsBeoHead->getLeftEyePanPos() >= itsHeadInfoEyePanPos.getVal() - 0.1) &&
                 (itsBeoHead->getLeftEyePanPos() <= itsHeadInfoEyePanPos.getVal() + 0.1) &&

                 (itsBeoHead->getHeadPanPos() >= itsHeadInfoHeadPanPos.getVal() - 0.01) &&
                 (itsBeoHead->getHeadPanPos() <= itsHeadInfoHeadPanPos.getVal() + 0.01)  &&
                 itsHeadInfoFrames == 0)
            {
              if (itsSpeakSaliency.getVal())
                itsSpeechSynth->sendCommand("(wave.play headinfo)", -10, false);
              itsHeadInfoFrames++;
            }

            if (itsHeadInfoFrames != 0) //dont increment if we are at 0
            {
              if (itsHeadInfoFrames++ > 50)
                itsHeadInfoFrames = 0;
            }

          break;

        default:
          break;
        }

    }
  else
    {
      enterCheckTargetState();
      itsTracking = false;
    }
#endif
}

// ######################################################################
void NeoBrain::saySaliencyVal(byte val)
{
  sayToken("new_target", val, 2);
}

// ######################################################################
void NeoBrain::updateBoringness(const Image<byte>& salmap, byte foaval)
{
  Point2D<int> truemaxpos;
  byte truemaxval;
  findMax(salmap, truemaxpos, truemaxval);

  const int val = int(foaval) - int(truemaxval);
  itsBoringness = (0.9f * itsBoringness) + (0.1f * -val);
}

// ######################################################################
void NeoBrain::updateExcitement(double vcxflicker)
{

  itsExcitementLevel =
    0.99f * itsExcitementLevel
    + 0.007f * (255.0f - itsBoringness)
    + 0.003f * (255.0f * vcxflicker);

  if (itsSleeping.getVal())
          itsSleep = 0.95f * itsSleep;
  else
          itsSleep = 0.995f * itsSleep; //if we are not sleeping, then wait longer

  if (itsSleep > 1000) itsSleep = 1000;
  if (itsSleep < 1) itsSleep = 1;
  if ( (255.0f * vcxflicker > 200
        && (itsState == CHECK_TARGET && !itsPrepSleep))
       || itsSleeping.getVal()) //if we are not moving
  {
          itsSleep += 0.2*(255.0f * vcxflicker);
  }

  //After going to sleep wait a few frames before adding back the motion
  //This is to avoid motion generated from pausing the cameras
  if (itsPrepSleep)
  {
          itsPrepSleep++;
          if (itsPrepSleep > 100)
          {
              itsPrepSleep = 0;
              itsSleep = 0;
          }
  }


  //go to sleep
  if (itsSleep <= 1 && !itsSleeping.getVal())
  {
      //itsPrepSleep = 1;
      //itsSleeping.setVal(true);
      //setUseHead(false);
      //sleep(2);
      //itsBeoHead->moveRestPos();
      //itsSleep = 0;
      //setRelaxNeck(true);
      //itsBeoHead->relaxHead();
      //this->sayText("Good night to all, and to all a good night.", 0, true);
      //itsSpeakSaliency.setVal(false);
      //sleep(2);
      //itsSleep = 0;
  }

  //wake up if we are sleeping and not prepreing to sleep
  if (itsSleep > 200 && itsSleeping.getVal() && !itsPrepSleep)
  {
          itsPrepSleep = 0;
          itsSpeakSaliency.setVal(true);
          this->sayText("Good Morning to you.", 0, false);
          setRelaxNeck(false);
          sleep(2);
          setUseHead(true);
          itsSleep = 1000;
          itsSleeping.setVal(false);

  }



  if (itsExcitementLevel > itsExcitementThresh.getVal())
    {
      if (itsSpeakSaliency.getVal())
        itsSpeechSynth->sendCommand("(wave.play daisy)", -10, false);
      itsExcitementLevel = 0;
      itsAlmostSinging = false;
    }
  else if (itsExcitementLevel + 10.f > itsExcitementThresh.getVal())
    {
      if (!itsAlmostSinging)
        this->sayText("You have excited me. Any more excitment "
                      "and I will start to sing", 0, false);

      itsAlmostSinging = true;
    }
  else if (itsAlmostSinging)
    {
      this->sayText("Now you have stopped exciting me.", 0, false);
      itsAlmostSinging = false;
    }
}

void NeoBrain::gotoSleep()
{
        itsSleeping.setVal(true);
        setUseHead(false);
        sleep(2);
        itsBeoHead->moveRestPos();
        setRelaxNeck(true);
        itsBeoHead->relaxHead();
        this->sayText("Good night to all, and to all a good night.", 0, true);
        itsSpeakSaliency.setVal(false);
        sleep(2);
        itsSleep = 0;
}

void NeoBrain::wakeUp()
{
        itsSpeakSaliency.setVal(true);
        this->sayText("Good Morning to you.", 0, false);
        setRelaxNeck(false);
        sleep(2);
        setUseHead(true);
        itsSleeping.setVal(false);
}


// ######################################################################
float NeoBrain::getBoringness() const
{
  return itsBoringness;
}

// ######################################################################
float NeoBrain::getExcitementLevel() const
{
  return itsExcitementLevel;
}
float NeoBrain::getSleepLevel() const
{
        return itsSleep;
}

// ######################################################################
bool NeoBrain::sayText(const std::string& text, int priority,
                       bool block) const
{
 // if (itsSpeakSaliency.getVal())
  return itsSpeechSynth->sayText(text.c_str(), priority, block);

  // else...
  return false;
}

// ######################################################################
void NeoBrain::paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  if (param == &itsRelaxNeck)
    {
      if (itsRelaxNeck.getVal())
        itsBeoHead->relaxNeck();
      else
        itsBeoHead->moveRestPos();
    }
  else if (param == &itsRefreshSpeechFile
           && valueChanged == true
           && itsRefreshSpeechFile.getVal() == true)
    {
      try
        {
          TokenMap newmap;
          readSpeechFile(newmap, itsSpeechFile.getVal());

          {
            GVX_MUTEX_LOCK(&itsSpeechTokenMapMutex);
            itsSpeechTokenMap.swap(newmap);
          }

          LINFO("reloaded utterances from %s",
                itsSpeechFile.getVal().c_str());
        }
      catch (...)
        {
          REPORT_CURRENT_EXCEPTION;
          *status = ParamClient::CHANGE_REJECTED;
        }
    }
}

// ######################################################################
void NeoBrain::enterCheckTargetState()
{
  itsStats.bigerrframes = 0;
  itsStats.targetframes = 0;
  itsStats.nomoveframes = 0;
  itsStats.stopframes = 0;
  itsStats.last_err = 0.0f;
  itsStats.last_xerr = 0.0f;
  itsStats.last_yerr = 0.0f;

  itsTargetFrames = 0;
  itsBigErrFrames = 0;
  itsNoMoveFrames = 0;
  itsStopFrames = 0;
  itsPrevTargetX = -1.0f;
  itsPrevTargetY = -1.0f;
  itsBoringness = 0.0f;
  itsLastSpokenLabel = "";
  itsState = CHECK_TARGET;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
