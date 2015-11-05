/*!@file Neuro/NeoBrainVss.H for the vss demo*/

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/NeoBrain.H $
// $Id: NeoBrain.H 13716 2010-07-28 22:07:03Z itti $
//

#ifndef NEURO_NEOBRAIN_H_DEFINED
#define NEURO_NEOBRAIN_H_DEFINED

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/BeoHead.H"
#include "Devices/SpeechSynth.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"
#include "Image/CutPaste.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/OpenCVUtil.H"
#include "Component/ParamMap.H"
#include "Util/sformat.H"
#include "rutz/mutex.h"
#include "Util/WorkThreadServer.H"
#include "Util/JobWithSemaphore.H"

#include <pthread.h>
#include <string>
#include <vector>
#include <map>

//Define VSS_DEMO so that neovision2.C would include some spacific functions
#define VSS_DEMO 1

class NeoBrain : public ModelComponent
{
public:
  enum State {CHECK_TARGET, TRACK_TARGET};

  //! Constructor
  NeoBrain(OptionManager& mgr,
           const std::string& descrName = "NeoBrain",
           const std::string& tagName = "NeoBrain");

  //! virtual destructor for safe inheritance
  virtual ~NeoBrain();

  void init(Dims imageDims, int nPoints = 1, int wz = 10);

  //tracking
  void setTarget(Point2D<int> loc, const Image<byte>& grey,
      int saliencyval=-1, bool changeState = false, bool forceNewLocation = false);
  void setTarget(Point2D<int> loc, const Image<PixRGB<byte> >& rgb,
      int saliencyval=-1, bool changeState = false, bool forceNewLocation = false)
  {
    if (itsAllowTracking.getVal())
      this->setTarget(loc, luminance(rgb), saliencyval, changeState, forceNewLocation);
  }
  Point2D<int> trackObject(const Image<byte>& grey);
  std::vector<Point2D<int> > getTrackersLoc(const Image<byte>& grey);
  Point2D<int> trackObject(const Image<PixRGB<byte> >& rgb)
  {
    if (itsAllowTracking.getVal())
      return this->trackObject(luminance(rgb));
    else return Point2D<int>(-1,-1);
  }



  void saySaliencyVal(byte val);

  void moveHeadToTarget();

  void updateBoringness(const Image<byte>& salmap, byte foaval);

  void updateExcitement(double vcxflicker);

  float getBoringness() const;

  float getExcitementLevel() const;
  float getSleepLevel() const;

  bool sayText(const std::string& text, int priority, bool block = false) const;

  std::string getToken(const std::string& token, int val) const;

  bool sayToken(const std::string& token, int val, int priority) const;

  bool sayObjectLabel(const std::string& label, int confidence, bool foceLabel=false);

  struct Stats
  {
    unsigned long bigerrframes, bigerrframes_thresh;
    unsigned long targetframes, targetframes_thresh;
    unsigned long nomoveframes, nomoveframes_thresh;
    unsigned long stopframes, stopframes_thresh;
    float last_err, last_xerr, last_yerr, err_tol, dist_tol;
    float leftEyePanPos, leftEyeTiltPos;
    float rightEyePanPos, rightEyeTiltPos;
    float headPanPos, headTiltPos, headYawPos;
  };

  Stats getStats() const { return itsStats; }

  Image<PixRGB<byte> > getSaliencyHisto(const Dims& dims,
                                        const PixRGB<byte>& back,
                                        const PixRGB<byte>& fore)
  {
    if (!itsSpeakSaliency.getVal())
      {
        Image<PixRGB<byte> > result(dims, NO_INIT);
        result.clear(back);
        return result;
      }

    TokenType ttype;
    {
      GVX_MUTEX_LOCK(&itsSpeechTokenMapMutex);
      TokenMap::const_iterator itr = itsSpeechTokenMap.find("new_target");
      if (itr == itsSpeechTokenMap.end())
        LERROR("couldn't find new_target token type");
      else
        ttype = (*itr).second;
    }
    return ttype.makeTokenHistogram(dims, back, fore);
  }

  //for setting model params

 bool getUseHead() {return itsUseHead.getVal(); }
 bool getRelaxNeck() {return itsRelaxNeck.getVal(); }
 int  getBoringnessThresh() {return itsBoringnessThresh.getVal(); }
 int  getErrTolerance() {return itsErrTolerance.getVal(); }
 int  getDistTolerance() {return itsDistTolerance.getVal(); }

 long getTrackDelayFrames()     { return itsTrackDelayFrames.getVal(); }
 long getBigErrFramesThresh()   { return itsBigErrFramesThresh.getVal(); }
 long getTargetFramesThresh()   { return itsTargetFramesThresh.getVal(); }
 long getNoMoveFramesThresh()   { return itsNoMoveFramesThresh.getVal(); }
 long getStopTrackDelayFrames() { return itsStopTrackDelayFrames.getVal(); }


 void gotoSleep();
 void wakeUp();
 bool isSleeping() { return itsSleeping.getVal();}

 void setUseHead(bool val) { itsUseHead.setVal(val); }
 void setRelaxNeck(bool val) { itsRelaxNeck.setVal(val); }
 void setBoringnessThresh(int val) { itsBoringnessThresh.setVal(val); }
 void setErrTolerance(int val) { itsErrTolerance.setVal(val); }
 void setDistTolerance(int val) { itsDistTolerance.setVal(val); }

 void setTrackDelayFrames(long val)     {  itsTrackDelayFrames.setVal(val); }
 void setBigErrFramesThresh(long val)   {  itsBigErrFramesThresh.setVal(val); }
 void setTargetFramesThresh(long val)   {  itsTargetFramesThresh.setVal(val); }
 void setNoMoveFramesThresh(long val)   {  itsNoMoveFramesThresh.setVal(val); }
 void setStopTrackDelayFrames(long val) {  itsStopTrackDelayFrames.setVal(val); }
 void setKeepTracking(bool val) { itsKeepTracking.setVal(val); }

 void relaxHead(){itsBeoHead->relaxHead();}//stop all servo
protected:
  virtual void start2();

  void paramChanged(ModelParamBase* const param,
                    const bool valueChanged,
                    ParamClient::ChangeStatus* status);

private:
  void enterCheckTargetState();

  OModelParam<bool> itsAllowTracking;
  OModelParam<bool> itsKeepTracking; //Keep the tracking the target at much as posible
  NModelParam<bool> itsUseHead;
  NModelParam<bool> itsRelaxNeck;
  NModelParam<bool> itsSleeping;
  OModelParam<int> itsBoringnessThresh;
  NModelParam<unsigned int> itsErrTolerance;
  NModelParam<unsigned int> itsDistTolerance;

  OModelParam<unsigned long> itsTrackDelayFrames;
  NModelParam<unsigned long> itsBigErrFramesThresh;
  NModelParam<unsigned long> itsTargetFramesThresh;
  NModelParam<unsigned long> itsNoMoveFramesThresh;
  OModelParam<int> itsStopTrackDelayFrames;

  NModelParam<float> itsHeadInfoEyeTiltPos;
  NModelParam<float> itsHeadInfoEyePanPos;
  NModelParam<float> itsHeadInfoHeadPanPos;

  OModelParam<bool> itsSpeakSaliency;
  OModelParam<bool> itsSpeakObjects;
  OModelParam<std::string> itsSpeechFile;

  NModelParam<bool> itsRefreshSpeechFile;

  NModelParam<float> itsExcitementThresh;

  nub::soft_ref<BeoHead> itsBeoHead;
  nub::soft_ref<SpeechSynth> itsSpeechSynth;

  unsigned long itsTargetFrames;
  unsigned long itsBigErrFrames;
  unsigned long itsNoMoveFrames;
  unsigned long itsStopFrames;
  unsigned long itsHeadInfoFrames;

  int win_size;

#ifdef HAVE_OPENCV
  int MAX_COUNT;
  int count;
  CvPoint2D32f* points[2], *swap_points;
  IplImage *pyramid;
  IplImage *prev_pyramid;
  Image<byte> prev_grey;
  char* status;
#endif
  int flags;
  State itsState;
  Dims itsImageDims;
  bool itsTracking;

  float itsPrevTargetX;
  float itsPrevTargetY;

  float itsBoringness;
  int itsBoringCount;
  float itsExcitementLevel;
  float itsSleep;
  int itsPrepSleep;

  bool itsAlmostSinging;

  std::string itsLastSpokenLabel;

  struct SpeechToken
  {
    SpeechToken() : low(0), high(100), nspoken(0) {}

    std::vector<std::string> textList;
    int low;
    int high;
    mutable int nspoken;
  };

  struct TokenType
  {
    TokenType()
      :
      lastReturnedToken(0)
    {
      for (int i = 0; i < 256; ++i) this->histo[i] = 0;
    }

    std::string getTextItemForVal(int val) const
    {
      if (val < 0)
        LERROR("val too low (%d)", val);
      else if (val > 255)
        LERROR("val too high (%d)", val);

      this->histo[std::min(255, std::max(0,val))]++;

      int percentile=0, total=0;

      for (int i = 0; i < 256; ++i)
        {
          if (i <= val)
            percentile += this->histo[i];
          total += this->histo[i];
        }

      const int fpercentile = int(100.0 * double(percentile) / double(total));

      for (size_t i = 0; i < this->tokens.size(); ++i)
        {
          if (fpercentile >= this->tokens.at(i).low
              && fpercentile <= this->tokens.at(i).high)
            {
              int n = randomUpToNotIncluding
                (this->tokens.at(i).textList.size());

              if (n == lastReturnedToken)
                n = (n+1) % this->tokens.at(i).textList.size();

              if (n < 0)
                { LERROR("n too low (%d)", n); n = 0; }
              else if (n >= int(this->tokens.at(i).textList.size()))
                { LERROR("n too high (%d)", n); n = int(this->tokens.at(i).textList.size()); }

              lastReturnedToken = n;

              LDEBUG("fpercentile = %d; returning %s",
                     fpercentile, this->tokens.at(i).textList.at(n).c_str());

              this->tokens.at(i).nspoken++;

              return this->tokens.at(i).textList.at(n);
            }
        }

      return std::string();
    }

    Image<PixRGB<byte> > makeTokenHistogram(const Dims& dims,
                                            const PixRGB<byte>& back,
                                            const PixRGB<byte>& fore)
    {
      Image<PixRGB<byte> > result(dims, NO_INIT);
      result.clear(back);

      int smallhisto[16] = { 0 };
      for (int i = 0; i < 256; ++i)
        smallhisto[(i * 16) / 256] += this->histo[i];

      int maxval = 1;
      for (size_t i = 0; i < 16; ++i)
          if (smallhisto[i] > maxval) maxval = smallhisto[i];

      const int top = std::max(1, dims.h() - 10);
      for (size_t i = 0; i < 16; ++i)
        {
          const int left = (i * dims.w()) / 16;
          const int right =
            std::min(dims.w()-1, int(((i + 1) * dims.w()) / 16) - 1);

          for (int x = left; x <= right; ++x)
            {
              drawLine(result,
                       Point2D<int>(x, dims.h() - 1),
                       Point2D<int>(x, dims.h() - (smallhisto[i] * top) / maxval),
                       fore, 1);
            }
        }

      return result;
    }

    std::vector<SpeechToken> tokens;
    mutable int histo[256];
    mutable int lastReturnedToken;
  };

  typedef std::map<const std::string, TokenType> TokenMap;
  TokenMap itsSpeechTokenMap;
  mutable pthread_mutex_t itsSpeechTokenMapMutex;

  Stats itsStats;

  /// returns false if there is no such file; parsing errors will generate exceptions
  static bool readSpeechFile(TokenMap& tokenMap,
                             const std::string& fname);
  static void saveSpeechFile(const TokenMap& tokenMap,
                             const std::string& fname);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_NEOBRAIN_H_DEFINED
