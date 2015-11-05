/*!@file Devices/BeoMonkey.H Interface to robot monkey head, derived from
BeoChip which is an to interface Brian Hudson's BeoChip .*/

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeoMonkey.H $
// $Id: BeoMonkey.H 6795 2006-06-29 20:45:32Z rjpeters $
//

#ifndef BEOMONKEY_H_DEFINED
#define BEOMONKEY_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/BeoChip.H"
#include "Devices/BeoMonkeyTypes.H"
#include "Util/Types.H"
#include <deque>
#include <pthread.h>
#include <stdarg.h>

class BeoMonkey : public BeoChip
{
public:
  //Constructor
  BeoMonkey(OptionManager& mgr,
          const std::string& descrName = "BeoMonkey",
          const std::string& tagName = "BeoMonkey");

  // These are monkey specific functions to call servo by its name, i.e. mouth these should be split into a subclass and we should make a beoAnimation superclass
  //############################################################
  void setHEyePos(const float position,const int velocity);

  std::deque<Position> getPathHEyePos(const float position,
                                       const int velocity,
                                      const float starpos = -999);

  void setHEyePosRel(const float offset,const int velocity);

  void setVEyePos(const float position,const int velocity);

  std::deque<Position> getPathVEyePos(const float position,
                                      const int velocity,
                                      const float starpos = -999);

  void setVEyePosRel(const float offset,const int velocity);

  void setEyeLidsPos(const float position,const int velocity);

  std::deque<Position> getPathEyeLidsPos(const float position,
                                         const int velocity,
                                      const float starpos = -999);

  void setEyeLidsRel(const float offset,const int velocity);

  void setEyeBrowPos(const float position,const int velocity);

  std::deque<Position> getPathEyeBrowPos(const float position,
                                          const int velocity,
                                      const float starpos = -999);

  void setEyeBrowPosRel(const float offset,const int velocity);

  void setMouthPos(const float position,const int velocity);

  std::deque<Position> getPathMouthPos(const float position,
                                        const int velocity,
                                      const float starpos = -999);

  void setMouthPosRel(const float offset,const int velocity);

  void setMuzzlePos(const float position,const int velocity);

  std::deque<Position> getPathMuzzlePos(const float position,
                                         const int velocity,
                                      const float starpos = -999);

  void setMuzzlePosRel(const float offset,const int velocity);

  void setHHeadPos(const float position,const int velocity);

  std::deque<Position> getPathHHeadPos(const float position,
                                        const int velocity,
                                      const float starpos = -999);

  void setHHeadPosRel(const float offset,const int velocity);

  void setVHeadPos(const float position,const int velocity);

  std::deque<Position> getPathVHeadPos(const float position,
                                        const int velocity,
                                      const float starpos = -999);

  void setVHeadPosRel(const float offset,const int velocity);

  //make a surprsing face
  void surpriseFace(float surprise);

  //###########################################################
  void start2();

  //set everthing to zerp
  void initializeServos();

  //put a sequence in the que, with a servo name and a sequence of positions
  void addSequence(const byte servo, std::deque<float> sequence);

  //put a sequence in the que from another deque of positions
  void addSequence(std::deque<Position> pos);

  //put the sequence at the end of the movement que
  void addEndSequence(std::deque<Position> pos);

  //put the sequence at the end of another sequence
  std::deque<Position> addEndSequence(std::deque<Position> sequence,
                                      std::deque<Position> pos);

  //return a sequence given a servo and an array of positions
  std::deque<Position> getSequence(const byte servo,
                                   std::deque<float> sequence);

  //blend an array of sequences togeather for smooth animation and add it to the que
  void blendSequence(std::deque< std::deque<Position> > elements);

   //blend an array of sequences togeather for smooth animation and return it
  std::deque<Position> getBlendSequence(std::deque<
                                         std::deque<Position> > elements);

  //blend one sequeunce with another
  std::deque<Position> blend(std::deque<Position> p1,
                             std::deque<Position> p2);

  //blend sequenc with current que  to the que a sequence with the
  void setServo(const byte servo,const float pos, const int velocity);

  //return a deque of positions for a given servo movement
  std::deque<Position> getPathServo(const byte servo, const float pos,
                                     const int velocity,
                                     const float startpos = -999);

  //concatenate a deque of deque's into a single deque
  std::deque<Position> concatSequence(std::deque< std::deque<Position> >);

  //return the current movement que
  std::deque<Position> getQue();

  //check if the que is empty
  bool isQueEmpty();

  //perform the next motor movement, return false if que is empty
  bool nextTimeStep();

  //clear the movement que
  void clearQue();

  //remove all movements from a servo
  void removeServo(const byte servo);

  //diognostic: print the Positions in a curent que
  void printPath(std::deque<Position> p);

  //check if a servo is in the movement que
  float isServoInQue(byte servo);

  //check how many movements are in the que
  int queSize();


  //servo numbers
  static const int H_EYE   = 1;
  static const int V_EYE   = 3;
  static const int EYELIDS = 2;
  static const int EYEBROW = 4;
  static const int MOUTH   = 0;
  static const int MUZZLE  = 6;
  static const int H_HEAD  = 7;
  static const int V_HEAD  = 5;

  //min mechanical positions
  static const byte H_EYE_MIN= 56;
  static const byte V_EYE_MIN= 16;
  static const byte EYELIDS_MIN= 5;
  static const byte EYEBROW_MIN= 85;
  static const byte MOUTH_MIN= 85;
  static const byte MUZZLE_MIN= 255;
  static const byte H_HEAD_MIN= 10;
  static const byte V_HEAD_MIN= 10;

  //center position
  static const byte H_EYE_MID= 115;
  static const byte V_EYE_MID= 55;
  static const byte EYELIDS_MID= 90;
  static const byte EYEBROW_MID= 100;
  static const byte MOUTH_MID= 180;
  static const byte MUZZLE_MID= 127;
  static const byte H_HEAD_MID= 90;
  static const byte V_HEAD_MID= 60;

  //max mechanical position
  static const byte H_EYE_MAX= 190;
  static const byte V_EYE_MAX= 110;
  static const byte EYELIDS_MAX= 150;
  static const byte EYEBROW_MAX= 165;
  static const byte MOUTH_MAX= 215;
  static const byte MUZZLE_MAX= 0;
  static const byte H_HEAD_MAX= 235;
  static const byte V_HEAD_MAX= 190;

private:

  //the movemement que
  std::deque<Position> moveQue;

};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
