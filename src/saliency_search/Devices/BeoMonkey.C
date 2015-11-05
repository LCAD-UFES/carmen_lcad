/*!@file Devices/BeoMonkey.C Interfaces to robot monkey head, derived from
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeoMonkey.C $
// $Id: BeoMonkey.C 6795 2006-06-29 20:45:32Z rjpeters $
//

#include "Devices/BeoMonkey.H"
#include "Component/OptionManager.H"
#include "Util/Assert.H"
#include "rutz/compat_snprintf.h"

// ######################################################################
BeoMonkey::BeoMonkey(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName) : BeoChip(mgr,descrName,tagName)
{
  //setup Servos
  calibrateServo(H_EYE,H_EYE_MID,H_EYE_MIN,H_EYE_MAX);
  calibrateServo(V_EYE,V_EYE_MID,V_EYE_MIN,V_EYE_MAX);
  calibrateServo(H_HEAD,H_HEAD_MID,H_HEAD_MIN,H_HEAD_MAX);
  calibrateServo(V_HEAD,V_HEAD_MID,V_HEAD_MIN,V_HEAD_MAX);
  calibrateServo(EYELIDS,EYELIDS_MID,EYELIDS_MIN,EYELIDS_MAX);
  calibrateServo(EYEBROW,EYEBROW_MID,EYEBROW_MIN,EYEBROW_MAX);
  calibrateServo(MOUTH,MOUTH_MID,MOUTH_MIN,MOUTH_MAX);
  calibrateServo(MUZZLE,MUZZLE_MID,MUZZLE_MIN,MUZZLE_MAX);
}


// ######################################################################
void BeoMonkey::setHEyePos(const float position,
                                       const int velocity)
{
  setServo(H_EYE,position,velocity);
}
// ######################################################################
std::deque<Position> BeoMonkey::getPathHEyePos(const float position,
                                       const int velocity,
                                               const float startpos)
{
  return getPathServo(H_EYE,position,velocity,startpos);
}

// ######################################################################
void BeoMonkey::setHEyePosRel(const float offset,
                                       const int velocity)
{
   setHEyePos(getServo(H_EYE)+offset,velocity);

}

// ######################################################################
void BeoMonkey::setVEyePos(const float position,
                                       const int velocity)
{
   setServo(V_EYE,position,velocity);
}

// ######################################################################
std::deque<Position> BeoMonkey::getPathVEyePos(const float position,
                                       const int velocity,
                                               const float startpos)
{
  return getPathServo(V_EYE,position,velocity,startpos);
}

// ######################################################################
void BeoMonkey::setVEyePosRel(const float offset,
                                       const int velocity)
{
   setVEyePos(getServo(V_EYE)+offset,velocity);
}

// ######################################################################
void BeoMonkey::setEyeLidsPos(const float position,
                                       const int velocity)
{
   setServo(EYELIDS,position,velocity);
}

// ######################################################################
std::deque<Position> BeoMonkey::getPathEyeLidsPos(const float position,
                                       const int velocity
,
                                               const float startpos)
{
  return getPathServo(EYELIDS,position,velocity,startpos);
}

// ######################################################################
void BeoMonkey::setEyeLidsRel(const float offset,
                                       const int velocity)
{
   setEyeLidsPos(getServo(EYELIDS)+offset,velocity);
}

// ######################################################################
void BeoMonkey::setEyeBrowPos(const float position,
                                       const int velocity)
{
   setServo(EYEBROW,position,velocity);
}

// ######################################################################
std::deque<Position> BeoMonkey::getPathEyeBrowPos(const float position,
                                       const int velocity,
                                               const float startpos)
{
  return getPathServo(EYEBROW,position,velocity,startpos);
}

// ######################################################################
void BeoMonkey::setEyeBrowPosRel(const float offset,
                                       const int velocity)
{
   setEyeBrowPos(getServo(EYEBROW)+offset,velocity);
}

// ######################################################################
void BeoMonkey::setMouthPos(const float position,
                                       const int velocity)
{
   setServo(MOUTH,position,velocity);
}

// ######################################################################
std::deque<Position> BeoMonkey::getPathMouthPos(const float position,
                                       const int velocity,
                                               const float startpos)
{
  return getPathServo(MOUTH,position,velocity,startpos);
}

// ######################################################################
void BeoMonkey::setMouthPosRel(const float offset,
                                       const int velocity)
{
   setMouthPos(getServo(MOUTH)+offset,velocity);
}

// ######################################################################
void BeoMonkey::setMuzzlePos(const float position,
                                       const int velocity)
{
   setServo(MUZZLE,position,velocity);
}

// ######################################################################
std::deque<Position> BeoMonkey::getPathMuzzlePos(const float position,
                                       const int velocity,
                                               const float startpos)
{
  return getPathServo(MUZZLE,position,velocity,startpos);
}

// ######################################################################
void BeoMonkey::setMuzzlePosRel(const float offset,
                                       const int velocity)
{
   setMuzzlePos(getServo(MUZZLE)+offset,velocity);
}

// ######################################################################
void BeoMonkey::setHHeadPos(const float position,
                                       const int velocity)
{
   setServo(H_HEAD,position,velocity);
}
// ######################################################################
std::deque<Position> BeoMonkey::getPathHHeadPos(const float position,
                                       const int velocity,
                                               const float startpos)
{
  return getPathServo(H_HEAD,position,velocity,startpos);
}

// ######################################################################
void BeoMonkey::setHHeadPosRel(const float offset,
                                       const int velocity)
{
   setHHeadPos(getServo(H_HEAD)+offset,velocity);
}



// ######################################################################
void BeoMonkey::setVHeadPos(const float position,
                                       const int velocity)
{
   setServo(V_HEAD,position,velocity);
}

// ######################################################################
std::deque<Position> BeoMonkey::getPathVHeadPos(const float position,
                                       const int velocity,
                                               const float startpos)
{
  return getPathServo(V_HEAD,position,velocity,startpos);
}

// ######################################################################
void BeoMonkey::setVHeadPosRel(const float offset,
                                       const int velocity)
{
   setVHeadPos(getServo(V_HEAD)+offset,velocity);
}

// ######################################################################
void BeoMonkey::start2()
{
        initializeServos();
}

// ######################################################################
void BeoMonkey::initializeServos()
{
  LINFO("Initializing Servos...");
  //set servos to center
  setServo(H_EYE,0.0F,1);
  setServo(V_EYE,0.0F,1);
  setServo(H_HEAD,0.0F,1);
  setServo(V_HEAD,0.0F,1);
  setServo(EYELIDS,0.0F,1);
  setServo(EYEBROW,0.0F,1);
  setServo(MOUTH,0.0F,1);
  setServo(MUZZLE,0.0F,1);
  nextTimeStep();
  LINFO("Initialization Complete");
}

//set the servos and add blend to the current movement que
//#######################################################################
void BeoMonkey::setServo(const byte servo, const float position,
                         const int velocity)
{
  addSequence(getPathServo(servo,position,velocity));
}

//return the path created from setting a servo to a particular pos and vel
//#######################################################################
std::deque<Position> BeoMonkey::getPathServo(const byte servo,
                                          const float position,
                                          const int velocity,
                                          const float startpos)

{
  std::deque<Position> pos;
  float cur;
  if (startpos > -999)
    cur = startpos;
  else
    {
      cur = isServoInQue(servo); //shhould switch this burdain to add toend?
      if (cur < -2)
        cur = getServo(servo);
    }

  float step = (position - cur)/(float)velocity;
  for (int ii = 1; ii <= velocity; ii++)
    {
      Position p(servo,cur+step*ii,ii);
      pos.push_back(p);
    }
  //should add a function to change the velocty profile so its smooth
  return pos;
}

//add sequence and blend with current movement que
//#######################################################################
void BeoMonkey::addSequence(const byte servo,
                            std::deque<float> sequence)
{
  addSequence(getSequence(servo,sequence));
}

//add sequencey and blend to current movement que
//#######################################################################
void BeoMonkey::addSequence(std::deque<Position> element)
{
  moveQue = blend(moveQue,element);
}

//add sequencey to the end of the movement que
//#######################################################################
void BeoMonkey::addEndSequence(std::deque<Position> pos)
{
    std::deque<Position>::iterator ii;
  if (moveQue.empty())
    moveQue = pos;
  else
    {
      for (ii = pos.begin(); ii != pos.end(); ii++)
        ii->time+= moveQue.back().time;
      moveQue.insert(moveQue.end(),pos.begin(),pos.end());
    }
}

//add sequencey to the end of the movement que
//#######################################################################
std::deque<Position> BeoMonkey::addEndSequence(std::deque<Position> seq,
                               std::deque<Position> pos)
{
  std::deque<Position>::iterator ii;
  if (seq.empty())
    seq = pos;
  else
    {
      for (ii = pos.begin(); ii != pos.end(); ii++)
        ii->time+= seq.back().time;
      seq.insert(seq.end(),pos.begin(),pos.end());
    }
  return seq;
}

//create a position sequence from a deque of servo positions
//#######################################################################
std::deque<Position> BeoMonkey::getSequence(const byte servo,
                                             std::deque<float> sequence)
{
  std::deque<Position> out;
  std::deque<float>::iterator ii;
  int time = 1;
  for(ii = sequence.begin(); ii != sequence.end(); ii++)
    {
      Position p(servo,*(ii),time);
      out.push_back(p);
      time++;
    }
  return out;
}

//take a deque of sequences and blend them togeather them with the current que
//#######################################################################
void BeoMonkey::blendSequence(std::deque< std::deque<Position> > elements)
{
  elements.push_back(moveQue);
  moveQue = getBlendSequence(elements);
}

//blend a series of sequences togeather and return a deque
//fix this to use an iterator
//#######################################################################
std::deque<Position> BeoMonkey::getBlendSequence(std::deque<
                                                  std::deque<Position>
                                                  > elements)
{
 std::deque<Position> b;
  if (elements.size() > 1)
   {
    b  = blend(elements.at(0),elements.at(1));

    for (uint ii = 2; ii < elements.size(); ii++)
      {
        b = blend(b,elements.at(ii));
      }
   }
 return b;
}

//blend two sequences
//#######################################################################
std::deque<Position> BeoMonkey::blend(std::deque<Position> p1,
                                     std::deque<Position> p2)
{
  std::deque<Position> p;
  std::deque<Position>::iterator ii,jj;
  int time;
  if (p1.empty())
    {
      p1.insert(p1.end(),p2.begin(),p2.end());
      return p1;
    }
  else
    {
      time = p1.back().time;
      if (p2.back().time > time)
        time = p2.back().time;
      ii = p1.begin();
      jj = p2.begin();

      for (int k = 1; k <= time; k++)
        {
          for (; ii != p1.end(); ii++)
            {
              if (ii->time == k)
                p.push_back(*ii);
              else
                break;
            }
          for (; jj != p2.end(); jj++)
            {
              if (jj->time == k)
                p.push_back(*jj);
              else
                break;
            }
        }
    }
  return p;
}

//#######################################################################
bool BeoMonkey::isQueEmpty()
{
  return moveQue.empty();
}

//#######################################################################
bool BeoMonkey::nextTimeStep()
{
  if (moveQue.empty() )
    return false;

  else
    {
      while (moveQue.front().time == 1)
        {
          if (!BeoChip::setServo(moveQue.front().servo,
                                 moveQue.front().position))
            return false;
          moveQue.pop_front();
        }
      std::deque<Position>::iterator ii;
      for (ii = moveQue.begin(); ii != moveQue.end(); ii++)
        ii->time--;
    }
  return true;
}

//#######################################################################
std::deque<Position> BeoMonkey::concatSequence(std::deque<
                                                std::deque<Position> > e)
{
  std::deque<Position> f;
  std::deque< std::deque<Position> >::iterator ii;
  for (ii = e.begin(); ii != e.end(); ii++)
    {
      if (f.size() > 0)
        for (std::deque<Position>::iterator jj = ii->begin();
             jj != ii->end(); jj++)
          jj->time+= f.back().time;

      f.insert(f.end(),ii->begin(),ii->end());
    }
  return f;
}

//#######################################################################
void BeoMonkey::clearQue()
{
  moveQue.clear();
}

std::deque<Position> BeoMonkey::getQue()
{
  return moveQue;
}

void  BeoMonkey::removeServo(const byte servo)
{
  //doesn't make sure times are all ok
  std::deque<Position>::iterator ii;
  for (ii = moveQue.begin(); ii != moveQue.end(); )
    {
      if (ii->servo == servo)
        ii =  moveQue.erase(ii);
      else
        ii++;
    }
}

//######################################################################
void BeoMonkey::surpriseFace(float surprise)
{
  //  setEyeLidsPos(1.0,25);
  //setEyeBrowPos(1.0,25);

  std::deque< std::deque<Position> > p;
  p.push_back(getPathMouthPos(-1.0,2));
  p.push_back(getPathMouthPos(1.0,4,-1.0));
  p.push_back(getPathMouthPos(-1.0,4,1.0));
  p.push_back(getPathMouthPos(.5,2,-1.0));
  addSequence(concatSequence(p));
}

//######################################################################
void BeoMonkey::printPath(std::deque<Position> pr)
{
  std::deque<Position>::iterator ii;
  for (ii = pr.begin(); ii != pr.end(); ii++)
          LINFO("Servo: %d Position: %f Time: %d\n",ii->servo,
                ii->position,ii->time);
}

//######################################################################
float BeoMonkey::isServoInQue(byte servo)
{
  if (moveQue.empty())
    return -999;

  float temp = -999;
  std::deque<Position>::reverse_iterator ii;

  for (ii = moveQue.rbegin(); ii != moveQue.rend(); ii++)
    {
      if (ii->servo == servo)
        {
          temp = ii->position;
          return temp;
        }
    }
  return temp;

}

//######################################################################
int BeoMonkey::queSize()
{
return moveQue.size();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
