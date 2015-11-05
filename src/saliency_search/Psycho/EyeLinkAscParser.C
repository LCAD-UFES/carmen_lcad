/*!@file Psycho/EyeLinkAscParser.C EyeLink eye-tracker parses the
  *.asc eyetracking file. Usually this comes from the *.edf file that the
  eyetracker produces, then processed by EDF2ASC executable.            */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeLinkAsc.C $
// $Id: $
//

#include "Psycho/EyeLinkAscParser.H"
#include "Util/Timer.H"
#include "Raster/Raster.H"

#include <cstdio>
#include <math.h>

#define trialLength     15.0
#define frameRate       30.0

#define D_WIDTH         1024.0
#define D_HEIGHT        768.0
#define IM_WIDTH        640
#define IM_HEIGHT       480 

#define CREATE_EYE_S    false

// ######################################################################
EyeLinkAscParser::EyeLinkAscParser(std::string fileName)
{  
  std::string::size_type dpos = fileName.find_last_of('.');
  std::string::size_type spos = fileName.find_last_of('/');
  if(spos != std::string::npos) 
    itsSubjectName = fileName.substr(spos+1, dpos-spos-1);
  else
    itsSubjectName = fileName.substr(0, dpos);

  LDEBUG("<<%s>> -> <<%s>>", fileName.c_str(), itsSubjectName.c_str());

  parse(fileName);
}

// ######################################################################
EyeLinkAscParser::~EyeLinkAscParser()
{  }

// ######################################################################
void EyeLinkAscParser::parse(std::string fileName)
{
  FILE *fp;  char inLine[200]; 
  FILE *eyesFile = 0;
  bool createEyeS = CREATE_EYE_S;
  // open the file
  LINFO("Parsing: %s", fileName.c_str());
  if((fp = fopen(fileName.c_str(),"rb")) == NULL)
    { LINFO("not found"); return; }

  int ldpos = fileName.find_last_of('.');
  std::string eyeSfile_prefix = fileName.substr(0, ldpos);
  LINFO("string: %s", eyeSfile_prefix.c_str());

  // determine recorded eye
  // also skip the header information
  std::string recordedEye;
  while(fgets(inLine, 200, fp) != NULL)
    {
      // get the word tokens
      std::vector<std::string> currLine = tokenize(inLine);
      
      if(currLine.size() >= 4 && !currLine[0].compare("EVENTS"))
        {
          if(!currLine[2].compare("RIGHT"))
            recordedEye = std::string("RIGHT");
          else if (!currLine[2].compare("LEFT") && 
                   !currLine[3].compare("RIGHT"))
            recordedEye = std::string("BOTH");
          else
            recordedEye = std::string("LEFT");
          break;          
        }
    }
  fclose(fp);

  LINFO("Recorded Eye: %s", recordedEye.c_str());

  // start over and reread the file
  // read and store data
  if((fp = fopen(fileName.c_str(),"rb")) == NULL)
    { LINFO("not found"); return; }

  uint iTrial  = 0;
  uint iTime   = 0;
  uint fTime   = 0;
  uint iSacc   = 0;
  //uint iBlink  = 0;
  uint iFix    = 0;
  bool isDriftCorrecting = false;
  bool isBlinking  = false;
  bool isSaccading = false;

  // this is for .eyeS file 
  float  targetx    = 0.0F;
  float  targety    = 0.0F;
  float  amp        = 0.0F;
  float  pvel       = 0.0F;
  double timeOn     = 0.0F;
  double timeOff    = 0.0F;
  float  interval   = 0.0F;   
  double startTime  = 0.0F;
 
  // get each line in the file
  while(fgets(inLine, 200, fp) != NULL)
    {
      // get the word tokens
      std::vector<std::string> currLine = tokenize(inLine);

      if(currLine.size() == 0) continue; 
      
      // skip calibration lines    
      if(currLine.size() == 3 && !currLine[2].compare("!CAL"))
        continue;

      // check the line first argument

      // events: MSG
      if(currLine.size() > 0 && !currLine[0].compare("MSG"))
        {
          // get rid of unnecessary MSG lines
          // if length(tmp2) < 5
          //   tmp2
          // continue
          // end
            
          // START of the current trial
          if(currLine.size() > 6 && 
             !currLine[4].compare("SESSION") && !currLine[5].compare("START"))
            {                     
              fTime   = 0; 
              iTime   = 0;
              iSacc   = 0;
              //iBlink  = 0;
              iFix    = 0;
              iTrial  = iTrial + 1; 
              itsTrialStart.push_back(atoi(currLine[1].c_str()));
              // for now to see if drift correction 
              // is in the middle of the trial
              itsTrialEnd.push_back(-1); 

              itsFlipTime.push_back(std::vector<int>());
              itsFlipFixations.push_back
                (std::vector<std::vector<Point2D<float> > >());
              itsFlipFrame.push_back(std::vector<int>());

              itsSaccadeStart.push_back(std::vector<int>());
              itsSaccadeEnd.push_back(std::vector<int>());
              itsSaccadeDuration.push_back(std::vector<int>());
              itsSaccadeStartLocation.push_back(std::vector<Point2D<float> >());
              itsSaccadeEndLocation.push_back(std::vector<Point2D<float> >());
              itsSaccadeAmplitude.push_back(std::vector<float>()); 
              itsSaccadePeakVel.push_back(std::vector<float>());
 
              itsFixationStart.push_back(std::vector<int>());
              itsFixationEnd.push_back(std::vector<int>());
              itsFixationDuration.push_back(std::vector<int>());
              itsFixationAvgLocation.push_back(std::vector<Point2D<float> >());
              itsFixationAvgPupilSize.push_back(std::vector<float>());

              itsGazeTime.push_back(std::vector<int>());
              itsLeftGaze.push_back(std::vector<Point2D<float> >());
              itsLeftGazePupilSize.push_back(std::vector<float>());

              itsRightGaze.push_back(std::vector<Point2D<float> >());
              itsRightGazePupilSize.push_back(std::vector<float>());

              if(iTrial != uint(atoi(currLine[6].c_str()))) 
                LINFO("Trial Mismatch!");
              LDEBUG("%s",inLine);
              LDEBUG("Start trial %s", currLine[6].c_str());

              // print in the header
              if(createEyeS)
                {
                  std::string fName = 
                    sformat("%s_%d.eyeS", eyeSfile_prefix.c_str(), iTrial);
                  eyesFile = fopen(fName.c_str(), "wt");

                  std::string period("period = 1000Hz\n");
                  fputs (period.c_str(), eyesFile);

                  std::string ppd("ppd = 36.3\n");
                  fputs (ppd.c_str(), eyesFile);

                  std::string trash("trash = 0\n");
                  fputs (trash.c_str(), eyesFile);

                  std::string header("cols = x y pd status *targetx *targety "
                                     "*amp *pvel *timeon *timeoff "
                                     "*interval *typenum\n");
                  fputs (header.c_str(), eyesFile);

                  LINFO("creating: %s", fName.c_str());
                }
            }
         
          // Nothing to do until the start of the first trial is found
          if(iTrial == 0) continue;            
 
          // New Video Frame is just shown
          if(currLine.size() == 4 && 
             !currLine[2].compare("MARKERID") && !currLine[3].compare("Flip"))
            {
              itsFlipTime[iTrial-1].push_back(atof(currLine[1].c_str()));
          
              if(fTime == 0)
                itsFlipFrame[iTrial-1].push_back(0);
              else
                {
                 if((itsFlipTime[iTrial-1][fTime] - itsFlipTime[iTrial-1][0]) 
                    > trialLength * 1000)
                   {
                     //LINFO("[%d][%" ZU "]", iTrial-1, itsFlipFrame[iTrial-1].size());
                     itsFlipFrame[iTrial-1].push_back(trialLength * frameRate);
                   }
                 else
                   {
                     int tDiff = 
                       itsFlipTime[iTrial-1][fTime] - itsFlipTime[iTrial-1][0];
                     float mspfr = 1000/frameRate;
                     int frNum = floor(tDiff/mspfr);
                     itsFlipFrame[iTrial-1].push_back(frNum);
                   }
                }
              //LINFO("Flip at %d to frame %d\n", 
              //      itsFlipTime[iTrial-1][fTime], itsFlipFrame[iTrial-1][fTime]);  
              fTime = fTime + 1;
              itsFlipFixations[iTrial-1].push_back
                (std::vector<Point2D<float> >());

              if(fTime == 1) startTime = atoi(currLine[1].c_str());  
            }
         
          // END of the current trial
          if (currLine.size() > 6 && 
             !currLine[4].compare("SESSION") && !currLine[5].compare("END"))
            {
              itsTrialEnd[iTrial-1] = atoi(currLine[1].c_str());
              if (iTrial != uint(atoi(currLine[6].c_str()))) 
                LINFO("Trial Mismatch!");
                           
              LDEBUG("%s",inLine);
              LDEBUG("trial: %d to %d",
                     itsFlipTime[iTrial-1][0], itsFlipTime[iTrial-1][fTime-1]);
              LDEBUG("there are %" ZU " saccades and %" ZU " fixations", 
                     itsSaccadeStart[iTrial-1].size(), 
                     itsFixationStart[iTrial-1].size());

              if(createEyeS)
                fclose (eyesFile);
            }
         
          // Nothing to do if the ending of the trial is already found
          if (itsTrialEnd[iTrial-1] != -1.0) continue;            

          // drift correction
          if (currLine.size() > 4 && !currLine[4].compare("CORRECTION"))
            {
              LINFO("in between drifting: %s",inLine);
              //
              if(currLine.size() == 5 && !currLine[3].compare("DRIFT"))
                {
                  isDriftCorrecting = 1;
                  itsDriftCorrectionStart.push_back(atoi(currLine[1].c_str()));
                  LINFO("Start in between drifting correction at: %d", 
                        itsDriftCorrectionStart[iTrial-1]);
                }
              else if(currLine.size() == 6 && !currLine[5].compare("DONE"))
                {  
                  isDriftCorrecting = 0;
                  itsDriftCorrectionDone.push_back(atoi(currLine[1].c_str()));
                  LINFO("End   in between drifting correction at: %d",
                        itsDriftCorrectionDone[iTrial-1]);
                }
              else
                LINFO("Wrong Drift Correction Message!");              
            }
        }
         
      //
      else if (currLine.size() > 0 && 
               (!currLine[0].compare("SSACC")  || 
                !currLine[0].compare("ESACC")  || 
                !currLine[0].compare("SFIX")   || 
                !currLine[0].compare("EFIX")   || 
                !currLine[0].compare("SBLINK") || 
                !currLine[0].compare("EBLINK")   ))
        {
          // Nothing to do until the start of the first trial is found
          if (iTrial == 0) continue;            
          
          // Nothing to do if the ending of the trial is already found
          if (itsTrialEnd[iTrial-1] != -1.0) continue;            
          
          //LINFO("Saccading, etc: %s",inLine);
               
          // SACCADE of the current saccade
          if(!currLine[0].compare("SSACC")) 
            {
              isSaccading = true;
              iSacc = iSacc + 1;
              
              // we have to go move forward
              // that has the saccade information
              
              // get current position
              fpos_t pos;
              fgetpos(fp, &pos);
              char tline[200]; 
              bool foundESACC = false;
              while(!foundESACC && fgets(tline, 200, fp) != NULL)
                {
                  // get the word tokens
                  std::vector<std::string> cLine = tokenize(tline);
                  
                  // found ESACC
                  if(cLine.size() > 0 && !cLine[0].compare("ESACC"))
                    {
                      targetx    = atof(cLine[7].c_str())/D_WIDTH*IM_WIDTH;
                      targety    = atof(cLine[8].c_str())/D_HEIGHT*IM_HEIGHT;
                      amp        = atof(cLine[9].c_str());
                      pvel       = atof(cLine[10].c_str());
                      timeOn     = atof(cLine[2].c_str()) - startTime;
                      timeOff    = atof(cLine[3].c_str()) - startTime;
                      interval   = atof(cLine[4].c_str());
                      foundESACC = true;

                      LDEBUG("%.1f %.1f %.1f %.1f %.1f %.1f %.1f %d", 
                             targetx, targety, amp, pvel, 
                             timeOn, timeOff, interval, iSacc);
                    }
                }

              // then we move back to the original line
              fsetpos(fp, &pos);
            }

          // END of the current saccade
          if(!currLine[0].compare("ESACC")) 
            {
              itsSaccadeStart        [iTrial-1].push_back(atoi(currLine[2].c_str()));
              itsSaccadeEnd          [iTrial-1].push_back(atoi(currLine[3].c_str()));
              itsSaccadeDuration     [iTrial-1].push_back(atoi(currLine[4].c_str()));
              itsSaccadeStartLocation[iTrial-1].push_back
                (Point2D<float>(atof(currLine[5].c_str()), atof(currLine[6].c_str())));
              itsSaccadeEndLocation  [iTrial-1].push_back
                (Point2D<float>(atof(currLine[7].c_str()), atof(currLine[8].c_str())));
              itsSaccadeAmplitude    [iTrial-1].push_back(atof(currLine[9].c_str()));
              itsSaccadePeakVel      [iTrial-1].push_back(atof(currLine[10].c_str()));
              //fprintf('Saccade %d\n', iSacc)

              isSaccading = false;
            }

          // we won't process: SFIX
          
          // END of the current fixation 
          else if(!currLine[0].compare("EFIX"))
            {
              iFix = iFix + 1;
              
              itsFixationStart       [iTrial-1].push_back(atoi(currLine[2].c_str()));
              itsFixationEnd         [iTrial-1].push_back(atoi(currLine[3].c_str()));
              itsFixationDuration    [iTrial-1].push_back(atoi(currLine[4].c_str()));
              itsFixationAvgLocation [iTrial-1].push_back
                (Point2D<float>(atof(currLine[5].c_str()), atof(currLine[6].c_str())));
              itsFixationAvgPupilSize[iTrial-1].push_back(atof(currLine[7].c_str()));
              //fprintf('Fixation %d:  at: (%7.2d %7.2d) for %f \n', iFix, iT(iTrial).FIX{iFix}.avg_x, iT(iTrial).FIX{iFix}.avg_y, iT(iTrial).FIX{iFix}.duration);
            }  
          else if(!currLine[0].compare("SBLINK"))
            isBlinking = true;
          else if(!currLine[0].compare("EBLINK"))
            isBlinking = false;                     
        }
                 
      // not processing these commands    
      else if (currLine.size() > 0 && 
               (!currLine[0].compare("START")      || 
                !currLine[0].compare("END")        || 
                !currLine[0].compare("PRESCALER")  || 
                !currLine[0].compare("VPRESCALER") || 
                !currLine[0].compare("PUPIL")      || 
                !currLine[0].compare("SAMPLES")    ||
                !currLine[0].compare(">>>>>>>")      ))
        { continue; }
      
      else if (currLine.size() > 0 && !currLine[0].compare("EVENTS"))
        {         
          // if changing specs for which eye is being recorded
          if(!currLine[1].compare("GAZE"))
            {
              if(!currLine[2].compare("RIGHT"))
                recordedEye = std::string("RIGHT");
              else if(!currLine[2].compare("LEFT") && !currLine[3].compare("RIGHT"))
                recordedEye = std::string("BOTH");
              else
                recordedEye = std::string("LEFT");                
              LDEBUG("Recorded Eye: %s", recordedEye.c_str());
            }
        }
     
     // skipping 'INPUT .... 120'
     // not sure who inserted this line
      else if (currLine.size() > 0 && !currLine[0].compare("INPUT"))
        { continue; }

      // first element exception
      else if(atoi(currLine[0].c_str()) == 0)
        {
          LDEBUG("First Element Exception: Skipped: %s", inLine);
          continue;
        }

      // otherwise it's gaze for current timeStamp         
      else
        {
          // Nothing to do until the start of the first trial is found
          if (iTrial == 0) continue;            
          
          // Nothing to do if the ending of the trial is already found
          if(itsTrialEnd[iTrial-1] != -1.0) continue;            
          
          // Nothing to do if currently correcting drift
          if (isDriftCorrecting) continue;

          // we record to eyeS file even on blinks
          // status:
          // 0: regular fixation
          // 1: saccade
          // 2: blink
          // 3: saccade + blink
          // 4: smooth pursuit: NOT IMPLEMENTED HERE
          // 5: invalid
          // 6: combined saccade: NOT IMPLEMENTED HERE
          if(fTime != 0)
            {
              uint status = 0;
              if(!isBlinking &&  isSaccading) status = 1;
              if( isBlinking && !isSaccading) status = 2;
              if( isBlinking &&  isSaccading) status = 3;

              uint freq = 0; if(targetx != 0.0F) freq = iSacc;
              std::string line = 
                sformat("%.1f %.1f %.1f %d %.1f %.1f %.1f %.1f %.1f %.1f %.1f %d \n", 
                        atof(currLine[1].c_str())/D_WIDTH *IM_WIDTH, 
                        atof(currLine[2].c_str())/D_HEIGHT*IM_HEIGHT,
                        atof(currLine[3].c_str()), status, 
                        targetx, targety, amp, pvel, timeOn, timeOff, interval, freq);
              if(createEyeS)
                fputs (line.c_str(), eyesFile);          
              
              if(targetx != 0.0F)
                {
                  targetx    = 0.0F;
                  targety    = 0.0F;
                  amp        = 0.0F;
                  pvel       = 0.0F;
                  timeOn     = 0.0F;
                  timeOff    = 0.0F;
                  interval   = 0.0F;
                }
            }

          // Nothing to do until stop blink
          if (isBlinking) continue;            
          
          // retrieve the data
          //LINFO("times: %s",inLine);
          iTime = iTime + 1;

          // get time stamp
          itsGazeTime[iTrial-1].push_back(atoi(currLine[0].c_str()));
          if(fTime != 0)
            {
              itsFlipFixations[iTrial-1][fTime-1].push_back
                (Point2D<float>(atof(currLine[1].c_str()), atof(currLine[2].c_str())));
            }
          LDEBUG("GAZE[%5d][%5d]: %s size: %" ZU , 
                iTrial-1, fTime-1, inLine, itsGazeTime[iTrial-1].size());

          if(!recordedEye.compare("LEFT") || !recordedEye.compare("BOTH"))
            {
              // left eye X, Y 
              if (!currLine[1].compare(".") || !currLine[2].compare("."))
                itsLeftGaze[iTrial-1].push_back(Point2D<float>(-1.0, -1.0));
              else
                itsLeftGaze[iTrial-1].push_back
                  (Point2D<float>(atof(currLine[1].c_str()), atof(currLine[2].c_str())));
              
              // left eye pupil
              if (!currLine[3].compare("."))
                itsLeftGazePupilSize[iTrial-1].push_back(-1.0);
              else
                itsLeftGazePupilSize[iTrial-1].push_back(atof(currLine[3].c_str()));
            }
          
          if(!recordedEye.compare("BOTH"))
            {
              // right eye X, Y 
              if (!currLine[4].compare(".") || !currLine[5].compare("."))
                itsRightGaze[iTrial-1].push_back(Point2D<float>(-1.0, -1.0));
              else
                itsRightGaze[iTrial-1].push_back
                  (Point2D<float>(atof(currLine[4].c_str()), atof(currLine[5].c_str())));
              
              // right eye pupil
              if (!currLine[6].compare("."))
                itsRightGazePupilSize[iTrial-1].push_back(-1.0);
              else
                itsRightGazePupilSize[iTrial-1].push_back(atof(currLine[6].c_str()));
            }
          
          if(!recordedEye.compare("RIGHT"))
            {
              // right eye X, Y 
              if (!currLine[1].compare(".") || !currLine[2].compare("."))
                itsRightGaze[iTrial-1].push_back(Point2D<float>(-1.0, -1.0));
              else
                itsRightGaze[iTrial-1].
                  push_back(Point2D<float>(atof(currLine[1].c_str()), atof(currLine[2].c_str())));
              
              // right eye pupil
              if (!currLine[3].compare("."))
                itsRightGazePupilSize[iTrial-1].push_back(-1.0);
              else
                itsRightGazePupilSize[iTrial-1].push_back(atof(currLine[3].c_str()));
            }
        }
    }
}

// ######################################################################
std::vector<std::string> EyeLinkAscParser::tokenize(std::string line)
{
  bool endOfLine = false;

  // take out the carriage return at the back of the line
  if(!line.substr(line.length()-1).compare(std::string("\n")))
    line = line.substr(0, line.length()-2);

  std::vector<std::string> currLine;
  while(!endOfLine)
    { 
      std::string token; std::string rest;
      std::string::size_type fspos = line.find_first_of(' ');
      std::string::size_type ftpos = line.find_first_of('\t');      
      if(ftpos != std::string::npos && ftpos < fspos) fspos = ftpos;

      if(fspos != std::string::npos)
        {
          token = line.substr(0,fspos);
          rest =  trim(line.substr(fspos));

          LDEBUG("[[%s]] [[%s]]", token.c_str(), rest.c_str());
          
          currLine.push_back(token);
          if(!rest.substr(0,1).compare(std::string("\t")))
            rest = trim(rest.substr(1));
          line = rest;
        }
      else
        {
          if(line.length() != 0)
            {
              currLine.push_back(line);
              LDEBUG("[[%s]]", line.c_str());
            }
          endOfLine = true;
        }
    }

  return currLine;
}

// ######################################################################
std::string EyeLinkAscParser::trim(std::string str)
{
  std::string::size_type pos = str.find_last_not_of(' ');

  if(pos != std::string::npos) {
    str.erase(pos + 1);
    pos = str.find_first_not_of(' ');
    if(pos != std::string::npos) str.erase(0, pos);
  }
  else str.erase(str.begin(), str.end());

  return str;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
