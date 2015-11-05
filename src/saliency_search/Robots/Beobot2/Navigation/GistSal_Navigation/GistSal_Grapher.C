/*!@file Robots2/Beobot2/Hardware/Navigation/GistSal_Navigation/GistSal_Grapher.C
 * Ice Module display log data in graph   */
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/GistSal_Navigation/GistSal_Grapher.C
// $ $Id: GistSal_Grapher.C 13084 2010-03-30 02:42:00Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/GistSal_Navigation/GistSal_Grapher.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Ice/IceImageUtils.H"

#include <sys/stat.h>
#include <stdio.h>
#include <dirent.h>
#include <unistd.h>



#define INDOOR
//#define  MANUAL


#ifdef INDOOR
//#define  LOG_FOLDER "../data/logs/IROS10_HNB"
#define  LOG_FOLDER "../data/logs/IROS10_Equad"
#else
#define  LOG_FOLDER "../data/logs/IROS10_Equad"
#endif

// ######################################################################
GistSal_Grapher::GistSal_Grapher(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsTimer(1000000),
        itsDispImage(1600,600,ZEROS),
        itsXwin(itsDispImage,"GistSal Grapher")
  //  itsOfs(new OutputFrameSeries(mgr))
{
  //  addSubComponent(itsOfs);

}

// ######################################################################
GistSal_Grapher::~GistSal_Grapher()
{ }

// ######################################################################
void GistSal_Grapher::start1()
{
//  initLogFile();

  // set start time
  itsTimer.reset();
        searchLogFile();
}

// ######################################################################
bool GistSal_Grapher::initLogFile()
{
  // get the time of day
  time_t rawtime; struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  char buffer [80];
  strftime (buffer,80,
            "%Y_%m_%d__%H_%M_%S",timeinfo);
  std::string startTime(buffer);

  itsLogFoldername =
    std::string(sformat("%s%s", LOG_FOLDER, startTime.c_str()));
  LINFO("logFoldername: %s", itsLogFoldername.c_str());

  // create a log directory
  if (mkdir(itsLogFoldername.c_str(), 0777) == -1)
    {
      LFATAL("Cannot create log folder: %s", itsLogFoldername.c_str());
      return(EXIT_FAILURE);
    }

  std::string logFilename
    (sformat("%s/Log_%s.txt", itsLogFoldername.c_str(), startTime.c_str()));
  LINFO("logFilename: %s", itsLogFilename.c_str());

  std::string cTime = std::string("Time of day: ") + startTime;
  LINFO("%s", cTime.c_str());
  cTime += std::string("\n");

  // save  in a file by appending to the file
  itsLogFilename = logFilename;
  FILE *rFile = fopen(itsLogFilename.c_str(), "at");
  if (rFile != NULL)
    {
      LDEBUG("saving result to %s", logFilename.c_str());
      fputs(cTime.c_str(), rFile);
      fclose (rFile);
    }
  else LFATAL("can't create file: %s", itsLogFilename.c_str());

  return true;
}

// ######################################################################
void GistSal_Grapher::registerTopics()
{
  // subscribe to all sensor data
}

// ######################################################################
void GistSal_Grapher::evolve()
{
        updateGUI();
}

// ######################################################################
void GistSal_Grapher::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  // record the time
  //uint64 time = itsTimer.get();

}

// ######################################################################
void GistSal_Grapher::writeToLogFile(std::string line)
{
  FILE *rFile = fopen(itsLogFilename.c_str(), "at");
  if (rFile != NULL)
    {
      fputs(line.c_str(), rFile);
      fclose (rFile);
    }
  else LFATAL("can't append to file: %s", itsLogFilename.c_str());

}

// ######################################################################
void GistSal_Grapher::searchLogFile()
{
        LINFO("Search for all log file");
        DIR *dir_p;
        struct dirent *entry_p;
        dir_p = ::opendir(LOG_FOLDER);
        if(dir_p == NULL)
                LFATAL("Count Not Open ../data/logs directory");

	std::vector<std::string> subdir;
	while((entry_p = ::readdir(dir_p)))
	{
		std::string subdirname(entry_p->d_name);
		if(subdirname != "." && subdirname !="..")
		{
			//LINFO("File[%d]:[%s]",i++,subdirname.c_str());
			if(subdirname.c_str()[0] == 'S')
				subdir.push_back(subdirname);
		}
	}
	(void)::closedir(dir_p);


        //sort filename
        std::sort(subdir.begin(),subdir.end());
        for(int i = 0;i < (int)subdir.size();i++)
        {
                LINFO("File[%d]:[%s]",i+1,subdir[i].c_str());
        }
#ifdef MANUAL
        int option;

        do{
                LINFO("Please Choose a File Number:");
                scanf("%d",&option);//FIXX Any better idea?
                if(option <1 ||option >int(subdir.size()))
                        LINFO("Option Invalid, please try again");
        }while(option < 1 || option > int(subdir.size()));
#endif




//        LINFO("Your Choose is [%d] filename[%s] ",option,subdir[option-1].c_str());
#ifndef MANUAL
for(int option = 1;option <=int(subdir.size());option++){
#endif
                        //===============================================================
#ifdef INDOOR
        int run,segt;
        char buf[255];
                        sscanf(subdir[option-1].c_str(),"S%d_R%d_%s",&run,&segt,buf);
#endif
                        std::string logFileName(
                                sformat(
#ifdef INDOOR
                                        "%s/%s/Log_%s.txt",
#else
                                        "%s/%s/%s.txt",
#endif
                                        LOG_FOLDER,
                                        subdir[option-1].c_str(),
#ifdef INDOOR
                                        buf
#else
                                        subdir[option-1].c_str()
#endif
                                        ));
                        FILE *logFile = fopen(logFileName.c_str(),"r");


                        if(logFile == NULL)
                        {
                        LFATAL("can't not open file: %s",logFileName.c_str());
                        }else
                        {

				std::string cvsFileName(
						sformat("%s/%s/%s.csv",
							LOG_FOLDER,
							subdir[option-1].c_str(),
							subdir[option-1].c_str()
							));
				itsLogFilename = cvsFileName;
				// save  in a file by creat a new file
				FILE *rFile = fopen(cvsFileName.c_str(), "w");
				if (rFile != NULL)
				{
					fclose (rFile);
				}
				else LFATAL("can't create file: %s", itsLogFilename.c_str()); 

                                char line[512];
                                float x_loc = 0.0;
                                float y_loc = 0.0;
                                float gs = 0.0;
                                int seg = -1;
                                int totalGS = 0;
                                int segCount[4]  = {0,0,0,0};
                                int rc = -1;
                                int frame = 0;
                                while(fgets(line,sizeof line,logFile)!= NULL)
                                {
                                        float lenTrav;
                                        int segNum;
                                        float time;
                                        int ret = sscanf(line,"[%f] LOC seg:%d ltrav:%f",&time,&segNum,&lenTrav);
                                        if(ret == 3)
                                        {
                                                locData loc;
                                                loc.time = time;
                                                loc.segNum = segNum;
                                                loc.lenTrav = lenTrav;
                                                //LINFO("Got Loc Data [%f] seg[%d] len[%f]",time,segNum,lenTrav);
                                                itsLogLocData.push_back(loc);
                                                itsLogSegData.push_back(lenTrav);
                                                gs = lenTrav;
                                                seg = segNum;
                                                totalGS ++;
                                                segCount[seg]++;
                                                if(rc == 3)
                                                        frame ++;

                                        }
                                        float transVel,rotVel,encoderX,encoderY,encoderOri,rcTransVel,rcRotVel;
                                        int rcMode;
                                        ret = sscanf (line,"[%f] MTR rcMode: %d, transVel: %f  rotVel: %f encoderX: %f encoderY: %f encoderOri: %f rcTransVel: %f rcRotVel: %f",
                                                        &time, &rcMode, &transVel, &rotVel, &encoderX,  &encoderY, &encoderOri, &rcTransVel, &rcRotVel);
                                        if(ret == 9)
                                        {
                                                motorData tmp;
                                                tmp.time = time;
                                                tmp.rcMode = rcMode;
                                                tmp.transVel = transVel;
                                                tmp.rotVel = rotVel;
                                                tmp.encoderX = encoderX;
                                                tmp.encoderY = encoderY;
                                                tmp.encoderOri = encoderOri;
                                                tmp.rcTransVel = rcTransVel;
                                                tmp.rcRotVel = rcRotVel;
                                                itsLogMotorData.push_back(tmp);
                                                //LINFO("Got Motor Data %f %d",time,rcMode);
                                                x_loc += encoderX;
                                                y_loc += encoderY;
                                                std::string csvLine (sformat( "%15f,%10f,%10f,%3f,%2d,%3d,%10f,%10f\n",time,x_loc,y_loc,gs,seg,rcMode,transVel,rotVel));
                                                writeToLogFile(csvLine);
                                                //LINFO("%s",csvLine.c_str());
                                                rc = rcMode;
                                        }//end if(ret==9)


                                        float lrfMin,lrfMax;
                                        ret = sscanf(line,"[%f] LRF MIN:%f MAX:%f", &time,&lrfMin,&lrfMax);
                                        if(ret == 3)
                                        {
                                                lrfData lrf;
                                                lrf.time = time;
                                                lrf.lrfMin = lrfMin;
                                                lrf.lrfMax = lrfMax;
                                                LINFO("Got Lrf Data [%f] min[%f] max[%f]",time,lrfMin,lrfMax);
                                                itsLogLrfData.push_back(lrf);
                                        }
                                }//end while


                                float sr[4] = {
                                        ((float)segCount[0]/(float)totalGS)*100.0,
                                        ((float)segCount[1]/(float)totalGS)*100.0,
                                        ((float)segCount[2]/(float)totalGS)*100.0,
                                        ((float)segCount[3]/(float)totalGS)*100.0
                                };
                                std::string segLine (sformat( "T,%4d,%4d,%4d,%4d,%4d,%5.2f%%,%5.2f%%,%5.2f%%,%5.2f%%\n",
                                                        totalGS,segCount[0],segCount[1],segCount[2],segCount[3],sr[0],sr[1],sr[2],sr[3]));
                                //writeToLogFile(segLine);
                                //LINFO("Seg Count: %s",segLine.c_str());
                                //LINFO("Load Log file with %d motor command lines",(int)itsLogMotorData.size());
                                LINFO("Load %s with %d LOC lines ,auto mode %d",subdir[option-1].c_str(),(int)itsLogLocData.size(),frame);
                                itsLogLocData.clear();
                                //LINFO("Load Log file with %d Lrf lines",(int)itsLogLrfData.size());



        }//        if(logFile == NULL)
#ifndef MANUAL
}//end for loop
#endif
}
// ######################################################################
void GistSal_Grapher::updateGUI()
{
                std::vector<std::vector<float> > lines;
                lines.push_back(itsLogSegData);
//                lines.push_back(itsVelocityTargetQue.getVector());

                std::vector<PixRGB<byte> > linesColor;
                linesColor.push_back(PixRGB<byte>(255,0,0));
                //linesColor.push_back(PixRGB<byte>(255,165,0));
                Image<PixRGB<byte> > plotImage = multilinePlot(

//                                itsVelocityQue.getVector(),
                                lines,
                                1600,600,
                                0.0f,1.0f,
                                "","","",
                                linesColor,
                                PixRGB<byte>(255,255,255),
                                PixRGB<byte>(0,0,0)

                                );
                        if(itsXwin.pressedCloseButton()) exit(0);
                        itsXwin.drawImage(plotImage);


}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
