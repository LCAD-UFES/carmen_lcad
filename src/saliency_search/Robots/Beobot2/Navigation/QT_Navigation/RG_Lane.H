/*!@file Robots2/Beobot2/LaneFollowing/RG_Lane/RG_Lane.H Ice Module to Log data    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/LaneFollowing/RG_Lane/RG_Lane.H
// $Id: RG_Lane.H 12962 2010-03-06 02:13:53Z irock $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include "Robots/Beobot2/BeoCommon.H"

#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "Image/Point3D.H"
#include "Raster/Raster.H"

#include <vector>
#include "Transport/FrameInfo.H"

#ifndef RG_LANE
#define RG_LANE

typedef struct _RegionInformation
{
  uint start;
  uint end;

  float minDistance;
  uint minIndex;
}
RegionInformation;

class imageDB 
{
	std::string path;//Full path such as "../data/corner/HNB/2010_12_34/image00000"
	int start;
	int end;
	int current;	
public:
		imageDB(){}

		imageDB(std::string p,int s,int e){
			start = s;
			end = e;
			path = p;
			current = s;
		}
		Image<PixRGB<byte> > nextImg(){
			std::string imgpath = sformat("%s%05d.ppm",path.c_str(),current);
			LDEBUG("Image path is %s",imgpath.c_str());
			if(current++ > end)
				current = start;			
			return Raster::ReadRGB(imgpath.c_str());
		}
		~imageDB(){}
};
class kalman_filter
{
		float est_var,sensor_var;
		float k;
		float est_x;
	public:
		kalman_filter(float ini_ev,float ini_sv){
			est_var = ini_ev;
			sensor_var = ini_sv;
			k = est_var/(est_var + sensor_var);
			est_x = 0;
		}
		float update(float measure){
			if(est_x == 0){
					est_x = measure;
					return est_x;
			}
			est_x = est_x + 10*k*(measure - est_x);//update estimate x
			est_var = est_var*sensor_var/(est_var+sensor_var);//update p
			k = est_var/(est_var+sensor_var);//update k
			return est_x;
		}

};

class RG_Lane: public RobotBrainComponent
{
public:

  RG_Lane(OptionManager& mgr,
                const std::string& descrName = "RG_Lane",
                const std::string& tagName = "RG_Lane");

  ~RG_Lane();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage (const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&);

  virtual void registerTopics();

  void start1();

private:

	void openDB(const std::string& path);
  void loadFrame();

  void loadDB(const std::string& path);
  void drawState();
  Image<PixRGB<byte> > itsCurrImg;
  Image<PixRGB<byte> > itsProcImg;
  nub::soft_ref<OutputFrameSeries> itsOfs;
	imageDB itsImageDB;
	kalman_filter itsKf;
	int itsKfx;
  //! Overall Display Image
  Image<PixRGB<byte> > itsDispImg;


  Timer itsTimer;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
