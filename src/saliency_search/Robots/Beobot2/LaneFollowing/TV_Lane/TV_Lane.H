/*!@file Robots2/Beobot2/LaneFollowing/TV_Lane/TV_Lane.H Ice Module to Log data    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/LaneFollowing/TV_Lane/TV_Lane.H
// $Id: TV_Lane.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef TV_LANE
#define TV_LANE
#include "Image/OpenCVUtil.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

//#include "Demo/SaliencyMT.H"
#include "Robots/Beobot2/LaneFollowing/TV_Lane/SaliencyMT.H"
#include "Robots/Beobot2/BeoCommon.H"

#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "Image/Point3D.H"
#include "Raster/Raster.H"

#include <vector>
#include "Transport/FrameInfo.H"
#include "Gist/SuperPixel.H"


typedef struct _RegionInformation
{
  uint start;
  uint end;

  float minDistance;
  uint minIndex;
}
RegionInformation;

//! for loading simulation images
class imageDB 
{
  std::string path;//Full path such as "../data/corner/HNB/2010_12_34/image00000"
  int start;
  int end;
  int current;	
public:
  imageDB(){}
  
  imageDB(std::string p,int s,int e)
  {
    start = s;
    end = e;
    path = p;
    current = s;
  }
  Image<PixRGB<byte> > nextImg()
  {
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
  kalman_filter(float ini_ev,float ini_sv)
  {
    est_var = ini_ev;
    sensor_var = ini_sv;
    k = est_var/(est_var + sensor_var);
    est_x = 0;
  }
  float update(float measure)
  {
    if(est_x == 0)
      {
        est_x = measure;
        return est_x;
      }
    est_x = est_x + 10*k*(measure - est_x);//update estimate x
    est_var = est_var*sensor_var/(est_var+sensor_var);//update p
    k = est_var/(est_var+sensor_var);//update k
    return est_x;
  }

};

class TV_Lane: public RobotBrainComponent
{
public:

  TV_Lane(OptionManager& mgr,
                const std::string& descrName = "TV_Lane",
                const std::string& tagName = "TV_Lane");

  ~TV_Lane();

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
  void fitLine(std::vector<Point2D<int> > points, Point2D<int>&p1,Point2D<int>&p2);
  Image<PixRGB<byte> > getSuperPixel(Image<PixRGB<byte> > img);
  Image<PixRGB<byte> > getCannyEdge(Image<PixRGB<byte> > img, Image<PixRGB<byte> > &rawCannyImg);
  Image<PixRGB<byte> > getSaliency(Image<PixRGB<byte> > img);
	int getVotingCount(Image<PixRGB<byte> > img,int topX,int topY,int windowSize,PixRGB<byte> bgColor); 
	PixRGB<byte> getBackgroundColor(Image<PixRGB<byte> >img);
	Image<float> getVotingBlock(Image<PixRGB<byte> > img);
	Image<float> updateVoteBlock(Image<float> map);
	Image<float> itsVoteMap;

  Image<PixRGB<byte> > itsCurrImg;
  Image<PixRGB<byte> > itsProcImg;
  nub::soft_ref<SaliencyMT> itsSmt;
  nub::soft_ref<OutputFrameSeries> itsOfs;
  nub::soft_ref<InputFrameSeries> itsIfs;
  imageDB itsImageDB;
  kalman_filter itsKf;
  int itsKfx;
  //! Overall Display Image
  Image<PixRGB<byte> > itsDispImg;

  void updateMotor(double tran, double rot);

  IceUtil::Mutex its_Curr_Img_mutex;        //!< locking log filename
  IceUtil::Mutex its_Curr_Mtr_mutex;        //!< locking log filename
  int itsCurrImgID;
  double itsRcTransSpeed;
  double itsRcRotSpeed;
  int itsRemoteMode;
  int itsPrevProcImgID;
  Timer itsTimer;
	
		// Compute probability given variance s,mean u
		// #####################################################################
		double getNormal(double u, double s, double x){
			double coefficient = 1.0/sqrt(2.0 * M_PI * s*s);
			return coefficient * exp(-(x-u)*(x-u)/(2.0 * s*s));
		}
		double getNormalWeight(double x){
			double u = 0;
			double s = 1;
			return getNormal(u,s,x)/getNormal(u,s,0);
		}
	/*
	float myLinearity(CvSeq *seq)
	{
		int i;
		CvPoint2D32f *p;
		float *x = new float[seq->total];
		float *y = new float[seq->total];
		float x_bar=0.0, y_bar=0.0;
		float u11=0.0, u20=0.0, u02=0.0;
		float linearity=0.0;
		for (i=0; i < seq->total; i++){
			p=(CvPoint2D32f*)cvGetSeqElem(seq,i);
			x=p->x;
			y=p->y;
		}
		//x_bar, y_bar
		for (i=0; i < seq->total; i++){
			x_bar+=x;
			y_bar+=y;
		}
		x_bar/=seq->total;
		y_bar/=seq->total;
		for (i=0; i < seq->total; i++){
			u11+=((x-x_bar)*(y-y_bar));
			u20+=pow(x-x_bar,2.0f);
			u02+=pow(y-y_bar,2.0f);
		}
		u11/=seq->total;
		u20/=seq->total;
		u02/=seq->total;
		linearity = sqrt(4*pow(u11,2.0f)+pow(u20-u02,2.0f))/(u20+u02);
		return linearity;
	}*/
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
