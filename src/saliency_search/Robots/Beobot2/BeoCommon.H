#include "Image/Pixels.H"
#include "Image/Image.H"
#include "Raster/Raster.H"
#include "GUI/XWinManaged.H"//for keyboard & mouse click
#include <vector>
#ifndef BEOCOMMON_H
#define BEOCOMMON_H

//! typical maximum translational speed
#define TYPICAL_MAXIMUM_SPEED            0.6   // out of 1.0
#define BEOBOT2_STDEV_ODOMETRY_ERROR     0.005 // <m>

//! <mm> including wheel thickness and some slack (~2cm)  
#define BEOBOT2_ROBOT_WIDTH              600 // <mm>
#define BEOBOT2_ROBOT_HEIGHT             600 // <mm>

//! <mm> distance between two wheel
#define BEOBOT2_ROBOT_WHEEL_DIST              510 // <mm> 0.51m
//! Assume robot body is cycle, clearance radius
#define BEOBOT2_ROBOT_RADIUS 		 420 // <mm>

//! local map information
#define LOCAL_MAP_NUM_HORZ_GRIDS         64 
#define LOCAL_MAP_NUM_VERT_GRIDS         64
#define LOCAL_MAP_GRID_WIDTH             BEOBOT2_ROBOT_WIDTH/4 // <mm> 150mm
#define LOCAL_MAP_GRID_HEIGHT            BEOBOT2_ROBOT_WIDTH/4 // <mm> 150mm
#define LOCAL_MAP_HORZ_ORIGIN            0.5
#define LOCAL_MAP_VERT_ORIGIN            0.5
#define LOCAL_MAP_DISPLAY_GRID_SIZE      8
#define LOCAL_MAP_DEFAULT_GRID_VALUE     -1.0

//! immediate map information
#define IMMEDIATE_MAP_WIDTH              3000 // <mm> 
#define IMMEDIATE_MAP_HEIGHT             3000 // <mm> 
#define IMMEDIATE_MAP_GRID_WIDTH         30   // <mm>
#define IMMEDIATE_MAP_GRID_HEIGHT        30   // <mm>
#define IMMEDIATE_MAP_DISPLAY_GRID_SIZE  4
#define IMMEDIATE_MAP_HORZ_ORIGIN        0.5
#define IMMEDIATE_MAP_VERT_ORIGIN        0.75

#define ROBOT_CENTER_RATIO_I             0.5
#define ROBOT_CENTER_RATIO_J             0.75

//! different type of BeoNavigator Goal type
#define BEONAVIGATOR_TURN_GOAL              1010100
#define BEONAVIGATOR_STATIONARY_TARGET_GOAL 1010101
#define BEONAVIGATOR_MOVING_TARGET_GOAL     1010102



//! libboost function
#define foreach         BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH


//Application ID
enum {BEO_ALL,BEO_NAVIGATOR,BEO_LOCALIZER,BEO_ROADFINDER,BEO_PILOT,BEO_WEB,BEO_SIM};
//Camera Type
enum {RGB,RGBD,DEPTH};

namespace Beobot2
{
	typedef struct 
	{
		float translation;
		float rotation;
	} MotorCommand;

	typedef struct {
		double dist,angle;
	} sonarData;

	typedef struct da{
		double dist,angle;
		da():dist(0.0),angle(0.0){}
		da(double d,double a):dist(d),angle(a){}
	} distAngle;

	typedef struct arc{
		Point2D<double> center;
		double startAng;
		double endAng;
		arc():center(Point2D<double>(0.0,0.0)),startAng(0.0),endAng(M_PI){}
	}Arc;

	typedef struct{
		double time,transVel,rotVel,encoderX,encoderY,encoderOri;
		double rcTransVel,rcRotVel,rcTransCap,rcRotCap;
		double robotTransVel,robotRotVel;
		int rcMode;

    double rawEncoderX,rawEncoderY,rawEncoderOri;
    double imuHeading;
    double trajLen,dist;
    double battery;


	} motorData;

	typedef struct{
		int frameID;
		int cameraID;
		double time;
		std::string imagePath ;
		std::string fileName;
	} imageData;

	typedef struct{
		double time;
		double roll;
		double pitch;
		double yaw;
	} imuData;

	typedef struct{
		double time;
		double lrfMax;
		double lrfMin;
		std::vector<double> dist;
    int deviceID;
	} lrfData;
}

// ######################################################################
template<typename T>
class Point3DwithColor
{
public:
  //Point3DwithColor(const Point3DwithColor&);
  //Point3DwithColor& operator=(const Point3DwithColor&);
  T x,y,z;
  PixRGB<byte> color;
  Point3DwithColor(const T xx,const T yy, const T zz)
  {
    x = xx;y=yy;z=zz;
    color = PixRGB<byte>(0,0,0);
  }

  ~Point3DwithColor(){};
};

// ######################################################################
template<typename T>
class velocityQue{
	// Prevent copy and assignment
	velocityQue(const velocityQue&);
	velocityQue& operator=(const velocityQue&);
	//Max size of data
	int m_size;
	//Current size total
	int m_total;
	//max val in queue
	std::deque<T> m_data;
	public:
	velocityQue():m_size(0)
	{
	}
	velocityQue(int n):m_size(0)
	{
		m_size = n;
	}
	void add(const T& data){
		// if the queue is full yet, we pop the oldest data out
		if(m_total >= m_size)
		{
			m_data.pop_back();
		}else{
			m_total++;
		}
		m_data.push_front(data);
	}
	std::vector<T> getVector()
	{
		return std::vector<T>(m_data.begin(),m_data.end());
	}
	T getMax()			
	{
		T max = *std::max_element(m_data.begin(),m_data.end());	
		return max;
	}
	T getMin()			
	{
		return  *std::min_element(m_data.begin(),m_data.end());	
	}
	~velocityQue(){}
};
// ######################################################################
// random color
inline PixRGB<byte> random_rgb(){
  PixRGB<byte> c((byte)random(),
                 (byte)random(),
                 (byte)random());
  return c;
}
// ######################################################################

class DebugWin
{
	public:
	DebugWin():m_counter(0) { }
  ~DebugWin(){}
  void show(Image<PixRGB<byte> >disp, std::string title = "Debug Win")
  {
    //rutz::shared_ptr<XWinManaged> itsWin;
    //limit max window to 10
    if(m_counter < 10) m_counter ++;
    std::string t = std::string(sformat("%s %d",title.c_str(),m_counter));

    rutz::shared_ptr<XWinManaged> win;
    //if we don't have enough initialized window
    if(m_winVec.size() < m_counter)
    {
      win.reset(new XWinManaged(disp.getDims(), 0, 0, t.c_str()));
      m_winVec.push_back(win);

    }else{		 
      //win.reset(new XWinManaged(disp.getDims(), 0, 0, t.c_str()));
      win = m_winVec[m_counter-1];		 
    }

    win->drawImage(disp,0,0);

    // for(uint ti = 0; ti < rTemplate.size(); ti++)
    //   disp2.setVal(rTemplate[ti], 1.0F);
    // Raster::waitForKey();   
  }	
  void reset(){ m_counter = 0;}
  private:
		std::vector<rutz::shared_ptr<XWinManaged> > m_winVec;
		uint m_counter;

};
#endif
