// File: HawkSlammer.H
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <Ice/Ice.h>
#include <QtGui/qapplication.h>
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/PixelsTypes.H"
#include "Image/Dims.H"
#include "Image/CutPaste.H"
#include "GUI/XWinManaged.H"
#include "Robots/BeoHawk/core/HawkMessages.ice.H"
#include "Robots/BeoHawk/computer/HawkAgent.H"
#include <boost/smart_ptr.hpp>

#define HALF_WINDOW_SIZE 256

#define SIGN(A) ((A) >= 0.0 ? (1.0) : (-1.0))
#define SQUARE(A) (((A) * (A)))
#define MAX(A,B) ((A) >= (B) ? (A) : (B))
#define MIN(A,B) ((A) >= (B) ? (B) : (A))

class HawkSlammer : public HawkAgent {
	public:
		// These functions must be defined in all HawkAgents
		HawkSlammer(std::string myName, int argc, char* argv[]);
		bool scheduler();
		void registerTopics();
		void catchMessage(const HawkMessages::MessagePtr& msg, const Ice::Current&);
        
    private:
    	// Data types
    	typedef HawkMessages::SensorDataMessagePtr SensorDataMessagePtr;
    	typedef HawkMessages::SensorDataMessage SensorDataMessage;
    	typedef HawkMessages::LongSeq LongSeq;
    	typedef HawkMessages::Pose Pose;
    	struct Tree;
    	struct Particle;
    	struct Observation;
    	struct Tree {
			boost::shared_ptr<Particle> head;
			boost::shared_ptr<Particle> mostLikely;
			std::vector<boost::shared_ptr<Particle> > leaves;
    	};
    	struct Particle {
			boost::shared_ptr<Particle> parent;
			std::vector<boost::shared_ptr<Particle> > children;
			std::vector<boost::shared_ptr<Observation> > observations;
			Pose robotPose;
			double probability;
    	};
    	struct Observation {
			double laserTravel;
			int laserTerminations;
			boost::shared_ptr<Observation> ancestor;
			boost::shared_ptr<Particle> source;
			int x;
			int y;
    	};
    	
    	// Functions
    	void initializeSlam();
    	void slam();
    	void updateParticles(SensorDataMessagePtr msg);
    	double randomDouble();
    	double gaussian(double stdDev);
    	double cleanAngle(double angle);
    	void addLaserCast(boost::shared_ptr<Particle> treeLeaf, double scanAngle, double scanDistance);
    	void addObservation(boost::shared_ptr<Particle> treeLeaf, int x, int y, double distanceTraveled, int hit);
    	double scoreLaserCast(boost::shared_ptr<Particle> treeLeaf, double scanAngle, double scanDistance);
    	double scoreObservation(boost::shared_ptr<Particle> treeLeaf, int x, int y, double distanceTraveled);
    	void sendSlamDataMessage();
    	void resampleParticles();
    	void pruneParticles();
    	void collapseParticle(boost::shared_ptr<Particle> particle);
    	int countDescendants(boost::shared_ptr<Particle> particle);
    	Image< PixRGB<byte> > makeMapImage(boost::shared_ptr<Particle> particle);
    	Image< PixRGB<byte> > makeScannerImage(SensorDataMessagePtr msg);
    	
    	// General variables
    	bool pauseEachStep;
    	enum Mode {REALTIME_DROP, REALTIME_COMBINE, SIMULATION};
    	Mode mode;
    	XWinManaged window;
    	
    	// Motion model variables
    	double robotXStdDev; // mm
    	double robotYStdDev; // mm
    	double robotThetaStdDev; // mm
    	double scannerVariance; // mm
    	
    	// SLAM variables
    	int numberOfParticles;
    	int mapResolution; // mm per grid cell, 30 means each grid has an area of 900mm^2
    	int maxLaserDistance; // mm
    	std::vector<SensorDataMessagePtr> sensorDataMessages;
    	typedef boost::shared_ptr<
				std::map<
					boost::shared_ptr<Particle>, boost::shared_ptr<Observation>
				>
			> occupancyMapPix_t;
    	typedef Image< occupancyMapPix_t > occupancyMap_t;
		occupancyMap_t occupancyGrid;
    	Tree ancestryTree;
};
