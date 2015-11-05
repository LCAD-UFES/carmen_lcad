// File: HawkScanner.H
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include <string>
#include <vector>
#include <Ice/Ice.h>
#include <urg/UrgCtrl.h>
#include "Util/Timer.H"
#include "Robots/BeoHawk/core/HawkMessages.ice.H"
#include "Robots/BeoHawk/computer/HawkAgent.H"

#define SCANNER_ADDRESS "/dev/ttyACM0"
#define SCANNER_RESOLUTION (M_PI/512.) // URG-04LX-UG01, radians

class HawkScanner : public HawkAgent {
	public:
		// These functions must be defined in all HawkAgents
		HawkScanner(std::string myName, int argc, char* argv[]);
        bool scheduler();
        void registerTopics();
        void catchMessage(const HawkMessages::MessagePtr& msg, const Ice::Current&);
        
    private:
    	// These functions are specific to the individual agents
    	int refreshPeriod;
		Timer timer;
		qrk::UrgCtrl urg;
    	void sendScannerMessage();
};
