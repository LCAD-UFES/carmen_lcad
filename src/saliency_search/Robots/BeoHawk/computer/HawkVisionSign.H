// File: HawkVisionSign.H
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Kevin, Chris, Justin
// Date: May 2010

#include <string>
#include <Ice/Ice.h>
#include "Robots/BeoHawk/core/HawkMessages.ice.H"
#include "Robots/BeoHawk/computer/HawkAgent.H"

class HawkVisionSign : public HawkAgent {
	public:
		// These functions must be defined in all HawkAgents
		HawkVisionSign(std::string myName, int argc, char* argv[]);
        bool scheduler();
        void registerTopics();
        void catchMessage(const HawkMessages::MessagePtr& msg, const Ice::Current&);
        
    private:
    	// These functions are specific to the individual agents
        enum States {INIT, IDLE, PROCESSING};
        States state;
    	void doSignSearch();
};
