// File: HawkExample.H
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include <string>
#include <Ice/Ice.h>
#include "Robots/BeoHawk/core/HawkMessages.ice.H"
#include "Robots/BeoHawk/computer/HawkAgent.H"

class HawkExample : public HawkAgent {
	public:
		// These functions must be defined in all HawkAgents
		HawkExample(std::string myName, int argc, char* argv[]);
        bool scheduler();
        void registerTopics();
        void catchMessage(const HawkMessages::MessagePtr& msg, const Ice::Current&);
        
    private:
    	// These functions are specific to the individual agents
        enum States {INIT, RECEIVED_FIRST_MESSAGE, WAITING};
        States state;
        //void doSomeBigTask():
    	void sendExampleMessageOne();
    	void sendExampleMessageTwo();
};
