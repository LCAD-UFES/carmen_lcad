// File: HawkFreezer.H
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include <string>
#include <vector>
#include <fstream>
#include <Ice/Ice.h>
#include "Util/Timer.H"
#include "Robots/BeoHawk/core/HawkMessages.ice.H"
#include "Robots/BeoHawk/computer/HawkAgent.H"

class HawkFreezer : public HawkAgent {
	public:
		// These functions must be defined in all HawkAgents
		HawkFreezer(std::string myName, int argc, char* argv[]);
        bool scheduler();
        void registerTopics();
        void catchMessage(const HawkMessages::MessagePtr& msg, const Ice::Current&);
        
    private:
    	// These functions are specific to the individual agents
        enum States {INIT_FAIL, LOAD_PLAYBACK, PLAYBACK, RECORD};
        States state;
		Timer timer;
		struct Message {
			double timestamp;
			HawkMessages::SensorDataMessagePtr sensorData;
		};
    	std::vector<Message> messages;
        double frequency; // messages per seconds
        double duration; // seconds
        double period;
        double lastMessageSeconds;
        std::string file;
        void playNextMessages();
        void loadMessages();
        void saveMessages();
};
