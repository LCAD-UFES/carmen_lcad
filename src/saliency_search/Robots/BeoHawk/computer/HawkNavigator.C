// File: HawkNavigator.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include "Robots/BeoHawk/computer/HawkNavigator.H"

// ######################################################################
HawkNavigator::HawkNavigator(std::string myName, int argc, char* argv[])
: HawkAgent(myName, argc, argv) {
	state = INIT;
}

// ######################################################################
void HawkNavigator::registerTopics() {
	//registerPublisher("ExampleMessage");
	registerSubscription("SlamDataMessage");
}

// ######################################################################
bool HawkNavigator::scheduler() {
	/*if(state == INIT) {
		actionSendFirst();
		return true;
	}
	else if(state == RECEIVED_FIRST_MESSAGE) {
		actionSendSecond();
		return true;
	}*/
	
	return false;
}

// ######################################################################
void HawkNavigator::catchMessage(const HawkMessages::MessagePtr& hawkMessage, const Ice::Current&) {
	if(hawkMessage->ice_isA("::HawkMessages::SlamDataMessage"))
	{
		HawkMessages::SlamDataMessagePtr msg = HawkMessages::SlamDataMessagePtr::dynamicCast(hawkMessage);
		std::cout << "Caught a SlamDataMessage!" << std::endl;
		HawkMessages::Pose hawkPose = msg->hawkPose;
		std::cout << "The BeoHawk's current Pose is (" << hawkPose.x << ", " << hawkPose.y;
		std::cout << ", " << hawkPose.z << ", " << hawkPose.theta << ")" << std::endl;
		
		// Update State / Wake Up Agent
		/*if(msg->name != itsName && msg->chatter == "Hello?") {
			state = RECEIVED_FIRST_MESSAGE;
			wakeUp();
		}*/
	}
}

// ######################################################################
int main (int argc, char* argv[]) {
	std::cout << "HawkNavigator: starting..." << std::endl;
	
	HawkNavigator agent("HawkNavigator", argc, argv);
	agent.start();
	
	std::cout << "HawkNavigator: all done!" << std::endl;
}
