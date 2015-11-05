// File: HawkExample.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include "Robots/BeoHawk/computer/HawkExample.H"

// ######################################################################
HawkExample::HawkExample(std::string myName, int argc, char* argv[])
: HawkAgent(myName, argc, argv) {
	// Help section
	helpTitle = "HawkExample";
	helpDescription = "A basic implementation of HawkAgent.";
	helpOptions.push_back("\t--test\t\t(12.3) Try changing this number!");
	
	// Parameters
	double test = loadDoubleParameter("test", 12.3);
	std::cout << "test: " << test << std::endl;
	
	// Set state
	state = INIT;
}

// ######################################################################
void HawkExample::registerTopics() {
	registerPublisher("ExampleMessage");
	registerSubscription("ExampleMessage");
}

// ######################################################################
bool HawkExample::scheduler() {
	if(state == INIT) {
		sendExampleMessageOne();
		return true;
	}
	else if(state == RECEIVED_FIRST_MESSAGE) {
		sendExampleMessageTwo();
		return true;
	}
	
	return false;
}

// ######################################################################
void HawkExample::catchMessage(const HawkMessages::MessagePtr& hawkMessage, const Ice::Current&) {
	if(hawkMessage->ice_isA("::HawkMessages::ExampleMessage"))
	{
		HawkMessages::ExampleMessagePtr msg = HawkMessages::ExampleMessagePtr::dynamicCast(hawkMessage);
		std::cout << "Caught a message! " << msg->name << " says: " << msg->chatter << std::endl;
		
		// Update State / Wake Up Agent
		if(msg->name != itsName && msg->chatter == "Hello?") {
			state = RECEIVED_FIRST_MESSAGE;
			wakeUp();
		}
	}
}

// ######################################################################
void HawkExample::sendExampleMessageOne() {
	// Create Message
	HawkMessages::ExampleMessagePtr msg = new HawkMessages::ExampleMessage;
    msg->name = itsName;
    msg->chatter = "Hello?";
    
    // Send Message
	publish("ExampleMessage", msg);
	
	// Update State
	state = WAITING;
}

// ######################################################################
void HawkExample::sendExampleMessageTwo() {
	// Create Message
	HawkMessages::ExampleMessagePtr msg = new HawkMessages::ExampleMessage;
    msg->name = itsName;
    msg->chatter = "O hai! ^_^";
    
    // Send Message
	publish("ExampleMessage", msg);
	
	// Update State
	state = WAITING;
}

// ######################################################################
int main (int argc, char* argv[]) {
	std::cout << "HawkExample: starting..." << std::endl;
	
	HawkExample agent("HawkExample", argc, argv);
	agent.start();
	
	std::cout << "HawkExample: all done!" << std::endl;
}
