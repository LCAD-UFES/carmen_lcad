// File: HawkVisionSign.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Kevin, Chris, Justin
// Date: May 2010

#include "Robots/BeoHawk/computer/HawkVisionSign.H"

// ######################################################################
HawkVisionSign::HawkVisionSign(std::string myName, int argc, char* argv[])
: HawkAgent(myName, argc, argv) {
	state = INIT;
}

// ######################################################################
void HawkVisionSign::registerTopics() {
	registerPublisher("RoomFinder");
	registerSubscription("ControlRoomVision");
	registerSubscription("CameraImage");
}

// ######################################################################
bool HawkVisionSign::scheduler() {
	if(state == INIT) {
		//sendExampleMessageOne();
		return true;
	}
	else if(state == IDLE) {
		//sendExampleMessageTwo();
		return true;
	}else if(state == PROCESSING){
		doSignSearch();
		return true;
	}
	
	return false;
}

// ######################################################################
void HawkVisionSign::catchMessage(const HawkMessages::MessagePtr& hawkMessage, const Ice::Current&) {
	if(hawkMessage->ice_isA("::HawkMessages::ControlDriveVision"))
	{
		HawkMessages::ControlDriveVisionMessage msg = HawkMessages::ControlDriveVisionMessage::dynamicCast(hawkMessage);
		std::cout << "signVisionOn" << std::endl;
		
		// Update State / Wake Up Agent
		if(msg->signVisionOn) {
			state = IDLE;
			wakeUp();
		}
	}
	if(hawkMessage->is_A("::HawkMessages::CameraImageMessage") && state != PROCESSING)
	{
		HawkMessages::CameraImageMessage msg = HawkMessages::ControlDriveVisionMessage::dynamicCast(hawkMessage);
		std::cout<<"Got camera msg!"<<std::endl;
		//Update state
		if(msg->cameraID == "forward"/*I'm not sure what the downward camera id is*/){
			state = PROCESSING;
		}
	}
}

/**
* Tries to find a sign in the image
**/
void HawkVisionSign::doSignSearch(){
	
}

// ######################################################################
/*
void HawkVisionSign::sendExampleMessageOne() {
	// Create Message
	HawkMessages::ExampleMessagePtr msg = new HawkMessages::ExampleMessage;
    msg->name = itsName;
    msg->chatter = "Hello?";
    
    // Send Message
	publish("ExampleMessage", msg);
	
	// Update State
	state = WAITING;
}*/

// ######################################################################
/*
void HawkVisionSign::sendExampleMessageTwo() {
	// Create Message
	HawkMessages::ExampleMessagePtr msg = new HawkMessages::ExampleMessage;
    msg->name = itsName;
    msg->chatter = "O hai! ^_^";
    
    // Send Message
	publish("ExampleMessage", msg);
	
	// Update State
	state = WAITING;
}
*/

// ######################################################################
int main (int argc, char* argv[]) {
	std::cout << "HawkVisionSign: starting..." << std::endl;
	
	HawkVisionSign agent("HawkVisionSign", argc, argv);
	agent.start();
	
	std::cout << "HawkVisionSign: all done!" << std::endl;
}
