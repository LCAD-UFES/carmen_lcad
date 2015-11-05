// File: HawkVisionDrive.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Kevin, Chris, Justin
// Date: April 2010

#include "Robots/BeoHawk/computer/HawkVisionDrive.H"

// ######################################################################
HawkVisionDrive::HawkVisionDrive(std::string myName, int argc, char* argv[])
: HawkAgent(myName, argc, argv) {
	state = INIT;
}

// ######################################################################
void HawkVisionDrive::registerTopics() {
	registerPublisher("DriveFinder");
	registerSubscription("ControlDriveVision");
	registerSubscription("CameraImage");
}

// ######################################################################
bool HawkVisionDrive::scheduler() {
	if(state == INIT) {
		//sendExampleMessageOne();
		return true;
	}
	else if(state == IDLE) {
		//sendExampleMessageTwo();
		return true;
	}else if(state == PROCESSING){
		doDriveSearch();
		return true;
	}
	
	return false;
}

// ######################################################################
void HawkVisionDrive::catchMessage(const HawkMessages::MessagePtr& hawkMessage, const Ice::Current&) {
	if(hawkMessage->ice_isA("::HawkMessages::ControlDriveVision"))
	{
		HawkMessages::ControlDriveVisionMessage msg = HawkMessages::ControlDriveVisionMessage::dynamicCast(hawkMessage);
		std::cout << "driveVisionOn" << std::endl;
		
		// Update State / Wake Up Agent
		if(msg->driveVisionOn) {
			state = IDLE;
			wakeUp();
		}
	}
	if(hawkMessage->is_A("::HawkMessages::CameraImageMessage") && state != PROCESSING)
	{
		HawkMessages::CameraImageMessage msg = HawkMessages::ControlDriveVisionMessage::dynamicCast(hawkMessage);
		std::cout<<"Got camera msg!"<<std::endl;
		//Update state
		if(msg->cameraID == "down"/*I'm not sure what the downward camera id is*/){
			state = PROCESSING;
		}
	}
}

/**
* Tries to find a flash drive in the image
**/
void HawkVisionDrive::doDriveSearch(){
	
}

// ######################################################################
/*
void HawkVisionDrive::sendExampleMessageOne() {
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
void HawkVisionDrive::sendExampleMessageTwo() {
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
	std::cout << "HawkVisionDrive: starting..." << std::endl;
	
	HawkVisionDrive agent("HawkVisionDrive", argc, argv);
	agent.start();
	
	std::cout << "HawkVisionDrive: all done!" << std::endl;
}
