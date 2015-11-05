// File: HawkScanner.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include "Robots/BeoHawk/computer/HawkScanner.H"

// ######################################################################
HawkScanner::HawkScanner(std::string myName, int argc, char* argv[])
: HawkAgent(myName, argc, argv) {
	// Help section
	helpTitle = "HawkScanner";
	helpDescription = "Interface between the URG-04LX-UG01 laser scanner and ICE.";
	helpOptions.push_back("\t--refreshPeriod\t(100000) Microseconds between each scanner message.");
	
	// Parameters
	refreshPeriod = loadIntParameter("refreshPeriod", 100000);
	
	if(!helpParameter()) {
		// Setup to laser scanner
		if(!urg.connect(SCANNER_ADDRESS)) {
			printf("UrgCtrl::connect: %s\n", urg.what());
			exit(1);
		}
	
		// Print useful parameters
		std::cout << "frontIndex: " << urg.rad2index(0.0) << std::endl;
		std::cout << "halfRangeRadians: " << urg.index2rad(0) << std::endl;
		std::cout << "maxScanLines: " << urg.maxScanLines() << std::endl;
	
		// Setup timer
		timer = Timer(1000000);
	}
}

// ######################################################################
void HawkScanner::registerTopics() {
	registerPublisher("SensorDataMessage");
	//registerSubscription("");
}

// ######################################################################
bool HawkScanner::scheduler() {
	// Send ICE message
	sendScannerMessage();
	
	// Ensure consistent output
	int sleepTime = refreshPeriod - timer.getSecs()*1000000;
	if(sleepTime > 0) usleep(sleepTime);
	//std::cout << timer.getSecs()*1000000 << " " << sleepTime << std::endl;
	timer.reset();
	
	// Immediatly rerun scheduler
	return true;
}

// ######################################################################
void HawkScanner::catchMessage(const HawkMessages::MessagePtr& hawkMessage, const Ice::Current&) {
	
}

// ######################################################################
void HawkScanner::sendScannerMessage() {
    // Get laser data
    long timestamp = 0;
    std::vector<long> data;
    urg.capture(data, &timestamp);

	// Convert to LongSeq (and trim bad edge)
	int trimEdgeOffset = 2 * urg.rad2index(0.0) - urg.maxScanLines();
	HawkMessages::LongSeq scannerData;
	for(int i = 0; i < (int)data.size(); i++) {
		if(i >= trimEdgeOffset) scannerData.push_back(data.at(i));
	}

	// Create fake attemptedMove
	HawkMessages::Pose attemptedMove;
	attemptedMove.x = 0;
	attemptedMove.y = 0;
	attemptedMove.z = 0;
	attemptedMove.theta = 0;
    
    // Create message
	HawkMessages::SensorDataMessagePtr msg = new HawkMessages::SensorDataMessage;
	msg->angularResolution = SCANNER_RESOLUTION;
	msg->scannerData = scannerData;
	msg->attemptedMove = attemptedMove;
    
    // Send message
	publish("SensorDataMessage", msg);
}

// ######################################################################
int main (int argc, char* argv[]) {
	std::cout << "HawkScanner: starting..." << std::endl;
	
	HawkScanner agent("HawkScanner", argc, argv);
	agent.start();
	
	std::cout << "HawkScanner: all done!" << std::endl;
}
