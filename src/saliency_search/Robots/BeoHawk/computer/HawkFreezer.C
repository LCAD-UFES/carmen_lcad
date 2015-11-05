// File: HawkFreezer.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#include "Robots/BeoHawk/computer/HawkFreezer.H"

// ######################################################################
HawkFreezer::HawkFreezer(std::string myName, int argc, char* argv[])
: HawkAgent(myName, argc, argv) {
	// Help section
	helpTitle = "HawkFreezer";
	helpDescription = "Stores ICE messages for future playback. Also plays them back.";
	helpOptions.push_back("\t--mode\t\t(record) Should be 'record' or 'playback'.");
	helpOptions.push_back("\t--file\t\t(default.freezer) File to read or write from.");
	helpOptions.push_back("\t--duration\t(30) Duration to playback or record ICE messages.");
	helpOptions.push_back("\t--frequency\t(0) Maximum messages per second to play or record. Set to 0 to ignore this setting.");
	
	// Parameters
	duration = loadDoubleParameter("duration", 30);
	frequency = loadDoubleParameter("frequency", 0);
	std::string modeString = loadStringParameter("mode", "record");
	file = loadStringParameter("file", "default.freezer");
	timer = Timer(1000000);
	
	// Set skip period
	if(frequency == 0) period = 0;
	else period = 1/frequency;
	lastMessageSeconds = timer.getSecs() - period;
	
	// Set state
	if(modeString == "record") {
		std::cout << "starting to record to " << file << " for " << duration << " seconds" << std::endl;
		state = RECORD;
	}
	else {
		std::cout << "set to playback from " << file << std::endl;
		state = LOAD_PLAYBACK;
	}
}

// ######################################################################
void HawkFreezer::registerTopics() {
	registerPublisher("SensorDataMessage");
	registerSubscription("SensorDataMessage");
}

// ######################################################################
bool HawkFreezer::scheduler() {
	if(state == RECORD) {
		if(timer.getSecs() < duration) {
			usleep(1000000); // this does not respect non-integer durations, who cares
			std::cout << "captured " << messages.size() << " messages..." << std::endl;
			return true;
		}
		else {
			std::cout << "record duration reached, writing to file..." << std::endl;
			saveMessages();
		}
	}
	else if(state == LOAD_PLAYBACK) {
		loadMessages();
		return true;
	}
	else if(state == PLAYBACK) {
		if(messages.size() > 0 && timer.getSecs() < duration) {
			std::cout << messages.size() << " messages left to play..." << std::endl;
			playNextMessages();
			return true;
		}
		else {
			std::cout << "no more messages to play back (or duration elapsed)..." << std::endl;
		}
	}
	
	stop();
	return false;
}

// ######################################################################
void HawkFreezer::catchMessage(const HawkMessages::MessagePtr& hawkMessage, const Ice::Current&) {
	if(state == RECORD && hawkMessage->ice_isA("::HawkMessages::SensorDataMessage") && timer.getSecs() > (lastMessageSeconds + period))
	{
		// Save message to storage queue
		Message msg;
		msg.sensorData = HawkMessages::SensorDataMessagePtr::dynamicCast(hawkMessage);
		msg.timestamp = timer.getSecs();
		messages.push_back(msg);
		
		// Reset
		lastMessageSeconds = timer.getSecs();
	}
}

// ######################################################################
void HawkFreezer::playNextMessages() {
	// Play messages
	while(messages.size() > 0 && messages.front().timestamp <= timer.getSecs()) {
		publish("SensorDataMessage", messages.front().sensorData);
		lastMessageSeconds = messages.front().timestamp;
		messages.erase(messages.begin());
	}
	
	// Delete future skipped messages
	while(messages.size() > 0 && messages.front().timestamp < (lastMessageSeconds+period)) {
		messages.erase(messages.begin());
	}
	
	// Sleep until next message
	if(messages.size() > 0)
		usleep((int)((messages.front().timestamp - timer.getSecs())*1000000));
}

// ######################################################################
void HawkFreezer::loadMessages() {
	std::ifstream fileStream;
	fileStream.open(file.c_str());
	if(fileStream) {
		// Load number of messages
		int numberOfMessages;
		fileStream >> numberOfMessages;
		
		// Load each message
		for(int i = 0; i < numberOfMessages; i++) {
			Message msg;
			
			// Timestamp
			fileStream >> msg.timestamp;
			
			// Attempted move
			HawkMessages::Pose attemptedMove;
			fileStream >> attemptedMove.x;
			fileStream >> attemptedMove.y;
			fileStream >> attemptedMove.z;
			fileStream >> attemptedMove.theta;
			
			// Scanner data
			double angularResolution;
			fileStream >> angularResolution;
			int scannerDataSize;
			fileStream >> scannerDataSize;
			HawkMessages::LongSeq scannerData;
			long temp;
			for(int j = 0; j < scannerDataSize; j++) {
				fileStream >> temp;
				scannerData.push_back(temp);
			}
			
			// Add to message vector
			msg.sensorData = new HawkMessages::SensorDataMessage;
			msg.sensorData->attemptedMove = attemptedMove;
			msg.sensorData->angularResolution = angularResolution;
			msg.sensorData->scannerData = scannerData;
			messages.push_back(msg);
		}
		
		std::cout << "playback file loaded, now starting playback..." << std::endl;
		state = PLAYBACK;
	}
	else {
		std::cout << "Error: file could not be opened." << std::endl;
		state = INIT_FAIL;
	}
	fileStream.close();
}

// ######################################################################
void HawkFreezer::saveMessages() {
	std::ofstream fileStream;
	fileStream.open(file.c_str());
	if(fileStream) {
		// Write number of messages
		fileStream << messages.size() << std::endl;
		
		// Write each message
		for(int i = 0; i < (int)messages.size(); i++) {
			// Timestamp
			fileStream << messages[i].timestamp << std::endl;
			
			// Attempted move
			fileStream << messages[i].sensorData->attemptedMove.x << " ";
			fileStream << messages[i].sensorData->attemptedMove.y << " ";
			fileStream << messages[i].sensorData->attemptedMove.z << " ";
			fileStream << messages[i].sensorData->attemptedMove.theta << std::endl;
			
			// Scanner data
			fileStream << messages[i].sensorData->angularResolution << " ";
			fileStream << messages[i].sensorData->scannerData.size();
			for(int j = 0; j < (int)messages[i].sensorData->scannerData.size(); j++) {
				fileStream << " " << messages[i].sensorData->scannerData[j];
			}
			fileStream << std::endl;
		}
	}
	else {
		std::cout << "Error: file could not be opened. That sucks." << std::endl;
	}
	fileStream.close();
}

// ######################################################################
int main (int argc, char* argv[]) {
	std::cout << "HawkFreezer: starting..." << std::endl;
	
	HawkFreezer agent("HawkFreezer", argc, argv);
	agent.start();
	
	std::cout << "HawkFreezer: all done!" << std::endl;
}
