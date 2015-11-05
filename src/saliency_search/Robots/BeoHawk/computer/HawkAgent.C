// File: HawkAgent.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#ifndef HAWKAGENT_C
#define HAWKAGENT_C

#include "Robots/BeoHawk/computer/HawkAgent.H"

// ######################################################################
HawkAgent::HawkAgent(std::string myName, int argc, char* argv[]) {
	// Help stuff
	helpTitle = "Generic HawkAgent";
	helpDescription = "A generic HawkAgent carries out tasks as a part of a network of agents that communicate through the ICE protocol.";
	helpOptions.push_back("\t--icestorm-ip\t(127.0.0.1) The IP of the IceStorm server.");
	helpOptions.push_back("\t--help\t\tLaunches this help section.");
	
	// Variable stuff
	itsName = myName;
	char * iceStormIP = (char *)"127.0.0.1";
	helpFlag = false;
	killAgent = false;
	firstRun = true;
	
	// Parse command line args
    for(int i = 0; i < argc && !helpFlag; i++) {
    	if(strstr(argv[i], "--help") != NULL) {
    		helpFlag = true;
    	}
    	else if(argv[i][0] == '-' && argv[i][1] == '-' && (i+1) < argc) {
    		// Check for Ice Storm IP
    		if(strstr(argv[i], "--icestorm-ip") != NULL) iceStormIP = argv[i+1];
    		
    		// Load into parameterDefaults
    		std::string name = argv[i];
    		name = name.substr(2, name.size()-2);
    		Parameter p;
    		p.setValue(argv[i+1]);
    		parameterDefaults.insert(std::pair<std::string, Parameter>(name, p));
    	}
    }
    
    if(!helpFlag) {
		// Create Ice communicator
		itsIc = Ice::initialize(argc, argv);

		// Connect Ice communicator to open port
		char buf[255];
		bool connected = false;
		int port = DEFAULT_STARTING_PORT;
		while (!connected) {
			try {
				sprintf(buf, "default -p %i", port);
				itsAdapter = itsIc->createObjectAdapterWithEndpoints(itsName, buf);
				connected = true;
			} catch (Ice::SocketException) {
				port++;
			}
		}

		// Store port
		std::cout << "using port: " << port << std::endl;
		std::stringstream ss;
		ss << port;
		itsName += ":" + ss.str();
	
		// Connect to Ice Storm
		itsObjectPrx = itsAdapter->add((Ice::ObjectPtr) this, itsIc->stringToIdentity(itsName));
		sprintf(buf, "SimEvents/TopicManager:tcp -p 11111 -h %s -t %d", iceStormIP, DEFAULT_TIMEOUT);
		Ice::ObjectPrx TopicMgrObjPrx = itsIc->stringToProxy(buf);
		itsTopicManagerPrx = IceStorm::TopicManagerPrx::checkedCast(TopicMgrObjPrx);
	}
}

// ######################################################################
HawkAgent::~HawkAgent() {
    /*if(itsIc) {
		try {
			std::cout << "waitForShutdown" << std::endl;
		    itsIc->waitForShutdown();
			std::cout << "destroy" << std::endl;
		    itsIc->destroy();
		} catch (const Ice::Exception& e) {}
    }*/
}

// ######################################################################
bool HawkAgent::helpParameter() {
	return helpFlag;
}

// ######################################################################
void HawkAgent::help() {
	std::cout << std::endl << "HELP: " << helpTitle << std::endl;
	std::cout << helpDescription << std::endl;
	
	std::cout << std::endl << "Options:" << std::endl;
	std::sort(helpOptions.begin(), helpOptions.end());
	for(int i = 0; i < (int)helpOptions.size(); i++) {
		std::cout << helpOptions[i] << std::endl;
	}
	std::cout << std::endl;
}

// ######################################################################
int HawkAgent::loadIntParameter(std::string name, int defaultValue) {
	std::map<std::string, Parameter>::iterator it;
	
	it = parameterDefaults.find(name);
	if(it != parameterDefaults.end()) return parameterDefaults[name].getIntValue();
	else return defaultValue;
}

// ######################################################################
bool HawkAgent::loadBoolParameter(std::string name, bool defaultValue) {
	std::map<std::string, HawkAgent::Parameter>::iterator it;
	
	it = parameterDefaults.find(name);
	if(it != parameterDefaults.end()) return parameterDefaults[name].getBoolValue();
	else return defaultValue;
}

// ######################################################################
double HawkAgent::loadDoubleParameter(std::string name, double defaultValue) {
	std::map<std::string, HawkAgent::Parameter>::iterator it;
	
	it = parameterDefaults.find(name);
	if(it != parameterDefaults.end()) return parameterDefaults[name].getDoubleValue();
	else return defaultValue;
}

// ######################################################################
std::string HawkAgent::loadStringParameter(std::string name, std::string defaultValue) {
	std::map<std::string, Parameter>::iterator it;
	
	it = parameterDefaults.find(name);
	if(it != parameterDefaults.end()) return parameterDefaults[name].getStringValue();
	else return defaultValue;
}

// ######################################################################
bool HawkAgent::parameterExists(std::string name) {
	std::map<std::string, Parameter>::iterator it;
	
	it = parameterDefaults.find(name);
	if(it != parameterDefaults.end()) return true;
	else return false;
}

// ######################################################################
void HawkAgent::wakeUp() {
	//std::cout << "unlocking..." << std::endl;
	itsLock.post();
}

// ######################################################################
bool HawkAgent::start() {
	if(helpFlag) {
		help();
		return true;
	}
	else {
		try {
			if(firstRun) {
				itsAdapter->activate();
				registerTopics();
			}
	
			while(!killAgent) {
				if(!firstRun) {
					//std::cout << "locking..." << std::endl;
					itsLock.wait();
				}
		
				while(!killAgent && scheduler());
			
				if(firstRun) firstRun = false;
			}
		
			return true;
		}
		catch(IceUtil::NullHandleException) {
			std::cout << "Error: agent cannot start because an error occured during initialization" << std::endl;
			return false;
		}
	}
}

// ######################################################################
bool HawkAgent::stop() {
	if(!killAgent) {
		killAgent = true;
		return true;
	}
	else return false;
}

// ######################################################################
bool HawkAgent::registerSubscription(std::string MessageTopic) {
	MessageTopic += "Topic";
    IceStorm::TopicPrx topic;
    IceStorm::QoS qos;

    try {
        topic = itsTopicManagerPrx->retrieve(MessageTopic.c_str());

        topic->subscribeAndGetPublisher(qos, itsObjectPrx);

        itsTopicSubscriptions.insert( make_pair(MessageTopic, topic) );
    } catch (const IceStorm::AlreadySubscribed&) {
        topic->unsubscribe(itsObjectPrx);

        topic = itsTopicManagerPrx->retrieve(MessageTopic.c_str());
        topic->subscribeAndGetPublisher(qos, itsObjectPrx);
        itsTopicSubscriptions.insert( make_pair(MessageTopic, topic) );
    }
    return true;
}

// ######################################################################
bool HawkAgent::registerPublisher(std::string MessageTopic) {
	MessageTopic += "Topic";
    IceStorm::TopicPrx topic;

    try {
    	topic = itsTopicManagerPrx->retrieve(MessageTopic.c_str());
    } catch (const IceStorm::NoSuchTopic&) {
    	topic = itsTopicManagerPrx->create(MessageTopic.c_str());
    }

    Ice::ObjectPrx pub = topic->getPublisher()->ice_oneway();
    HawkMessages::MessageAgentPrx publisher = HawkMessages::MessageAgentPrx::uncheckedCast(pub);
    itsTopicPublishers.insert( make_pair(MessageTopic, publisher) );

    return true;
}

// ######################################################################
bool HawkAgent::publish(std::string MessageTopic, HawkMessages::MessagePtr msg) {
	MessageTopic += "Topic";
    std::map<std::string, HawkMessages::MessageAgentPrx>::iterator iter = itsTopicPublishers.find(MessageTopic);

    if (iter == itsTopicPublishers.end()) return false;

    HawkMessages::MessageAgentPrx publisher = iter->second;

    publisher->catchMessage(msg);

    return true;
}

#endif
