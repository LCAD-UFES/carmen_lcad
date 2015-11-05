// File: HawkAgent.C
// Author: Josh Villbrandt <josh.villbrandt@usc.edu>
// Date: April 2010

#ifndef HAWKAGENT_H
#define HAWKAGENT_H

#include <string>
#include <sstream>
#include <map>
#include <algorithm>
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include "Util/Semaphore.H"
#include "Robots/BeoHawk/core/HawkMessages.ice.H"
#include "Ice/ImageIce.ice.H" // this is a hack to force ImageIce to link correctly for HawkMessages.ice

#define DEFAULT_STARTING_PORT	4295
#define DEFAULT_TIMEOUT			1000

class HawkAgent : public HawkMessages::MessageAgent
{
	public:
        HawkAgent(std::string myName, int argc, char* argv[]);
        ~HawkAgent();
        bool start();
        bool stop();
        void wakeUp();

        // Implement these
        virtual bool scheduler() { return false; };
        virtual void registerTopics() {};
        virtual void catchMessage(const HawkMessages::MessagePtr& msg, const Ice::Current&) {};

	protected:
        // Use these
		std::string itsName;
        bool registerSubscription(std::string MessageTopic);
        bool registerPublisher(std::string MessageTopic);
        bool publish(std::string MessageTopic, HawkMessages::MessagePtr msg);
        
		// Parameter stuff
		class Parameter {
			public:
				void setValue(std::string set) {
					value = set;
				}
				int getIntValue() {
					std::istringstream buffer(value);
					int temp;
					buffer >> temp;
					return temp;
				}
				bool getBoolValue() {
					if(value == "1" || value == "TRUE" || value == "true")
						return true;
					else return false;
				}
				double getDoubleValue() {
					std::istringstream buffer(value);
					double temp;
					buffer >> temp;
					return temp;
				}
				std::string getStringValue() {
					return value;
				}
			private:
				std::string value;
		};
		std::map<std::string, Parameter> parameterDefaults;
        int loadIntParameter(std::string name, int defaultValue);
        bool loadBoolParameter(std::string name, bool defaultValue);
        double loadDoubleParameter(std::string name, double defaultValue);
        std::string loadStringParameter(std::string name, std::string defaultValue);
        bool parameterExists(std::string name);
        
        // Help stuff
        std::string helpTitle;
        std::string helpDescription;
        std::vector<std::string> helpOptions;
        bool helpParameter();
        
    private:
		bool killAgent;
		bool firstRun;
		bool helpFlag;
        void help();
		Semaphore itsLock;
        Ice::CommunicatorPtr itsIc;
        Ice::ObjectAdapterPtr itsAdapter;
        Ice::ObjectPrx itsObjectPrx;
        IceStorm::TopicManagerPrx itsTopicManagerPrx;
        std::map<std::string, IceStorm::TopicPrx> itsTopicSubscriptions;
        std::map<std::string, HawkMessages::MessageAgentPrx> itsTopicPublishers;
};

#endif
