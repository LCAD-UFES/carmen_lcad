#ifndef SIMPLEROBOTBRAINCOMPONENT_H_
#define SIMPLEROBOTBRAINCOMPONENT_H_

#include <string>
#include <IceE/IceE.h>
#include "miniIce/SimpleRobotSimEvents.h"
#include "miniIce/IceStorm.h"


#define DEFAULT_STARTING_PORT                2121
#define DEFAULT_TIMEOUT                                1000



class SimpleRobotBrainComponent : public RobotSimEvents::Events
{
public:
        SimpleRobotBrainComponent(char * iceStormIP, char * myName, Ice::CommunicatorPtr ic);
        virtual ~SimpleRobotBrainComponent();

        bool registerSubscription(const std::string& MessageTopic);
        bool registerPublisher(const std::string& MessageTopic);
        bool publish(const std::string& MessageTopic, RobotSimEvents::EventMessagePtr msg);
        void start();


        //This is what you must implement
        // There is no threading as these are intended to be SIMPLE
        // so you must put a while(true) loop inside this method to
        // keep executing you.
        // There is no sleeping like in RobotBrainComponent, if you
        // want that feature then implement it in your inherited
        // object, as not all of us want it on the robot.
        // That's about it :)
        virtual void run() = 0;

        virtual void registerTopics() = 0;

        virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                const Ice::Current&) {};


private:
        Ice::CommunicatorPtr                itsIc;
        Ice::ObjectAdapterPtr                itsAdapter;
        Ice::ObjectPrx                                itsObjectPrx;
        IceStorm::TopicManagerPrx        itsTopicManagerPrx;

        std::map<std::string, IceStorm::TopicPrx> itsTopicSubscriptions;
        std::map<std::string, RobotSimEvents::EventsPrx> itsTopicPublishers;

};

#endif /*SIMPLEROBOTBRAINCOMPONENT_H_*/
