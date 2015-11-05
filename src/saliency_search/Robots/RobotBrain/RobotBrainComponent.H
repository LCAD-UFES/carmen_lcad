#ifndef ROBOTBRAINCOMPONENT_H
#define ROBOTBRAINCOMPONENT_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelManager.H"
#include "Component/OptionManager.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"

#include <Ice/Ice.h>
#include <IceUtil/Thread.h>
#include <IceStorm/IceStorm.h>

#include <string>
#include <map>

class RobotBrainComponent : public ModelComponent, public RobotSimEvents::Events, public IceUtil::Thread
{
  public:
    //! Robot Brain Component Constructor: Simply calls the ModelComponent constructor.
    //! Make sure that you include this constructor in your inherited class' initialization list
    //! Also, make sure that you use a unique tagName for each object that you want to add
    RobotBrainComponent(OptionManager& mgr,
        const std::string& descrName = "GenericRobotBrainComponent",
        const std::string& tagName = "GenericRobotBrainComponent");

    virtual ~RobotBrainComponent(){};

    //! Init: Sets up all of the Ice runtime details, and ends with a call to registerTopics
    void init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter);

    //! Register Subscription: Register this object as a subscriber to a specified topic
    bool registerSubscription(const std::string& MessageTopic);

    //! Register Publisher: Register this object as a publisher of a specified topic
    void registerPublisher(const std::string& MessageTopic);

    //! Publish: Send out a message to all subscribers
    bool publish(const::std::string& MessageTopic, RobotSimEvents::EventMessagePtr msg);

    //! Start2: Overload of the ModelComponent start2 - don't mess with this
    void start2();

    //! Stop1: Overload of the ModelComponent stop1 - don't mess with this
    void stop1();

    //! Run: This function gets forked into a new thread once the ModelManager is started. By default, it calls the evolve function.
    //! It will likely become non-virtual in the future, so please put all run-time functionality into evolve
    virtual void run();

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //
                // USERS: Below are the functions you should overload to make your own RobotBrainComponents.
                //
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //! Register Topics: This function is called at the end of init, and is virtual so that is can be overloaded
    //!                                                                  by chid classes. In your overloaded version you should put all of the registerSubsciption
                //!                                                             and registerPublisher calls that you would like to happen automatically after init.
    virtual void registerTopics() {};

    //! Evolve: Gets called by run in a while(1) loop. Anything that you want to happen continuously should go in here
    virtual void evolve() {};

    //! Update Message: Gets called by the Ice runtime when a publisher of a message that your object subscribes
                //!                                                                        to publishes a message.
    virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
        const Ice::Current&) {};

                //! start3: Gets called immediately after start2, which is after all of the subcomponents have been started.
                //!         If you have any code that needs to run only once at the beginning before the general run/evolve
                //!                                        loop, but after the constructor, then you can put it here.
                virtual void start3() {};

  protected:
                //A unique name for this instance of RobotBrainComponent
    OModelParam<std::string>              itsIceIdentity;
                //The IP of the IceStorm server
    OModelParam<std::string>              itsIcestormIP;
                //Time to wait before giving up trying to connect to IceStorm
    OModelParam<int>                      itsConnectionTimeout;
                //The amount of microseconds that run() will wait between subsequent evolve() calls.
    int itsEvolveSleepTime;

  private:
                //Ice connection details
    Ice::CommunicatorPtr                  itsIcPtr;
    Ice::ObjectAdapterPtr                 itsAdapterPtr;
    Ice::ObjectPrx                        itsObjectPrx;
    IceStorm::TopicManagerPrx             itsTopicManagerPrx;

    //A map to hold all of our topic subscriptions
    std::map<std::string, IceStorm::TopicPrx>  itsTopicSubscriptions;

    //A map to hold all of our topic publishers
    std::map<std::string, RobotSimEvents::EventsPrx>  itsTopicPublishers;

    bool itsAlive;

  std::string itsTagName;


};

#endif

