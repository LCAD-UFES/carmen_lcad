#include "SimpleRobotBrainComponent.H"

SimpleRobotBrainComponent::SimpleRobotBrainComponent(char * iceStormIP,
 char * myName, Ice::CommunicatorPtr ic) {

         //initialize the communicator (this is auto in normal ice):
         itsIc = ic;

         char buf[255];
         bool connected = false;
         int port = DEFAULT_STARTING_PORT;

         while (!connected) {
                try {
                        sprintf(buf, "default -p %i", port);
                        itsAdapter = itsIc->createObjectAdapterWithEndpoints(myName, buf);
                        connected = true;
                } catch (Ice::SocketException) {
                        port++;
                }
         }

         itsObjectPrx = itsAdapter->add((Ice::ObjectPtr) this,
                                                                         itsIc->stringToIdentity(myName));

         sprintf(buf, "SimEvents/TopicManager:tcp -p 11111 -h %s -t %d",
                 iceStormIP, DEFAULT_TIMEOUT);
         Ice::ObjectPrx TopicMgrObjPrx = itsIc->stringToProxy(buf);
         itsTopicManagerPrx = IceStorm::TopicManagerPrx::checkedCast(TopicMgrObjPrx);
}

SimpleRobotBrainComponent::~SimpleRobotBrainComponent(){

        try {
                itsIc->waitForShutdown();
                itsIc->destroy();
        } catch (const Ice::Exception& e) {}
}

void SimpleRobotBrainComponent::start() {
        itsAdapter->activate();
}

bool SimpleRobotBrainComponent::registerSubscription(const std::string& MessageTopic) {

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

bool SimpleRobotBrainComponent::registerPublisher(const std::string& MessageTopic) {

        IceStorm::TopicPrx topic;

        try {
                topic = itsTopicManagerPrx->retrieve(MessageTopic.c_str());
        } catch (const IceStorm::NoSuchTopic&) {
                topic = itsTopicManagerPrx->create(MessageTopic.c_str());
        }

        Ice::ObjectPrx pub = topic->getPublisher()->ice_oneway();
        RobotSimEvents::EventsPrx publisher = RobotSimEvents::EventsPrx::uncheckedCast(pub);
        itsTopicPublishers.insert( make_pair(MessageTopic, publisher) );

        return true;
}

bool SimpleRobotBrainComponent::publish (const std::string& MessageTopic,
                RobotSimEvents::EventMessagePtr msg) {

        std::map<std::string, RobotSimEvents::EventsPrx>::iterator iter =
                itsTopicPublishers.find(MessageTopic);

        if (iter == itsTopicPublishers.end())
                return false;

        RobotSimEvents::EventsPrx publisher = iter->second;

        publisher->updateMessage(msg);

        return true;
}

