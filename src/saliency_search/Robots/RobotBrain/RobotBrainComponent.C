#ifndef ROBOTBRAINCOMPONENT_C
#define ROBOTBRAINCOMPONENT_C

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"


const ModelOptionCateg MOC_RobotBrainComponent = {
    MOC_SORTPRI_3, "RobotBrainComponent Related Options" };

const ModelOptionDef OPT_IceIdentity =
{ MODOPT_ARG(std::string), "IceIdentity", &MOC_RobotBrainComponent, OPTEXP_CORE,
    "Unique IceStorm Identity for this RobotBrainComponent",
    "ice-identity", '\0', "<string>", "" };

const ModelOptionDef OPT_IcestormIP =
{ MODOPT_ARG(std::string), "IcestormIP", &MOC_RobotBrainComponent, OPTEXP_CORE,
    "IP Address to the host running the IceStorm server",
    "icestorm-ip", '\0', "<ip address>", "127.0.0.1" };

const ModelOptionDef OPT_ConnectionTimeout =
{ MODOPT_ARG(int), "ConnectionTimeout", &MOC_RobotBrainComponent, OPTEXP_CORE,
    "Timeout time when creating a connection to the ice storm server (in ms)",
    "timeout", '\0', "<int>", "1000" };

RobotBrainComponent::RobotBrainComponent(OptionManager& mgr, const std::string& descrName, const std::string& tagName)
 : ModelComponent(mgr, descrName, tagName),
   itsIceIdentity(&OPT_IceIdentity, this, 0),
   itsIcestormIP(&OPT_IcestormIP, this, 0),
   itsConnectionTimeout(&OPT_ConnectionTimeout, this, 0),
   itsEvolveSleepTime(10000),
   itsTagName(tagName)
{}


void RobotBrainComponent::init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{
  itsIcPtr = ic;
  itsAdapterPtr = adapter;
}

void RobotBrainComponent::start2()
{
  Ice::ObjectPtr objPtr = this;

  //Create an ICE proxy using the --ice-identity command line option if specified
  //  int suffix = 0;
  std::string myIDname;
  if(itsIceIdentity.getVal() != "")
    myIDname = itsIceIdentity.getVal().c_str();
  else
    myIDname = itsTagName;

  //////////////////////////////////////////////////////////////////////////////////////////
  //This currently does not work - need to find a way to auto-increment the ID if one exists
  //already...
  /*Ice::ObjectPtr servant = itsAdapterPtr->find(itsIcPtr->stringToIdentity(myIDname.c_str()));
  while(servant != NULL)
  {
    LINFO("Adapter name already taken: %s", myIDname.c_str());
    suffix++;
    myIDname = itsIceIdentity.getVal() + convertToString(suffix);
    servant = itsAdapterPtr->find(itsIcPtr->stringToIdentity(myIDname.c_str()));
  }
  LINFO("Creating Ice Identity Adapter: %s", myIDname.c_str());
  */
  itsObjectPrx = itsAdapterPtr->add(objPtr, itsIcPtr->stringToIdentity(myIDname.c_str()));


  //Create a topic manager to handle messages for us
  LDEBUG("Connecting to Ice Storm at %s:11111", itsIcestormIP.getVal().c_str());
  char buffer[256];
  sprintf(buffer, "SimEvents/TopicManager:tcp -p 11111 -h %s -t %d", itsIcestormIP.getVal().c_str(), itsConnectionTimeout.getVal());
  Ice::ObjectPrx TopicMgrObjPrx;
  try
  {
    TopicMgrObjPrx = itsIcPtr->stringToProxy(buffer);
    LDEBUG("Got Proxy, Casting...");
    itsTopicManagerPrx = IceStorm::TopicManagerPrx::checkedCast(TopicMgrObjPrx);
    LDEBUG("Cast Complete!");
  }
  catch (const Ice::Exception& e) {
    std::cerr << "Error Connecting to IceStorm: " << e << std::endl;
    exit(1);
  }

  LDEBUG("Got Connection To IceStorm Server");
  //Now that our topic manager is created, we should call a function to actually register all of our initial
  //topics. This function is virtual, and should be overloaded by child classes.
  registerTopics();

  itsAlive = true;
  IceUtil::ThreadPtr thread = this;

  LDEBUG("Starting Thread");

        start3();

  thread->start();
}

void RobotBrainComponent::run()
{
  while(itsAlive)
  {
    evolve();
    usleep(itsEvolveSleepTime);
  }
}


void RobotBrainComponent::stop1()
{
  itsAlive = false;

  std::map<std::string, IceStorm::TopicPrx>::iterator iter;
  for(iter = itsTopicSubscriptions.begin(); iter != itsTopicSubscriptions.end(); ++iter)
  {
    LINFO("%s Unsubscribing From %s", this->tagName().c_str(), iter->first.c_str());
    iter->second->unsubscribe(itsObjectPrx);
  }
}

bool RobotBrainComponent::registerSubscription(const std::string& MessageTopic)
{
  LINFO("Subscribing To Ice Topic: %s", MessageTopic.c_str());
  IceStorm::TopicPrx topic;

  //Create a dummy "Quality of Service" dictionary - we don't touch it, as the default
  //values are acceptable
  IceStorm::QoS qos;

  try
  {
    //Request a handle to the topic from our topic manager
    topic = itsTopicManagerPrx->retrieve(MessageTopic.c_str());

    //Actually subscribe to the requested topic
    topic->subscribeAndGetPublisher(qos, itsObjectPrx);

    //Store our topic in our internal topic map so that we can unsubscribe from it
    //later if we need to clean up
    itsTopicSubscriptions.insert( make_pair(MessageTopic, topic));
    LINFO("Inserted %s into itsTopicSubscriptions", MessageTopic.c_str());
  }
  catch(const IceStorm::NoSuchTopic&)
  {
    LERROR("Message Topic Not Found (%s). Make sure someone has already registered as a publisher.", MessageTopic.c_str() );
    return false;
  }
  catch(const IceStorm::AlreadySubscribed&)
  {
    LERROR("A RobotBrainComponent with the tagname \"%s\" has already subscribed to \"%s\"", this->tagName().c_str(), MessageTopic.c_str() );
    topic->unsubscribe(itsObjectPrx);

    //Request a handle to the topic from our topic manager
    topic = itsTopicManagerPrx->retrieve(MessageTopic.c_str());
    topic->subscribeAndGetPublisher(qos, itsObjectPrx);
    itsTopicSubscriptions.insert( make_pair(MessageTopic, topic));
  }
  return true;
}

void RobotBrainComponent::registerPublisher(const std::string& MessageTopic)
{
  LINFO("Registering as Publisher Of Ice Topic: %s", MessageTopic.c_str());

  IceStorm::TopicPrx topic;
  try
  {
    //Try to get a handle for our topic if it exists
    topic = itsTopicManagerPrx->retrieve(MessageTopic.c_str());
  }
  catch( const IceStorm::NoSuchTopic& )
  {
    //If our topic does not exist, then we create it
    topic = itsTopicManagerPrx->create(MessageTopic.c_str());
  }

  //Request the publisher from the Ice runtime
  Ice::ObjectPrx pub = topic->getPublisher()->ice_oneway();

  //Create our actual publisher proxy
  RobotSimEvents::EventsPrx publisher = RobotSimEvents::EventsPrx::uncheckedCast(pub);

  //Store our topic publisher so that we can access it later when we actually
  //have messages to publish
  itsTopicPublishers.insert( make_pair(MessageTopic, publisher) );
}

bool RobotBrainComponent::publish(const::std::string& MessageTopic, RobotSimEvents::EventMessagePtr msg)
{
  LDEBUG("Publishing Ice Topic: %s", MessageTopic.c_str());
  //Search for the MessageTopic/Publisher pair from our publisher map
  std::map<std::string, RobotSimEvents::EventsPrx>::iterator iter =
    itsTopicPublishers.find(MessageTopic);

  //Make sure we are actually registered as a publisher for the requested topic
  if(iter == itsTopicPublishers.end())
  {
    LERROR("Not currently a registered publisher of \"%s\"! Please call registerPublisher to register", MessageTopic.c_str());
    return false;
  }

  //If we found the publisher in our map, then let's grab it from the iterator
  RobotSimEvents::EventsPrx publisher = iter->second;

  //Publish the message
  publisher->updateMessage(msg);

  return true;
}

#endif

