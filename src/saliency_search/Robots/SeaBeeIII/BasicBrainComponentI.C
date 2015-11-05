#include "Robots/SeaBeeIII/BasicBrainComponentI.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#ifndef BASICBRAINCOMPONENTI_C
#define BASICBRAINCOMPONENTI_C

// ######################################################################
BasicBrainComponentI::BasicBrainComponentI(int id, OptionManager& mgr,
                const std::string& descrName, const std::string& tagName) :
        RobotBrainComponent(mgr, descrName, tagName)
{

}

// ######################################################################
BasicBrainComponentI::~BasicBrainComponentI()
{

}

// ######################################################################
void BasicBrainComponentI::registerTopics()
{
  /* Example of how to register as a publisher/subscriber to topics */

  registerPublisher("ChatMessage");
  registerSubscription("ChatMessage");
}

// ######################################################################
void BasicBrainComponentI::evolve()
{
        RobotSimEvents::ChatMessagePtr msg;
        msg = new RobotSimEvents::ChatMessage;

        string input;
        cin >> input;

        msg->text = input;
        msg->from = "John";



        publish("ChatMessage",msg);



  // evolve() loops forever. Do stuff here...
}

// ######################################################################
void BasicBrainComponentI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                const Ice::Current&)
{
  /* Example of how to parse received message. */

  if(eMsg->ice_isA("::RobotSimEvents::ChatMessage"))
    {
      RobotSimEvents::ChatMessagePtr msg = RobotSimEvents::ChatMessagePtr::dynamicCast(eMsg);

      LINFO("%s: %s", msg->from.c_str(), msg->text.c_str());

      }
}

#endif
