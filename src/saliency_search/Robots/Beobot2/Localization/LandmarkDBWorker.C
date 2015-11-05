#include "Robots/Beobot2/Localization/LandmarkDBWorker.H"

#ifndef LANDMARKDBWORKERI_C
#define LANDMARKDBWORKERI_C


// ######################################################################
LandmarkDBWorkerI::LandmarkDBWorkerI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName)
{
}

// ######################################################################
LandmarkDBWorkerI::~LandmarkDBWorkerI()
{

}

void LandmarkDBWorkerI::registerTopics()
{
  this->registerSubscription("LandmarkSearchQueueMessageTopic");
}

void LandmarkDBWorkerI::evolve()
{

}

// ######################################################################
void LandmarkDBWorkerI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  //Get a retina message
  if(eMsg->ice_isA("::BeobotEvents::LandmarkSearchQueueMessage"))
  {
    BeobotEvents::LandmarkSearchQueueMessagePtr LandmarkQueueMsg = BeobotEvents::LandmarkSearchQueueMessagePtr::dynamicCast(eMsg);

    //Get the current request ID
    int currRequestID = LandmarkQueueMsg->RequestID;

    LINFO("Got A New Landmark Queue Message with Request ID = %d", currRequestID);

  }
}

#endif
