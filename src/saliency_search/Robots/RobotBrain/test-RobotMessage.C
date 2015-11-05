#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Ice/SimEventsUtils.H"
#include <Ice/Ice.h>
#include "Ice/RobotSimEvents.ice.H"

int main(int argc, char *argv[]) {

    Ice::CommunicatorPtr ic;
    ic = Ice::initialize(argc, argv);

    ModelManager itsMgr("TestRobotMessageManager");
    itsMgr.parseCommandLine((const int)argc, (const char**)argv, "<ActionMessage=1|GPSMessage=2|GoalMessage=3|TrainImageMessage=4>", 1, -1);

    int messageType = atoi(itsMgr.getExtraArg(0).c_str());
    switch(messageType) {
      case 1:
        {
          float transVel = atof(itsMgr.getExtraArg(1).c_str());
          float rotVel   = atof(itsMgr.getExtraArg(2).c_str());
          LINFO("TransVel = %f, RotVel = %f", transVel, rotVel);

          RobotSimEvents::EventsPrx actionPublisher =
            RobotSimEvents::EventsPrx::uncheckedCast(
                SimEventsUtils::getPublisher(ic, "ActionMessageTopic")
                );

          RobotSimEvents::ActionMessagePtr actionMessage =
            new RobotSimEvents::ActionMessage;

          actionMessage->transVel = transVel;
          actionMessage->rotVel = rotVel;

          actionPublisher->updateMessage(actionMessage);
        }
        break;
      case 2:
        {
          float xPos = atof(itsMgr.getExtraArg(1).c_str());
          float yPos   = atof(itsMgr.getExtraArg(2).c_str());
          float ori   = atof(itsMgr.getExtraArg(3).c_str());
          LINFO("posX = %f, posY = %f, ori = %f", xPos, yPos, ori);

          RobotSimEvents::EventsPrx actionPublisher =
            RobotSimEvents::EventsPrx::uncheckedCast(
                SimEventsUtils::getPublisher(ic, "GPSMessageTopic")
                );

          RobotSimEvents::GPSMessagePtr gpsMessage =
            new RobotSimEvents::GPSMessage;

          gpsMessage->xPos = xPos;
          gpsMessage->yPos = yPos;
          gpsMessage->orientation = ori;


          actionPublisher->updateMessage(gpsMessage);
        }
        break;
      case 3:
        {
          float xPos = itsMgr.getExtraArgAs<float>(1);
          float yPos   = itsMgr.getExtraArgAs<float>(2);
          float ori   = itsMgr.getExtraArgAs<float>(3);
          LINFO("Setting goal posX = %f, posY = %f, ori = %f", xPos, yPos, ori);

          RobotSimEvents::EventsPrx actionPublisher =
            RobotSimEvents::EventsPrx::uncheckedCast(
                SimEventsUtils::getPublisher(ic, "GoalStateMessageTopic")
                );

          RobotSimEvents::GoalStateMessagePtr goalMessage =
            new RobotSimEvents::GoalStateMessage;

          goalMessage->xPos = xPos;
          goalMessage->yPos = yPos;
          goalMessage->orientation = ori;

          actionPublisher->updateMessage(goalMessage);
        }
        break;
      case 4:
        {

        }
        break;

    }

    return 0;
}

