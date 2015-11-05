#include "Robots/SeaBeeIII/SeaBeeInjector.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#ifndef SeaBeeInjector_C
#define SeaBeeInjector_C

// ######################################################################
SeaBeeInjector::SeaBeeInjector(int id, OptionManager& mgr,
                const std::string& descrName, const std::string& tagName) :
        RobotBrainComponent(mgr, descrName, tagName)
{

}

// ######################################################################
SeaBeeInjector::~SeaBeeInjector()
{

}

void SeaBeeInjector::registerTopics()
{
        registerPublisher("SeaBeePositionMessageTopic");
        registerPublisher("SeaBeeStateConditionMessageTopic");
}

void SeaBeeInjector::evolve()
{
        int input;

        RobotSimEvents::SeaBeeStateConditionMessagePtr msg;

        // This is a menu that will appear every evolve cycle
        printf("\n\n::::::SeaBeeInjector Menu:::::\n");
        printf("What Type of Message Do You Want To Send SeaBee?\n");
        printf("\t0) Start Over From Beginning\n");
        printf("\t1) Gate Finished\n");
        printf("\t2) Flare Finished\n");
        printf("\t3) Barbwire Finished\n");
        printf("\t4) Bombing Run Finished\n");
        printf("\t5) Briefcase Found\n");
        cout << "\nMock Message to Send SeaBee: ";
        cin >> input;

        switch(input)
        {
                case 0:
                        LINFO("\n\n1: Starting Over...\n\n");
                        msg =  new RobotSimEvents::SeaBeeStateConditionMessage;
                        send_message(msg,1,0,0,0,0,0,0,0,0,0,0);
                        publish("SeaBeeStateConditionMessageTopic",msg);
                        break;
                case 1:
                        //this is the Condition for Gate Finished
                        LINFO("\n\n1: Sending Gate Finished Message...\n\n");
                        msg =  new RobotSimEvents::SeaBeeStateConditionMessage;
                        send_message(msg,0,0,0,1,0,0,0,0,0,0,0);
                        publish("SeaBeeStateConditionMessageTopic",msg);
                        break;
                case 2:
                        //this is our Flare Finished Condition
                        LINFO("\n\n2: Sending Flare Done Message...\n\n");
                        msg =  new RobotSimEvents::SeaBeeStateConditionMessage;
                        send_message(msg,0,0,0,0,0,1,0,0,0,0,0);
                        publish("SeaBeeStateConditionMessageTopic",msg);
                        break;
                case 3:
                        //this is our message for Barbwire Finished
                        LINFO("\n\n3: Sending Barbwire Finished Message...\n\n");
                        msg =  new RobotSimEvents::SeaBeeStateConditionMessage;
                        send_message(msg,0,0,0,1,0,0,0,0,0,0,0);
                        publish("SeaBeeStateConditionMessageTopic",msg);
                        break;
                case 4:
                        //this is our Bombing Done Condition
                        LINFO("\n\n4: Sending Bombing Run Done Message...\n\n");
                        msg =  new RobotSimEvents::SeaBeeStateConditionMessage;
                        send_message(msg,0,0,0,0,0,0,0,0,0,1,0);
                        publish("SeaBeeStateConditionMessageTopic",msg);
                        break;
                case 5:
                        //this is message for Briefcase Found
                        LINFO("\n\n5: Sending Briefcase Found Message...\n\n");
                        msg =  new RobotSimEvents::SeaBeeStateConditionMessage;
                        send_message(msg,0,0,0,0,0,0,0,0,0,0,1);
                        publish("SeaBeeStateConditionMessageTopic",msg);
                        break;
                                default:
                        break;

        }
}

// ######################################################################
void SeaBeeInjector::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                const Ice::Current&)
{
        if(eMsg->ice_isA("::RobotSimEvents::XBox360RemoteControlMessage"))
        {
                RobotSimEvents::JoyStickControlMessagePtr msg = RobotSimEvents::JoyStickControlMessagePtr::dynamicCast(eMsg);

                LINFO("Got A XBox Message! Axis: %d AxisVal: %d Button: %d ButtonVal: %d", msg->axis, msg->axisVal, msg->button, msg->butVal);

        }
}

void SeaBeeInjector::send_message(RobotSimEvents::SeaBeeStateConditionMessagePtr msg, int s, int path, int a, int b, int c, int d, int e, int f, int g, int h, int i)
{
        msg->StartOver = s;
        msg->PathFollowFinished = path;
        msg->InitDone = a;
        msg->GateDone = b;
        msg->ContourFoundFlare = c;
        msg->FlareDone = d;
        msg->ContourFoundBarbwire = e;
        msg->BarbwireDone = f;
        msg->ContourFoundBoxes = g;
        msg->BombingRunDone = h;
        msg->BriefcaseFound = i;

        msg->TimeUp = 0;

}

#endif
