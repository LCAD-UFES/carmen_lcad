#include <Ice/Ice.h>

#include "Component/ModelComponent.H"
#include "Component/ModelManager.H"
#include "Robots/Scorbot/Scorbot.ice.H"
#include "Robots/Scorbot/ScorbotInterface.H"

#include <signal.h>


// ######################################################################
const ModelOptionCateg MOC_ScorbotServer = 
{ MOC_SORTPRI_3, "Scorbot Server Related Options" };

const ModelOptionDef OPT_Port = 
{ MODOPT_ARG(std::string), "Port", &MOC_ScorbotServer, OPTEXP_CORE,
	"Scorbot Server Port Number",
	"scorbot-port", '\0', "", "10000" };



class ScorbotI: public ScorbotIce::Scorbot, public ModelComponent
{
	public:
		ScorbotI(OptionManager& mgr,
				const std::string& descrName = "ScorbotServer",
				const std::string& tagName   = "ScorbotServer") :
			ModelComponent(mgr, descrName, tagName),
			itsScorbotInterface(new ScorbotInterface(mgr)), 
			itsPort(&OPT_Port, this, 0)
		{ 
			addSubComponent(itsScorbotInterface);

			JointType2Joint_t[ScorbotIce::Base]     = ScorbotInterface::Base;
			JointType2Joint_t[ScorbotIce::Shoulder] = ScorbotInterface::Shoulder;
			JointType2Joint_t[ScorbotIce::Elbow]    = ScorbotInterface::Elbow;
			JointType2Joint_t[ScorbotIce::Wrist1]   = ScorbotInterface::Wrist1;
			JointType2Joint_t[ScorbotIce::Wrist2]   = ScorbotInterface::Wrist2;
			JointType2Joint_t[ScorbotIce::Gripper]  = ScorbotInterface::Gripper;
			JointType2Joint_t[ScorbotIce::Slider]   = ScorbotInterface::Slider;

			Joint_t2JointType[ScorbotInterface::Base]     = ScorbotIce::Base;
			Joint_t2JointType[ScorbotInterface::Shoulder] = ScorbotIce::Shoulder;
			Joint_t2JointType[ScorbotInterface::Elbow]    = ScorbotIce::Elbow;
			Joint_t2JointType[ScorbotInterface::Wrist1]   = ScorbotIce::Wrist1;
			Joint_t2JointType[ScorbotInterface::Wrist2]   = ScorbotIce::Wrist2;
			Joint_t2JointType[ScorbotInterface::Gripper]  = ScorbotIce::Gripper;
			Joint_t2JointType[ScorbotInterface::Slider]   = ScorbotIce::Slider;
		}

		// ######################################################################
		ScorbotInterface::encoderVals_t encoderValsType2encoderVals_t(ScorbotIce::encoderValsType encodersT)
		{
			ScorbotInterface::encoderVals_t encoders_t;
			ScorbotIce::encoderValsType::iterator encIt;
			for(encIt = encodersT.begin(); encIt != encodersT.end(); ++encIt)
				encoders_t[JointType2Joint_t[encIt->first]] = encIt->second;
			return encoders_t;
		}

		// ######################################################################
		ScorbotIce::encoderValsType encoderVals_t2encoderValsType(ScorbotInterface::encoderVals_t encoders_t)
		{
			ScorbotIce::encoderValsType encodersT;
			ScorbotInterface::encoderVals_t::iterator encIt;
			for(encIt = encoders_t.begin(); encIt != encoders_t.end(); ++encIt)
				encodersT[Joint_t2JointType[encIt->first]] = encIt->second;
			return encodersT;
		}

		// ######################################################################
		ScorbotIce::pwmValsType pwmVals_t2pwmValsType(ScorbotInterface::pwmVals_t pwms_t)
		{
			ScorbotIce::pwmValsType pwmsT;
			ScorbotInterface::pwmVals_t::iterator encIt;
			for(encIt = pwms_t.begin(); encIt != pwms_t.end(); ++encIt)
				pwmsT[Joint_t2JointType[encIt->first]] = encIt->second;
			return pwmsT;
		}

		// ######################################################################
		void init()
		{
		}

		// ######################################################################
		//! Set the joint to move to the given encoder position in the given amount of time
		void setJoint(ScorbotIce::JointType joint, int encoderPos, int timeMS, const Ice::Current&)
		{	itsScorbotInterface->setJoint(JointType2Joint_t[joint], encoderPos, timeMS); }

		// ######################################################################
		//! A convenience function to set multiple joint positions
		/*! This function just loops over all positions in pos and sets them using setJointPos() !*/
		void setJoints(const ScorbotIce::encoderValsType &pos, int timeMS, const Ice::Current&)
		{ itsScorbotInterface->setJoints(encoderValsType2encoderVals_t(pos), timeMS);	}

		// ######################################################################
		//! Get the position of a joint
		int getEncoder(ScorbotIce::JointType joint, const Ice::Current&)
		{ return itsScorbotInterface->getEncoder(JointType2Joint_t[joint]);	}

		// ######################################################################
		//! Get the position of all joints
		ScorbotIce::encoderValsType getEncoders(const Ice::Current&)
		{ return encoderVals_t2encoderValsType(itsScorbotInterface->getEncoders());	}

		// ######################################################################
		//! Turn on/off the motors at the Scorpower box
		void setEnabled(bool enabled, const Ice::Current&)
		{	itsScorbotInterface->setEnabled(enabled);	}

		// ######################################################################
		//! Reset all encoders to 0, and set the desired position of all joints to 0
		void resetEncoders(const Ice::Current&)
		{ itsScorbotInterface->resetEncoders();	}

		// ######################################################################
		//! Get the current PWM value of a joint
		float getPWM(ScorbotIce::JointType joint, const Ice::Current&)
		{	return itsScorbotInterface->getPWM(JointType2Joint_t[joint]);	}

		// ######################################################################
		//! Get the current PWM values of all joints
		ScorbotIce::pwmValsType getPWMs(const Ice::Current&)
		{	return pwmVals_t2pwmValsType(itsScorbotInterface->getPWMs());	}

		// ######################################################################
		//! Set the PID control params for a joint
		void setControlParams(ScorbotIce::JointType joint, 
				float pGain, float iGain, float dGain, float maxI, float maxPWM, float pwmOffset, const Ice::Current&)
		{	itsScorbotInterface->setControlParams(JointType2Joint_t[joint], pGain, iGain, dGain, maxI, maxPWM, pwmOffset); }

		// ######################################################################
		//! Get the PID values for a joint from the microcontroller
		void getPIDVals(ScorbotIce::JointType joint, float &pGain, float &iGain, float &dGain, float &maxI, float &maxPWM, float &pwmOffset, const Ice::Current&)
		{	itsScorbotInterface->getPIDVals(JointType2Joint_t[joint], pGain, iGain, dGain, maxI, maxPWM, pwmOffset); }

		// ######################################################################
		//! Get the current target position and velocity
		void getTuningVals(ScorbotIce::JointType joint,
				int &targetPos, int &targetVel, float &gravityCompensation, const Ice::Current&)
		{ itsScorbotInterface->getTuningVals(JointType2Joint_t[joint], targetPos, targetVel, gravityCompensation);	}

		// ######################################################################
		//! Set the gravity compensation parameters
		void setGravityParameters(int upperArmMass, int foreArmMass, float compensationScale, const Ice::Current&)
		{	itsScorbotInterface->setGravityParameters(upperArmMass, foreArmMass, compensationScale);	}

		// ######################################################################
		//! Get the current gravity compensation parameters
		void getGravityParameters(int &upperArmMass, int &foreArmMass, float &compensationScale, const Ice::Current&)
		{	itsScorbotInterface->getGravityParameters(upperArmMass, foreArmMass, compensationScale);	}

		// ######################################################################
		//! Get the requested port number
		std::string getPort() { return itsPort.getVal(); }

		void prepareForExit()
		{
			std::cerr << "#################### PREPARING FOR EXIT ####################" << std::endl;

			sleep(1);
		}

	private:
		std::map<ScorbotIce::JointType, ScorbotInterface::Joint_t> JointType2Joint_t;
		std::map<ScorbotInterface::Joint_t, ScorbotIce::JointType> Joint_t2JointType;

		nub::soft_ref<ScorbotInterface> itsScorbotInterface;
		OModelParam<std::string> itsPort;
};
nub::soft_ref<ScorbotI> scorbotServer;

// ######################################################################
void terminate(int s)
{
	std::cerr <<
		std::endl << "#################### INTERRUPT - SHUTTING DOWN SCORBOT ####################" << std::endl;

	scorbotServer->prepareForExit();
	exit(0);
}

// ######################################################################
int main(int argc, char** argv)
{
	signal(SIGHUP, terminate);  signal(SIGINT, terminate);
	signal(SIGQUIT, terminate); signal(SIGTERM, terminate);
	signal(SIGALRM, terminate);

	ModelManager manager("ScorbotServerManager");
	scorbotServer.reset(new ScorbotI(manager));
	manager.addSubComponent(scorbotServer);
	manager.parseCommandLine(argc, argv, "", 0, 0);
	manager.start();

	std::string connString("default -p ");
	connString += scorbotServer->getPort();

	int status = 0;
	Ice::CommunicatorPtr ic;

	try {
		ic = Ice::initialize(argc, argv);
		Ice::ObjectAdapterPtr adapter = 
			ic->createObjectAdapterWithEndpoints(
					"ScorbotServerAdapter", connString.c_str());
		Ice::ObjectPtr object = scorbotServer.get();
		adapter->add(object, ic->stringToIdentity("ScorbotServer"));
		adapter->activate();
		scorbotServer->init();
		ic->waitForShutdown();
	} catch(const Ice::Exception& e) {
		std::cerr << e << std::endl;
		status = 1;
	} catch(const char* msg)  {
		std::cerr << msg << std::endl;
		status = 1;
	}
	if(ic) {
		try {
			ic->destroy(); 
		} catch(const Ice::Exception& e) {
			std::cerr << e << std::endl;
			status = 1;
		}
	}

	scorbotServer->prepareForExit();
	return status;
}

