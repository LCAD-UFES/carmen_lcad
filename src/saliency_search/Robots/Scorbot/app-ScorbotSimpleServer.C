#include <Ice/Ice.h>
#include "Devices/Serial.H"
#include "Component/ModelComponent.H"
#include "Component/ModelManager.H"
#include "Robots/Scorbot/ScorbotSimple.ice.H"

class ScorbotSimpleI: public ScorbotSimpleIce::ScorbotSimple, public ModelComponent
{
	public:
		ScorbotSimpleI(OptionManager& mgr, 
				const std::string& descrName = "",
				const std::string& tagName = "") :
			ModelComponent(mgr, descrName, tagName),
			itsSerial(new Serial(mgr)) 
			{
				addSubComponent(itsSerial);
				itsSerial->configure("/dev/ttyACM0", 115200, "8N1", false, false, 1000000);
			}

			bool getState(const Ice::Current&)
			{
				char cmd = 'a';
				itsSerial->write(&cmd, 1);
				usleep(10000);
				char retVal;
				itsSerial->read(&retVal, 1);
				return (retVal == 'B');
			}

			void setNext(const Ice::Current&)
			{
				char cmd = 'b';
				itsSerial->write(&cmd, 1);
			}

			void reset(const Ice::Current&)
			{
				char cmd = 'c';
				itsSerial->write(&cmd, 1);
				usleep(10000);
			}

	private:
			nub::ref<Serial> itsSerial;
};


int main(int argc, char** argv)
{
	ModelManager manager("ScorbotServerManager");
	nub::ref<ScorbotSimpleI> scorbot(new ScorbotSimpleI(manager));
	manager.addSubComponent(scorbot);

	manager.parseCommandLine(argc, argv, "", 0, 0);
	manager.start();

	std::string connString("default -p 10000");

	int status = 0;
	Ice::CommunicatorPtr ic;

	try {
		ic = Ice::initialize(argc, argv);
		Ice::ObjectAdapterPtr adapter = 
			ic->createObjectAdapterWithEndpoints(
					"ScorbotServerAdapter", connString.c_str());
		Ice::ObjectPtr object = scorbot.get();
		adapter->add(object, ic->stringToIdentity("ScorbotServer"));
		adapter->activate();
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

	return status;
}

