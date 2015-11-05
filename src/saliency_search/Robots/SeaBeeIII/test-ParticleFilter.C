#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/SeaBeeIII/ParticleFilter.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"
#include "XBox360RemoteControlI.H"

class RobotBrainServiceService: public Ice::Service
{
protected:
	virtual bool start(int, char* argv[]);
	virtual bool stop()
	{
		if (itsMgr)
			delete itsMgr;
		return true;
	}

private:
	Ice::ObjectAdapterPtr itsAdapter;
	ModelManager *itsMgr;
};

bool RobotBrainServiceService::start(int argc, char* argv[])
{
	char adapterStr[255];

	//Create the adapter
	int port = RobotBrainObjects::RobotBrainPort;
	bool connected = false;

	while (!connected)
	{
		try
		{
			LINFO("Trying Port:%d", port);
			sprintf(adapterStr, "default -p %i", port);
			itsAdapter = communicator()->createObjectAdapterWithEndpoints(
					"ParticleFilter", adapterStr);
			connected = true;
		}
		catch (Ice::SocketException)
		{
			port++;
		}
	}

	//Create the manager and its objects
	itsMgr = new ModelManager("ParticleFilterService");

	/*LINFO("Starting XBox360RemoteControl");
	XBox360RemoteControlI * controller = new XBox360RemoteControlI(0, *itsMgr, "XBox360RemoteControl1", "XBox360RemoteControl2");
	nub::ref<XBox360RemoteControlI> ret(controller);
	LINFO("XBox360RemoteControl Created");
	itsMgr->addSubComponent(ret);
	LINFO("XBox360RemoteControl Added As Sub Component");
	ret->init(communicator(), itsAdapter);
	LINFO("XBox360RemoteControl Inited");*/

	LINFO("Starting ParticleFilter");
	nub::ref<ParticleFilter> bbc(new ParticleFilter(0, *itsMgr, NULL /*controller*/,
			"ParticleFilter1", "ParticleFilter2"));
	LINFO("ParticleFilter Created");
	itsMgr->addSubComponent(bbc);
	LINFO("ParticleFilter Added As Sub Component");
	bbc->init(communicator(), itsAdapter);
	LINFO("ParticleFilter Inited");

	itsMgr->parseCommandLine((const int) argc, (const char**) argv, "", 0, 0);

	itsAdapter->activate();

	itsMgr->start();

	return true;
}

// ######################################################################
int main(int argc, char** argv)
{

	RobotBrainServiceService svc;
	return svc.main(argc, argv);
}

