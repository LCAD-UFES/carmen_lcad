#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#include "Devices/BeeStem3.H"
#include <vector>

#define HEADING_K 0
#define HEADING_P 15
#define HEADING_I 0
#define HEADING_D 0

#define DEPTH_K 0
#define DEPTH_P 33
#define DEPTH_I 0
#define DEPTH_D 0

class BeeStemI: public RobotBrainComponent
{
public:

	struct FiringDeviceID
	{
		const static int Null = -1;
		const static int MIN = 0;
		const static int Shooter = 0;
		const static int DropperStage1 = 1;
		const static int DropperStage2 = 2;
		const static int MAX = 2;
	};

	struct ShooterState
	{
		const static int Idle = 0;
		const static int Armed = 1;
		const static int Firing = 2;
		const static int Reset = 3;
		const static int MAX = 3;
	};

	/*
	 * Ensures proper functioning of the dropper;
	 * -Each stage must be armed prior to dropping
	 * -Stage2 can't be armed until Stage1 has been reset
	 */
	struct DropperState
	{
		const static int AllIdle = 0;
		const static int Stage1Armed = 1;
		const static int Stage1Dropping = 2;
		const static int Stage1Reset = 3;
		const static int Stage1Idle = 4;
		const static int Stage2Armed = 5;
		const static int Stage2Dropping = 6;
		const static int Stage2Reset = 7;
		const static int MAX = 7;
	};

	struct BeeStemFlags
	{
		bool needsUpdateFromJoystick;
		bool initFlag;

		BeeStemFlags()
		{
			needsUpdateFromJoystick = false;
			initFlag = false;
		}
	};

	BeeStemI(int id, OptionManager& mgr, const std::string& descrName =
			"BeeStemI", const std::string& tagName = "BeeStemI");

	~BeeStemI();

	virtual void evolve();

	//!Get a message
	virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
			const Ice::Current&);

	virtual void registerTopics();

	void fireDevice(int deviceID);

	static void getMotorControllerMsg(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, int mc0, int mc1, int mc2, int mc3, int mc4, int mc5, int mc6, int mc7, int mc8);

private:

	void initPose();
	void setValuesFromJoystick();

	nub::soft_ref<BeeStem3> itsStem;
	//  nub::soft_ref<Serial> itsKillSwitch;
	//  pthread_mutex_t itsKillSwitchLock;

	IceUtil::Mutex itsStemMutex;
	std::map<int, int> itsJSMappings;
	//std::vector<int> itsJSValues;
	std::vector<int> itsButValues;
	int itsDesiredHeading, itsDesiredDepth, itsDesiredSpeed;
	int itsUpdateHeading, itsUpdateDepth, itsUpdateSpeed;
	int itsLastUpdateHeading, itsLastUpdateDepth, itsLastUpdateSpeed;
	IceUtil::Mutex itsUpdateMutex;
	int mShooterState;
	int mDropperState;
	int mFiringDeviceID;

	BeeStemFlags mFlags;
};
