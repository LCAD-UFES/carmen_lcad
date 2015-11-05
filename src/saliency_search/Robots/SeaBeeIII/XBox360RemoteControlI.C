#include "Robots/SeaBeeIII/XBox360RemoteControlI.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

using namespace XBox360RemoteControl;

std::map<int, std::string> Keys::Actions::toString = Keys::Actions::populateMappingToString();
std::map<std::string, int> Keys::Actions::toInt = Keys::populateInverseMapping(Keys::Actions::toString);

/*const string Keys::Btns::L_BUMPER = "L_BUMPER_BTN";
const string Keys::Btns::R_BUMPER = "R_BUMPER_BTN";
const string Keys::Btns::BACK = "BACK_BTN";
const string Keys::Btns::START = "START_BTN";
const string Keys::Btns::CENTER = "CENTER_BTN";
const string Keys::Btns::L_JOYSTICK = "L_JOYSTICK_BTN";
const string Keys::Btns::R_JOYSTICK = "R_JOYSTICK_BTN";
const string Keys::Btns::X = "X_BTN";
const string Keys::Btns::Y = "Y_BTN";
const string Keys::Btns::A = "A_BTN";
const string Keys::Btns::B = "B_BTN";

const string Keys::Axes::L_TRIGGER = "L_TRIGGER_AXIS";
const string Keys::Axes::R_TRIGGER = "R_TRIGGER_AXIS";
const string Keys::Axes::L_JOYSTICK_X = "L_JOYSTICK_X_AXIS";
const string Keys::Axes::L_JOYSTICK_Y = "L_JOYSTICK_Y_AXIS";
const string Keys::Axes::R_JOYSTICK_X = "R_JOYSTICK_X_AXIS";
const string Keys::Axes::R_JOYSTICK_Y = "R_JOYSTICK_Y_AXIS";
const string Keys::Axes::DPAD_X = "DPAD_X_AXIS";
const string Keys::Axes::DPAD_Y = "DPAD_Y_AXIS";*/

using namespace std;

const ModelOptionCateg MOC_SeaBee3GUI = { MOC_SORTPRI_3,
		"SeaBee3GUI Related Options" };

const ModelOptionDef OPT_JoystickDisable = { MODOPT_ARG(int),
		"JoystickDisable", &MOC_SeaBee3GUI, OPTEXP_CORE,
		"Whether or not to disable the joystick control with GUI",
		"disable-joystick", '\0', "<int>", "0" };

#ifdef HAVE_LINUX_JOYSTICK_H
// ######################################################################
XBox360RemoteControlI::XBox360RemoteControlI(int id, OptionManager& mgr,
		const std::string& descrName, const std::string& tagName) :
RobotBrainComponent(mgr, descrName, tagName),
itsJS(new JoyStick(mgr)),
lis(new TestJoyStickListener(rutz::shared_ptr<XBox360RemoteControlI>(this)))//,
//  itsJoystickDisable(&OPT_JoystickDisable, this, 0)

{
	addSubComponent(itsJS);
	itsCalibrated = false;
	itsSetAxis = -1;

	//the numbering here only determines the order in which the axes are requested during configuration
	itsActionIds[0] = Keys::Actions::AXIS_INVERT;
	itsActionIds[1] = Keys::Actions::STRAFE;
	itsActionIds[2] = Keys::Actions::SPEED;
	itsActionIds[3] = Keys::Actions::HEADING;
	itsActionIds[4] = Keys::Actions::DIVE;
	itsActionIds[5] = Keys::Actions::SURFACE;
	itsActionIds[6] = Keys::Actions::ARM_NEXT_DEV;
	itsActionIds[8] = Keys::Actions::FIRE_DEV;
	//0,1,3,5,2
}

// ######################################################################
XBox360RemoteControlI::~XBox360RemoteControlI()
{
}

void XBox360RemoteControlI::registerTopics()
{
	/*  if(!itsJoystickDisable.getVal())
	 {*/
	this->registerPublisher("XBox360RemoteControlMessageTopic");

	lis2.dynCastFrom(lis); // cast down
	itsJS->setListener(lis2);
	//    }
}

void XBox360RemoteControlI::evolve()
{
	if(!itsCalibrated && isEnabled())
	{
		calibrate();
		itsCalibrated = true;
	}
}

void XBox360RemoteControlI::calibrate()
{
	map<int, int>::iterator it;
	map<int, int> tempIdMap;
	map<int, int> tempInversionMap;

	/*cout << "Press " << Keys::Actions::toString[Keys::Actions::AXIS_INVERT] << endl;
	updateSetBtn(-1);
	int setBtn = -1;
	while(setBtn < 0)
	{
		sleep(0.5);
		itsSetBtnMutex.lock();
		setBtn = itsSetBtn;
		itsSetBtnMutex.unlock();
	}
	itsAxisInversionBtn = setBtn;

	cout << "set to " << itsAxisInversionBtn << endl;*/

	for(it = itsActionIds.begin(); it != itsActionIds.end(); it++) //cycling through actions
	{
		//LINFO("Set %s...\n",Keys::Actions::mMappings[it->first].c_str());
		cout << "Set " << Keys::Actions::toString[it->second] << "..." << endl;

		itsSetAxis = ControlAxis::NONE;

		int setAxis = ControlAxis::NONE;
		selectionType = ControlAxis::NONE;

		while(setAxis == ControlAxis::NONE)
		{
			sleep(0.5);

			itsSetAxisMutex.lock();
			if(itsSetAxis != ControlAxis::NONE)
			{
				if(tempIdMap.find(itsSetAxis) == tempIdMap.end())
				{
					//cout << itsSetAxis << "," << selectionType << "," << itsTypeMappings[itsSetAxis] << endl;
					setAxis = itsSetAxis;
				}
				else
				{
					//cout << itsSetAxis << "," << selectionType << "," << itsTypeMappings[itsSetAxis] << endl;
					cout << "That axis is already mapped to a function." << endl;
					itsSetAxis = ControlAxis::NONE;
				}
			}

			itsSetAxisMutex.unlock();
		}

		//cout << it->second << " now bound to " << setAxis << endl;
		//LINFO("%s now bound to %d\n\n",Keys::Actions::mMappings[it->first], setAxis);
		cout << Keys::Actions::toString[it->second] << " now bound to " << selectionType << ":" << setAxis << endl;

		tempIdMap[setAxis] = it->second;
		itsTypeMappings[setAxis] = selectionType;
		itsInversionMappings[setAxis] = 1;

		//sleep(5);

	}

	itsActionIds = tempIdMap;
	itsAxisIndices = Keys::populateInverseMapping(itsActionIds);

	//default axes to be inverted (so we don't have to do this every time)
	itsInversionMappings[itsAxisIndices[Keys::Actions::STRAFE]] = -1;
	itsInversionMappings[itsAxisIndices[Keys::Actions::HEADING]] = -1;
	itsInversionMappings[itsAxisIndices[Keys::Actions::DIVE]] = -1;
	itsInversionMappings[itsAxisIndices[Keys::Actions::SURFACE]] = -1;

	cout << "Calibration complete!" << endl;
}

void XBox360RemoteControlI::updateSetBtn(int btn)
{
	if(btn == ControlAxis::NONE)
	{
		itsSetBtn = -1;
		//itsSetAxis = ControlAxis::NONE;
		return;
	}
	itsSetBtnMutex.lock();
	itsSetBtn = btn;
	updateSetAxis(-btn - 1); //btn[n] --> axis[-n-1]
	itsSetBtnMutex.unlock();
	selectionType = ControlAxis::BTN;
}

void XBox360RemoteControlI::updateSetAxis(int axis)
{
	//LINFO("%d",axis);
	itsSetAxisMutex.lock();
	if(itsCalibrated)
	{
		if(itsActionIds.find(axis) != itsActionIds.end() && itsActionIds[axis] != Keys::Actions::AXIS_INVERT)
		{
			cout << Keys::Actions::toString[itsActionIds[axis]] << " selected; inversion settings: " << itsInversionMappings[axis] << endl;
			itsSetAxis = axis;
		}
		else if(itsActionIds.find(axis) != itsActionIds.end() && itsActionIds[axis] == Keys::Actions::AXIS_INVERT)
		{
			//do nothing, this just saves some typing
		}
		else if(axis >= 0)
		{
			cout << "Axis (joystick code: " << axis << ") not bound" << endl;
		}
		else
		{
			cout << "Axis (button code: " << axis << ") not bound" << endl;
		}
	}
	else
	{
		itsSetAxis = axis;
	}
	itsSetAxisMutex.unlock();
	selectionType = ControlAxis::JSTK;
}

void XBox360RemoteControlI::updateSetAxisInversion()
{
	itsSetAxisInvertMutex.lock();
	itsSetAxisMutex.lock();

	if(itsSetAxis != ControlAxis::NONE)
	{
		itsInversionMappings[itsSetAxis] *= -1;
		cout << "Inverted " << Keys::Actions::toString[itsActionIds[itsSetAxis]] << ": " << itsInversionMappings[itsSetAxis] << endl;
	}

	itsSetAxisMutex.unlock();
	itsSetAxisInvertMutex.unlock();

	updateSetBtn(ControlAxis::NONE);
}

// ######################################################################
void XBox360RemoteControlI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
		const Ice::Current&)
{

}
#else
//No linux/joystick defined..

XBox360RemoteControlI::XBox360RemoteControlI(int id, OptionManager& mgr,
		const std::string& descrName, const std::string& tagName) :
	RobotBrainComponent(mgr, descrName, tagName)
{
	;
}

XBox360RemoteControlI::~XBox360RemoteControlI()
{
}

void XBox360RemoteControlI::evolve()
{
	;
}

//!Get a message
void XBox360RemoteControlI::updateMessage(
		const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
	;
}

void XBox360RemoteControlI::registerTopics()
{
	;
}

void XBox360RemoteControlI::calibrate()
{
	;
}

void XBox360RemoteControlI::updateSetBtn(int btn)
{
	;
}

void XBox360RemoteControlI::updateSetAxis(int axis)
{
	;
}

void XBox360RemoteControlI::updateSetAxisInversion()
{
	;
}

#endif

