#include "Component/ModelManager.H"
#include "Component/OptionManager.H"

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
#include <map>

#include "Devices/JoyStick.H"

#ifndef XBOX360REMOTECONTROLI_H_
#define XBOX360REMOTECONTROLI_H_

#define MAX_JS_AXIS_VAL 32767.0
#define JS_DEAD_ZONE 15
#define JS_AXIS_MAX 100

class TestJoyStickListener;

namespace XBox360RemoteControl
{
	struct ControlAxis
	{
		const static int BTN = 0; //button
		const static int JSTK = 1; //joystick
		const static int TRGR = 2; //trigger
		const static int NONE = -9999;

		struct Btn
		{
			static int Id;
			const static int mType = BTN;
		};

		struct Axis
		{
			static int Id;
			const static int mType = JSTK;
		};
	};

	struct Keys //store keys for specific actions and labels
	{
		struct Btns
		{
			enum Ids
			{
				L_BUMPER,
				R_BUMPER,
				BACK,
				START,
				CENTER,
				L_JOYSTICK,
				R_JOYSTICK,
				X,
				Y,
				A,
				B
			};
		};

		struct Axes
		{
			enum
			{
				L_TRIGGER,
				R_TRIGGER,
				L_JOYSTICK_X,
				L_JOYSTICK_Y,
				R_JOYSTICK_X,
				R_JOYSTICK_Y,
				DPAD_X, //you'd think it's a button, but it's actually a joystick
				DPAD_Y
			};
		};

		struct Actions
		{
			static std::map<int, std::string> toString;
			static std::map<std::string, int> toInt;

			enum Ids
			{
				AXIS_INVERT, DIVE, SURFACE, STRAFE, SPEED, HEADING, ARM_NEXT_DEV, FIRE_DEV, NONE
			};

			static std::map<int, std::string> populateMappingToString()
			{
				std::map<int, std::string> theResult;
				theResult[AXIS_INVERT] = "AXIS_INVERT";
				theResult[DIVE] = "DIVE";
				theResult[SURFACE] = "SURFACE";
				theResult[STRAFE] = "STRAFE";
				theResult[SPEED] = "SPEED";
				theResult[HEADING] = "HEADING";
				theResult[ARM_NEXT_DEV] = "ARM_NEXT_DEV";
				theResult[FIRE_DEV] = "FIRE_DEV";
				return theResult;
			}
		};

		template<class _T1, class _T2>
		static std::map<_T2, _T1> populateInverseMapping(std::map<_T1, _T2> m)
		{
			std::map<_T2, _T1> theResult;
			class std::map<_T1, _T2>::iterator it;
			for(it = m.begin(); it != m.end(); it ++)
			{
				theResult[it->second] = it->first;
			}
			return theResult;
		}
	};
}

class XBox360RemoteControlI: public RobotBrainComponent
{
public:

	XBox360RemoteControlI();

	XBox360RemoteControlI(int id, OptionManager& mgr,
			const std::string& descrName = "XBox360RemoteControl",
			const std::string& tagName = "XBox360RemoteControl");

	~XBox360RemoteControlI();

	virtual void evolve();

	//!Get a message
	virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
			const Ice::Current&);

	virtual void registerTopics();

	void calibrate();
	void updateSetBtn(int btn);
	void updateSetAxis(int axis);
	void updateSetAxisInversion();
	bool isEnabled()
	{
		return itsJS.get()->joystickPlugged();
	}
	bool isCalibrated()
	{
		return itsCalibrated;
	}

	std::map<int, int> itsTypeMappings; //indicates whether each axis is a button or a joystick
	std::map<int, int> itsActionIds; //maps axis indices to key names for actions
	std::map<int, int> itsAxisIndices; //maps key names for actions to axis indices
	std::map<int, int> itsInversionMappings; //maps axis indices to inversion values; 1 = normal, -1 = inverted

	//std::map<int, int> itsAxisIds; //maps action ids to axis ids, not useful right now
	//std::map<int, int> itsBtnIds; //maps action ids to button ids, not useful right now

private:
	int selectionType;
	int itsAxisInversionBtn;
	bool itsCalibrated;
	int itsSetBtn; //set each time a btn is pressed, holds the index of that btn
	int itsSetAxis; //set each time an axis is set to +/- max, used to determine which axis is being selected for use
	bool itsSetAxisInvert; //set each time btn A is pressed (indicates last axis should be inverted)
	IceUtil::Mutex itsSetBtnMutex;
	IceUtil::Mutex itsSetAxisMutex;
	IceUtil::Mutex itsSetAxisInvertMutex;

	//OModelParam<std::int>              itsJoystickDisable;

#ifdef HAVE_LINUX_JOYSTICK_H
	//JoyStick itsJSReal;
	nub::soft_ref<JoyStick> itsJS;
	rutz::shared_ptr<TestJoyStickListener> lis;
	rutz::shared_ptr<JoyStickListener> lis2;
#endif
};

#ifdef HAVE_LINUX_JOYSTICK_H

//! A simple joystick listener
class TestJoyStickListener : public JoyStickListener
{
public:
	TestJoyStickListener(rutz::shared_ptr<XBox360RemoteControlI> remote)
	{	itsRemote = remote;}

	virtual ~TestJoyStickListener()
	{}

	virtual void axis(const uint num, const int16 val)
	{
		//int invert = 1; //1 is normal, -1 is inverted
		RobotSimEvents::JoyStickControlMessagePtr msg = new RobotSimEvents::JoyStickControlMessage;
		if(itsRemote->itsActionIds.find(num) != itsRemote->itsActionIds.end())
		{
			msg->axisName = XBox360RemoteControl::Keys::Actions::toString[itsRemote->itsActionIds[num]];
			//msg->axis = itsRemote->itsActionIds[num]; //fetch the enum action mapping
		}
		else
		{
			msg->axisName = "AXIS_NOT_BOUND";
		}

		msg->axis = num;
		msg->axisVal = (int)(float(val)/MAX_JS_AXIS_VAL * 100.0);

		// make sure value is outside of dead zone
		if(abs(msg->axisVal) < JS_DEAD_ZONE)
		msg->axisVal = 0;
		else if(abs(msg->axisVal) > JS_AXIS_MAX &&
				msg->axis == 1)
		{
			if(msg->axisVal > 0)
			msg->axisVal = JS_AXIS_MAX;
			else
			msg->axisVal = JS_AXIS_MAX * -1;
		}

		if(msg->axisVal == 100 || msg->axisVal == -100)
		{
			//invert = msg->axisVal / 100;
			itsRemote->updateSetAxis(msg->axis);
		}

		if(itsRemote->itsInversionMappings.find(num) != itsRemote->itsInversionMappings.end() && itsRemote->itsInversionMappings[num] != 0)
		{
			msg->axisVal *= itsRemote->itsInversionMappings[num]; //apply possible inversion to the axis
		}

		msg->button = -1;
		msg->butVal = 0;

		if(msg->axisVal != 0)
		{
			//LINFO("Axis[%s] = %d", msg->axisName.c_str(), msg->axisVal);
		}

		if(itsRemote->isEnabled() && itsRemote->isCalibrated())
		{
			itsRemote->publish("XBox360RemoteControlMessageTopic", msg);
		}
	}

	virtual void button(const uint num, const bool state)
	{
		//LINFO("button: %d\n", num);

		if(state && itsRemote->itsActionIds.find(-num-1) != itsRemote->itsActionIds.end() && itsRemote->itsActionIds[-num-1] == XBox360RemoteControl::Keys::Actions::AXIS_INVERT)
		{
			itsRemote->updateSetAxisInversion();
		}
		RobotSimEvents::JoyStickControlMessagePtr msg = new RobotSimEvents::JoyStickControlMessage;
		msg->axis = -1;
		msg->axisVal = 0;
		if(itsRemote->itsTypeMappings.find(-num-1) != itsRemote->itsTypeMappings.end() && itsRemote->itsTypeMappings[-num-1] == XBox360RemoteControl::ControlAxis::BTN)
		{
			msg->axisName = XBox360RemoteControl::Keys::Actions::toString[itsRemote->itsActionIds[-num-1]];
			msg->axis = -num-1;
			msg->axisVal = state ? 100 * itsRemote->itsInversionMappings[-num-1] : 0;
		}
		msg->button = num;
		msg->butVal = state;
		if(state)
		{
			itsRemote->updateSetBtn(num);
		}
		else
		{
			//itsRemote->updateSetBtn(XBox360RemoteControl::ControlAxis::NONE);
		}
		//LINFO("Button[%d] = %d",msg->button, msg->butVal);
		if(itsRemote->isEnabled() && itsRemote->isCalibrated())
		{
			itsRemote->publish("XBox360RemoteControlMessageTopic", msg);
		}
	}

	rutz::shared_ptr<XBox360RemoteControlI> itsRemote;
};

#endif

#endif //XBOX360REMOTECONTROLI_H_
