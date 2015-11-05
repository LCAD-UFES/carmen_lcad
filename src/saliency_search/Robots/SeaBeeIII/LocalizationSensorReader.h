#include <vector>
#include "Image/Point2D.H"
#include "LocalizationParticle.h"
#include "LocalizationMap.h"
#include "Ice/RobotSimEvents.ice.H"
//#include "LocalizationSensorReaderMessage.h"
#include <iostream>

using namespace std;

#ifndef LOCALIZATIONSENSORREADER_H_
#define LOCALIZATIONSENSORREADER_H_

namespace VirtualSensorMessage
{
	class VirtualSensorMessage
	{
	public:

	};

	class IMUDataServerMessage : public VirtualSensorMessage
	{
	public:
		vector<float> orientation;
		IMUDataServerMessage()
		{
			//
		}
	};

	class PipeColorSegmentMessage : public VirtualSensorMessage
	{
	public:
		bool pipeIsVisible;
		PipeColorSegmentMessage()
		{
			//
		}
	};

	class VisionRectangleMessage : public VirtualSensorMessage
	{
	public:
		bool rectangleIsVisible;
		int rectangleRelOrientation;
		VisionRectangleMessage()
		{
			//
		}
	};

	class BuoyColorSegmentMessage : public VirtualSensorMessage
	{
	public:
		BuoyColorSegmentMessage()
		{
			//
		}
	};
}

//used to convert sensor data into particle weights
class LocalizationSensorReader
{
public:

	struct SensorType
	{
		const static int compass = 0;
		const static int hydrophone = 1;
		const static int detector_buoy = 2;
		const static int detector_bin = 3;
		const static int detector_pipe = 4;

		int mId;
	};

	struct Flags
	{
		const static float voteFailed = -1;
	};

	LocalizationSensorReader()
	{
		inited = false;
	}

	~LocalizationSensorReader()
	{
		delete mVirtualMsg;
	}

	void takeReading(const RobotSimEvents::EventMessagePtr& realMsg)
	{
		mRealMsg = realMsg;
		calculateInitialOffset();
		inited = true;
	}

	void takeVirtualReading(VirtualSensorMessage::VirtualSensorMessage * virtualMsg)
	{
		mVirtualMsg = virtualMsg;
	}

	virtual void calculateInitialOffset() = 0;

	virtual float getScaledVote() = 0;

	static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map);

	float mScale;
	RobotSimEvents::EventMessagePtr mRealMsg;
	VirtualSensorMessage::VirtualSensorMessage * mVirtualMsg;
	float mVote;
	float mScaledVote; //value calculated from last reading
	int mSensorType;

	bool inited;
};

class BuoySensor: public LocalizationSensorReader
{
public:
	BuoySensor()
	{
		mSensorType = LocalizationSensorReader::SensorType::detector_buoy;
	}

	BuoySensor(float scale)
	{
		mSensorType = LocalizationSensorReader::SensorType::detector_buoy;
		mScale = scale;
	}

	virtual void calculateInitialOffset()
	{
		//
	}

	virtual float getScaledVote()
	{
		if (mRealMsg->ice_isA("::RobotSimEvents::PipeColorSegmentMessage"))
		{
			RobotSimEvents::BuoyColorSegmentMessagePtr realMsg = RobotSimEvents::BuoyColorSegmentMessagePtr::dynamicCast(mRealMsg);
			VirtualSensorMessage::BuoyColorSegmentMessage * virtualMsg = static_cast<VirtualSensorMessage::BuoyColorSegmentMessage*>(mVirtualMsg);
			mVote = 1.0f;
			mScaledVote = mVote * mScale;
			return mScaledVote;
		}
		return Flags::voteFailed;
	}

	static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map)
	{
		VirtualSensorMessage::BuoyColorSegmentMessage * msg = new VirtualSensorMessage::BuoyColorSegmentMessage;
		return msg;
	}
};

class CompassSensor: public LocalizationSensorReader
{
public:
	float mInitialOffset;
	CompassSensor()
	{
		mSensorType = LocalizationSensorReader::SensorType::compass;
		mInitialOffset = 0;
	}

	CompassSensor(float scale)
	{
		mSensorType = LocalizationSensorReader::SensorType::compass;
		mInitialOffset = 0;
		mScale = scale;
	}

	virtual void calculateInitialOffset()
	{
		RobotSimEvents::IMUDataServerMessagePtr realMsg = RobotSimEvents::IMUDataServerMessagePtr::dynamicCast(mRealMsg);
		mInitialOffset = realMsg->orientation[2];
	}

	virtual float getScaledVote()
	{
		if (mRealMsg->ice_isA("::RobotSimEvents::IMUDataServerMessage"))
		{
			float realOrientation = 0.0f;
			float virtualOrientation = 0.0f;
			RobotSimEvents::IMUDataServerMessagePtr realMsg = RobotSimEvents::IMUDataServerMessagePtr::dynamicCast(mRealMsg);
			VirtualSensorMessage::IMUDataServerMessage * virtualMsg = static_cast<VirtualSensorMessage::IMUDataServerMessage*>(mVirtualMsg);
			realOrientation = realMsg->orientation[2] - mInitialOffset;
			virtualOrientation = virtualMsg->orientation[2];
			mVote = 1.0f - LocalizationUtil::linearAngleDiffRatio(realOrientation, virtualOrientation);
			mScaledVote = mVote * mScale;
			return mScaledVote;
		}
		return Flags::voteFailed;
	}

	static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map)
	{
		VirtualSensorMessage::IMUDataServerMessage * msg = new VirtualSensorMessage::IMUDataServerMessage;
		msg->orientation.resize(3);
		msg->orientation[2] = LocalizationUtil::polarToEuler(part.mState.mAngle); // conventional -> euler
		return msg;
	}
};

class PipeSensor: public LocalizationSensorReader
{
public:
	PipeSensor()
	{
		mSensorType = LocalizationSensorReader::SensorType::detector_pipe;
	}

	PipeSensor(float scale)
	{
		mSensorType = LocalizationSensorReader::SensorType::detector_pipe;
		mScale = scale;
	}

	virtual void calculateInitialOffset()
	{
		//
	}

	virtual float getScaledVote()
	{
		if (mRealMsg->ice_isA("::RobotSimEvents::PipeColorSegmentMessage"))
		{
			mVote = 0.0f;
			RobotSimEvents::PipeColorSegmentMessagePtr realMsg = RobotSimEvents::PipeColorSegmentMessagePtr::dynamicCast(mRealMsg);
			VirtualSensorMessage::PipeColorSegmentMessage * virtualMsg = static_cast<VirtualSensorMessage::PipeColorSegmentMessage*>(mVirtualMsg);
			if(virtualMsg->pipeIsVisible && realMsg->size > 1500) //make sure we see a decent size orange blob
			{
				mVote += realMsg->size / (2.0f * 8000.0f);
				if(mVote > 0.5f)
					mVote = 0.5f;
			}
			mVote += 0.5f;
			mScaledVote = mVote * mScale;
			return mScaledVote;
		}
		return Flags::voteFailed;
	}

	static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map)
	{
		VirtualSensorMessage::PipeColorSegmentMessage * msg = new VirtualSensorMessage::PipeColorSegmentMessage;
		msg->pipeIsVisible = false;
		for(unsigned int i = 0; i < map.mMapEntities.size(); i ++)
		{
			if(map.mMapEntities[i].mObjectType == LocalizationMapEntity::ObjectType::pipe)
			{
				float distToPipe = sqrt(pow(part.mState.mPoint.i - map.mMapEntities[i].mCenter.i, 2) + pow(part.mState.mPoint.j - map.mMapEntities[i].mCenter.j, 2));
				float largerLength = map.mMapEntities[i].mDim.i;
				if(map.mMapEntities[i].mDim.j > largerLength)
				{
					largerLength = map.mMapEntities[i].mDim.j;
				}
				if(distToPipe <= largerLength / 2)
				{
					msg->pipeIsVisible = true;
				}
			}
		}
		return msg;
	}
};

class RectangleSensor: public LocalizationSensorReader
{
public:
	RectangleSensor()
	{
		mSensorType = LocalizationSensorReader::SensorType::detector_pipe;
	}

	RectangleSensor(float scale)
	{
		mSensorType = LocalizationSensorReader::SensorType::detector_pipe;
		mScale = scale;
	}

	virtual void calculateInitialOffset()
	{
		//
	}

	virtual float getScaledVote()
	{
		if (mRealMsg->ice_isA("::RobotSimEvents::VisionRectangleMessage"))
		{
			RobotSimEvents::VisionRectangleMessagePtr realMsg = RobotSimEvents::VisionRectangleMessagePtr::dynamicCast(mRealMsg);
			VirtualSensorMessage::VisionRectangleMessage * virtualMsg = static_cast<VirtualSensorMessage::VisionRectangleMessage*>(mVirtualMsg);
			int realOrientation = 0; //fuck it I'm just gonna average all of them
			for(int i = 0; i < realMsg->quads.size(); i ++)
			{
				realOrientation += realMsg->quads[i].angle;
			}
			realOrientation /= realMsg->quads.size();
			if(virtualMsg->rectangleIsVisible)
			{
				mVote = 1 - 0.5 * LocalizationUtil::linearAngleDiffRatio(realOrientation, virtualMsg->rectangleRelOrientation);
			}
			else
			{
				mVote = 0.25f;
			}
			mScaledVote = mVote * mScale;
			return mScaledVote;
		}
		return Flags::voteFailed;
	}

	static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map)
	{
		VirtualSensorMessage::VisionRectangleMessage * msg = new VirtualSensorMessage::VisionRectangleMessage;
		msg->rectangleIsVisible = false;
		int closestRectIndex = -1;
		float closestRectDist = -1;
		for(unsigned int i = 0; i < map.mMapEntities.size(); i ++)
		{
			if((map.mMapEntities[i].mShapeType == LocalizationMapEntity::ShapeType::rect || map.mMapEntities[i].mShapeType == LocalizationMapEntity::ShapeType::square) && map.mMapEntities[i].mInteractionType != LocalizationMapEntity::InteractionType::external)
			{
				float distToRect = sqrt(pow(part.mState.mPoint.i - map.mMapEntities[i].mCenter.i, 2) + pow(part.mState.mPoint.j - map.mMapEntities[i].mCenter.j, 2));
				float largerLength = map.mMapEntities[i].mDim.i;
				if(map.mMapEntities[i].mDim.j > largerLength)
				{
					largerLength = map.mMapEntities[i].mDim.j;
				}
				if(distToRect <= largerLength / 2)
				{
					msg->rectangleIsVisible = true;
				}
				if(distToRect >= closestRectDist)
				{
					closestRectDist = distToRect;
					closestRectIndex = i;
				}
			}
		}
		msg->rectangleRelOrientation = map.mMapEntities[closestRectIndex].mOrientation;
		return msg;
	}
};

#endif /* LOCALIZATIONSENSORREADER_H_ */
