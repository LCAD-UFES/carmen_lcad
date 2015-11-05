#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>

#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"

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

#include "Devices/Serial.H"

#ifndef IMUDATASERVER_H
#define IMUDATASERVER_H

class IMUDataServer: public RobotBrainComponent
{
public:

	struct Mode
	{
		const static int quat = 0;
		const static int euler = 1;
		const static int cos_mat = 2;
	};
	//!Constructor
	IMUDataServer(int id, OptionManager& mgr, const std::string& descrName =
			"BasicBrainComponent", const std::string& tagName =
			"BasicBrainComponent");

	//!Destructor
	~IMUDataServer();

	//!Main run loop
	virtual void evolve();

	void stop2();

	//!Get a message
	virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
			const Ice::Current&);

	//!Component registers itself as publisher/subscriber to topic(s)
	virtual void registerTopics();

	bool initMe();
	int doHardwareScan(xsens::Cmt3 &, CmtDeviceId []);
	void doMtSettings(xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId []);
	void getUserInputs(CmtOutputMode &, CmtOutputSettings &);

private:
	xsens::Cmt3 cmt3;
	CmtDeviceId deviceIds[256];

	CmtOutputMode mode;
	CmtOutputSettings settings;

	XsensResultValue res;

	CmtCalData caldata;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;
	RobotSimEvents::IMUDataServerMessagePtr msg;
	double tdata;

	xsens::Packet * packet;

	bool inited;
	OModelParam<int> mEnableConfig;
	//nub::soft_ref<Serial> itsPort;   //!< Serial port to use
};

#endif
