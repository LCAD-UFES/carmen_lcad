#include <Ice/Ice.h>
#include "Component/ModelManager.H"
#include "Robots/Scorbot/Scorbot.ice.H"
#include "Media/FrameSeries.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "GUI/PrefsWindow.H"
#include "Devices/DeviceOpts.H"
#include <vector>
#include <pthread.h>



nub::soft_ref<OutputFrameSeries> ofs;
nub::soft_ref<InputFrameSeries> ifs;

void *displayThread(void*)
{
	PrefsWindow pWin("Camera Control", SimpleFont::FIXED(8));
	pWin.setValueNumChars(16);
	pWin.addPrefsForComponent(ifs.get(), true);

	while(1)
	{
		pWin.update();

		if(ifs->updateNext() == FRAME_COMPLETE) break;
		GenericFrame input = ifs->readFrame();
		if(!input.initialized()) break;

		Image<PixRGB<byte> > img = input.asRgb();
		Dims winSize(512, 512);
		Point2D<int> center(img.getWidth()/2, img.getHeight()/2);


//		drawCircle(img, center, 16, PixRGB<byte>(255, 0, 0), 2);
//		drawCircle(img, center, 2, PixRGB<byte>(255, 0, 0), 2);

		ofs->writeRGB(img, "Output", FrameInfo("output", SRC_POS));
		ofs->updateNext();

	}

	pthread_exit(NULL);

}

int main(int argc, char* argv[])
{
	ModelManager mgr("Test Scorbot Interface");

	ifs.reset(new InputFrameSeries(mgr));
	mgr.addSubComponent(ifs);

	ofs.reset(new OutputFrameSeries(mgr));
	mgr.addSubComponent(ofs);

	if(mgr.parseCommandLine(argc, argv, "RunPath [true|false]", 1, 1) == false) return -1;
	mgr.start();
	ifs->startStream();

	bool runPath = false;

	if(mgr.numExtraArgs() > 0)
	{
		if(mgr.getExtraArg(0) == "true") runPath = true;
	}


	ScorbotIce::ScorbotPrx scorbot;

	bool IceError = false;
	Ice::CommunicatorPtr ic;
	try {
		int argc_fake=1; char* argv_fake[argc_fake];
		argv[0] = new char[128]; sprintf(argv[0], "test-ScorbotServer");
		ic = Ice::initialize(argc_fake, argv_fake);
		Ice::ObjectPrx base = ic->stringToProxy(
				"ScorbotServer:default -p 10000 -h ihead");
		scorbot = ScorbotIce::ScorbotPrx::checkedCast(base);
		if(!scorbot)
			throw "Invalid Proxy";
	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		IceError = true;
	} catch(const char* msg) {
		std::cerr << msg << std::endl;
		IceError = true;
	}
	if(IceError) {
		std::cerr << "#################### ERROR - QUITTING ####################" << std::endl;
		if(ic) ic->destroy();
		exit(-1);
	}

	pthread_t videoThread;
	pthread_create(&videoThread, NULL, displayThread, NULL);

	scorbot->setEnabled(true);

	std::vector<int> focusVals;
	std::vector<ScorbotIce::encoderValsType> positions;
	ScorbotIce::encoderValsType encoderVals;
	int focus = 8;

	////////////////////////////////////////
	focus = 8;
	encoderVals[ScorbotIce::Base] = -4286;
	encoderVals[ScorbotIce::Shoulder] = 8976;
	encoderVals[ScorbotIce::Elbow] = -3940;
	encoderVals[ScorbotIce::Wrist1] = 1884;
	encoderVals[ScorbotIce::Wrist2] = -2291;
	encoderVals[ScorbotIce::Gripper] = 0;
	encoderVals[ScorbotIce::Slider] = -42130;
	positions.push_back(encoderVals);
	focusVals.push_back(focus);

	////////////////////////////////////////
	focus = 9;
	encoderVals[ScorbotIce::Base] = -3397;
	encoderVals[ScorbotIce::Shoulder] = 6985;
	encoderVals[ScorbotIce::Elbow] = 1502;
	encoderVals[ScorbotIce::Wrist1] = 1881;
	encoderVals[ScorbotIce::Wrist2] = -2288;
	encoderVals[ScorbotIce::Gripper] = 0;
	encoderVals[ScorbotIce::Slider] = -42130;
	positions.push_back(encoderVals);
	focusVals.push_back(focus);

	////////////////////////////////////////
	focus = 13;
	encoderVals[ScorbotIce::Base] = 1805;
	encoderVals[ScorbotIce::Shoulder] = 2720;
	encoderVals[ScorbotIce::Elbow] = 4072;
	encoderVals[ScorbotIce::Wrist1] = 2140;
	encoderVals[ScorbotIce::Wrist2] = -2416;
	encoderVals[ScorbotIce::Gripper] = 0;
	encoderVals[ScorbotIce::Slider] = 34534;
	positions.push_back(encoderVals);
	focusVals.push_back(focus);

	////////////////////////////////////////
	focus = 10;
	encoderVals[ScorbotIce::Base] = 2251;
	encoderVals[ScorbotIce::Shoulder] = 8416;
	encoderVals[ScorbotIce::Elbow] = 2537;
	encoderVals[ScorbotIce::Wrist1] = 1720;
	encoderVals[ScorbotIce::Wrist2] = -1793;
	encoderVals[ScorbotIce::Gripper] = 0;
	encoderVals[ScorbotIce::Slider] = 109292;
	positions.push_back(encoderVals);
	focusVals.push_back(focus);

	////////////////////////////////////////
	focus = 11;
	encoderVals[ScorbotIce::Base] = -821;
	encoderVals[ScorbotIce::Shoulder] = 5717;
	encoderVals[ScorbotIce::Elbow] = 2081;
	encoderVals[ScorbotIce::Wrist1] = 1492;
	encoderVals[ScorbotIce::Wrist2] = -1631;
	encoderVals[ScorbotIce::Gripper] = 0;
	encoderVals[ScorbotIce::Slider] = 32629;
	positions.push_back(encoderVals);
	focusVals.push_back(focus);
	////////////////////////////////////////
	focus = 10;
	encoderVals[ScorbotIce::Base] = -6493;
	encoderVals[ScorbotIce::Shoulder] = 6407;
	encoderVals[ScorbotIce::Elbow] = 1812;
	encoderVals[ScorbotIce::Wrist1] = 1831;
	encoderVals[ScorbotIce::Wrist2] = -1763;
	encoderVals[ScorbotIce::Gripper] = 0;
	encoderVals[ScorbotIce::Slider] = -3482;
	positions.push_back(encoderVals);
	focusVals.push_back(focus);

	mgr.setOptionValString(&OPT_FrameGrabberFocusAuto, "false");
	while(1)
	{
		if(runPath)
		{
			for(size_t posIdx=0; posIdx<positions.size(); posIdx++)
			{
				std::cout << "---------------------------------------" << std::endl;
				std::cout << "Going To Position " << posIdx << std::endl;

				// Set the focus
				char buffer[32];
				sprintf(buffer, "%d", focusVals[posIdx]);
				std::string focusString(buffer);
				mgr.setOptionValString(&OPT_FrameGrabberFocus, focusString);

				// Set the joints
				scorbot->setJoints(positions[posIdx], 2000);

				sleep(2);

				std::cout << "Done..." << std::endl;

				// Print the final positions
				ScorbotIce::encoderValsType currPos = scorbot->getEncoders();
				std::cout << "Base:     " << currPos[ScorbotIce::Base]     << " / " << positions[posIdx][ScorbotIce::Base]     << std::endl;
				std::cout << "Shoulder: " << currPos[ScorbotIce::Shoulder] << " / " << positions[posIdx][ScorbotIce::Shoulder] << std::endl;
				std::cout << "Elbow:    " << currPos[ScorbotIce::Elbow]    << " / " << positions[posIdx][ScorbotIce::Elbow]    << std::endl;
				std::cout << "Wrist1:   " << currPos[ScorbotIce::Wrist1]   << " / " << positions[posIdx][ScorbotIce::Wrist1]   << std::endl;
				std::cout << "Wrist2:   " << currPos[ScorbotIce::Wrist2]   << " / " << positions[posIdx][ScorbotIce::Wrist2]   << std::endl;
				std::cout << "Gripper:  " << currPos[ScorbotIce::Gripper]  << " / " << positions[posIdx][ScorbotIce::Gripper]  << std::endl;
				std::cout << "Slider:   " << currPos[ScorbotIce::Slider]   << " / " << positions[posIdx][ScorbotIce::Slider]   << std::endl;
				sleep(2);
			} // for
		} // if(runPath)

		sleep(1);

	} // while(1)

}

