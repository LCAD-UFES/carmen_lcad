#include <Ice/Ice.h>
#include "Raster/Raster.H"
#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "GUI/PrefsWindow.H"
#include "Devices/DeviceOpts.H"
#include "Devices/Serial.H"
#include "Image/DrawOps.H"
#include <vector>
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/PrefsWindow.H"
#include "GUI/DebugWin.H"
#include <pthread.h>
#include "GUI/PrefsWindow.H"
#include <iomanip>
#include <signal.h>


std::string filePrefix;
std::vector<Image<PixRGB<byte> > > imageStore;
nub::soft_ref<OutputFrameSeries> ofs;
nub::soft_ref<InputFrameSeries> ifs;



// ######################################################################
void terminate(int s)
{
	std::cout <<
		std::endl << "#################### INTERRUPT - WRITING IMAGES ####################" << std::endl;

	for(size_t i=0; i<imageStore.size(); ++i)
	{
		std::cout << i+1 << "/" << imageStore.size() << std::endl;
    std::ostringstream ss;
    ss << filePrefix << "/" << std::setfill('0') << std::setw(5) << i << ".pnm";
    Raster::WriteRGB(imageStore[i], ss.str());
	}
	exit(0);
}


class ScorbotReallySimple: public ModelComponent
{
  public:
    ScorbotReallySimple(OptionManager& mgr, 
        const std::string& descrName = "",
        const std::string& tagName = "") :
      ModelComponent(mgr, descrName, tagName),
      itsSerial(new Serial(mgr)) 
    {
      addSubComponent(itsSerial);
      itsSerial->configure("/dev/ttyUSB0", 115200, "8N1", false, false, 10000);
      //itsSerial->setBlocking(true);
    }

    bool getState()
    {

      int ret;
      char retVal = '?';
      char cmd = 'S';
      itsSerial->write(&cmd, 1);
      usleep(5000);

      ret = itsSerial->read(&retVal, 1);
      if (ret != 1 || retVal != '0' || retVal != '1')
      {
        LINFO("%i: RetVal %c", ret, retVal);
      }
      return (retVal == '1');
    }
    
    void setPos(int pos)
    {
      LINFO("Moving to %i", pos);
      char cmd[2];
      cmd[0] = 'M';
      cmd[1] = pos;
      itsSerial->write(&cmd, 2);

      sleep(5);

      //LINFO("Wait for 0");
      //while(getState()) usleep(100000);
      LINFO("Wait for 1");
      while(!getState()) usleep(10000);
      LINFO("Move Done");
    }

  private:
    nub::ref<Serial> itsSerial;
};

// ######################################################################
int getKey()
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("Output")
    : rutz::shared_ptr<XWinManaged>();
  return uiwin->getLastKeyPress();
}

// ######################################################################
int main(int argc, char* argv[])
{
	signal(SIGHUP, terminate);  signal(SIGINT, terminate);
	signal(SIGQUIT, terminate); signal(SIGTERM, terminate);
	signal(SIGALRM, terminate);

  LINFO("#############################################STARTING##############################################");
  //srand(unsigned(time(NULL)));

  ModelManager mgr("App Scorbot Label");

  nub::ref<ScorbotReallySimple> scorbot(new ScorbotReallySimple(mgr));
  mgr.addSubComponent(scorbot);

  ifs.reset(new InputFrameSeries(mgr));
  mgr.addSubComponent(ifs);

  ofs.reset(new OutputFrameSeries(mgr));
  mgr.addSubComponent(ofs);

  if(mgr.parseCommandLine(argc, argv, "FilePrefix", 1, 1) == false) return -1;
  mgr.start();
  
	filePrefix = mgr.getExtraArg(0);
  
	PrefsWindow pWin("Camera Control", SimpleFont::FIXED(8));
	pWin.setValueNumChars(16);
	pWin.addPrefsForComponent(ifs.get(), true);
	pWin.update();
	


  // state 0 -> 1 means arm just started moving toward home
  // state 1 means on the way to home position or waiting at the home postion
  // state 1 -> 0 means the path started
  
	bool isStartDetected      = false;
	bool isFirstEdgeDetected  = false;
	bool isSecondEdgeDetected = false;

  
  ifs->startStream();
  while(true)
  {
		pWin.update();
    if(ifs->updateNext() == FRAME_COMPLETE) break;
    GenericFrame input = ifs->readFrame();
    
    //Display the new frame
    Image<PixRGB<byte> > img = input.asRgb();
    ofs->writeRGB(img, "Output", FrameInfo("output", SRC_POS));
    ofs->updateNext();

		if(isStartDetected)
			imageStore.push_back(img);

    if (!isStartDetected)
    {
      if (!isFirstEdgeDetected)
      {
        LINFO("####### Restart the arm program...");
        if (!scorbot->getState()) continue;
        isFirstEdgeDetected = true;
        continue;
      }
      if (!isSecondEdgeDetected)
      {
        LINFO("####### Waiting for the arm to home...");
        if (scorbot->getState()) continue;
        isSecondEdgeDetected = true;
        isStartDetected = true;
        LINFO("####### Start recording...");
        continue;
      }
    }
  }  while(true) 
  
  return 0;
} // main

