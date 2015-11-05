#include <iomanip>
#include <vector>
#include <pthread.h>
#include <signal.h>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <list>

#include "Devices/V4L2grabber.H"
#include "Util/WorkThreadServer.H"
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
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/PrefsWindow.H"
#include "GUI/DebugWin.H"
#include "GUI/PrefsWindow.H"
#include "Util/StringUtil.H"
#include "rutz/time.h"
#include "Util/csignals.H"

class ScorbotSimple;

const char* dataDir = "/home2/robotscenes";
std::string sceneDir = "";
int pathID = 0;
int sceneID = 0;

nub::soft_ref<OutputFrameSeries> ofs;
nub::soft_ref<ScorbotSimple> scorbot;
XWinManaged *userInteractiveWindow = NULL;
volatile bool record = false;
volatile bool keepgoing = true;
volatile bool dolivedisplay = false;

// 'volatile' because we will modify this from signal handlers
volatile int signum = 0;

Image< PixRGB<byte> > inputImage;
volatile unsigned long currentVideoFrameNumber = 0;
pthread_mutex_t imgMutex = PTHREAD_MUTEX_INITIALIZER;

rutz::time epoch = rutz::time::wall_clock_now();

const int focusval[4] = { 10, 6, 7, 8 };

// number of cameras
#define NUMCAMS 4

// ######################################################################
//! An Async Image Writer which works on a separate thread 
class WriteJob : public JobServer::Job
{
public:
  WriteJob(const Image< PixRGB<byte> > img_, const std::string fname_) :
    img(img_), fname(fname_) { }

  virtual ~WriteJob() { }

  virtual void run() { Raster::WriteRGB(img, fname); }

  virtual const char* jobType() const { return "WriteJob"; }

private:
  const Image< PixRGB<byte> > img;
  const std::string fname;
};

WorkThreadServer writer("Write Server", 5, false); // write several files at once since png compression is bottleneck

void AsyncWriteImage(const Image< PixRGB<byte> > img, size_t cameraID, unsigned long frameNumber)
{
  const std::string fname = sformat("%s/RobotScene-s%04d-p%02d/RobotScene-s%04d-p%02d-c%02zu-f%06lu.ppm", 
				    dataDir, sceneID, pathID, sceneID, pathID, cameraID, frameNumber);
  writer.enqueueJob(rutz::make_shared(new WriteJob(img, fname)));
}

// ######################################################################
// grabber devices
std::vector< nub::ref<V4L2grabber> > grabbers;

//! A grab job (we grab from all cameras in parallel)
class GrabJob : public JobServer::Job
{
public:
  GrabJob(const size_t cameraID_) :
    cameraID(cameraID_),
    frameNumber(0)
  {
    ASSERT(cameraID < grabbers.size());
  }

  virtual ~GrabJob() {}

  virtual void run() 
  {
    nub::ref<V4L2grabber> grabber = grabbers[cameraID];
    bool recording = false;

    while(keepgoing) {
      // let's grab an image:
      GenericFrame frame = grabber->readFrame();
      Image< PixRGB<byte> > img;

      if (record) {
	if (recording == false) { LINFO("Start recording camera %" ZU " at %fms...", cameraID, (rutz::time::wall_clock_now() - epoch).msec()); recording = true; }
	img = frame.asRgb();
        AsyncWriteImage(img, cameraID, frameNumber);
        ++frameNumber;
	if ((frameNumber % 10) == 0) LINFO("Camera %" ZU " got frame %" ZU " at %fms", cameraID, frameNumber, (rutz::time::wall_clock_now() - epoch).msec());
      } else {
	if (recording == true) { LINFO("Stop recording camera %" ZU " frame %" ZU " at %fms...", cameraID, frameNumber, (rutz::time::wall_clock_now() - epoch).msec()); recording = false; }
	frameNumber = 0;
      }

      // pass the current frame to the display thread
      if (cameraID == 0 && record == false) {
        if (img.initialized() == false) img = frame.asRgb();
        pthread_mutex_lock(&imgMutex);
	inputImage = img;
        ++currentVideoFrameNumber;
        pthread_mutex_unlock(&imgMutex);
      }
    }
  }

  virtual const char* jobType() const { return "GrabJob"; }

private:
  const size_t cameraID;
  unsigned long frameNumber;
};

WorkThreadServer grabberThreadServer("Grab Server", NUMCAMS, false); // number of parallel grab jobs

void setupGrabbers(ModelManager& manager)
{
  size_t cameraID = 0;
  while(true)
  {
    std::string dev = sformat("/dev/video%" ZU , grabbers.size());
    LINFO("Trying to open %s", dev.c_str());
    int fd = open(dev.c_str(), O_RDONLY);
    if (fd == -1) { LINFO("%s not found -- Skipping.", dev.c_str()); break; } else close(fd);

    // instantiate and configure a grabber:
    nub::ref<V4L2grabber> grabber(new V4L2grabber(manager, dev, dev));
    
    // do not export any command-line option to avoid clashes among grabber instances:
    grabber->forgetExports();

    // let's set everything by hand here to ensure consistency:
    grabber->setModelParamVal("FrameGrabberDevice", dev);

    grabber->setModelParamVal("FrameGrabberNbuf", 2);
    grabber->setModelParamVal("FrameGrabberStreaming", true); 
    grabber->setModelParamVal("FrameGrabberByteSwap", false);
    grabber->setModelParamVal("FrameGrabberDims", Dims(1280,720));
    grabber->setModelParamVal("FrameGrabberMode", VIDFMT_YUYV);
    grabber->setModelParamVal("FrameGrabberChannel", 0);

    // to make sure we will force the hardware to set its values, first set some trash values, then set the real values:
    // turn all auto controls to on:
    grabber->setModelParamVal("FrameGrabberWhiteBalTempAuto", true);
    grabber->setModelParamVal("FrameGrabberPowerLineFreq", 2);
    grabber->setModelParamVal("FrameGrabberBacklightComp", 1);
    grabber->setModelParamVal("FrameGrabberFocusAuto", true);
 
    // now turn all auto controls to off:
    grabber->setModelParamVal("FrameGrabberWhiteBalTempAuto", false);
    grabber->setModelParamVal("FrameGrabberPowerLineFreq", 0);
    grabber->setModelParamVal("FrameGrabberBacklightComp", 0);
    grabber->setModelParamVal("FrameGrabberExposureAuto", 3); // that's still auto
    grabber->setModelParamVal("FrameGrabberFocusAuto", false);
    grabber->setModelParamVal("FrameGrabberExposureAuto", 1); // now we are full manual for exposure
 
    // set all manual settings 1-off from what we want:
    grabber->setModelParamVal("FrameGrabberBrightness", 134);
    grabber->setModelParamVal("FrameGrabberContrast", 6);
    grabber->setModelParamVal("FrameGrabberSaturation", 84);
    grabber->setModelParamVal("FrameGrabberWhiteBalTemp", 3101);
    grabber->setModelParamVal("FrameGrabberExposureAbs", 39);
    grabber->setModelParamVal("FrameGrabberSharpness", 26);
    grabber->setModelParamVal("FrameGrabberFocus", focusval[cameraID] + 1);
    grabber->setModelParamVal("FrameGrabberZoom", 1);
 
    // and now the real values:
    grabber->setModelParamVal("FrameGrabberPowerLineFreq", 0); // should be set already but just in case...
    grabber->setModelParamVal("FrameGrabberBacklightComp", 0);
    grabber->setModelParamVal("FrameGrabberExposureAuto", 1);
    grabber->setModelParamVal("FrameGrabberWhiteBalTempAuto", false);
 
    grabber->setModelParamVal("FrameGrabberBrightness", 133);
    grabber->setModelParamVal("FrameGrabberContrast", 5);
    grabber->setModelParamVal("FrameGrabberSaturation", 83);
    grabber->setModelParamVal("FrameGrabberWhiteBalTemp", 3100);
    grabber->setModelParamVal("FrameGrabberSharpness", 25);
    grabber->setModelParamVal("FrameGrabberExposureAbs", 156);
    grabber->setModelParamVal("FrameGrabberFocusAuto", false);
    grabber->setModelParamVal("FrameGrabberFocus", focusval[cameraID]);
    grabber->setModelParamVal("FrameGrabberZoom", 0);
    
    // keep track of it:
    manager.addSubComponent(grabber);
    grabbers.push_back(grabber);
    LINFO("Added V4L2grabber for %s", dev.c_str());

    ++cameraID;
  }
}

// ######################################################################
struct SceneSetup
{
  struct Object
  {
    std::string trayFileName;
    std::string trayName;
    int trayColumn;
    int trayRow;
    float trayX;
    float trayY;
    std::vector< Point2D<int> > outline;
  };

  std::string setupPath;
  int setupNum;
  int backgroundIdx;
  std::string backgroundFileName;
  std::vector<int> pathIndex;
  
  std::vector<SceneSetup::Object> objects;
};

// ######################################################################
// Parse the scene setup file found in the given path
SceneSetup loadSceneSetup(const int id)
{
  SceneSetup setup;
  setup.setupPath = sceneDir;

  std::string setupFileName = setup.setupPath + "/" + sformat("%04d.txt", id);
  std::ifstream setupFile(setupFileName.c_str());

  if (!setupFile.is_open()) LFATAL("Could not open setup file: %s", setupFileName.c_str());

  std::string line;

  try
  {
    //SetupNumber
    getline(setupFile, line);
    setup.setupNum = boost::lexical_cast<int>(line.substr(line.find("=")+1));

    //BackgroundIndex
    getline(setupFile, line);
    setup.backgroundIdx = boost::lexical_cast<int>(line.substr(line.find("=")+1));

    //BackgroundFileName
    getline(setupFile, line);
    setup.backgroundFileName = line.substr(line.find("=")+1);
    
    //PathIndex
    getline(setupFile, line);
    std::vector<std::string> tok;
    split(line.substr(line.find("=")+1), ",", std::back_inserter(tok));
    for (size_t ii = 0; ii < tok.size(); ++ii)
      setup.pathIndex.push_back(boost::lexical_cast<int>(tok[ii]));

    //NumberOfObjects
    getline(setupFile, line);
    int numObjects = boost::lexical_cast<int>(line.substr(line.find("=")+1));

    for(int i=0; i<numObjects; ++i)
    {
      SceneSetup::Object obj;

      //TrayFileName
      getline(setupFile, line);
      obj.trayFileName = line.substr(line.find("=")+1);

      //TrayName
      size_t s = obj.trayFileName.find_last_of('/');
      size_t e = obj.trayFileName.find_last_of('.');
      obj.trayName = obj.trayFileName.substr(s+1, e-s-1);

      //TrayColumn
      getline(setupFile, line);
      obj.trayColumn = boost::lexical_cast<int>(line.substr(line.find("=")+1));

      //TrayRow
      getline(setupFile, line);
      obj.trayRow = boost::lexical_cast<int>(line.substr(line.find("=")+1));

      //XOnTray
      getline(setupFile, line);
      obj.trayX = boost::lexical_cast<float>(line.substr(line.find("=")+1));

      //YOnTray
      getline(setupFile, line);
      obj.trayY = boost::lexical_cast<float>(line.substr(line.find("=")+1));

      setup.objects.push_back(obj);
    }
  }
  catch(boost::bad_lexical_cast& e)
  {
    LFATAL("Error Parsing Setup File (%s) [%s] -- Offending Line: %s", setupFileName.c_str(), e.what(), line.c_str());
  }

  return setup;
}

// ######################################################################
// Display the image and prompts to the user
void *displayThreadMethod(void*)
{
  // this is disabled for now...

  /*
  unsigned long frameNumber = 0;
  while(true)
  {
    Image< PixRGB<byte> > img;
    pthread_mutex_lock(&imgMutex);
    if (frameNumber != currentVideoFrameNumber) {
      img = inputImage;
      frameNumber = currentVideoFrameNumber;
    }
    pthread_mutex_unlock(&imgMutex);

    if (dolivedisplay && img.initialized()) {
      //      img = rescale(img, img.getDims() / 2);
      ofs->writeRGB(img, "Live Video from Main Camera");
      ofs->updateNext();
    }

    // go easy on the CPU:
    usleep(30000);
  }
  */

  return NULL;
}

// ######################################################################
class ScorbotSimple: public ModelComponent
{
  public:
    ScorbotSimple(OptionManager& mgr, 
        const std::string& descrName = "",
        const std::string& tagName = "") :
      ModelComponent(mgr, descrName, tagName),
      itsSerial(new Serial(mgr)) 
    {
      addSubComponent(itsSerial);
      itsSerial->configure("/dev/ttyUSB0", 115200, "8N1", false, false, 10000);
    }

    bool getBusyState()
    {
      char cmd = 'B';
      itsSerial->write(&cmd, 1);
      usleep(5000);

      char retVal = '?';
      itsSerial->read(&retVal, 1);
      //if (retVal == '?') LERROR("Serial Comminucation failed");
      return (retVal == '1');
    }

    bool getHomeState()
    {
      char cmd = 'H';
      itsSerial->write(&cmd, 1);
      usleep(5000);

      char retVal = '?';
      itsSerial->read(&retVal, 1);
      //if (retVal == '?') LERROR("Serial Comminucation failed");
      return (retVal == '1');
    }
    
    void emptyBuffer()
    {
       while(true)
       {
           usleep(10000);
           char retVal = '?';
           itsSerial->read(&retVal, 1);
           if(retVal == '?') break;
       }
    }
  
    void setPathNum(int pathnum)
    {
      //LINFO("Executing Path %i", pathnum);
      char cmd[2] = { 'P', pathnum };
      itsSerial->write(&cmd, 2);
    }

  private:
    nub::ref<Serial> itsSerial;
};

// ######################################################################
// Run the robot arm along the given path, blocking until it is finished
void executePath(int pathnum)
{
  LINFO("Waiting for Robot HOME ...");
  while(!scorbot->getHomeState())
  {
     if(signum != 0) { record = false; return; }
     usleep(10000);
  }
  LINFO("Going To Path %d", pathnum);
  scorbot->setPathNum(pathnum++);
  LINFO("Waiting for Robot BUSY ...");
  while(!scorbot->getBusyState())
  {
     if(signum != 0) { record = false; return; }
     usleep(10000);
  }
  epoch = rutz::time::wall_clock_now();
  usleep(1000000); // trash old frame in camera buffer
  LINFO("Camera recording on.");
  record = true;
  LINFO("Waiting for Robot ~BUSY ...");
  while(scorbot->getBusyState())
  {
     if(signum != 0) { record = false; return; }
     usleep(10000);
  }
  record = false;
  LINFO("Camera recording off.");

  scorbot->emptyBuffer();
}

// ######################################################################
// Prompt the user to place an object from a tray onto the table, and 
// then present them with an interface to outline that object
std::vector< Point2D<int> > promptPlaceObjectOnScene(SceneSetup const & setup, int objIdx)
{
  LINFO("Place object %d onto table. Follow instructions in User Interactive window...", objIdx);

  const SceneSetup::Object &obj = setup.objects[objIdx];

  Image< PixRGB<byte> > trayImg = Raster::ReadRGB(setup.setupPath + "/" + obj.trayFileName);
  trayImg = rescale(trayImg, trayImg.getDims()/2);

  Point2D<int> objPos(obj.trayX*trayImg.getWidth(), obj.trayY*trayImg.getHeight());

  drawCircle(trayImg, objPos, trayImg.getWidth()/12, PixRGB<byte>(255, 0, 0), 2);
  drawCross(trayImg, objPos, PixRGB<byte>(255, 0, 0), trayImg.getWidth()/8, 2);

  std::ostringstream ss;
  ss << "Place Tray " << obj.trayName << 
    " (row " << obj.trayRow << 
    ", col " << obj.trayColumn << 
    ") on table and ENTER";
  writeText(trayImg, Point2D<int>(0, 0), ss.str().c_str());

  userInteractiveWindow->drawImage(trayImg, 0, 0, true);
  while(userInteractiveWindow->getLastKeyPress() != 36) usleep(100000);
  
  pthread_mutex_lock(&imgMutex);
  Image<PixRGB<byte> > cameraImg = inputImage;
  pthread_mutex_unlock(&imgMutex);
  userInteractiveWindow->drawImage(cameraImg, 0, 0, true);
  std::vector< Point2D<int> > poly;
  std::string msg = "Outline new object using 4 points. ESCAPE to undo point. SPACE to refresh image. ENTER when finished.";

  // eat all previous mouse clicks and key presses, just in case:
  while (userInteractiveWindow->getLastMouseClick() != Point2D<int>(-1, -1)) { }
  while (userInteractiveWindow->getLastKeyPress() != -1) { }

  bool finished = false;
  while(!finished) {
    Point2D<int> mouseClick = userInteractiveWindow->getLastMouseClick();
    if(mouseClick != Point2D<int>(-1, -1) && poly.size() < 4)
      poly.push_back(mouseClick);

    int lastKeyPress = userInteractiveWindow->getLastKeyPress();
    switch(lastKeyPress) {
    case -1: // No Key
      break;
    case 9:  // ESCAPE
      if(poly.size()) poly.erase(poly.end()-1);
      break;
    case 36: // ENTER
      if(poly.size() == 4) finished = true;
      break;
    case 65: // SPACE
      pthread_mutex_lock(&imgMutex);
      cameraImg = inputImage;
      pthread_mutex_unlock(&imgMutex);
      userInteractiveWindow->drawImage(cameraImg, 0, 0, true);
      break;
    default:
      LINFO("Key Pressed: %d", lastKeyPress);
      break;
    }

    Image< PixRGB<byte> > dispImage = cameraImg;
    for(size_t i=0; i<poly.size(); ++i) drawCircle(dispImage, poly[i], 5, PixRGB<byte>(255, 0, 0), 3);
    drawOutlinedPolygon(dispImage, poly, PixRGB<byte>(0, 0, 255), Point2D<int>(0,0),0,1,0,0,3);
    writeText(dispImage, Point2D<int>(0,0), msg.c_str());
    userInteractiveWindow->drawImage(dispImage, 0, 0, true);

    usleep(100000);
  }

  LINFO("Done placing object %d onto table.", objIdx);

  return poly;
}

// ######################################################################
// Prompt the user to return the object back to the tray place an object
void promptReturnObjectToTray(SceneSetup const & setup, int objIdx)
{
  LINFO("Placing back object %d into its tray. Follow instructions in User Interactive window...", objIdx);
  const SceneSetup::Object &obj = setup.objects[objIdx];

  Image< PixRGB<byte> > trayImg = Raster::ReadRGB(setup.setupPath + "/" + obj.trayFileName);
  trayImg = rescale(trayImg, trayImg.getDims()/2);

  Point2D<int> objPos(obj.trayX*trayImg.getWidth(), obj.trayY*trayImg.getHeight());

  drawCircle(trayImg, objPos, trayImg.getWidth()/12, PixRGB<byte>(255, 0, 0), 2);
  drawCross(trayImg, objPos, PixRGB<byte>(255, 0, 0), trayImg.getWidth()/8, 2);

  std::ostringstream ss;
  ss << "Place back Obj(" <<
    "row " << obj.trayRow << 
    ", col " << obj.trayColumn << 
    ") into tray " << obj.trayName << 
    " and ENTER";
  writeText(trayImg, Point2D<int>(0, 0), ss.str().c_str());
  
  userInteractiveWindow->drawImage(trayImg, 0, 0, true);

  // eat all previous mouse clicks and key presses, just in case:
  while (userInteractiveWindow->getLastMouseClick() != Point2D<int>(-1, -1)) { }
  while (userInteractiveWindow->getLastKeyPress() != -1) { }

  // wait for ENTER:
  while (userInteractiveWindow->getLastKeyPress() != 36) usleep(100000);

  LINFO("Done placing back object %d into its tray.", objIdx);
}

// ######################################################################
//! Get/confirm an int value:
void getInt(const char *msg, int& val)
{
  printf("%s [%d]: ", msg, val); fflush(stdout);
  char input[1000]; while(gets(input) == NULL) LERROR("Invalid input, try again");

  if (strlen(input) > 0) val = atoi(input); // updated the value
};

// ######################################################################
int submain(const int argc, const char** argv)
{
  // catch signals and redirect them for a clean exit (in particular, this gives us a chance to do useful things like
  // flush and close output files that would otherwise be left in a bogus state, like mpeg output files):
  catchsignals(&signum);

  LINFO("#############################################STARTING##############################################");

  ModelManager mgr("App Scorbot MultiGrab");

  scorbot.reset(new ScorbotSimple(mgr));
  mgr.addSubComponent(scorbot);

  setupGrabbers(mgr);

  ofs.reset(new OutputFrameSeries(mgr));
  mgr.addSubComponent(ofs);

  if(mgr.parseCommandLine(argc, argv, "FilePrefix SceneID", 2, 2) == false) return -1;
  mgr.start();

  if (grabbers.size() < NUMCAMS) LFATAL("Only found %" ZU " cameras instead of %d. Reboot your machine and try again.", grabbers.size(), NUMCAMS);

  // get our grabbers to start grabbing:  
  for (size_t cameraID = 0; cameraID < grabbers.size(); ++cameraID)
    grabberThreadServer.enqueueJob(rutz::make_shared(new GrabJob(cameraID)));

  sceneDir = mgr.getExtraArg(0);
  sceneID = boost::lexical_cast<int>(mgr.getExtraArg(1));

  pthread_t displayThread;
  pthread_create(&displayThread, NULL, &displayThreadMethod, NULL);

  // Create the interactive window
  userInteractiveWindow = new XWinManaged(Dims(640,480), -1, -1, "User Interactive");
  userInteractiveWindow->setVisible(false);
  userInteractiveWindow->setPosition(0, 0);

  // Main loop:
  int runnumber = 0;
  while(true) {
    if(signum != 0) break;

    // home the robot once in a while:
    if ((runnumber % 5) == 0) {
      int gogo = 0; getInt("Perform robot homing sequence and press ENTER", gogo);
    }

    // select the scene:
    getInt("Enter scene ID (-1 to exit):", sceneID);

    if (sceneID == -1) break; // abort on scene -1

    // STEP 1. Load the scene file
    SceneSetup setup = loadSceneSetup(sceneID);

    // STEP 2. Show the interactive window:
    userInteractiveWindow->setVisible(true);

    // STEP 3. Display background image and ask the user to place it on the scene
    Image< PixRGB<byte> > backgroundImage = Raster::ReadRGB(setup.setupPath + "/" + setup.backgroundFileName);
    backgroundImage = rescale(backgroundImage, Dims(640, 480));
    writeText(backgroundImage, Point2D<int>(0, 0), "Please place this background on the scene and press ENTER.");
    userInteractiveWindow->drawImage(backgroundImage, 0, 0, true);
    LINFO("Place background map on scene and add houses, trees, and other background objects. See User Interactive window for instructions...");
    // eat all previous mouse clicks and key presses, just in case:
    while (userInteractiveWindow->getLastMouseClick() != Point2D<int>(-1, -1)) { }
    while (userInteractiveWindow->getLastKeyPress() != -1) { }
    // wait for ENTER:
    while(userInteractiveWindow->getLastKeyPress() != 36) usleep(100000);
    LINFO("Background map done. Make sure you have built a nice scene.");

    // STEP 4. Display each object and ask user to put on the scene and specify its bounding box
    for (size_t i = 0; i < setup.objects.size(); ++i)
      setup.objects[i].outline = promptPlaceObjectOnScene(setup, i);

    // STEP 5. Hide the interactive window
    userInteractiveWindow->setVisible(false);

    // STEP 6. Write out outlines to a file
    {
      std::string objectFileName = sformat("%s/RobotScene-s%04d-polygons.txt", dataDir, sceneID);
      std::ofstream objectFile(objectFileName.c_str());

      LINFO("Saving the object bounding boxes into: %s", objectFileName.c_str());
 
      for(size_t i=0; i<setup.objects.size(); ++i)
	{
	  for(size_t j=0; j<setup.objects[i].outline.size(); ++j)
	    {
	      Point2D<int> &pnt = setup.objects[i].outline[j];
	      if(j != 0) objectFile << ',';
	      objectFile << pnt.i << ',' << pnt.j;
	    }
	  objectFile << std::endl;
	}
    }

    // STEP 7. Execute the path and record the videos
    for (pathID = 0; pathID < int(setup.pathIndex.size()); ++pathID) {
      if(signum != 0) break;
      // create a directory for this scene / light:
      const std::string dir = sformat("%s/RobotScene-s%04d-p%02d", dataDir, sceneID, pathID);
      const std::string cmd = sformat("/bin/mkdir -p %s", dir.c_str());
      if (system(cmd.c_str()) == -1) PLFATAL("Could not create directory %s", dir.c_str());

      int gogo = pathID; getInt("Set light and press ENTER to start video recording", gogo);

      // make sure we don't have too many pending disk writes:
      while(writer.size() > 1000) {
	LINFO("Waiting for image writer thread, queue size = %" ZU "...", writer.size());
	usleep(1000000);
      }

      LINFO("Running Scene %04d Path %02d ...", sceneID, setup.pathIndex[pathID]);

      executePath(setup.pathIndex[pathID]);
    }
    if(signum != 0) break;

    // STEP 8. Instruct users to place the objects back into the bins
    userInteractiveWindow->setVisible(true);
    userInteractiveWindow->setPosition(0, 0);
    for(size_t i=0; i<setup.objects.size(); ++i) promptReturnObjectToTray(setup, i);
    userInteractiveWindow->setVisible(false);

    // STEP 9. Ready for next scene
    ++sceneID;
    ++runnumber;
  }

  // stop grabbing:
  keepgoing = false;

  // wait for all pics to be written (note: this just waits until the queue of pending jobs is empty, the writer's
  // destructor will wait until all jobs are complete):
  while(writer.size()) {
    LINFO("Waiting for image writer thread, queue size = %" ZU "...", writer.size());
    usleep(500000);
  }
  writer.flushQueue(250000, true);

  // make sure all is done
  LINFO("Cleaning up... Stand by...");
  usleep(2000000);

  // stop all our ModelComponents
  mgr.stop();

  LINFO("Finished.");

  return 0;
}

// ######################################################################
int main(int argc, const char** argv)
{
  int ret = -1;

  try { ret = submain(argc, argv); }
  catch(...) { REPORT_CURRENT_EXCEPTION; }

  return ret;
}
