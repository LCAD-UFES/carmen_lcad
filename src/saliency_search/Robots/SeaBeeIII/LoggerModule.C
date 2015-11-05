#include "Robots/SeaBeeIII/LoggerModule.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#ifndef LOGGERMODULE_C
#define LOGGERMODULE_C

const ModelOptionCateg MOC_SeaBeeIIILogger = {
    MOC_SORTPRI_3, "SeaBeeIII Logger Related Options" };

const ModelOptionDef OPT_LogPath =
{ MODOPT_ARG(string), "LogPath", &MOC_SeaBeeIIILogger, OPTEXP_CORE,
  "Path where the logger should write fwd/dwn retina images and sensor readings",
   "log-path", '\0', "<string>", "/home/uscr/log/test" };

// ######################################################################
LoggerModule::LoggerModule(int id, OptionManager& mgr,
                const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsLogPath(&OPT_LogPath, this, 0),
  itsFwdFrameCount(0),
  itsLastFwdFrameCount(0),
  itsDwnFrameCount(0),
  itsLastDwnFrameCount(0),
  itsImgWriter(),
  itsLogOpen(false)
{
}
// ######################################################################
LoggerModule::~LoggerModule()
{
  itsLogFile.close();
}

void LoggerModule::registerTopics()
{
  registerSubscription("BeeStemMessageTopic");
  registerSubscription("RetinaMessageTopic");
}

void LoggerModule::evolve()
{
  itsDataMutex.lock();

  if(!itsLogOpen)
    {
      itsLogOpen = true;

      char* lstr = new char[100];
      sprintf(lstr,"%s/log.txt",itsLogPath.getVal().c_str());
      itsLogFile.open(lstr, std::ios::out);
    }

  char* str = new char[100];
  sprintf(str,"%s/fwd_img/FwdImg_%06d.pnm",itsLogPath.getVal().c_str(),itsFwdFrameCount);

  if(itsLastFwdFrameCount != itsFwdFrameCount)
    {
      itsLastFwdFrameCount = itsFwdFrameCount;
      itsImgWriter.writeRGB(itsCurrentFwdImg,str);
    }

  str = new char[100];
  sprintf(str,"%s/dwn_img/DwnImg_%06d.pnm",itsLogPath.getVal().c_str(),itsDwnFrameCount);

  if(itsLastDwnFrameCount != itsDwnFrameCount)
    {
      itsLastDwnFrameCount = itsDwnFrameCount;
      itsImgWriter.writeRGB(itsCurrentDwnImg,str);
    }

  delete[] str;

  itsLogFile<<itsFwdFrameCount<<" "<<itsCurrentHeading<<" "
            <<itsCurrentIntPressure<<" "<<itsCurrentExtPressure<<std::endl;
  itsDataMutex.unlock();

  usleep(40);
}

// ######################################################################
void LoggerModule::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                const Ice::Current&)
{
  if(eMsg->ice_isA("::RobotSimEvents::BeeStemMessage"))
    {

      RobotSimEvents::BeeStemMessagePtr msg = RobotSimEvents::BeeStemMessagePtr::dynamicCast(eMsg);
      itsDataMutex.lock();

      itsCurrentHeading = msg->compassHeading;
      itsCurrentExtPressure= msg->externalPressure;
      itsCurrentIntPressure = msg->internalPressure;

      itsDataMutex.unlock();
    }
  else if(eMsg->ice_isA("::RobotSimEvents::RetinaMessage"))
    {
      RobotSimEvents::RetinaMessagePtr msg = RobotSimEvents::RetinaMessagePtr::dynamicCast(eMsg);
      itsDataMutex.lock();

      if(msg->cameraID == "FwdCamera")
        {
          itsCurrentFwdImg = Ice2Image<PixRGB<byte> >(msg->img);
          itsFwdFrameCount++;
        }
      else
        {
          itsCurrentDwnImg = Ice2Image<PixRGB<byte> >(msg->img);
          itsDwnFrameCount++;
        }

      itsDataMutex.unlock();

    }
}

#endif
